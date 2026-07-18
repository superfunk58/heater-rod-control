#include "net_manager.h"
#include <WiFi.h>
#include <ETH.h>
#include <SPI.h>
#include <ESPmDNS.h>
#include "secrets.h"     // WLAN_SSID / WLAN_PASS
#include "webserver.h"   // webLog()

// Network configuration globals (owned by main.cpp, persisted by ConfigStore).
extern String NET_MODE;   // "wifi" | "lan"
extern bool   LAN_DHCP;
extern String LAN_IP;
extern String LAN_GW;
extern String LAN_MASK;
extern String LAN_DNS;

namespace NetManager {

static String        s_hostname     = "Heizstabsteuerung";
static bool          s_lanMode      = false;   // NET_MODE == "lan"

// Explicit network state machine. Only one interface is ever intended to be
// active at a time, giving a clean WiFi/LAN separation.
enum class NetState : uint8_t {
  OFFLINE = 0,          // no interface up
  WIFI,                 // WiFi mode, WiFi active
  LAN,                  // LAN mode, LAN active, WiFi fully off
  LAN_WIFI_FALLBACK     // LAN mode, LAN down, WiFi fallback active
};
static NetState s_state = NetState::OFFLINE;

// W5500 state (volatile because set from network event handler)
static volatile bool s_ethHasIP     = false;   // W5500 obtained an IP
static volatile bool s_ethLinkUp    = false;   // W5500 PHY link state

// LAN link-loss debounce: after the LAN has been up, a brief link flap should
// NOT immediately switch to WiFi. We only fall back to WiFi if the LAN stays
// down continuously for LAN_DOWN_GRACE_MS.
static unsigned long s_lanDownSince = 0;       // millis() when LAN link dropped (0 = up)
static const unsigned long LAN_DOWN_GRACE_MS = 8000;

// LAN-first boot timeout
static unsigned long s_lanDeadline = 0;        // millis() when LAN first-attempt times out

static unsigned long s_wifiOutageStart   = 0;    // start of current WiFi outage
static unsigned long s_lastWifiReconnect = 0;   // rate-limit reconnect attempts

// ---- mDNS (re)start ------------------------------------------------------
static void restartMDNS() {
  MDNS.end();
  if (MDNS.begin(s_hostname.c_str())) {
    MDNS.setInstanceName(s_hostname.c_str());
    MDNS.addService("http", "tcp", 80);
  }
}

// ---- WiFi bring-up -------------------------------------------------------
static void startWifi() {
  if (WiFi.getMode() != WIFI_OFF) return; // already active
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(s_hostname.c_str());
  WiFi.setSleep(false);             // disable power saving for OTA stability
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  Serial.printf("[Net] WiFi STA started (SSID=%s)\n", WLAN_SSID);
  webLog("[Net] WiFi STA started (SSID=%s)", WLAN_SSID);
}

static void stopWifi() {
  if (WiFi.getMode() == WIFI_OFF) return; // already off
  WiFi.disconnect(true, false);  // disconnect + turn off radio, keep AP config
  delay(50);
  WiFi.mode(WIFI_OFF);
  Serial.println("[Net] WiFi switched OFF");
  webLog("[Net] WiFi switched OFF");
}

// ---- W5500 Ethernet bring-up ---------------------------------------------
static void startEthernet() {
  Serial.printf("[Net] W5500 init: SCLK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d\n",
         W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS, W5500_INT, W5500_RST);
  webLog("[Net] W5500 init: SCLK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d",
         W5500_SCLK, W5500_MISO, W5500_MOSI, W5500_CS, W5500_INT, W5500_RST);
  // Core 3.x SPI-Ethernet begin (creates the esp_netif interface).
  if (!ETH.begin(ETH_PHY_W5500, 1, W5500_CS, W5500_INT, W5500_RST,
                 SPI2_HOST, W5500_SCLK, W5500_MISO, W5500_MOSI)) {
    Serial.println("[Net] W5500 ETH.begin() FAILED (no chip / wiring?)");
    webLog("[Net] W5500 ETH.begin() FAILED (no chip / wiring?)");
    return;
  }
  ETH.setHostname(s_hostname.c_str());
  if (!LAN_DHCP) {
    IPAddress ip, gw, mask, dns;
    bool ok = ip.fromString(LAN_IP) && gw.fromString(LAN_GW) &&
              mask.fromString(LAN_MASK) && dns.fromString(LAN_DNS);
    if (ok && ETH.config(ip, gw, mask, dns)) {
      Serial.printf("[Net] W5500 static IP %s gw %s\n", LAN_IP.c_str(), LAN_GW.c_str());
      webLog("[Net] W5500 static IP %s gw %s", LAN_IP.c_str(), LAN_GW.c_str());
    } else {
      Serial.println("[Net] W5500 static IP invalid -> using DHCP");
      webLog("[Net] W5500 static IP invalid -> using DHCP");
    }
  } else {
    Serial.println("[Net] W5500 using DHCP");
    webLog("[Net] W5500 using DHCP");
  }
}

// ---- Unified network event handler ---------------------------------------
static void onNetEvent(arduino_event_id_t event, arduino_event_info_t info) {
  (void)info;
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("[Net] event: ETH_START");
      ETH.setHostname(s_hostname.c_str());
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      s_ethLinkUp = true;
      Serial.println("[Net] W5500 link UP");
      webLog("[Net] W5500 link UP");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
    case ARDUINO_EVENT_ETH_GOT_IP6:
      s_ethHasIP = true;
      s_ethLinkUp = true;
      Serial.printf("[Net] W5500 GOT IP %s\n", ETH.localIP().toString().c_str());
      webLog("[Net] W5500 GOT IP %s", ETH.localIP().toString().c_str());
      restartMDNS();
      s_lanDownSince = 0;   // LAN is back up -> cancel any pending fallback timer
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      s_ethLinkUp = false;
      s_ethHasIP = false;
      Serial.println("[Net] W5500 link DOWN");
      webLog("[Net] W5500 link DOWN");
      // Do NOT switch to WiFi immediately. Start the debounce timer; loop()
      // falls back to WiFi only if the LAN stays down past LAN_DOWN_GRACE_MS.
      if (s_lanMode && s_lanDownSince == 0) {
        s_lanDownSince = millis();
        Serial.println("[Net] LAN link down -> grace period before WiFi fallback");
      }
      break;
    case ARDUINO_EVENT_ETH_STOP:
      s_ethLinkUp = false;
      s_ethHasIP = false;
      Serial.println("[Net] event: ETH_STOP");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[Net] event: WIFI_STA_START");
      WiFi.setHostname(s_hostname.c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[Net] WiFi GOT IP %s\n", WiFi.localIP().toString().c_str());
      webLog("[Net] WiFi GOT IP %s", WiFi.localIP().toString().c_str());
      restartMDNS();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[Net] WiFi disconnected (reason=%d)\n", info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("[Net] WiFi lost IP");
      break;
    default:
      break;
  }
}

// ---- Public API ----------------------------------------------------------
void begin(const char *hostname) {
  if (hostname && *hostname) s_hostname = hostname;
  s_lanMode = (NET_MODE == "lan");

  Network.onEvent(onNetEvent);

  Serial.printf("[Net] begin() NET_MODE=%s LAN_DHCP=%s\n", NET_MODE.c_str(), LAN_DHCP ? "true" : "false");

  if (s_lanMode) {
    // LAN-first boot: start Ethernet, wait for link/IP, then start WiFi fallback if needed.
    // In LAN mode WiFi must be fully off at boot so it does not scan/connect in the background.
    Serial.println("[Net] LAN mode: starting W5500 first, WiFi fallback starts after timeout if LAN fails");
    WiFi.mode(WIFI_OFF);
    startEthernet();
    s_state = NetState::LAN;
    s_lanDeadline = millis() + 15000;   // 15 s for LAN DHCP/static to come up
  } else {
    Serial.println("[Net] WiFi mode: starting WiFi immediately");
    s_state = NetState::WIFI;
    startWifi();
  }
}

bool loop() {
  const unsigned long nowMs = millis();

  // LAN-first boot phase: wait for link/IP, start WiFi fallback if deadline passes.
  if (s_lanMode && s_state == NetState::LAN && s_lanDeadline != 0) {
    if (s_ethHasIP) {
      Serial.println("[Net] LAN first-attempt SUCCESS");
      s_lanDeadline = 0;
      return true;
    }
    if ((long)(nowMs - s_lanDeadline) >= 0) {
      // LAN did not come up in time -> start WiFi fallback.
      Serial.println("[Net] LAN first-attempt TIMEOUT -> switching to WiFi fallback");
      s_state = NetState::LAN_WIFI_FALLBACK;
      s_lanDeadline = 0;
      startWifi();
    }
    return false;   // still waiting for LAN during first attempt
  }

  // In LAN state: make sure WiFi is fully off (RF silence, clean separation).
  if (s_state == NetState::LAN && WiFi.getMode() != WIFI_OFF) {
    stopWifi();
  }

  // Debounced LAN-loss fallback: the LAN was up but the link dropped.
  // Only switch to WiFi once it has stayed down past the grace period.
  if (s_lanMode && s_state == NetState::LAN && !s_ethHasIP && s_lanDownSince != 0 &&
      (nowMs - s_lanDownSince >= LAN_DOWN_GRACE_MS)) {
    Serial.println("[Net] LAN down past grace -> starting WiFi fallback");
    webLog("[Net] LAN down -> WiFi fallback");
    s_state = NetState::LAN_WIFI_FALLBACK;
    s_lanDownSince = 0;
    startWifi();
  }

  // LAN recovered while on WiFi fallback: switch back to LAN and turn WiFi off.
  if (s_lanMode && s_state == NetState::LAN_WIFI_FALLBACK && s_ethHasIP) {
    Serial.println("[Net] LAN available while on WiFi -> switching back to LAN");
    s_state = NetState::LAN;
    s_lanDownSince = 0;
    stopWifi();
  }

  // WiFi active states
  if (s_state == NetState::WIFI || s_state == NetState::LAN_WIFI_FALLBACK) {
    if (WiFi.status() == WL_CONNECTED) {
      s_wifiOutageStart = 0;
      return true;
    }

    if (s_wifiOutageStart == 0) s_wifiOutageStart = nowMs;

    // Full re-begin after a prolonged outage; otherwise rate-limited reconnect.
    if (nowMs - s_wifiOutageStart > 30000) {
      Serial.println("[Net] WiFi outage > 30s, restarting WiFi");
      WiFi.disconnect();
      delay(50);
      WiFi.begin(WLAN_SSID, WLAN_PASS);
      s_wifiOutageStart = nowMs;
      s_lastWifiReconnect = nowMs;
    } else if (nowMs - s_lastWifiReconnect > 2000) {
      WiFi.reconnect();
      s_lastWifiReconnect = nowMs;
    }
    return false;
  }

  // LAN active state
  if (s_state == NetState::LAN && s_ethHasIP) return true;

  return false;
}

bool isOnline()       {
  if (s_state == NetState::LAN) return s_ethHasIP;
  if (s_state == NetState::WIFI || s_state == NetState::LAN_WIFI_FALLBACK)
    return WiFi.status() == WL_CONNECTED;
  return false;
}
bool usingEthernet()  { return s_state == NetState::LAN && s_ethHasIP; }
bool ethLinkUp()      { return s_ethLinkUp; }

const char *activeIface() {
  switch (s_state) {
    case NetState::LAN:  return "lan";
    case NetState::WIFI:
    case NetState::LAN_WIFI_FALLBACK: return "wifi";
    default: return "none";
  }
}

String activeIP() {
  if (s_state == NetState::LAN && s_ethHasIP) return ETH.localIP().toString();
  if ((s_state == NetState::WIFI || s_state == NetState::LAN_WIFI_FALLBACK) &&
      WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
  return "0.0.0.0";
}

void fillStatus(JsonVariant v) {
  v["netMode"]    = NET_MODE;
  v["netState"]   = (int)s_state;   // 0=offline,1=wifi,2=lan,3=lan_wifi_fallback
  v["netIface"]   = activeIface();
  v["netIP"]      = activeIP();
  v["ethLinkUp"]  = s_ethLinkUp;
  v["ethHasIP"]   = (bool)s_ethHasIP;
  v["lanDhcp"]    = LAN_DHCP;
  v["lanIP"]      = LAN_IP;
  v["lanGw"]      = LAN_GW;
  v["lanMask"]    = LAN_MASK;
  v["lanDns"]     = LAN_DNS;
}

}  // namespace NetManager
