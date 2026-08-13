#include "net_manager.h"
#include <WiFi.h>
#include <ETH.h>
#include <SPI.h>
#include <ESPmDNS.h>
#include <lwip/sockets.h>
#include <errno.h>
#include "secrets.h"     // WLAN_SSID / WLAN_PASS / AIO_SERVER / AIO_SERVERPORT
#include "webserver.h"   // webLog(), webserver_closeAllSseClients()

// Network configuration globals (owned by main.cpp, persisted by ConfigStore).
// Fixed-size char arrays: no heap, no fragmentation.
extern char NET_MODE[8];   // "wifi" | "lan"
extern bool LAN_DHCP;
extern char LAN_IP[16];
extern char LAN_GW[16];
extern char LAN_MASK[16];
extern char LAN_DNS[16];

namespace NetManager {

static char          s_hostname[33] = "Heizstabsteuerung";
static char          s_ssid[33]     = "";   // cached WiFi SSID (set on GOT_IP)
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

// ---- LAN verification via MQTT broker probe --------------------------------
// WiFi is only switched off once the LAN has proven it can reach the MQTT
// broker END-TO-END through the W5500: a TCP socket is bound to the ETH IP
// (so lwIP routes it out of the Ethernet netif even while the WiFi fallback
// is the default route) and a non-blocking connect() to the broker must
// succeed. A bare DHCP lease is not enough (wrong VLAN, missing gateway, ...).
// The broker is the probe target (not NTP) because it is the service this
// device actually needs — and it stays reachable when the internet is down.
static int           s_probeSock      = -1;   // lwIP socket of the current attempt
static IPAddress     s_probeServer;           // resolved MQTT broker IP
static bool          s_probeActive    = false;
static bool          s_lanVerified    = false;  // broker reached since link-up
static uint8_t       s_probeAttempts  = 0;
static unsigned long s_probeSince     = 0;    // millis() when current connect started
static unsigned long s_probeNextStart = 0;    // backoff after a failed probe round

static const uint8_t       PROBE_MAX_ATTEMPTS = 3;
static const unsigned long PROBE_TIMEOUT_MS   = 2000;   // per connect attempt
static const unsigned long PROBE_RETRY_MS     = 30000;  // after a failed round
static const unsigned long LIVENESS_INTERVAL_MS = 30000; // re-probe interval when verified

// Liveness re-verification: even a verified LAN is re-probed periodically.
// A wedged W5500 keeps PHY link + IP (no events fire) but passes no traffic;
// only an active probe detects that. After LIVENESS_MAX_FAIL_ROUNDS failed
// rounds we escalate to the WiFi fallback.
static unsigned long s_lastVerifyOkMs     = 0;
static uint8_t       s_livenessFailRounds = 0;
static const uint8_t LIVENESS_MAX_FAIL_ROUNDS = 3;

// W5500 wedge recovery: before falling back to WiFi, try a full W5500 restart
// (hardware reset + driver re-init). If MAX restarts don't recover the chip,
// then fall back to WiFi. While on WiFi fallback, periodically retry the W5500
// restart so LAN is restored automatically once the chip recovers.
static uint8_t       s_w5500ResetAttempts = 0;
static const uint8_t W5500_MAX_RESET_ATTEMPTS = 2;
static unsigned long s_w5500RecoveryAttemptMs = 0;
static const unsigned long W5500_RECOVERY_INTERVAL_MS = 120000;  // 2 min between recovery attempts on WiFi fallback

// Format an IPAddress as "a.b.c.d" without heap Strings (IPAddress::toString()
// allocates; used on the 2 s status path and in network event handlers).
static void fmtIP(char *buf, size_t len, const IPAddress &ip) {
  snprintf(buf, len, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

// Resolve AIO_SERVER (hostname or IP literal — hostByName handles both).
static bool resolveProbeHost(const char *host, IPAddress &out) {
  return Network.hostByName(host, out) == 1;
}

static void probeClose() {
  if (s_probeSock >= 0) {
    close(s_probeSock);
    s_probeSock = -1;
  }
}

// Start one non-blocking TCP connect to the broker, bound to the W5500's IP
// so the probe is forced out of the Ethernet interface.
static bool probeConnectStart(unsigned long nowMs) {
  probeClose();
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) return false;
  fcntl(s, F_SETFL, O_NONBLOCK);

  struct sockaddr_in local = {};
  local.sin_family      = AF_INET;
  local.sin_port        = 0;                       // ephemeral
  local.sin_addr.s_addr = (uint32_t)ETH.localIP();
  if (bind(s, (struct sockaddr *)&local, sizeof(local)) < 0) {
    close(s);
    return false;
  }

  struct sockaddr_in srv = {};
  srv.sin_family      = AF_INET;
  srv.sin_port        = htons(AIO_SERVERPORT);
  srv.sin_addr.s_addr = (uint32_t)s_probeServer;
  if (connect(s, (struct sockaddr *)&srv, sizeof(srv)) < 0 && errno != EINPROGRESS) {
    close(s);
    return false;
  }
  s_probeSock  = s;
  s_probeSince = nowMs;
  return true;
}

// Start one verification round: resolve the broker address, then let
// lanVerifyService() run the connect attempts non-blocking.
static void lanVerifyStart(unsigned long nowMs) {
  probeClose();   // drop any stale socket from a link flap
  if (!resolveProbeHost(AIO_SERVER, s_probeServer)) {
    Serial.println("[Net] LAN verify: broker DNS lookup failed -> retry in 60s");
    webLog("[Net] LAN verify: broker DNS lookup failed");
    s_probeNextStart = nowMs + PROBE_RETRY_MS;
    return;
  }
  {
    char ip[16]; fmtIP(ip, sizeof(ip), s_probeServer);
    Serial.printf("[Net] LAN verify: probing MQTT broker %s:%u via W5500\n",
                  ip, (unsigned)AIO_SERVERPORT);
    webLog("[Net] LAN verify: probing MQTT broker %s:%u via W5500", ip, (unsigned)AIO_SERVERPORT);
  }
  s_probeAttempts = 0;
  s_probeActive   = true;
}

// Non-blocking probe driver. Call every loop() while the LAN has an IP but is
// not yet verified. Sets s_lanVerified = true once a TCP connect to the MQTT
// broker succeeds through the W5500.
static void lanVerifyService(unsigned long nowMs) {
  if (s_lanVerified) return;
  if (!s_probeActive) {
    if ((long)(nowMs - s_probeNextStart) >= 0) lanVerifyStart(nowMs);
    return;
  }

  // No connect currently in flight: start the next attempt (or give up).
  if (s_probeSock < 0) {
    if (s_probeAttempts >= PROBE_MAX_ATTEMPTS) {
      s_probeActive    = false;
      s_probeNextStart = nowMs + PROBE_RETRY_MS;
      if (s_livenessFailRounds < 255) s_livenessFailRounds++;
      Serial.println("[Net] LAN verify FAILED (broker unreachable) -> keeping WiFi, retry in 60s");
      webLog("[Net] LAN verify: broker unreachable (fail round %u)", s_livenessFailRounds);
      return;
    }
    probeConnectStart(nowMs);   // a false return counts as a failed attempt
    s_probeAttempts++;
    return;
  }

  // Connect in flight: poll for completion (writable) without blocking.
  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(s_probeSock, &wfds);
  struct timeval tv = {0, 0};
  if (select(s_probeSock + 1, nullptr, &wfds, nullptr, &tv) > 0) {
    int err = 0;
    socklen_t errLen = sizeof(err);
    getsockopt(s_probeSock, SOL_SOCKET, SO_ERROR, &err, &errLen);
    probeClose();
    if (err == 0) {
      s_probeActive = false;
      s_lanVerified = true;
      s_lastVerifyOkMs     = nowMs;
      s_livenessFailRounds = 0;
      Serial.println("[Net] LAN verified (MQTT broker reachable via W5500) -> WiFi may be switched off");
      webLog("[Net] LAN verified: MQTT broker reachable");
    }
    return;   // err != 0 (refused/unreachable) -> next attempt
  }

  if (nowMs - s_probeSince >= PROBE_TIMEOUT_MS) {
    probeClose();   // attempt timed out -> next attempt
  }
}

// ---- W5500 full restart -------------------------------------------------
// Forward declaration: restartW5500() is defined here but calls startEthernet()
// which is defined further below.
static void startEthernet();

// Hardware-reset the W5500 via the RST pin, then re-init the ESP-IDF ETH
// driver (ETH.end + ETH.begin). This recovers from a wedged chip where SPI
// communication has stalled or internal state is corrupt. The link will
// re-negotiate and events (ETH_CONNECTED, ETH_GOT_IP) will fire normally.
// Total blocking time: ~160 ms (well under the 5 s task watchdog).
static void restartW5500() {
  Serial.println("[Net] W5500 full restart (HW reset + driver re-init)");
  webLog("[Net] W5500 full restart (HW reset + driver re-init)");

  // Clear state — events will set these again when the link comes back.
  s_ethHasIP     = false;
  s_ethLinkUp    = false;
  s_lanVerified  = false;
  s_probeActive  = false;
  probeClose();

  // Hardware reset: RST low for 10 ms, then high. The W5500 datasheet
  // specifies a minimum 10 us reset pulse; 10 ms is generous.
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  delay(10);
  digitalWrite(W5500_RST, HIGH);
  delay(50);   // W5500 needs ~50 ms after reset before SPI is responsive

  // Full driver re-init: tear down the esp_netif + ETH driver, then bring
  // it back up. This re-writes MAC registers, restarts DHCP/static config,
  // and re-registers the netif — something a bare hardware reset can't do.
  ETH.end();
  delay(100);  // let ESP-IDF fully clean up before re-init
  startEthernet();
}

// ---- mDNS (re)start ------------------------------------------------------
static void restartMDNS() {
  MDNS.end();
  if (MDNS.begin(s_hostname)) {
    MDNS.setInstanceName(s_hostname);
    MDNS.addService("http", "tcp", 80);
  }
}

// Deferred mDNS restart: event handlers run on the arduino_events task, where
// MDNS.end()/begin() heap operations race with the loop task. Set a flag and
// let loop() do the actual restart.
static volatile bool s_mdnsRestartPending = false;

// ---- WiFi bring-up -------------------------------------------------------
static void startWifi() {
  if (WiFi.getMode() != WIFI_OFF) return; // already active
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(s_hostname);
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
  ETH.setHostname(s_hostname);
  if (!LAN_DHCP) {
    IPAddress ip, gw, mask, dns;
    bool ok = ip.fromString(LAN_IP) && gw.fromString(LAN_GW) &&
              mask.fromString(LAN_MASK) && dns.fromString(LAN_DNS);
    if (ok && ETH.config(ip, gw, mask, dns)) {
      Serial.printf("[Net] W5500 static IP %s gw %s\n", LAN_IP, LAN_GW);
      webLog("[Net] W5500 static IP %s gw %s", LAN_IP, LAN_GW);
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
      ETH.setHostname(s_hostname);
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
      {
        char ip[16]; fmtIP(ip, sizeof(ip), ETH.localIP());
        Serial.printf("[Net] W5500 GOT IP %s\n", ip);
        webLog("[Net] W5500 GOT IP %s", ip);
      }
      s_mdnsRestartPending = true;  // deferred to loop() — no heap ops in event handler
      s_lanDownSince = 0;   // LAN is back up -> cancel any pending fallback timer
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      s_ethLinkUp = false;
      s_ethHasIP = false;
      s_lanVerified = false;      // connectivity must be re-proven after a flap
      s_probeActive = false;      // loop() closes the socket on the next round
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
      s_lanVerified = false;
      s_probeActive = false;
      Serial.println("[Net] event: ETH_STOP");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[Net] event: WIFI_STA_START");
      WiFi.setHostname(s_hostname);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      {
        char ip[16]; fmtIP(ip, sizeof(ip), WiFi.localIP());
        Serial.printf("[Net] WiFi GOT IP %s\n", ip);
        webLog("[Net] WiFi GOT IP %s", ip);
      }
      // Cache the SSID without heap allocation: WiFi.SSID() returns a String
      // (heap) which fragments the event-handler task. We always connect to
      // WLAN_SSID, so use it directly.
      strlcpy(s_ssid, WLAN_SSID, sizeof(s_ssid));
      s_mdnsRestartPending = true;  // deferred to loop() — no heap ops in event handler
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      s_ssid[0] = '\0';
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
  if (hostname && *hostname) strlcpy(s_hostname, hostname, sizeof(s_hostname));
  s_lanMode = (strcmp(NET_MODE, "lan") == 0);

  Network.onEvent(onNetEvent);

  Serial.printf("[Net] begin() NET_MODE=%s LAN_DHCP=%s\n", NET_MODE, LAN_DHCP ? "true" : "false");

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

  // Service deferred mDNS restart from event handler (prevents heap races).
  if (s_mdnsRestartPending) {
    s_mdnsRestartPending = false;
    restartMDNS();
  }

  // LAN verification driver: whenever the LAN has an IP but has not yet proven
  // end-to-end connectivity, run the MQTT broker probe (non-blocking). WiFi is only
  // switched off once this sets s_lanVerified.
  if (s_lanMode && s_ethHasIP && !s_lanVerified) {
    lanVerifyService(nowMs);
  }

  // Periodic liveness re-probe: clear the verified flag once per interval so
  // the probe above runs again. Detects a wedged W5500 despite up link/IP.
  if (s_lanMode && s_state == NetState::LAN && s_lanVerified &&
      (long)(nowMs - s_lastVerifyOkMs) >= (long)LIVENESS_INTERVAL_MS) {
    s_lanVerified = false;
  }

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
      webserver_closeAllSseClients();  // kill zombie SSE sockets from LAN
      startWifi();
    }
    return false;   // still waiting for LAN during first attempt
  }

  // In LAN state: make sure WiFi is fully off (RF silence, clean separation) —
  // but only once the LAN has been verified (MQTT broker reachable). Before that, WiFi may
  // still be the only working uplink and must stay alive.
  if (s_state == NetState::LAN && s_lanVerified && WiFi.getMode() != WIFI_OFF) {
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
    webserver_closeAllSseClients();  // kill zombie SSE sockets from LAN
    startWifi();
  }

  // LAN recovered while on WiFi fallback: switch back to LAN and turn WiFi
  // off — but ONLY after the LAN has been verified end-to-end (MQTT broker probe).
  // Until then the WiFi fallback stays up as the working uplink.
  if (s_lanMode && s_state == NetState::LAN_WIFI_FALLBACK && s_ethHasIP && s_lanVerified) {
    Serial.println("[Net] LAN verified while on WiFi -> switching back to LAN, WiFi OFF");
    webLog("[Net] LAN verified -> switching back from WiFi, WiFi OFF");
    s_state = NetState::LAN;
    s_lanDownSince = 0;
    s_w5500ResetAttempts = 0;
    s_w5500RecoveryAttemptMs = 0;
    webserver_closeAllSseClients();  // kill zombie SSE sockets from WiFi
    stopWifi();
  }

  // Liveness escalation: LAN still has link/IP but repeated broker probes
  // failed (e.g. wedged W5500). Try a full W5500 restart first — if the chip
  // recovers, we stay on Ethernet without needing WiFi at all. Only after
  // MAX restart attempts fail do we fall back to WiFi.
  if (s_lanMode && s_state == NetState::LAN &&
      s_livenessFailRounds >= LIVENESS_MAX_FAIL_ROUNDS) {
    if (s_w5500ResetAttempts < W5500_MAX_RESET_ATTEMPTS) {
      s_w5500ResetAttempts++;
      Serial.printf("[Net] LAN liveness failed -> W5500 restart %u/%u\n",
                    s_w5500ResetAttempts, W5500_MAX_RESET_ATTEMPTS);
      webLog("[Net] LAN liveness failed -> W5500 restart %u/%u",
             s_w5500ResetAttempts, W5500_MAX_RESET_ATTEMPTS);
      restartW5500();
      s_livenessFailRounds = 0;
      s_lanDeadline = nowMs + 15000;  // wait up to 15 s for link/IP after restart
    } else {
      Serial.println("[Net] W5500 restarts exhausted -> WiFi fallback");
      webLog("[Net] W5500 restarts exhausted -> WiFi fallback");
      s_state = NetState::LAN_WIFI_FALLBACK;
      s_livenessFailRounds = 0;
      s_w5500ResetAttempts = 0;
      s_w5500RecoveryAttemptMs = nowMs;  // start recovery timer
      webserver_closeAllSseClients();  // kill zombie SSE sockets from LAN
      startWifi();
    }
  }

  // Active W5500 recovery while on WiFi fallback: the W5500 may have been
  // wedged temporarily. Every 2 minutes, try a full restart so LAN is
  // restored automatically once the chip recovers. The existing
  // LAN_WIFI_FALLBACK -> LAN switch logic (above) handles the transition
  // once the W5500 gets an IP and passes verification.
  if (s_lanMode && s_state == NetState::LAN_WIFI_FALLBACK &&
      (s_w5500RecoveryAttemptMs == 0 ||
       (nowMs - s_w5500RecoveryAttemptMs) >= W5500_RECOVERY_INTERVAL_MS)) {
    s_w5500RecoveryAttemptMs = nowMs;
    Serial.println("[Net] WiFi fallback: attempting W5500 recovery");
    webLog("[Net] WiFi fallback: attempting W5500 recovery");
    restartW5500();
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

const char *activeIP() {
  static char buf[16];
  if (s_state == NetState::LAN && s_ethHasIP) {
    fmtIP(buf, sizeof(buf), ETH.localIP());
  } else if ((s_state == NetState::WIFI || s_state == NetState::LAN_WIFI_FALLBACK) &&
             WiFi.status() == WL_CONNECTED) {
    fmtIP(buf, sizeof(buf), WiFi.localIP());
  } else {
    strcpy(buf, "0.0.0.0");
  }
  return buf;
}

const char *ssid() { return s_ssid; }

void fillStatus(JsonVariant v) {
  v["netMode"]    = NET_MODE;
  v["netState"]   = (int)s_state;   // 0=offline,1=wifi,2=lan,3=lan_wifi_fallback
  v["netIface"]   = activeIface();
  v["netIP"]      = activeIP();
  v["ethLinkUp"]  = s_ethLinkUp;
  v["ethHasIP"]   = (bool)s_ethHasIP;
  v["lanVerified"] = s_lanVerified;
  v["lanDhcp"]    = LAN_DHCP;
  v["lanIP"]      = LAN_IP;
  v["lanGw"]      = LAN_GW;
  v["lanMask"]    = LAN_MASK;
  v["lanDns"]     = LAN_DNS;
}

}  // namespace NetManager
