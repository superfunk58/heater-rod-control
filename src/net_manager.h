#pragma once
// =========================================================================
// Network manager: WiFi vs. W5500 (SPI) Ethernet selection + failover.
//
// Modes (NET_MODE, persisted in NVS via ConfigStore):
//   "wifi" : WiFi only. W5500 is not touched.
//   "lan"  : W5500 Ethernet is primary. On boot the LAN interface is started
//            first (DHCP by default, or static IP when LAN_DHCP=false). If the
//            LAN does not obtain a link/IP within ~15 s, WiFi is started as a
//            fallback. Once the LAN obtains an IP, WiFi is switched OFF fully.
//            If the LAN later goes down, WiFi is brought back automatically.
//
// DHCP (LAN_DHCP=true) is the default for LAN and is adjustable in the web UI.
// When LAN_DHCP=false the W5500 uses the static IP/gateway/mask/DNS below.
//
// Requires Arduino-ESP32 Core 3.x (ETH_PHY_W5500). The W5500 registers as an
// esp_netif interface, so the existing PsychicHttp server / SSE / OTA / mDNS
// all work transparently over Ethernet.
// =========================================================================
#include <Arduino.h>
#include <ArduinoJson.h>

namespace NetManager {

// W5500 SPI pin map (ESP32 DevKit v1). All free, non-strapping, non-input-only.
// SCLK=18, MISO=19, MOSI=17, CS=16, INT=4, RST=32.
static constexpr int W5500_SCLK = 18;
static constexpr int W5500_MISO = 19;
static constexpr int W5500_MOSI = 17;
static constexpr int W5500_CS   = 16;
static constexpr int W5500_INT  = 4;
static constexpr int W5500_RST  = 32;

// Bring up the configured interface(s). Call ONCE in setup() AFTER
// ConfigStore::load(). Replaces the old inline WiFi.begin() bootstrap.
void begin(const char *hostname);

// Periodic housekeeping: WiFi reconnect (when applicable) and LAN->WiFi
// shutdown bookkeeping. Call every loop() iteration. Returns true if at least
// one interface currently has connectivity (i.e. the control loop may run).
bool loop();

// True if any interface currently has an IP (device is online).
bool isOnline();

// True if the W5500 LAN currently has an IP (WiFi is then off).
bool usingEthernet();

// "lan" | "wifi" | "none"
const char *activeIface();

// IP of the active interface ("0.0.0.0" if offline).
String activeIP();

// Raw W5500 PHY link state (cable plugged + negotiated), independent of IP.
bool ethLinkUp();

// Append network status fields to the /api/status JSON document.
void fillStatus(JsonVariant v);

}  // namespace NetManager
