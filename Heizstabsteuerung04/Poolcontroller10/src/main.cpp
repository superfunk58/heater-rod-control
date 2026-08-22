/*
 * Poolcontroller10 - ESP32 DevKit v1
 * --------------------------------------------------------------------------
 * Controls pool pump + 3x electrolysis relays + transformer relay.
 * Reads DS18B20 pool temperature, ORP via ADS1115, flow switch, calibration
 * button. Renders local status on ST7920 128x64. Remote control via:
 *   - PsychicHttp web UI (served from LittleFS /www/)
 *   - Server-Sent Events on /events for realtime telemetry (no polling)
 *   - MQTT (PubSubClient) with LWT + JSON status
 *   - HomeKit (HomeSpan) on port 1201
 *   - ArduinoOTA + HTTP OTA
 *
 * Why PsychicHttp instead of ESPAsyncWebServer:
 *   - Built on esp_http_server (Espressif-maintained, stable)
 *   - Handlers run on a worker thread pool, synchronously - no AsyncTCP
 *     socket-corruption races (handler calling mqttClient.publish() while
 *     loop() calls mqttClient.loop() used to wedge the server).
 *   - SSE built in (PsychicEventSource) - perfect for sensor telemetry.
 *
 * Pin layout: bottom header row of 30-pin DevKit when USB faces LEFT.
 * See Pinout.md for full wiring map, incl. 3.3V <-> 5V voltage notes.
 * --------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include <PsychicHttp.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <time.h>

#include <U8g2lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_ORP_PRO.h"
#include "homekit.h"
#include "history.h"
#include "manual.h"

// =========================================================================
// Identity / firmware version
// =========================================================================
// NOT static: homekit.cpp references these by `extern` linkage.
// DEVICE_HOSTNAME is filled at boot with a MAC suffix so two controllers
// in the same LAN don't collide on mDNS / WiFi-hostname / MQTT-clientId.
// Format: "poolcontroller-AABBCC" (last 3 bytes of WiFi MAC, hex).
static char  g_hostnameBuf[32]   = "poolcontroller";
const char  *DEVICE_HOSTNAME     = g_hostnameBuf;
const char  *FIRMWARE_VERSION    = "0.10.0-esp32";
static const char *FIRMWARE_BUILD_DATE = __DATE__;
static const char *FIRMWARE_BUILD_TIME = __TIME__;

// =========================================================================
// PIN MAP  -  see Pinout.md for the full 30-pin map, voltage notes and
// rationale. All actuators / sensors live on the BOTTOM row; display
// (HSPI) and I2C (ADS1115) live on the TOP row.
// =========================================================================

// ---- Bottom row (actuators + local I/O) --------------------------------
// ESP32 DevKit V1 bottom row (USB left): 3V3, GND, D16(RX2), D2, D4, D17(TX2), D5, D18, D19, D21, RX0, TX0, D22, D23
constexpr uint8_t PIN_PUMP        = 23;  // D23 - Pump relay        (active HIGH)
constexpr uint8_t PIN_ELY_A       = 22;  // D22 - Electrolysis A    (active HIGH)
constexpr uint8_t PIN_ELY_B       = 21;  // D21 - Electrolysis B    (active HIGH)
constexpr uint8_t PIN_ELY_K       = 12;  // D12 - Electrolysis K    (active HIGH)
constexpr uint8_t PIN_TRANSFORMER = 18;  // D18 - Transformer relay (active HIGH)
constexpr uint8_t PIN_TEMPSENS    = 5;   // D5  - DS18B20 data (needs 4.7k pull-up to 3V3)
constexpr uint8_t PIN_FLOW        = 4;   // D4  - Flow switch (INPUT_PULLUP, active LOW)
constexpr uint8_t PIN_BUTTON      = 16;  // D16 - Ely A/B toggle button (INPUT_PULLUP, active LOW)

// Built-in LED (onboard, bottom row - strap pin but output-only here)
constexpr uint8_t PIN_STATUS_LED  = 2;

// ---- Top row (display HSPI + I2C) ---------------------------------------
// HSPI default pins on ESP32: SCK=14, MISO=12, MOSI=13. CS is user-chosen.
constexpr uint8_t PIN_LCD_SCK     = 14;
constexpr uint8_t PIN_LCD_MOSI    = 13;
constexpr uint8_t PIN_LCD_CS      = 27;  // spare top-row output
constexpr uint8_t PIN_I2C_SDA     = 25;  // ADS1115 SDA (custom, not default 21)
constexpr uint8_t PIN_I2C_SCL     = 26;  // ADS1115 SCL (custom, not default 22)

// =========================================================================
// Misc constants
// =========================================================================
#define TEMPERATURE_PRECISION 12
#ifndef ADS1115_DR_8SPS
#define ADS1115_DR_8SPS 0x0000
#endif

// 1 Hz is plenty for the static info panel (no scrolling ticker any more).
static const unsigned long DISPLAY_REFRESH_INTERVAL = 1000;
static const unsigned long STATUS_PUBLISH_INTERVAL  = 10000; // MQTT + serial
static const unsigned long TEMP_REFRESH_INTERVAL    = 5000;
static const unsigned long SENSOR_READ_INTERVAL     = 1000;
static const unsigned long MAXFLOWERROR_DURATION    = 10000;
static const unsigned long FLOW_DEBOUNCE_MS         = 200;
static const unsigned long MQTT_RECONNECT_INTERVAL  = 5000;

// =========================================================================
// Hardware instances
// =========================================================================
// ST7920 via HW SPI (VSPI). U8g2 constructor for ESP32 HW-SPI:
static U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R3, PIN_LCD_CS, U8X8_PIN_NONE);

// =========================================================================
// Virtual display recorder
// =========================================================================
// Mirrors every draw call from drawDisplay() into a small op-list so the
// web UI can replay the screen contents exactly — without having to guess
// u8g2's internal framebuffer layout. See /api/lcd.json.
namespace DisplayRec {

enum OpType : uint8_t { OP_TEXT = 0, OP_BOX = 1 };

struct Op {
  uint8_t type;
  uint8_t invert;        // 1 => draw color was 0 (inverted text / erased box)
  int16_t x, y, w, h;    // y is text baseline for OP_TEXT; w,h unused there
  // Sized to fit the full row-9 quote ticker (<= ~140 chars). Short
  // labels only consume strlen+1 bytes at runtime since JSON serializes
  // the C-string; the excess is just static reservation.
  char    text[160];     // OP_TEXT only, null-terminated
};

static constexpr size_t MAX_OPS = 40;
static Op           ops[MAX_OPS];
static size_t       count     = 0;
static uint8_t      drawColor = 1;
static int16_t      cx = 0, cy = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     frameCounter = 0;

static void clear() {
  portENTER_CRITICAL(&mux);
  count = 0;
  portEXIT_CRITICAL(&mux);
  drawColor = 1;
  cx = cy = 0;
}
static void setColor(uint8_t c)              { drawColor = c ? 1 : 0; }
static void setCursor(int16_t x, int16_t y)  { cx = x; cy = y; }

static void pushText(const char *s) {
  portENTER_CRITICAL(&mux);
  if (count < MAX_OPS) {
    Op &o = ops[count++];
    o.type    = OP_TEXT;
    o.invert  = (drawColor == 0) ? 1 : 0;
    o.x = cx; o.y = cy; o.w = 0; o.h = 0;
    strlcpy(o.text, s, sizeof(o.text));
  }
  portEXIT_CRITICAL(&mux);
  cx += (int16_t)(strlen(s) * 7);   // 7x13B glyph advance
}

static void pushBox(int16_t x, int16_t y, int16_t w, int16_t h) {
  portENTER_CRITICAL(&mux);
  if (count < MAX_OPS) {
    Op &o = ops[count++];
    o.type   = OP_BOX;
    o.invert = (drawColor == 0) ? 1 : 0;
    o.x = x; o.y = y; o.w = w; o.h = h;
    o.text[0] = 0;
  }
  portEXIT_CRITICAL(&mux);
}

static void frameDone() { frameCounter++; }

// Snapshot the op list into a caller-supplied buffer under the lock.
static size_t snapshot(Op *out, size_t max) {
  portENTER_CRITICAL(&mux);
  const size_t n = count < max ? count : max;
  memcpy(out, ops, n * sizeof(Op));
  portEXIT_CRITICAL(&mux);
  return n;
}

} // namespace DisplayRec

// Thin wrappers: every call tees the operation to both the real u8g2
// framebuffer and the recorder above. drawDisplay() uses these exclusively.
namespace disp {
  static inline void clear()                               { u8g2.clearBuffer();      DisplayRec::clear(); }
  static inline void cursor(int16_t x, int16_t y)          { u8g2.setCursor(x, y);    DisplayRec::setCursor(x, y); }
  static inline void color(uint8_t c)                      { u8g2.setDrawColor(c);    DisplayRec::setColor(c); }
  static inline void text(const char *s)                   { u8g2.print(s);           DisplayRec::pushText(s); }
  static inline void text(int v) {
    char b[16]; snprintf(b, sizeof(b), "%d", v);
    u8g2.print(v);           DisplayRec::pushText(b);
  }
  static inline void box(int16_t x, int16_t y, int16_t w, int16_t h) {
    u8g2.drawBox(x, y, w, h); DisplayRec::pushBox(x, y, w, h);
  }
  static inline void send()                                { u8g2.sendBuffer();       DisplayRec::frameDone(); }
}

static Adafruit_ADS1115   ads;
static DFRobot_ORP_PRO    ORP(/*calibration*/ 0);
static OneWire            oneWire(PIN_TEMPSENS);
static DallasTemperature  Tempsensors(&oneWire);
// Two DS18B20s supported on the bus: pool + air. Role is assigned
// deterministically by ROM address: lower ROM -> pool, higher -> air.
static uint8_t            tempAddrPool[8] = {0};
static uint8_t            tempAddrAir[8]  = {0};
// All DS18B20 ROMs found on the bus (for UI sensor mapping)
static uint8_t            tempDiscoveredRoms[4][8] = {{0}};
static uint8_t            tempDiscoveredCount = 0;

static WiFiClient         espClient;
static PubSubClient       mqttClient(espClient);
static PsychicHttpServer  server;
static PsychicEventSource events;          // SSE broadcaster on /events
static Preferences        prefs;

// =========================================================================
// Configuration (persisted in NVS via Preferences)
// =========================================================================
struct AppConfig {
  String   wifiSsid       = "funknetz";
  String   wifiPassword   = "ja33tune";

  #ifdef MQTT_HOST_DEFAULT
  String   mqttHost       = MQTT_HOST_DEFAULT;
  #else
  String   mqttHost       = "192.168.178.83";
  #endif
  uint16_t mqttPort       = 1883;
  String   mqttUser       = "benson";
  String   mqttPassword   = "ja33tune";
  String   mqttBaseTopic  = "poolcontroller";
  uint16_t mqttIntervalSecs = 10;

  int16_t  orpCalibration = 0;    // mV offset vs 2480mV base

  // Ely A/B are mutually exclusive. When one is commanded ON the
  // alternator runs and swaps A<->B every elyABSwitchSecs seconds.
  // uint32_t (not 16) so values up to 24 h (86400 s) fit.
  uint32_t elyABSwitchSecs = 600; // default 10 min

  // Ely A/B automatic mode: when true, Ely A/B is started automatically
  // whenever the weekly schedule is active (and stopped outside of it).
  // When false, Ely A/B is only started via manual command / button.
  bool     elyABAuto       = false;

  bool     pumpStartOn     = true;  // initial pump state after (re)boot

  // Weekly schedule gating Ely A/B (Ely K and pump are NOT affected).
  // 7 days (0=Sunday..6=Saturday), 24 bits per day (bit h = HH:00..HH:59).
  // Default: all hours enabled (0xFFFFFF).
  bool     scheduleEnabled = false;
  uint32_t schedule[7] = {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF,
                          0xFFFFFF, 0xFFFFFF, 0xFFFFFF};

  // DS18B20 sensor mapping - manual override for sensor assignment
  // Stored as hex strings (e.g., "28FF1234567890A1")
  String   tempSensorPoolId = "";  // Empty = auto-detect (lowest ROM)
  String   tempSensorAirId  = "";  // Empty = auto-detect (second lowest ROM)
};
static AppConfig config;

// =========================================================================
// Runtime state
// =========================================================================
// These six globals are also referenced from homekit.cpp (extern), so they
// must have external linkage -- do NOT mark them `static`.
bool pumping            = true;   // re-initialized from config.pumpStartOn in setup()
bool flowing            = false;
static bool elyAcommand        = false;
static bool elyBcommand        = false;
bool elyKcommand        = false;
static bool transformercommand = false;
static bool flowerror          = false;
static bool ds18b20_found      = false;   // alias: pool sensor present
static bool ds18b20_air_found  = false;
static bool ads_available      = false;

float temp_pool = DEVICE_DISCONNECTED_C;   // extern'd by homekit.cpp
float temp_air  = DEVICE_DISCONNECTED_C;   // extern'd by homekit.cpp
static float voltage   = 0.0f;
static int   orpValue  = 0;

static unsigned long lastRefreshTime      = 0;
static unsigned long lastStatusPublish    = 0;
static unsigned long lastSsePush          = 0;
static unsigned long lastTempReadTime     = 0;
static unsigned long lastSensorReadTime   = 0;
static unsigned long starttime_flowerror  = 0;
static unsigned long lastMqttReconnect    = 0;

static bool          flowRawLast   = false;
static unsigned long flowStableAt  = 0;

static bool          tempConvPending = false;
static unsigned long tempConvStarted = 0;
static uint16_t      tempConvDelayMs = 750;
static uint8_t       tempFailStreak     = 0;
static uint8_t       tempAirFailStreak  = 0;
static const uint8_t TEMP_FAIL_LIMIT    = 3;

static bool          g_rebootPending = false;
static unsigned long g_rebootAt      = 0;

// Failsafe: if WiFi not connected for 10 seconds, enter normal operation mode
// (allow pump/ely control even without network)
static unsigned long g_wifiLostAt    = 0;
static bool          g_failsafeMode  = false;

// ---- Cross-task fences ---------------------------------------------------
// PsychicHttp handlers run on esp_http_server worker threads (not the main
// loop task). PubSubClient / WiFiClient are NOT thread-safe; calling
// mqttClient.publish() while loop() is doing mqttClient.loop() corrupts
// the TCP state and eventually wedges the whole web server. Handlers
// therefore just set these flags and the main loop performs the work.
static volatile bool g_mqttPublishPending = false;
static volatile bool g_mqttReconfigPending = false;
// SSE push request: set by handlers that change state (cmd/config/schedule),
// drained by loop() which calls events.send(). Coalesces rapid clicks into
// a single broadcast per loop iteration.
static volatile bool g_ssePushPending = false;

// Ely A/B alternator: A and B may never be ON simultaneously.
// Phase 0 = OFF (both off), 1 = A is the conducting cell, 2 = B is conducting.
// When active, we swap A<->B every config.elyABSwitchSecs seconds.
uint8_t       elyABPhase         = 0;   // 0/1/2  -- extern'd by homekit.cpp
unsigned long elyABLastSwitchMs  = 0;

// NTP
static bool          g_ntpSynced         = false;
static unsigned long g_lastNtpCheckMs    = 0;
static const unsigned long NTP_CHECK_INTERVAL_MS = 2000;

// =========================================================================
// Prefs load/save - table-driven so adding a field = one row
// =========================================================================
// Each row maps a short NVS key (<=15 chars) to one AppConfig member, plus
// the JSON/HTTP-param name used by the web UI. `min`/`max` bound numeric
// values at load time and on HTTP POST (0/0 = unbounded).
enum ConfType { CF_STR, CF_BOOL, CF_U16, CF_U32, CF_I16 };
struct PrefF {
  const char *nvs;      // NVS key
  const char *api;      // JSON/POST parameter name
  ConfType    type;
  void       *ptr;      // &config.field
  uint32_t    mn, mx;   // bounds for numeric types (mx==0 => unbounded)
  bool        secret;   // POST: don't overwrite with empty; GET: emit <api>_set
};
static const PrefF PREF_FIELDS[] = {
  {"wifiSsid",     "wifi_ssid",           CF_STR,  &config.wifiSsid,         0,0,        false},
  {"wifiPassword", "wifi_password",       CF_STR,  &config.wifiPassword,     0,0,        true },
  {"mqttHost",     "mqtt_host",           CF_STR,  &config.mqttHost,         0,0,        false},
  {"mqttPort",     "mqtt_port",           CF_U16,  &config.mqttPort,         1,65535,    false},
  {"mqttUser",     "mqtt_user",           CF_STR,  &config.mqttUser,         0,0,        false},
  {"mqttPassword", "mqtt_password",       CF_STR,  &config.mqttPassword,     0,0,        true },
  {"mqttBase",     "mqtt_base_topic",     CF_STR,  &config.mqttBaseTopic,    0,0,        false},
  {"mqttInterval", "mqtt_interval_secs",  CF_U16,  &config.mqttIntervalSecs, 1,300,      false},
  {"orpCal",       "orp_calibration",     CF_I16,  &config.orpCalibration,   0,0,        false},
  {"elyABSwitch",  "ely_ab_switch_secs",  CF_U32,  &config.elyABSwitchSecs,  10,86400,   false},
  {"elyABAuto",    "ely_ab_auto",         CF_BOOL, &config.elyABAuto,        0,0,        false},
  {"pumpStartOn",  "pump_start_on",       CF_BOOL, &config.pumpStartOn,      0,0,        false},
  {"schedEn",      "schedule_enabled",    CF_BOOL, &config.scheduleEnabled,  0,0,        false},
  {"tempPoolId",   "temp_sensor_pool_id", CF_STR,  &config.tempSensorPoolId, 0,0,        false},
  {"tempAirId",    "temp_sensor_air_id",  CF_STR,  &config.tempSensorAirId,  0,0,        false},
};

static void saveConfig() {
  prefs.begin("poolctl", false);
  for (auto &f : PREF_FIELDS) {
    switch (f.type) {
      case CF_STR:  prefs.putString(f.nvs, *(String*)f.ptr);                break;
      case CF_BOOL: prefs.putBool  (f.nvs, *(bool*)f.ptr);                  break;
      case CF_U16:  prefs.putUShort(f.nvs, *(uint16_t*)f.ptr);              break;
      case CF_U32:  prefs.putUInt  (f.nvs, *(uint32_t*)f.ptr);              break;
      case CF_I16:  prefs.putShort (f.nvs, *(int16_t*)f.ptr);               break;
    }
  }
  for (int d = 0; d < 7; d++) {
    char k[10]; snprintf(k, sizeof(k), "sched_%d", d);
    prefs.putUInt(k, config.schedule[d]);
  }
  prefs.end();
}

static void loadConfig() {
  prefs.begin("poolctl", true);
  for (auto &f : PREF_FIELDS) {
    switch (f.type) {
      case CF_STR:  *(String*)f.ptr   = prefs.getString(f.nvs, *(String*)f.ptr);        break;
      case CF_BOOL: *(bool*)f.ptr     = prefs.getBool  (f.nvs, *(bool*)f.ptr);          break;
      case CF_U16: {
        uint16_t v = prefs.getUShort(f.nvs, *(uint16_t*)f.ptr);
        if (f.mx && v > f.mx) v = f.mx;
        if (f.mn && v < f.mn) v = f.mn;
        *(uint16_t*)f.ptr = v;                                                          break;
      }
      case CF_U32: {
        uint32_t v = prefs.getUInt(f.nvs, *(uint32_t*)f.ptr);
        if (f.mx && v > f.mx) v = f.mx;
        if (f.mn && v < f.mn) v = f.mn;
        *(uint32_t*)f.ptr = v;                                                          break;
      }
      case CF_I16:  *(int16_t*)f.ptr  = prefs.getShort (f.nvs, *(int16_t*)f.ptr);       break;
    }
  }
  for (int d = 0; d < 7; d++) {
    char k[10]; snprintf(k, sizeof(k), "sched_%d", d);
    config.schedule[d] = prefs.getUInt(k, config.schedule[d]);
  }
  prefs.end();
}

// =========================================================================
// NTP helpers
// =========================================================================
// Poll getLocalTime() with a short timeout and cache the sync state.
// Call at most every NTP_CHECK_INTERVAL_MS from loop().
// After 15 failed attempts (~30s at 2s interval), stop polling to avoid
// wasting WiFi bandwidth. This gives NTP enough time at startup.
static uint8_t s_ntpFailCount = 0;
static bool ntpRefresh() {
  struct tm ti;
  const bool ok = getLocalTime(&ti, 1);  // 1ms timeout: fail fast if NTP not synced yet
  if (ok && !g_ntpSynced) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
    Serial.printf("[NTP] synced: %s\n", buf);
    s_ntpFailCount = 0;  // reset on success
  } else if (!ok) {
    s_ntpFailCount++;
    if (s_ntpFailCount >= 15) {
      if (s_ntpFailCount == 15) Serial.println("[NTP] gave up after 15 attempts (~30s)");
      return false;  // stop polling
    }
  }
  g_ntpSynced = ok;
  return ok;
}

// Fill `out` with "YYYY-MM-DD HH:MM:SS" (local time) or "" if NTP not synced.
static void ntpNowString(char *out, size_t len) {
  struct tm ti;
  if (!getLocalTime(&ti, 1)) { if (len) out[0] = '\0'; return; }
  strftime(out, len, "%Y-%m-%d %H:%M:%S", &ti);
}

// =========================================================================
// Weekly schedule helpers
// =========================================================================
// Returns true when the current local hour is enabled in the schedule
// (or the schedule is disabled, or NTP hasn't synced yet - fail-safe).
static bool isScheduleActive() {
  if (!config.scheduleEnabled) return true;
  if (!g_ntpSynced)            return true;   // NTP not ready -> allow
  struct tm ti;
  if (!getLocalTime(&ti, 1))   return true;
  const int wday = ti.tm_wday;                // 0=Sun..6=Sat
  const int hour = ti.tm_hour;                // 0..23
  return (config.schedule[wday] & (1UL << hour)) != 0;
}

// =========================================================================
// 1-Wire scan - finds the first valid DS18B20 (0x28/0x22/0x10) on the bus.
// =========================================================================
// Helper: convert hex string to ROM address
static bool hexStringToRom(const String &hexStr, uint8_t *rom) {
  if (hexStr.length() != 16) return false; // Must be 16 hex chars (8 bytes)
  for (int i = 0; i < 8; i++) {
    String byteStr = hexStr.substring(i * 2, i * 2 + 2);
    rom[i] = (uint8_t)strtol(byteStr.c_str(), nullptr, 16);
  }
  return true;
}

// Helper: convert ROM address to hex string
static String romToHexString(const uint8_t *rom) {
  String hex = "";
  for (int i = 0; i < 8; i++) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%02X", rom[i]);
    hex += buf;
  }
  return hex;
}

static bool scanOneWireBus(bool verbose = true) {
  ds18b20_found     = false;
  ds18b20_air_found = false;
  memset(tempAddrPool, 0, 8);
  memset(tempAddrAir,  0, 8);
  if (!oneWire.reset()) {
    if (verbose) Serial.println(F("[1W] no presence pulse"));
    return false;
  }
  oneWire.reset_search();

  // Check if manual sensor IDs are configured
  bool useManualPool = !config.tempSensorPoolId.isEmpty();
  bool useManualAir  = !config.tempSensorAirId.isEmpty();
  uint8_t manualPoolRom[8] = {0};
  uint8_t manualAirRom[8] = {0};

  if (useManualPool) {
    if (hexStringToRom(config.tempSensorPoolId, manualPoolRom)) {
      if (verbose) Serial.printf("[1W] Manual pool sensor ID: %s\n", config.tempSensorPoolId.c_str());
    } else {
      if (verbose) Serial.printf("[1W] Invalid pool sensor ID format, using auto-detect\n");
      useManualPool = false;
    }
  }
  if (useManualAir) {
    if (hexStringToRom(config.tempSensorAirId, manualAirRom)) {
      if (verbose) Serial.printf("[1W] Manual air sensor ID: %s\n", config.tempSensorAirId.c_str());
    } else {
      if (verbose) Serial.printf("[1W] Invalid air sensor ID format, using auto-detect\n");
      useManualAir = false;
    }
  }

  // Collect all DS18B20 ROMs on the bus
  uint8_t roms[4][8];
  uint8_t nFound = 0;
  uint8_t addr[8];
  while (oneWire.search(addr) && nFound < 4) {
    const bool crcOk = (OneWire::crc8(addr, 7) == addr[7]);
    const bool isDs  = (addr[0] == 0x28 || addr[0] == 0x22 || addr[0] == 0x10);
    if (crcOk && isDs) memcpy(roms[nFound++], addr, 8);
  }
  // Save discovered ROMs for UI display
  tempDiscoveredCount = nFound;
  for (uint8_t i = 0; i < nFound; i++) memcpy(tempDiscoveredRoms[i], roms[i], 8);

  // If using manual IDs, find matching ROMs
  if (useManualPool || useManualAir) {
    for (uint8_t i = 0; i < nFound; i++) {
      if (useManualPool && memcmp(roms[i], manualPoolRom, 8) == 0) {
        memcpy(tempAddrPool, roms[i], 8);
        ds18b20_found = true;
        if (verbose) Serial.printf("[1W] Found pool sensor by manual ID\n");
        useManualPool = false; // Found it
      }
      if (useManualAir && memcmp(roms[i], manualAirRom, 8) == 0) {
        memcpy(tempAddrAir, roms[i], 8);
        ds18b20_air_found = true;
        if (verbose) Serial.printf("[1W] Found air sensor by manual ID\n");
        useManualAir = false; // Found it
      }
    }
    // Warn if manual IDs not found
    if (useManualPool && verbose) Serial.printf("[1W] WARNING: Manual pool sensor ID not found on bus\n");
    if (useManualAir && verbose) Serial.printf("[1W] WARNING: Manual air sensor ID not found on bus\n");
  }

  // Auto-assign remaining sensors (if any)
  if (!ds18b20_found || !ds18b20_air_found) {
    // Simple selection sort by ROM bytes
    for (uint8_t i = 0; i + 1 < nFound; i++) {
      uint8_t min = i;
      for (uint8_t j = i + 1; j < nFound; j++) {
        if (memcmp(roms[j], roms[min], 8) < 0) min = j;
      }
      if (min != i) { uint8_t t[8]; memcpy(t, roms[i], 8); memcpy(roms[i], roms[min], 8); memcpy(roms[min], t, 8); }
    }
    if (!ds18b20_found && nFound >= 1) {
      memcpy(tempAddrPool, roms[0], 8);
      ds18b20_found = true;
      if (verbose) Serial.printf("[1W] Auto-assigned pool sensor: %s\n", romToHexString(roms[0]).c_str());
    }
    if (!ds18b20_air_found && nFound >= 2) {
      memcpy(tempAddrAir, roms[1], 8);
      ds18b20_air_found = true;
      if (verbose) Serial.printf("[1W] Auto-assigned air sensor: %s\n", romToHexString(roms[1]).c_str());
    }
  }

  if (verbose) Serial.printf("[1W] DS18B20 pool=%s air=%s\n",
                             ds18b20_found     ? "yes" : "no",
                             ds18b20_air_found ? "yes" : "no");
  return ds18b20_found;
}

// =========================================================================
// Daily rotating "Sinnspruch" for display row 9 (scrolling ticker).
// - Tries quotable.io (HTTPS, setInsecure) once per UTC day once NTP synced.
// - Persists last quote + fetch-day in NVS so a reboot doesn't lose it.
// - Falls back to a built-in list indexed by day-of-year when offline.
// =========================================================================
namespace Quote {
  // 15 short built-in Sinnsprueche (ASCII only - 7x13B has no UTF-8 glyphs).
  static const char *const FALLBACK[] = {
    "Wer lacht, lebt laenger. - Anon",
    "Der Weg ist das Ziel. - Konfuzius",
    "Carpe diem - nutze den Tag.",
    "Das Leben ist wie ein Pool - mal warm, mal kalt.",
    "Stilles Wasser ist tief.",
    "Wer schwimmt, der rostet nicht.",
    "Ein Tag ohne Lachen ist verloren. - Chaplin",
    "In der Ruhe liegt die Kraft.",
    "Auch kleine Wellen formen das Ufer.",
    "Sei das Chlor in der trueben Suppe des Lebens.",
    "Glueck ist das einzige, das sich verdoppelt wenn man es teilt.",
    "Alle Wege fuehren zum Pool.",
    "Heute schon geplanscht?",
    "Erst messen, dann dosieren. - Alter Poolweiser",
    "Der fruehe Vogel faengt den Keim."
  };
  static constexpr size_t FALLBACK_N = sizeof(FALLBACK) / sizeof(FALLBACK[0]);

  static String   s_current;
  static uint32_t s_fetchedDay = 0;   // epoch / 86400, UTC days since 1970

  // Async-fetch hand-off: the HTTPS GET blocks 3-6 s, far too long to run
  // inline from loop() (wedges sensor reads, flow debounce, display, SSE).
  // A one-shot task writes its result into s_fetchResult + s_fetchDone;
  // loop() picks it up the next time tick() runs.
  static volatile bool s_fetchDone       = false;
  static volatile bool s_fetchInProgress = false;
  static String        s_fetchResult;

  // Stores the current quote + day-of-fetch to NVS (namespace 'quote').
  static void persist() {
    Preferences p;
    if (p.begin("quote", false)) {
      p.putString("q",    s_current);
      p.putUInt  ("day",  s_fetchedDay);
      p.end();
    }
  }
  static void load() {
    Preferences p;
    if (p.begin("quote", true)) {
      s_current    = p.getString("q", "");
      s_fetchedDay = p.getUInt("day", 0);
      p.end();
    }
    if (s_current.isEmpty()) {
      // Start with today's fallback so row 9 is never empty on first boot.
      time_t now; time(&now);
      const uint32_t doy = (now > 1700000000) ? ((uint32_t)now / 86400UL) : 0;
      s_current = FALLBACK[doy % FALLBACK_N];
    }
  }

  // Replaces any non-ASCII byte with '?' so the 7x13B font can render it.
  static void asciify(String &s) {
    for (size_t i = 0; i < s.length(); i++) {
      const unsigned char c = (unsigned char)s[i];
      if (c < 0x20 || c > 0x7E) s[i] = '?';
    }
  }

  // Best-effort HTTPS GET of one random English quote (<=120 chars).
  // Returns "" on any error. Blocking - only called from main loop.
  static String fetchOnline() {
    if (WiFi.status() != WL_CONNECTED) return "";
    WiFiClientSecure cli;
    cli.setInsecure();
    cli.setTimeout(5);
    HTTPClient http;
    http.setTimeout(6000);
    if (!http.begin(cli, "https://api.quotable.io/random?maxLength=120")) return "";
    String out;
    const int code = http.GET();
    if (code == 200) {
      const String payload = http.getString();
      StaticJsonDocument<512> doc;
      if (!deserializeJson(doc, payload)) {
        const char *content = doc["content"] | "";
        const char *author  = doc["author"]  | "";
        if (*content) {
          out = content;
          if (*author) { out += " - "; out += author; }
        }
      }
    }
    http.end();
    return out;
  }

  // FreeRTOS task entry: performs the blocking HTTPS GET off the main
  // loop, then parks the result in s_fetchResult and raises s_fetchDone.
  // mbedTLS needs a big stack (~12 KB) when setInsecure() is used.
  static void fetchTaskFn(void *) {
    String q = fetchOnline();
    s_fetchResult     = q;
    s_fetchDone       = true;
    s_fetchInProgress = false;
    vTaskDelete(NULL);
  }

  // Called from loop(): consumes any completed fetch, then (at most once
  // per UTC day) spawns a background task to grab a fresh quote.
  // Never blocks more than a few microseconds.
  static void tick() {
    // 1) Harvest a completed background fetch.
    if (s_fetchDone) {
      s_fetchDone = false;
      if (!s_fetchResult.isEmpty()) {
        asciify(s_fetchResult);
        s_current  = s_fetchResult;
        persist();
      }
      s_fetchResult = String();
    }

    // 2) Decide whether to kick off a new fetch.
    time_t now; time(&now);
    if (now < 1700000000) return;                 // wait for NTP
    const uint32_t today = (uint32_t)now / 86400UL;
    if (today == s_fetchedDay && !s_current.isEmpty()) return;
    if (s_fetchInProgress) return;                // already running
    if (WiFi.status() != WL_CONNECTED) {
      // No net: use today's fallback so row 9 rotates daily anyway.
      s_current    = FALLBACK[today % FALLBACK_N];
      s_fetchedDay = today;
      persist();
      return;
    }

    // Mark the day immediately so we don't respawn every loop iteration.
    s_fetchedDay      = today;
    s_fetchInProgress = true;
    BaseType_t ok = xTaskCreate(fetchTaskFn, "quote", 12288,
                                nullptr, tskIDLE_PRIORITY + 1, nullptr);
    if (ok != pdPASS) {
      s_fetchInProgress = false;
      s_current = FALLBACK[today % FALLBACK_N];
      persist();
    }
  }

  static const String& current() { return s_current; }
} // namespace Quote

// =========================================================================
// Display
// =========================================================================
// Physisches Display - direkt wie pc07 mit u8g2 ansteuern
// Layout: oben Luft/H2O/ORP, Mitte Relais, unten Raw Voltage
static void drawPhysicalDisplay() {
  // Defensive recovery: if a corrupted SPI byte ever flips the ST7920 out
  // of graphics mode, sendBuffer() alone won't bring it back -- the
  // controller would keep rendering pixel data as CJK character codes
  // forever. Re-asserting the function-set every frame is cheap (3 bytes)
  // and guarantees we recover within one refresh cycle.
  u8g2.initDisplay();
  u8g2.setPowerSave(0);

  u8g2.clearBuffer();

  char floatstring[12];
  const int rowHeight = 14;
  const int startY    = 11;

  // Row 1: LUFT
  u8g2.setCursor(0, startY);
  u8g2.print("LUFT");
  u8g2.setCursor(36, startY);
  if (ds18b20_air_found && temp_air > -50.0f && temp_air < 125.0f) {
    dtostrf(temp_air, 4, 1, floatstring);
    u8g2.print(floatstring);
  } else {
    u8g2.print("----");
  }

  // Row 2: H2O
  u8g2.setCursor(0, startY + rowHeight);
  u8g2.print("H2O");
  u8g2.setCursor(36, startY + rowHeight);
  if (ds18b20_found && temp_pool > -50.0f && temp_pool < 125.0f) {
    dtostrf(temp_pool, 4, 1, floatstring);
    u8g2.print(floatstring);
  } else {
    u8g2.print("----");
  }

  // Row 3: ORP
  u8g2.setCursor(0, startY + rowHeight * 2);
  u8g2.print("ORP");
  u8g2.setCursor(36, startY + rowHeight * 2);
  dtostrf(orpValue, 4, 0, floatstring);
  u8g2.print(floatstring);

  // Rows 4-8: FLOW / PUMP / EL A / EL B / EL K
  struct Row { const char *label; bool state; };
  const Row rows[] = {
    {"FLOW", flowing},
    {"PUMP", pumping},
    {"EL A", elyAcommand},
    {"EL B", elyBcommand},
    {"EL K", elyKcommand},
  };
  for (uint8_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
    const int y = startY + rowHeight * (3 + i);
    u8g2.setCursor(0, y);
    u8g2.print(rows[i].label);
    if (rows[i].state) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(36, y - 10, 26, 12);
      u8g2.setDrawColor(0);
      u8g2.setCursor(36, y);
      u8g2.print(" ON");
      u8g2.setDrawColor(1);
    } else {
      u8g2.setCursor(36, y);
      u8g2.print("OFF");
    }
  }

  // Row 9 (letzte): Raw ADC voltage
  dtostrf(voltage, 7, 4, floatstring);
  u8g2.setCursor(0, startY + rowHeight * 8);
  u8g2.print(floatstring);

  u8g2.sendBuffer();
}

// Virtuelles Display - schreibt NUR in den Recorder (nicht zum physischen LCD,
// das wird parallel in drawPhysicalDisplay() bedient).
// Layout: oben Luft/H2O/ORP, Mitte Relais, unten Raw Voltage (identisch zu physisch)
static void drawVirtualDisplay() {
  DisplayRec::clear();

  char floatstring[12];
  const int rowHeight = 14;
  const int startY    = 11;

  // Row 1: LUFT
  DisplayRec::setCursor(0, startY);
  DisplayRec::pushText("LUFT");
  DisplayRec::setCursor(36, startY);
  if (ds18b20_air_found && temp_air > -50.0f && temp_air < 125.0f) {
    dtostrf(temp_air, 4, 1, floatstring);
    DisplayRec::pushText(floatstring);
  } else {
    DisplayRec::pushText("----");
  }

  // Row 2: H2O
  DisplayRec::setCursor(0, startY + rowHeight);
  DisplayRec::pushText("H2O");
  DisplayRec::setCursor(36, startY + rowHeight);
  if (ds18b20_found && temp_pool > -50.0f && temp_pool < 125.0f) {
    dtostrf(temp_pool, 4, 1, floatstring);
    DisplayRec::pushText(floatstring);
  } else {
    DisplayRec::pushText("----");
  }

  // Row 3: ORP
  DisplayRec::setCursor(0, startY + rowHeight * 2);
  DisplayRec::pushText("ORP");
  DisplayRec::setCursor(36, startY + rowHeight * 2);
  dtostrf(orpValue, 4, 0, floatstring);
  DisplayRec::pushText(floatstring);

  // Rows 4-8: FLOW / PUMP / EL A / EL B / EL K
  struct Row { const char *label; bool state; };
  const Row rows[] = {
    {"FLOW", flowing},
    {"PUMP", pumping},
    {"EL A", elyAcommand},
    {"EL B", elyBcommand},
    {"EL K", elyKcommand},
  };
  for (uint8_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
    const int y = startY + rowHeight * (3 + i);
    DisplayRec::setCursor(0, y);
    DisplayRec::pushText(rows[i].label);
    if (rows[i].state) {
      DisplayRec::pushBox(36, y - 10, 26, 12);
      DisplayRec::setColor(0);
      DisplayRec::setCursor(36, y);
      DisplayRec::pushText(" ON");
      DisplayRec::setColor(1);
    } else {
      DisplayRec::setCursor(36, y);
      DisplayRec::pushText("OFF");
    }
  }

  // Row 9 (letzte): Raw ADC voltage
  dtostrf(voltage, 7, 4, floatstring);
  DisplayRec::setCursor(0, startY + rowHeight * 8);
  DisplayRec::pushText(floatstring);

  DisplayRec::frameDone();
}

// =========================================================================
// JSON status builder (reused for /api/status and MQTT)
// =========================================================================
static void buildStatusJson(JsonDocument &doc) {
  doc["firmware"]     = FIRMWARE_VERSION;
  doc["build"]        = String(FIRMWARE_BUILD_DATE) + " " + FIRMWARE_BUILD_TIME;
  doc["hostname"]     = DEVICE_HOSTNAME;
  doc["uptime_s"]     = (unsigned long)(millis() / 1000);
  doc["wifi_ssid"]    = WiFi.SSID();
  doc["wifi_rssi"]    = WiFi.RSSI();
  doc["ip"]           = WiFi.localIP().toString();

  doc["pump"]         = pumping;
  doc["ely_a"]        = elyAcommand;
  doc["ely_b"]        = elyBcommand;
  doc["ely_k"]        = elyKcommand;
  doc["transformer"]  = transformercommand;
  doc["flow"]         = flowing;
  doc["flow_error"]   = flowerror;

  if (ds18b20_found && temp_pool > -50.0f && temp_pool < 125.0f) {
    doc["temp_pool"]  = roundf(temp_pool * 10) / 10.0f;
  } else {
    doc["temp_pool"]  = nullptr;
  }
  if (ds18b20_air_found && temp_air > -50.0f && temp_air < 125.0f) {
    doc["temp_air"]   = roundf(temp_air * 10) / 10.0f;
  } else {
    doc["temp_air"]   = nullptr;
  }
  doc["quote"] = Quote::current();

  if (ads_available) {
    doc["orp_mv"]     = orpValue;
    doc["voltage"]    = roundf(voltage * 10000) / 10000.0f;
  } else {
    doc["orp_mv"]     = nullptr;
    doc["voltage"]    = nullptr;
  }
  doc["sensor_ds18b20_ok"] = ds18b20_found;
  doc["sensor_ads1115_ok"] = ads_available;
  doc["mqtt_connected"]    = mqttClient.connected();

  // DS18B20 ROM addresses (for sensor mapping UI)
  doc["temp_pool_rom"] = ds18b20_found     ? romToHexString(tempAddrPool) : String();
  doc["temp_air_rom"]  = ds18b20_air_found ? romToHexString(tempAddrAir)  : String();
  JsonArray discovered = doc["temp_discovered"].to<JsonArray>();
  for (uint8_t i = 0; i < tempDiscoveredCount; i++) {
    discovered.add(romToHexString(tempDiscoveredRoms[i]));
  }

  // HomeKit
  doc["homekit_paired"]     = homekitIsPaired();
  doc["homekit_setup_code"] = homekitGetSetupCode();

  // NTP / clock
  doc["ntp_synced"] = g_ntpSynced;
  if (g_ntpSynced) {
    char buf[32];
    ntpNowString(buf, sizeof(buf));
    doc["now"] = buf;
    time_t t = time(nullptr);
    doc["now_epoch"] = (uint32_t)t;
  } else {
    doc["now"]       = nullptr;
    doc["now_epoch"] = nullptr;
  }

  // Schedule
  doc["schedule_enabled"] = config.scheduleEnabled;
  doc["schedule_active"]  = isScheduleActive();

  // Ely A/B alternator
  doc["ely_ab_phase"]        = elyABPhase;               // 0/1/2
  doc["ely_ab_switch_secs"]  = config.elyABSwitchSecs;
  doc["ely_ab_auto"]         = config.elyABAuto;
  if (elyABPhase != 0) {
    const unsigned long intervalMs = (unsigned long)config.elyABSwitchSecs * 1000UL;
    const unsigned long elapsed    = millis() - elyABLastSwitchMs;
    const long remain = (long)intervalMs - (long)elapsed;
    doc["ely_ab_next_switch_secs"] = (remain > 0) ? (remain / 1000) : 0;
  } else {
    doc["ely_ab_next_switch_secs"] = nullptr;
  }
}

// =========================================================================
// MQTT
// =========================================================================
static String topic(const char *leaf) {
  return config.mqttBaseTopic + "/" + leaf;
}

static bool parseOnOff(const String &v) {
  String s = v; s.toLowerCase();
  return (s == "on" || s == "1" || s == "true");
}

static void applyCommand(const String &what, const String &payload) {
  const bool on = parseOnOff(payload);
  if (what == "pump") {
    pumping = on;
  } else if (what == "ely_a") {
    if (on) {
      elyABPhase = 1;                         // A conducts, B forced off
      elyABLastSwitchMs = millis();
    } else if (elyABPhase == 1) {
      elyABPhase = 0;                         // user turned off the active cell
    }
    // commanding ely_a=off while phase==2 (B active) is a no-op
  } else if (what == "ely_b") {
    if (on) {
      elyABPhase = 2;
      elyABLastSwitchMs = millis();
    } else if (elyABPhase == 2) {
      elyABPhase = 0;
    }
  } else if (what == "ely_k") {
    elyKcommand = on;
  } else {
    return;
  }
  // Keep the derived outputs in lock-step with the phase so the status
  // response that handleCmd() builds right after this call already
  // reflects the new state.
  elyAcommand = (elyABPhase == 1);
  elyBcommand = (elyABPhase == 2);
  // NB: no Serial.printf here - this function runs on the AsyncTCP task
  // when called from /api/cmd; a full Serial TX FIFO would block the
  // task and wedge the web server under rapid clicking.
}

static void publishStatus() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<2048> doc;
  buildStatusJson(doc);
  char buf[2048];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqttClient.publish(topic("status").c_str(), (const uint8_t*)buf, n, true);
}

static void onMqttMessage(char *topicC, byte *payload, unsigned int len) {
  String t = topicC;
  String msg;
  msg.reserve(len + 1);
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];

  String base = config.mqttBaseTopic + "/cmd/";
  if (t.startsWith(base)) {
    String what = t.substring(base.length());
    applyCommand(what, msg);
  } else if (t == config.mqttBaseTopic + "/cmd") {
    // JSON bulk command: {"pump":"on","ely_a":"off"...}
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, msg) == DeserializationError::Ok) {
      for (JsonPair kv : doc.as<JsonObject>()) {
        applyCommand(kv.key().c_str(), kv.value().as<String>());
      }
    }
  }
}

static void mqttReconnect() {
  if (mqttClient.connected() || WiFi.status() != WL_CONNECTED) return;
  const unsigned long now = millis();
  if (now - lastMqttReconnect < MQTT_RECONNECT_INTERVAL) return;
  lastMqttReconnect = now;

  String willTopic = topic("available");
  // DEVICE_HOSTNAME already carries the MAC suffix (see g_hostnameBuf in
  // setup()), so it doubles as a unique MQTT client ID. The broker would
  // otherwise kick off whichever session shares the ID.
  Serial.printf("[MQTT] Attempting connect to %s:%u as %s\n",
                config.mqttHost.c_str(), config.mqttPort, DEVICE_HOSTNAME);
  bool ok = mqttClient.connect(
    DEVICE_HOSTNAME,
    config.mqttUser.c_str(),
    config.mqttPassword.c_str(),
    willTopic.c_str(), 0, true, "offline");

  if (ok) {
    Serial.println("[MQTT] connected");
    mqttClient.publish(willTopic.c_str(), "online", true);
    mqttClient.subscribe((config.mqttBaseTopic + "/cmd/#").c_str());
    mqttClient.subscribe((config.mqttBaseTopic + "/cmd").c_str());
    publishStatus();
  } else {
    Serial.printf("[MQTT] connect failed, rc=%d\n", mqttClient.state());
  }
}

// =========================================================================
// Web handlers  (PsychicHttp)
// =========================================================================
// Threading model recap:
//   * Each handler runs synchronously on one of the esp_http_server worker
//     threads (see server.config.max_uri_handlers / stack_size in setup()).
//   * PubSubClient / WiFiClient (MQTT) are NOT thread-safe, so we never
//     call mqttClient.publish() / .connect() from a handler - they are
//     deferred to the main loop via g_mqttPublishPending / g_mqttReconfig-
//     Pending flags.
//   * Serial is buffered (1024 B TX ring in setup()) so print/printf can
//     be called cheaply without blocking.
//   * I2C (ADS1115) is owned by the main loop; handlers consume the
//     cached `voltage` / `orpValue` globals.
//
// Request-parameter helper. PsychicHttp merges URL query params and
// x-www-form-urlencoded POST body params into the same getParam() API,
// so unlike AsyncWebServer we don't need the POST-vs-GET distinction.
static String paramOr(PsychicRequest *req, const char *name, const String &def) {
  if (req->hasParam(name)) return req->getParam(name)->value();
  return def;
}

static esp_err_t handleStatus(PsychicRequest *req) {
  DynamicJsonDocument doc(2048);
  buildStatusJson(doc);
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

static esp_err_t handleCmd(PsychicRequest *req) {
  const char *keys[] = {"pump", "ely_a", "ely_b", "ely_k"};
  bool changed = false;
  for (auto k : keys) {
    String v = paramOr(req, k, "");
    if (v.length()) { applyCommand(k, v); changed = true; }
  }
  // MQTT: rapid clicks used to flood the broker and stall the main loop
  // inside mqttClient.publish(). The periodic status publisher in loop()
  // catches up automatically; subscribers see state within mqttIntervalSecs.
  //
  // SSE: broadcast the new state immediately so the web UI updates in
  // <50 ms (no more 2 s polling lag). events.send() is thread-safe.
  if (changed) g_ssePushPending = true;
  return req->reply(200, "application/json", "{\"success\":true}");
}

static esp_err_t handleConfigGet(PsychicRequest *req) {
  DynamicJsonDocument doc(2048);
  for (auto &f : PREF_FIELDS) {
    if (f.secret) {
      // e.g. "wifi_password_set":true/false  (value is never leaked)
      String k = String(f.api) + "_set";
      doc[k] = ((String*)f.ptr)->length() > 0;
      continue;
    }
    switch (f.type) {
      case CF_STR:  doc[f.api] = *(String*)f.ptr;          break;
      case CF_BOOL: doc[f.api] = *(bool*)f.ptr;            break;
      case CF_U16:  doc[f.api] = *(uint16_t*)f.ptr;        break;
      case CF_U32:  doc[f.api] = *(uint32_t*)f.ptr;        break;
      case CF_I16:  doc[f.api] = *(int16_t*)f.ptr;         break;
    }
  }
  JsonArray sch = doc.createNestedArray("schedule");
  for (int d = 0; d < 7; d++) sch.add(config.schedule[d]);
  doc["firmware_version"]  = FIRMWARE_VERSION;
  doc["build_date"]        = FIRMWARE_BUILD_DATE;
  doc["build_time"]        = FIRMWARE_BUILD_TIME;
  doc["hostname"]          = DEVICE_HOSTNAME;
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

static esp_err_t handleConfigPost(PsychicRequest *req) {
  bool mqttChanged = false;
  bool elyABSwitchChanged = false;
  bool sensorIdChanged = false;

  for (auto &f : PREF_FIELDS) {
    if (!req->hasParam(f.api)) continue;
    const String v = req->getParam(f.api)->value();

    // Track side-effects
    const String api = f.api;
    if (api.startsWith("mqtt_")) mqttChanged = true;
    if (api == "ely_ab_switch_secs") elyABSwitchChanged = true;
    if (api == "temp_sensor_pool_id" || api == "temp_sensor_air_id") sensorIdChanged = true;

    switch (f.type) {
      case CF_STR:
        if (f.secret && v.length() == 0) break;   // don't wipe passwords
        *(String*)f.ptr = v;
        break;
      case CF_BOOL: {
        String s = v; s.toLowerCase();
        *(bool*)f.ptr = (s == "true" || s == "1" || s == "on" || s == "yes");
        break;
      }
      case CF_U16: {
        long x = v.toInt();
        if (f.mn && x < (long)f.mn) break;
        if (f.mx && x > (long)f.mx) break;
        *(uint16_t*)f.ptr = (uint16_t)x;
        break;
      }
      case CF_U32: {
        long x = v.toInt();
        if (f.mn && x < (long)f.mn) break;
        if (f.mx && x > (long)f.mx) break;
        *(uint32_t*)f.ptr = (uint32_t)x;
        break;
      }
      case CF_I16:
        *(int16_t*)f.ptr = (int16_t)v.toInt();
        if (api == "orp_calibration") ORP.setCalibration(config.orpCalibration);
        break;
    }
  }

  if (elyABSwitchChanged) elyABLastSwitchMs = millis();

  saveConfig();

  // Trigger 1-Wire scan if sensor IDs changed
  if (sensorIdChanged) {
    scanOneWireBus(true);
    Serial.println("[Config] Sensor IDs changed, bus scan triggered");
  }

  // Defer MQTT reconfigure to the main loop (thread-safety).
  if (mqttChanged) g_mqttReconfigPending = true;
  g_ssePushPending = true;

  return req->reply(200, "application/json", "{\"success\":true}");
}

static esp_err_t handleSchedulePost(PsychicRequest *req) {
  if (req->hasParam("schedule_enabled")) {
    String v = req->getParam("schedule_enabled")->value();
    v.toLowerCase();
    config.scheduleEnabled = (v == "true" || v == "1" || v == "on");
  }
  for (int d = 0; d < 7; d++) {
    String key = "schedule_" + String(d);
    if (req->hasParam(key.c_str())) {
      uint32_t v = (uint32_t)strtoul(req->getParam(key.c_str())->value().c_str(), nullptr, 10);
      config.schedule[d] = v & 0xFFFFFFul;   // mask to 24 bits
    }
  }
  saveConfig();
  g_ssePushPending = true;
  return req->reply(200, "application/json", "{\"success\":true}");
}

static esp_err_t handleHomekitReset(PsychicRequest *req) {
  // Ack first, then wipe pairings + reboot. homekitResetPairings() calls
  // ESP.restart() internally, so we must send the response before that.
  esp_err_t rc = req->reply(200, "application/json",
    "{\"success\":true,\"hk_reset\":true,\"rebooting\":true}");
  Serial.println("[HK] Pairing reset requested via web");
  homekitResetPairings();
  return rc;  // unreachable in practice - device has rebooted
}

static esp_err_t handleReboot(PsychicRequest *req) {
  esp_err_t rc = req->reply(200, "application/json",
    "{\"success\":true,\"rebooting\":true}");
  g_rebootPending = true;
  g_rebootAt = millis() + 500;
  return rc;
}

// ---- Tab order persistence (UI preference) -------------------------------
// The web UI lets users drag tabs to reorder them. We persist the resulting
// CSV ("status,schedule,display,...") in its own NVS namespace so a single
// order is shared across browsers and survives reboots / OTA (NVS is not
// touched by app-partition OTA). Total payload is <100 B.
static esp_err_t handleTabsGet(PsychicRequest *req) {
  Preferences p;
  String order;
  if (p.begin("ui", true)) { order = p.getString("tabs", ""); p.end(); }
  String out = "{\"order\":\"" + order + "\"}";
  return req->reply(200, "application/json", out.c_str());
}

static esp_err_t handleTabsPost(PsychicRequest *req) {
  if (!req->hasParam("order")) return req->reply(400, "text/plain", "missing order");
  String order = req->getParam("order")->value();
  // Whitelist: alnum + comma + dash + underscore, <= 128 chars, defends NVS from garbage.
  if (order.length() > 128) return req->reply(400, "text/plain", "too long");
  for (size_t i = 0; i < order.length(); i++) {
    const char c = order[i];
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                    (c >= '0' && c <= '9') || c == ',' || c == '-' || c == '_';
    if (!ok) return req->reply(400, "text/plain", "bad chars");
  }
  Preferences p;
  if (!p.begin("ui", false)) {
    Serial.println("[Tabs] Failed to open NVS namespace 'ui'");
    return req->reply(500, "text/plain", "nvs");
  }
  p.putString("tabs", order);
  Serial.printf("[Tabs] Saved order: %s\n", order.c_str());
  p.end();
  return req->reply(200, "application/json", "{\"success\":true}");
}

static esp_err_t handleWifiScan(PsychicRequest *req) {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED) {
    WiFi.scanNetworks(true);
    return req->reply(200, "application/json", "{\"scanning\":true}");
  }
  if (n == WIFI_SCAN_RUNNING) {
    return req->reply(200, "application/json", "{\"scanning\":true}");
  }
  DynamicJsonDocument doc(4096);
  doc["scanning"] = false;
  JsonArray arr = doc.createNestedArray("networks");
  for (int i = 0; i < n; i++) {
    JsonObject net = arr.createNestedObject();
    net["ssid"]    = WiFi.SSID(i);
    net["rssi"]    = WiFi.RSSI(i);
    net["channel"] = WiFi.channel(i);
    net["secure"]  = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    net["bssid"]   = WiFi.BSSIDstr(i);
  }
  WiFi.scanDelete();
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

// Trigger a 1-Wire bus scan from the web UI (no restart needed).
static esp_err_t handleOneWireScan(PsychicRequest *req) {
  DynamicJsonDocument doc(512);
  bool ok = scanOneWireBus(true);
  if (ok) {
    Tempsensors.setResolution(tempAddrPool, TEMPERATURE_PRECISION);
    if (ds18b20_air_found) Tempsensors.setResolution(tempAddrAir, TEMPERATURE_PRECISION);
    tempConvDelayMs = Tempsensors.millisToWaitForConversion(TEMPERATURE_PRECISION);
  }
  doc["success"] = ok;
  doc["pool_found"] = ds18b20_found;
  doc["air_found"]  = ds18b20_air_found;
  if (ds18b20_found)     doc["pool_rom"] = romToHexString(tempAddrPool);
  if (ds18b20_air_found) doc["air_rom"]  = romToHexString(tempAddrAir);
  JsonArray discovered = doc["discovered"].to<JsonArray>();
  for (uint8_t i = 0; i < tempDiscoveredCount; i++) {
    discovered.add(romToHexString(tempDiscoveredRoms[i]));
  }
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

// Serve the virtual-display recorder's op list as JSON. The browser replays
// these ops onto a Canvas - no framebuffer guessing required.
static esp_err_t handleLcdJson(PsychicRequest *req) {
  // Snapshot on the heap, not the stack: sizeof(Op)*MAX_OPS is ~7 KB
  // (text[160] per op), which overflows the esp_http_server worker stack.
  std::unique_ptr<DisplayRec::Op[]> localOps(new DisplayRec::Op[DisplayRec::MAX_OPS]);
  if (!localOps) return req->reply(500, "text/plain", "oom");
  const size_t n = DisplayRec::snapshot(localOps.get(), DisplayRec::MAX_OPS);

  // Sized for worst case: ~40 ops, 2 of them with 160-char quote strings,
  // plus keys and JSON tree overhead.
  DynamicJsonDocument doc(8192);
  doc["w"]     = u8g2.getDisplayWidth();     // logical width  (64 with R3)
  doc["h"]     = u8g2.getDisplayHeight();    // logical height (128 with R3)
  doc["font"]  = "7x13B";
  doc["cell"]  = 7;                          // glyph advance in px
  doc["line"]  = 13;                         // line height / glyph height
  doc["frame"] = DisplayRec::frameCounter;

  JsonArray arr = doc.createNestedArray("ops");
  for (size_t i = 0; i < n; i++) {
    const DisplayRec::Op &o = localOps[i];
    JsonObject j = arr.createNestedObject();
    if (o.type == DisplayRec::OP_TEXT) {
      j["t"] = "t";
      j["x"] = o.x; j["y"] = o.y;
      j["s"] = o.text;
      if (o.invert) j["i"] = 1;
    } else { // OP_BOX
      j["t"] = "b";
      j["x"] = o.x; j["y"] = o.y;
      j["w"] = o.w; j["h"] = o.h;
      if (o.invert) j["i"] = 1;
    }
  }

  String out; serializeJson(doc, out);
  PsychicResponse res(req);
  res.setCode(200);
  res.setContentType("application/json");
  res.addHeader("Cache-Control", "no-store");
  res.setContent(out.c_str());
  return res.send();
}

static esp_err_t handleOrpCalibrate(PsychicRequest *req) {
  // User must have shorted the ORP input and waited for a stable reading.
  // Use the LAST value sampled by loop() (1 Hz) instead of calling I2C
  // from this worker thread - the I2C bus and Adafruit_ADS1115 state are
  // not thread-safe against the main loop's reader.
  if (!ads_available) {
    return req->reply(500, "application/json", "{\"success\":false,\"error\":\"no_ads\"}");
  }
  const float mv = voltage * 1000.0f;
  float offset = ORP.calibrate(mv);
  config.orpCalibration = (int16_t)offset;
  saveConfig();

  StaticJsonDocument<128> doc;
  doc["success"] = true;
  doc["calibration"] = config.orpCalibration;
  doc["raw_mv"] = mv;
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

// =========================================================================
// SSE broadcaster
// =========================================================================
// Called from the main loop (never from a handler) when:
//   * g_ssePushPending was set by a state-changing handler (immediate)
//   * periodic 2 s heartbeat (so browsers that connect mid-session still
//     see fresh telemetry, and we detect clock drift / missed updates)
static void ssePushStatus() {
  if (events.count() == 0) return;        // no clients -> no work
  DynamicJsonDocument doc(2048);
  buildStatusJson(doc);
  String out; serializeJson(doc, out);
  events.send(out.c_str(), "status");
}

static void setupWebServer() {
  // ---- Server config ----------------------------------------------------
  // Worker threads: one SSE connection holds a worker thread for its whole
  // lifetime, so we must have >= 1 + (expected simultaneous SSE clients).
  // 5 covers: 2 SSE clients + 3 concurrent AJAX/static requests. Raise
  // via server.config.max_open_sockets if running more browser tabs.
  server.config.max_uri_handlers = 20;
  server.config.max_open_sockets = 7;
  // PsychicHttp default is 16 KB body / 2 MB upload; raise both so HTTP-OTA
  // POSTs to /update (firmware.bin ~1.6 MB + multipart overhead, littlefs.bin
  // up to 1.2 MB) aren't rejected by the pre-handle size check.
  server.maxRequestBodySize = 4UL * 1024UL * 1024UL;
  server.maxUploadSize      = 4UL * 1024UL * 1024UL;
  server.listen(80);

  // ---- API routes FIRST -------------------------------------------------
  // Registering these before serveStatic prevents the static handler from
  // trying to open /littlefs/www/api/status[.gz] etc. on every request.
  server.on("/api/status",        HTTP_GET,  handleStatus);
  server.on("/api/cmd",           HTTP_POST, handleCmd);
  server.on("/api/cmd",           HTTP_GET,  handleCmd);   // GET for quick toggles
  server.on("/api/config",        HTTP_GET,  handleConfigGet);
  server.on("/api/config",        HTTP_POST, handleConfigPost);
  server.on("/api/reboot",        HTTP_POST, handleReboot);
  server.on("/api/tabs",          HTTP_GET,  handleTabsGet);
  server.on("/api/tabs",          HTTP_POST, handleTabsPost);
  server.on("/api/homekit/reset", HTTP_POST, handleHomekitReset);
  server.on("/api/schedule",      HTTP_POST, handleSchedulePost);
  server.on("/api/wifi/scan",     HTTP_GET,  handleWifiScan);
  server.on("/api/lcd.json",      HTTP_GET,  handleLcdJson);
  server.on("/api/orp/cal",       HTTP_POST, handleOrpCalibrate);
  server.on("/api/1w/scan",       HTTP_POST, handleOneWireScan);

  // ---- Server-Sent Events (realtime telemetry) --------------------------
  // Browser opens `new EventSource("/events")` and receives named events
  // "status" (JSON status blob) whenever state changes + every 2 s as a
  // heartbeat. No more /api/status polling needed.
  server.on("/events", &events);

  // Pool-temp / ORP history (chunked JSON + CSV) ---------------------------
  //   GET /api/history      -> JSON points array
  //   GET /api/history.csv  -> human-readable CSV download
  History::registerRoutes(server);

  // Manual measurements (chlorine, pH) ------------------------------------
  //   GET    /api/manual              -> { chlorine:[], ph:[] }
  //   POST   /api/manual              -> {kind, value, epoch?}
  //   DELETE /api/manual?kind=&epoch= -> remove an entry
  Manual::registerRoutes(server);

  // ---- Static assets (gzip-encoded on LittleFS) -------------------------
  // PsychicStaticFileHandler auto-detects a .gz sibling and adds
  // Content-Encoding: gzip. Gzipping shrinks the bundle ~71 %.
  // max-age=3600 lets the browser cache for an hour.
  PsychicStaticFileHandler *staticHandler =
      server.serveStatic("/", LittleFS, "/www/");
  staticHandler->setDefaultFile("index.html");
  staticHandler->setCacheControl("public, max-age=3600");

  // ---- HTTP OTA (multipart firmware upload) -----------------------------
  // PsychicHttp splits uploads into an UploadHandler (data chunks) + a
  // normal request handler (called after the upload finishes).
  PsychicUploadHandler *ota = new PsychicUploadHandler();
  ota->onUpload([](PsychicRequest *req, const String &filename,
                   uint64_t index, uint8_t *data, size_t len, bool last) -> esp_err_t {
    if (index == 0) {
      Serial.printf("[OTA] Begin: %s\n", filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
        return ESP_FAIL;
      }
    }
    if (!Update.hasError() && Update.write(data, len) != len) {
      Update.printError(Serial);
      return ESP_FAIL;
    }
    if (last) {
      if (Update.end(true)) Serial.printf("[OTA] Success: %u bytes\n",
                                          (unsigned)(index + len));
      else { Update.printError(Serial); return ESP_FAIL; }
    }
    return ESP_OK;
  });
  ota->onRequest([](PsychicRequest *req) -> esp_err_t {
    const bool ok = !Update.hasError();
    const char *body = ok ? "{\"success\":true}" : "{\"success\":false}";
    esp_err_t rc = req->reply(200, "application/json", body);
    if (ok) { g_rebootPending = true; g_rebootAt = millis() + 500; }
    return rc;
  });
  server.on("/update", HTTP_POST, ota);

  // 404 fallback: PsychicHttp calls this for any unregistered path.
  server.onNotFound([](PsychicRequest *req) -> esp_err_t {
    return req->reply(404, "text/plain", "Not Found");
  });

  Serial.println("[HTTP] PsychicHttp server listening on :80");
}

// =========================================================================
// WiFi + OTA
// =========================================================================
static void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  // Performance: disable modem-sleep so the radio stays awake.
  // Default DTIM-based modem sleep adds 100-300 ms to every inbound packet,
  // which makes the web UI feel sluggish (we measured avg 230 ms ping).
  // Also max out TX power (8.5 dBm -> ~19.5 dBm) for reliable link.
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
  Serial.printf("[WiFi] Connecting to %s ", config.wifiSsid.c_str());
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) {
    delay(250); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] OK  IP=%s  RSSI=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (MDNS.begin(DEVICE_HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("[mDNS] http://%s.local/\n", DEVICE_HOSTNAME);
    }
  } else {
    Serial.println("[WiFi] FAILED - continuing offline");
  }
}

static void setupOTA() {
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
  ArduinoOTA
    .onStart([]() { Serial.println("[OTA] start"); })
    .onEnd  ([]() { Serial.println("[OTA] end");   })
    .onError([](ota_error_t e) { Serial.printf("[OTA] err %u\n", e); });
  ArduinoOTA.begin();
}

// =========================================================================
// Setup
// =========================================================================
void setup() {
  // A decent TX ring buffer so Serial.print/printf can never block the
  // caller (AsyncTCP task, main loop, HomeSpan task...) waiting for the
  // 128-byte UART FIFO to drain at 115200 baud (~14 KB/s).
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(200);
  Serial.println();

  // Build the per-device hostname BEFORE WiFi/HomeSpan/MQTT use it.
  // WiFi.macAddress() works without WiFi.begin() - it just reads the eFuse.
  {
    uint8_t mac[6]; WiFi.macAddress(mac);
    snprintf(g_hostnameBuf, sizeof(g_hostnameBuf), "poolcontroller-%02X%02X%02X",
             mac[3], mac[4], mac[5]);
  }

  Serial.printf("Poolcontroller10 %s (%s %s)  host=%s\n",
                FIRMWARE_VERSION, FIRMWARE_BUILD_DATE, FIRMWARE_BUILD_TIME,
                DEVICE_HOSTNAME);

  // Pins
  pinMode(PIN_PUMP,        OUTPUT); digitalWrite(PIN_PUMP,        LOW);
  pinMode(PIN_ELY_A,       OUTPUT); digitalWrite(PIN_ELY_A,       LOW);
  pinMode(PIN_ELY_B,       OUTPUT); digitalWrite(PIN_ELY_B,       LOW);
  pinMode(PIN_ELY_K,       OUTPUT); digitalWrite(PIN_ELY_K,       LOW);
  pinMode(PIN_TRANSFORMER, OUTPUT); digitalWrite(PIN_TRANSFORMER, LOW);
  pinMode(PIN_FLOW,        INPUT_PULLUP);
  pinMode(PIN_BUTTON,      INPUT_PULLUP);
  pinMode(PIN_STATUS_LED,  OUTPUT);

  // LittleFS (web assets)
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS mount failed!");
  } else {
    Serial.println("[FS] LittleFS mounted");
    History::begin();  // restore pool-temp ring buffer from /hist.bin
    Manual::begin();   // restore manual chlorine + pH measurements
  }

  // Config from NVS
  loadConfig();
  pumping = config.pumpStartOn;   // apply configured startup state

  // ORP calibration from stored offset
  ORP.setCalibration(config.orpCalibration);

  // Display (VSPI HW-SPI)
  SPI.begin(PIN_LCD_SCK, -1, PIN_LCD_MOSI, PIN_LCD_CS);
  u8g2.begin();
  // ST7920 has a built-in CJK character ROM. If a single bit flips in the
  // Function-Set command byte, the controller drops out of graphics mode
  // and starts interpreting subsequent pixel bytes as GB2312 character
  // codes -> real chinese glyphs appear on the screen. Confirmed observed
  // symptom on this hardware. Mitigations:
  //   1) Run SPI at 100 kHz - the absolute minimum that keeps frame time
  //      reasonable (~85 ms / frame at 1 Hz refresh, plenty of headroom).
  //      ST7920 has no minimum clock spec (static CMOS), and slowing the
  //      bus down maximises signal-integrity margin against the 3.3 V vs.
  //      5 V V_IH mismatch.
  //   2) Periodic initDisplay() in drawPhysicalDisplay() to recover from
  //      any mode-loss that still slips through (defensive; real fix is
  //      a 3.3->5 V level shifter such as 74HCT125).
  u8g2.setBusClock(100000UL);
  u8g2.setDisplayRotation(U8G2_R3);   // portrait 64x128 (set ONCE)
  u8g2.setFont(u8g2_font_7x13B_tr);   // bold mono, 7px advance, set ONCE
  u8g2.setFontMode(0);                // solid: glyph also clears its bg
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  Quote::load();

  // I2C + ADS1115
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  ads_available = ads.begin();
  if (ads_available) {
    ads.setGain(GAIN_TWOTHIRDS);
    ads.setDataRate(ADS1115_DR_8SPS);
    Serial.println("[ADS1115] OK");
  } else {
    Serial.println("[ADS1115] NOT FOUND");
  }

  // WiFi, OTA, HTTP, MQTT
  setupWiFi();
  setupOTA();
  setupWebServer();
  mqttClient.setServer(config.mqttHost.c_str(), config.mqttPort);
  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(onMqttMessage);
  // PubSubClient's default socket timeout is 15 s. If the broker drops
  // under load, connect()/loop() block the main loop for that long,
  // stalling the AsyncWebServer. 2 s keeps the worst-case hiccup short.
  mqttClient.setSocketTimeout(2);

  // HomeKit (HAP R2 via HomeSpan). Must be called after WiFi is up because
  // HomeSpan needs an IP to bind its HAP TCP listener + advertise via mDNS.
  if (WiFi.status() == WL_CONNECTED) {
    homekitSetup();
  } else {
    Serial.println("[HK] WiFi not connected; HomeKit disabled this boot.");
  }

  // NTP (best effort, non-blocking). Use CET/CEST so the weekly schedule
  // works with local wall-clock hours. Use fixed IP (no DNS) for field WiFi.
  // 195.154.0.200 = pool.ntp.org
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "195.154.0.200", "216.239.35.0", "192.168.178.83");

  // DS18B20 (init last, after WiFi/NTP/HTTP are up, so boot doesn't block
  // if sensor is broken/disconnected). Reconnect checked every 10s in loop().
  Tempsensors.begin();
  Tempsensors.setWaitForConversion(false);
  scanOneWireBus(true);
  if (ds18b20_found) {
    Tempsensors.setResolution(tempAddrPool, TEMPERATURE_PRECISION);
    if (ds18b20_air_found) Tempsensors.setResolution(tempAddrAir, TEMPERATURE_PRECISION);
    tempConvDelayMs = Tempsensors.millisToWaitForConversion(TEMPERATURE_PRECISION);
  }
}

// =========================================================================
// Main loop
// =========================================================================
void loop() {
  ArduinoOTA.handle();
  homekitLoop();  // HomeSpan event-queue service (cheap; no-op if disabled)
  const unsigned long now = millis();

  // --- Reboot scheduled? -------------------------------------------------
  if (g_rebootPending && (long)(now - g_rebootAt) >= 0) {
    Serial.println("[SYS] rebooting...");
    delay(50);
    ESP.restart();
  }

  // --- MQTT keepalive + reconnect ---------------------------------------
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) mqttReconnect();
    mqttClient.loop();
  }

  // --- NTP sync check (polls local-time availability every 2 s) ---------
  if (now - g_lastNtpCheckMs >= NTP_CHECK_INTERVAL_MS) {
    g_lastNtpCheckMs = now;
    if (WiFi.status() == WL_CONNECTED) ntpRefresh();
  }

  // --- DS18B20 reconnect check (every 10 s if not found) -----------------
  static unsigned long lastOneWireCheck = 0;
  if (!ds18b20_found && (now - lastOneWireCheck >= 10000)) {
    lastOneWireCheck = now;
    scanOneWireBus(false);  // silent scan, only log if found
    if (ds18b20_found) {
      Tempsensors.setResolution(tempAddrPool, TEMPERATURE_PRECISION);
      if (ds18b20_air_found) Tempsensors.setResolution(tempAddrAir, TEMPERATURE_PRECISION);
      tempConvDelayMs = Tempsensors.millisToWaitForConversion(TEMPERATURE_PRECISION);
      Serial.println("[1W] reconnected!");
    }
  }

  // --- ADS1115 / ORP -----------------------------------------------------
  if (ads_available && (now - lastSensorReadTime >= SENSOR_READ_INTERVAL)) {
    lastSensorReadTime = now;
    const int16_t adc3 = ads.readADC_SingleEnded(3);
    const float   mv   = adc3 * 0.1875f;
    voltage  = mv * 0.001f;
    orpValue = (int)ORP.getORP(mv);
  }

  // --- DS18B20 async state machine (pool + optional air) ----------------
  // Failsafe: if sensor fails, synthetic data keeps the device running.
  // Only reconnect every 10s (not every loop iteration).
  if (ds18b20_found) {
    if (!tempConvPending && (now - lastTempReadTime >= TEMP_REFRESH_INTERVAL)) {
      Tempsensors.requestTemperatures();
      tempConvStarted = now;
      tempConvPending = true;
    }
    if (tempConvPending && (now - tempConvStarted >= tempConvDelayMs)) {
      const float tp = Tempsensors.getTempC(tempAddrPool);
      tempConvPending  = false;
      lastTempReadTime = now;
      if (tp == DEVICE_DISCONNECTED_C || tp < -50.0f || tp > 125.0f) {
        if (++tempFailStreak >= TEMP_FAIL_LIMIT) {
          ds18b20_found  = false;
          tempFailStreak = 0;
          Serial.println("[1W] sensor failed, using synthetic data");
        }
      } else {
        tempFailStreak = 0;
        temp_pool      = tp;
      }
      if (ds18b20_air_found) {
        const float ta = Tempsensors.getTempC(tempAddrAir);
        if (ta == DEVICE_DISCONNECTED_C || ta < -50.0f || ta > 125.0f) {
          if (++tempAirFailStreak >= TEMP_FAIL_LIMIT) {
            ds18b20_air_found  = false;
            tempAirFailStreak  = 0;
            Serial.println("[1W] air sensor failed, using synthetic data");
          }
        } else {
          tempAirFailStreak = 0;
          temp_air = ta;
        }
      }
    }
  }
  // Reconnect check: only every 10s if sensor not found (not every loop)
  static unsigned long lastTempReconnect = 0;
  if (!ds18b20_found && (now - lastTempReconnect >= 10000)) {
    lastTempReconnect = now;
    if (scanOneWireBus(false)) {
      Tempsensors.setResolution(tempAddrPool, TEMPERATURE_PRECISION);
      if (ds18b20_air_found) Tempsensors.setResolution(tempAddrAir, TEMPERATURE_PRECISION);
      tempConvDelayMs = Tempsensors.millisToWaitForConversion(TEMPERATURE_PRECISION);
    }
  }

  // --- Button: toggle Ely A/B phase (0->1->2->0) with debounce -----------
  {
    static bool buttonRawLast = true;  // HIGH = not pressed
    static unsigned long buttonStableAt = 0;
    const bool raw = (digitalRead(PIN_BUTTON) == LOW);
    if (raw != buttonRawLast) {
      buttonRawLast  = raw;
      buttonStableAt = now;
    } else if (raw && (now - buttonStableAt) >= 50) {  // 50ms debounce, detect release
      // Button released: toggle phase
      elyABPhase = (elyABPhase == 0) ? 1 : (elyABPhase == 1) ? 2 : 0;
      elyABLastSwitchMs = now;  // reset alternator timer
      buttonStableAt = now + 500;  // debounce: ignore for 500ms
    }
  }

  // --- Flow sensor with debounce ----------------------------------------
  {
    const bool raw = (digitalRead(PIN_FLOW) == LOW);
    if (raw != flowRawLast) {
      flowRawLast  = raw;
      flowStableAt = now;
    } else if (raw != flowing && (now - flowStableAt) >= FLOW_DEBOUNCE_MS) {
      flowing = raw;
    }
  }

  // --- Ely A/B automatic mode --------------------------------------------
  // When auto is enabled, the schedule drives the Ely cell on/off:
  //   schedule_active == true  && phase == 0  -> start at phase A
  //   schedule_active == false && phase != 0  -> stop (handled below)
  // When auto is disabled, only manual commands / button start the cell
  // (but the schedule still forces OFF for safety when inactive).
  {
    const bool schedOn = isScheduleActive();
    if (config.elyABAuto && schedOn && elyABPhase == 0) {
      elyABPhase        = 1;
      elyABLastSwitchMs = now;
    }
  }

  // --- Weekly schedule: force Ely A/B OFF when current hour is inactive (only in auto mode) -
  // Manual commands are allowed even when schedule is inactive (auto mode disabled)
  if (config.elyABAuto && elyABPhase != 0 && !isScheduleActive()) {
    elyABPhase = 0;
  }

  // --- Ely A/B alternator: swap every config.elyABSwitchSecs ------------
  if (elyABPhase != 0) {
    const unsigned long intervalMs =
        (unsigned long)config.elyABSwitchSecs * 1000UL;
    if (now - elyABLastSwitchMs >= intervalMs) {
      elyABPhase = (elyABPhase == 1) ? 2 : 1;
      elyABLastSwitchMs = now;
    }
  }
  // Derive the mutually-exclusive output commands from the phase.
  elyAcommand = (elyABPhase == 1);
  elyBcommand = (elyABPhase == 2);

  // --- Flow-error / transformer control ---------------------------------
  if (!flowing && !flowerror) {
    flowerror = true;
    starttime_flowerror = now;
  }
  if (flowing) flowerror = false;

  const bool anyEly = (elyAcommand || elyBcommand || elyKcommand);
  if (flowerror && (now - starttime_flowerror > MAXFLOWERROR_DURATION)) {
    transformercommand = false;
  } else if (anyEly && !flowerror) {
    transformercommand = true;
  } else if (!anyEly) {
    transformercommand = false;
  }

  // --- Failsafe: WiFi loss tracking & mode switch --------------------------
  const bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected) {
    g_wifiLostAt = 0;
    g_failsafeMode = false;
  } else {
    if (g_wifiLostAt == 0) {
      g_wifiLostAt = now;
    } else if (now - g_wifiLostAt >= 10000) {
      g_failsafeMode = true;
    }
  }

  // --- Drive outputs -----------------------------------------------------
  digitalWrite(PIN_PUMP,        pumping            ? HIGH : LOW);
  digitalWrite(PIN_TRANSFORMER, transformercommand ? HIGH : LOW);
  digitalWrite(PIN_ELY_A,       elyAcommand        ? HIGH : LOW);
  digitalWrite(PIN_ELY_B,       elyBcommand        ? HIGH : LOW);
  digitalWrite(PIN_ELY_K,       elyKcommand        ? HIGH : LOW);
  // LED: only on when both WiFi AND MQTT are connected
  const bool ledOn = wifiConnected && mqttClient.connected();
  digitalWrite(PIN_STATUS_LED,  ledOn ? HIGH : LOW);

  // --- History sample + persist -----------------------------------------
  // tickSample() is a no-op until NTP is synced and 15 min have elapsed
  // since the last sample, so calling it every loop is cheap.
  //
  // When a sensor is missing we substitute a plausible value so the
  // timeline / chart stays populated on demo benches with no probes
  // attached. The synthetic temperature follows a simple diurnal curve
  // (peak ~16:00 local); synthetic ORP is a slow sinusoid around 700 mV.
  {
    const bool tempValid = ds18b20_found && temp_pool > -50.0f && temp_pool < 125.0f;
    float tVal;
    if (tempValid) {
      tVal = temp_pool;
    } else {
      struct tm ti;
      float hourFrac = 12.0f;
      if (getLocalTime(&ti, 0)) hourFrac = ti.tm_hour + ti.tm_min / 60.0f;
      // 24 C baseline +/- 2 C diurnal, peak at 16:00.
      const float ang = 2.0f * PI * (hourFrac - 10.0f) / 24.0f;
      tVal = 24.0f + 2.0f * sinf(ang);
    }
    int16_t orp;
    if (ads_available) {
      orp = (int16_t)orpValue;
    } else {
      // Slow wobble (~4 h period) between 680 and 720 mV.
      const uint32_t t = (uint32_t)(millis() / 1000);
      orp = 700 + (int16_t)(20.0f * sinf(2.0f * PI * (t % 14400) / 14400.0f));
    }
    History::tickSample(tVal, orp);
  }
  History::tickSave();  // rate-limited: persists at most every 5 min
  Quote::tick();        // rate-limited: fetches at most once per UTC day

  // --- Display + status publish -----------------------------------------
  if (now - lastRefreshTime >= DISPLAY_REFRESH_INTERVAL) {
    lastRefreshTime = now;
    drawPhysicalDisplay();  // Physisches Display direkt wie pc07
    drawVirtualDisplay();   // Virtuelles Display mit wrapper
  }
  if (now - lastStatusPublish >= (unsigned long)config.mqttIntervalSecs * 1000UL) {
    lastStatusPublish = now;
    g_mqttPublishPending = true;
  }

  // --- SSE heartbeat: push status to browser clients every 2 s so the
  // UI stays fresh even without state changes. When a handler flipped
  // g_ssePushPending (cmd / config / schedule), we push immediately.
  const unsigned long SSE_HEARTBEAT_MS = 2000;
  if (g_ssePushPending || (now - lastSsePush >= SSE_HEARTBEAT_MS)) {
    g_ssePushPending = false;
    lastSsePush = now;
    ssePushStatus();
  }

  // Drain deferred MQTT work requested by HTTP handlers.
  if (g_mqttReconfigPending) {
    g_mqttReconfigPending = false;
    if (mqttClient.connected()) mqttClient.disconnect();
    mqttClient.setServer(config.mqttHost.c_str(), config.mqttPort);
  }
  if (g_mqttPublishPending) {
    g_mqttPublishPending = false;
    publishStatus();
  }
}
