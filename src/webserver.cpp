// HTTP + Server-Sent Events for the Heizstabsteuerung (ESP32 / PsychicHttp).
//
// Static assets served from LittleFS at /www/. Dynamic API:
//   GET  /api/status    -> last broadcast JSON status (all values + parameters)
//   GET  /api/params    -> current parameters only (JSON)
//   POST /api/cmd       -> body: cmd=pump_on|pump_off|regulate_on|regulate_off
//   POST /api/config    -> form-encoded settings (persisted to NVS)
//   POST /api/reboot    -> reboot in ~500 ms (saves history first)
//   GET  /api/history       -> JSON ring buffer (registered in History::)
//   GET  /api/history.csv   -> CSV export
//   GET  /events        -> SSE stream, named event "status" (real-time updates)

#include "webserver.h"
#include "config_store.h"
#include "history.h"
#include "energy.h"
#include "temp_sensors.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <PsychicHttp.h>

// State owned by main.cpp
extern volatile int powerdrawnumber;
extern volatile int powerdrawsetpoint;
extern int powerToConsume;
extern int ZERO_FEED_IN_TARGET;
extern int MAX_HEATING_POWER;
extern int MIN_POWER_THRESHOLD;
extern int DEADBAND;
extern int POWER_CHANGE_THRESHOLD;
extern int MAX_BOILER_TEMP_C;
extern int MAX_HEATER_ROD_TEMP_C;
extern float correctionGain;
extern volatile bool pumpmanualpower;
extern bool pumpautocontrolled;
extern volatile bool regulating_power;
extern bool heating;
extern int DACoutput;
extern int wattneeded;
extern unsigned long PUMP_MIN_RUNTIME_MS;
extern unsigned long PUMP_CYCLE_INTERVAL_MIN;
extern unsigned long PUMP_CYCLE_DURATION_SEC;
extern bool  PUMP_TEMP_COND_ENABLED;
extern float PUMP_TEMP_HYST_C;
extern unsigned long lastPowerDrawUpdate;
extern bool  VOL_ENABLED;
extern int   VOL_WINDOW_MIN;
extern int   VOL_THRESHOLD_W;
extern int   ONEWIRE_PIN;
extern bool  HISTORY_AVERAGING;

// Network config globals (owned by main.cpp)
extern String NET_MODE;
extern bool   LAN_DHCP;
extern String LAN_IP;
extern String LAN_GW;
extern String LAN_MASK;
extern String LAN_DNS;

// MQTT status publish config (owned by main.cpp)
extern bool MQTT_STATUS_ENABLED;
extern unsigned long MQTT_STATUS_INTERVAL_MS;

// Drain-compressor relay globals (owned by main.cpp)
extern volatile bool drainTriggerReq;
extern volatile bool drainCancelReq;
extern bool                   drainActive;
extern volatile unsigned long drainPulseMs;

extern void resetRebootCounter();

void sendupdate(bool force);

// ---- Module state ------------------------------------------------------
static PsychicHttpServer  server;
static PsychicEventSource events;
static constexpr int MAX_SSE_CLIENTS = 4;  // hard cap: each idle client still owns a LWIP socket
static String s_statusPayload;
static SemaphoreHandle_t s_statusMutex = nullptr;
static bool   rebootPending = false;
static unsigned long rebootAt = 0;

// Set by HTTP handlers when state changes; drained by loop() -> sendupdate().
// This ensures events.send() is only ever called from the loop task,
// matching the PoolController pattern that runs stable without mallocs.
volatile bool webserver_ssePushPending = false;

bool webserver_pauseSSE = false;  // set true during OTA to reduce WiFi load

// Set by webLog() whenever a new log line is stored; drained by webserver_loop()
// so events.send() is only ever called from the loop task.
volatile bool webserver_logPushPending = false;

// Pending configuration changes set by handleConfig (httpd task) and applied
// in main.cpp loop() to keep all global state mutations on the loop task.
static SemaphoreHandle_t s_configMutex = nullptr;
volatile bool webserver_configPending = false;
PendingConfig webserver_pendingConfig;

void webserver_getAndClearPendingConfig(PendingConfig &out) {
  out = PendingConfig();  // default-empty if take fails
  if (!s_configMutex) s_configMutex = xSemaphoreCreateMutex();
  if (s_configMutex && xSemaphoreTake(s_configMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    out = webserver_pendingConfig;
    webserver_pendingConfig = PendingConfig();
    xSemaphoreGive(s_configMutex);
  }
}

// ---- SSE ------------------------------------------------------------------
// No custom session tracking/TTL/forced-close churn. Matches Poolcontroller10,
// which relies entirely on PsychicHttp/esp_http_server's native connection
// handling plus a generous max_open_sockets. A previous custom TTL-based
// forced-close system was removed: its own comments warned it could itself
// slowly exhaust the socket pool if LWIP was slow to reclaim closed sockets.

// Count active SSE clients for diagnostics (thin wrapper over PsychicHttp's
// own client list, declared after `events` further below).
int webserver_getSseClientCount();

static inline String snapshotStatus() {
  if (!s_statusMutex) return s_statusPayload;
  String copy;
  if (xSemaphoreTake(s_statusMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    // Reserve before copy so String::operator= never triggers a reallocation.
    copy.reserve(s_statusPayload.length());
    copy = s_statusPayload;
    xSemaphoreGive(s_statusMutex);
  }
  return copy;
}

// ---- Web log ring buffer (for browser console) --------------------------
static constexpr uint8_t  LOG_MAX_LINES = 20;
static constexpr uint16_t LOG_LINE_LEN  = 128;
static char   s_logBuf[LOG_MAX_LINES][LOG_LINE_LEN];
static uint8_t s_logHead = 0;
static uint8_t s_logCount = 0;
static SemaphoreHandle_t s_logMutex = nullptr;

void webLog(const char* fmt, ...) {
  char line[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  Serial.println(line);

  // The ring buffer is shared between arbitrary caller contexts (httpd task,
  // network event handlers, MQTT callback, loop task) and the loop task that
  // drains it. Protect writes/reads with a short-timeout mutex.
  if (!s_logMutex) s_logMutex = xSemaphoreCreateMutex();
  if (s_logMutex && xSemaphoreTake(s_logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    strncpy(s_logBuf[s_logHead], line, LOG_LINE_LEN - 1);
    s_logBuf[s_logHead][LOG_LINE_LEN - 1] = '\0';
    s_logHead = (s_logHead + 1) % LOG_MAX_LINES;
    if (s_logCount < LOG_MAX_LINES) s_logCount++;
    xSemaphoreGive(s_logMutex);
  }

  // Defer SSE log send to the loop task. webLog() is called from many contexts
  // including WiFi/Ethernet event handlers and the MQTT callback, where
  // events.send() is not safe. webserver_loop() drains this flag.
  webserver_logPushPending = true;
}

static String paramOr(PsychicRequest *req, const char *name, const String &def) {
  if (req->hasParam(name)) return req->getParam(name)->value();
  return def;
}

// ---- Route handlers ----------------------------------------------------
static esp_err_t handleStatus(PsychicRequest *req) {
  String snap = snapshotStatus();
  if (snap.length() == 0) {
    return req->reply(200, "application/json", "{}");
  }
  return req->reply(200, "application/json", snap.c_str());
}

static esp_err_t handleCmd(PsychicRequest *req) {
  String cmd = paramOr(req, "cmd", "");
  bool changed = false;
  bool persist = false;
  if      (cmd == "pump_on")      { pumpmanualpower = true;  changed = true; }
  else if (cmd == "pump_off")     { pumpmanualpower = false; changed = true; }
  else if (cmd == "regulate_on")  { regulating_power = true;  changed = true; persist = true; }
  else if (cmd == "regulate_off") { regulating_power = false; changed = true; persist = true; }
  else if (cmd == "drain_on") {
    String durStr = paramOr(req, "dur", "3");
    int durSec = durStr.toInt();
    if (durSec < 1 || durSec > 60) durSec = 3;  // clamp to 1-60 seconds
    drainPulseMs = durSec * 1000;
    drainTriggerReq = true;
    changed = true;
  }
  else if (cmd == "drain_off")    { drainCancelReq  = true;  changed = true; }
  else { return req->reply(400, "text/plain", "unknown cmd"); }

  if (persist) ConfigStore::save();
  if (changed) webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "ok");
}

static esp_err_t handleConfig(PsychicRequest *req) {
  JsonDocument doc;

  // Start fresh: each config request carries the values to change. Fill a local
  // struct first; the global pending struct is swapped atomically at the end.
  PendingConfig local;

  // Try to parse JSON body if Content-Type is application/json
  if (req->contentType() == "application/json" && req->contentLength() > 0) {
    // Get the body as a string
    String body = req->body();
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      return req->reply(400, "text/plain", "Invalid JSON");
    }
  } else {
    // Fall back to form parameters
    if (req->hasParam("zero_feed_target")) {
      doc["zero_feed_target"] = req->getParam("zero_feed_target")->value().toInt();
    }
    if (req->hasParam("max_heating_power")) {
      doc["max_heating_power"] = req->getParam("max_heating_power")->value().toInt();
    }
    if (req->hasParam("min_power_threshold")) {
      doc["min_power_threshold"] = req->getParam("min_power_threshold")->value().toInt();
    }
    if (req->hasParam("deadband")) {
      doc["deadband"] = req->getParam("deadband")->value().toInt();
    }
    if (req->hasParam("power_change_threshold")) {
      doc["power_change_threshold"] = req->getParam("power_change_threshold")->value().toInt();
    }
    if (req->hasParam("correction_gain")) {
      doc["correction_gain"] = req->getParam("correction_gain")->value().toInt();
    }
    if (req->hasParam("pump_min_runtime")) {
      doc["pump_min_runtime"] = req->getParam("pump_min_runtime")->value().toInt();
    }
    if (req->hasParam("pump_cycle_interval")) {
      doc["pump_cycle_interval"] = req->getParam("pump_cycle_interval")->value().toInt();
    }
    if (req->hasParam("pump_cycle_duration")) {
      doc["pump_cycle_duration"] = req->getParam("pump_cycle_duration")->value().toInt();
    }
    if (req->hasParam("pump_temp_cond")) {
      doc["pump_temp_cond"] = req->getParam("pump_temp_cond")->value();
    }
    if (req->hasParam("pump_temp_hyst")) {
      doc["pump_temp_hyst"] = req->getParam("pump_temp_hyst")->value().toFloat();
    }
    if (req->hasParam("vol_enabled")) {
      doc["vol_enabled"] = req->getParam("vol_enabled")->value();
    }
    if (req->hasParam("history_averaging")) {
      doc["history_averaging"] = req->getParam("history_averaging")->value();
    }
    if (req->hasParam("vol_window_min")) {
      doc["vol_window_min"] = req->getParam("vol_window_min")->value().toInt();
    }
    if (req->hasParam("vol_threshold_w")) {
      doc["vol_threshold_w"] = req->getParam("vol_threshold_w")->value().toInt();
    }
    if (req->hasParam("onewire_pin")) {
      doc["onewire_pin"] = req->getParam("onewire_pin")->value().toInt();
    }
    if (req->hasParam("max_boiler_temp")) {
      doc["max_boiler_temp"] = req->getParam("max_boiler_temp")->value().toInt();
    }
    if (req->hasParam("max_heater_rod_temp")) {
      doc["max_heater_rod_temp"] = req->getParam("max_heater_rod_temp")->value().toInt();
    }
    if (req->hasParam("net_mode")) {
      doc["net_mode"] = req->getParam("net_mode")->value();
    }
    if (req->hasParam("lan_dhcp")) {
      doc["lan_dhcp"] = req->getParam("lan_dhcp")->value();
    }
    if (req->hasParam("lan_ip")) {
      doc["lan_ip"] = req->getParam("lan_ip")->value();
    }
    if (req->hasParam("lan_gw")) {
      doc["lan_gw"] = req->getParam("lan_gw")->value();
    }
    if (req->hasParam("lan_mask")) {
      doc["lan_mask"] = req->getParam("lan_mask")->value();
    }
    if (req->hasParam("lan_dns")) {
      doc["lan_dns"] = req->getParam("lan_dns")->value();
    }
    if (req->hasParam("mqtt_status_enabled")) {
      doc["mqtt_status_enabled"] = req->getParam("mqtt_status_enabled")->value();
    }
    if (req->hasParam("mqtt_status_interval")) {
      doc["mqtt_status_interval"] = req->getParam("mqtt_status_interval")->value().toInt();
    }
  }
  
  // Queue all changes for application on the loop task instead of mutating
  // shared globals directly from the httpd task.
  if (!doc["zero_feed_target"].isNull()) {
    int n = doc["zero_feed_target"];
    if (n >= -1000 && n <= 1000) {
      local.hasZeroFeedTarget = true;
      local.zeroFeedTarget = n;
    }
  }
  if (!doc["max_heating_power"].isNull()) {
    int n = doc["max_heating_power"];
    if (n >= 1 && n <= 2000) {
      local.hasMaxHeatingPower = true;
      local.maxHeatingPower = n;
    }
  }
  if (!doc["min_power_threshold"].isNull()) {
    int n = doc["min_power_threshold"];
    if (n >= 0 && n <= 99) {
      local.hasMinPowerThreshold = true;
      local.minPowerThreshold = n;
    }
  }
  if (!doc["deadband"].isNull()) {
    int n = doc["deadband"];
    if (n >= 0 && n <= 499) {
      local.hasDeadband = true;
      local.deadband = n;
    }
  }
  if (!doc["power_change_threshold"].isNull()) {
    int n = doc["power_change_threshold"];
    if (n >= 1 && n <= 100) {
      local.hasPowerChangeThreshold = true;
      local.powerChangeThreshold = n;
    }
  }
  if (!doc["correction_gain"].isNull()) {
    int pct = doc["correction_gain"];
    if (pct >= 1 && pct <= 150) {
      local.hasCorrectionGain = true;
      local.correctionGainPct = pct;
    }
  }
  if (!doc["pump_min_runtime"].isNull()) {
    long sec = doc["pump_min_runtime"];
    if (sec >= 5 && sec <= 300) {
      local.hasPumpMinRuntime = true;
      local.pumpMinRuntimeSec = (unsigned long)sec;
    }
  }
  if (!doc["pump_cycle_interval"].isNull()) {
    long min = doc["pump_cycle_interval"];
    if (min >= 0 && min <= 1440) {
      local.hasPumpCycleInterval = true;
      local.pumpCycleIntervalMin = (unsigned long)min;
    }
  }
  if (!doc["pump_cycle_duration"].isNull()) {
    long sec = doc["pump_cycle_duration"];
    if (sec >= 0 && sec <= 3600) {
      local.hasPumpCycleDuration = true;
      local.pumpCycleDurationSec = (unsigned long)sec;
    }
  }
  if (!doc["pump_temp_cond"].isNull()) {
    String v = doc["pump_temp_cond"];
    local.hasPumpTempCond = true;
    local.pumpTempCond = (v == "1" || v == "true" || v == "on");
  }
  if (!doc["pump_temp_hyst"].isNull()) {
    float v = doc["pump_temp_hyst"];
    if (v >= 0.0f && v <= 30.0f) {
      local.hasPumpTempHyst = true;
      local.pumpTempHyst = v;
    }
  }
  if (!doc["vol_enabled"].isNull()) {
    String v = doc["vol_enabled"];
    local.hasVolEnabled = true;
    local.volEnabled = (v == "1" || v == "true" || v == "on");
  }
  if (!doc["history_averaging"].isNull()) {
    String v = doc["history_averaging"];
    local.hasHistoryAveraging = true;
    local.historyAveraging = (v == "1" || v == "true" || v == "on");
  }
  if (!doc["vol_window_min"].isNull()) {
    int n = doc["vol_window_min"];
    if (n >= 1 && n <= 15) {
      local.hasVolWindowMin = true;
      local.volWindowMin = n;
    }
  }
  if (!doc["vol_threshold_w"].isNull()) {
    int n = doc["vol_threshold_w"];
    if (n >= 0 && n <= 5000) {
      local.hasVolThresholdW = true;
      local.volThresholdW = n;
    }
  }
  if (!doc["onewire_pin"].isNull()) {
    int n = doc["onewire_pin"];
    // Exclude input-only (34-39) and flash SPI (6-11) pins
    bool usable = (n >= 0 && n <= 39) && !(n >= 6 && n <= 11) && !(n >= 34 && n <= 39);
    if (usable) {
      local.hasOnewirePin = true;
      local.onewirePin = n;
    }
  }
  if (!doc["max_boiler_temp"].isNull()) {
    int n = doc["max_boiler_temp"];
    if (n >= 0 && n <= 100) {
      local.hasMaxBoilerTemp = true;
      local.maxBoilerTemp = n;
    }
  }
  if (!doc["max_heater_rod_temp"].isNull()) {
    int n = doc["max_heater_rod_temp"];
    if (n >= 0 && n <= 100) {
      local.hasMaxHeaterRodTemp = true;
      local.maxHeaterRodTemp = n;
    }
  }
  if (!doc["net_mode"].isNull()) {
    String v = doc["net_mode"];
    if (v == "wifi" || v == "lan") {
      local.hasNetMode = true;
      local.netMode = v;
    }
  }
  if (!doc["lan_dhcp"].isNull()) {
    String v = doc["lan_dhcp"];
    local.hasLanDhcp = true;
    local.lanDhcp = (v == "1" || v == "true" || v == "on");
  }
  if (!doc["lan_ip"].isNull()) {
    String v = doc["lan_ip"];
    local.hasLanIp = true;
    local.lanIp = v;
  }
  if (!doc["lan_gw"].isNull()) {
    String v = doc["lan_gw"];
    local.hasLanGw = true;
    local.lanGw = v;
  }
  if (!doc["lan_mask"].isNull()) {
    String v = doc["lan_mask"];
    local.hasLanMask = true;
    local.lanMask = v;
  }
  if (!doc["lan_dns"].isNull()) {
    String v = doc["lan_dns"];
    local.hasLanDns = true;
    local.lanDns = v;
  }
  if (!doc["mqtt_status_enabled"].isNull()) {
    String v = doc["mqtt_status_enabled"];
    local.hasMqttStatusEnabled = true;
    local.mqttStatusEnabled = (v == "1" || v == "true" || v == "on");
  }
  if (!doc["mqtt_status_interval"].isNull()) {
    int n = doc["mqtt_status_interval"];
    if (n >= 0 && n <= 3600) {
      local.hasMqttStatusInterval = true;
      local.mqttStatusIntervalSec = n;
    }
  }

  // Detect network-interface changes by comparing pending values against the
  // current globals. NetManager only reads these once during setup(), so a
  // reboot is required for them to take effect.
  bool netChanged = false;
  if (local.hasNetMode) netChanged |= (local.netMode != NET_MODE);
  if (local.hasLanIp)   netChanged |= (local.lanIp != LAN_IP);
  if (local.hasLanGw)   netChanged |= (local.lanGw != LAN_GW);
  if (local.hasLanMask) netChanged |= (local.lanMask != LAN_MASK);
  if (local.hasLanDns)  netChanged |= (local.lanDns != LAN_DNS);
  if (local.hasLanDhcp) netChanged |= (local.lanDhcp != LAN_DHCP);

  if (netChanged) {
    webLog("[Net] config changed - manual reboot required to apply new interface settings");
    History::saveNow();
  }

  // Atomically publish the parsed local config to the loop task. The mutex
  // keeps this race-free against applyPendingConfig() in main.cpp.
  if (!s_configMutex) s_configMutex = xSemaphoreCreateMutex();
  if (s_configMutex && xSemaphoreTake(s_configMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    webserver_pendingConfig = local;
    webserver_configPending = true;
    xSemaphoreGive(s_configMutex);
  }
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", netChanged ? "ok - manual reboot required to apply network change" : "ok");
}

static esp_err_t handleParams(PsychicRequest *req) {
  JsonDocument doc;
  doc["zeroFeedTarget"] = ZERO_FEED_IN_TARGET;
  doc["maxHeatingPower"] = MAX_HEATING_POWER;
  doc["minPowerThreshold"] = MIN_POWER_THRESHOLD;
  doc["deadband"] = DEADBAND;
  doc["powerChangeThreshold"] = POWER_CHANGE_THRESHOLD;
  doc["correctionGain"] = correctionGain;
  doc["pumpMinRuntime"] = (int)(PUMP_MIN_RUNTIME_MS / 1000);
  doc["pumpCycleInterval"] = (int)PUMP_CYCLE_INTERVAL_MIN;
  doc["pumpCycleDuration"] = (int)PUMP_CYCLE_DURATION_SEC;
  doc["regulatingPower"] = regulating_power;
  doc["maxBoilerTemp"] = MAX_BOILER_TEMP_C;
  doc["maxHeaterRodTemp"] = MAX_HEATER_ROD_TEMP_C;
  doc["onewirePin"] = ONEWIRE_PIN;
  doc["netMode"] = NET_MODE;
  doc["lanDhcp"] = LAN_DHCP;
  doc["lanIp"] = LAN_IP;
  doc["lanGw"] = LAN_GW;
  doc["lanMask"] = LAN_MASK;
  doc["lanDns"] = LAN_DNS;
  doc["mqttStatusEnabled"] = MQTT_STATUS_ENABLED;
  doc["mqttStatusInterval"] = (int)(MQTT_STATUS_INTERVAL_MS / 1000);

  String jsonString;
  serializeJson(doc, jsonString);
  return req->reply(200, "application/json", jsonString.c_str());
}

static esp_err_t handleReboot(PsychicRequest *req) {
  History::saveNow();       // flush ring buffer before going down
  rebootPending = true;
  rebootAt = millis() + 500;
  return req->reply(200, "text/plain", "rebooting");
}

// ---- Temperature sensor routes ----------------------------------------
// GET /api/temp/scan -> manuelles Rescan + Liste aller gefundenen DS18B20 mit Live-Wert
static esp_err_t handleTempScan(PsychicRequest *req) {
  // Defer the actual (blocking, array-mutating) rescan to the loop task to
  // avoid racing with tick(). Wait briefly for it to complete so the UI gets
  // fresh values, but never block the httpd task indefinitely.
  TempSensors::requestRescan();
  for (int i = 0; i < 30; i++) {           // up to ~1.5 s
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!TempSensors::rescanPending()) break;
  }
  auto list = TempSensors::scanList();
  JsonDocument doc;
  JsonArray sensors = doc["sensors"].to<JsonArray>();
  for (const auto &f : list) {
    JsonObject obj = sensors.add<JsonObject>();
    obj["rom"] = f.romHex;
    if (f.current_c > -50.0f && f.current_c < 150.0f) obj["current_c"] = round(f.current_c * 10.0f) / 10.0f;
    else obj["current_c"] = nullptr;
  }
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

// GET /api/temp/config -> aktuelle Rolle->ROM-Zuweisung
static esp_err_t handleTempConfigGet(PsychicRequest *req) {
  JsonDocument doc;
  doc["boiler_rom"]  = TempSensors::boilerRom()    ? TempSensors::romToHex(TempSensors::boilerRom())    : "";
  doc["inlet_rom"]   = TempSensors::inletRom()     ? TempSensors::romToHex(TempSensors::inletRom())     : "";
  doc["outlet_rom"]  = TempSensors::outletRom()    ? TempSensors::romToHex(TempSensors::outletRom())    : "";
  doc["hrod_rom"]    = TempSensors::heaterRodRom() ? TempSensors::romToHex(TempSensors::heaterRodRom()) : "";
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

// POST /api/temp/assign -> body: boiler_rom=...&inlet_rom=...
// Leerer String -> Rolle löschen.
static esp_err_t handleTempAssign(PsychicRequest *req) {
  bool any = false;
  if (req->hasParam("boiler_rom")) {
    String v = req->getParam("boiler_rom")->value();
    TempSensors::assignBoiler(v.length() ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (req->hasParam("inlet_rom")) {
    String v = req->getParam("inlet_rom")->value();
    TempSensors::assignInlet(v.length() ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (req->hasParam("outlet_rom")) {
    String v = req->getParam("outlet_rom")->value();
    TempSensors::assignOutlet(v.length() ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (req->hasParam("hrod_rom")) {
    String v = req->getParam("hrod_rom")->value();
    TempSensors::assignHeaterRod(v.length() ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (!any) return req->reply(400, "text/plain", "no fields");
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "ok");
}

// POST /api/energy/reset -> alle Energie-Zähler löschen
static esp_err_t handleEnergyReset(PsychicRequest *req) {
  Energy::resetAll();
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "reset");
}

// POST /api/reboots/reset -> persistenten Reboot-Zähler auf 0 zurücksetzen
static esp_err_t handleRebootsReset(PsychicRequest *req) {
  resetRebootCounter();
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "reset");
}

// ---- Public API --------------------------------------------------------
void webserver_begin() {
  // Idempotent: safe to call repeatedly from loop() until network is up.
  static bool s_begun = false;
  if (s_begun) return;
  s_begun = true;

  if (!s_statusMutex) s_statusMutex = xSemaphoreCreateMutex();
  server.config.max_uri_handlers = 20;
  server.config.max_open_sockets   = 7;
  server.listen(80);

  // API routes registered FIRST so the static-file fallback doesn't
  // accidentally try to look up /littlefs/www/api/status etc.
  server.on("/api/status", HTTP_GET,  handleStatus);
  server.on("/api/params", HTTP_GET,  handleParams);
  server.on("/api/cmd",    HTTP_POST, handleCmd);
  server.on("/api/cmd",    HTTP_GET,  handleCmd);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/reboot", HTTP_POST, handleReboot);
  server.on("/api/temp/scan",   HTTP_GET,  handleTempScan);
  server.on("/api/temp/config", HTTP_GET,  handleTempConfigGet);
  server.on("/api/temp/assign", HTTP_POST, handleTempAssign);
  server.on("/api/energy/reset",      HTTP_POST, handleEnergyReset);
  server.on("/api/reboots/reset",     HTTP_POST, handleRebootsReset);

  History::registerRoutes(server);

  events.onOpen([](PsychicEventSourceClient *client) {
    // Hard cap on concurrent SSE clients: each open connection owns a LWIP
    // socket, and a slow/non-reading client can block events.send() for the
    // whole loop task. Drop the newest connection if we're already at the cap.
    if (events.count() > MAX_SSE_CLIENTS) {
      webLog("[SSE] client cap %d exceeded, closing connection", MAX_SSE_CLIENTS);
      client->close();
      return;
    }
    // Don't call send() from the httpd task context. Defer to the loop task;
    // webserver_loop() / main.cpp will drain the flag and broadcast status to
    // all clients (including this new one) from the loop task.
    webserver_ssePushPending = true;
  });
  server.on("/events", &events);

  // Static assets from LittleFS. PsychicStaticFileHandler auto-serves
  // index.html for "/" and adds Cache-Control.
  PsychicStaticFileHandler *staticHandler =
      server.serveStatic("/", LittleFS, "/www/");
  staticHandler->setDefaultFile("index.html");
  staticHandler->setCacheControl("public, max-age=3600");

  server.onNotFound([](PsychicRequest *req) -> esp_err_t {
    return req->reply(404, "text/plain", "Not Found");
  });

  // Server + httpd task are up.
}

void webserver_broadcastStatus(const String &json) {
  if (s_statusMutex) {
    if (xSemaphoreTake(s_statusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      s_statusPayload = json;
      xSemaphoreGive(s_statusMutex);
    }
    // If take times out, skip this update rather than writing unsafely.
  } else {
    s_statusPayload = json;  // only before webserver_begin()
  }
  if (!webserver_pauseSSE) {
    events.send(json.c_str(), "status", millis());
  }
}

void webserver_broadcastPowerFast(int powerdraw, int powerToConsume, int powerDrawAge) {
  // Stack-only, no heap, no JSON library — absolute minimum latency.
  // Rate-limit to 1 Hz: powerdraw MQTT messages may arrive every second, but
  // sending to all SSE clients every second can block the loop task if a client
  // is slow. 1 Hz is plenty for the UI power gauge.
  static unsigned long s_lastFastPush = 0;
  const unsigned long now = millis();
  if (now - s_lastFastPush < 1000) return;
  s_lastFastPush = now;

  char buf[96];
  int len = snprintf(buf, sizeof(buf),
    "{\"Powerdraw\":%d,\"powerToConsume\":%d,\"powerDrawAge\":%d}",
    powerdraw, powerToConsume, powerDrawAge);
  if (len > 0 && len < (int)sizeof(buf) && !webserver_pauseSSE) {
    events.send(buf, "status", millis());
  }
}

// Count active SSE clients for diagnostics — thin wrapper over PsychicHttp's
// own client list (no custom tracking needed).
int webserver_getSseClientCount() {
  return events.count();
}

void webserver_loop() {
  // Automatic reboot disabled; only manual reboot via /api/reboot is allowed.
  // if (rebootPending && millis() >= rebootAt) {
  //   ESP.restart();
  // }

  // Diagnostic heartbeat: track heap + SSE client count over time so we can
  // correlate a future "connection LED red" / unreachable-webserver report
  // with a heap drop or socket-count plateau (see plans/webserver-sse-hang).
  static unsigned long lastDiagLog = 0;
  static constexpr unsigned long DIAG_LOG_MS = 300000;  // every 5 min
  if (millis() - lastDiagLog >= DIAG_LOG_MS) {
    lastDiagLog = millis();
    webLog("[Diag] heap=%u minHeap=%u sse=%d",
           (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
           webserver_getSseClientCount());
  }

  // Drain pending SSE log push. events.send() is only ever called here, from
  // the loop task. Send the most recently stored log line.
  if (webserver_logPushPending && !webserver_pauseSSE) {
    webserver_logPushPending = false;
    if (s_logMutex && xSemaphoreTake(s_logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint8_t idx = (s_logHead + LOG_MAX_LINES - 1) % LOG_MAX_LINES;
      char json[LOG_LINE_LEN + 16];
      int n = snprintf(json, sizeof(json), "{\"log\":\"%s\"}", s_logBuf[idx]);
      xSemaphoreGive(s_logMutex);
      if (n > 0 && n < (int)sizeof(json)) {
        events.send(json, "log", millis());
      }
    }
  }
}
