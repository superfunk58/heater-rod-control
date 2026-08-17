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
#include "history.h"
#include "temp_sensors.h"
#include "json_arena.h"
#include <Update.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <PsychicHttp.h>
#include "tee_print.h"
#include <cstring>
#include <strings.h>

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

// Network config globals (owned by main.cpp) — fixed char arrays, no heap.
extern char NET_MODE[8];
extern bool LAN_DHCP;
extern char LAN_IP[16];
extern char LAN_GW[16];
extern char LAN_MASK[16];
extern char LAN_DNS[16];

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

// Global tee printer: mirrors all output to USB Serial + webLog ring buffer.
TeePrint Log(Serial);
static constexpr int MAX_SSE_CLIENTS = 4;  // hard cap: each idle client still owns a LWIP socket
static constexpr size_t STATUS_PAYLOAD_CAP = 3072;
static char   s_statusPayload[STATUS_PAYLOAD_CAP] = "{}";
static SemaphoreHandle_t s_statusMutex = nullptr;
static bool   rebootPending = false;
static unsigned long rebootAt = 0;
static bool   s_updateUploadOk = false;
static char   s_updateError[96] = "";

// Set by HTTP handlers when state changes; drained by loop() -> sendupdate().
// This ensures events.send() is only ever called from the loop task,
// matching the PoolController pattern that runs stable without mallocs.
volatile bool webserver_ssePushPending = false;

bool webserver_pauseSSE = false;  // set true during OTA to reduce WiFi load

// Set by webLog() whenever a new log line is stored; drained by webserver_loop()
// so events.send() is only ever called from the SSE task.
volatile bool webserver_logPushPending = false;

// ---- SSE send task ------------------------------------------------------
// PsychicHttp's sendEvent() busy-loops on httpd_socket_send() (blocking, flags=0)
// and retries on timeout. If a client's TCP buffer is full (background tab, slow
// network), this can block for seconds. Running events.send() on the loop task
// means a single slow SSE client stalls regulation, drain timer, temp sensors,
// and MQTT — the entire firmware freezes.
//
// Fix: a dedicated SSE task does all events.send() calls. The loop task copies
// the payload into a shared buffer and notifies the SSE task (non-blocking). If
// the SSE task is still busy, the update is dropped — the 2s telemetry heartbeat
// re-syncs the UI automatically.
static TaskHandle_t      s_sseTask = nullptr;
static SemaphoreHandle_t s_sseBusy = nullptr;   // available = SSE task is idle
static char              s_sseStatusBuf[3072];   // shared payload for status sends
static volatile bool     s_sseStatusPending = false;
static char              s_sseLogBuf[128 + 16];   // LOG_LINE_LEN + 16 (defined below)
static volatile bool     s_sseLogPending = false;

static void sseSendTask(void *) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (s_sseStatusPending) {
      s_sseStatusPending = false;
      if (events.safeCount() > 0)
        events.send(s_sseStatusBuf, "status", millis());
    }
    if (s_sseLogPending) {
      s_sseLogPending = false;
      if (events.safeCount() > 0)
        events.send(s_sseLogBuf, "log", millis());
    }
    xSemaphoreGive(s_sseBusy);
  }
}

// Deferred NVS writes: set by HTTP handlers, drained by loop() so flash writes
// never block the httpd task or race with loop-task state.
volatile bool webserver_configSavePending = false;
volatile bool webserver_energyResetPending = false;

// Pending energy injection (set by HTTP handler, drained by loop task)
volatile bool webserver_energyInjectPending = false;
PendingEnergyInject webserver_pendingEnergyInject;

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

static bool hasBinSuffix(const char *filename) {
  if (!filename) return false;
  const char *dot = strrchr(filename, '.');
  return dot && strcasecmp(dot, ".bin") == 0;
}

static void setUpdateError(const char *msg) {
  if (!msg || !*msg) msg = "Update fehlgeschlagen";
  snprintf(s_updateError, sizeof(s_updateError), "%s", msg);
}

// ---- Web log ring buffer (for browser console) --------------------------
static constexpr uint8_t  LOG_MAX_LINES = 30;
static constexpr uint16_t LOG_LINE_LEN  = 128;
static char   s_logBuf[LOG_MAX_LINES][LOG_LINE_LEN];
static uint8_t s_logHead = 0;
static uint8_t s_logCount = 0;
static SemaphoreHandle_t s_logMutex = nullptr;

// Dump the webLog ring buffer to LittleFS /crash.log. Called via
// esp_register_shutdown_handler so it runs on panic/watchdog/brownout
// reboots. Flash writes here are safe: the system is already going down.
static void crashLogDump() {
  if (!s_logMutex) return;
  // Best-effort: no mutex (shutdown context), just read the ring directly.
  File f = LittleFS.open("/crash.log", "w");
  if (!f) return;
  for (uint8_t i = 0; i < s_logCount; i++) {
    uint8_t idx = (s_logHead + LOG_MAX_LINES - s_logCount + i) % LOG_MAX_LINES;
    f.println(s_logBuf[idx]);
  }
  f.close();
}

// Copy up to maxLines log entries into the caller's buffer (oldest first).
// Returns the number of lines copied. Used by the /api/crashlog endpoint.
size_t webserver_getLogLines(char *buf, size_t bufLen, uint8_t maxLines) {
  if (!s_logMutex || !buf || bufLen == 0) return 0;
  if (xSemaphoreTake(s_logMutex, pdMS_TO_TICKS(50)) != pdTRUE) return 0;
  size_t pos = 0;
  uint8_t n = s_logCount < maxLines ? s_logCount : maxLines;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx = (s_logHead + LOG_MAX_LINES - s_logCount + i) % LOG_MAX_LINES;
    size_t len = strlen(s_logBuf[idx]);
    if (pos + len + 2 >= bufLen) break;
    memcpy(buf + pos, s_logBuf[idx], len);
    buf[pos + len] = '\n';
    pos += len + 1;
  }
  buf[pos] = '\0';
  xSemaphoreGive(s_logMutex);
  return n;
}

// Store a line into the webLog ring buffer without Serial output.
// Called by TeePrint (Log.print*) and by webLog().
void webLogBufferLine(const char *line) {
  if (!s_logMutex) s_logMutex = xSemaphoreCreateMutex();
  if (s_logMutex && xSemaphoreTake(s_logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    strncpy(s_logBuf[s_logHead], line, LOG_LINE_LEN - 1);
    s_logBuf[s_logHead][LOG_LINE_LEN - 1] = '\0';
    s_logHead = (s_logHead + 1) % LOG_MAX_LINES;
    if (s_logCount < LOG_MAX_LINES) s_logCount++;
    xSemaphoreGive(s_logMutex);
  }
  webserver_logPushPending = true;
}

void webLog(const char* fmt, ...) {
  char line[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  // Log.println writes to Serial AND webLogBufferLine via TeePrint.
  // Do NOT call webLogBufferLine() directly here — that would double-log.
  /*KEEP*/Log.println(line);/*KEEP*/
}

// Direct pointer into the request's parameter value (no String copy).
static const char *paramOr(PsychicRequest *req, const char *name, const char *def) {
  if (req->hasParam(name)) return req->getParam(name)->value().c_str();
  return def;
}

// Interpret a JSON value as bool: accepts real booleans and the string forms
// used by the form-encoded fallback ("1"/"true"/"on").
static bool jsonToBool(JsonVariantConst v) {
  if (v.is<bool>()) return v.as<bool>();
  const char *s = v.as<const char *>();
  return s && (!strcmp(s, "1") || !strcmp(s, "true") || !strcmp(s, "on"));
}

// Arena for the JSON documents built by the HTTP handlers below. All handlers
// run on the single esp_http_server task (serialized), so one shared arena is
// safe. reset() before each use — never touches the heap.
alignas(8) static uint8_t s_httpArenaBuf[2048];
static JsonArena s_httpArena(s_httpArenaBuf, sizeof(s_httpArenaBuf));

// ---- Route handlers ----------------------------------------------------
static esp_err_t handleStatus(PsychicRequest *req) {
  if (!s_statusMutex) {
    return req->reply(200, "application/json", s_statusPayload);
  }
  if (xSemaphoreTake(s_statusMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return req->reply(503, "application/json", "{\"error\":\"busy\"}");
  }
  // Static: 3 KB on the 4 KB httpd task stack would overflow it. Safe because
  // all handlers are serialized on the single esp_http_server task.
  static char snapshot[STATUS_PAYLOAD_CAP];
  memcpy(snapshot, s_statusPayload, sizeof(snapshot));
  xSemaphoreGive(s_statusMutex);
  return req->reply(200, "application/json", snapshot);
}

static esp_err_t handleCmd(PsychicRequest *req) {
  const char *cmd = paramOr(req, "cmd", "");
  bool changed = false;
  bool persist = false;
  if      (!strcmp(cmd, "pump_on"))      { pumpmanualpower = true;  changed = true; }
  else if (!strcmp(cmd, "pump_off"))     { pumpmanualpower = false; changed = true; }
  else if (!strcmp(cmd, "regulate_on"))  { regulating_power = true;  changed = true; persist = true; }
  else if (!strcmp(cmd, "regulate_off")) { regulating_power = false; changed = true; persist = true; }
  else if (!strcmp(cmd, "drain_on")) {
    int durSec = atoi(paramOr(req, "dur", "3"));
    if (durSec < 1 || durSec > 60) durSec = 3;  // clamp to 1-60 seconds
    drainPulseMs = durSec * 1000;
    drainTriggerReq = true;
    changed = true;
  }
  else if (!strcmp(cmd, "drain_off"))    { drainCancelReq  = true;  changed = true; }
  else { return req->reply(400, "text/plain", "unknown cmd"); }

  if (persist) webserver_configSavePending = true;  // deferred to loop task
  if (changed) webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "ok");
}

static esp_err_t handleConfig(PsychicRequest *req) {
  s_httpArena.reset();
  JsonDocument doc(&s_httpArena);

  // Start fresh: each config request carries the values to change. Fill a local
  // struct first; the global pending struct is swapped atomically at the end.
  PendingConfig local;

  // Try to parse JSON body if Content-Type is application/json
  if (req->contentType() == "application/json" && req->contentLength() > 0) {
    DeserializationError err = deserializeJson(doc, req->body());
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
      doc["pump_temp_cond"] = req->getParam("pump_temp_cond")->value().c_str();
    }
    if (req->hasParam("pump_temp_hyst")) {
      doc["pump_temp_hyst"] = req->getParam("pump_temp_hyst")->value().toFloat();
    }
    if (req->hasParam("vol_enabled")) {
      doc["vol_enabled"] = req->getParam("vol_enabled")->value().c_str();
    }
    if (req->hasParam("history_averaging")) {
      doc["history_averaging"] = req->getParam("history_averaging")->value().c_str();
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
      doc["net_mode"] = req->getParam("net_mode")->value().c_str();
    }
    if (req->hasParam("lan_dhcp")) {
      doc["lan_dhcp"] = req->getParam("lan_dhcp")->value().c_str();
    }
    if (req->hasParam("lan_ip")) {
      doc["lan_ip"] = req->getParam("lan_ip")->value().c_str();
    }
    if (req->hasParam("lan_gw")) {
      doc["lan_gw"] = req->getParam("lan_gw")->value().c_str();
    }
    if (req->hasParam("lan_mask")) {
      doc["lan_mask"] = req->getParam("lan_mask")->value().c_str();
    }
    if (req->hasParam("lan_dns")) {
      doc["lan_dns"] = req->getParam("lan_dns")->value().c_str();
    }
    if (req->hasParam("mqtt_status_enabled")) {
      doc["mqtt_status_enabled"] = req->getParam("mqtt_status_enabled")->value().c_str();
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
    local.hasPumpTempCond = true;
    local.pumpTempCond = jsonToBool(doc["pump_temp_cond"]);
  }
  if (!doc["pump_temp_hyst"].isNull()) {
    float v = doc["pump_temp_hyst"];
    if (v >= 0.0f && v <= 30.0f) {
      local.hasPumpTempHyst = true;
      local.pumpTempHyst = v;
    }
  }
  if (!doc["vol_enabled"].isNull()) {
    local.hasVolEnabled = true;
    local.volEnabled = jsonToBool(doc["vol_enabled"]);
  }
  if (!doc["history_averaging"].isNull()) {
    local.hasHistoryAveraging = true;
    local.historyAveraging = jsonToBool(doc["history_averaging"]);
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
    // Exclude input-only (34-39), flash SPI (6-11), UART0 (1,3), boot-strapping
    // pins (0,2,12) and pins already used by other hardware: 13 = drain relay,
    // 23 = pump, 21/22 = I2C DAC, 4/16/17/18/19/32 = W5500 SPI (net_manager.h).
    // A bad pin is persisted to NVS and only applies at the NEXT boot — GPIO12
    // would prevent the device from booting at all.
    static const uint8_t forbidden[] = {0, 1, 2, 3, 4, 12, 13, 16, 17, 18, 19, 21, 22, 23, 32};
    bool usable = (n >= 0 && n <= 39) && !(n >= 6 && n <= 11) && !(n >= 34 && n <= 39);
    for (size_t i = 0; i < sizeof(forbidden); i++) {
      if (n == forbidden[i]) { usable = false; break; }
    }
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
    const char *v = doc["net_mode"];
    if (v && (!strcmp(v, "wifi") || !strcmp(v, "lan"))) {
      local.hasNetMode = true;
      strlcpy(local.netMode, v, sizeof(local.netMode));
    }
  }
  if (!doc["lan_dhcp"].isNull()) {
    local.hasLanDhcp = true;
    local.lanDhcp = jsonToBool(doc["lan_dhcp"]);
  }
  if (!doc["lan_ip"].isNull()) {
    const char *v = doc["lan_ip"];
    if (v) { local.hasLanIp = true; strlcpy(local.lanIp, v, sizeof(local.lanIp)); }
  }
  if (!doc["lan_gw"].isNull()) {
    const char *v = doc["lan_gw"];
    if (v) { local.hasLanGw = true; strlcpy(local.lanGw, v, sizeof(local.lanGw)); }
  }
  if (!doc["lan_mask"].isNull()) {
    const char *v = doc["lan_mask"];
    if (v) { local.hasLanMask = true; strlcpy(local.lanMask, v, sizeof(local.lanMask)); }
  }
  if (!doc["lan_dns"].isNull()) {
    const char *v = doc["lan_dns"];
    if (v) { local.hasLanDns = true; strlcpy(local.lanDns, v, sizeof(local.lanDns)); }
  }
  if (!doc["mqtt_status_enabled"].isNull()) {
    local.hasMqttStatusEnabled = true;
    local.mqttStatusEnabled = jsonToBool(doc["mqtt_status_enabled"]);
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
  if (local.hasNetMode) netChanged |= (strcmp(local.netMode, NET_MODE) != 0);
  if (local.hasLanIp)   netChanged |= (strcmp(local.lanIp,   LAN_IP)   != 0);
  if (local.hasLanGw)   netChanged |= (strcmp(local.lanGw,   LAN_GW)   != 0);
  if (local.hasLanMask) netChanged |= (strcmp(local.lanMask, LAN_MASK) != 0);
  if (local.hasLanDns)  netChanged |= (strcmp(local.lanDns,  LAN_DNS)  != 0);
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
  s_httpArena.reset();
  JsonDocument doc(&s_httpArena);
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

  char json[768];
  serializeJson(doc, json, sizeof(json));
  return req->reply(200, "application/json", json);
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
  // Request a non-blocking rescan on the loop task. The rescan enumerates
  // sensors and issues an async conversion (~250ms). Results appear in
  // scanList() on the next status poll — no need to block the httpd task.
  TempSensors::requestRescan();
  // Static: together with the 2 KB output buffer below this would overflow the
  // 4 KB httpd task stack. Safe: handlers are serialized on the httpd task.
  static TempSensors::Found list[TempSensors::MAX_SENSORS];
  const uint8_t listCount = TempSensors::scanList(list, TempSensors::MAX_SENSORS);
  s_httpArena.reset();
  JsonDocument doc(&s_httpArena);
  JsonArray sensors = doc["sensors"].to<JsonArray>();
  for (uint8_t k = 0; k < listCount; k++) {
    JsonObject obj = sensors.add<JsonObject>();
    obj["rom"] = list[k].romHex;
    if (list[k].current_c > -50.0f && list[k].current_c < 150.0f) obj["current_c"] = round(list[k].current_c * 10.0f) / 10.0f;
    else obj["current_c"] = nullptr;
  }
  static char out[2048];  // static: 2 KB stack buffer would overflow httpd task
  serializeJson(doc, out, sizeof(out));
  return req->reply(200, "application/json", out);
}

// GET /api/temp/config -> aktuelle Rolle->ROM-Zuweisung
static esp_err_t handleTempConfigGet(PsychicRequest *req) {
  s_httpArena.reset();
  JsonDocument doc(&s_httpArena);
  char boilerHex[24], inletHex[24], outletHex[24], hrodHex[24];
  const uint64_t rBoiler = TempSensors::boilerRom();
  const uint64_t rInlet  = TempSensors::inletRom();
  const uint64_t rOutlet = TempSensors::outletRom();
  const uint64_t rHrod   = TempSensors::heaterRodRom();
  if (rBoiler) TempSensors::romToHex(rBoiler, boilerHex);
  if (rInlet)  TempSensors::romToHex(rInlet,  inletHex);
  if (rOutlet) TempSensors::romToHex(rOutlet, outletHex);
  if (rHrod)   TempSensors::romToHex(rHrod,   hrodHex);
  doc["boiler_rom"]  = rBoiler ? boilerHex : "";
  doc["inlet_rom"]   = rInlet  ? inletHex  : "";
  doc["outlet_rom"]  = rOutlet ? outletHex : "";
  doc["hrod_rom"]    = rHrod   ? hrodHex   : "";
  char out[384];
  serializeJson(doc, out, sizeof(out));
  return req->reply(200, "application/json", out);
}

// POST /api/temp/assign -> body: boiler_rom=...&inlet_rom=...
// Leerer String -> Rolle löschen.
static esp_err_t handleTempAssign(PsychicRequest *req) {
  bool any = false;
  if (req->hasParam("boiler_rom")) {
    const char *v = req->getParam("boiler_rom")->value().c_str();
    TempSensors::assignBoiler(v[0] ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (req->hasParam("inlet_rom")) {
    const char *v = req->getParam("inlet_rom")->value().c_str();
    TempSensors::assignInlet(v[0] ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (req->hasParam("outlet_rom")) {
    const char *v = req->getParam("outlet_rom")->value().c_str();
    TempSensors::assignOutlet(v[0] ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (req->hasParam("hrod_rom")) {
    const char *v = req->getParam("hrod_rom")->value().c_str();
    TempSensors::assignHeaterRod(v[0] ? TempSensors::romFromHex(v) : 0);
    any = true;
  }
  if (!any) return req->reply(400, "text/plain", "no fields");
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "ok");
}

// POST /api/energy/reset -> alle Energie-Zähler löschen
static esp_err_t handleEnergyReset(PsychicRequest *req) {
  webserver_energyResetPending = true;  // deferred to loop task (avoids race with Energy::tick)
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "reset");
}

// POST /api/energy/inject -> inject Wh for a previous month
// Body: {"year":2026,"month":7,"wh":12345}  (wh in Watt-hours)
static esp_err_t handleEnergyInject(PsychicRequest *req) {
  s_httpArena.reset();
  JsonDocument doc(&s_httpArena);

  int year = 0, month = 0, wh = 0;

  if (req->contentType() == "application/json" && req->contentLength() > 0) {
    DeserializationError err = deserializeJson(doc, req->body());
    if (err) return req->reply(400, "text/plain", "Invalid JSON");
    year  = doc["year"]  | 0;
    month = doc["month"] | 0;
    wh    = doc["wh"]    | 0;
  } else {
    if (req->hasParam("year"))  year  = req->getParam("year")->value().toInt();
    if (req->hasParam("month")) month = req->getParam("month")->value().toInt();
    if (req->hasParam("wh"))    wh    = req->getParam("wh")->value().toInt();
  }

  if (year < 2000 || year > 3000 || month < 1 || month > 12 || wh < 0)
    return req->reply(400, "text/plain", "Invalid year/month/wh");

  webserver_pendingEnergyInject.year  = (uint16_t)year;
  webserver_pendingEnergyInject.month = (uint8_t)month;
  webserver_pendingEnergyInject.wh    = (uint32_t)wh;
  webserver_energyInjectPending = true;
  webserver_ssePushPending = true;

  return req->reply(200, "text/plain", "ok");
}

// POST /api/reboots/reset -> persistenten Reboot-Zähler auf 0 zurücksetzen
static esp_err_t handleRebootsReset(PsychicRequest *req) {
  resetRebootCounter();
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "reset");
}

// GET /api/crashlog -> return crash.log from LittleFS (if present after a crash)
//                      plus the current RAM log ring buffer.
static esp_err_t handleCrashLog(PsychicRequest *req) {
  static char buf[2048];
  size_t pos = 0;

  // 1. LittleFS crash.log (from last shutdown/panic)
  if (LittleFS.exists("/crash.log")) {
    File f = LittleFS.open("/crash.log", "r");
    if (f) {
      pos += snprintf(buf + pos, sizeof(buf) - pos, "=== crash.log (last shutdown) ===\n");
      while (f.available() && pos < sizeof(buf) - 256) {
        size_t n = f.read((uint8_t*)(buf + pos), sizeof(buf) - pos - 256);
        if (n == 0) break;
        pos += n;
      }
      f.close();
      if (pos < sizeof(buf) - 2) buf[pos++] = '\n';
    }
  }

  // 2. Current RAM log ring buffer
  pos += snprintf(buf + pos, sizeof(buf) - pos, "=== RAM log (current session) ===\n");
  webserver_getLogLines(buf + pos, sizeof(buf) - pos - 1, LOG_MAX_LINES);

  return req->reply(200, "text/plain", buf);
}

static esp_err_t handleFirmwareUploadChunk(PsychicRequest *req, const String &filename, uint64_t index, uint8_t *data, size_t len, bool last) {
  (void)req;
  if (index == 0) {
    s_updateUploadOk = false;
    s_updateError[0] = '\0';
    if (!hasBinSuffix(filename.c_str())) {
      setUpdateError("Nur .bin Firmware-Dateien sind erlaubt");
      return ESP_FAIL;
    }
    webserver_pauseSSE = true;
    Update.clearError();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      setUpdateError(Update.errorString());
      webserver_pauseSSE = false;
      return ESP_FAIL;
    }
    webLog("[OTA] HTTP upload gestartet: %s", filename.c_str());
  }

  if (len > 0 && Update.write(data, len) != len) {
    setUpdateError(Update.errorString());
    Update.abort();
    webserver_pauseSSE = false;
    return ESP_FAIL;
  }

  if (last) {
    if (!Update.end(true)) {
      setUpdateError(Update.errorString());
      Update.abort();
      webserver_pauseSSE = false;
      return ESP_FAIL;
    }
    s_updateUploadOk = true;
    History::saveNow();
    rebootPending = true;
    rebootAt = millis() + 1500;
    webLog("[OTA] HTTP upload abgeschlossen: %s", filename.c_str());
  }

  return ESP_OK;
}

static esp_err_t handleFirmwareUploadDone(PsychicRequest *req) {
  if (s_updateUploadOk) {
    return req->reply(200, "text/plain", "ok - firmware uploaded, rebooting");
  }
  webserver_pauseSSE = false;
  const char *msg = s_updateError[0] ? s_updateError : "Upload fehlgeschlagen";
  return req->reply(500, "text/plain", msg);
}

// ---- Public API --------------------------------------------------------
void webserver_begin() {
  // Idempotent: safe to call repeatedly from loop() until network is up.
  static bool s_begun = false;
  if (s_begun) return;
  s_begun = true;

  if (!s_statusMutex) s_statusMutex = xSemaphoreCreateMutex();

  // Create the SSE send task (pinned to core 0 alongside the httpd task so it
  // never competes with the loop task on core 1). Stack: events.send() allocates
  // a String on the heap, so 4 KB stack is plenty.
  if (!s_sseBusy) {
    s_sseBusy = xSemaphoreCreateBinary();
    xSemaphoreGive(s_sseBusy);  // initially available (SSE task is idle)
  }
  if (!s_sseTask) {
    xTaskCreatePinnedToCore(sseSendTask, "sse", 4096, nullptr, 1, &s_sseTask, 0);
  }
  // Default httpd stack is only 4 KB; several handlers build/serialize JSON
  // responses. 6 KB gives comfortable margin (RAM is plentiful).
  server.config.stack_size       = 6144;
  server.config.max_uri_handlers = 20;
  server.config.max_open_sockets   = 7;
  server.config.send_wait_timeout  = 1;   // seconds: limit SSE/httpd send block during relay EMI
  server.config.lru_purge_enable   = true; // close zombie connections when out of sockets
  server.maxUploadSize = 3 * 1024 * 1024;
  // Register crash log dump so it runs automatically on shutdown/panic.
  esp_register_shutdown_handler(crashLogDump);

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
  server.on("/api/energy/inject",     HTTP_POST, handleEnergyInject);
  server.on("/api/reboots/reset",     HTTP_POST, handleRebootsReset);
  server.on("/api/crashlog",           HTTP_GET,  handleCrashLog);

  PsychicUploadHandler *firmwareUploadHandler = new PsychicUploadHandler();
  firmwareUploadHandler->onUpload(handleFirmwareUploadChunk);
  firmwareUploadHandler->onRequest(handleFirmwareUploadDone);
  server.on("/api/update/firmware", HTTP_POST, firmwareUploadHandler);

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

void webserver_broadcastStatus(const char *json) {
  if (!json) json = "{}";
  if (s_statusMutex) {
    if (xSemaphoreTake(s_statusMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      size_t len = strnlen(json, STATUS_PAYLOAD_CAP - 1);
      memcpy(s_statusPayload, json, len);
      s_statusPayload[len] = '\0';
      xSemaphoreGive(s_statusMutex);
    }
    // If take times out, skip this update rather than writing unsafely.
  } else {
    size_t len = strnlen(json, STATUS_PAYLOAD_CAP - 1);
    memcpy(s_statusPayload, json, len);
    s_statusPayload[len] = '\0';
  }
  // Offload SSE send to the dedicated task so a slow client can never block
  // the loop task. If the SSE task is still busy, drop this update — the 2s
  // telemetry heartbeat will re-sync the UI.
  if (!webserver_pauseSSE && events.safeCount() > 0 && s_sseBusy) {
    if (xSemaphoreTake(s_sseBusy, 0) == pdTRUE) {
      strncpy(s_sseStatusBuf, json, sizeof(s_sseStatusBuf) - 1);
      s_sseStatusBuf[sizeof(s_sseStatusBuf) - 1] = '\0';
      s_sseStatusPending = true;
      xTaskNotifyGive(s_sseTask);
    }
  }
}

void webserver_broadcastPowerFast(int powerdraw, int powerToConsume, int powerDrawAge) {
  // Stack-only, no heap, no JSON library — absolute minimum latency.
  // Rate-limit to 1 Hz: powerdraw MQTT messages may arrive every second, but
  // sending to all SSE clients every second can overload slow clients. 1 Hz is
  // plenty for the UI power gauge.
  static unsigned long s_lastFastPush = 0;
  const unsigned long now = millis();
  if (now - s_lastFastPush < 1000) return;
  s_lastFastPush = now;

  char buf[96];
  int len = snprintf(buf, sizeof(buf),
    "{\"Powerdraw\":%d,\"powerToConsume\":%d,\"powerDrawAge\":%d}",
    powerdraw, powerToConsume, powerDrawAge);
  if (len > 0 && len < (int)sizeof(buf) && !webserver_pauseSSE && events.safeCount() > 0 && s_sseBusy) {
    if (xSemaphoreTake(s_sseBusy, 0) == pdTRUE) {
      strncpy(s_sseStatusBuf, buf, sizeof(s_sseStatusBuf) - 1);
      s_sseStatusBuf[sizeof(s_sseStatusBuf) - 1] = '\0';
      s_sseStatusPending = true;
      xTaskNotifyGive(s_sseTask);
    }
  }
}

// Count active SSE clients for diagnostics — thin wrapper over PsychicHttp's
// own client list (no custom tracking needed).
int webserver_getSseClientCount() {
  return events.safeCount();
}

// Close all SSE clients. Called when the active network interface changes
// (LAN↔WiFi fallback) so zombie connections from the old interface are cleaned
// up immediately. Without this, dead sockets block the SSE task for up to
// send_wait_timeout (2s) each and exhaust max_open_sockets.
void webserver_closeAllSseClients() {
  int n = events.safeCount();
  if (n == 0) return;
  webLog("[SSE] closing all %d clients (interface change)", n);
  events.closeAll();
}

void webserver_loop() {
  if (rebootPending && millis() >= rebootAt) {
    ESP.restart();
  }

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

  // Drain pending SSE log push. Offloaded to the SSE task so events.send()
  // can never block the loop task.
  if (webserver_logPushPending && !webserver_pauseSSE) {
    webserver_logPushPending = false;
    if (s_logMutex && xSemaphoreTake(s_logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint8_t idx = (s_logHead + LOG_MAX_LINES - 1) % LOG_MAX_LINES;
      int n = snprintf(s_sseLogBuf, sizeof(s_sseLogBuf), "{\"log\":\"%s\"}", s_logBuf[idx]);
      xSemaphoreGive(s_logMutex);
      if (n > 0 && n < (int)sizeof(s_sseLogBuf) && events.safeCount() > 0 && s_sseBusy) {
        if (xSemaphoreTake(s_sseBusy, 0) == pdTRUE) {
          s_sseLogPending = true;
          xTaskNotifyGive(s_sseTask);
        }
      }
    }
  }
}
