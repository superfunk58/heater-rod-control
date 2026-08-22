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
#include "pid_controller.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <PsychicHttp.h>

// State owned by main.cpp
extern int powerdrawnumber;
extern int powerdrawsetpoint;
extern int powerToConsume;
extern int ZERO_FEED_IN_TARGET;
extern int MAX_HEATING_POWER;
extern int MIN_POWER_THRESHOLD;
extern int DEADBAND;
extern int POWER_CHANGE_THRESHOLD;
extern int MAX_BOILER_TEMP_C;
extern int MAX_HEATER_ROD_TEMP_C;
extern float correctionGain;
extern bool pumpmanualpower;
extern bool pumpautocontrolled;
extern bool regulating_power;
extern bool heating;
extern int DACoutput;
extern int wattneeded;
extern unsigned long PUMP_MIN_RUNTIME_MS;
extern unsigned long PUMP_CYCLE_INTERVAL_MIN;
extern unsigned long PUMP_CYCLE_DURATION_SEC;
extern bool  PUMP_TEMP_COND_ENABLED;
extern float PUMP_TEMP_HYST_C;
extern unsigned long lastPowerDrawUpdate;
extern String controllerMode;
extern bool  VOL_ENABLED;
extern int   VOL_WINDOW_MIN;
extern int   VOL_THRESHOLD_W;
extern int   ONEWIRE_PIN;

void sendupdate(bool force);

// ---- Module state ------------------------------------------------------
static PsychicHttpServer  server;
static PsychicEventSource events;
static String s_statusPayload;
static SemaphoreHandle_t s_statusMutex = nullptr;
static bool   rebootPending = false;
static unsigned long rebootAt = 0;

// Set by HTTP handlers when state changes; drained by loop() -> sendupdate().
// This ensures events.send() is only ever called from the loop task,
// matching the PoolController pattern that runs stable without mallocs.
volatile bool webserver_ssePushPending = false;

bool webserver_pauseSSE = false;  // set true during OTA to reduce WiFi load

// ---- SSE session lifetime management ------------------------------------
// Prevents zombie SSE connections from exhausting the 7-socket pool.
// Each client is tracked with its connection timestamp. Clients older than
// SSE_TTL_MS are forcibly closed; the browser's EventSource auto-reconnects
// within ~2 s (reconnect hint sent in onOpen). No data loss.
// Reduced TTL and max tracked clients to minimize WiFi interference
static constexpr unsigned long SSE_TTL_MS       = 60000;   // 60 sec max per SSE session
static constexpr uint8_t      SSE_MAX_TRACKED   = 3;       // max 3 SSE clients
static constexpr unsigned long SSE_CLEANUP_MS    = 30000;   // check every 30 s

struct SseSlot {
  PsychicEventSourceClient *client = nullptr;
  unsigned long openedAt = 0;
};
static SseSlot s_sseSlots[SSE_MAX_TRACKED];

static void sseTrackAdd(PsychicEventSourceClient *c) {
  // find free slot
  for (uint8_t i = 0; i < SSE_MAX_TRACKED; i++) {
    if (s_sseSlots[i].client == nullptr) {
      s_sseSlots[i].client = c;
      s_sseSlots[i].openedAt = millis();
      return;
    }
  }
  // all slots full — close the oldest, reuse its slot
  uint8_t oldest = 0;
  for (uint8_t i = 1; i < SSE_MAX_TRACKED; i++) {
    if (s_sseSlots[i].openedAt < s_sseSlots[oldest].openedAt) oldest = i;
  }
  if (s_sseSlots[oldest].client) s_sseSlots[oldest].client->close();
  s_sseSlots[oldest].client = c;
  s_sseSlots[oldest].openedAt = millis();
}

static void sseTrackRemove(PsychicClient *c) {
  for (uint8_t i = 0; i < SSE_MAX_TRACKED; i++) {
    if ((void*)s_sseSlots[i].client == (void*)c) {
      s_sseSlots[i].client = nullptr;
      s_sseSlots[i].openedAt = 0;
      return;
    }
  }
}

// Count active SSE clients for diagnostics
int webserver_getSseClientCount() {
  int count = 0;
  for (uint8_t i = 0; i < SSE_MAX_TRACKED; i++) {
    if (s_sseSlots[i].client != nullptr) count++;
  }
  return count;
}

static void sseCleanupStale() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < SSE_MAX_TRACKED; i++) {
    if (s_sseSlots[i].client && (now - s_sseSlots[i].openedAt) >= SSE_TTL_MS) {
      s_sseSlots[i].client->close();
      s_sseSlots[i].client = nullptr;
      s_sseSlots[i].openedAt = 0;
    }
  }
}


static inline String snapshotStatus() {
  if (!s_statusMutex) return s_statusPayload;
  String copy;
  if (xSemaphoreTake(s_statusMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
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

void webLog(const char* fmt, ...) {
  char line[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  Serial.println(line);

  // Store in ring buffer
  strncpy(s_logBuf[s_logHead], line, LOG_LINE_LEN - 1);
  s_logBuf[s_logHead][LOG_LINE_LEN - 1] = '\0';
  s_logHead = (s_logHead + 1) % LOG_MAX_LINES;
  if (s_logCount < LOG_MAX_LINES) s_logCount++;

  // Send as SSE log event (skip during OTA to reduce WiFi load).
  // Use a stack buffer instead of String concatenation — repeated small heap
  // allocations here were a major source of long-run heap fragmentation.
  if (!webserver_pauseSSE) {
    char json[LOG_LINE_LEN + 16];
    // Note: no JSON-escaping; webLog callers control the format strings.
    int n = snprintf(json, sizeof(json), "{\"log\":\"%s\"}", line);
    if (n > 0 && n < (int)sizeof(json)) {
      events.send(json, "log", millis());
    }
  }
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
  else { return req->reply(400, "text/plain", "unknown cmd"); }

  if (persist) ConfigStore::save();
  if (changed) webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "ok");
}

static esp_err_t handleConfig(PsychicRequest *req) {
  JsonDocument doc;
  
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
    if (req->hasParam("controller_mode")) {
      doc["controller_mode"] = req->getParam("controller_mode")->value();
    }
    if (req->hasParam("online_adapt")) {
      doc["online_adapt"] = req->getParam("online_adapt")->value();
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
  }
  
  // Process JSON document
  if (!doc["zero_feed_target"].isNull()) {
    int n = doc["zero_feed_target"];
    if (n >= -1000 && n <= 1000) ZERO_FEED_IN_TARGET = n;
  }
  if (!doc["max_heating_power"].isNull()) {
    int n = doc["max_heating_power"];
    if (n >= 1 && n <= 2000) MAX_HEATING_POWER = n;
  }
  if (!doc["min_power_threshold"].isNull()) {
    int n = doc["min_power_threshold"];
    if (n >= 0 && n <= 99) MIN_POWER_THRESHOLD = n;
  }
  if (!doc["deadband"].isNull()) {
    int n = doc["deadband"];
    if (n >= 0 && n <= 499) DEADBAND = n;
  }
  if (!doc["power_change_threshold"].isNull()) {
    int n = doc["power_change_threshold"];
    if (n >= 1 && n <= 100) POWER_CHANGE_THRESHOLD = n;
  }
  if (!doc["correction_gain"].isNull()) {
    int pct = doc["correction_gain"];
    if (pct >= 1 && pct <= 150) correctionGain = pct / 100.0f;
  }
  if (!doc["pump_min_runtime"].isNull()) {
    long sec = doc["pump_min_runtime"];
    if (sec >= 5 && sec <= 300) PUMP_MIN_RUNTIME_MS = (unsigned long)sec * 1000UL;
  }
  if (!doc["pump_cycle_interval"].isNull()) {
    long min = doc["pump_cycle_interval"];
    if (min >= 0 && min <= 1440) PUMP_CYCLE_INTERVAL_MIN = (unsigned long)min;
  }
  if (!doc["pump_cycle_duration"].isNull()) {
    long sec = doc["pump_cycle_duration"];
    if (sec >= 0 && sec <= 3600) PUMP_CYCLE_DURATION_SEC = (unsigned long)sec;
  }
  if (!doc["controller_mode"].isNull()) {
    String v = doc["controller_mode"];
    if (v == "classic" || v == "pid") controllerMode = v;
  }
  if (!doc["pid_kp"].isNull()) {
    float v = doc["pid_kp"];
    if (v >= 0.5f && v <= 30.0f) PidController::setKp(v);
  }
  if (!doc["pid_ki"].isNull()) {
    float v = doc["pid_ki"];
    if (v >= 0.05f && v <= 5.0f) PidController::setKi(v);
  }
  if (!doc["pid_solar_ff"].isNull()) {
    float v = doc["pid_solar_ff"];
    if (v >= 0.0f && v <= 100.0f) PidController::setSolarFf(v);
  }
  if (!doc["online_adapt"].isNull()) {
    String v = doc["online_adapt"];
    PidController::setOnlineAdaptEnabled(v == "1" || v == "true" || v == "on");
  }
  if (!doc["pump_temp_cond"].isNull()) {
    String v = doc["pump_temp_cond"];
    PUMP_TEMP_COND_ENABLED = (v == "1" || v == "true" || v == "on");
  }
  if (!doc["pump_temp_hyst"].isNull()) {
    float v = doc["pump_temp_hyst"];
    if (v >= 0.0f && v <= 30.0f) PUMP_TEMP_HYST_C = v;
  }
  if (!doc["vol_enabled"].isNull()) {
    String v = doc["vol_enabled"];
    VOL_ENABLED = (v == "1" || v == "true" || v == "on");
  }
  if (!doc["vol_window_min"].isNull()) {
    int n = doc["vol_window_min"];
    if (n >= 1 && n <= 15) VOL_WINDOW_MIN = n;
  }
  if (!doc["vol_threshold_w"].isNull()) {
    int n = doc["vol_threshold_w"];
    if (n >= 0 && n <= 5000) VOL_THRESHOLD_W = n;
  }
  if (!doc["onewire_pin"].isNull()) {
    int n = doc["onewire_pin"];
    // Exclude input-only (34-39) and flash SPI (6-11) pins
    bool usable = (n >= 0 && n <= 39) && !(n >= 6 && n <= 11) && !(n >= 34 && n <= 39);
    if (usable) ONEWIRE_PIN = n;
  }
  if (!doc["max_boiler_temp"].isNull()) {
    int n = doc["max_boiler_temp"];
    if (n >= 0 && n <= 100) MAX_BOILER_TEMP_C = n;
  }
  if (!doc["max_heater_rod_temp"].isNull()) {
    int n = doc["max_heater_rod_temp"];
    if (n >= 0 && n <= 100) MAX_HEATER_ROD_TEMP_C = n;
  }

  ConfigStore::save();      // persist all updated values to NVS
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "ok");
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

// ---- PID controller routes --------------------------------------------
// GET /api/pid/status -> kp/ki/ff + autotune state + recent error samples
static esp_err_t handlePidStatus(PsychicRequest *req) {
  int16_t errs[60];
  size_t  n = PidController::getRecentErrors(errs, 60);

  JsonDocument doc;
  doc["controllerMode"] = controllerMode;
  doc["kp"] = PidController::kp();
  doc["ki"] = PidController::ki();
  doc["solar_ff"] = PidController::solarFf();
  doc["integrator"] = (long)PidController::integrator();
  doc["last_good_dac"] = PidController::lastGoodDac();
  doc["online_adapt"] = PidController::onlineAdaptEnabled();
  doc["last_adapt_epoch"] = (unsigned long)PidController::lastAdaptEpoch();
  doc["autotune_state"] = PidController::autotuneStateStr();
  doc["autotune_progress"] = PidController::autotuneProgressPercent();
  doc["autotune_timestamp"] = (unsigned long)PidController::autotuneTimestamp();
  JsonArray arr = doc["recent_errors"].to<JsonArray>();
  for (size_t i = 0; i < n; i++) arr.add((int)errs[i]);
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

// POST /api/pid/autotune -> Relay-Feedback Autotune starten
static esp_err_t handlePidAutotuneStart(PsychicRequest *req) {
  if (!PidController::startAutotune()) {
    return req->reply(409, "text/plain", "autotune busy or preconditions failed");
  }
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "started");
}

// POST /api/pid/autotune/stop -> laufenden Autotune abbrechen
static esp_err_t handlePidAutotuneStop(PsychicRequest *req) {
  PidController::stopAutotune();
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "stopped");
}

// POST /api/energy/reset -> alle Energie-Zähler löschen
static esp_err_t handleEnergyReset(PsychicRequest *req) {
  Energy::resetAll();
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "reset");
}

// POST /api/pid/reset -> Gains auf Defaults
static esp_err_t handlePidReset(PsychicRequest *req) {
  PidController::resetToDefaults();
  webserver_ssePushPending = true;
  return req->reply(200, "text/plain", "reset");
}

// ---- Public API --------------------------------------------------------
void webserver_begin() {
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
  server.on("/api/pid/status",        HTTP_GET,  handlePidStatus);
  server.on("/api/pid/autotune",      HTTP_POST, handlePidAutotuneStart);
  server.on("/api/pid/autotune/stop", HTTP_POST, handlePidAutotuneStop);
  server.on("/api/pid/reset",         HTTP_POST, handlePidReset);
  server.on("/api/energy/reset",      HTTP_POST, handleEnergyReset);

  History::registerRoutes(server);

  events.onOpen([](PsychicEventSourceClient *client) {
    sseTrackAdd(client);
    String snap = snapshotStatus();
    if (snap.length()) {
      client->send(snap.c_str(), "status", millis(), 2000);
    }
  });
  events.onClose([](PsychicClient *client) {
    sseTrackRemove(client);
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
  if (s_statusMutex && xSemaphoreTake(s_statusMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s_statusPayload = json;
    xSemaphoreGive(s_statusMutex);
  } else {
    s_statusPayload = json;  // fallback before webserver_begin()
  }
  if (!webserver_pauseSSE) {
    events.send(json.c_str(), "status", millis());
  }
}

void webserver_broadcastPowerFast(int powerdraw, int powerToConsume, int powerDrawAge) {
  // Stack-only, no heap, no JSON library — absolute minimum latency.
  char buf[96];
  int len = snprintf(buf, sizeof(buf),
    "{\"Powerdraw\":%d,\"powerToConsume\":%d,\"powerDrawAge\":%d}",
    powerdraw, powerToConsume, powerDrawAge);
  if (len > 0 && len < (int)sizeof(buf) && !webserver_pauseSSE) {
    events.send(buf, "status", millis());
  }
}

void webserver_loop() {
  if (rebootPending && millis() >= rebootAt) {
    ESP.restart();
  }
  // Periodically close stale SSE connections to free sockets.
  static unsigned long lastSSECleanup = 0;
  if (millis() - lastSSECleanup >= SSE_CLEANUP_MS) {
    lastSSECleanup = millis();
    sseCleanupStale();
  }
}
