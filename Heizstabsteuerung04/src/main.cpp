#include <Arduino.h>          // Grundlegende Arduino-Funktionen
#include <WiFi.h>             // WiFi für ESP32
#include <WiFiUdp.h>          // UDP für aktiven NTP-Konnektivitätstest
#include <ESPmDNS.h>          // mDNS für .local Namensauflösung
#include <ArduinoJson.h>      // Für JSON-Verarbeitung (effizienter als Arduino_JSON)
#include <DFRobot_GP8XXX.h>   // Für die DAC-Steuerung
#include <ArduinoOTA.h>       // Für Over-The-Air Updates
#include <Adafruit_MQTT.h>    // MQTT Grundfunktionen
#include <Adafruit_MQTT_Client.h>  // MQTT Client
#include "InterpolationLib.h"
#include <Ticker.h>           // For software watchdog
#include <esp_task_wdt.h>     // Hardware-backed task watchdog (timer group peripheral)
#include "secrets.h"          // WiFi/MQTT credentials (gitignored)
#include "webserver.h"        // HTTP + SSE web UI
#include "config_store.h"     // Persistent settings (NVS Preferences)
#include "history.h"          // Power history ring buffer (dedicated NVS partition)
#include "energy.h"           // Heizstab Energy integrator (Wh, monthly chunks)
#include "temp_sensors.h"     // DS18B20 OneWire temp sensors (Boiler + Heizstab-Zulauf)
#include "pid_controller.h"   // Reiner PID auf DAC mit Online-Adaption + Relay-Autotune
#include <LittleFS.h>         // Web UI assets
#include <time.h>             // NTP-based time for history timestamps

// ---- Watchdogs (defense-in-depth) ------------------------------------------
// 1) Hardware-backed esp_task_wdt: timer group peripheral + interrupt. Fires
//    even if FreeRTOS tasks starve (lwIP/WiFi deadlock, critical section
//    forgotten, etc.). On timeout: panic + stack trace + reboot.
// 2) Software Ticker WDT (runs in esp_timer task): redundant secondary; only
//    useful if HW WDT subscription somehow fails. Kept for belt-and-suspenders.
Ticker watchdogTimer;
static volatile unsigned long s_lastLoopMs = 0;
static unsigned long s_heartbeatCounter = 0;
#define WATCHDOG_TIMEOUT_MS 45000  // 45 seconds (software ticker, generous)
#define HW_WDT_TIMEOUT_S    30     // hardware WDT: generous enough that a slow
                                   // mqtt.connect() (TCP handshake + CONNECT
                                   // packet, up to ~15 s) does not trip it.

// Forward-Deklarationen
void failsafe_off();
void MQTT_connect();
void sendupdate(bool force = false);
void applyDAC(int value, unsigned long now);

const int numValues = 17;
double wattValues[17] = { 0, 0, 4, 7, 35, 55, 61, 97, 186, 356, 526, 978, 1241, 1306, 1330, 1352, 1367};
double daccommandValues[17] = { 7000,  8000,  8750,  9000,  10000,  10400,  10500,  11000, 12000, 13500, 15000, 20000, 25000, 26800, 28000, 30000, 31000 };

int daccommandValueinterpolated;
int wattneeded;

// Removed timing-based updates for instant MQTT reaction

// Pin-Modi setzen
DFRobot_GP8413 GP8413(0x58); //Standard-Adresse mit allen DIP-Schaltern OFF

int powerdrawnumber;
int powerdrawsetpoint = 0;
int powerToConsume = 0;  // Tracks how much power we want to consume

// External sensor value (from MQTT)
int solarAcPowerValue = 0;

// Zero feed-in control parameters
int ZERO_FEED_IN_TARGET = 0;        // Target feed-in power (0W = no feed-in)
int MAX_HEATING_POWER = 1367;        // Maximum power of heating element in watts
int MIN_POWER_THRESHOLD = 5;         // Expected lower end of useful solar excess (W)
int DEADBAND = 20;                   // Deadband in watts to prevent oscillation (default: 20W)
int POWER_CHANGE_THRESHOLD = 5;      // Minimum power change to trigger DAC update (default: 5W)
int MAX_BOILER_TEMP_C = 65;          // Temperature limit for boiler (0 = disabled)
int MAX_HEATER_ROD_TEMP_C = 75;      // Temperature limit for heater rod (0 = disabled)
const int POWERDRAW_MIN_VALID = -5000;  // Minimum valid powerdraw value
const int POWERDRAW_MAX_VALID = 5000;   // Maximum valid powerdraw value

// Meter freshness / control loop tuning
unsigned long lastPowerDrawUpdate = 0;
const unsigned long POWERDRAW_STALE_MS = 10000;       // Reading older than 10s is stale
const unsigned long DAC_SETTLE_TIME_MS = 2000;        // Wait this long after DAC change before reacting to new readings
float correctionGain = 0.8f;                          // PI proportional gain (0..1). 0.8 = converge in ~1-2 iterations
bool pumpmanualpower = false;
bool pumpautocontrolled = false;
bool regulating_power = true;
bool heating = false;
int DACoutput;
// Reglerwahl: "classic" = Tabelle + Watt-PI (default, kein Bruch), "pid" = reiner PID auf DAC.
String controllerMode = "classic";

// Pin-Definitionen (ESP32 DevKit v1)
#define STATUS_LED 2          // GPIO2 = on-board blue LED. ESP32 LED is active-HIGH (HIGH = on).
#define STATUS_LED_ON  HIGH
#define STATUS_LED_OFF LOW
const int PUMP_PIN = 23;      // GPIO23 - free pin, drives pump relay
int ONEWIRE_PIN = 15;         // Default GPIO15 for DS18B20; adjustable via settings
// I2C uses default Wire pins: SDA=GPIO21, SCL=GPIO22 (handled by DFRobot_GP8XXX)

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish statusfeed = Adafruit_MQTT_Publish(&mqtt, "heizstabsteuerung/tele/status");
Adafruit_MQTT_Subscribe powerdraw = Adafruit_MQTT_Subscribe(&mqtt, "heizstabsteuerung/command/powerdraw");
Adafruit_MQTT_Subscribe regulate = Adafruit_MQTT_Subscribe(&mqtt, "heizstabsteuerung/command/regulate");
Adafruit_MQTT_Subscribe powerdrawsetpointcommand = Adafruit_MQTT_Subscribe(&mqtt, "heizstabsteuerung/command/powerdrawsetpoint");
Adafruit_MQTT_Subscribe runpump = Adafruit_MQTT_Subscribe(&mqtt, "heizstabsteuerung/command/runpump");

// MQTT-Subscriber für konfigurierbare Parameter (statisch, kein Heap)
Adafruit_MQTT_Subscribe zeroFeedTarget(&mqtt, "heizstabsteuerung/command/zero_feed_target");
Adafruit_MQTT_Subscribe maxHeatingPower(&mqtt, "heizstabsteuerung/command/max_heating_power");
Adafruit_MQTT_Subscribe minPowerThreshold(&mqtt, "heizstabsteuerung/command/min_power_threshold");
Adafruit_MQTT_Subscribe deadband(&mqtt, "heizstabsteuerung/command/deadband");
Adafruit_MQTT_Subscribe pumpMinRuntime(&mqtt, "heizstabsteuerung/command/pump_min_runtime");
Adafruit_MQTT_Subscribe powerChangeThreshold(&mqtt, "heizstabsteuerung/command/power_change_threshold");
Adafruit_MQTT_Subscribe correctionGainTopic(&mqtt, "heizstabsteuerung/command/correction_gain");
Adafruit_MQTT_Subscribe pumpCycleInterval(&mqtt, "heizstabsteuerung/command/pump_cycle_interval");
Adafruit_MQTT_Subscribe pumpCycleDuration(&mqtt, "heizstabsteuerung/command/pump_cycle_duration");

// External sensor subscription (single value)
Adafruit_MQTT_Subscribe solarAcPower(&mqtt, "solar/ac/power");

// Pump control
// Nachlauf (coast-down) nach Heizungs-Abschaltung, damit Restwärme ausgespült wird.
unsigned long PUMP_MIN_RUNTIME_MS = 30000; // 30 s default
unsigned long heatingStoppedAt    = 0;     // millis() der letzten heating: true->false Flanke (0 = aktiv/abgelaufen)

// Periodischer Zirkulations-Zyklus (unabhängig von Heizung / Regelung).
// Pumpe läuft alle PUMP_CYCLE_INTERVAL_MIN Minuten für PUMP_CYCLE_DURATION_SEC Sekunden.
// Intervall = 0 deaktiviert den Zyklus.
unsigned long PUMP_CYCLE_INTERVAL_MIN = 360;   // 6 h default
unsigned long PUMP_CYCLE_DURATION_SEC = 60;    // 1 min default
unsigned long lastPumpCycleTime  = 0;
unsigned long cyclePumpStart     = 0;
bool          cyclePumpActive    = false;

// Pump temperature condition: auto-run during heating only if heater-rod > boiler.
bool  PUMP_TEMP_COND_ENABLED = true;
// Hysteresis: once pump is on (heater rod > boiler), keep it on until heater rod
// drops PUMP_TEMP_HYST_C below boiler.  Prevents short cycling around equality.
float PUMP_TEMP_HYST_C       = 2.0f;

// Volatility filter for PV power: smooths rapid oscillations.
bool VOL_ENABLED      = false;
int  VOL_WINDOW_MIN   = 5;    // rolling-average window in minutes
int  VOL_THRESHOLD_W  = 400;  // W: max-min range above which PV is considered volatile

// Volatility ring buffer (one 10-s sample per slot, max 96 slots = 16 min)
static constexpr uint8_t  VOL_BUF_SIZE = 96;
struct VolEntry { uint32_t ts_ms; int16_t val; };
static VolEntry       s_volBuf[VOL_BUF_SIZE];
static uint8_t        s_volHead  = 0;
static uint8_t        s_volCount = 0;
static unsigned long  s_lastVolSampleMs = 0;
static int            s_smoothedPowerdraw = 0;
static int            s_powerRange        = 0;
static bool           s_volatileActive    = false;

// Heartbeat: force a publish every N ms even if nothing changed (liveness signal)
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 2000; // 2s heartbeat (matches Poolcontroller10)

// Rate limit MQTT telemetry publish (SSE is always instant)
unsigned long lastMqttPublishTime = 0;
const unsigned long MQTT_PUBLISH_INTERVAL = 5000; // 5s between MQTT publishes (SSE is unaffected)

// Rate limiting for DAC updates
unsigned long lastDACUpdateTime = 0;
const unsigned long DAC_UPDATE_INTERVAL = 500; // 500ms minimum between DAC updates

// Calculation backoff mechanism (from OpenDTU)
unsigned long lastCalculationTime = 0;
unsigned long calculationBackoffMs = 100; // Start with 100ms backoff
const unsigned long CALCULATION_BACKOFF_MAX = 1024; // Maximum backoff (1 second)
const unsigned long CALCULATION_BACKOFF_DEFAULT = 100; // Default backoff when system changes

// CRC32-based dedupe: avoids keeping a full ~1.5 KB JSON String on the heap
// permanently (heap fragmentation killer). Only 4 bytes instead of ~1500.
static uint32_t lastJsonCRC = 0;
static uint32_t crc32(const char *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint8_t)data[i];
    for (int j = 0; j < 8; j++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

// Adaptive correction: learn the actual DAC-to-Watt relationship
// Tracks cumulative error to adjust future interpolation
float dacCorrectionFactor = 1.0f;  // Multiplier for interpolated watt values (1.0 = no correction)
unsigned long lastCorrectionUpdate = 0;
const unsigned long CORRECTION_UPDATE_INTERVAL = 30000;  // Update every 30 seconds when stable

void sendupdate(bool force)
{
  JsonDocument doc;
  doc["Powerdraw"] = powerdrawnumber;
  doc["powerdrawsetpoint"] = powerdrawsetpoint;
  doc["wattneeded"] = wattneeded;
  doc["DACOutput"] = DACoutput;
  doc["pumpmanualpower"] = pumpmanualpower;
  doc["pumpautocontrolled"] = pumpautocontrolled;
  doc["Regulating"] = regulating_power;
  doc["powerToConsume"] = powerToConsume;
  doc["zeroFeedTarget"] = ZERO_FEED_IN_TARGET;
  doc["maxHeatingPower"] = MAX_HEATING_POWER;
  doc["minPowerThreshold"] = MIN_POWER_THRESHOLD;
  doc["deadband"] = DEADBAND;
  doc["currentDAC"] = DACoutput;
  doc["currentWatt"] = wattneeded;
  doc["powerChangeThreshold"] = POWER_CHANGE_THRESHOLD;
  doc["maxBoilerTemp"] = MAX_BOILER_TEMP_C;
  doc["maxHeaterRodTemp"] = MAX_HEATER_ROD_TEMP_C;
  doc["pumpMinRuntime"] = (int)(PUMP_MIN_RUNTIME_MS / 1000);
  doc["pumpCycleInterval"] = (int)PUMP_CYCLE_INTERVAL_MIN;   // minutes
  doc["pumpCycleDuration"] = (int)PUMP_CYCLE_DURATION_SEC;   // seconds
  doc["pumpCycleActive"]   = cyclePumpActive;
  doc["heating"] = heating;
  doc["correctionGain"] = correctionGain;
  doc["powerDrawAge"] = lastPowerDrawUpdate ? (int)((millis() - lastPowerDrawUpdate) / 1000) : -1;
  doc["ipAddress"] = WiFi.localIP().toString();
  doc["wifiSSID"] = WiFi.SSID();
  int uptimeSec = (int)(millis() / 1000);
  doc["uptime"] = uptimeSec;
  doc["uptimeDays"] = uptimeSec / 86400;
  doc["uptimeWeeks"] = uptimeSec / 604800;
  doc["heap"] = ESP.getFreeHeap();
  doc["rssi"] = WiFi.RSSI();
  doc["mqttConnected"] = mqtt.connected();
  Energy::fillStatus(doc.as<JsonVariant>());
  doc["dacCorrectionFactor"] = dacCorrectionFactor;
  doc["solarAcPower"] = solarAcPowerValue;

  // NTP sync status
  time_t now;
  time(&now);
  doc["ntpSynced"] = (now > 1700000000);  // NTP synced if epoch > 2023-11-15

  // Hardware config
  doc["onewirePin"] = ONEWIRE_PIN;


  // DS18B20 Temperaturen (ungültig -> null im JSON)
  float tb = TempSensors::getBoilerC();
  float ti = TempSensors::getInletC();
  float to = TempSensors::getOutletC();
  float th = TempSensors::getHeaterRodC();
  auto validT = [](float v) { return v > -50.0f && v < 150.0f; };
  auto rnd1   = [](float v) { return roundf(v * 10.0f) / 10.0f; };
  if (!validT(tb)) doc["t_boiler"] = nullptr; else doc["t_boiler"] = rnd1(tb);
  if (!validT(ti)) doc["t_inlet"]  = nullptr; else doc["t_inlet"]  = rnd1(ti);
  if (!validT(to)) doc["t_outlet"] = nullptr; else doc["t_outlet"] = rnd1(to);
  if (!validT(th)) doc["t_hrod"]   = nullptr; else doc["t_hrod"]   = rnd1(th);

  doc["temp_sensor_count"] = TempSensors::sensorCount();

  // Pump cycle countdown (-1 = disabled, 0 = active, >0 = seconds remaining)
  {
    long ncs = -1;
    unsigned long nowMs = millis();
    if (PUMP_CYCLE_INTERVAL_MIN > 0) {
      if (cyclePumpActive) { ncs = 0; }
      else if (lastPumpCycleTime != 0) {
        unsigned long iMs = PUMP_CYCLE_INTERVAL_MIN * 60UL * 1000UL;
        unsigned long el  = nowMs - lastPumpCycleTime;
        ncs = (long)((iMs > el) ? (iMs - el) / 1000UL : 0);
      }
    }
    doc["nextCycleSec"] = ncs;
  }

  // Volatility filter status
  doc["pumpTempCondEnabled"] = PUMP_TEMP_COND_ENABLED;
  doc["pumpTempHystC"]       = PUMP_TEMP_HYST_C;
  doc["volEnabled"]          = VOL_ENABLED;
  doc["volActive"]           = s_volatileActive;
  doc["smoothedPowerdraw"]   = s_smoothedPowerdraw;
  doc["powerRange"]          = s_powerRange;
  doc["volWindowMin"]        = VOL_WINDOW_MIN;
  doc["volThresholdW"]       = VOL_THRESHOLD_W;

  // PID-Regler Status (auch im Classic-Mode für UI-Toggle erkennbar)
  doc["controllerMode"]  = controllerMode;
  doc["pidKp"]           = PidController::kp();
  doc["pidKi"]           = PidController::ki();
  doc["pidSolarFF"]      = PidController::solarFf();
  doc["pidIntegrator"]   = PidController::integrator();
  doc["pidLastGoodDac"]  = PidController::lastGoodDac();
  doc["pidOnlineAdapt"]  = PidController::onlineAdaptEnabled();
  doc["pidLastAdapt"]    = PidController::lastAdaptEpoch();
  doc["autotuneState"]   = PidController::autotuneStateStr();
  doc["autotuneProgress"]= PidController::autotuneProgressPercent();
  doc["autotuneTimestamp"] = PidController::autotuneTimestamp();

  // Recent error samples for UI mini-chart (via SSE, no separate polling needed)
  JsonArray errArray = doc["recent_errors"].to<JsonArray>();
  int16_t errBuf[60];
  size_t errCount = PidController::getRecentErrors(errBuf, 60);
  for (size_t i = 0; i < errCount; i++) {
    errArray.add((int)errBuf[i]);
  }

  String jsonString;
  serializeJson(doc, jsonString);

  // Skip publish if payload identical to previous one (CRC32 check, unless heartbeat forces it)
  uint32_t jsonCRC = crc32(jsonString.c_str(), jsonString.length());
  if (!force && jsonCRC == lastJsonCRC) { return; }
  lastJsonCRC = jsonCRC;
  lastTelemetryTime = millis();  // any publish counts as a heartbeat
  webserver_broadcastStatus(jsonString);  // SSE push FIRST for minimal UI latency
  // Rate-limit MQTT publish to avoid blocking the loop on every SSE update
  if (millis() - lastMqttPublishTime >= MQTT_PUBLISH_INTERVAL) {
    statusfeed.publish(jsonString.c_str());
    lastMqttPublishTime = millis();
  }
}

void MQTT_connect() {
  static bool wasConnected = false;
  
  if (mqtt.connected()) {
    if (!wasConnected) {
      digitalWrite(STATUS_LED, STATUS_LED_ON);  // Turn on LED when connected
      wasConnected = true;
    }
    return;
  }
  
  // Warte auf WiFi-Verbindung (non-blocking)
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    return; // let loop handle it next iteration
  }
  // mqtt.connect() can block 5-15 s on broker outage (TCP handshake +
  // CONNECT packet timeout). Feed both watchdogs immediately before the
  // call so the timer doesn't trip mid-connect.
  s_lastLoopMs = millis();
  esp_task_wdt_reset();
  uint8_t retries = 3;
  while (mqtt.connect() != 0) { // connect will return 0 for connected
    mqtt.disconnect();
    digitalWrite(STATUS_LED, STATUS_LED_OFF);  // Turn off LED when disconnected
    wasConnected = false;
    retries--;
    if (retries == 0) {
      failsafe_off();
      return;
    }
    // Non-blocking: return and retry next loop iteration
    return;
  }
  digitalWrite(STATUS_LED, STATUS_LED_ON); // Turn on LED when connected
  wasConnected = true;
}

// Update volatility ring buffer and compute effective powerdraw.
// Call once per loop iteration; sampling is rate-limited to 10 s.
static void tickVolatility(int rawPD, unsigned long nowMs) {
  if (nowMs - s_lastVolSampleMs >= 10000UL) {
    s_lastVolSampleMs = nowMs;
    s_volBuf[s_volHead] = { (uint32_t)nowMs, (int16_t)constrain(rawPD, -32767, 32767) };
    s_volHead = (s_volHead + 1) % VOL_BUF_SIZE;
    if (s_volCount < VOL_BUF_SIZE) s_volCount++;
  }
  if (s_volCount == 0) {
    s_smoothedPowerdraw = rawPD; s_powerRange = 0; s_volatileActive = false; return;
  }
  const uint32_t windowMs = (uint32_t)VOL_WINDOW_MIN * 60UL * 1000UL;
  long sum = 0; int cnt = 0, mn = 32767, mx = -32768;
  for (uint8_t i = 0; i < s_volCount; i++) {
    uint8_t idx = (uint8_t)((s_volHead + VOL_BUF_SIZE - s_volCount + i) % VOL_BUF_SIZE);
    if ((uint32_t)nowMs - s_volBuf[idx].ts_ms <= windowMs) {
      sum += s_volBuf[idx].val; cnt++;
      if (s_volBuf[idx].val < mn) mn = s_volBuf[idx].val;
      if (s_volBuf[idx].val > mx) mx = s_volBuf[idx].val;
    }
  }
  if (cnt == 0) { s_smoothedPowerdraw = rawPD; s_powerRange = 0; s_volatileActive = false; return; }
  s_smoothedPowerdraw = (int)(sum / cnt);
  s_powerRange        = mx - mn;
  s_volatileActive    = VOL_ENABLED && (s_powerRange > VOL_THRESHOLD_W);
}

void failsafe_off() {
    DACoutput = 0;
    wattneeded = 0;
    daccommandValueinterpolated = 0;
    heating = false;
    pumpautocontrolled = false;
    // pumpmanualpower intentionally NOT reset: manual on/off via button/MQTT survives MQTT loss
    heatingStoppedAt = 0;       // Nachlauf unterdrücken
    cyclePumpActive = false;    // Zyklus abbrechen
    GP8413.setDACOutVoltage(0, 0);
    digitalWrite(PUMP_PIN, pumpmanualpower ? HIGH : LOW);  // respect manual state
}

// Helper: write DAC with rate limiting; retries on next call if blocked.
int pendingDACoutput = -1;
void applyDAC(int value, unsigned long now) {
  pendingDACoutput = value;
  if (now - lastDACUpdateTime >= DAC_UPDATE_INTERVAL) {
    GP8413.setDACOutVoltage(pendingDACoutput, 0);
    lastDACUpdateTime = now;
    pendingDACoutput = -1;
  }
}

// Watchdog callback: runs in esp_timer task (not ISR), checks loop timestamp
static void checkWatchdog() {
  if (millis() - s_lastLoopMs > WATCHDOG_TIMEOUT_MS) {
    ESP.restart();
  }
}

// ---- Active WiFi connectivity check ----------------------------------------
// WiFi.status() can report WL_CONNECTED while the link is actually dead (the
// ESP32 WiFi stack occasionally stalls without dropping the association). To
// catch this we send a real NTP request over UDP every 60 s and wait briefly
// for a reply. Two consecutive failures => force a full WiFi reconnect.
static const unsigned long WIFI_HEALTH_INTERVAL_MS = 60000UL;
static unsigned long s_lastWifiHealthMs = 0;
static uint8_t       s_wifiHealthFails  = 0;

static bool ntpProbe() {
  WiFiUDP udp;
  IPAddress ntpIp;
  if (!WiFi.hostByName("pool.ntp.org", ntpIp)) return false;  // DNS itself tests the stack
  if (!udp.begin(2390)) return false;

  uint8_t pkt[48];
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0b11100011;  // LI=3 (unsync), VN=4, Mode=3 (client)

  if (udp.beginPacket(ntpIp, 123) == 0) { udp.stop(); return false; }
  udp.write(pkt, sizeof(pkt));
  if (udp.endPacket() == 0) { udp.stop(); return false; }

  // Wait up to ~1 s for a reply (feed soft WDT meanwhile).
  unsigned long start = millis();
  while (millis() - start < 1000UL) {
    if (udp.parsePacket() >= 48) { udp.stop(); return true; }
    s_lastLoopMs = millis();
    delay(10);
  }
  udp.stop();
  return false;
}

static void wifiHealthCheck() {
  unsigned long nowMs = millis();
  if (nowMs - s_lastWifiHealthMs < WIFI_HEALTH_INTERVAL_MS) return;
  s_lastWifiHealthMs = nowMs;

  if (ntpProbe()) {
    s_wifiHealthFails = 0;
    return;
  }

  s_wifiHealthFails++;
  Serial.printf("HB: WiFi health probe failed (%u)\n", s_wifiHealthFails);
  if (s_wifiHealthFails >= 2) {
    Serial.println("HB: WiFi stalled - forcing reconnect");
    webLog("[WiFi] Stall detected via NTP probe - reconnecting");
    WiFi.disconnect();
    delay(100);
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    s_wifiHealthFails = 0;
  }
}

void setup() {
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, STATUS_LED_OFF); // Start with LED off

  // Setup software watchdog timer: check every 1s if loop() is still alive
  watchdogTimer.attach(1.0, checkWatchdog);

  // Initialise the HW task watchdog now, but DO NOT subscribe loopTask yet:
  // setup() below contains a blocking WiFi-wait loop that can legitimately
  // take longer than HW_WDT_TIMEOUT_S during a slow router boot. We subscribe
  // loopTask at the END of setup(), once everything is ready. panic=true
  // ensures a stack trace + reboot on real loop hangs.
  esp_task_wdt_init(HW_WDT_TIMEOUT_S, true);

  Serial.begin(115200);
  delay(200);
  webLog("[Boot] Starting Heizstabsteuerung...");

  // Load persisted settings from NVS FIRST so ONEWIRE_PIN reflects user choice
  // before we initialise the OneWire bus.
  ConfigStore::load();
  webLog("[Boot] ONEWIRE_PIN from NVS = GPIO%d", ONEWIRE_PIN);

  // Initialise DS18B20 OneWire bus BEFORE WiFi starts. WiFi interrupts can
  // disrupt OneWire's bit-bang timing during the initial bus scan, causing
  // sensors to be missed. Roles (ROM-mapping) loaded from NVS.
  TempSensors::begin(ONEWIRE_PIN);  // adjustable via settings

  // ESP32 quirks:
  //   1. setHostname() must be called on STA_START to be honoured by DHCP.
  //   2. mDNS becomes unresponsive after every WiFi reconnect → re-init on GOT_IP.
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_START) {
      WiFi.setHostname("Heizstabsteuerung");
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      MDNS.end();
      if (MDNS.begin("Heizstabsteuerung")) {
        MDNS.setInstanceName("Heizstabsteuerung");
        MDNS.addService("http", "tcp", 80);
      }
    }
  });
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("Heizstabsteuerung");  // belt-and-suspenders
  WiFi.setSleep(false);  // disable WiFi power saving for OTA stability
  WiFi.setAutoReconnect(true);  // auto-reconnect on WiFi drop
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // max power for stability
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    s_lastLoopMs = millis();  // feed soft WDT during long WiFi-wait
  }

  // Start webserver EARLY so browser can see logs during boot
  webserver_begin();
  webLog("[Boot] WiFi connected, webserver started");

  // (mDNS is started in the WiFi GOT_IP event handler above for reliability.)

  mqtt.subscribe(&powerdraw);
  mqtt.subscribe(&runpump);
  mqtt.subscribe(&regulate);
  mqtt.subscribe(&powerdrawsetpointcommand);
  mqtt.subscribe(&zeroFeedTarget);
  mqtt.subscribe(&maxHeatingPower);
  mqtt.subscribe(&minPowerThreshold);
  mqtt.subscribe(&deadband);
  mqtt.subscribe(&pumpMinRuntime);
  mqtt.subscribe(&powerChangeThreshold);
  mqtt.subscribe(&correctionGainTopic);
  mqtt.subscribe(&pumpCycleInterval);
  mqtt.subscribe(&pumpCycleDuration);
  mqtt.subscribe(&solarAcPower);
  
  ArduinoOTA.setHostname("Heizstabsteuerung");
  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Start");
    webserver_pauseSSE = true;  // stop SSE traffic during upload
    mqtt.disconnect();          // disconnect MQTT to reduce WiFi load
    watchdogTimer.detach();     // disable software watchdog during OTA
    esp_task_wdt_delete(NULL);  // unsubscribe loopTask from HW WDT during OTA
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] End");
    webserver_pauseSSE = false;  // resume SSE
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Feed watchdog every ~32KB to prevent timeout during long upload
    static unsigned int lastFeed = 0;
    if (progress - lastFeed >= 32768) {  // every 32KB
      s_lastLoopMs = millis();
      lastFeed = progress;
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    webserver_pauseSSE = false;  // resume SSE on error
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    else Serial.println("Unknown");
  });
  ArduinoOTA.begin();

  // Mount LittleFS (web UI). format-on-fail so first boot after partition
  // change doesn't brick the UI.
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS mount failed");
  }

  // (ConfigStore::load() already called early, before TempSensors::begin)
  lastPumpCycleTime = millis();  // anchor pump cycle timer at boot (first cycle after full interval)

  // Initialise history ring buffer (loads from dedicated NVS partition).
  History::begin();
  Energy::begin();

  // PID-Regler-State aus NVS laden (auch wenn nicht aktiv ist es harmlos).
  PidController::begin();

  // NTP time for history timestamps. Europe/Berlin with DST handling.
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");

  webLog("[Boot] NTP configured, DAC init...");
  GP8413.begin();
  GP8413.setDACOutRange(GP8413.eOutputRange10V);
  pinMode(PUMP_PIN, OUTPUT);
  failsafe_off();

  // All slow init done — NOW subscribe loopTask to the HW WDT. From here on
  // loop() must call esp_task_wdt_reset() at least every HW_WDT_TIMEOUT_S.
  esp_task_wdt_add(NULL);
  webLog("[Boot] Setup complete, HW watchdog armed (%ds)", HW_WDT_TIMEOUT_S);
}

void loop() {
  s_lastLoopMs = millis();    // feed software watchdog timestamp
  esp_task_wdt_reset();       // feed hardware task watchdog

  // Heartbeat for freeze diagnosis
  s_heartbeatCounter++;
  if (s_heartbeatCounter % 1000 == 0) {
    Serial.printf("HB: %lu\n", s_heartbeatCounter);
  }

  // Handle OTA FIRST, before any blocking work.  OneWire critical sections
  // (~750 ms with IRQs off) drop OTA WiFi packets and abort the upload.
  ArduinoOTA.handle();

  // Skip ALL blocking work while OTA upload in progress (flag set in onStart).
  extern bool webserver_pauseSSE;
  if (webserver_pauseSSE) {
    delay(1);
    return;
  }

  // Temp sensors are independent of MQTT — always poll.
  // (async OneWire: tick() is non-blocking; conversion runs in background)
  TempSensors::tick();

  // Failsafe if WiFi or MQTT are not connected
  static unsigned long wifiDisconnectTime = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiDisconnectTime == 0) wifiDisconnectTime = millis();
    if (s_heartbeatCounter % 1000 == 0) {
      Serial.printf("HB: WiFi disconnected for %lu ms, SSE clients: %d\n",
                   millis() - wifiDisconnectTime, webserver_getSseClientCount());
    }
    failsafe_off();
    // Rate-limited reconnect attempt (every 2 seconds)
    if (millis() - wifiDisconnectTime > 2000) {
      WiFi.reconnect();
      wifiDisconnectTime = millis();  // reset timer
    }
    // Full WiFi reset after 30 seconds disconnected
    if (millis() - wifiDisconnectTime > 30000) {
      Serial.println("HB: WiFi reset - full begin()");
      WiFi.disconnect();
      delay(100);
      WiFi.begin(WLAN_SSID, WLAN_PASS);
      wifiDisconnectTime = millis();
    }
    return; // non-blocking
  } else {
    wifiDisconnectTime = 0;  // reset when connected
    // Actively verify connectivity (WiFi.status() can lie when the stack stalls)
    wifiHealthCheck();
  }

  // Handle MQTT connection
  if (!mqtt.connected()) {
    if (s_heartbeatCounter % 1000 == 0) {
      Serial.println("HB: MQTT disconnected");
    }
    failsafe_off();
    MQTT_connect();
    return; // non-blocking - retry next iteration
  }

  webserver_loop();  // process pending reboot requests

  if (s_heartbeatCounter % 1000 == 0) {
    Serial.println("HB: after webserver_loop");
  }

  // ---- MQTT: read FIRST, before any blocking I/O (History, etc.) ----
  // This ensures powerdraw updates reach the UI with minimal latency.
  Adafruit_MQTT_Subscribe *subscription = mqtt.readSubscription(0); // non-blocking
  bool needsConfigSave = false;
  if (subscription) {
    if (subscription == &powerdraw) {
      int newPowerDraw = atoi((char *)powerdraw.lastread);
      // Input validation
      if (newPowerDraw >= POWERDRAW_MIN_VALID && newPowerDraw <= POWERDRAW_MAX_VALID) {
        powerdrawnumber = newPowerDraw;
        powerToConsume = ZERO_FEED_IN_TARGET - powerdrawnumber;
        lastPowerDrawUpdate = millis();
        // INSTANT SSE push — no JSON library, no heap, no serialization delay.
        webserver_broadcastPowerFast(powerdrawnumber, powerToConsume, 0);
      }
    }
    if (subscription == &runpump) {
      int tempnummer = atoi((char *)runpump.lastread);
      pumpmanualpower = (tempnummer > 0 && tempnummer < 2);
      sendupdate();
    }
    if (subscription == &regulate) {
      int tempnummer = atoi((char *)regulate.lastread);
      regulating_power = (tempnummer > 0 && tempnummer < 2);
      sendupdate();
    }
    if (subscription == &powerdrawsetpointcommand) {
      powerdrawsetpoint = atoi((char *)powerdrawsetpointcommand.lastread);
    }
    if (subscription == &zeroFeedTarget) {
      int newVal = atoi((char *)zeroFeedTarget.lastread);
      if (newVal >= 0 && newVal <= 10000) {
        ZERO_FEED_IN_TARGET = newVal;
        powerToConsume = ZERO_FEED_IN_TARGET - powerdrawnumber;
      }
    }
    if (subscription == &maxHeatingPower) {
      int newMax = atoi((char *)maxHeatingPower.lastread);
      if (newMax > 0 && newMax <= 2000) {
        MAX_HEATING_POWER = newMax;
      }
    }
    if (subscription == &minPowerThreshold) {
      int newMin = atoi((char *)minPowerThreshold.lastread);
      if (newMin >= 0 && newMin < 100) {
        MIN_POWER_THRESHOLD = newMin;
      }
    }
    if (subscription == &deadband) {
      int newDeadband = atoi((char *)deadband.lastread);
      if (newDeadband >= 0 && newDeadband < 500) {  // Reasonable limits for deadband
        DEADBAND = newDeadband;
      }
    }
    if (subscription == &pumpMinRuntime) {
      long newRuntimeSec = atoi((char *)pumpMinRuntime.lastread);
      if (newRuntimeSec >= 5 && newRuntimeSec <= 300) {  // 5 seconds to 5 minutes
        PUMP_MIN_RUNTIME_MS = (unsigned long)newRuntimeSec * 1000UL;
      }
    }
    if (subscription == &pumpCycleInterval) {
      long n = atoi((char *)pumpCycleInterval.lastread);
      if (n >= 0 && n <= 1440) PUMP_CYCLE_INTERVAL_MIN = (unsigned long)n;   // 0 = aus
    }
    if (subscription == &pumpCycleDuration) {
      long n = atoi((char *)pumpCycleDuration.lastread);
      if (n >= 0 && n <= 3600) PUMP_CYCLE_DURATION_SEC = (unsigned long)n;
    }
    if (subscription == &powerChangeThreshold) {
      int newThreshold = atoi((char *)powerChangeThreshold.lastread);
      if (newThreshold >= 1 && newThreshold <= 100) {  // 1W to 100W
        POWER_CHANGE_THRESHOLD = newThreshold;
      }
    }
    if (subscription == &correctionGainTopic) {
      // Sent as integer percent 0..150 (e.g. 80 -> 0.8). Higher = faster but more oscillation.
      int newGainPct = atoi((char *)correctionGainTopic.lastread);
      if (newGainPct >= 1 && newGainPct <= 150) {
        correctionGain = newGainPct / 100.0f;
      }
    }
    // External sensor subscription (single value)
    if (subscription == &solarAcPower) {
      const char *str = (char *)solarAcPower.lastread;
      if (str && str[0] != '\0') {
        int val = atoi(str);
        // Validate: solar power should be non-negative and reasonable
        if (val >= 0 && val <= 10000) {
          solarAcPowerValue = val;
        }
      }
    }
    // Only persist to NVS when an actual config parameter changed (not for
    // high-frequency sensor data like powerdraw, solarAcPower, setpoint).
    if (subscription != &powerdraw &&
        subscription != &powerdrawsetpointcommand &&
        subscription != &solarAcPower) {
      needsConfigSave = true;
    }
    // For non-powerdraw messages, send full state update
    if (subscription != &powerdraw) {
      sendupdate();
    }
  }

  // Deferred NVS save (outside the time-critical MQTT handler)
  if (needsConfigSave) ConfigStore::save();

  // History: cheap no-ops most of the time (interval-gated internally).
  History::tickSample(powerdrawnumber, wattneeded, solarAcPowerValue,
                      TempSensors::getBoilerC(), TempSensors::getInletC(),
                      NAN, TempSensors::getOutletC(), TempSensors::getHeaterRodC());
  History::tickSave();

  // Heizstab-Energie: feingranular integrieren (Wh), monatliche Chunks.
  Energy::tick(heating ? wattneeded : 0);
  Energy::tickSave();

  // Process MQTT keepalive (rate-limited to avoid unnecessary blocking)
  static unsigned long lastMqttProcess = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastMqttProcess >= 100) {
    mqtt.processPackets(1);
    lastMqttProcess = currentTime;
  }
  // Heartbeat: force-publish full status every TELEMETRY_INTERVAL ms.
  // This keeps all UI fields (temps, PID, pump status, etc.) in sync.
  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    sendupdate(true);  // force, bypass dedupe cache
  }
  
  // Skip regulation entirely if the meter reading is stale (sensor offline / MQTT broker dropped messages)
  // NOTE: we no longer return early here — instead we set a flag and skip only
  // the regulation logic.  The pump-control block and SSE heartbeat still run
  // so the UI never sees stale values.
  bool skipRegulationStale = false;
  if (lastPowerDrawUpdate == 0 || (currentTime - lastPowerDrawUpdate) > POWERDRAW_STALE_MS) {
    if (heating) {
      DACoutput = 0;
      wattneeded = 0;
      daccommandValueinterpolated = 0;
      applyDAC(0, currentTime);
      heating = false;
      sendupdate();  // notify subscribers about the failsafe shutdown
    }
    skipRegulationStale = true;
  }

  // Wait after a DAC change so the meter reading reflects the new heater consumption
  // (prevents oscillation from acting on stale readings)
  bool skipRegulationSettle = (!skipRegulationStale && (currentTime - lastDACUpdateTime) < DAC_SETTLE_TIME_MS);

  // Track Solar-AC power in the volatility filter (detect unstable PV output).
  tickVolatility(solarAcPowerValue, currentTime);

  // Volatility filter: shut down heater immediately if it is running while PV is volatile.
  if (s_volatileActive && heating) {
    DACoutput = 0;
    wattneeded = 0;
    daccommandValueinterpolated = 0;
    applyDAC(0, currentTime);
    heating = false;
    sendupdate();
  }

  // Temperature limit: hard shutdown if boiler or heater rod exceeds max temp
  if (heating) {
    float tBoil = TempSensors::getBoilerC();
    float tHrod = TempSensors::getHeaterRodC();
    bool overTemp = false;
    if (MAX_BOILER_TEMP_C > 0 && tBoil > -50.0f && tBoil < 150.0f && tBoil >= MAX_BOILER_TEMP_C) overTemp = true;
    if (MAX_HEATER_ROD_TEMP_C > 0 && tHrod > -50.0f && tHrod < 150.0f && tHrod >= MAX_HEATER_ROD_TEMP_C) overTemp = true;
    if (overTemp) {
      DACoutput = 0;
      wattneeded = 0;
      daccommandValueinterpolated = 0;
      applyDAC(0, currentTime);
      heating = false;
      sendupdate();
    }
  }

  // Regulation always uses raw grid draw (smoothed solar != grid draw).
  int currentPowerDraw = powerdrawnumber;
  powerToConsume = ZERO_FEED_IN_TARGET - currentPowerDraw;
  
  // Negative power protection: if we're importing from the grid AND the heater is
  // currently OFF, just stay off (no point starting up just to overshoot).
  // If the heater is ON, fall through to the PI controller below which will
  // reduce wattneeded gradually. Forcing OFF here would cause pulsing because
  // the heater would jump from 0 to 500W next iteration (and back).
  bool skipRegulation = skipRegulationStale || skipRegulationSettle;
  if (!skipRegulation && powerToConsume < 0 && !heating) {
    calculationBackoffMs = min(calculationBackoffMs * 2, CALCULATION_BACKOFF_MAX);
    lastCalculationTime = currentTime;
    skipRegulation = true;
  }

  // Calculation backoff mechanism (from OpenDTU)
  if (!skipRegulation && (currentTime - lastCalculationTime) < calculationBackoffMs) {
    skipRegulation = true;
  }

  // Retry pending DAC write if rate-limit had blocked it earlier
  if (pendingDACoutput >= 0 && (currentTime - lastDACUpdateTime) >= DAC_UPDATE_INTERVAL) {
    GP8413.setDACOutVoltage(pendingDACoutput, 0);
    lastDACUpdateTime = currentTime;
    pendingDACoutput = -1;
  }

  // Regulation disabled: force heater off (DAC=0) and release the pump to
  // manual control. Without this, switching `regulating_power` off mid-cycle
  // would freeze DAC + heating state at their last values.
  // NOTE: We do NOT set pumpautocontrolled=false here because coast-down and
  // pump cycle should still work even when regulation is OFF.
  if (!regulating_power && (heating || DACoutput != 0)) {
    DACoutput = 0;
    wattneeded = 0;
    daccommandValueinterpolated = 0;
    applyDAC(0, currentTime);
    heating = false;
    sendupdate();
  }

  // Always feed the PID error stat buffer so the UI mini-chart has data
  // (also in classic mode). recordSample() is rate-limited internally to 1Hz.
  PidController::recordSample(powerToConsume, currentTime);

  // -------- Reglerwahl: PID übernimmt die komplette Regelung -------------
  if (controllerMode == "pid") {
    PidController::regulate(currentTime);
    PidController::tickAdapt(currentTime);
  } else if (!skipRegulation && regulating_power) {
    // Check if we need to change the heating state
    if (!heating) {
      // Heater is currently off - only turn on if we need more than MIN_POWER_THRESHOLD + DEADBAND
      if (powerToConsume > (MIN_POWER_THRESHOLD + DEADBAND)) {
        // Temperature hysteresis: don't turn on if close to limit (2°C margin)
        bool tempBlocked = false;
        float tBoil = TempSensors::getBoilerC();
        float tHrod = TempSensors::getHeaterRodC();
        if (MAX_BOILER_TEMP_C > 0 && tBoil > -50.0f && tBoil < 150.0f && tBoil >= MAX_BOILER_TEMP_C - 2) tempBlocked = true;
        if (MAX_HEATER_ROD_TEMP_C > 0 && tHrod > -50.0f && tHrod < 150.0f && tHrod >= MAX_HEATER_ROD_TEMP_C - 2) tempBlocked = true;
        if (tempBlocked) {
          // skip turn-on, but still allow pump control to run below
        } else {
          // Calculate required power, ensuring it's at least MIN_POWER_THRESHOLD
          wattneeded = max(MIN_POWER_THRESHOLD, min(powerToConsume, MAX_HEATING_POWER));
        
        // Calculate DAC value using interpolation
        daccommandValueinterpolated = int(Interpolation::CatmullSpline(
          wattValues, daccommandValues, numValues, wattneeded));
        
        // Ensure value is within valid range
        int maxDAC = (int)daccommandValues[numValues-1];
        DACoutput = min(daccommandValueinterpolated, maxDAC);
        
        // Apply DAC value with rate limiting. Pump state is derived in the
        // unified pump-control block below (single source of truth).
        applyDAC(DACoutput, currentTime);
        heating = true;
        sendupdate();  // event: heater turned on
        
        // Reset calculation backoff when system changes
        calculationBackoffMs = CALCULATION_BACKOFF_DEFAULT;
        lastCalculationTime = currentTime;
        }
      }
    } else {
      // Heater is currently on
      // Closed-loop iterative correction (PI-style):
      // currentPowerDraw represents net grid draw. If it's > target, we need to consume MORE
      // (the heater's interpolation table is slightly off, so the meter tells us the truth).
      // delta = how much additional consumption is needed to reach zero feed-in target
      int delta = powerToConsume; // = TARGET - currentPowerDraw, positive => need more heat
      int newWattNeeded = wattneeded + (int)(delta * correctionGain);
      
      // Slowly regulate down instead of switching off abruptly.
      // If the needed power drops below the minimum threshold, we turn off completely.
      if (newWattNeeded < MIN_POWER_THRESHOLD) {
        newWattNeeded = 0;
      } else {
        newWattNeeded = min(newWattNeeded, MAX_HEATING_POWER);
      }
      
      if (newWattNeeded == 0) {
        // Only switch off when the PI controller regulates down to 0 (below MIN_POWER_THRESHOLD)
        DACoutput = 0;
        daccommandValueinterpolated = 0;
        wattneeded = 0;
        applyDAC(0, currentTime);
        heating = false;
        sendupdate();  // event: heater turned off
        
        calculationBackoffMs = CALCULATION_BACKOFF_DEFAULT;
        lastCalculationTime = currentTime;
      } else {
        if (abs(newWattNeeded - wattneeded) > POWER_CHANGE_THRESHOLD) { // Only update if significant change
          wattneeded = newWattNeeded;
          
          daccommandValueinterpolated = int(Interpolation::CatmullSpline(
            wattValues, daccommandValues, numValues, wattneeded));
          
          int maxDAC = (int)daccommandValues[numValues-1];   
          DACoutput = min(daccommandValueinterpolated, maxDAC);
          
          // Apply DAC value with rate limiting
          applyDAC(DACoutput, currentTime);
          sendupdate();
          
          // Reset calculation backoff when system changes
          calculationBackoffMs = CALCULATION_BACKOFF_DEFAULT;
          lastCalculationTime = currentTime;
        } else {
          // Increase backoff when system is stable (no significant power change)
          calculationBackoffMs = min(calculationBackoffMs * 2, CALCULATION_BACKOFF_MAX);
          lastCalculationTime = currentTime;
          
          // Adaptive correction: learn the actual DAC-to-Watt relationship
          // When system is stable (small power error), adjust correction factor
          if (abs(delta) < 20 && (currentTime - lastCorrectionUpdate) > CORRECTION_UPDATE_INTERVAL) {
            // System is stable and close to target. Adjust correction factor.
            // If delta is small positive, we're slightly over-consuming (interpolation is optimistic)
            // If delta is small negative, we're under-consuming (interpolation is pessimistic)
            float adjustment = 1.0f + (delta / (float)wattneeded * 0.01f);  // 1% adjustment per 1W error
            dacCorrectionFactor = dacCorrectionFactor * 0.95f + adjustment * 0.05f;  // Slow exponential moving average
            dacCorrectionFactor = max(0.8f, min(1.2f, dacCorrectionFactor));  // Clamp to ±20%
            lastCorrectionUpdate = currentTime;
          }
        }
      }
    }
  }
  
  // ================= Unified pump control =================================
  // Pumpe läuft wenn IRGENDEINE Quelle sie anfordert:
  //   (a) Heizung aktiv UND Heizstab wärmer als Speicher               (heating)
  //   (b) Heizstab wärmer als Speicher (unabhängig von Heizung)
  //   (c) Nachlauf nach Heizungs-Abschaltung (Restwärme-Spülung)
  //   (d) Periodischer Zirkulations-Zyklus (alle X min für Y sec)
  //   (e) Manuelle Betätigung (pumpmanualpower)
  // `pumpautocontrolled` = true genau dann, wenn (a)..(d) greift.
  // ------------------------------------------------------------------------
  unsigned long currentMillis = currentTime;

  // Heizung → aus Flanke erkennen (Nachlauf starten).
  static bool prevHeating = false;
  if (prevHeating && !heating) heatingStoppedAt = currentMillis;
  prevHeating = heating;

  // Nachlauf aktiv?
  bool inCoastdown = (heatingStoppedAt != 0 &&
                     (currentMillis - heatingStoppedAt) < PUMP_MIN_RUNTIME_MS);
  if (heatingStoppedAt != 0 && !inCoastdown) heatingStoppedAt = 0;  // abgelaufen

  // Pump temperature condition WITH HYSTERESIS:
  //   ON  when heater rod > boiler           (rod is delivering heat)
  //   OFF when heater rod < boiler - HYST_C  (rod has dropped clearly below boiler)
  //   between → keep previous state (latched)
  // This prevents short-cycling around equality and lets the pump finish
  // recovering remaining heat from the rod after heating stops.
  bool heatingTempMet = true;
  bool tempDiffOn = false;
  if (PUMP_TEMP_COND_ENABLED) {
    static bool s_tempHystOn = false;
    float tHrod = TempSensors::getHeaterRodC();
    float tBoil = TempSensors::getBoilerC();
    if (tHrod > -50.0f && tHrod < 150.0f && tBoil > -50.0f && tBoil < 150.0f) {
      if (tHrod > tBoil) {
        s_tempHystOn = true;
      } else if (tHrod < tBoil - PUMP_TEMP_HYST_C) {
        s_tempHystOn = false;
      }
      heatingTempMet = s_tempHystOn;
      tempDiffOn     = s_tempHystOn;
    }
  }

  // Periodischer Zyklus starten (0 = deaktiviert, unabhängig von Regelung).
  if (PUMP_CYCLE_INTERVAL_MIN > 0) {
    const unsigned long intervalMs = PUMP_CYCLE_INTERVAL_MIN * 60UL * 1000UL;
    if (lastPumpCycleTime == 0) {
      lastPumpCycleTime = currentMillis;  // anchor timer at boot, don't run immediately
    } else if ((currentMillis - lastPumpCycleTime) >= intervalMs) {
      cyclePumpActive   = true;
      cyclePumpStart    = currentMillis;
      lastPumpCycleTime = currentMillis;
    }
  }
  // Laufenden Zyklus beenden, wenn Dauer abgelaufen.
  if (cyclePumpActive) {
    const unsigned long durMs = PUMP_CYCLE_DURATION_SEC * 1000UL;
    if ((currentMillis - cyclePumpStart) >= durMs) cyclePumpActive = false;
  }

  const bool autoOn = (heating && heatingTempMet) || inCoastdown || cyclePumpActive || tempDiffOn;
  pumpautocontrolled = autoOn;   // single source of truth für Status-JSON
  bool pumpShouldBeOn = autoOn || pumpmanualpower;

  // Apply pump state
  digitalWrite(PUMP_PIN, pumpShouldBeOn ? HIGH : LOW);

  // Any pump activity (manual/auto/cycle) resets the cycle timer: next scheduled
  // cycle runs PUMP_CYCLE_INTERVAL_MIN AFTER the last actual run, not on a
  // fixed grid. This avoids unnecessary cycles right after a manual/auto run.
  if (pumpShouldBeOn) {
    lastPumpCycleTime = currentMillis;
  }

  // HTTP handlers set this flag when they mutate state; we drain it once
  // per loop iteration so events.send() is only ever called from the loop
  // task (PoolController pattern — eliminates cross-task _clients races).
  if (webserver_ssePushPending) {
    webserver_ssePushPending = false;
    sendupdate(true);
  }
}
