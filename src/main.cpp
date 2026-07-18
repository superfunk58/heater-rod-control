#include <Arduino.h>          // Grundlegende Arduino-Funktionen
#include <WiFi.h>             // WiFi für ESP32
#include <ESPmDNS.h>          // mDNS für .local Namensauflösung
#include <ArduinoJson.h>      // Für JSON-Verarbeitung (effizienter als Arduino_JSON)
#include <DFRobot_GP8XXX.h>   // Für die DAC-Steuerung
#include <ArduinoOTA.h>       // Für Over-The-Air Updates
#include <PubSubClient.h>     // MQTT Client
#include "InterpolationLib.h"
#include "secrets.h"          // WiFi/MQTT credentials (gitignored)
#include "webserver.h"        // HTTP + SSE web UI
#include "config_store.h"     // Persistent settings (NVS Preferences)
#include "history.h"          // Power history ring buffer (dedicated NVS partition)
#include "energy.h"           // Heizstab Energy integrator (Wh, monthly chunks)
#include "temp_sensors.h"     // DS18B20 OneWire temp sensors (Boiler + Heizstab-Zulauf)
#include "pid_controller.h"   // Reiner PID auf DAC mit Online-Adaption + Relay-Autotune
#include "net_manager.h"      // WiFi/W5500 LAN network management
#include <LittleFS.h>         // Web UI assets
#include <time.h>             // NTP-based time for history timestamps
#include <Preferences.h>      // NVS for persistent reboot counter

// Persistent reboot counter (NVS) + boot wall-clock epoch (set once NTP syncs).
static uint32_t s_rebootCount = 0;
static uint32_t s_bootEpoch   = 0;  // 0 = not yet known (NTP not synced)

void resetRebootCounter() {
  Preferences sys;
  if (sys.begin("sys", /*readOnly*/ false)) {
    sys.putUInt("reboots", 0);
    sys.end();
  }
  s_rebootCount = 0;
}

// MQTT connection check interval: only attempt reconnect every 30s when
// disconnected to avoid thrashing the TCP stack during network flaps.
// Initialize to 0 to allow immediate first connect on boot.
static unsigned long s_lastMqttCheckMs = 0;
static const unsigned long MQTT_CHECK_INTERVAL_MS = 30000;

// Forward-Deklarationen
void failsafe_off();
void MQTT_connect();
void sendupdate(bool force = false);
void applyDAC(int value, unsigned long now);
void drainService();

const int numValues = 17;
double wattValues[17] = { 0, 0, 4, 7, 35, 55, 61, 97, 186, 356, 526, 978, 1241, 1306, 1330, 1352, 1367};
double daccommandValues[17] = { 7000,  8000,  8750,  9000,  10000,  10400,  10500,  11000, 12000, 13500, 15000, 20000, 25000, 26800, 28000, 30000, 31000 };

int daccommandValueinterpolated;
int wattneeded;

// Removed timing-based updates for instant MQTT reaction

// Pin-Modi setzen
DFRobot_GP8413 GP8413(0x58); //Standard-Adresse mit allen DIP-Schaltern OFF

volatile int powerdrawnumber;
volatile int powerdrawsetpoint = 0;
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
volatile bool pumpmanualpower = false;
bool pumpautocontrolled = false;
volatile bool regulating_power = true;
bool HISTORY_AVERAGING = true;       // History: zeit-gewichtetes Mitteln (true) vs. Momentanwert-Snapshot (false)
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

// Drain-compressor relay ("Entwässern Kompressor"). Active-HIGH (HIGH = relay
// energized). MUST be LOW at boot — forced LOW as the very first action in
// setup(). Web button triggers a single pulse of at most drainPulseMs, then
// it returns to LOW automatically.
const int DRAIN_PIN = 13;     // GPIO13 - boot-safe output (no strapping role)
volatile unsigned long drainPulseMs = 3000;  // requested pulse duration (ms), set by HTTP/MQTT

// ---- Network configuration (WiFi vs. W5500 LAN) ---------------------------
// Persisted in NVS via ConfigStore. NET_MODE selects the interface:
//   "wifi" = WiFi only (no Ethernet hardware needed)
//   "lan"  = W5500 Ethernet primary. LAN is tried first; if it fails,
//            the device falls back to WiFi. Once LAN gets an IP, WiFi is off.
// DHCP (LAN_DHCP=true) is the default for LAN; static IP is adjustable in the UI.
String NET_MODE  = "wifi";              // "wifi" | "lan"
bool   LAN_DHCP  = true;                // true = DHCP, false = static IP below
String LAN_IP    = "192.168.178.60";    // static IP for the LAN interface
String LAN_GW    = "192.168.178.1";     // gateway
String LAN_MASK  = "255.255.255.0";     // subnet mask
String LAN_DNS   = "192.168.178.1";     // DNS server
volatile bool drainTriggerReq = false;  // set by HTTP task -> start pulse
volatile bool drainCancelReq  = false;  // set by HTTP task -> stop pulse early
bool          drainActive     = false;  // true while relay energized (loop-owned)
unsigned long drainStartMs    = 0;
bool          drainOffPending = false;  // flag to publish OFF when MQTT is connected

WiFiClient client;
// PubSubClient (matches Poolcontroller10). Credentials come from secrets.h
// (AIO_USERNAME / AIO_KEY), same as the previous Adafruit_MQTT_Client setup.
PubSubClient mqttClient(client);
// TCP timeout set in setup() via client.setTimeout(2) to prevent
// MQTT publish from blocking 30-60s on zombie TCP sockets.

// MQTT topics. PubSubClient dispatches incoming messages by topic string via
// a single callback (onMqttMessage) instead of per-topic subscribe objects.
static const char* MQTT_TOPIC_STATUS                = "heizstabsteuerung/tele/status";
static const char* MQTT_TOPIC_DRAIN_STATUS          = "heizstabsteuerung/tele/drain_status";
static const char* MQTT_TOPIC_POWERDRAW             = "heizstabsteuerung/command/powerdraw";
static const char* MQTT_TOPIC_REGULATE              = "heizstabsteuerung/command/regulate";
static const char* MQTT_TOPIC_POWERDRAWSETPOINT     = "heizstabsteuerung/command/powerdrawsetpoint";
static const char* MQTT_TOPIC_RUNPUMP               = "heizstabsteuerung/command/runpump";
static const char* MQTT_TOPIC_ZERO_FEED_TARGET      = "heizstabsteuerung/command/zero_feed_target";
static const char* MQTT_TOPIC_MAX_HEATING_POWER     = "heizstabsteuerung/command/max_heating_power";
static const char* MQTT_TOPIC_MIN_POWER_THRESHOLD   = "heizstabsteuerung/command/min_power_threshold";
static const char* MQTT_TOPIC_DEADBAND              = "heizstabsteuerung/command/deadband";
static const char* MQTT_TOPIC_PUMP_MIN_RUNTIME      = "heizstabsteuerung/command/pump_min_runtime";
static const char* MQTT_TOPIC_POWER_CHANGE_THRESHOLD= "heizstabsteuerung/command/power_change_threshold";
static const char* MQTT_TOPIC_CORRECTION_GAIN       = "heizstabsteuerung/command/correction_gain";
static const char* MQTT_TOPIC_PUMP_CYCLE_INTERVAL   = "heizstabsteuerung/command/pump_cycle_interval";
static const char* MQTT_TOPIC_PUMP_CYCLE_DURATION   = "heizstabsteuerung/command/pump_cycle_duration";
// External sensor (single value)
static const char* MQTT_TOPIC_SOLAR_AC_POWER        = "solar/ac/power";
// Drain compressor MQTT commands (for HomeBridge/MQTT-Thing integration)
static const char* MQTT_TOPIC_DRAIN_1S              = "heizstabsteuerung/command/drain_1s";
static const char* MQTT_TOPIC_DRAIN_2S              = "heizstabsteuerung/command/drain_2s";
static const char* MQTT_TOPIC_DRAIN_3S              = "heizstabsteuerung/command/drain_3s";
static const char* MQTT_TOPIC_DRAIN_10S             = "heizstabsteuerung/command/drain_10s";

// Set inside onMqttMessage() (called synchronously from mqttClient.loop(),
// i.e. from the loop task -- same threading model as before); drained once
// per loop() iteration right after mqttClient.loop() returns.
static bool g_mqttNeedsConfigSave  = false;
static bool g_mqttNeedsSendUpdate  = false;

// Fast powerdraw SSE push is requested inside the MQTT callback but executed
// outside of it, so events.send() is only ever called from the loop task.
static bool g_mqttNeedsFastPowerPush  = false;
static int  g_fastPowerdraw           = 0;
static int  g_fastPowerToConsume      = 0;
static int  g_fastPowerDrawAge        = 0;

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
bool MQTT_STATUS_ENABLED = true;                 // MQTT status publish on/off
unsigned long MQTT_STATUS_INTERVAL_MS = 60000;   // interval between MQTT status publishes

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

// millis() of the last successful NTP UDP probe (0 = never since boot).
// Set by wifiHealthCheck(); read by sendupdate() to expose the age in the UI.
static unsigned long s_lastNtpOkMs = 0;

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
  doc["netMode"] = NetManager::activeIface();
  doc["ipAddress"] = NetManager::activeIP();
  // SSID/RSSI are only meaningful on WiFi; report empty/0 over LAN.
  bool onWifi = (String(NetManager::activeIface()) == "wifi");
  doc["wifiSSID"] = onWifi ? WiFi.SSID() : String("");
  doc["rssi"] = onWifi ? WiFi.RSSI() : 0;
  doc["lanConnected"] = NetManager::usingEthernet();
  uint32_t uptimeSec = (uint32_t)(millis() / 1000);
  doc["uptime"] = uptimeSec;
  doc["rebootCount"] = s_rebootCount;
  doc["drainActive"] = drainActive;
  if (drainActive) {
    long remainingMs = (long)drainPulseMs - (long)(millis() - drainStartMs);
    if (remainingMs < 0) remainingMs = 0;
    doc["drainRemaining"] = (int)((remainingMs + 999L) / 1000L);
  } else {
    doc["drainRemaining"] = 0;
  }
  // Boot wall-clock epoch: compute once NTP is available (now - uptime).
  {
    time_t nowT; time(&nowT);
    if (nowT > 1700000000) s_bootEpoch = (uint32_t)nowT - uptimeSec;
  }
  doc["bootEpoch"] = s_bootEpoch;
  doc["heap"] = ESP.getFreeHeap();
  doc["mqttConnected"] = mqttClient.connected();
  Energy::fillStatus(doc.as<JsonVariant>());
  doc["solarAcPower"] = solarAcPowerValue;

  // NTP sync status
  time_t now;
  time(&now);
  bool ntpSynced = (now > 1700000000);  // NTP synced if epoch > 2023-11-15
  doc["ntpSynced"] = ntpSynced;
  if (ntpSynced) s_lastNtpOkMs = millis();

  doc["netConnected"] = NetManager::isOnline();
  doc["mqttStatusEnabled"] = MQTT_STATUS_ENABLED;
  doc["mqttStatusInterval"] = (int)(MQTT_STATUS_INTERVAL_MS / 1000);


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

  // Volatility filter status (nur Zustand, keine internen Fensterdaten)
  doc["pumpTempCondEnabled"] = PUMP_TEMP_COND_ENABLED;
  doc["pumpTempHystC"]       = PUMP_TEMP_HYST_C;
  doc["volEnabled"]          = VOL_ENABLED;
  doc["volActive"]           = s_volatileActive;

  // PID-Regler Status (nur Modus, keine internen Reglerdaten)
  doc["controllerMode"]  = controllerMode;

  String jsonString;
  serializeJson(doc, jsonString);

  // Skip publish if payload identical to previous one (CRC32 check, unless heartbeat forces it)
  uint32_t jsonCRC = crc32(jsonString.c_str(), jsonString.length());
  if (!force && jsonCRC == lastJsonCRC) { return; }
  lastJsonCRC = jsonCRC;
  lastTelemetryTime = millis();  // any publish counts as a heartbeat
  webserver_broadcastStatus(jsonString);  // SSE push FIRST for minimal UI latency
  // Rate-limit MQTT publish to avoid blocking the loop on every SSE update.
  // Publish failures are handled by the 30s MQTT reconnect interval; we do
  // NOT disconnect aggressively here to avoid TCP stack thrashing.
  if (MQTT_STATUS_ENABLED && millis() - lastMqttPublishTime >= MQTT_STATUS_INTERVAL_MS) {
    mqttClient.publish(MQTT_TOPIC_STATUS, jsonString.c_str());
    lastMqttPublishTime = millis();
  }
}

// PubSubClient message callback. Called synchronously from mqttClient.loop()
// (i.e. from the loop task), same threading model as the previous
// Adafruit_MQTT readSubscription() dispatch it replaces. Payload is NOT
// null-terminated, so copy into a local buffer first.
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  char buf[64];
  size_t n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);
  buf[n] = '\0';

  if (strcmp(topic, MQTT_TOPIC_DRAIN_1S) == 0 || strcmp(topic, MQTT_TOPIC_DRAIN_2S) == 0 ||
      strcmp(topic, MQTT_TOPIC_DRAIN_3S) == 0 || strcmp(topic, MQTT_TOPIC_DRAIN_10S) == 0) {
    webLog("[MQTT] Received - Topic: %s, Payload: %s", topic, buf);
  }

  if (strcmp(topic, MQTT_TOPIC_POWERDRAW) == 0) {
    int newPowerDraw = atoi(buf);
    if (newPowerDraw >= POWERDRAW_MIN_VALID && newPowerDraw <= POWERDRAW_MAX_VALID) {
      powerdrawnumber = newPowerDraw;
      powerToConsume = ZERO_FEED_IN_TARGET - powerdrawnumber;
      lastPowerDrawUpdate = millis();
      // Defer SSE push to the loop task (events.send() must not run inside the
      // PubSubClient callback context).
      g_fastPowerdraw = powerdrawnumber;
      g_fastPowerToConsume = powerToConsume;
      g_fastPowerDrawAge = 0;
      g_mqttNeedsFastPowerPush = true;
    }
  } else if (strcmp(topic, MQTT_TOPIC_RUNPUMP) == 0) {
    int tempnummer = atoi(buf);
    pumpmanualpower = (tempnummer > 0 && tempnummer < 2);
  } else if (strcmp(topic, MQTT_TOPIC_REGULATE) == 0) {
    int tempnummer = atoi(buf);
    regulating_power = (tempnummer > 0 && tempnummer < 2);
  } else if (strcmp(topic, MQTT_TOPIC_POWERDRAWSETPOINT) == 0) {
    powerdrawsetpoint = atoi(buf);
  } else if (strcmp(topic, MQTT_TOPIC_ZERO_FEED_TARGET) == 0) {
    int newVal = atoi(buf);
    if (newVal >= 0 && newVal <= 10000) {
      ZERO_FEED_IN_TARGET = newVal;
      powerToConsume = ZERO_FEED_IN_TARGET - powerdrawnumber;
    }
  } else if (strcmp(topic, MQTT_TOPIC_MAX_HEATING_POWER) == 0) {
    int newMax = atoi(buf);
    if (newMax > 0 && newMax <= 2000) {
      MAX_HEATING_POWER = newMax;
    }
  } else if (strcmp(topic, MQTT_TOPIC_MIN_POWER_THRESHOLD) == 0) {
    int newMin = atoi(buf);
    if (newMin >= 0 && newMin < 100) {
      MIN_POWER_THRESHOLD = newMin;
    }
  } else if (strcmp(topic, MQTT_TOPIC_DEADBAND) == 0) {
    int newDeadband = atoi(buf);
    if (newDeadband >= 0 && newDeadband < 500) {  // Reasonable limits for deadband
      DEADBAND = newDeadband;
    }
  } else if (strcmp(topic, MQTT_TOPIC_PUMP_MIN_RUNTIME) == 0) {
    long newRuntimeSec = atoi(buf);
    if (newRuntimeSec >= 5 && newRuntimeSec <= 300) {  // 5 seconds to 5 minutes
      PUMP_MIN_RUNTIME_MS = (unsigned long)newRuntimeSec * 1000UL;
    }
  } else if (strcmp(topic, MQTT_TOPIC_PUMP_CYCLE_INTERVAL) == 0) {
    long n2 = atoi(buf);
    if (n2 >= 0 && n2 <= 1440) PUMP_CYCLE_INTERVAL_MIN = (unsigned long)n2;   // 0 = aus
  } else if (strcmp(topic, MQTT_TOPIC_PUMP_CYCLE_DURATION) == 0) {
    long n2 = atoi(buf);
    if (n2 >= 0 && n2 <= 3600) PUMP_CYCLE_DURATION_SEC = (unsigned long)n2;
  } else if (strcmp(topic, MQTT_TOPIC_POWER_CHANGE_THRESHOLD) == 0) {
    int newThreshold = atoi(buf);
    if (newThreshold >= 1 && newThreshold <= 100) {  // 1W to 100W
      POWER_CHANGE_THRESHOLD = newThreshold;
    }
  } else if (strcmp(topic, MQTT_TOPIC_CORRECTION_GAIN) == 0) {
    // Sent as integer percent 0..150 (e.g. 80 -> 0.8). Higher = faster but more oscillation.
    int newGainPct = atoi(buf);
    if (newGainPct >= 1 && newGainPct <= 150) {
      correctionGain = newGainPct / 100.0f;
    }
  } else if (strcmp(topic, MQTT_TOPIC_SOLAR_AC_POWER) == 0) {
    if (buf[0] != '\0') {
      int val = atoi(buf);
      // Validate: solar power should be non-negative and reasonable
      if (val >= 0 && val <= 10000) {
        solarAcPowerValue = val;
      }
    }
  } else if (strcmp(topic, MQTT_TOPIC_DRAIN_1S) == 0) {
    if (strcmp(buf, "ON") == 0 || strcmp(buf, "1") == 0 || strcmp(buf, "true") == 0) {
      drainPulseMs = 1000;
      drainTriggerReq = true;
      webLog("[MQTT] drain_1s triggered");
    }
  } else if (strcmp(topic, MQTT_TOPIC_DRAIN_2S) == 0) {
    if (strcmp(buf, "ON") == 0 || strcmp(buf, "1") == 0 || strcmp(buf, "true") == 0) {
      drainPulseMs = 2000;
      drainTriggerReq = true;
      webLog("[MQTT] drain_2s triggered");
    }
  } else if (strcmp(topic, MQTT_TOPIC_DRAIN_3S) == 0) {
    if (strcmp(buf, "ON") == 0 || strcmp(buf, "1") == 0 || strcmp(buf, "true") == 0) {
      drainPulseMs = 3000;
      drainTriggerReq = true;
      webLog("[MQTT] drain_3s triggered");
    }
  } else if (strcmp(topic, MQTT_TOPIC_DRAIN_10S) == 0) {
    if (strcmp(buf, "ON") == 0 || strcmp(buf, "1") == 0 || strcmp(buf, "true") == 0) {
      drainPulseMs = 10000;
      drainTriggerReq = true;
      webLog("[MQTT] drain_10s triggered");
    }
  }

  // Only persist to NVS when an actual config parameter changed (not for
  // high-frequency sensor data like powerdraw, solarAcPower, setpoint).
  bool isPowerdraw = strcmp(topic, MQTT_TOPIC_POWERDRAW) == 0;
  bool isSetpoint  = strcmp(topic, MQTT_TOPIC_POWERDRAWSETPOINT) == 0;
  bool isSolar     = strcmp(topic, MQTT_TOPIC_SOLAR_AC_POWER) == 0;
  if (!isPowerdraw && !isSetpoint && !isSolar) {
    g_mqttNeedsConfigSave = true;
  }
  // For non-powerdraw messages, send full state update
  if (!isPowerdraw) {
    g_mqttNeedsSendUpdate = true;
  }
}

// Subscribe to all command topics. PubSubClient does not persist
// subscriptions across reconnects (unlike Adafruit_MQTT), so this must be
// called again after every successful connect.
static void mqttSubscribeAll() {
  mqttClient.subscribe(MQTT_TOPIC_POWERDRAW);
  mqttClient.subscribe(MQTT_TOPIC_RUNPUMP);
  mqttClient.subscribe(MQTT_TOPIC_REGULATE);
  mqttClient.subscribe(MQTT_TOPIC_POWERDRAWSETPOINT);
  mqttClient.subscribe(MQTT_TOPIC_ZERO_FEED_TARGET);
  mqttClient.subscribe(MQTT_TOPIC_MAX_HEATING_POWER);
  mqttClient.subscribe(MQTT_TOPIC_MIN_POWER_THRESHOLD);
  mqttClient.subscribe(MQTT_TOPIC_DEADBAND);
  mqttClient.subscribe(MQTT_TOPIC_PUMP_MIN_RUNTIME);
  mqttClient.subscribe(MQTT_TOPIC_POWER_CHANGE_THRESHOLD);
  mqttClient.subscribe(MQTT_TOPIC_CORRECTION_GAIN);
  mqttClient.subscribe(MQTT_TOPIC_PUMP_CYCLE_INTERVAL);
  mqttClient.subscribe(MQTT_TOPIC_PUMP_CYCLE_DURATION);
  mqttClient.subscribe(MQTT_TOPIC_SOLAR_AC_POWER);
  mqttClient.subscribe(MQTT_TOPIC_DRAIN_1S);
  mqttClient.subscribe(MQTT_TOPIC_DRAIN_2S);
  mqttClient.subscribe(MQTT_TOPIC_DRAIN_3S);
  mqttClient.subscribe(MQTT_TOPIC_DRAIN_10S);
}

void MQTT_connect() {
  static bool wasConnected = false;
  unsigned long nowMs = millis();

  // If already connected, just update LED state and return.
  if (mqttClient.connected()) {
    if (!wasConnected) {
      digitalWrite(STATUS_LED, STATUS_LED_ON);
      wasConnected = true;
      Serial.println("[MQTT] Connected");
      webLog("[MQTT] Connected");
    }
    return;
  }

  // Rate-limit reconnect attempts to every 30s to avoid TCP stack thrashing
  // during network flaps (LAN/WiFi switching, brief outages).
  // Allow first connect immediately (s_lastMqttCheckMs == 0).
  if (s_lastMqttCheckMs != 0 && nowMs - s_lastMqttCheckMs < MQTT_CHECK_INTERVAL_MS) {
    return;
  }
  s_lastMqttCheckMs = nowMs;

  // Require an active interface (WiFi OR W5500 LAN). In LAN mode WiFi is
  // intentionally OFF, so gating on WiFi.status() would block MQTT forever.
  // The WiFiClient is a NetworkClient (Core 3.x) and routes over Ethernet too.
  if (!NetManager::isOnline()) {
    Serial.println("[MQTT] No network, waiting...");
    webLog("[MQTT] No network, waiting...");
    return;
  }

  // Disconnect only if actually connected (idempotent).
  if (wasConnected) {
    mqttClient.disconnect();
    digitalWrite(STATUS_LED, STATUS_LED_OFF);
    wasConnected = false;
    Serial.println("[MQTT] Disconnected, will retry");
    webLog("[MQTT] Disconnected");
  }

  // Attempt connection. Non-blocking: try once per 30s interval.
  // Credentials come from secrets.h (same as the previous Adafruit setup).
  bool ok = mqttClient.connect("Heizstabsteuerung", AIO_USERNAME, AIO_KEY);
  if (ok) {
    digitalWrite(STATUS_LED, STATUS_LED_ON);
    wasConnected = true;
    Serial.println("[MQTT] Reconnected");
    webLog("[MQTT] Reconnected");
    mqttSubscribeAll();
  } else {
    Serial.printf("[MQTT] Connect failed (state=%d), retry in 30s\n", mqttClient.state());
    webLog("[MQTT] Connect failed (state=%d), retry in 30s", mqttClient.state());
    failsafe_off();
  }
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

// NOTE: The former WiFi-specific NTP UDP probe (ntpProbe/wifiHealthCheck) was
// removed. Link health is now handled by NetManager (event-driven for both
// WiFi and W5500 LAN), and the "last NTP OK" UI marker is derived from the
// real SNTP-backed system clock in sendupdate().

void setup() {
  // Task watchdog: intentionally left untouched (framework default), matching
  // Poolcontroller10 which never calls esp_task_wdt_* and has no hang/reboot
  // issues. Repeatedly reconfiguring the TWDT here caused several rounds of
  // regressions (permanent hangs, boot-loops) without fixing anything.

  // Drain-compressor relay: force LOW as the very FIRST action on boot. This
  // must never be HIGH on power-up. Doing it before any other init minimizes
  // the window in which the pin could float/glitch.
  pinMode(DRAIN_PIN, OUTPUT);
  digitalWrite(DRAIN_PIN, LOW);
  drainActive = false;

  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, STATUS_LED_OFF); // Start with LED off

  Serial.begin(115200);
  delay(200);
  webLog("[Boot] Starting Heizstabsteuerung...");

  // Persistent reboot counter: increment once per boot and store in NVS.
  {
    Preferences sys;
    if (sys.begin("sys", /*readOnly*/ false)) {
      s_rebootCount = sys.getUInt("reboots", 0) + 1;
      sys.putUInt("reboots", s_rebootCount);
      sys.end();
    }
    webLog("[Boot] Reboot #%u", s_rebootCount);
  }

  // Load persisted settings from NVS FIRST so ONEWIRE_PIN reflects user choice
  // before we initialise the OneWire bus.
  ConfigStore::load();
  webLog("[Boot] ONEWIRE_PIN from NVS = GPIO%d", ONEWIRE_PIN);
  History::setAveragingEnabled(HISTORY_AVERAGING);

  // Initialise DS18B20 OneWire bus BEFORE network starts. Network interrupts can
  // disrupt OneWire's bit-bang timing during the initial bus scan, causing
  // sensors to be missed. Roles (ROM-mapping) loaded from NVS.
  TempSensors::begin(ONEWIRE_PIN);  // adjustable via settings

  // Initialize network (WiFi or W5500 LAN based on NET_MODE setting)
  // NetManager handles WiFi/LAN switching, mDNS, and event handling
  NetManager::begin("Heizstabsteuerung");

  // Start the webserver unconditionally and synchronously here, regardless
  // of whether an interface already has an IP (matches Poolcontroller10's
  // setupWebServer() pattern). PsychicHttp/esp_http_server binds fine before
  // any interface is up; it simply becomes reachable once one connects.
  webserver_begin();
  webLog("[Boot] Webserver gestartet");

  // Configure PubSubClient: broker address/credentials come from secrets.h
  // (AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY). Callback dispatches
  // incoming command messages by topic string.
  mqttClient.setServer(AIO_SERVER, AIO_SERVERPORT);
  mqttClient.setCallback(onMqttMessage);
  // Default buffer (256 B) is too small for the ~1.5 KB status JSON. Pool
  // size must fit topic + payload; 2048 B leaves comfortable headroom.
  mqttClient.setBufferSize(2048);

  ArduinoOTA.setHostname("Heizstabsteuerung");
  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Start");
    webserver_pauseSSE = true;  // stop SSE traffic during upload
    // NOTE: Do NOT call mqttClient.disconnect() here from the OTA callback.
    // PubSubClient is not reentrant-safe; disconnecting from this context
    // can corrupt the client state and cause crashes. The 30s reconnect
    // interval in loop() will naturally reconnect after OTA completes.
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] End");
    webserver_pauseSSE = false;  // resume SSE
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

  // Limit WiFiClient TCP timeout to 2s (default is 30-60s).
  // Prevents MQTT publish/loop() from blocking on zombie sockets.
  client.setTimeout(2);

  webLog("[Boot] Setup complete");
}

// Apply config changes queued by HTTP handlers. Runs on the loop task so all
// global state mutations stay off the httpd task.
static void applyPendingConfig() {
  PendingConfig p;
  webserver_getAndClearPendingConfig(p);

  if (p.hasZeroFeedTarget) ZERO_FEED_IN_TARGET = p.zeroFeedTarget;
  if (p.hasMaxHeatingPower) MAX_HEATING_POWER = p.maxHeatingPower;
  if (p.hasMinPowerThreshold) MIN_POWER_THRESHOLD = p.minPowerThreshold;
  if (p.hasDeadband) DEADBAND = p.deadband;
  if (p.hasPowerChangeThreshold) POWER_CHANGE_THRESHOLD = p.powerChangeThreshold;
  if (p.hasCorrectionGain) correctionGain = p.correctionGainPct / 100.0f;

  if (p.hasPumpMinRuntime) PUMP_MIN_RUNTIME_MS = p.pumpMinRuntimeSec * 1000UL;
  if (p.hasPumpCycleInterval) PUMP_CYCLE_INTERVAL_MIN = p.pumpCycleIntervalMin;
  if (p.hasPumpCycleDuration) PUMP_CYCLE_DURATION_SEC = p.pumpCycleDurationSec;

  if (p.hasControllerMode) controllerMode = p.controllerMode;

  if (p.hasPidKp) PidController::setKp(p.pidKp);
  if (p.hasPidKi) PidController::setKi(p.pidKi);
  if (p.hasPidSolarFf) PidController::setSolarFf(p.pidSolarFf);
  if (p.hasOnlineAdapt) PidController::setOnlineAdaptEnabled(p.onlineAdapt);

  if (p.hasPumpTempCond) PUMP_TEMP_COND_ENABLED = p.pumpTempCond;
  if (p.hasPumpTempHyst) PUMP_TEMP_HYST_C = p.pumpTempHyst;

  if (p.hasVolEnabled) VOL_ENABLED = p.volEnabled;
  if (p.hasHistoryAveraging) {
    HISTORY_AVERAGING = p.historyAveraging;
    History::setAveragingEnabled(HISTORY_AVERAGING);
  }

  if (p.hasVolWindowMin) VOL_WINDOW_MIN = p.volWindowMin;
  if (p.hasVolThresholdW) VOL_THRESHOLD_W = p.volThresholdW;

  if (p.hasOnewirePin) ONEWIRE_PIN = p.onewirePin;

  if (p.hasMaxBoilerTemp) MAX_BOILER_TEMP_C = p.maxBoilerTemp;
  if (p.hasMaxHeaterRodTemp) MAX_HEATER_ROD_TEMP_C = p.maxHeaterRodTemp;

  if (p.hasNetMode) NET_MODE = p.netMode;
  if (p.hasLanDhcp) LAN_DHCP = p.lanDhcp;
  if (p.hasLanIp) LAN_IP = p.lanIp;
  if (p.hasLanGw) LAN_GW = p.lanGw;
  if (p.hasLanMask) LAN_MASK = p.lanMask;
  if (p.hasLanDns) LAN_DNS = p.lanDns;

  if (p.hasMqttStatusEnabled) MQTT_STATUS_ENABLED = p.mqttStatusEnabled;
  if (p.hasMqttStatusInterval) MQTT_STATUS_INTERVAL_MS = (unsigned long)p.mqttStatusIntervalSec * 1000UL;

  ConfigStore::save();
  webserver_ssePushPending = true;
}

// Drain-compressor relay state machine. Runs on the loop task every iteration,
// BEFORE any early-return paths, so the max-on time limit is always enforced
// (even during a WiFi outage or OTA). HTTP handlers only set request flags.
void drainService() {
  // Early cancel (manual OFF from UI) takes priority over a fresh trigger.
  if (drainCancelReq) {
    drainCancelReq  = false;
    drainTriggerReq = false;
    if (drainActive) {
      digitalWrite(DRAIN_PIN, LOW);
      drainActive = false;
      drainOffPending = true;
      webLog("[Drain] Kompressor-Entwaesserung AUS (manuell)");
      webserver_ssePushPending = true;
    }
    return;
  }
  // Start a new pulse on request.
  if (drainTriggerReq) {
    drainTriggerReq = false;
    digitalWrite(DRAIN_PIN, HIGH);
    drainActive  = true;
    drainStartMs = millis();
    webLog("[Drain] Kompressor-Entwaesserung AN (max %lus)", drainPulseMs / 1000);
    webserver_ssePushPending = true;
  }
  // Enforce the maximum on-time: auto-return to LOW.
  if (drainActive && (millis() - drainStartMs >= drainPulseMs)) {
    digitalWrite(DRAIN_PIN, LOW);
    drainActive = false;
    drainOffPending = true;
    webLog("[Drain] Kompressor-Entwaesserung AUS (auto nach %lus)", drainPulseMs / 1000);
    webserver_ssePushPending = true;
  }
}

void loop() {
  // Drain-compressor pulse: serviced first so the 3 s auto-off limit is
  // guaranteed regardless of WiFi/OTA early-returns further down.
  drainService();

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

  // History + Energy: independent of WiFi/MQTT (local NVS + RTC clock).
  // Moved before WiFi check so data is recorded even during WiFi outages.
  History::tickSample(powerdrawnumber, wattneeded, solarAcPowerValue,
                      TempSensors::getBoilerC(), TempSensors::getInletC(),
                      NAN, TempSensors::getOutletC(), TempSensors::getHeaterRodC());
  History::tickSave();
  Energy::tick(heating ? wattneeded : 0);
  Energy::tickSave();

  // webserver_loop() always runs — process reboot requests and SSE cleanup
  // even during network/MQTT outages (httpd task may still serve cached pages).
  webserver_loop();

  // Network manager loop: handles WiFi/LAN switching, reconnection, and state
  NetManager::loop();

  // Failsafe if network or MQTT are not connected
  if (!NetManager::isOnline()) {
    failsafe_off();
    return; // non-blocking
  }

  // ---- Core timing reference for this iteration ----
  unsigned long currentTime = millis();

  // ---- SSE telemetry heartbeat (runs even during MQTT outage) ----
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

  // Apply any config changes queued by HTTP handlers. This stays on the loop
  // task so shared globals (including String objects) are never mutated from the
  // httpd task. Must happen before draining webserver_ssePushPending so the
  // subsequent broadcast reflects the new values.
  if (webserver_configPending) {
    webserver_configPending = false;
    applyPendingConfig();
  }

  // HTTP handlers set this flag when they mutate state; we drain it once
  // per loop iteration so events.send() is only ever called from the loop
  // task (PoolController pattern — eliminates cross-task _clients races).
  if (webserver_ssePushPending) {
    webserver_ssePushPending = false;
    sendupdate(true);
  }

  // ================= MQTT (at end of loop — may block on zombie TCP) ======
  // Moved after regulation/pump/telemetry so a stalled MQTT socket only
  // delays MQTT communication, not the core control loop or SSE updates.

  // Handle MQTT connection
  if (!mqttClient.connected()) {
    failsafe_off();
    MQTT_connect();
    return; // retry next iteration
  }

  // Process inbound MQTT traffic and keepalive. The PubSubClient callback
  // (onMqttMessage) runs synchronously inside this call, same task as loop().
  mqttClient.loop();

  // Drain the fast powerdraw SSE push requested by the MQTT callback.
  if (g_mqttNeedsFastPowerPush) {
    g_mqttNeedsFastPowerPush = false;
    webserver_broadcastPowerFast(g_fastPowerdraw, g_fastPowerToConsume, g_fastPowerDrawAge);
  }

  // Publish drain OFF status if pending (only when MQTT is connected)
  if (drainOffPending) {
    drainOffPending = false;
    mqttClient.publish(MQTT_TOPIC_DRAIN_STATUS, "OFF");
  }

  // Drain deferred side effects set inside onMqttMessage() outside the
  // PubSubClient callback context (avoid heavy work during packet handling).
  if (g_mqttNeedsConfigSave) {
    g_mqttNeedsConfigSave = false;
    ConfigStore::save();
  }
  if (g_mqttNeedsSendUpdate) {
    g_mqttNeedsSendUpdate = false;
    sendupdate();
  }
}
