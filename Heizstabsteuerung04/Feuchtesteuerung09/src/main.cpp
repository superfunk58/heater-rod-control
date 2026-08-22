#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <FS.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <time.h>
#include <Preferences.h>
#include <Update.h>

// ------------ Hostname ------------
static const char *DEVICE_HOSTNAME = "luefter_bad_og";

// ------------ Firmware Version ------------
static const char *FIRMWARE_VERSION = "1.2.0";
static const char *FIRMWARE_FOLDER = "Feuchtesteuerung08";
static const char *FIRMWARE_BUILD_DATE = __DATE__;
static const char *FIRMWARE_BUILD_TIME = __TIME__;

// ------------ Pin configuration ------------
// Adjust these to your wiring
static const uint8_t PIN_I2C_SDA = 23;      // SHT31 SDA
static const uint8_t PIN_I2C_SCL = 22;      // SHT31 SCL
static const uint8_t PIN_ONEWIRE = 5;       // 1-wire bus for DS18B20 sensors
static const uint8_t PIN_FAN_RELAY = 15;     // Main power switch for IBT-2
static const uint8_t PIN_IBT2_RPWM = 19;     // IBT-2 Right PWM (forward)
static const uint8_t PIN_IBT2_LPWM = 21;     // IBT-2 Left PWM (reverse)
static const uint8_t PIN_IBT2_EN = 18;       // IBT-2 Enable (R_EN/L_EN bridged)
static const uint8_t PIN_HEATER_SSR = 4;
static const uint8_t PIN_STATUS_LED = 2;   // Onboard LED on most ESP32 dev boards

// W5500 Ethernet SPI Pins (optional)
static const uint8_t PIN_ETH_CS   = 14;   // W5500 CS
static const uint8_t PIN_ETH_MISO = 12;   // W5500 MISO
static const uint8_t PIN_ETH_MOSI = 13;   // W5500 MOSI
static const uint8_t PIN_ETH_SCLK = 27;   // W5500 SCLK
static const uint8_t PIN_ETH_INT  = 26;   // W5500 INT (optional, -1 if not used)
static const uint8_t PIN_ETH_RST  = 25;   // W5500 RST (optional, -1 if not used)

// Fan PWM settings
static const uint8_t FAN_MIN_PWM_PERCENT = 35;  // Minimum PWM percentage (motor won't run below this)

// PWM settings for IBT-2 fan control
static const uint16_t FAN_PWM_FREQ = 25000;  // 25kHz - high frequency for smooth operation
static const uint8_t FAN_PWM_RESOLUTION = 8;   // 256 steps (0-255)
static const uint8_t FAN_RPWM_CHANNEL = 0;   // RPWM channel
static const uint8_t FAN_LPWM_CHANNEL = 1;   // LPWM channel

// ------------ Global objects ------------
Adafruit_SHT31 sht31 = Adafruit_SHT31();
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);
DeviceAddress ds18b20_beforeFan;
DeviceAddress ds18b20_afterHeater;
bool ds18b20_assigned = false;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
AsyncWebServer server(80);

// ------------ Configuration structure ------------
struct AppConfig {
  String wifiSsid = "funknetz";
  String wifiPassword = "ja33tune";

  String mqttHost = "192.168.178.67";
  uint16_t mqttPort = 1883;
  String mqttUser = "benson";
  String mqttPassword = "ja33tune";
  String mqttBaseTopic = "luefter_bad_og"; // e.g. luefter_bad_og

  float humidityOn = 70.0f;   // %RH threshold to turn fan/heater ON
  float humidityOff = 60.0f;  // %RH threshold to turn fan/heater OFF

  bool heaterEnabled = true;
  float heaterOnTempDiff = 5.0f;   // Heater ON when air before fan is X°C cooler than wall temp (DS18B20 after heater)
  float heaterOffTempDiff = 2.0f;  // Heater OFF when wall temp is only Y°C warmer than air before fan
  bool fanDirection = false;   // Default fan direction: false=EINBLASEN (default), true=ABSAUGEN
  uint8_t fanSpeedPercent = 75;  // Default fan speed percentage (0-100)
  unsigned int absaugenDurationSecs = 300; // ABSAUGEN mode duration in seconds (default 5 minutes)
  uint8_t absaugenSpeedPercent = 100; // Fan speed during ABSAUGEN (0-100%), separate from EINBLASEN
  bool absaugenCyclicEnabled = false;       // Periodic ABSAUGEN cycles
  unsigned int absaugenCyclicIntervalMins = 120; // Minutes between ABSAUGEN cycles (1-480)
  unsigned int absaugenCyclicDurationMins = 5;   // Duration of each ABSAUGEN cycle (1-30)
  
  float heaterMinRoomTemp = 0.0f;     // Second heater condition: heater ON when room temp < this (0 = disabled)
  float heaterMaxTemp = 55.0f;        // Maximum temperature limit for bang-bang heater control (30-50°C range)
  float tempSafetyDiff = 10.0f;         // Temp safety: force fan ON when tempAfterHeater - tempBeforeFan >= this value
  float tempSafetyHyst = 2.0f;           // Temp safety hysteresis: release when diff < tempSafetyDiff - tempSafetyHyst
  // Cyclic timer: periodic fan runs regardless of humidity
  bool cyclicEnabled = false;            // Whether cyclic timer is active
  unsigned int cyclicIntervalMins = 60;  // Minutes between cycles (1-120)
  unsigned int cyclicDurationMins = 5;   // Duration of each cycle in minutes (1-20)
  bool cyclicHeater = false;             // Whether heater runs during cyclic timer (with temp condition)
  bool cyclicHeaterForce = false;         // Force heater ON during cyclic run (ignore temp condition)
  uint8_t cyclicSpeedPercent = 100;       // Fan speed during cyclic run (0-100%)
  
  bool autoEnabled = true;              // Automatic control permanently enabled/disabled
  unsigned int bathPauseMins = 30;       // Bath pause default duration in minutes
  
  // Cyclic start minute alignment (0=full hour, 15=quarter, 30=half, 45=three-quarter)
  uint8_t cyclicStartMin = 0;
  uint8_t absaugenCyclicStartMin = 0;
  
  // DS18B20 sensor assignment by ROM address (hex string, 16 chars)
  String ds18b20_beforeFan_id = "";
  String ds18b20_afterHeater_id = "";
  
  unsigned int mqttIntervalSecs = 10;   // MQTT publish interval in seconds (1-300)
  
  // Humidity descent monitoring: if humidity drops too slowly during ventilation, turn heater on
  float humDescentMinDrop = 2.0f;        // Minimum humidity drop in %RH expected within the time window
  unsigned int humDescentTimeSecs = 300;  // Time window in seconds (default 5 min)
  bool humDescentHeaterEnable = true;     // Enable heater when humidity descends too slowly
  
  bool scheduleEnabled = false;        // Whether weekly schedule is active
  // Weekly schedule: 7 days (0=Sunday..6=Saturday), 24 bits per day (bit 0=00:00, bit 23=23:00)
  // If bit is set, control is ACTIVE during that hour
  uint32_t schedule[7] = {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF}; // Default: all hours active
  
  // Energy monitor settings
  unsigned int heaterWatts = 2000;     // Heater power in Watts
  float energyPriceKwh = 0.35f;        // Electricity price in €/kWh
};

AppConfig config;

// Runtime state
float g_temperatureC = NAN;
float g_humidity = NAN;
float g_tempBeforeFanC = NAN;      // DS18B20 #1
float g_tempAfterHeaterC = NAN;    // DS18B20 #2
bool g_fanOn = false;
bool g_heaterOn = false;
uint8_t g_fanSpeedPercent = 75;     // Fan speed percentage (0-100), default 75%
bool g_fanDirection = false;        // Fan direction: true=ABSAUGEN, false=EINBLASEN (default)
bool g_sht31Available = false;     // Track if SHT31 sensor is detected
unsigned long g_lastMqttPublish = 0;
unsigned long g_lastIpPrint = 0;

// ABSAUGEN mode timer - automatic fallback to EINBLASEN
unsigned long g_absaugenUntilMillis = 0;  // millis() timestamp when ABSAUGEN mode expires

// ABSAUGEN cyclic timer runtime state
bool g_absaugenCyclicRunning = false;          // Currently in an ABSAUGEN cyclic run
unsigned long g_absaugenCyclicLastOffMillis = 0; // When last ABSAUGEN cyclic run ended
unsigned long g_absaugenCyclicRunUntilMillis = 0; // When current ABSAUGEN cyclic run should end

// Control mode (automatic vs. manual override)
enum ControlMode { MODE_AUTO = 0, MODE_MANUAL = 1 };
ControlMode g_controlMode = MODE_AUTO;
bool g_manualFanOn = false;
bool g_manualHeaterOn = false;
unsigned long g_manualUntilMillis = 0;  // millis() timestamp when manual mode expires

// Fan on/off tracking
bool g_fanWasOn = false;                  // Previous fan state for edge detection

// Temperature safety: fan forced on due to high temp difference
bool g_tempSafetyFanForced = false;

// Cyclic timer runtime state
bool g_cyclicFanOn = false;                  // Fan is currently on due to cyclic timer
unsigned long g_cyclicLastOffMillis = 0;     // millis() when last cyclic run ended (for interval timing)
unsigned long g_cyclicRunUntilMillis = 0;    // millis() when current cyclic run should end

// NTP time availability
bool g_ntpSynced = false;

// Bath pause: entire system inactive for X minutes
bool g_bathPauseActive = false;
unsigned long g_bathPauseUntilMillis = 0;

// Heater absolute temperature limit - SSR forced off without clearing manual command
bool g_heaterOvertemp = false;

// Heater runtime tracking (seconds)
unsigned long g_heaterOnTotalSecs = 0;       // Total heater ON time in seconds (cumulative)
unsigned long g_heaterOnStartMillis = 0;     // millis() when heater was last turned ON
bool g_heaterWasOn = false;                  // Previous heater state for edge detection
unsigned long g_heaterTrackingStartEpoch = 0; // Unix epoch when tracking started (for display)

// Pending reboot (set by web handler, executed in loop after response is sent)
bool g_rebootPending = false;

// Humidity descent tracking: monitor if humidity drops fast enough during ventilation
float g_humDescentStartHumidity = NAN;   // Humidity when fan started (or last check)
unsigned long g_humDescentStartMillis = 0; // millis() when tracking started
bool g_humDescentHeaterForced = false;    // Heater forced on due to slow humidity descent

// Network type: "wifi" or "ethernet"
String g_networkType = "wifi";
bool g_ethConnected = false;
unsigned long g_rebootAtMillis = 0;

// Cached DS18B20 sensor list (populated once at boot, never re-enumerated)
static const int DS18B20_MAX_SENSORS = 4;
DeviceAddress g_ds18b20_addrs[DS18B20_MAX_SENSORS];
int g_ds18b20_count = 0;

// Direction change control - non-blocking with millis()
bool g_directionChangeInProgress = false;
unsigned long g_directionChangePauseUntil = 0;
bool g_targetDirection = false;
uint8_t g_savedSpeed = 0;

// Last slider value storage
uint8_t g_lastSliderSpeed = 75; // Default to 75%

// Forward declarations (needed because mqttCallback uses functions defined later)
bool isScheduleActive();
void setFanPWM(bool on);
void setFanSpeed(uint8_t percent);
void setFanDirection(bool absaugen);

// ------------ Humidity history (last ~4h) ------------

struct HumiditySample {
  uint32_t epochAt;   // Unix epoch timestamp when sampled (seconds since 1970)
  float humidity;     // %RH
  float temperature;  // °C
  bool fanOn;         // fan state
  bool heaterOn;      // heater state
  bool fanDirection;  // false=Einblasen, true=Absaugen
};

static const size_t HISTORY_CAPACITY = 4320;       // 4320 samples @ 1 min = 72h
HumiditySample g_history[HISTORY_CAPACITY];
size_t g_historyCount = 0;                         // number of valid samples (<= capacity)
size_t g_historyIndex = 0;                         // next position to write (ring buffer)
unsigned long g_lastHistorySampleMillis = 0;       // last time we added a sample

// Non-blocking direction change with 2 second pause
void updateDirectionChange() {
  if (!g_directionChangeInProgress) {
    return;
  }
  
  unsigned long now = millis();
  if (now >= g_directionChangePauseUntil) {
    // Pause complete - apply new direction and restore speed
    g_fanDirection = g_targetDirection;
    g_directionChangeInProgress = false;
    
    Serial.printf("[DIR] Pause complete - direction now: %s\n", g_fanDirection ? "ABSAUGEN" : "EINBLASEN");
    
    // If switching to ABSAUGEN, start the fallback timer
    if (g_fanDirection) {
      g_absaugenUntilMillis = millis() + (config.absaugenDurationSecs * 1000UL);
      Serial.printf("[ABSAUGEN] Timer started: %d seconds (until %lu)\n", config.absaugenDurationSecs, g_absaugenUntilMillis);
    } else {
      // Switching to EINBLASEN - cancel timer
      g_absaugenUntilMillis = 0;
      Serial.println("[EINBLASEN] Timer cancelled");
    }
    
    if (g_fanOn && g_savedSpeed > 0) {
      // Enforce minimum PWM
      uint8_t actualSpeed = (g_savedSpeed < FAN_MIN_PWM_PERCENT) ? FAN_MIN_PWM_PERCENT : g_savedSpeed;
      uint32_t pwmValue = (actualSpeed * 255) / 100;
      
      if (g_fanDirection) {
        // ABSAUGEN - RPWM active, LPWM = 0
        ledcWrite(FAN_RPWM_CHANNEL, pwmValue);
        ledcWrite(FAN_LPWM_CHANNEL, 0);
      } else {
        // EINBLASEN - LPWM active, RPWM = 0
        ledcWrite(FAN_RPWM_CHANNEL, 0);
        ledcWrite(FAN_LPWM_CHANNEL, pwmValue);
      }
      Serial.printf("[DIR] Fan restarted at %d%% (requested: %d%%)\n", actualSpeed, g_savedSpeed);
    }
  }
}

void startDirectionChange(bool newDirection) {
  if (newDirection == g_fanDirection) {
    return;
  }
  
  Serial.printf("[DIR] Starting direction change: %s -> %s\n", 
                g_fanDirection ? "ABSAUGEN" : "EINBLASEN",
                newDirection ? "ABSAUGEN" : "EINBLASEN");
  
  // Stop PWM but keep relay ON, start 1 second pause
  if (g_fanOn) {
    g_savedSpeed = g_fanSpeedPercent;
    ledcWrite(FAN_RPWM_CHANNEL, 0);
    ledcWrite(FAN_LPWM_CHANNEL, 0);
    // Relay stays ON (PIN_FAN_RELAY remains HIGH)
    
    g_directionChangeInProgress = true;
    g_targetDirection = newDirection;
    g_directionChangePauseUntil = millis() + 1000UL; // 1 second pause
    
    Serial.printf("[DIR] PWM stopped (relay ON), 1s pause started (saved speed: %d%%)\n", g_savedSpeed);
  } else {
    // Fan is off - just update direction immediately
    g_fanDirection = newDirection;
    ledcWrite(FAN_RPWM_CHANNEL, 0);
    ledcWrite(FAN_LPWM_CHANNEL, 0);
    
    Serial.println("[DIR] Fan is off - direction changed immediately");
    
    // Still need to handle ABSAUGEN timer even when fan is off
    if (g_fanDirection) {
      g_absaugenUntilMillis = millis() + (config.absaugenDurationSecs * 1000UL);
      Serial.printf("[ABSAUGEN] Timer started (fan off): %d seconds\n", config.absaugenDurationSecs);
    } else {
      g_absaugenUntilMillis = 0;
      Serial.println("[EINBLASEN] Timer cancelled (fan off)");
    }
  }
}

// ------------ Preferences-based config handling ------------
Preferences prefs;

// Load heater runtime from Preferences
void loadHeaterRuntime() {
  prefs.begin("runtime", true);
  g_heaterOnTotalSecs = prefs.getULong("heat_secs", 0);
  g_heaterTrackingStartEpoch = prefs.getULong("track_start", 0);
  prefs.end();
  
  // If no tracking start time saved, initialize it when NTP syncs
  if (g_heaterTrackingStartEpoch == 0) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
      time_t now;
      time(&now);
      g_heaterTrackingStartEpoch = (unsigned long)now;
      // Save immediately
      prefs.begin("runtime", false);
      prefs.putULong("track_start", g_heaterTrackingStartEpoch);
      prefs.end();
    }
  }
  
  Serial.printf("[RUNTIME] Loaded heater runtime: %lu seconds, tracking since epoch %lu\n", 
                g_heaterOnTotalSecs, g_heaterTrackingStartEpoch);
}

// Save heater runtime to Preferences (call periodically)
void saveHeaterRuntime() {
  static unsigned long lastSave = 0;
  unsigned long now = millis();
  // Save every 5 minutes
  if (now - lastSave < 300000UL && lastSave > 0) return;
  lastSave = now;
  
  unsigned long totalSecs = g_heaterOnTotalSecs;
  if (g_heaterWasOn && g_heaterOnStartMillis > 0) {
    totalSecs += (now - g_heaterOnStartMillis) / 1000UL;
  }
  
  // Initialize tracking start if not set and NTP is synced
  if (g_heaterTrackingStartEpoch == 0 && g_ntpSynced) {
    time_t nowTime;
    time(&nowTime);
    g_heaterTrackingStartEpoch = (unsigned long)nowTime;
  }
  
  prefs.begin("runtime", false);
  prefs.putULong("heat_secs", totalSecs);
  if (g_heaterTrackingStartEpoch > 0) {
    prefs.putULong("track_start", g_heaterTrackingStartEpoch);
  }
  prefs.end();
}

// Save history to separate NVS partition "histdata" - persistent across firmware/filesystem uploads
// This partition is not touched by firmware or filesystem uploads
void saveHistoryToFS() {
  if (g_historyCount == 0) return;
  
  Preferences histPrefs;
  histPrefs.begin("hist", false, "histdata"); // read-write, "histdata" partition
  
  histPrefs.putUInt("count", g_historyCount);
  histPrefs.putUInt("index", g_historyIndex);
  
  // Save samples in chunks (NVS blob limit is ~4000 bytes)
  // Each sample is ~14 bytes, save 100 samples per chunk (1400 bytes)
  const size_t CHUNK_SIZE = 100;
  for (size_t chunk = 0; chunk < (HISTORY_CAPACITY + CHUNK_SIZE - 1) / CHUNK_SIZE; chunk++) {
    size_t start = chunk * CHUNK_SIZE;
    size_t count = min(CHUNK_SIZE, HISTORY_CAPACITY - start);
    String key = "d" + String(chunk);
    histPrefs.putBytes(key.c_str(), &g_history[start], count * sizeof(HumiditySample));
  }
  
  histPrefs.end();
  Serial.printf("[HISTORY] Saved %d samples to NVS\n", g_historyCount);
}

// Load history from separate NVS partition "histdata"
void loadHistoryFromFS() {
  Preferences histPrefs;
  histPrefs.begin("hist", true, "histdata"); // read-only, "histdata" partition
  
  g_historyCount = histPrefs.getUInt("count", 0);
  g_historyIndex = histPrefs.getUInt("index", 0);
  
  // Validate ranges
  if (g_historyCount == 0 || g_historyCount > HISTORY_CAPACITY || g_historyIndex >= HISTORY_CAPACITY) {
    g_historyCount = 0;
    g_historyIndex = 0;
    histPrefs.end();
    Serial.println("[HISTORY] Invalid count/index - resetting");
    return;
  }
  
  // Load samples in chunks
  const size_t CHUNK_SIZE = 100;
  for (size_t chunk = 0; chunk < (HISTORY_CAPACITY + CHUNK_SIZE - 1) / CHUNK_SIZE; chunk++) {
    size_t start = chunk * CHUNK_SIZE;
    size_t count = min(CHUNK_SIZE, HISTORY_CAPACITY - start);
    String key = "d" + String(chunk);
    histPrefs.getBytes(key.c_str(), &g_history[start], count * sizeof(HumiditySample));
  }
  
  histPrefs.end();
  
  // With epoch-based timestamps, no adjustment needed - they are absolute
  g_lastHistorySampleMillis = millis() - 60000UL; // Allow new sample soon
  
  Serial.printf("[HISTORY] Loaded %d samples from NVS\n", g_historyCount);
}

void loadConfigFromPrefs() {
  prefs.begin("cfg", true); // read-only
  
  config.wifiSsid = prefs.getString("wifi_ssid", config.wifiSsid);
  config.wifiPassword = prefs.getString("wifi_pw", config.wifiPassword);
  config.mqttHost = prefs.getString("mqtt_host", config.mqttHost);
  config.mqttPort = prefs.getUShort("mqtt_port", config.mqttPort);
  config.mqttUser = prefs.getString("mqtt_user", config.mqttUser);
  config.mqttPassword = prefs.getString("mqtt_pw", config.mqttPassword);
  config.mqttBaseTopic = prefs.getString("mqtt_topic", config.mqttBaseTopic);
  config.humidityOn = prefs.getFloat("hum_on", config.humidityOn);
  config.humidityOff = prefs.getFloat("hum_off", config.humidityOff);
  config.heaterEnabled = prefs.getBool("heat_en", config.heaterEnabled);
  config.heaterOnTempDiff = prefs.getFloat("heat_on_d", config.heaterOnTempDiff);
  config.heaterOffTempDiff = prefs.getFloat("heat_off_d", config.heaterOffTempDiff);
  config.heaterMinRoomTemp = prefs.getFloat("heat_min_w", config.heaterMinRoomTemp);
  config.heaterMaxTemp = prefs.getFloat("heat_max", config.heaterMaxTemp);
  config.fanSpeedPercent = prefs.getUChar("fan_spd", config.fanSpeedPercent);
  config.fanDirection = prefs.getBool("fan_dir", config.fanDirection);
  config.absaugenDurationSecs = prefs.getUInt("abs_dur", config.absaugenDurationSecs);
  config.absaugenSpeedPercent = prefs.getUChar("abs_spd", config.absaugenSpeedPercent);
  config.absaugenCyclicEnabled = prefs.getBool("abs_cyc_en", config.absaugenCyclicEnabled);
  config.absaugenCyclicIntervalMins = prefs.getUInt("abs_cyc_iv", config.absaugenCyclicIntervalMins);
  config.absaugenCyclicDurationMins = prefs.getUInt("abs_cyc_dm", config.absaugenCyclicDurationMins);
  config.autoEnabled = prefs.getBool("auto_en", config.autoEnabled);
  config.bathPauseMins = prefs.getUInt("bath_mins", config.bathPauseMins);
  config.ds18b20_beforeFan_id = prefs.getString("ds_before", config.ds18b20_beforeFan_id);
  config.ds18b20_afterHeater_id = prefs.getString("ds_after", config.ds18b20_afterHeater_id);
  config.tempSafetyDiff = prefs.getFloat("tsafe_diff", config.tempSafetyDiff);
  config.tempSafetyHyst = prefs.getFloat("tsafe_hyst", config.tempSafetyHyst);
  config.cyclicEnabled = prefs.getBool("cyc_en", config.cyclicEnabled);
  config.cyclicIntervalMins = prefs.getUInt("cyc_iv", config.cyclicIntervalMins);
  config.cyclicDurationMins = prefs.getUInt("cyc_dm", config.cyclicDurationMins);
  config.cyclicHeater = prefs.getBool("cyc_heat", config.cyclicHeater);
  config.cyclicHeaterForce = prefs.getBool("cyc_heatf", config.cyclicHeaterForce);
  config.cyclicSpeedPercent = prefs.getUChar("cyc_spd", config.cyclicSpeedPercent);
  config.cyclicStartMin = prefs.getUChar("cyc_stmin", config.cyclicStartMin);
  config.absaugenCyclicStartMin = prefs.getUChar("abs_stmin", config.absaugenCyclicStartMin);
  config.mqttIntervalSecs = prefs.getUInt("mqtt_iv", config.mqttIntervalSecs);
  config.humDescentMinDrop = prefs.getFloat("hd_drop", config.humDescentMinDrop);
  config.humDescentTimeSecs = prefs.getUInt("hd_time", config.humDescentTimeSecs);
  config.humDescentHeaterEnable = prefs.getBool("hd_en", config.humDescentHeaterEnable);
  config.scheduleEnabled = prefs.getBool("sched_en", config.scheduleEnabled);
  for (int d = 0; d < 7; d++) {
    String key = "sched_" + String(d);
    config.schedule[d] = prefs.getUInt(key.c_str(), config.schedule[d]);
  }
  config.heaterWatts = prefs.getUInt("heat_watts", config.heaterWatts);
  config.energyPriceKwh = prefs.getFloat("energy_price", config.energyPriceKwh);
  prefs.end();
  Serial.println("[CFG] Config loaded from Preferences.");
}

void saveConfigToPrefs() {
  prefs.begin("cfg", false); // read-write
  prefs.putString("wifi_ssid", config.wifiSsid);
  prefs.putString("wifi_pw", config.wifiPassword);
  prefs.putString("mqtt_host", config.mqttHost);
  prefs.putUShort("mqtt_port", config.mqttPort);
  prefs.putString("mqtt_user", config.mqttUser);
  prefs.putString("mqtt_pw", config.mqttPassword);
  prefs.putString("mqtt_topic", config.mqttBaseTopic);
  prefs.putFloat("hum_on", config.humidityOn);
  prefs.putFloat("hum_off", config.humidityOff);
  prefs.putBool("heat_en", config.heaterEnabled);
  prefs.putFloat("heat_on_d", config.heaterOnTempDiff);
  prefs.putFloat("heat_off_d", config.heaterOffTempDiff);
  prefs.putFloat("heat_min_w", config.heaterMinRoomTemp);
  prefs.putFloat("heat_max", config.heaterMaxTemp);
  prefs.putUChar("fan_spd", config.fanSpeedPercent);
  prefs.putBool("fan_dir", config.fanDirection);
  prefs.putUInt("abs_dur", config.absaugenDurationSecs);
  prefs.putUChar("abs_spd", config.absaugenSpeedPercent);
  prefs.putBool("abs_cyc_en", config.absaugenCyclicEnabled);
  prefs.putUInt("abs_cyc_iv", config.absaugenCyclicIntervalMins);
  prefs.putUInt("abs_cyc_dm", config.absaugenCyclicDurationMins);
  prefs.putBool("auto_en", config.autoEnabled);
  prefs.putUInt("bath_mins", config.bathPauseMins);
  prefs.putString("ds_before", config.ds18b20_beforeFan_id);
  prefs.putString("ds_after", config.ds18b20_afterHeater_id);
  prefs.putFloat("tsafe_diff", config.tempSafetyDiff);
  prefs.putFloat("tsafe_hyst", config.tempSafetyHyst);
  prefs.putBool("cyc_en", config.cyclicEnabled);
  prefs.putUInt("cyc_iv", config.cyclicIntervalMins);
  prefs.putUInt("cyc_dm", config.cyclicDurationMins);
  prefs.putBool("cyc_heat", config.cyclicHeater);
  prefs.putBool("cyc_heatf", config.cyclicHeaterForce);
  prefs.putUChar("cyc_spd", config.cyclicSpeedPercent);
  prefs.putUChar("cyc_stmin", config.cyclicStartMin);
  prefs.putUChar("abs_stmin", config.absaugenCyclicStartMin);
  prefs.putUInt("mqtt_iv", config.mqttIntervalSecs);
  prefs.putFloat("hd_drop", config.humDescentMinDrop);
  prefs.putUInt("hd_time", config.humDescentTimeSecs);
  prefs.putBool("hd_en", config.humDescentHeaterEnable);
  prefs.putBool("sched_en", config.scheduleEnabled);
  for (int d = 0; d < 7; d++) {
    String key = "sched_" + String(d);
    prefs.putUInt(key.c_str(), config.schedule[d]);
  }
  prefs.putUInt("heat_watts", config.heaterWatts);
  prefs.putFloat("energy_price", config.energyPriceKwh);
  prefs.end();
  Serial.println("[CFG] Config saved to Preferences.");
}

// ------------ WiFi / MQTT / Ethernet ------------

bool tryEthernet() {
  // W5500 Ethernet support via SPI
  // This tries to detect a W5500 on the configured SPI pins.
  // If no W5500 is present, it returns false and WiFi is used instead.
  Serial.println("[ETH] Checking for W5500 Ethernet adapter...");
  
  // Initialize SPI for W5500
  SPI.begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
  pinMode(PIN_ETH_CS, OUTPUT);
  digitalWrite(PIN_ETH_CS, HIGH);
  
  // Reset W5500 if RST pin defined
  pinMode(PIN_ETH_RST, OUTPUT);
  digitalWrite(PIN_ETH_RST, LOW);
  delay(50);
  digitalWrite(PIN_ETH_RST, HIGH);
  delay(200);
  
  // Quick chip detection: read W5500 version register (0x0039 in common block)
  // W5500 should return 0x04
  digitalWrite(PIN_ETH_CS, LOW);
  SPI.transfer16(0x0039);  // Address high/low
  SPI.transfer(0x00);      // Control: common register, read
  uint8_t ver = SPI.transfer(0x00);
  digitalWrite(PIN_ETH_CS, HIGH);
  
  if (ver != 0x04) {
    Serial.printf("[ETH] W5500 not detected (version reg=0x%02X, expected 0x04)\n", ver);
    SPI.end();
    return false;
  }
  
  Serial.println("[ETH] W5500 detected! Attempting DHCP via ETH.begin()...");
  
  // Use ESP32 ETH library for W5500
  // API varies by ESP32 core version; try the common signature
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    // ESP32 Arduino Core 3.x uses different API
    if (!ETH.begin(W5500_PHY_ADDR, PIN_ETH_CS, PIN_ETH_INT, PIN_ETH_RST, SPI, PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI)) {
      Serial.println("[ETH] ETH.begin() failed");
      return false;
    }
  #else
    // ESP32 Arduino Core 2.x 
    // Note: The built-in ETH library on Core 2.x may not support W5500 natively.
    // In that case we fall through to WiFi.
    Serial.println("[ETH] W5500 detected but ESP32 Core 2.x - using WiFi instead (upgrade to Core 3.x for W5500 ETH support)");
    SPI.end();
    return false;
  #endif
  
  // Wait for DHCP (up to 5 seconds)
  unsigned long ethStart = millis();
  while (!ETH.localIP()[0] && (millis() - ethStart) < 5000) {
    delay(100);
  }
  
  if (ETH.localIP()[0]) {
    g_ethConnected = true;
    g_networkType = "ethernet";
    Serial.printf("[ETH] Connected! IP: %s\n", ETH.localIP().toString().c_str());
    return true;
  }
  
  Serial.println("[ETH] No DHCP address received, falling back to WiFi");
  return false;
}

void connectWiFi() {
  // Try Ethernet first
  if (tryEthernet()) {
    Serial.println("[NET] Using Ethernet (W5500)");
    if (!MDNS.begin(DEVICE_HOSTNAME)) {
      Serial.println("[MDNS] Error starting mDNS");
    } else {
      Serial.println(String("[MDNS] mDNS responder started as ") + DEVICE_HOSTNAME + ".local");
    }
    return;
  }
  
  // Fallback to WiFi
  g_networkType = "wifi";
  Serial.printf("[WIFI] Connecting to %s\n", config.wifiSsid.c_str());
  WiFi.mode(WIFI_STA);
  // Hostname for DHCP / mDNS
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());

  uint8_t retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] Connected, IP: ");
    Serial.println(WiFi.localIP());

    if (!MDNS.begin(DEVICE_HOSTNAME)) {
      Serial.println("[MDNS] Error starting mDNS");
    } else {
      Serial.println(String("[MDNS] mDNS responder started as ") + DEVICE_HOSTNAME + ".local");
    }
  } else {
    Serial.println("[WIFI] Failed to connect.");
  }
}

void setupOTA() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] WiFi not connected, OTA disabled.");
    return;
  }

  ArduinoOTA.setHostname(DEVICE_HOSTNAME);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS (LittleFS)
      type = "filesystem";
    }
    Serial.println("[OTA] Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] End");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");
}

String mqttTopic(const String &suffix) {
  if (config.mqttBaseTopic.endsWith("/")) {
    return config.mqttBaseTopic + suffix;
  }
  return config.mqttBaseTopic + "/" + suffix;
}

void publishStatus();
void setFanDirection(bool direction);

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String t = String(topic);
  String msg;
  for (unsigned int i = 0; i < length; ++i) msg += (char) payload[i];

  Serial.printf("[MQTT] Message on %s: %s\n", t.c_str(), msg.c_str());

  // Simple command interface, e.g. base/cmd/fan => "on"/"off"/"auto"
  if (t == mqttTopic("cmd/fan")) {
    if (msg == "on") {
      g_fanOn = true;
    } else if (msg == "off") {
      g_fanOn = false;
    }
  } else if (t == mqttTopic("cmd/heater")) {
    if (msg == "on") {
      g_heaterOn = true;
      config.heaterEnabled = true;
    } else if (msg == "off") {
      g_heaterOn = false;
      config.heaterEnabled = false;
    }
    saveConfigToPrefs();
  } else if (t == mqttTopic("cmd/manual")) {
    // Manual control via MQTT
    // Payload options:
    //  - "auto" -> return to automatic mode
    //  - "fan=on,heater=off,secs=300" (comma-separated key=value)
    msg.trim();
    msg.toLowerCase();

    if (msg == "auto" || msg == "off" || msg == "cancel") {
      g_controlMode = MODE_AUTO;
      return;
    }

    // Defaults: keep current outputs, 300s if not specified
    bool fanSet = false;
    bool heaterSet = false;
    unsigned long secs = 300;

    int start = 0;
    while (start < (int)msg.length()) {
      int comma = msg.indexOf(',', start);
      if (comma < 0) comma = msg.length();
      String part = msg.substring(start, comma);
      part.trim();
      int eq = part.indexOf('=');
      if (eq > 0) {
        String key = part.substring(0, eq);
        String val = part.substring(eq + 1);
        key.trim();
        val.trim();

        if (key == "fan") {
          fanSet = true;
          if (val == "on") g_manualFanOn = true;
          else if (val == "off") g_manualFanOn = false;
        } else if (key == "heater") {
          heaterSet = true;
          if (val == "on") g_manualHeaterOn = true;
          else if (val == "off") g_manualHeaterOn = false;
        } else if (key == "secs" || key == "seconds") {
          long s = val.toInt();
          if (s > 0) secs = (unsigned long)s;
        }
      }

      start = comma + 1;
    }

    if (!fanSet) g_manualFanOn = g_fanOn;
    if (!heaterSet) g_manualHeaterOn = g_heaterOn;

    g_controlMode = MODE_MANUAL;
    // Don't override permanent manual mode if no SHT31 sensor
    if (g_sht31Available) {
      g_manualUntilMillis = millis() + secs * 1000UL;
    }
  } else if (t == mqttTopic("cmd/absaugen")) {
    // MQTT command to start ABSAUGEN (suction) for X minutes
    // Payload: duration in minutes (e.g. "5"), or "off"/"stop" to cancel
    msg.trim();
    msg.toLowerCase();
    if (msg == "off" || msg == "stop" || msg == "cancel") {
      Serial.println("[MQTT] ABSAUGEN cancelled via MQTT");
      g_absaugenUntilMillis = 0;
      if (g_fanDirection) {
        setFanDirection(false); // Switch back to EINBLASEN
      }
    } else {
      // Block absaugen when schedule is inactive
      if (config.scheduleEnabled && !isScheduleActive()) {
        Serial.println("[MQTT] ABSAUGEN blocked - schedule inactive");
      } else {
        int mins = msg.toInt();
        if (mins <= 0) mins = config.absaugenDurationSecs / 60;
        if (mins < 1) mins = 5; // fallback default
        Serial.printf("[MQTT] ABSAUGEN started for %d minutes via MQTT\n", mins);
        setFanDirection(true); // Switch to ABSAUGEN
        g_absaugenUntilMillis = millis() + (unsigned long)mins * 60UL * 1000UL;
      }
    }
  } else if (t == mqttTopic("cmd/bath_pause")) {
    // MQTT command for bath pause
    // Payload: "on"/minutes or "off"/"stop"/"cancel"
    msg.trim();
    msg.toLowerCase();
    if (msg == "off" || msg == "stop" || msg == "cancel") {
      g_bathPauseActive = false;
      g_bathPauseUntilMillis = 0;
      Serial.println("[MQTT] Bath pause cancelled");
    } else {
      int mins = msg.toInt();
      if (mins <= 0) mins = config.bathPauseMins;
      g_bathPauseActive = true;
      g_bathPauseUntilMillis = millis() + (unsigned long)mins * 60UL * 1000UL;
      g_fanOn = false;
      g_heaterOn = false;
      setFanPWM(false);
      digitalWrite(PIN_FAN_RELAY, LOW);
      digitalWrite(PIN_HEATER_SSR, LOW);
      Serial.printf("[MQTT] Bath pause started for %d minutes\n", mins);
    }
  } else if (t == mqttTopic("cmd/auto")) {
    // MQTT command to toggle auto mode
    // Payload: "on"/"true"/"1" or "off"/"false"/"0" or "toggle"
    msg.trim();
    msg.toLowerCase();
    if (msg == "toggle") {
      config.autoEnabled = !config.autoEnabled;
    } else if (msg == "on" || msg == "true" || msg == "1") {
      config.autoEnabled = true;
    } else if (msg == "off" || msg == "false" || msg == "0") {
      config.autoEnabled = false;
    }
    saveConfigToPrefs();
    Serial.printf("[MQTT] Auto mode: %s\n", config.autoEnabled ? "ENABLED" : "DISABLED");
  } else if (t == mqttTopic("cmd/direction")) {
    // MQTT command to change fan direction
    // Payload: "absaugen"/"true" or "einblasen"/"false"
    msg.trim();
    msg.toLowerCase();
    if (msg == "absaugen" || msg == "true" || msg == "1") {
      if (config.scheduleEnabled && !isScheduleActive()) {
        Serial.println("[MQTT] Direction change blocked - schedule inactive");
      } else {
        setFanDirection(true);
        g_absaugenUntilMillis = millis() + (unsigned long)config.absaugenDurationSecs * 1000UL;
      }
    } else if (msg == "einblasen" || msg == "false" || msg == "0") {
      g_absaugenUntilMillis = 0;
      setFanDirection(false);
    }
  } else if (t == mqttTopic("cmd/fan_speed")) {
    // MQTT command to set fan speed (0-100)
    msg.trim();
    int speed = msg.toInt();
    if (speed >= 0 && speed <= 100) {
      g_fanSpeedPercent = (uint8_t)speed;
      g_lastSliderSpeed = g_fanSpeedPercent;
      if (g_fanOn) setFanSpeed(g_fanSpeedPercent);
      Serial.printf("[MQTT] Fan speed set to %d%%\n", speed);
    }
  }
}

void ensureMqttConnected() {
  if (mqttClient.connected()) {
    digitalWrite(PIN_STATUS_LED, HIGH); // MQTT OK
    return;
  }
  if (WiFi.status() != WL_CONNECTED && !g_ethConnected) {
    digitalWrite(PIN_STATUS_LED, LOW); // No network -> LED off
    return;
  }

  mqttClient.setBufferSize(1024);  // Default 256 is too small for status JSON!
  mqttClient.setServer(config.mqttHost.c_str(), config.mqttPort);
  mqttClient.setCallback(mqttCallback);

  String clientId = String("esp32-feuchte-") + String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.printf("[MQTT] Connecting to %s:%d ...\n", config.mqttHost.c_str(), config.mqttPort);

  bool ok;
  if (config.mqttUser.length() > 0) {
    ok = mqttClient.connect(clientId.c_str(), config.mqttUser.c_str(), config.mqttPassword.c_str());
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }

  if (ok) {
    Serial.println("[MQTT] Connected");
    mqttClient.subscribe(mqttTopic("cmd/#").c_str());
    digitalWrite(PIN_STATUS_LED, HIGH); // MQTT connected -> LED on
  } else {
    Serial.printf("[MQTT] Failed, rc=%d\n", mqttClient.state());
    digitalWrite(PIN_STATUS_LED, LOW); // Connection failed -> LED off
  }
}

// ------------ Sensor + control logic ------------

// Read temperature from a specific DS18B20 by ROM address using raw OneWire commands.
// Call AFTER issuing a global CONVERT_T and waiting for conversion.
// Returns NAN on failure.
float readDS18B20Raw(const uint8_t *addr) {
  if (!oneWire.reset()) return NAN;
  oneWire.select(addr);       // MATCH_ROM - select specific sensor
  oneWire.write(0xBE);        // READ_SCRATCHPAD

  uint8_t data[9];
  for (int i = 0; i < 9; i++) {
    data[i] = oneWire.read();
  }

  // Verify CRC
  if (OneWire::crc8(data, 8) != data[8]) return NAN;

  int16_t raw = (data[1] << 8) | data[0];
  // Default 12-bit resolution: divide by 16
  float tempC = (float)raw / 16.0f;

  // Sanity check
  if (tempC < -55.0f || tempC > 125.0f) return NAN;
  return tempC;
}

void updateSensor() {
  float t = NAN, h = NAN;

  // Only read SHT31 if it was successfully initialized
  if (g_sht31Available) {
    double tt = sht31.readTemperature();
    double hh = sht31.readHumidity();

    if (!isnan(tt) && !isnan(hh)) {
      t = (float)tt;
      h = (float)hh;
    }

    if (!isnan(t) && !isnan(h)) {
      g_temperatureC = t;
      g_humidity = h;
    }
  }
  // If SHT31 not available, temperature and humidity remain NAN

  // Read DS18B20 temperatures using raw OneWire commands (no DallasTemperature re-enumeration)
  // Step 1: Issue global CONVERT_T to all sensors on the bus
  oneWire.reset();
  oneWire.write(0xCC);  // SKIP_ROM - address all devices
  oneWire.write(0x44);  // CONVERT_T - start temperature conversion
  delay(750);           // Wait for 12-bit conversion (750ms max)

  if (ds18b20_assigned) {
    g_tempBeforeFanC = readDS18B20Raw(ds18b20_beforeFan);
    g_tempAfterHeaterC = readDS18B20Raw(ds18b20_afterHeater);
  } else {
    g_tempBeforeFanC = NAN;
    g_tempAfterHeaterC = NAN;
  }
}

// Forward declarations
void setIBT2FanSpeed(uint8_t percent, bool direction);
void startDirectionChange(bool newDirection);

// ------------ PWM Fan Control ------------

void setupFanPWM() {
  Serial.printf("[PWM] Setting up PWM: RPWM Channel=%d, LPWM Channel=%d, Freq=%dHz, Resolution=%d bits\n", 
                FAN_RPWM_CHANNEL, FAN_LPWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  Serial.printf("[PWM] Using 25kHz frequency for smooth motor operation\n");
  
  // Setup RPWM channel
  ledcSetup(FAN_RPWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  Serial.printf("[PWM] Attaching RPWM to GPIO%d (Channel %d)\n", PIN_IBT2_RPWM, FAN_RPWM_CHANNEL);
  ledcAttachPin(PIN_IBT2_RPWM, FAN_RPWM_CHANNEL);
  
  // Setup LPWM channel
  ledcSetup(FAN_LPWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  Serial.printf("[PWM] Attaching LPWM to GPIO%d (Channel %d)\n", PIN_IBT2_LPWM, FAN_LPWM_CHANNEL);
  ledcAttachPin(PIN_IBT2_LPWM, FAN_LPWM_CHANNEL);
  
  // Initialize IBT-2 control pin - ALWAYS ON
  pinMode(PIN_IBT2_EN, OUTPUT);
  digitalWrite(PIN_IBT2_EN, HIGH);
  Serial.printf("[PWM] Enable pin GPIO%d set to HIGH (always on)\n", PIN_IBT2_EN);
  
  // Test PWM output on both channels
  Serial.println("[PWM] Testing PWM output...");
  ledcWrite(FAN_RPWM_CHANNEL, 128); // 50% duty on RPWM (8-bit: 128/255)
  ledcWrite(FAN_LPWM_CHANNEL, 0);    // 0% duty on LPWM
  delay(1000);
  ledcWrite(FAN_RPWM_CHANNEL, 0);    // 0% duty on RPWM
  ledcWrite(FAN_LPWM_CHANNEL, 128);  // 50% duty on LPWM (8-bit: 128/255)
  delay(1000);
  ledcWrite(FAN_RPWM_CHANNEL, 0);    // Stop both
  ledcWrite(FAN_LPWM_CHANNEL, 0);
  
  Serial.println("[PWM] IBT-2 Fan PWM initialized with 25kHz frequency");
}

void setIBT2FanSpeed(uint8_t percent, bool direction) {
  // Clamp percent to 0-100 range
  if (percent > 100) percent = 100;
  
  // CRITICAL: Do NOT modify g_fanSpeedPercent or g_fanDirection here!
  // Direction changes are handled by startDirectionChange/updateDirectionChange
  // Only update on/off state
  g_fanOn = (percent > 0);
  
  if (g_fanOn && percent > 0) {
    // Enforce minimum PWM
    if (percent < FAN_MIN_PWM_PERCENT) {
      Serial.printf("[FAN] PWM %d%% below minimum, clamping to %d%%\n", percent, FAN_MIN_PWM_PERCENT);
      percent = FAN_MIN_PWM_PERCENT;
    }
    
    // Direct PWM output - NO SOFT START
    // Enable IBT-2 (always on now)
    digitalWrite(PIN_IBT2_EN, HIGH);
    
    // Set PWM speed with 8-bit resolution
    uint32_t duty = (percent * 255) / 100;
    
    if (direction) {
      // Forward: RPWM = PWM, LPWM = 0 (ABSAUGEN)
      ledcWrite(FAN_RPWM_CHANNEL, duty);
      ledcWrite(FAN_LPWM_CHANNEL, 0);
    } else {
      // Reverse: RPWM = 0, LPWM = PWM (EINBLASEN)
      ledcWrite(FAN_RPWM_CHANNEL, 0);
      ledcWrite(FAN_LPWM_CHANNEL, duty);
    }
  } else {
    // Stop motor PWM
    digitalWrite(PIN_IBT2_EN, LOW);
    ledcWrite(FAN_RPWM_CHANNEL, 0);
    ledcWrite(FAN_LPWM_CHANNEL, 0);
    // Do NOT turn off relay during direction change - only when fan is actually OFF
    if (!g_directionChangeInProgress) {
      digitalWrite(PIN_FAN_RELAY, LOW); // Turn off relay only if not changing direction
    }
  }
}

void setFanDirection(bool direction) {
  // Turn off heater when switching to ABSAUGEN mode
  if (direction && g_heaterOn) {
    g_heaterOn = false;
    g_manualHeaterOn = false;
  }
  
  // Simple immediate direction change
  startDirectionChange(direction);
}

void setFanSpeed(uint8_t percent) {
  // If direction change is in progress, update saved speed but don't apply yet
  if (g_directionChangeInProgress) {
    g_savedSpeed = percent;
    Serial.printf("[FAN] Speed change during direction change: saved speed updated to %d%% (will apply after)\n", percent);
    return;
  }
  
  // Enforce minimum PWM if fan is on
  if (percent > 0 && percent < FAN_MIN_PWM_PERCENT) {
    Serial.printf("[FAN] Speed %d%% below minimum, clamping to %d%%\n", percent, FAN_MIN_PWM_PERCENT);
    percent = FAN_MIN_PWM_PERCENT;
  }
  
  // Apply speed directly to hardware without modifying g_fanSpeedPercent
  digitalWrite(PIN_IBT2_EN, HIGH);
  uint32_t duty = (percent * 255) / 100;
  if (g_fanDirection) {
    ledcWrite(FAN_RPWM_CHANNEL, duty);
    ledcWrite(FAN_LPWM_CHANNEL, 0);
  } else {
    ledcWrite(FAN_RPWM_CHANNEL, 0);
    ledcWrite(FAN_LPWM_CHANNEL, duty);
  }
  Serial.printf("[FAN] Speed applied: %d%% (g_fanSpeedPercent=%d%% UNCHANGED)\n", percent, g_fanSpeedPercent);
}

void setFanPWM(bool fanOn) {
  // If direction change is in progress, keep PWM at 0 but do NOT touch relay or g_fanOn
  if (g_directionChangeInProgress) {
    ledcWrite(FAN_RPWM_CHANNEL, 0);
    ledcWrite(FAN_LPWM_CHANNEL, 0);
    return;
  }
  
  // IBT-2 control: Use setIBT2FanSpeed instead
  if (fanOn && g_fanSpeedPercent > 0) {
    setIBT2FanSpeed(g_fanSpeedPercent, g_fanDirection);
  } else {
    setIBT2FanSpeed(0, g_fanDirection);
  }
}

// Reset fan speed to default when fan is turned on
void resetFanSpeedToDefault() {
  // Only reset if user explicitly requests it via settings
  // Do NOT automatically reset during operation
  Serial.printf("[FAN] Current speed: %d%%, Default: %d%% (not auto-resetting)\n", 
                g_fanSpeedPercent, config.fanSpeedPercent);
}

// ------------ NTP / Schedule ------------

void setupNTP() {
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
  Serial.println("[NTP] Time sync started (CET/CEST)");
}

bool isScheduleActive() {
  if (!config.scheduleEnabled) return true;  // Schedule disabled = always active
  if (!g_ntpSynced) return true;             // No time yet = assume active
  
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) return true;  // Can't get time = assume active
  
  int wday = timeinfo.tm_wday;  // 0=Sunday, 1=Monday, ..., 6=Saturday
  int hour = timeinfo.tm_hour;  // 0-23
  
  return (config.schedule[wday] & (1UL << hour)) != 0;
}

void applyControl() {
  unsigned long now = millis();

  // BATH PAUSE CHECK - entire system inactive UNLESS temp safety override is active
  if (g_bathPauseActive) {
    if (now >= g_bathPauseUntilMillis) {
      g_bathPauseActive = false;
      Serial.println("[BATH-PAUSE] Ended, resuming normal operation");
    } else if (!g_tempSafetyFanForced) {
      // Everything off during bath pause - but ONLY if no overheat protection active
      g_fanOn = false;
      g_heaterOn = false;
      setFanPWM(false);
      digitalWrite(PIN_FAN_RELAY, LOW);
      digitalWrite(PIN_HEATER_SSR, LOW);
      return;
    } else {
      // Overheat protection active during bath pause: fan MUST run, heater OFF
      g_fanOn = true;
      g_heaterOn = false;
      setFanPWM(true);
      digitalWrite(PIN_FAN_RELAY, HIGH);
      digitalWrite(PIN_HEATER_SSR, LOW);
      Serial.println("[BATH-PAUSE] Overridden by overheat protection - fan forced ON");
      return;
    }
  }

  // TEMPERATURE SAFETY CHECK - ALWAYS ACTIVE (even when auto disabled, manual, schedule off)
  // Must run BEFORE auto-enabled check so fan always runs for safety
  float tempDiff = g_tempAfterHeaterC - g_tempBeforeFanC;
  if (!isnan(tempDiff)) {
    if (!g_tempSafetyFanForced && tempDiff >= config.tempSafetyDiff) {
      g_tempSafetyFanForced = true;
      Serial.printf("[TEMP-SAFETY] Fan FORCED ON: diff %.1f°C >= %.1f°C (before: %.1f°C, after: %.1f°C)\n",
                    tempDiff, config.tempSafetyDiff, g_tempBeforeFanC, g_tempAfterHeaterC);
    } else if (g_tempSafetyFanForced && tempDiff < (config.tempSafetyDiff - config.tempSafetyHyst)) {
      g_tempSafetyFanForced = false;
      g_fanOn = false;
      g_heaterOn = false;
      g_fanWasOn = false;
      Serial.printf("[TEMP-SAFETY] Fan released: diff %.1f°C < %.1f°C, fan/heater reset\n",
                    tempDiff, config.tempSafetyDiff - config.tempSafetyHyst);
    }
  }

  // ABSOLUTE HEATER TEMPERATURE LIMIT (configurable, default 55°C) - bit-bang SSR off, don't clear manual command
  // This is a hardware safety: if air after heater exceeds limit, cut heater immediately
  if (!isnan(g_tempAfterHeaterC)) {
    float heaterLimit = config.heaterMaxTemp;
    float heaterHyst = 3.0f;  // 3°C hysteresis
    if (g_tempAfterHeaterC >= heaterLimit) {
      if (!g_heaterOvertemp) {
        g_heaterOvertemp = true;
        Serial.printf("[HEATER-LIMIT] Air after heater %.1f°C >= %.1f°C, SSR forced OFF\n", g_tempAfterHeaterC, heaterLimit);
      }
      digitalWrite(PIN_HEATER_SSR, LOW);  // Immediate SSR off
    } else if (g_heaterOvertemp && g_tempAfterHeaterC < (heaterLimit - heaterHyst)) {
      g_heaterOvertemp = false;
      Serial.printf("[HEATER-LIMIT] Air after heater %.1f°C < %.1f°C, heater allowed again\n", g_tempAfterHeaterC, heaterLimit - heaterHyst);
    }
  }

  // AUTO ENABLED CHECK - if auto is disabled, only manual mode works
  // BUT: temp safety still forces fan on even when auto is off
  if (!config.autoEnabled && g_controlMode != MODE_MANUAL) {
    if (g_tempSafetyFanForced) {
      // Safety override: fan ON, heater OFF
      g_fanOn = true;
      g_heaterOn = false;
      setFanPWM(true);
      digitalWrite(PIN_FAN_RELAY, HIGH);
      digitalWrite(PIN_HEATER_SSR, LOW);
    } else {
      g_fanOn = false;
      g_heaterOn = false;
      setFanPWM(false);
      digitalWrite(PIN_FAN_RELAY, LOW);
      digitalWrite(PIN_HEATER_SSR, LOW);
    }
    return;
  }

  // If we are in manual mode, honor manual outputs until timeout
  if (g_controlMode == MODE_MANUAL) {
    if (!g_sht31Available || g_manualUntilMillis == 0xFFFFFFFF) {
      // Permanent manual mode (no SHT31 or explicit permanent)
      g_fanOn = g_manualFanOn;
      g_heaterOn = g_manualHeaterOn;
      
      // Safety: if heater is manually turned on, force fan on
      if (g_manualHeaterOn) {
        g_fanOn = true;
      }
      // Temperature safety override: force fan on
      if (g_tempSafetyFanForced) {
        g_fanOn = true;
      }
      
      setFanPWM(g_fanOn);
      digitalWrite(PIN_FAN_RELAY, g_fanOn ? HIGH : LOW);
      bool heaterActuallyOn = g_heaterOn && g_fanOn && !g_heaterOvertemp;
      digitalWrite(PIN_HEATER_SSR, heaterActuallyOn ? HIGH : LOW);
      
      // Track heater ON time for manual mode
      if (heaterActuallyOn && !g_heaterWasOn) {
        g_heaterOnStartMillis = millis();
      } else if (!heaterActuallyOn && g_heaterWasOn) {
        g_heaterOnTotalSecs += (millis() - g_heaterOnStartMillis) / 1000UL;
      }
      g_heaterWasOn = heaterActuallyOn;
      return;
    } else if (now >= g_manualUntilMillis) {
      g_controlMode = MODE_AUTO;
      g_fanOn = false;
      g_heaterOn = false;
      g_fanWasOn = false;
      // Accumulate any remaining heater time before switching to AUTO
      if (g_heaterWasOn) {
        g_heaterOnTotalSecs += (millis() - g_heaterOnStartMillis) / 1000UL;
        g_heaterWasOn = false;
      }
      Serial.println("[CTRL] Manual timer expired -> AUTO mode, fan/heater reset");
    } else {
      // Temporary manual mode (with timer)
      g_fanOn = g_manualFanOn;
      g_heaterOn = g_manualHeaterOn;
      
      // Safety: if heater is manually turned on, force fan on
      if (g_manualHeaterOn) {
        g_fanOn = true;
      }
      // Temperature safety override: force fan on
      if (g_tempSafetyFanForced) {
        g_fanOn = true;
      }
      
      setFanPWM(g_fanOn);
      digitalWrite(PIN_FAN_RELAY, g_fanOn ? HIGH : LOW);
      bool heaterActuallyOn = g_heaterOn && g_fanOn && !g_heaterOvertemp;
      digitalWrite(PIN_HEATER_SSR, heaterActuallyOn ? HIGH : LOW);
      
      // Track heater ON time for manual mode
      if (heaterActuallyOn && !g_heaterWasOn) {
        g_heaterOnStartMillis = millis();
      } else if (!heaterActuallyOn && g_heaterWasOn) {
        g_heaterOnTotalSecs += (millis() - g_heaterOnStartMillis) / 1000UL;
      }
      g_heaterWasOn = heaterActuallyOn;
      return;
    }
  }

  // Schedule check: if schedule is active and current hour is not enabled, turn everything off
  // BUT: temperature safety overrides schedule
  if (!isScheduleActive()) {
    g_fanOn = g_tempSafetyFanForced;  // Only run fan if temp safety requires it
    g_heaterOn = false;
    setFanPWM(g_fanOn);
    digitalWrite(PIN_FAN_RELAY, g_fanOn ? HIGH : LOW);
    digitalWrite(PIN_HEATER_SSR, LOW);
    if (!g_fanOn) {
      g_fanWasOn = false;
    }
    return;
  }

  // Cyclic timer: periodic fan runs regardless of humidity
  // Only active when schedule allows it (wochenplan) and cyclic is enabled
  // Cycles start ONLY at fixed quarter-hour times based on cyclicStartMin and intervalMins
  // Example: startMin=0, interval=60 -> starts at :00 every hour
  // Example: startMin=15, interval=30 -> starts at :15 and :45
  if (config.cyclicEnabled) {
    if (!g_ntpSynced) {
      // NTP not synced yet - do NOT start any cycles, wait for time sync
      // This prevents random starts before we know the actual time
    } else {
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 0)) {
        int currentMin = timeinfo.tm_min;
        int currentSec = timeinfo.tm_sec;
        int startMin = config.cyclicStartMin;  // 0, 15, 30, or 45
        int intervalMins = config.cyclicIntervalMins;
        
        // Calculate total minutes in hour from startMin with interval steps
        // Valid start minutes are: startMin, startMin+interval, startMin+2*interval, etc. (mod 60)
        bool isValidStartMinute = false;
        for (int m = startMin; m < 60; m += intervalMins) {
          if (currentMin == m) {
            isValidStartMinute = true;
            break;
          }
        }
        
        // Only start in the first 30 seconds of a valid start minute
        bool atCycleStart = isValidStartMinute && (currentSec < 30);
        
        if (g_cyclicFanOn) {
          // Currently in a cyclic run - check if duration expired
          if (now >= g_cyclicRunUntilMillis) {
            g_cyclicFanOn = false;
            g_cyclicLastOffMillis = now;
            Serial.printf("[CYCLIC] Run ended at %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
          }
        } else {
          // Not in a cyclic run - check if we're at a valid start time
          // Also ensure at least 60 seconds since last cycle ended (prevent double-start)
          if (atCycleStart && (g_cyclicLastOffMillis == 0 || (now - g_cyclicLastOffMillis) >= 60000UL)) {
            g_cyclicFanOn = true;
            g_cyclicRunUntilMillis = now + (unsigned long)config.cyclicDurationMins * 60UL * 1000UL;
            Serial.printf("[CYCLIC] Starting %d min run at %02d:%02d (startMin=%d, interval=%d, heater=%s)\n",
                          config.cyclicDurationMins, timeinfo.tm_hour, timeinfo.tm_min,
                          startMin, intervalMins, config.cyclicHeater ? "yes" : "no");
          }
        }
      }
    }
  } else {
    // Cyclic disabled - reset state
    if (g_cyclicFanOn) {
      g_cyclicFanOn = false;
      g_cyclicLastOffMillis = now;
    }
  }

  // ABSAUGEN cyclic timer: periodic direction reversal (only when schedule active)
  if (config.absaugenCyclicEnabled && isScheduleActive()) {
    if (g_absaugenCyclicRunning) {
      // Currently in an ABSAUGEN cyclic run - check if duration expired
      if (now >= g_absaugenCyclicRunUntilMillis) {
        g_absaugenCyclicRunning = false;
        g_fanOn = true; // Keep fan on after returning to EINBLASEN
        g_absaugenCyclicLastOffMillis = now;
        g_absaugenUntilMillis = 0;
        // Restore standard EINBLASEN speed from config
        g_fanSpeedPercent = config.fanSpeedPercent;
        g_lastSliderSpeed = g_fanSpeedPercent;
        if (g_fanDirection) {
          setFanDirection(false); // Switch back to EINBLASEN
        }
        Serial.printf("[ABSAUGEN-CYCLIC] Run ended, restored speed %d%%, next in %d min\n", g_fanSpeedPercent, config.absaugenCyclicIntervalMins);
      }
    } else {
      // Not in a run - check if interval has passed
      unsigned long intervalMs = (unsigned long)config.absaugenCyclicIntervalMins * 60UL * 1000UL;
      if (g_absaugenCyclicLastOffMillis == 0 || (now - g_absaugenCyclicLastOffMillis) >= intervalMs) {
        g_absaugenCyclicRunning = true;
        unsigned long durationMs = (unsigned long)config.absaugenCyclicDurationMins * 60UL * 1000UL;
        g_absaugenCyclicRunUntilMillis = now + durationMs;
        g_absaugenUntilMillis = now + durationMs;
        g_fanOn = true; // Force fan ON during absaugen
        setFanDirection(true); // Switch to ABSAUGEN
        Serial.printf("[ABSAUGEN-CYCLIC] Starting %d min ABSAUGEN run\n", config.absaugenCyclicDurationMins);
      }
    }
  } else {
    // Absaugen cyclic disabled - reset state
    if (g_absaugenCyclicRunning) {
      g_absaugenCyclicRunning = false;
      g_absaugenCyclicLastOffMillis = now;
      g_absaugenUntilMillis = 0;
      if (g_fanDirection) {
        setFanDirection(false);
      }
    }
  }

  // Automatic control requires valid humidity data
  if (isnan(g_humidity)) {
    // Still apply current state to hardware so we don't leave outputs stuck
    if (g_tempSafetyFanForced) g_fanOn = true;
    setFanPWM(g_fanOn);
    digitalWrite(PIN_FAN_RELAY, g_fanOn ? HIGH : LOW);
    digitalWrite(PIN_HEATER_SSR, (g_heaterOn && !g_heaterOvertemp) ? HIGH : LOW);
    return;
  }

  // Simple hysteresis on humidity
  if (!g_fanOn && g_humidity >= config.humidityOn) {
    g_fanOn = true;
  } else if (g_fanOn && !g_cyclicFanOn && g_humidity <= config.humidityOff) {
    g_fanOn = false;
  }

  // Cyclic timer override: force fan on during cyclic run with custom speed
  if (g_cyclicFanOn) {
    g_fanOn = true;
    g_fanSpeedPercent = config.cyclicSpeedPercent;
  }
  
  // Absaugen cyclic override: force fan on during absaugen cyclic run
  if (g_absaugenCyclicRunning) {
    g_fanOn = true;
  }
  
  // ABSAUGEN mode: fan MUST be ON and use separate speed
  if (g_fanDirection && g_absaugenUntilMillis > 0) {
    g_fanOn = true;
    g_fanSpeedPercent = config.absaugenSpeedPercent;
  }
  
  // Track fan on/off edge for humidity descent monitoring
  if (g_fanOn && !g_fanWasOn) {
    // Fan just turned on - start humidity descent tracking
    g_humDescentStartHumidity = g_humidity;
    g_humDescentStartMillis = now;
    g_humDescentHeaterForced = false;
    Serial.printf("[HUM-DESCENT] Tracking started at %.1f%%\n", g_humidity);
  } else if (!g_fanOn && g_fanWasOn) {
    // Fan just turned off - reset descent tracking
    g_humDescentStartHumidity = NAN;
    g_humDescentStartMillis = 0;
    g_humDescentHeaterForced = false;
  }
  g_fanWasOn = g_fanOn;

  // Humidity descent check: if fan is running and humidity drops too slowly, force heater
  if (g_fanOn && !g_fanDirection && config.humDescentHeaterEnable && !isnan(g_humidity) && !isnan(g_humDescentStartHumidity)) {
    unsigned long elapsed = now - g_humDescentStartMillis;
    if (elapsed >= (unsigned long)config.humDescentTimeSecs * 1000UL) {
      float drop = g_humDescentStartHumidity - g_humidity;
      if (drop < config.humDescentMinDrop) {
        if (!g_humDescentHeaterForced) {
          g_humDescentHeaterForced = true;
          Serial.printf("[HUM-DESCENT] Humidity dropped only %.1f%% in %ds (need %.1f%%) -> heater ON\n",
                        drop, config.humDescentTimeSecs, config.humDescentMinDrop);
        }
      } else {
        // Humidity is dropping fast enough - reset tracking window
        g_humDescentStartHumidity = g_humidity;
        g_humDescentStartMillis = now;
        if (g_humDescentHeaterForced) {
          g_humDescentHeaterForced = false;
          Serial.printf("[HUM-DESCENT] Humidity dropping well (%.1f%% in window) -> heater released\n", drop);
        }
      }
    }
  } else {
    if (g_humDescentHeaterForced) {
      g_humDescentHeaterForced = false;
    }
  }

  // Heater logic with temperature hysteresis
  // Priority order (highest first):
  //   1. Wall temp diff (wallTemp - airTemp >= threshold) - HIGHEST PRIORITY
  //   2. Wall temp below minimum
  //   3. Humidity descent too slow (fan running but humidity not dropping)
  // Heater allowed when: enabled, fan running, EINBLASEN mode (not absaugen)
  bool cyclicHeaterForced = g_cyclicFanOn && config.cyclicHeaterForce;
  if (config.heaterEnabled && g_fanOn && !g_fanDirection) {
    if (cyclicHeaterForced) {
      // Cyclic heater force: heater always ON during cyclic run, no temp check
      if (!g_heaterOn) {
        g_heaterOn = true;
        Serial.println("[HEATER] FORCED ON by cyclic timer");
      }
    } else {
      float roomTemp = g_tempAfterHeaterC;   // DS18B20 after heater = room temperature
      float airTemp = g_tempBeforeFanC;      // DS18B20 before fan = incoming air (Zuluft) temperature
      
      // Condition 1 (HIGHEST): temp diff threshold
      // Heater ON when: Zuluft minus Raumtemp >= threshold (incoming air is warmer than room)
      bool diffConditionOn = false;
      bool diffConditionOff = false;
      if (!isnan(roomTemp) && !isnan(airTemp)) {
        float diff = airTemp - roomTemp;  // positive = incoming air warmer than room
        diffConditionOn = (diff >= config.heaterOnTempDiff);
        diffConditionOff = (diff <= config.heaterOffTempDiff);
      }
      
      // Condition 2: room temp below minimum (0 = disabled)
      bool roomColdCondition = (!isnan(roomTemp) && config.heaterMinRoomTemp > 0.0f && roomTemp < config.heaterMinRoomTemp);
      
      // Condition 3: humidity descent too slow
      bool humDescentCondition = g_humDescentHeaterForced;
      
      if (!g_heaterOn && (diffConditionOn || roomColdCondition || humDescentCondition)) {
        g_heaterOn = true;
        if (diffConditionOn) {
          float diff = airTemp - roomTemp;
          Serial.printf("[HEATER] ON: air-room diff %.1f°C >= %.1f°C (Zuluft wärmer als Raum)\n", diff, config.heaterOnTempDiff);
        } else if (roomColdCondition) {
          Serial.printf("[HEATER] ON: room %.1f°C < min %.1f°C\n", roomTemp, config.heaterMinRoomTemp);
        } else {
          Serial.println("[HEATER] ON: humidity descent too slow");
        }
      } else if (g_heaterOn) {
        // OFF conditions: air-room diff is highest priority for keeping heater on
        // Only turn off if diff condition says off AND room is warm enough AND humidity descent is OK
        bool canTurnOff = true;
        if (diffConditionOn) canTurnOff = false;  // Diff still high -> keep on
        if (roomColdCondition) canTurnOff = false; // Room still cold -> keep on
        if (humDescentCondition && !diffConditionOff) canTurnOff = false; // Humidity still slow and diff not below off threshold
        
        if (canTurnOff && (diffConditionOff || (isnan(roomTemp) || isnan(airTemp)))) {
          g_heaterOn = false;
          if (!isnan(roomTemp) && !isnan(airTemp)) {
            float diff = airTemp - roomTemp;
            Serial.printf("[HEATER] OFF: air-room diff %.1f°C <= %.1f°C, room warm, humidity OK\n", diff, config.heaterOffTempDiff);
          } else {
            Serial.println("[HEATER] OFF: no valid temp data");
          }
        }
      }
    }
  } else {
    g_heaterOn = false;
  }
  
  // Temperature safety override (already computed at top of applyControl)
  if (g_tempSafetyFanForced) {
    g_fanOn = true;
    g_heaterOn = false;  // Heater MUST be off during temp safety!
  }

  // Set fan relay and PWM based on fan state
  setFanPWM(g_fanOn);
  digitalWrite(PIN_FAN_RELAY, g_fanOn ? HIGH : LOW);  // Set relay AFTER PWM to override direction change logic
  digitalWrite(PIN_HEATER_SSR, (g_heaterOn && !g_heaterOvertemp) ? HIGH : LOW);
  
  // Track heater ON time (seconds)
  bool heaterActuallyOn = g_heaterOn && !g_heaterOvertemp;
  if (heaterActuallyOn && !g_heaterWasOn) {
    // Heater just turned ON - start tracking
    g_heaterOnStartMillis = millis();
  } else if (!heaterActuallyOn && g_heaterWasOn) {
    // Heater just turned OFF - accumulate time
    unsigned long onDuration = (millis() - g_heaterOnStartMillis) / 1000UL;
    g_heaterOnTotalSecs += onDuration;
  }
  g_heaterWasOn = heaterActuallyOn;
}

void recordHumidityHistory() {
  // Record history if we have either humidity data (SHT31) or temperature data (DS18B20)
  if (isnan(g_humidity) && isnan(g_temperatureC) && isnan(g_tempBeforeFanC) && isnan(g_tempAfterHeaterC)) return;
  
  // Need NTP sync for epoch-based timestamps
  if (!g_ntpSynced) return;

  unsigned long now = millis();
  // Add a sample at most once per minute (handles millis() overflow correctly with unsigned subtraction)
  unsigned long elapsed = now - g_lastHistorySampleMillis;
  if (elapsed < 60000UL && g_historyCount > 0) return;

  g_lastHistorySampleMillis = now;

  time_t nowEpoch;
  time(&nowEpoch);
  
  g_history[g_historyIndex].epochAt = (uint32_t)nowEpoch;
  g_history[g_historyIndex].humidity = g_humidity;
  // Use available temperature: prefer SHT31, fallback to DS18B20 temp before fan
  g_history[g_historyIndex].temperature = isnan(g_temperatureC) ? g_tempBeforeFanC : g_temperatureC;
  g_history[g_historyIndex].fanOn = g_fanOn;
  g_history[g_historyIndex].heaterOn = g_heaterOn;
  g_history[g_historyIndex].fanDirection = g_fanDirection;

  g_historyIndex = (g_historyIndex + 1) % HISTORY_CAPACITY;
  if (g_historyCount < HISTORY_CAPACITY) {
    g_historyCount++;
  }
}

// Forward declaration so publishStatus() can use it
static void buildStatusJson(StaticJsonDocument<2048> &doc);

void publishStatus() {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<2048> doc;
  buildStatusJson(doc);

  char buf[2048];
  size_t len = serializeJson(doc, buf);
  // Use string-based publish overload (topic, payload)
  mqttClient.publish(mqttTopic("status").c_str(), buf);
}

// Build a JSON status document compatible with publishStatus() output
static void buildStatusJson(StaticJsonDocument<2048> &doc) {
  doc["temperature_c"] = g_temperatureC;
  doc["humidity"] = g_humidity;
  doc["temp_before_fan_c"] = g_tempBeforeFanC;
  doc["temp_after_heater_c"] = g_tempAfterHeaterC;
  doc["fan_on"] = g_fanOn;
  doc["heater_on"] = g_heaterOn;
  doc["fan_speed_percent"] = g_fanSpeedPercent;
  doc["fan_direction"] = g_fanDirection;
  doc["control_mode"] = (g_controlMode == MODE_MANUAL) ? "manual" : "auto";
  doc["sht31_available"] = g_sht31Available;
  doc["schedule_active"] = isScheduleActive();
  doc["schedule_enabled"] = config.scheduleEnabled;
  doc["ntp_synced"] = g_ntpSynced;
  doc["temp_safety_active"] = g_tempSafetyFanForced;
  doc["heater_overtemp"] = g_heaterOvertemp;
  doc["heater_enabled"] = config.heaterEnabled;
  doc["direction_change_active"] = g_directionChangeInProgress;
  doc["humidity_active"] = (!isnan(g_humidity) && g_humidity > config.humidityOff);
  doc["cyclic_enabled"] = config.cyclicEnabled;
  doc["cyclic_fan_on"] = g_cyclicFanOn;
  if (g_cyclicFanOn && g_cyclicRunUntilMillis > millis()) {
    doc["cyclic_secs_remaining"] = (g_cyclicRunUntilMillis - millis()) / 1000UL;
  } else if (config.cyclicEnabled && !g_cyclicFanOn && g_ntpSynced) {
    // Calculate seconds until next valid start minute
    struct tm ti;
    if (getLocalTime(&ti, 0)) {
      int currentMin = ti.tm_min;
      int currentSec = ti.tm_sec;
      int startMin = config.cyclicStartMin;
      int intervalMins = config.cyclicIntervalMins;
      
      // Find next valid start minute
      int nextStartMin = -1;
      for (int m = startMin; m < 60; m += intervalMins) {
        if (m > currentMin || (m == currentMin && currentSec < 30)) {
          nextStartMin = m;
          break;
        }
      }
      // If no valid minute found in this hour, next is startMin of next hour
      int secsToNext;
      if (nextStartMin >= 0) {
        secsToNext = (nextStartMin - currentMin) * 60 - currentSec;
      } else {
        // Next hour at startMin
        secsToNext = (60 - currentMin + startMin) * 60 - currentSec;
      }
      if (secsToNext < 0) secsToNext = 0;
      doc["cyclic_next_secs"] = secsToNext;
    }
  } else if (config.cyclicEnabled && !g_cyclicFanOn && !g_ntpSynced) {
    doc["cyclic_next_secs"] = -1;  // Waiting for NTP
  }
  if (g_ntpSynced) {
    struct tm ti;
    if (getLocalTime(&ti, 0)) {
      char tbuf[32];
      const char* wdays[] = {"So","Mo","Di","Mi","Do","Fr","Sa"};
      snprintf(tbuf, sizeof(tbuf), "%s, %02d.%02d.%04d %02d:%02d:%02d",
               wdays[ti.tm_wday], ti.tm_mday, ti.tm_mon+1, ti.tm_year+1900,
               ti.tm_hour, ti.tm_min, ti.tm_sec);
      doc["current_time"] = tbuf;
    }
  }

  if (g_controlMode == MODE_MANUAL) {
    if (!g_sht31Available) {
      // No SHT31 sensor - permanent manual mode
      doc["manual_secs_remaining"] = 999999;
    } else if (g_manualUntilMillis == 0xFFFFFFFF) {
      // Permanent manual mode (other reasons)
      doc["manual_secs_remaining"] = 999999;
    } else {
      unsigned long now = millis();
      long remaining = (long)((g_manualUntilMillis > now) ? (g_manualUntilMillis - now) / 1000UL : 0);
      doc["manual_secs_remaining"] = remaining;
    }
  } else {
    doc["manual_secs_remaining"] = 0;
  }

  // Absaugen timer remaining
  if (g_fanDirection && g_absaugenUntilMillis > 0 && g_absaugenUntilMillis > millis()) {
    doc["absaugen_secs_remaining"] = (g_absaugenUntilMillis - millis()) / 1000UL;
  } else {
    doc["absaugen_secs_remaining"] = 0;
  }

  // Absaugen cyclic timer status
  doc["auto_enabled"] = config.autoEnabled;
  doc["network_type"] = g_networkType;
  doc["bath_pause_active"] = g_bathPauseActive;
  if (g_bathPauseActive && g_bathPauseUntilMillis > millis()) {
    doc["bath_pause_secs_remaining"] = (g_bathPauseUntilMillis - millis()) / 1000UL;
  }
  doc["hum_descent_heater_forced"] = g_humDescentHeaterForced;
  // Heater runtime: current session + accumulated (if heater is currently on, add live time)
  unsigned long heaterTotalSecs = g_heaterOnTotalSecs;
  // Check actual heater state (g_heaterOn && !g_heaterOvertemp) for live tracking
  bool heaterCurrentlyOn = g_heaterOn && !g_heaterOvertemp;
  if (heaterCurrentlyOn && g_heaterOnStartMillis > 0) {
    heaterTotalSecs += (millis() - g_heaterOnStartMillis) / 1000UL;
  }
  doc["heater_on_total_secs"] = heaterTotalSecs;
  doc["heater_tracking_start_epoch"] = g_heaterTrackingStartEpoch;
  doc["heater_max_temp"] = config.heaterMaxTemp;
  doc["heater_watts"] = config.heaterWatts;
  doc["energy_price_kwh"] = config.energyPriceKwh;
  doc["absaugen_cyclic_enabled"] = config.absaugenCyclicEnabled;
  doc["absaugen_cyclic_running"] = g_absaugenCyclicRunning;
  if (g_absaugenCyclicRunning && g_absaugenCyclicRunUntilMillis > millis()) {
    doc["absaugen_cyclic_secs_remaining"] = (g_absaugenCyclicRunUntilMillis - millis()) / 1000UL;
  } else if (config.absaugenCyclicEnabled && !g_absaugenCyclicRunning) {
    unsigned long intervalMs = (unsigned long)config.absaugenCyclicIntervalMins * 60UL * 1000UL;
    unsigned long elapsed = (g_absaugenCyclicLastOffMillis > 0) ? (millis() - g_absaugenCyclicLastOffMillis) : intervalMs;
    doc["absaugen_cyclic_next_secs"] = (elapsed < intervalMs) ? (intervalMs - elapsed) / 1000UL : 0;
  }
  doc["free_heap"] = ESP.getFreeHeap();
  doc["uptime_secs"] = millis() / 1000UL;
  doc["fw_version"] = FIRMWARE_VERSION;
  doc["fw_build"] = String(FIRMWARE_BUILD_DATE) + " " + String(FIRMWARE_BUILD_TIME);
}

// ------------ Async Web Request Helpers ------------

// Helper: get POST param value (form-urlencoded body)
static String getPostParam(AsyncWebServerRequest *request, const String &name) {
  if (request->hasParam(name, true)) {
    return request->getParam(name, true)->value();
  }
  return String();
}

static bool hasPostParam(AsyncWebServerRequest *request, const String &name) {
  return request->hasParam(name, true);
}

// Helper: send JSON success response
static void sendJsonSuccess(AsyncWebServerRequest *request) {
  request->send(200, "application/json", "{\"success\":true}");
}

// ------------ Serve static files from LittleFS (async) ------------

void serveFile(AsyncWebServerRequest *request, const char *path, const char *contentType) {
  if (LittleFS.exists(path)) {
    request->send(LittleFS, path, contentType);
  } else {
    Serial.printf("[WEB] File not found: %s\n", path);
    request->send(404, "text/plain", String("File not found: ") + path);
  }
}

void handleRoot(AsyncWebServerRequest *request) {
  serveFile(request, "/www/index.html", "text/html");
}

void handleCSS(AsyncWebServerRequest *request) {
  serveFile(request, "/www/style.css", "text/css");
}

void handleJS(AsyncWebServerRequest *request) {
  serveFile(request, "/www/app.js", "application/javascript");
}

void handleDebug(AsyncWebServerRequest *request) {
  String out = "LittleFS Debug:\n";
  out += "  /www/index.html: " + String(LittleFS.exists("/www/index.html") ? "EXISTS" : "MISSING") + "\n";
  out += "  /www/style.css: " + String(LittleFS.exists("/www/style.css") ? "EXISTS" : "MISSING") + "\n";
  out += "  /www/app.js: " + String(LittleFS.exists("/www/app.js") ? "EXISTS" : "MISSING") + "\n";
  out += "  /config.yaml: " + String(LittleFS.exists("/config.yaml") ? "EXISTS" : "MISSING") + "\n";
  
  File root = LittleFS.open("/");
  if (root && root.isDirectory()) {
    out += "\nRoot directory listing:\n";
    File f = root.openNextFile();
    while (f) {
      out += "  " + String(f.name()) + " (" + String(f.size()) + " bytes)\n";
      f = root.openNextFile();
    }
  }
  
  File www = LittleFS.open("/www");
  if (www && www.isDirectory()) {
    out += "\n/www directory listing:\n";
    File f = www.openNextFile();
    while (f) {
      out += "  " + String(f.name()) + " (" + String(f.size()) + " bytes)\n";
      f = www.openNextFile();
    }
  } else {
    out += "\n/www directory: NOT FOUND or not a directory\n";
  }
  
  request->send(200, "text/plain", out);
}

void handleConfigGet(AsyncWebServerRequest *request) {
  DynamicJsonDocument doc(2048);
  doc["humidity_on"] = config.humidityOn;
  doc["humidity_off"] = config.humidityOff;
  doc["heater_enabled"] = config.heaterEnabled;
  doc["heater_on_temp_diff"] = config.heaterOnTempDiff;
  doc["heater_off_temp_diff"] = config.heaterOffTempDiff;
  doc["fan_speed_percent"] = config.fanSpeedPercent;
  doc["fan_direction"] = config.fanDirection;
  doc["absaugen_duration_secs"] = config.absaugenDurationSecs;
  doc["absaugen_speed_percent"] = config.absaugenSpeedPercent;
  doc["absaugen_cyclic_enabled"] = config.absaugenCyclicEnabled;
  doc["absaugen_cyclic_interval_mins"] = config.absaugenCyclicIntervalMins;
  doc["absaugen_cyclic_duration_mins"] = config.absaugenCyclicDurationMins;
  doc["heater_min_room_temp"] = config.heaterMinRoomTemp;
  doc["heater_max_temp"] = config.heaterMaxTemp;
  doc["auto_enabled"] = config.autoEnabled;
  doc["bath_pause_mins"] = config.bathPauseMins;
  doc["temp_safety_diff"] = config.tempSafetyDiff;
  doc["temp_safety_hyst"] = config.tempSafetyHyst;
  doc["mqtt_base_topic"] = config.mqttBaseTopic;
  doc["schedule_enabled"] = config.scheduleEnabled;
  for (int d = 0; d < 7; d++) {
    doc["schedule"][d] = config.schedule[d];
  }
  doc["cyclic_enabled"] = config.cyclicEnabled;
  doc["cyclic_interval_mins"] = config.cyclicIntervalMins;
  doc["cyclic_duration_mins"] = config.cyclicDurationMins;
  doc["cyclic_heater"] = config.cyclicHeater;
  doc["cyclic_heater_force"] = config.cyclicHeaterForce;
  doc["cyclic_speed_percent"] = config.cyclicSpeedPercent;
  doc["cyclic_start_min"] = config.cyclicStartMin;
  doc["absaugen_cyclic_start_min"] = config.absaugenCyclicStartMin;
  doc["mqtt_interval_secs"] = config.mqttIntervalSecs;
  doc["hum_descent_min_drop"] = config.humDescentMinDrop;
  doc["hum_descent_time_secs"] = config.humDescentTimeSecs;
  doc["hum_descent_heater_enable"] = config.humDescentHeaterEnable;
  doc["heater_watts"] = config.heaterWatts;
  doc["energy_price_kwh"] = config.energyPriceKwh;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["firmware_folder"] = FIRMWARE_FOLDER;
  doc["firmware_build_date"] = FIRMWARE_BUILD_DATE;
  doc["firmware_build_time"] = FIRMWARE_BUILD_TIME;
  doc["device_hostname"] = DEVICE_HOSTNAME;

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

// Legacy htmlPage() removed - web UI now served from LittleFS files:
//   /data/www/index.html
//   /data/www/style.css
//   /data/www/app.js

// ------------ Web Request Handlers (async) ------------

void handleStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<2048> doc;
  buildStatusJson(doc);
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleManualPost(AsyncWebServerRequest *request) {
  // Read duration
  unsigned long secs = 0;
  if (hasPostParam(request, "manual_secs")) {
    secs = getPostParam(request, "manual_secs").toInt();
  }

  if (secs == 0) {
    // Zero or invalid duration -> back to AUTO
    g_controlMode = MODE_AUTO;
    g_fanOn = false;
    g_heaterOn = false;
    g_fanWasOn = false;
    Serial.println("[CTRL] Manual -> AUTO, fan/heater reset");
  } else {
    String fanArg = hasPostParam(request, "manual_fan") ? getPostParam(request, "manual_fan") : String("");
    String heaterArg = hasPostParam(request, "manual_heater") ? getPostParam(request, "manual_heater") : String("");
    fanArg.toLowerCase();
    heaterArg.toLowerCase();

    if (fanArg == "on") g_manualFanOn = true;
    else if (fanArg == "off") g_manualFanOn = false;
    else g_manualFanOn = g_fanOn; // keep current

    // Heater only allowed in EINBLASEN mode (g_fanDirection = false)
    if (heaterArg == "on") {
      if (!g_fanDirection) {
        g_manualHeaterOn = true;
        Serial.println("[MANUAL] Heater ON (EINBLASEN mode)");
      } else {
        g_manualHeaterOn = false;
        Serial.println("[MANUAL] Heater blocked - only allowed in EINBLASEN mode!");
      }
    } else if (heaterArg == "off") {
      g_manualHeaterOn = false;
    } else {
      g_manualHeaterOn = g_heaterOn; // keep current
    }

    g_controlMode = MODE_MANUAL;
    // Don't override permanent manual mode if no SHT31 sensor
    if (g_sht31Available) {
      g_manualUntilMillis = millis() + (secs * 1000UL);
    }
  }

  sendJsonSuccess(request);
}

// State for chunked history response
struct HistoryChunkState {
  time_t nowEpoch;
  size_t nextI;       // next iteration index into ring buffer
  size_t total;       // g_historyCount snapshot
  size_t startIdx;    // ring buffer start
  int phase;          // 0=header, 1=samples, 2=closing, 3=done
  bool first;         // first JSON element (no comma prefix)
};

static HistoryChunkState g_histChunk;

void handleHistory(AsyncWebServerRequest *request) {
  // Snapshot state for chunked iteration
  g_histChunk.nowEpoch = 0;
  if (g_ntpSynced) {
    time(&g_histChunk.nowEpoch);
  }
  g_histChunk.total = g_historyCount;
  g_histChunk.startIdx = (g_historyIndex + HISTORY_CAPACITY - g_historyCount) % HISTORY_CAPACITY;
  g_histChunk.nextI = 0;
  g_histChunk.phase = 0;
  g_histChunk.first = true;

  AsyncWebServerResponse *response = request->beginChunkedResponse("application/json",
    [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      char *buf = (char *)buffer;
      size_t written = 0;

      // Phase 0: Send JSON header
      if (g_histChunk.phase == 0) {
        char tmp[64];
        int n;
        if (g_histChunk.nowEpoch > 0) {
          n = snprintf(tmp, sizeof(tmp), "{\"now_epoch\":%lu,\"points\":[", (unsigned long)g_histChunk.nowEpoch);
        } else {
          n = snprintf(tmp, sizeof(tmp), "{\"points\":[");
        }
        if (n > 0 && (size_t)n < sizeof(tmp) && (size_t)n <= maxLen) {
          memcpy(buf, tmp, (size_t)n);
          g_histChunk.phase = 1;
          return (size_t)n;
        }
        // Buffer too small, write a space to keep alive (never return 0 before done)
        buf[0] = ' ';
        return 1;
      }

      // Phase 1: Send samples one at a time
      if (g_histChunk.phase == 1) {
        while (g_histChunk.nextI < g_histChunk.total) {
          size_t idx = (g_histChunk.startIdx + g_histChunk.nextI) % HISTORY_CAPACITY;
          g_histChunk.nextI++;
          const HumiditySample &s = g_history[idx];
          if (s.epochAt == 0) continue;
          long ageSecs = (long)g_histChunk.nowEpoch - (long)s.epochAt;
          if (ageSecs < 0 || ageSecs > 259200) continue;

          char tmp[160];
          int n = snprintf(tmp, sizeof(tmp),
            "%s{\"dt_sec\":%ld,\"humidity\":%.1f,\"temperature\":%.1f,\"fan_on\":%s,\"heater_on\":%s,\"fan_dir\":\"%s\"}",
            g_histChunk.first ? "" : ",",
            -ageSecs, s.humidity, s.temperature,
            s.fanOn ? "true" : "false", s.heaterOn ? "true" : "false",
            s.fanDirection ? "absaugen" : "einblasen");
          if (n <= 0 || (size_t)n >= sizeof(tmp)) continue;
          // Does it fit in current buffer?
          if (written + (size_t)n > maxLen) {
            // Put back and return what we have so far
            g_histChunk.nextI--;
            if (written > 0) return written;
            // Single entry doesn't fit - shouldn't happen with 160 byte entry and typical maxLen
            buf[0] = ' ';
            return 1;
          }
          memcpy(buf + written, tmp, (size_t)n);
          written += (size_t)n;
          g_histChunk.first = false;
        }

        // All samples written, move to closing
        g_histChunk.phase = 2;
        // Try to append closing in same chunk
        if (written + 2 <= maxLen) {
          buf[written++] = ']';
          buf[written++] = '}';
          g_histChunk.phase = 3;
        }
        return written;
      }

      // Phase 2: Send closing bracket (if didn't fit in phase 1)
      if (g_histChunk.phase == 2) {
        if (maxLen >= 2) {
          buf[0] = ']';
          buf[1] = '}';
          g_histChunk.phase = 3;
          return 2;
        }
        buf[0] = ' ';
        return 1;
      }

      // Phase 3: Done
      return 0;
    }
  );
  request->send(response);
}

// State for chunked CSV response
struct CsvChunkState {
  size_t nextI;
  size_t total;
  size_t startIdx;
  bool headerSent;
};

static CsvChunkState g_csvChunk;

void handleHistoryCsv(AsyncWebServerRequest *request) {
  g_csvChunk.total = g_historyCount;
  g_csvChunk.startIdx = (g_historyIndex + HISTORY_CAPACITY - g_historyCount) % HISTORY_CAPACITY;
  g_csvChunk.nextI = 0;
  g_csvChunk.headerSent = false;

  AsyncWebServerResponse *response = request->beginChunkedResponse("text/csv",
    [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      char *buf = (char *)buffer;
      size_t written = 0;

      if (!g_csvChunk.headerSent) {
        int n = snprintf(buf, maxLen, "Zeitstempel;Feuchte_%%;Temperatur_C;Luefter;Heizung;Richtung\r\n");
        g_csvChunk.headerSent = true;
        return (size_t)n;
      }

      size_t batchCount = 0;
      while (g_csvChunk.nextI < g_csvChunk.total && written < maxLen - 120 && batchCount < 50) {
        size_t idx = (g_csvChunk.startIdx + g_csvChunk.nextI) % HISTORY_CAPACITY;
        g_csvChunk.nextI++;
        const HumiditySample &s = g_history[idx];
        if (s.epochAt == 0) continue;

        time_t t = (time_t)s.epochAt;
        struct tm ti;
        localtime_r(&t, &ti);
        int n = snprintf(buf + written, maxLen - written,
          "%04d-%02d-%02d %02d:%02d:%02d;%.1f;%.1f;%s;%s;%s\r\n",
          ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
          ti.tm_hour, ti.tm_min, ti.tm_sec,
          s.humidity, s.temperature,
          s.fanOn ? "AN" : "AUS", s.heaterOn ? "AN" : "AUS",
          s.fanDirection ? "Absaugen" : "Einblasen");
        if (n <= 0 || written + (size_t)n >= maxLen - 10) break;
        written += (size_t)n;
        batchCount++;
      }

      return written; // 0 when done = end of response
    }
  );
  response->addHeader("Content-Disposition", "attachment; filename=\"history.csv\"");
  request->send(response);
}

void handleConfigPost(AsyncWebServerRequest *request) {
  if (hasPostParam(request, "humidity_on")) config.humidityOn = getPostParam(request, "humidity_on").toFloat();
  if (hasPostParam(request, "humidity_off")) config.humidityOff = getPostParam(request, "humidity_off").toFloat();
  if (hasPostParam(request, "heater_enabled")) {
    String v = getPostParam(request, "heater_enabled");
    v.toLowerCase();
    config.heaterEnabled = (v == "true" || v == "1" || v == "yes" || v == "on");
  }
  if (hasPostParam(request, "heater_on_temp_diff")) {
    config.heaterOnTempDiff = getPostParam(request, "heater_on_temp_diff").toFloat();
  }
  if (hasPostParam(request, "heater_off_temp_diff")) {
    config.heaterOffTempDiff = getPostParam(request, "heater_off_temp_diff").toFloat();
  }
  if (hasPostParam(request, "fan_speed_percent")) {
    int speed = getPostParam(request, "fan_speed_percent").toInt();
    if (speed >= 0 && speed <= 100) {
      config.fanSpeedPercent = (uint8_t)speed;
      Serial.printf("[CONFIG] Default fan speed updated to %d%% (current speed %d%% unchanged)\n", 
                    config.fanSpeedPercent, g_fanSpeedPercent);
    }
  }
  if (hasPostParam(request, "fan_direction")) {
    String v = getPostParam(request, "fan_direction");
    v.toLowerCase();
    bool newDirection = (v == "true" || v == "1");
    config.fanDirection = newDirection;
    setFanDirection(newDirection);
  }
  if (hasPostParam(request, "absaugen_duration_secs")) {
    int dur = getPostParam(request, "absaugen_duration_secs").toInt();
    if (dur >= 10 && dur <= 3600) {
      config.absaugenDurationSecs = (unsigned int)dur;
    }
  }
  if (hasPostParam(request, "absaugen_speed_percent")) {
    int v = getPostParam(request, "absaugen_speed_percent").toInt();
    if (v >= 0 && v <= 100) config.absaugenSpeedPercent = (uint8_t)v;
  }
  if (hasPostParam(request, "auto_enabled")) {
    String v = getPostParam(request, "auto_enabled");
    v.toLowerCase();
    config.autoEnabled = (v == "true" || v == "1");
  }
  if (hasPostParam(request, "bath_pause_mins")) {
    int v = getPostParam(request, "bath_pause_mins").toInt();
    if (v >= 1 && v <= 120) config.bathPauseMins = (unsigned int)v;
  }
  if (hasPostParam(request, "absaugen_cyclic_enabled")) {
    String v = getPostParam(request, "absaugen_cyclic_enabled");
    v.toLowerCase();
    config.absaugenCyclicEnabled = (v == "true" || v == "1");
  }
  if (hasPostParam(request, "absaugen_cyclic_interval_mins")) {
    int v = getPostParam(request, "absaugen_cyclic_interval_mins").toInt();
    if (v >= 1 && v <= 480) config.absaugenCyclicIntervalMins = (unsigned int)v;
  }
  if (hasPostParam(request, "absaugen_cyclic_duration_mins")) {
    int v = getPostParam(request, "absaugen_cyclic_duration_mins").toInt();
    if (v >= 1 && v <= 30) config.absaugenCyclicDurationMins = (unsigned int)v;
  }
  if (hasPostParam(request, "heater_min_room_temp")) {
    config.heaterMinRoomTemp = getPostParam(request, "heater_min_room_temp").toFloat();
  }
  if (hasPostParam(request, "heater_max_temp")) {
    float v = getPostParam(request, "heater_max_temp").toFloat();
    if (v >= 30.0f && v <= 50.0f) config.heaterMaxTemp = v;
  }
  if (hasPostParam(request, "temp_safety_diff")) {
    float v = getPostParam(request, "temp_safety_diff").toFloat();
    if (v >= 2.0f && v <= 30.0f) {
      config.tempSafetyDiff = v;
    }
  }
  if (hasPostParam(request, "temp_safety_hyst")) {
    float v = getPostParam(request, "temp_safety_hyst").toFloat();
    if (v >= 0.5f && v <= 10.0f) {
      config.tempSafetyHyst = v;
    }
  }
  if (hasPostParam(request, "mqtt_base_topic")) config.mqttBaseTopic = getPostParam(request, "mqtt_base_topic");
  if (hasPostParam(request, "absaugen_cyclic_start_min")) {
    int v = getPostParam(request, "absaugen_cyclic_start_min").toInt();
    if (v == 0 || v == 15 || v == 30 || v == 45) config.absaugenCyclicStartMin = (uint8_t)v;
  }
  if (hasPostParam(request, "mqtt_interval_secs")) {
    int v = getPostParam(request, "mqtt_interval_secs").toInt();
    if (v >= 1 && v <= 300) config.mqttIntervalSecs = (unsigned int)v;
  }
  if (hasPostParam(request, "hum_descent_min_drop")) {
    float v = getPostParam(request, "hum_descent_min_drop").toFloat();
    if (v >= 0.1f && v <= 20.0f) config.humDescentMinDrop = v;
  }
  if (hasPostParam(request, "hum_descent_time_secs")) {
    int v = getPostParam(request, "hum_descent_time_secs").toInt();
    if (v >= 60 && v <= 1800) config.humDescentTimeSecs = (unsigned int)v;
  }
  if (hasPostParam(request, "hum_descent_heater_enable")) {
    String v = getPostParam(request, "hum_descent_heater_enable");
    v.toLowerCase();
    config.humDescentHeaterEnable = (v == "true" || v == "1");
  }
  if (hasPostParam(request, "heater_watts")) {
    int v = getPostParam(request, "heater_watts").toInt();
    if (v >= 100 && v <= 5000) config.heaterWatts = (unsigned int)v;
  }
  if (hasPostParam(request, "energy_price_kwh")) {
    float v = getPostParam(request, "energy_price_kwh").toFloat();
    if (v >= 0.01f && v <= 2.0f) config.energyPriceKwh = v;
  }

  saveConfigToPrefs();
  sendJsonSuccess(request);
}

void handleFanSpeedPost(AsyncWebServerRequest *request) {
  String fanSpeedArg = getPostParam(request, "fan_speed");
  if (fanSpeedArg.length() > 0) {
    int speed = fanSpeedArg.toInt();
    if (speed >= 0 && speed <= 100) {
      g_fanSpeedPercent = (uint8_t)speed;
      g_lastSliderSpeed = g_fanSpeedPercent;
      
      // Apply fan speed immediately if fan is on
      if (g_fanOn) {
        setFanSpeed(g_fanSpeedPercent);
      }
      
      Serial.printf("[FAN] Temporary speed set to %d%% (default unchanged: %d%%, last slider: %d%%)\n", 
                    g_fanSpeedPercent, config.fanSpeedPercent, g_lastSliderSpeed);
    }
  }
  
  StaticJsonDocument<128> doc;
  doc["success"] = true;
  doc["fan_speed_percent"] = g_fanSpeedPercent;
  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handleFanDirectionPost(AsyncWebServerRequest *request) {
  String directionArg = getPostParam(request, "fan_direction");
  if (directionArg.length() > 0) {
    bool newDirection = (directionArg == "true");
    
    // Block ABSAUGEN when schedule is inactive
    if (newDirection && config.scheduleEnabled && !isScheduleActive()) {
      StaticJsonDocument<128> doc;
      doc["success"] = false;
      doc["error"] = "schedule_inactive";
      String response;
      serializeJson(doc, response);
      request->send(200, "application/json", response);
      return;
    }
    
    // If ABSAUGEN is already running and user requests ABSAUGEN again, STOP the cycle
    if (newDirection && g_fanDirection && g_absaugenUntilMillis > 0) {
      Serial.println("[ABSAUGEN] Cycle stopped by user (toggle off)");
      g_absaugenUntilMillis = 0;
      g_absaugenCyclicRunning = false;
      // Restore standard EINBLASEN speed before switching back
      g_fanSpeedPercent = config.fanSpeedPercent;
      g_lastSliderSpeed = g_fanSpeedPercent;
      setFanDirection(false); // Switch back to EINBLASEN
      
      StaticJsonDocument<128> doc;
      doc["success"] = true;
      doc["fan_direction"] = false;
      doc["stopped"] = true;
      String response;
      serializeJson(doc, response);
      request->send(200, "application/json", response);
      return;
    }
    
    // Use setFanDirection to trigger direction change with 2s pause
    setFanDirection(newDirection);
    // Do NOT save to config - EINBLASEN is always default, ABSAUGEN is runtime-only
    
    StaticJsonDocument<128> doc;
    doc["success"] = true;
    doc["fan_direction"] = newDirection;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
    return;
  }
  
  request->send(400, "text/plain", "Missing fan_direction parameter");
}

void handleSchedulePost(AsyncWebServerRequest *request) {
  if (hasPostParam(request, "schedule_enabled")) {
    String v = getPostParam(request, "schedule_enabled");
    v.toLowerCase();
    config.scheduleEnabled = (v == "true" || v == "1");
  }
  for (int d = 0; d < 7; d++) {
    String key = "schedule_" + String(d);
    if (hasPostParam(request, key)) {
      config.schedule[d] = (uint32_t)getPostParam(request, key).toInt();
    }
  }
  
  saveConfigToPrefs();
  
  Serial.printf("[SCHEDULE] Saved: enabled=%s\n", config.scheduleEnabled ? "true" : "false");
  for (int d = 0; d < 7; d++) {
    Serial.printf("[SCHEDULE] Day %d: 0x%06X\n", d, config.schedule[d]);
  }
  
  sendJsonSuccess(request);
}

void handleCyclicPost(AsyncWebServerRequest *request) {
  if (hasPostParam(request, "cyclic_enabled")) {
    String v = getPostParam(request, "cyclic_enabled");
    v.toLowerCase();
    config.cyclicEnabled = (v == "true" || v == "1");
  }
  if (hasPostParam(request, "cyclic_interval_mins")) {
    unsigned int v = (unsigned int)getPostParam(request, "cyclic_interval_mins").toInt();
    if (v >= 1 && v <= 120) config.cyclicIntervalMins = v;
  }
  if (hasPostParam(request, "cyclic_duration_mins")) {
    unsigned int v = (unsigned int)getPostParam(request, "cyclic_duration_mins").toInt();
    if (v >= 1 && v <= 20) config.cyclicDurationMins = v;
  }
  if (hasPostParam(request, "cyclic_heater")) {
    String v = getPostParam(request, "cyclic_heater");
    v.toLowerCase();
    config.cyclicHeater = (v == "true" || v == "1");
  }
  if (hasPostParam(request, "cyclic_heater_force")) {
    String v = getPostParam(request, "cyclic_heater_force");
    v.toLowerCase();
    config.cyclicHeaterForce = (v == "true" || v == "1");
  }
  if (hasPostParam(request, "cyclic_speed_percent")) {
    int v = getPostParam(request, "cyclic_speed_percent").toInt();
    if (v >= 0 && v <= 100) config.cyclicSpeedPercent = (uint8_t)v;
  }
  if (hasPostParam(request, "cyclic_start_min")) {
    int v = getPostParam(request, "cyclic_start_min").toInt();
    if (v == 0 || v == 15 || v == 30 || v == 45) config.cyclicStartMin = (uint8_t)v;
  }

  saveConfigToPrefs();

  Serial.printf("[CYCLIC] Saved: enabled=%s interval=%dmin duration=%dmin heater=%s speed=%d%%\n",
                config.cyclicEnabled ? "true" : "false",
                config.cyclicIntervalMins, config.cyclicDurationMins,
                config.cyclicHeater ? "true" : "false", config.cyclicSpeedPercent);

  sendJsonSuccess(request);
}

// ------------ WiFi Manager Handlers ------------

void handleWifiInfo(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;
  doc["ssid"] = WiFi.SSID();
  doc["ip"] = WiFi.localIP().toString();
  doc["gateway"] = WiFi.gatewayIP().toString();
  doc["subnet"] = WiFi.subnetMask().toString();
  doc["dns"] = WiFi.dnsIP().toString();
  doc["mac"] = WiFi.macAddress();
  doc["rssi"] = WiFi.RSSI();
  doc["channel"] = WiFi.channel();
  doc["hostname"] = DEVICE_HOSTNAME;
  doc["connected"] = (WiFi.status() == WL_CONNECTED) || g_ethConnected;
  doc["cfg_ssid"] = config.wifiSsid;
  doc["network_type"] = g_networkType;
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleWifiScan(AsyncWebServerRequest *request) {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED) {
    // Start async scan
    WiFi.scanNetworks(true);
    request->send(200, "application/json", "{\"scanning\":true}");
    return;
  }
  if (n == WIFI_SCAN_RUNNING) {
    request->send(200, "application/json", "{\"scanning\":true}");
    return;
  }
  
  DynamicJsonDocument doc(4096);
  doc["scanning"] = false;
  JsonArray arr = doc.createNestedArray("networks");
  for (int i = 0; i < n; i++) {
    JsonObject net = arr.createNestedObject();
    net["ssid"] = WiFi.SSID(i);
    net["rssi"] = WiFi.RSSI(i);
    net["channel"] = WiFi.channel(i);
    net["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "secured";
  }
  WiFi.scanDelete();
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleWifiConnect(AsyncWebServerRequest *request) {
  if (!hasPostParam(request, "ssid")) {
    request->send(400, "text/plain", "Missing ssid parameter");
    return;
  }
  String newSsid = getPostParam(request, "ssid");
  String newPass = hasPostParam(request, "password") ? getPostParam(request, "password") : String("");
  
  config.wifiSsid = newSsid;
  config.wifiPassword = newPass;
  saveConfigToPrefs();
  
  Serial.printf("[WIFI] New credentials saved: SSID=%s\n", newSsid.c_str());
  
  StaticJsonDocument<128> doc;
  doc["success"] = true;
  doc["message"] = "Credentials saved. Reconnecting...";
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
  
  // Delay then reconnect
  delay(500);
  WiFi.disconnect();
  delay(200);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
  Serial.println("[WIFI] Reconnecting with new credentials...");
}

// Helper: convert DeviceAddress to hex string
String deviceAddressToString(DeviceAddress addr) {
  String s = "";
  for (int b = 0; b < 8; b++) {
    if (addr[b] < 0x10) s += "0";
    s += String(addr[b], HEX);
  }
  s.toUpperCase();
  return s;
}

// Helper: parse hex string to DeviceAddress
bool parseDeviceAddress(const String &hexStr, DeviceAddress addr) {
  if (hexStr.length() != 16) return false;
  for (int i = 0; i < 8; i++) {
    String byteStr = hexStr.substring(i * 2, i * 2 + 2);
    addr[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
  }
  return true;
}

void handleReboot(AsyncWebServerRequest *request) {
  request->send(200, "text/plain", "Rebooting...");
  g_rebootPending = true;
  g_rebootAtMillis = millis() + 500;
}

void handleBathPause(AsyncWebServerRequest *request) {
  unsigned int mins = config.bathPauseMins;
  
  if (hasPostParam(request, "minutes")) {
    int v = getPostParam(request, "minutes").toInt();
    if (v >= 1 && v <= 120) mins = (unsigned int)v;
  }
  
  if (hasPostParam(request, "action")) {
    String action = getPostParam(request, "action");
    action.toLowerCase();
    if (action == "stop" || action == "cancel") {
      g_bathPauseActive = false;
      g_bathPauseUntilMillis = 0;
      Serial.println("[BATH-PAUSE] Cancelled");
      sendJsonSuccess(request);
      return;
    }
  }
  
  g_bathPauseActive = true;
  g_bathPauseUntilMillis = millis() + (unsigned long)mins * 60UL * 1000UL;
  
  // Immediately turn everything off
  g_fanOn = false;
  g_heaterOn = false;
  setFanPWM(false);
  digitalWrite(PIN_FAN_RELAY, LOW);
  digitalWrite(PIN_HEATER_SSR, LOW);
  
  Serial.printf("[BATH-PAUSE] Started for %d minutes\n", mins);
  sendJsonSuccess(request);
}

void handleAutoToggle(AsyncWebServerRequest *request) {
  if (hasPostParam(request, "auto_enabled")) {
    String v = getPostParam(request, "auto_enabled");
    v.toLowerCase();
    config.autoEnabled = (v == "true" || v == "1");
  } else {
    config.autoEnabled = !config.autoEnabled;
  }
  saveConfigToPrefs();
  Serial.printf("[AUTO] Automatic control: %s\n", config.autoEnabled ? "ENABLED" : "DISABLED");
  
  StaticJsonDocument<128> doc;
  doc["success"] = true;
  doc["auto_enabled"] = config.autoEnabled;
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleSensorSwap(AsyncWebServerRequest *request) {
  // Swap the two DS18B20 sensor assignments
  DeviceAddress temp;
  memcpy(temp, ds18b20_beforeFan, 8);
  memcpy(ds18b20_beforeFan, ds18b20_afterHeater, 8);
  memcpy(ds18b20_afterHeater, temp, 8);
  
  // Save to config
  config.ds18b20_beforeFan_id = deviceAddressToString(ds18b20_beforeFan);
  config.ds18b20_afterHeater_id = deviceAddressToString(ds18b20_afterHeater);
  saveConfigToPrefs();
  
  Serial.print("[DS18B20] Swapped! beforeFan=");
  Serial.print(config.ds18b20_beforeFan_id);
  Serial.print(" afterHeater=");
  Serial.println(config.ds18b20_afterHeater_id);
  
  sendJsonSuccess(request);
}

void handleSensorInfo(AsyncWebServerRequest *request) {
  DynamicJsonDocument doc(1024);
  
  // Use cached sensor list (populated at boot) - never re-enumerate the bus here
  doc["count"] = g_ds18b20_count;
  JsonArray sensors = doc.createNestedArray("sensors");
  for (int i = 0; i < g_ds18b20_count; i++) {
    JsonObject sensor = sensors.createNestedObject();
    sensor["index"] = i;
    sensor["id"] = deviceAddressToString(g_ds18b20_addrs[i]);
    // Use last known temperatures from globals instead of reading bus
    if (memcmp(g_ds18b20_addrs[i], ds18b20_beforeFan, 8) == 0) {
      sensor["temp"] = isnan(g_tempBeforeFanC) ? 0 : g_tempBeforeFanC;
    } else if (memcmp(g_ds18b20_addrs[i], ds18b20_afterHeater, 8) == 0) {
      sensor["temp"] = isnan(g_tempAfterHeaterC) ? 0 : g_tempAfterHeaterC;
    } else {
      sensor["temp"] = 0;
    }
  }
  
  // Current assignment
  doc["before_fan_id"] = config.ds18b20_beforeFan_id;
  doc["after_heater_id"] = config.ds18b20_afterHeater_id;
  doc["assigned"] = ds18b20_assigned;
  
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleSensorAssign(AsyncWebServerRequest *request) {
  if (hasPostParam(request, "before_fan_id") && hasPostParam(request, "after_heater_id")) {
    String beforeId = getPostParam(request, "before_fan_id");
    String afterId = getPostParam(request, "after_heater_id");
    beforeId.toUpperCase();
    afterId.toUpperCase();
    
    DeviceAddress addrBefore, addrAfter;
    if (parseDeviceAddress(beforeId, addrBefore) && parseDeviceAddress(afterId, addrAfter)) {
      memcpy(ds18b20_beforeFan, addrBefore, 8);
      memcpy(ds18b20_afterHeater, addrAfter, 8);
      ds18b20_assigned = true;
      
      config.ds18b20_beforeFan_id = beforeId;
      config.ds18b20_afterHeater_id = afterId;
      saveConfigToPrefs();
      
      Serial.printf("[DS18B20] Assigned: beforeFan=%s afterHeater=%s\n", beforeId.c_str(), afterId.c_str());
      sendJsonSuccess(request);
      return;
    }
  }
  request->send(400, "text/plain", "Invalid sensor IDs");
}

void handleSimple(AsyncWebServerRequest *request) {
  serveFile(request, "/www/simple.html", "text/html");
}

void setupWeb() {
  // Static files from LittleFS
  server.on("/", HTTP_GET, handleRoot);
  server.on("/simple.html", HTTP_GET, handleSimple);
  server.on("/style.css", HTTP_GET, handleCSS);
  server.on("/app.js", HTTP_GET, handleJS);
  
  // API endpoints - GET
  server.on("/debug", HTTP_GET, handleDebug);
  server.on("/config", HTTP_GET, handleConfigGet);
  server.on("/history", HTTP_GET, handleHistory);
  server.on("/history.csv", HTTP_GET, handleHistoryCsv);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/wifi/info", HTTP_GET, handleWifiInfo);
  server.on("/wifi/scan", HTTP_GET, handleWifiScan);
  
  // Firmware update via web upload
  server.on("/update", HTTP_POST,
    // Response handler (called after upload completes)
    [](AsyncWebServerRequest *request) {
      bool success = !Update.hasError();
      AsyncWebServerResponse *response = request->beginResponse(200, "application/json",
        success ? "{\"success\":true,\"msg\":\"Update OK. Neustart...\"}" : "{\"success\":false,\"msg\":\"Update fehlgeschlagen!\"}");
      response->addHeader("Connection", "close");
      request->send(response);
      if (success) {
        delay(500);
        ESP.restart();
      }
    },
    // Upload handler (called for each chunk)
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (!index) {
        // Determine update type from filename
        int cmd = (filename.indexOf("littlefs") >= 0 || filename.indexOf("spiffs") >= 0) ? U_SPIFFS : U_FLASH;
        Serial.printf("[UPDATE] Start: %s (%s)\n", filename.c_str(), cmd == U_SPIFFS ? "filesystem" : "firmware");
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
          Update.printError(Serial);
        }
      }
      if (Update.isRunning()) {
        if (Update.write(data, len) != len) {
          Update.printError(Serial);
        }
      }
      if (final) {
        if (Update.end(true)) {
          Serial.printf("[UPDATE] Success: %u bytes\n", index + len);
        } else {
          Update.printError(Serial);
        }
      }
    }
  );

  // API endpoints - POST
  server.on("/cfg", HTTP_POST, handleConfigPost);
  server.on("/manual", HTTP_POST, handleManualPost);
  server.on("/fan_speed", HTTP_POST, handleFanSpeedPost);
  server.on("/fan_direction", HTTP_POST, handleFanDirectionPost);
  server.on("/schedule", HTTP_POST, handleSchedulePost);
  server.on("/cyclic", HTTP_POST, handleCyclicPost);
  server.on("/wifi/connect", HTTP_POST, handleWifiConnect);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/heater_reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    g_heaterOnTotalSecs = 0;
    g_heaterOnStartMillis = millis();
    // Reset tracking start to current time
    time_t nowTime;
    time(&nowTime);
    g_heaterTrackingStartEpoch = (unsigned long)nowTime;
    
    prefs.begin("runtime", false);
    prefs.putULong("heat_secs", 0);
    prefs.putULong("track_start", g_heaterTrackingStartEpoch);
    prefs.end();
    Serial.printf("[HEATER] Runtime counter reset, new tracking start: %lu\n", g_heaterTrackingStartEpoch);
    request->send(200, "text/plain", "OK");
  });
  server.on("/bath_pause", HTTP_POST, handleBathPause);
  server.on("/auto_toggle", HTTP_POST, handleAutoToggle);
  server.on("/absaugen_cyclic_toggle", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (g_absaugenCyclicRunning) {
      // Stop current cycle
      g_absaugenCyclicRunning = false;
      g_absaugenCyclicLastOffMillis = millis();
      g_absaugenUntilMillis = 0;
      if (g_fanDirection) {
        g_fanDirection = false;
        g_fanSpeedPercent = config.fanSpeedPercent;
      }
      Serial.println("[ABSAUGEN-CYCLIC] Stopped by user");
    } else {
      // Start a cycle immediately
      config.absaugenCyclicEnabled = true;
      g_absaugenCyclicRunning = true;
      unsigned long durationMs = (unsigned long)config.absaugenCyclicDurationMins * 60UL * 1000UL;
      g_absaugenCyclicRunUntilMillis = millis() + durationMs;
      g_absaugenUntilMillis = millis() + durationMs;
      g_fanDirection = true;
      g_fanSpeedPercent = config.absaugenSpeedPercent;
      g_fanOn = true;
      saveConfigToPrefs();
      Serial.printf("[ABSAUGEN-CYCLIC] Started immediately for %d mins\n", config.absaugenCyclicDurationMins);
    }
    sendJsonSuccess(request);
  });
  server.on("/sensor/swap", HTTP_POST, handleSensorSwap);
  server.on("/sensor/info", HTTP_GET, handleSensorInfo);
  server.on("/sensor/assign", HTTP_POST, handleSensorAssign);
  
  server.begin();
  Serial.println("[WEB] AsyncWebServer started (LittleFS mode)");
}

// ------------ Setup / Loop ------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] ESP32 Feuchtesteuerung");

  pinMode(PIN_FAN_RELAY, OUTPUT);
  pinMode(PIN_IBT2_EN, OUTPUT);
  pinMode(PIN_HEATER_SSR, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  
  // Load config from Preferences (NVS flash)
  loadConfigFromPrefs();
  
  // Mount LittleFS for web files
  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS mount failed!");
  } else {
    Serial.println("[FS] LittleFS mounted");
  }
  
  // Initialize PWM for fan control
  setupFanPWM();
  
  // Load fan speed from config (now loaded)
  g_fanSpeedPercent = config.fanSpeedPercent;
  g_lastSliderSpeed = g_fanSpeedPercent; // Initialize with config value
  g_fanDirection = false;  // Always start in EINBLASEN (default mode)
  setFanSpeed(0); // Start with fan off
  
  Serial.printf("[SETUP] Fan initialized: current=%d%%, last slider=%d%%, direction=%s\n", 
                g_fanSpeedPercent, g_lastSliderSpeed, g_fanDirection ? "ABSAUGEN" : "EINBLASEN");
  Serial.println("[SETUP] IBT-2 ready - soft start will be used for first startup");
  
  digitalWrite(PIN_FAN_RELAY, LOW);
  digitalWrite(PIN_HEATER_SSR, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);

  // Init I2C for SHT31 (very slow clock for longer cable)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(10000); // 10 kHz
  if (!sht31.begin(0x44)) {
    Serial.println("[SHT31] Not found at 0x44! Switching to MANUAL mode.");
    g_sht31Available = false;
    g_controlMode = MODE_MANUAL;
    g_manualFanOn = false;
    g_manualHeaterOn = false;
    g_manualUntilMillis = 0xFFFFFFFF; // Stay in manual mode indefinitely
  } else {
    Serial.println("[SHT31] Sensor initialised at 0x44");
    g_sht31Available = true;
  }

  // DS18B20: Enumerate sensors using raw OneWire search (no DallasTemperature)
  // This is done ONCE at boot. Sensor addresses are cached and never re-enumerated.
  {
    g_ds18b20_count = 0;
    uint8_t addr[8];
    oneWire.reset_search();
    while (oneWire.search(addr) && g_ds18b20_count < DS18B20_MAX_SENSORS) {
      if (OneWire::crc8(addr, 7) != addr[7]) continue;  // CRC check
      if (addr[0] != 0x28) continue;  // Only DS18B20 family (0x28)
      memcpy(g_ds18b20_addrs[g_ds18b20_count], addr, 8);
      Serial.printf("[DS18B20] Found sensor %d: ", g_ds18b20_count);
      for (int b = 0; b < 8; b++) Serial.printf("%02X", addr[b]);
      Serial.println();
      g_ds18b20_count++;
    }
    oneWire.reset_search();
    Serial.printf("[DS18B20] Total: %d sensor(s) on 1-wire bus\n", g_ds18b20_count);
  }

  // Assign sensors ONLY by saved ROM IDs from Preferences - never by bus index
  if (config.ds18b20_beforeFan_id.length() == 16 && config.ds18b20_afterHeater_id.length() == 16) {
    DeviceAddress addrBefore, addrAfter;
    if (parseDeviceAddress(config.ds18b20_beforeFan_id, addrBefore) &&
        parseDeviceAddress(config.ds18b20_afterHeater_id, addrAfter)) {
      // Verify sensors actually exist on bus
      bool foundBefore = false, foundAfter = false;
      for (int i = 0; i < g_ds18b20_count; i++) {
        if (memcmp(g_ds18b20_addrs[i], addrBefore, 8) == 0) foundBefore = true;
        if (memcmp(g_ds18b20_addrs[i], addrAfter, 8) == 0) foundAfter = true;
      }
      if (foundBefore && foundAfter) {
        memcpy(ds18b20_beforeFan, addrBefore, 8);
        memcpy(ds18b20_afterHeater, addrAfter, 8);
        ds18b20_assigned = true;
        Serial.println("[DS18B20] Assigned from saved IDs (verified on bus)");
      } else {
        Serial.printf("[DS18B20] WARNING: Saved IDs not found on bus (before=%s after=%s)\n",
                      foundBefore ? "OK" : "MISSING", foundAfter ? "OK" : "MISSING");
        // Keep IDs in config but don't assign - user must reassign via UI
      }
    }
  }
  // NO automatic index-based assignment! Sensors MUST be assigned via saved IDs
  // in Preferences or manually by the user via the web UI sensor assignment page.
  // This prevents sensor swapping on reboot when bus enumeration order changes.
  if (!ds18b20_assigned && g_ds18b20_count >= 1) {
    Serial.println("[DS18B20] WARNING: No saved sensor IDs found. Please assign sensors manually via web UI (Sensoren tab).");
    Serial.printf("[DS18B20] Found %d sensor(s) on bus - awaiting manual assignment\n", g_ds18b20_count);
  }
  if (ds18b20_assigned) {
    Serial.print("[DS18B20] beforeFan  = "); for (int b=0;b<8;b++) Serial.printf("%02X",ds18b20_beforeFan[b]); Serial.println();
    Serial.print("[DS18B20] afterHeater= "); for (int b=0;b<8;b++) Serial.printf("%02X",ds18b20_afterHeater[b]); Serial.println();
  }

  connectWiFi();
  setupNTP();
  setupOTA();
  setupWeb();
  
  // Load persistent data
  loadHeaterRuntime();
  loadHistoryFromFS();
  
  Serial.println("[BOOT] Setup complete - ready");
}

void loop() {
  updateSensor();
  applyControl();
  recordHumidityHistory();
  
  // Update direction change (simple immediate)
  updateDirectionChange();
  
  // Pending reboot (deferred from web handler so response can be sent)
  if (g_rebootPending && millis() >= g_rebootAtMillis) {
    Serial.println("[REBOOT] Executing deferred reboot...");
    delay(100);
    ESP.restart();
  }

  // Check ABSAUGEN timer - automatic fallback to EINBLASEN
  unsigned long now = millis();
  if (g_absaugenUntilMillis > 0 && now >= g_absaugenUntilMillis) {
    // ABSAUGEN timer expired - fallback to EINBLASEN
    g_absaugenUntilMillis = 0;
    if (g_fanDirection) {
      Serial.println("[ABSAUGEN] Timer expired - fallback to EINBLASEN");
      // Restore standard EINBLASEN speed from config
      g_fanSpeedPercent = config.fanSpeedPercent;
      g_lastSliderSpeed = g_fanSpeedPercent;
      Serial.printf("[ABSAUGEN] Restored EINBLASEN speed to %d%%\n", g_fanSpeedPercent);
      setFanDirection(false); // Switch to EINBLASEN
    }
  }
  
  // Check NTP sync status periodically
  if (!g_ntpSynced) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
      g_ntpSynced = true;
      Serial.printf("[NTP] Time synced: %02d:%02d:%02d, wday=%d\n", 
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_wday);
    }
  }
  
  // Status check every 5 seconds
  static unsigned long lastStatusCheck = 0;
  if (millis() - lastStatusCheck > 5000) {
    lastStatusCheck = millis();
    Serial.printf("[STATUS] Fan: %s, Speed: %d%%, Direction: %s, Schedule: %s\n",
                  g_fanOn ? "ON" : "OFF", g_fanSpeedPercent,
                  g_fanDirection ? "ABSAUGEN" : "EINBLASEN",
                  isScheduleActive() ? "ACTIVE" : "INACTIVE");
  }

  // Regular WiFi reconnect check (every 30 seconds) - skip if using Ethernet
  static unsigned long lastWifiCheck = 0;
  if (g_ethConnected) {
    // Ethernet mode - always handle MQTT and OTA
    ensureMqttConnected();
    mqttClient.loop();
    ArduinoOTA.handle();
  } else if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiCheck > 30000UL) {
      lastWifiCheck = now;
      Serial.println("[WIFI] Connection lost, attempting reconnect...");
      WiFi.disconnect();
      delay(200);
      WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
    }
  } else {
    lastWifiCheck = now;
    ensureMqttConnected();
    mqttClient.loop();
    ArduinoOTA.handle();
  }

  // Note: AsyncWebServer handles requests in the background - no handleClient() needed

  unsigned long mqttIvMs = (unsigned long)config.mqttIntervalSecs * 1000UL;
  if (mqttIvMs < 1000) mqttIvMs = 1000; // minimum 1s
  if (now - g_lastMqttPublish > mqttIvMs) {
    g_lastMqttPublish = now;
    publishStatus();
  }

  // Print current IP every 10 seconds if WiFi is connected
  if (WiFi.status() == WL_CONNECTED && now - g_lastIpPrint > 10000) {
    g_lastIpPrint = now;
    Serial.print("[WIFI] Current IP: ");
    Serial.println(WiFi.localIP());
  }
  
  // Save persistent data periodically
  saveHeaterRuntime();
  
  // Save history every 2 minutes
  static unsigned long lastHistorySave = 0;
  if (now - lastHistorySave >= 120000UL) {
    lastHistorySave = now;
    saveHistoryToFS();
  }
}
