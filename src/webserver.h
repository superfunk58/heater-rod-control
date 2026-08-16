#pragma once
#include <Arduino.h>

// Initialize HTTP server + SSE. Call after WiFi is connected.
void webserver_begin();

// Broadcast the given JSON status to all connected SSE clients.
// Called from sendupdate() in main.cpp whenever state changes.
void webserver_broadcastStatus(const char *json);

// Ultra-fast powerdraw SSE push. Sends a minimal JSON fragment so the
// browser can update the Netzbezug display with zero delay.
void webserver_broadcastPowerFast(int powerdraw, int powerToConsume, int powerDrawAge);

// Process pending work (reboot request, etc). Call from loop().
void webserver_loop();

// Log a message to Serial and to the web UI log buffer (sent via SSE).
void webLog(const char* fmt, ...);

// Pause/resume SSE broadcasts (call during OTA to reduce WiFi load).
extern bool webserver_pauseSSE;

// Set by HTTP handlers when state changes; drained by loop() -> sendupdate().
extern volatile bool webserver_ssePushPending;

// Set by webLog(); drained by webserver_loop() so events.send() stays on the
// loop task.
extern volatile bool webserver_logPushPending;

// Set by HTTP handlers when ConfigStore::save() is needed; drained by loop()
// so NVS writes stay on the loop task (avoids blocking httpd + cross-task races).
extern volatile bool webserver_configSavePending;

// Set by HTTP handler when Energy::resetAll() is requested; drained by loop()
// so the reset + NVS write stays on the loop task (avoids race with Energy::tick()).
extern volatile bool webserver_energyResetPending;

// Pending energy injection for a previous month (set by HTTP handler, drained
// by loop task so NVS writes don't race with Energy::tick()).
struct PendingEnergyInject {
  bool pending = false;
  uint16_t year = 0;
  uint8_t  month = 0;
  uint32_t wh = 0;
};
extern volatile bool webserver_energyInjectPending;
extern PendingEnergyInject webserver_pendingEnergyInject;

// Pending configuration changes set by HTTP handlers and applied in loop()
// to avoid cross-task modification of String/numeric globals.
struct PendingConfig {
  bool pending = false;

  bool hasZeroFeedTarget = false;       int zeroFeedTarget = 0;
  bool hasMaxHeatingPower = false;      int maxHeatingPower = 0;
  bool hasMinPowerThreshold = false;    int minPowerThreshold = 0;
  bool hasDeadband = false;             int deadband = 0;
  bool hasPowerChangeThreshold = false; int powerChangeThreshold = 0;
  bool hasCorrectionGain = false;       int correctionGainPct = 0;

  bool hasPumpMinRuntime = false;       unsigned long pumpMinRuntimeSec = 0;
  bool hasPumpCycleInterval = false;    unsigned long pumpCycleIntervalMin = 0;
  bool hasPumpCycleDuration = false;    unsigned long pumpCycleDurationSec = 0;

  bool hasPumpTempCond = false;         bool pumpTempCond = false;
  bool hasPumpTempHyst = false;         float pumpTempHyst = 0.0f;

  bool hasVolEnabled = false;           bool volEnabled = false;
  bool hasHistoryAveraging = false;     bool historyAveraging = false;

  bool hasVolWindowMin = false;         int volWindowMin = 0;
  bool hasVolThresholdW = false;       int volThresholdW = 0;

  bool hasOnewirePin = false;           int onewirePin = 0;

  bool hasMaxBoilerTemp = false;        int maxBoilerTemp = 0;
  bool hasMaxHeaterRodTemp = false;    int maxHeaterRodTemp = 0;

  // Fixed-size char arrays (not String): this struct is copied in/out of the
  // config mutex on every settings change — String members would churn the heap.
  bool hasNetMode = false;              char netMode[8] = "";
  bool hasLanDhcp = false;              bool lanDhcp = false;
  bool hasLanIp = false;                char lanIp[16] = "";
  bool hasLanGw = false;                char lanGw[16] = "";
  bool hasLanMask = false;              char lanMask[16] = "";
  bool hasLanDns = false;               char lanDns[16] = "";

  bool hasMqttStatusEnabled = false;    bool mqttStatusEnabled = false;
  bool hasMqttStatusInterval = false;   int mqttStatusIntervalSec = 0;
};

extern volatile bool webserver_configPending;
extern PendingConfig webserver_pendingConfig;

// Atomically copy the current pending config into `out` and clear the global
// pending struct. Called from the loop task in main.cpp.
void webserver_getAndClearPendingConfig(PendingConfig &out);

// Get current number of active SSE clients (for diagnostics)
int webserver_getSseClientCount();

// Close all SSE clients. Called when the active network interface changes
// (LAN↔WiFi fallback) so zombie connections from the old interface don't
// block the SSE task or exhaust the socket pool.
void webserver_closeAllSseClients();

// Copy up to maxLines log entries (oldest first) into buf. Returns line count.
size_t webserver_getLogLines(char *buf, size_t bufLen, uint8_t maxLines);
