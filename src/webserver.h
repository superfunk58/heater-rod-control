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

  bool hasNetMode = false;              String netMode;
  bool hasLanDhcp = false;              bool lanDhcp = false;
  bool hasLanIp = false;                String lanIp;
  bool hasLanGw = false;                String lanGw;
  bool hasLanMask = false;              String lanMask;
  bool hasLanDns = false;               String lanDns;

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
