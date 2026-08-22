#pragma once
#include <Arduino.h>

// Initialize HTTP server + SSE. Call after WiFi is connected.
void webserver_begin();

// Broadcast the given JSON status to all connected SSE clients.
// Called from sendupdate() in main.cpp whenever state changes.
void webserver_broadcastStatus(const String &json);

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

// Get current number of active SSE clients (for diagnostics)
int webserver_getSseClientCount();
