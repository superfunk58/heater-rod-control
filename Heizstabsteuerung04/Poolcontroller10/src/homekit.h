// =========================================================================
// Apple HomeKit (HAP R2) integration, implemented with HomeSpan.
//
// We expose the pool controller as a HomeKit **bridge** with five
// accessories:
//   1. Pump          - Service::Switch
//   2. Ely A/B       - Service::Switch (on = alternator running, off = stop)
//   3. Ely K         - Service::Switch
//   4. Pool temp     - Service::TemperatureSensor (celsius)
//   5. Flow detected - Service::ContactSensor    (detected/no detected)
//
// Lifecycle:
//   * homekitSetup() is called from setup() AFTER WiFi is connected.
//     It initialises HomeSpan, registers the bridge + accessories, and
//     returns immediately. HomeSpan runs its HAP server + pairing state
//     machine from a dedicated FreeRTOS task.
//   * homekitLoop() is called once per Arduino loop() iteration to let
//     HomeSpan service its event queue (timeouts, heartbeats).
//   * homekitResetPairings() wipes the HAP pairing database and reboots;
//     exposed from the Web UI via /api/homekit/reset.
//   * homekitGetSetupCode() returns the 8-digit setup code (e.g. "46637726")
//     for display in the UI.
// =========================================================================
#pragma once

#include <Arduino.h>

// Call once from setup(), AFTER WiFi has an IP.
void homekitSetup();

// Call every loop() iteration. Cheap (event queue poll).
void homekitLoop();

// Wipe the HAP pairing NVS partition and reboot. Called from /api/homekit/reset.
void homekitResetPairings();

// Current HomeKit pairing status: true if at least one controller is paired.
bool homekitIsPaired();

// 8-digit HomeKit setup code (no dashes) used at pairing time.
const char *homekitGetSetupCode();
