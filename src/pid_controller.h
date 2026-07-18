// ============================================================================
// PID-Regler direkt auf DAC-Output (alternative Regelstrategie zum Classic-PI).
// ----------------------------------------------------------------------------
//   * Konservative Defaults (Kp=5, Ki=0.5, FF=25), sofort lauffähig.
//   * Online-Adaption: nutzt Solar-Schwankungen als Testsignal, passt Gains
//     alle 60 s anhand der Fehler-Statistik an (KP/KI hard-clamped).
//   * Manueller Relay-Feedback-Autotune (Tyreus-Luyben) mit Solar-Feedforward
//     zur Kompensation von Solar-Sprüngen während des Tests.
//   * Warm-Start: letzter "guter" DAC-Wert wird in NVS persistiert; bei
//     OFF→ON wird Integrator vorgeladen ("bumpless transfer").
//   * Anti-Windup via Conditional Integration (kein Integrator-Update wenn
//     Output saturiert).
//   * Hysterese identisch zum Classic-Regler (MIN_POWER_THRESHOLD ± DEADBAND).
//
// Aktiv NUR wenn `controllerMode == "pid"`. Default ist "classic" (kein Bruch).
// ============================================================================
#pragma once

#include <Arduino.h>

namespace PidController {

enum class AutotuneState : uint8_t {
  IDLE = 0,
  BASELINE,
  RELAY,
  DONE,
  FAILED
};

// Initialisierung: NVS-Werte laden, Statistik-Buffer leeren.
void begin();

// Eigentlicher Regelschritt — ersetzt den Classic-PI-Block in loop() wenn
// controllerMode == "pid". Schreibt DAC via applyDAC() (extern), aktualisiert
// die globalen Variablen heating/wattneeded/DACoutput und sendet sendupdate()
// bei signifikanten Zustandsänderungen.
//
// Vorbedingungen (gleich wie Classic): meter frisch, DAC settled, regulating_power.
void regulate(unsigned long now);

// Sammelt Fehler-Sample für Online-Adapter (Aufruf alle paar Sekunden).
// nowError = err_grid (positive = mehr Verbrauch nötig).
void recordSample(int errGrid, unsigned long now);

// Online-Adaption: alle 60 s. Adjustiert Kp/Ki anhand Fehler-Statistik.
void tickAdapt(unsigned long now);

// ----- Autotune ---------------------------------------------------------
bool startAutotune();         // false wenn Vorbedingungen verletzt
void stopAutotune();
AutotuneState autotuneState();
const char *autotuneStateStr();
int  autotuneProgressPercent();

// ----- Accessoren für Webserver/UI --------------------------------------
float kp();
float ki();
float solarFf();
float integrator();
int   lastGoodDac();
bool  onlineAdaptEnabled();
void  setOnlineAdaptEnabled(bool en);
void  setKp(float v);
void  setKi(float v);
void  setSolarFf(float v);
uint32_t lastAdaptEpoch();
uint32_t autotuneTimestamp();

// Setzt alle Gains auf Defaults zurück (NVS-persistiert).
void resetToDefaults();

// Liefert die letzten N (max 60) Error-Samples für Mini-Chart in der UI.
// errOut: int16, count: in/out (max 60). Liefert Anzahl gültiger Samples.
size_t getRecentErrors(int16_t *errOut, size_t maxCount);

}  // namespace PidController
