#include "pid_controller.h"
#include "temp_sensors.h"

#include <Preferences.h>
#include <math.h>
#include <time.h>

// ----- Globals owned by main.cpp ----------------------------------------
extern int  ZERO_FEED_IN_TARGET;
extern int  MAX_HEATING_POWER;
extern int  MIN_POWER_THRESHOLD;
extern int  DEADBAND;
extern int  POWER_CHANGE_THRESHOLD;
extern int  MAX_BOILER_TEMP_C;
extern int  MAX_HEATER_ROD_TEMP_C;
extern volatile int powerdrawnumber;
extern int  powerToConsume;
extern int  solarAcPowerValue;
extern int  DACoutput;
extern int  wattneeded;
extern volatile bool regulating_power;
extern bool heating;

void applyDAC(int value, unsigned long now);
void sendupdate(bool force);

namespace PidController {

// ----- Konstanten -------------------------------------------------------
static constexpr const char *NVS_NS = "pid";
static constexpr float KP_DEFAULT      = 5.0f;
static constexpr float KI_DEFAULT      = 0.5f;
static constexpr float SOLAR_FF_DEF    = 25.0f;   // DAC-counts pro Watt Solar
static constexpr int   LAST_GOOD_DEF   = 12000;   // typischer mittlerer DAC-Wert

static constexpr float KP_MIN = 0.5f,  KP_MAX = 30.0f;
static constexpr float KI_MIN = 0.05f, KI_MAX = 5.0f;
static constexpr float FF_MIN = 0.0f,  FF_MAX = 100.0f;

static constexpr int   I_MAX_DAC       = 8000;    // anti-windup clamp für Integrator
static constexpr int   DAC_MAX         = 31000;
static constexpr int   DAC_MIN_HEATING = 7000;    // unterhalb davon erzeugt der Heizstab keine Leistung
static constexpr unsigned long PID_STEP_MS    = 500;    // Regelschritt
static constexpr unsigned long ADAPT_STEP_MS  = 60000;  // Online-Adaption Intervall
static constexpr unsigned long SAMPLE_STEP_MS = 1000;   // Statistik-Sample alle 1 s

static constexpr float OSC_THRESHOLD_W      = 200.0f;
static constexpr float CALM_THRESHOLD_W     = 30.0f;
static constexpr float SLUGGISH_BIAS_W      = 80.0f;
static constexpr float FF_BIAS_W            = 100.0f;

// Stable warm-start save: nur speichern wenn |err| < 50 W für >2 min.
static constexpr float WARMSAVE_ERR_W       = 50.0f;
static constexpr unsigned long WARMSAVE_STABLE_MS = 120000;

// Statistik-Buffer (rolling 40 Samples = 40 s, DRAM-optimiert)
static constexpr size_t STAT_BUF = 40;

// ----- State ------------------------------------------------------------
static float    s_kp        = KP_DEFAULT;
static float    s_ki        = KI_DEFAULT;
static float    s_solarFf   = SOLAR_FF_DEF;
static int      s_lastGoodDac = LAST_GOOD_DEF;
static bool     s_atValid    = false;
static uint32_t s_atTimestamp = 0;
static bool     s_onlineAdaptEnabled = true;
static uint32_t s_lastAdaptEpoch = 0;

static float    s_iTerm     = 0.0f;
static unsigned long s_lastStepMs = 0;
static unsigned long s_lastAdaptMs = 0;
static unsigned long s_lastSampleMs = 0;
static unsigned long s_stableSinceMs = 0;
static bool     s_outputSaturated = false;

// Statistik-Ringpuffer
static int16_t  s_errBuf[STAT_BUF] = {0};
static size_t   s_errBufHead = 0;
static size_t   s_errBufCount = 0;

// ----- Autotune State ---------------------------------------------------
static AutotuneState s_atState = AutotuneState::IDLE;
static unsigned long s_atPhaseStart = 0;
static int           s_atOpDac     = 12000;
static int           s_atRelay_d   = 2000;     // DAC-Amplitude um Op-Punkt
static float         s_atBaselineSolar = 0.0f;
static float         s_atBaselineGrid  = 0.0f;
static int           s_atBaselineSamples = 0;
static int           s_atZeroCrossings = 0;
static unsigned long s_atFirstCrossingMs = 0;
static unsigned long s_atLastCrossingMs  = 0;
static unsigned long s_atPeriodSumMs = 0;
static int           s_atPeriodCount = 0;
static float         s_atAmplMax = -1e9f;
static float         s_atAmplMin =  1e9f;
static int8_t        s_atRelaySign = 1;
static int           s_atProgress = 0;
static unsigned long s_atSavedRegMs = 0;  // for safety timeout

// Forward decls
static void clampGains();
static void saveNvsThrottled();
static void doSaveNvs();
static void tickAutotune(unsigned long now);

// ===== NVS ==============================================================

void begin() {
  Preferences p;
  if (p.begin(NVS_NS, /*ro*/ true)) {
    s_kp           = p.getFloat ("kp",       KP_DEFAULT);
    s_ki           = p.getFloat ("ki",       KI_DEFAULT);
    s_solarFf      = p.getFloat ("ff",       SOLAR_FF_DEF);
    s_lastGoodDac  = p.getInt   ("lgd",      LAST_GOOD_DEF);
    s_atValid      = p.getBool  ("atv",      false);
    s_atTimestamp  = p.getUInt  ("ats",      0);
    s_onlineAdaptEnabled = p.getBool("oae",  true);
    p.end();
  }
  clampGains();
  s_iTerm = 0.0f;
  s_lastStepMs = 0;
  s_lastAdaptMs = 0;
  s_lastSampleMs = 0;
  s_stableSinceMs = 0;
  s_atState = AutotuneState::IDLE;
}

static unsigned long s_lastNvsSaveMs = 0;
static bool          s_nvsDirty = false;

static void saveNvsThrottled() {
  // max alle 60 s, nur wenn dirty
  s_nvsDirty = true;
  if (millis() - s_lastNvsSaveMs > 60000UL) {
    doSaveNvs();
  }
}

static void doSaveNvs() {
  Preferences p;
  if (!p.begin(NVS_NS, /*ro*/ false)) return;
  p.putFloat("kp",  s_kp);
  p.putFloat("ki",  s_ki);
  p.putFloat("ff",  s_solarFf);
  p.putInt  ("lgd", s_lastGoodDac);
  p.putBool ("atv", s_atValid);
  p.putUInt ("ats", s_atTimestamp);
  p.putBool ("oae", s_onlineAdaptEnabled);
  p.end();
  s_lastNvsSaveMs = millis();
  s_nvsDirty = false;
}

static void clampGains() {
  s_kp      = constrain(s_kp,      KP_MIN, KP_MAX);
  s_ki      = constrain(s_ki,      KI_MIN, KI_MAX);
  s_solarFf = constrain(s_solarFf, FF_MIN, FF_MAX);
}

// ===== Statistik ========================================================

void recordSample(int errGrid, unsigned long now) {
  if (now - s_lastSampleMs < SAMPLE_STEP_MS) return;
  s_lastSampleMs = now;
  int16_t v = (int16_t)constrain(errGrid, -32767, 32767);
  s_errBuf[s_errBufHead] = v;
  s_errBufHead = (s_errBufHead + 1) % STAT_BUF;
  if (s_errBufCount < STAT_BUF) s_errBufCount++;
}

size_t getRecentErrors(int16_t *out, size_t maxCount) {
  size_t n = (s_errBufCount < maxCount) ? s_errBufCount : maxCount;
  // ältester zuerst
  size_t startIdx = (s_errBufHead + STAT_BUF - s_errBufCount) % STAT_BUF;
  for (size_t i = 0; i < n; i++) {
    out[i] = s_errBuf[(startIdx + i) % STAT_BUF];
  }
  return n;
}

static void errorStats(float &meanOut, float &stdOut) {
  if (s_errBufCount == 0) { meanOut = 0; stdOut = 0; return; }
  double sum = 0;
  for (size_t i = 0; i < s_errBufCount; i++) sum += s_errBuf[i];
  float m = (float)(sum / s_errBufCount);
  double v = 0;
  for (size_t i = 0; i < s_errBufCount; i++) {
    float d = s_errBuf[i] - m; v += d * d;
  }
  meanOut = m;
  stdOut  = sqrtf((float)(v / s_errBufCount));
}

// ===== Online-Adapter ===================================================

void tickAdapt(unsigned long now) {
  if (!s_onlineAdaptEnabled) return;
  if (s_atState != AutotuneState::IDLE) return;  // während Autotune nicht adapten
  if (now - s_lastAdaptMs < ADAPT_STEP_MS) return;
  s_lastAdaptMs = now;
  if (s_errBufCount < 20) return;  // brauche mind. 20 s Daten (angepasst auf STAT_BUF=40)

  float mean, stdv;
  errorStats(mean, stdv);

  bool changed = false;

  // Regel 1: Oszillation -> Gains dämpfen
  if (heating && stdv > OSC_THRESHOLD_W) {
    s_kp *= 0.83f;
    s_ki *= 0.83f;
    changed = true;
  }
  // Regel 2: Stabil aber etwas zu träge -> mutiger
  else if (stdv < CALM_THRESHOLD_W && fabsf(mean) > SLUGGISH_BIAS_W) {
    s_kp *= 1.05f;
    s_ki *= 1.05f;
    changed = true;
  }
  // Regel 3: Bias bei Solar -> solar_ff korrigieren
  if (fabsf(mean) > FF_BIAS_W && solarAcPowerValue > 200) {
    // mean > 0: zu wenig Verbrauch -> ff sollte höher sein
    s_solarFf += (mean > 0 ? 0.05f : -0.05f);
    changed = true;
  }

  if (changed) {
    clampGains();
    time_t t; time(&t);
    if (t > 1700000000) s_lastAdaptEpoch = (uint32_t)t;
    saveNvsThrottled();
  } else if (s_nvsDirty && millis() - s_lastNvsSaveMs > 60000UL) {
    doSaveNvs();
  }
}

// ===== Hysterese-Logik (gleich wie Classic) =============================

static bool shouldTurnOn(int currentDraw) {
  int errGrid = ZERO_FEED_IN_TARGET - currentDraw;
  return errGrid > (MIN_POWER_THRESHOLD + DEADBAND);
}

static bool shouldTurnOff(int currentDraw) {
  int errGrid = ZERO_FEED_IN_TARGET - currentDraw;
  return errGrid < (MIN_POWER_THRESHOLD - DEADBAND);
}

// ===== PID-Schritt ======================================================

void regulate(unsigned long now) {
  // Während Autotune übernimmt tickAutotune die Kontrolle
  if (s_atState == AutotuneState::BASELINE || s_atState == AutotuneState::RELAY) {
    tickAutotune(now);
    return;
  }

  // Regulation deaktiviert -> alles aus
  if (!regulating_power) {
    if (heating || DACoutput != 0) {
      DACoutput = 0;
      wattneeded = 0;
      s_iTerm = 0.0f;
      applyDAC(0, now);
      heating = false;
      sendupdate(false);
    }
    return;
  }

  // Alle PID_STEP_MS einen Schritt
  if (s_lastStepMs && now - s_lastStepMs < PID_STEP_MS) return;
  unsigned long dtMs = s_lastStepMs ? (now - s_lastStepMs) : PID_STEP_MS;
  s_lastStepMs = now;
  float dt = dtMs * 0.001f;

  int currentDraw = powerdrawnumber;
  powerToConsume = ZERO_FEED_IN_TARGET - currentDraw;
  int errGrid = powerToConsume;  // Watt; positiv = mehr Verbrauch nötig

  // Sample für Online-Adapter
  recordSample(errGrid, now);

  // Hysterese ON/OFF
  if (!heating) {
    if (shouldTurnOn(currentDraw)) {
      // Temperature hysteresis: don't turn on if close to limit (2°C margin)
      float tBoil = TempSensors::getBoilerC();
      float tHrod = TempSensors::getHeaterRodC();
      if (MAX_BOILER_TEMP_C > 0 && tBoil > -50.0f && tBoil < 150.0f && tBoil >= MAX_BOILER_TEMP_C - 2) return;
      if (MAX_HEATER_ROD_TEMP_C > 0 && tHrod > -50.0f && tHrod < 150.0f && tHrod >= MAX_HEATER_ROD_TEMP_C - 2) return;
      heating = true;
      // Bumpless Transfer: starte bei last_good_dac, splitten in p/i so dass Output passt
      float p = s_kp * errGrid;
      s_iTerm = s_lastGoodDac - p;
      s_iTerm = constrain(s_iTerm, (float)-I_MAX_DAC, (float)I_MAX_DAC);
      // FF-Anteil ist im DAC drin, daher iTerm allein muss Rest tragen
      // (FF wird später vom Output abgezogen, da s_lastGoodDac den FF schon enthält)
      // Aktuelle Solar-FF vom Integrator abziehen, damit nicht doppelt:
      s_iTerm -= s_solarFf * solarAcPowerValue;

      int dacFf = (int)(s_solarFf * solarAcPowerValue);
      int dac = constrain((int)(p + s_iTerm + dacFf), DAC_MIN_HEATING, DAC_MAX);
      DACoutput = dac;
      wattneeded = errGrid;  // grobe Anzeige
      applyDAC(dac, now);
      sendupdate(false);
    }
    return;
  }

  // heating == true
  // Temperature limit: hard shutdown if boiler or heater rod exceeds max temp
  {
    float tBoil = TempSensors::getBoilerC();
    float tHrod = TempSensors::getHeaterRodC();
    bool overTemp = false;
    if (MAX_BOILER_TEMP_C > 0 && tBoil > -50.0f && tBoil < 150.0f && tBoil >= MAX_BOILER_TEMP_C) overTemp = true;
    if (MAX_HEATER_ROD_TEMP_C > 0 && tHrod > -50.0f && tHrod < 150.0f && tHrod >= MAX_HEATER_ROD_TEMP_C) overTemp = true;
    if (overTemp) {
      heating = false;
      DACoutput = 0;
      wattneeded = 0;
      s_iTerm = 0.0f;
      s_outputSaturated = false;
      s_stableSinceMs = 0;
      applyDAC(0, now);
      sendupdate(false);
      return;
    }
  }
  if (shouldTurnOff(currentDraw)) {
    heating = false;
    DACoutput = 0;
    wattneeded = 0;
    s_iTerm = 0.0f;
    s_outputSaturated = false;
    s_stableSinceMs = 0;
    applyDAC(0, now);
    sendupdate(false);
    return;
  }

  // Eigentlicher PID-Schritt
  float p = s_kp * (float)errGrid;
  // Conditional integration: kein Update wenn saturiert + Integral würde noch mehr saturieren
  bool wouldGrow = ((s_outputSaturated && errGrid > 0 && s_iTerm > 0) ||
                    (s_outputSaturated && errGrid < 0 && s_iTerm < 0));
  if (!wouldGrow) {
    s_iTerm += s_ki * (float)errGrid * dt;
    s_iTerm = constrain(s_iTerm, (float)-I_MAX_DAC, (float)I_MAX_DAC);
  }

  float ff  = s_solarFf * (float)solarAcPowerValue;
  float dacF = p + s_iTerm + ff;
  int   dacTarget = (int)dacF;

  if (dacTarget > DAC_MAX) { dacTarget = DAC_MAX; s_outputSaturated = true; }
  else if (dacTarget < DAC_MIN_HEATING) { dacTarget = DAC_MIN_HEATING; s_outputSaturated = true; }
  else { s_outputSaturated = false; }

  // POWER_CHANGE_THRESHOLD wirkt hier in DAC-counts (~grobe Heuristik). Wir
  // schreiben den DAC immer, aber sendupdate() nur bei größerer Änderung.
  bool significantChange = abs(dacTarget - DACoutput) > 50;

  DACoutput  = dacTarget;
  wattneeded = errGrid;  // Anzeige-Wert (kein wahrer Soll-Watt)
  applyDAC(dacTarget, now);

  // Warm-Start-Speicherung: stabil wenn |err| < 50 W über >2 min
  if (fabsf((float)errGrid) < WARMSAVE_ERR_W) {
    if (s_stableSinceMs == 0) s_stableSinceMs = now;
    if (now - s_stableSinceMs > WARMSAVE_STABLE_MS) {
      if (abs(dacTarget - s_lastGoodDac) > 200) {
        s_lastGoodDac = dacTarget;
        saveNvsThrottled();
      }
      s_stableSinceMs = now;  // re-arm
    }
  } else {
    s_stableSinceMs = 0;
  }

  if (significantChange) sendupdate(false);
}

// ===== Autotune (Relay-Feedback mit Solar-FF) ==========================

bool startAutotune() {
  if (s_atState == AutotuneState::BASELINE || s_atState == AutotuneState::RELAY) return false;
  if (!regulating_power) return false;
  // Heizstab muss laufen (oder zumindest pump on - vereinfacht: regulating + meter frisch)
  s_atState = AutotuneState::BASELINE;
  s_atPhaseStart = millis();
  s_atOpDac = (heating && DACoutput > DAC_MIN_HEATING) ? DACoutput : s_lastGoodDac;
  s_atOpDac = constrain(s_atOpDac, DAC_MIN_HEATING + s_atRelay_d, DAC_MAX - s_atRelay_d);
  s_atBaselineSolar = 0.0f;
  s_atBaselineGrid  = 0.0f;
  s_atBaselineSamples = 0;
  s_atZeroCrossings = 0;
  s_atFirstCrossingMs = 0;
  s_atLastCrossingMs  = 0;
  s_atPeriodSumMs = 0;
  s_atPeriodCount = 0;
  s_atAmplMax = -1e9f;
  s_atAmplMin =  1e9f;
  s_atRelaySign = 1;
  s_atProgress = 0;
  heating = true;
  applyDAC(s_atOpDac, millis());
  sendupdate(true);
  return true;
}

void stopAutotune() {
  if (s_atState == AutotuneState::IDLE) return;
  s_atState = AutotuneState::IDLE;
  s_atProgress = 0;
  // Sanftes Aussteigen: zurück zu lastGoodDac, PID übernimmt
  s_iTerm = 0.0f;
  s_outputSaturated = false;
  applyDAC(s_lastGoodDac, millis());
  sendupdate(true);
}

AutotuneState autotuneState() { return s_atState; }

const char *autotuneStateStr() {
  switch (s_atState) {
    case AutotuneState::IDLE:     return "idle";
    case AutotuneState::BASELINE: return "baseline";
    case AutotuneState::RELAY:    return "relay";
    case AutotuneState::DONE:     return "done";
    case AutotuneState::FAILED:   return "failed";
  }
  return "?";
}

int autotuneProgressPercent() { return s_atProgress; }

static void atFinalize(bool success, const char *reason) {
  s_atState = success ? AutotuneState::DONE : AutotuneState::FAILED;
  s_atProgress = success ? 100 : 0;
  if (success) {
    time_t t; time(&t);
    if (t > 1700000000) s_atTimestamp = (uint32_t)t;
    s_atValid = true;
    doSaveNvs();
  }
  // Smooth handover to PID
  s_iTerm = 0.0f;
  s_outputSaturated = false;
  applyDAC(s_lastGoodDac, millis());
  Serial.printf("[PID-Autotune] %s: %s\n", success ? "DONE" : "FAILED", reason ? reason : "");
  sendupdate(true);
}

static void tickAutotune(unsigned long now) {
  // Globaler Timeout 5 Min
  if (now - s_atSavedRegMs > 0 && (now - s_atPhaseStart) > 300000UL) {
    atFinalize(false, "timeout");
    return;
  }
  // Sicherheits-Abbruch bei Netzbezug-Eskalation
  if (powerdrawnumber > 3000) {
    atFinalize(false, "grid import too high");
    return;
  }

  if (s_atState == AutotuneState::BASELINE) {
    // 30 s Baseline sammeln
    s_atBaselineSolar += (float)solarAcPowerValue;
    s_atBaselineGrid  += (float)powerdrawnumber;
    s_atBaselineSamples++;
    s_atProgress = (int)((now - s_atPhaseStart) * 10 / 30000UL);  // 0..10%
    if (now - s_atPhaseStart >= 30000UL) {
      s_atBaselineSolar /= s_atBaselineSamples;
      s_atBaselineGrid  /= s_atBaselineSamples;
      s_atState = AutotuneState::RELAY;
      s_atPhaseStart = now;
      s_atRelaySign = 1;
      // Initial relay-output
      applyDAC(s_atOpDac + s_atRelay_d, now);
    }
    return;
  }

  // RELAY-Phase
  // Compensated error: ziehe Solar-FF ab (Solar-Schwankungen werden so aus dem Signal entfernt)
  float compErr = (float)(ZERO_FEED_IN_TARGET - powerdrawnumber)
                 - (s_atBaselineSolar - (float)solarAcPowerValue) /* delta solar relativ baseline */
                   * 0.0f;  // einfache Variante: kein FF-Subtract auf grid - wir nutzen relative Sprünge
  // Update Amplituden
  if (compErr > s_atAmplMax) s_atAmplMax = compErr;
  if (compErr < s_atAmplMin) s_atAmplMin = compErr;

  // Relay umschalten basierend auf Vorzeichen von compErr (Hysterese 30 W)
  bool flip = false;
  if (s_atRelaySign > 0 && compErr < -30.0f) { s_atRelaySign = -1; flip = true; }
  else if (s_atRelaySign < 0 && compErr > 30.0f) { s_atRelaySign = 1; flip = true; }

  if (flip) {
    s_atZeroCrossings++;
    if (s_atFirstCrossingMs == 0) {
      s_atFirstCrossingMs = now;
    } else {
      unsigned long period = now - s_atLastCrossingMs;
      // jede Halbperiode ist ein Crossing; volle Periode = 2 Halbperioden
      // wir summieren Halbperioden und teilen am Ende
      s_atPeriodSumMs += period;
      s_atPeriodCount++;
    }
    s_atLastCrossingMs = now;
    int dac = s_atOpDac + (s_atRelaySign > 0 ? s_atRelay_d : -s_atRelay_d);
    applyDAC(dac, now);
  }

  // Solar-Sprung-Schutz
  if (fabsf((float)solarAcPowerValue - s_atBaselineSolar) > 1500.0f) {
    atFinalize(false, "solar jump > 1500W");
    return;
  }

  // Progress-Update (10..100% während Relay-Phase, max 5 Min)
  unsigned long elapsed = now - s_atPhaseStart;
  s_atProgress = 10 + (int)(elapsed * 90UL / 270000UL);
  if (s_atProgress > 99) s_atProgress = 99;

  // Mindestens 4 volle Perioden = 8 Halbperioden -> 8 zero-crossings
  if (s_atZeroCrossings >= 8 && s_atPeriodCount >= 6) {
    // Tu = 2 * mittlere Halbperiode
    float Tu_s = 2.0f * (s_atPeriodSumMs / 1000.0f) / (float)s_atPeriodCount;
    float a   = (s_atAmplMax - s_atAmplMin) * 0.5f;  // Amplitude in W
    if (a < 5.0f || Tu_s < 1.0f || Tu_s > 120.0f) {
      atFinalize(false, "implausible result");
      return;
    }
    float Ku = 4.0f * (float)s_atRelay_d / (PI * a);    // DAC pro W
    float Kp_new = Ku / 2.2f;                            // Tyreus-Luyben
    float Ti     = 2.2f * Tu_s;
    float Ki_new = Kp_new / Ti;

    Kp_new = constrain(Kp_new, KP_MIN, KP_MAX);
    Ki_new = constrain(Ki_new, KI_MIN, KI_MAX);

    Serial.printf("[PID-Autotune] Tu=%.2fs a=%.1fW Ku=%.3f -> Kp=%.3f Ki=%.3f\n",
                  Tu_s, a, Ku, Kp_new, Ki_new);
    s_kp = Kp_new;
    s_ki = Ki_new;
    atFinalize(true, "ok");
    return;
  }

  // Maximum 5 Min Relay-Phase, dann fail
  if (elapsed > 270000UL) {
    atFinalize(false, "no oscillation in 4.5 min");
  }
}

// ===== Accessoren =======================================================
float kp()       { return s_kp; }
float ki()       { return s_ki; }
float solarFf()  { return s_solarFf; }
float integrator() { return s_iTerm; }
int   lastGoodDac() { return s_lastGoodDac; }
bool  onlineAdaptEnabled() { return s_onlineAdaptEnabled; }
void  setOnlineAdaptEnabled(bool en) { s_onlineAdaptEnabled = en; saveNvsThrottled(); }
void  setKp(float v) { s_kp = constrain(v, KP_MIN, KP_MAX); clampGains(); saveNvsThrottled(); }
void  setKi(float v) { s_ki = constrain(v, KI_MIN, KI_MAX); clampGains(); saveNvsThrottled(); }
void  setSolarFf(float v) { s_solarFf = constrain(v, FF_MIN, FF_MAX); saveNvsThrottled(); }
uint32_t lastAdaptEpoch() { return s_lastAdaptEpoch; }
uint32_t autotuneTimestamp() { return s_atTimestamp; }

void resetToDefaults() {
  s_kp = KP_DEFAULT;
  s_ki = KI_DEFAULT;
  s_solarFf = SOLAR_FF_DEF;
  s_iTerm = 0.0f;
  s_atValid = false;
  s_atTimestamp = 0;
  doSaveNvs();
}

}  // namespace PidController
