#include "config_store.h"
#include <Preferences.h>
#include <Arduino.h>

// Globals owned by main.cpp
extern int   ZERO_FEED_IN_TARGET;
extern int   MAX_HEATING_POWER;
extern int   MIN_POWER_THRESHOLD;
extern int   DEADBAND;
extern int   POWER_CHANGE_THRESHOLD;
extern int   MAX_BOILER_TEMP_C;
extern int   MAX_HEATER_ROD_TEMP_C;
extern float correctionGain;
extern unsigned long PUMP_MIN_RUNTIME_MS;
extern unsigned long PUMP_CYCLE_INTERVAL_MIN;
extern unsigned long PUMP_CYCLE_DURATION_SEC;
extern bool  PUMP_TEMP_COND_ENABLED;
extern float PUMP_TEMP_HYST_C;
extern bool  regulating_power;
extern String controllerMode;  // "classic" | "pid"
extern bool  VOL_ENABLED;
extern int   VOL_WINDOW_MIN;
extern int   VOL_THRESHOLD_W;
extern int   ONEWIRE_PIN;


namespace ConfigStore {

static constexpr const char *NS = "heater";

void load() {
  Preferences p;
  if (!p.begin(NS, /*readOnly*/ true)) return;
  ZERO_FEED_IN_TARGET    = p.getInt   ("zft",   ZERO_FEED_IN_TARGET);
  MAX_HEATING_POWER      = p.getInt   ("max",   MAX_HEATING_POWER);
  MIN_POWER_THRESHOLD    = p.getInt   ("min",   MIN_POWER_THRESHOLD);
  DEADBAND               = p.getInt   ("db",    DEADBAND);
  POWER_CHANGE_THRESHOLD = p.getInt   ("pct",   POWER_CHANGE_THRESHOLD);
  MAX_BOILER_TEMP_C      = p.getInt   ("mbtc",  MAX_BOILER_TEMP_C);
  MAX_HEATER_ROD_TEMP_C  = p.getInt   ("mhrc",  MAX_HEATER_ROD_TEMP_C);
  correctionGain         = p.getFloat ("gain",  correctionGain);
  PUMP_MIN_RUNTIME_MS    = p.getULong ("pmrt",  PUMP_MIN_RUNTIME_MS);
  PUMP_CYCLE_INTERVAL_MIN  = p.getULong ("pcim",  PUMP_CYCLE_INTERVAL_MIN);
  PUMP_CYCLE_DURATION_SEC  = p.getULong ("pcds",  PUMP_CYCLE_DURATION_SEC);
  PUMP_TEMP_COND_ENABLED   = p.getBool  ("ptce",  PUMP_TEMP_COND_ENABLED);
  PUMP_TEMP_HYST_C         = p.getFloat ("pthy",  PUMP_TEMP_HYST_C);
  regulating_power         = p.getBool  ("reg",   regulating_power);
  controllerMode           = p.getString("cmode", controllerMode);
  VOL_ENABLED              = p.getBool  ("volen", VOL_ENABLED);
  VOL_WINDOW_MIN           = p.getInt   ("volwm", VOL_WINDOW_MIN);
  VOL_THRESHOLD_W          = p.getInt   ("volth", VOL_THRESHOLD_W);
  ONEWIRE_PIN              = p.getInt   ("owpin", ONEWIRE_PIN);

  p.end();

  // Auto-correct old default GPIO5 to GPIO15 if still set (user hardware changed)
  if (ONEWIRE_PIN == 5) {
    ONEWIRE_PIN = 15;
    save();  // persist corrected value
  }
}

void save() {
  Preferences p;
  if (!p.begin(NS, /*readOnly*/ false)) return;
  p.putInt   ("zft",  ZERO_FEED_IN_TARGET);
  p.putInt   ("max",  MAX_HEATING_POWER);
  p.putInt   ("min",  MIN_POWER_THRESHOLD);
  p.putInt   ("db",   DEADBAND);
  p.putInt   ("pct",  POWER_CHANGE_THRESHOLD);
  p.putInt   ("mbtc", MAX_BOILER_TEMP_C);
  p.putInt   ("mhrc", MAX_HEATER_ROD_TEMP_C);
  p.putFloat ("gain", correctionGain);
  p.putULong ("pmrt", PUMP_MIN_RUNTIME_MS);
  p.putULong ("pcim", PUMP_CYCLE_INTERVAL_MIN);
  p.putULong ("pcds", PUMP_CYCLE_DURATION_SEC);
  p.putBool  ("ptce", PUMP_TEMP_COND_ENABLED);
  p.putFloat ("pthy", PUMP_TEMP_HYST_C);
  p.putBool  ("reg",  regulating_power);
  p.putString("cmode", controllerMode);
  p.putBool  ("volen", VOL_ENABLED);
  p.putInt   ("volwm", VOL_WINDOW_MIN);
  p.putInt   ("volth", VOL_THRESHOLD_W);
  p.putInt   ("owpin", ONEWIRE_PIN);
  p.end();
}

}  // namespace ConfigStore
