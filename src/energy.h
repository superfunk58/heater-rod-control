// ============================================================================
// Energy accumulator for the heater (Wh / kWh).
// ----------------------------------------------------------------------------
// Integrates the commanded heater power (`wattneeded`) over time using
// millis() deltas. Persisted to NVS (namespace "energy") and snapshotted into
// monthly chunks (last 6 months) on month rollover (NTP required).
// ============================================================================
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

namespace Energy {

static constexpr size_t MONTHLY_CHUNKS = 6;

struct MonthlyChunk {
  uint16_t year;     // 1970..3000  (0 = empty)
  uint8_t  month;    // 1..12
  uint8_t  _pad;
  uint32_t wh;       // accumulated Wh for that month
};

void begin();
void tick(int watts);     // call every loop iteration with current commanded W
void tickSave();          // throttled NVS persistence
void saveNow();
void resetAll();          // wipe everything (current + history)

void fillStatus(JsonVariant doc);   // populate energy_* fields into doc

}  // namespace Energy
