// =========================================================================
// Power-history ring buffer + dedicated-NVS persistence (Heizstabsteuerung).
// Adapted from the Poolcontroller10 history module.
//
// STORAGE:
//   - Short-term: 5min average, 2 days retention = 576 samples
//   - Long-term: 15min average (downsampled), 14 days retention = 1344 samples
//     (so ~12 days remain at 15min resolution after the 2-day short window)
//
// Persistence: chunked blobs in the dedicated `histdata` NVS partition (see
// partitions_heater.csv). Survives firmware OTA *and* `pio run -t uploadfs`.
// Only `esptool erase_flash` (or explicit `histdata` erase) wipes it.
// =========================================================================
#pragma once
#include <Arduino.h>
#include <PsychicHttp.h>

namespace History {

// Short-term: 5min average, 2 days = 576 samples
static constexpr size_t CAPACITY_SHORT = 576;
static constexpr uint32_t INTERVAL_SHORT_SEC = 300;  // 5 minutes

// Long-term: 15min average, 14 days = 1344 samples
static constexpr size_t CAPACITY_LONG = 1344;
static constexpr uint32_t INTERVAL_LONG_SEC = 900;  // 15 minutes

// 16-byte sample. Powers in W (int16 range), Temperaturen in 0.1 °C.
// INT16_MIN als Sentinel = "kein Wert" (Sensor nicht verfügbar/zugewiesen).
#pragma pack(push, 1)
struct Sample {
  uint32_t epoch;      // Unix timestamp
  int16_t  powerdraw;  // grid power in W (negative = feed-in)
  int16_t  wattneeded; // commanded heater power in W
  int16_t  solarpower; // solar power in W (0 if not available)
  int16_t  heater_power; // measured heater power (W)
  int16_t  t_boiler;   // 0.1 °C; INT16_MIN = N/A  (Speicher)
  int16_t  t_inlet;    // 0.1 °C; INT16_MIN = N/A  (Zulauf, vor Heizstab)
  int16_t  t_outlet;   // 0.1 °C; INT16_MIN = N/A  (Ablauf, nach Heizstab)
  int16_t  t_hrod;     // 0.1 °C; INT16_MIN = N/A  (Heizstab-Fühler)
} __attribute__((packed));
#pragma pack(pop)
static_assert(sizeof(Sample) == 20, "History::Sample must be 20 bytes");

void   begin();
// Schaltet zeit-gewichtetes Mitteln an/aus. Bei false wird pro Intervall der
// zuletzt an tickSample() übergebene Momentanwert (Snapshot) gespeichert.
void   setAveragingEnabled(bool enabled);
void   tickSample(int powerdraw, int wattneeded, int solarpower = 0,
                  float t_boiler_c = NAN, float t_inlet_c = NAN,
                  float heater_power = NAN, float t_outlet_c = NAN,
                  float t_hrod_c = NAN);
void   tickSave();
size_t saveNow();
size_t count();
uint32_t lastEpoch();
void   registerRoutes(PsychicHttpServer &srv);

}  // namespace History
