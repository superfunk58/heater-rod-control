// =========================================================================
// Pool temperature / ORP history (ring buffer + dedicated-NVS persistence)
// =========================================================================
// Stores up to HISTORY_CAPACITY samples (one every SAMPLE_INTERVAL_SEC) of
// pool temperature and ORP, together with an absolute epoch timestamp.
// Current configuration: 1 sample every 15 min, 14 days = 1344 slots.
//
// Persistence: the whole ring buffer is saved as chunked blobs into a
// *dedicated* NVS partition called "histdata" (see partitions_homekit.csv).
// This partition is:
//   - NOT written by firmware OTA (only app0/app1 are)
//   - NOT written by `pio run -t uploadfs` (only spiffs/LittleFS is)
// so the history survives both updates. Only `esptool erase_flash`
// (or an explicit erase of the histdata partition) will wipe it.
//
// All functions are safe to call from both the Arduino loop() task and
// from PsychicHttp worker-thread handlers (data snapshots for chunked
// responses are taken under a portMUX critical section inside the .cpp).
// =========================================================================
#pragma once

#include <Arduino.h>
#include <PsychicHttp.h>

namespace History {

// 14 days at 15 min cadence: 14*24*4 = 1344 slots. 1344 * 8 B = 10.5 KB RAM.
// Trivially fits the 64 KB histdata NVS partition.
static constexpr size_t CAPACITY = 1344;

// Minimal 8-byte sample. Temperature is stored as int16 * 10 (0.1 °C
// resolution, range ±3276.7 °C) to save 2 B vs float; the field is
// INT16_MIN when the DS18B20 was missing at sample time.
#pragma pack(push, 1)
struct Sample {
  uint32_t epoch;      // Unix seconds. 0 => slot empty
  int16_t  tempTenths; // 0.1 °C units. INT16_MIN => missing
  int16_t  orpMv;      // mV. INT16_MIN => ADS1115 missing
};
#pragma pack(pop)
static_assert(sizeof(Sample) == 8, "History::Sample must be 8 bytes");

static constexpr int16_t TEMP_MISSING = INT16_MIN;
static constexpr int16_t ORP_MISSING  = INT16_MIN;

// Load the ring buffer from histdata NVS (if present). Safe to call before
// WiFi is up.
void begin();

// Record a sample if SAMPLE_INTERVAL_SEC seconds have passed since the last
// one AND NTP is synced. Called from the Arduino main loop; cheap no-op the
// rest of the time. Pass NAN for tempPool / INT16_MIN for orpMv when the
// respective sensor is unavailable.
void tickSample(float tempPool, int16_t orpMv);

// Persist the current buffer to /hist.bin. Called periodically from loop().
void tickSave();

// Force an immediate save (e.g. before reboot). Returns bytes written or 0.
size_t saveNow();

// Current number of valid samples (0..CAPACITY). For status JSON.
size_t count();

// Epoch (seconds) of the most recent valid sample, or 0 if none.
uint32_t lastEpoch();

// Register /api/history and /api/history.csv on the given PsychicHttpServer.
void registerRoutes(PsychicHttpServer &srv);

}  // namespace History
