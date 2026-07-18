// Power-history ring buffer for the Heizstabsteuerung. See history.h.
// 1min average (7 days) + 15min average (90 days) with downsampling

#include "history.h"
#include "webserver.h"
#include <Preferences.h>
#include <nvs_flash.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace History {

// ----- Static state: Short-term (1min, 7 days) -----
static Sample        s_buf_short[CAPACITY_SHORT];
static size_t        s_count_short = 0;
static size_t        s_index_short = 0;
static uint32_t      s_lastSampleEpoch_short = 0;

// ----- Static state: Long-term (15min, 90 days) -----
static Sample        s_buf_long[CAPACITY_LONG];
static size_t        s_count_long = 0;
static size_t        s_index_long = 0;
static uint32_t      s_lastSampleEpoch_long = 0;

// ----- Common state -----
static unsigned long s_lastSaveMs = 0;
// Guards the ring buffers + indices. A FreeRTOS mutex (not a portMUX spinlock)
// so longer hold times never disable interrupts (which would starve WiFi and
// trip the watchdog). Only ever taken from task context (loop + httpd tasks).
static SemaphoreHandle_t s_mux = nullptr;

static inline void histLock()   { if (s_mux) xSemaphoreTake(s_mux, portMAX_DELAY); }
static inline void histUnlock() { if (s_mux) xSemaphoreGive(s_mux); }

// Zeit-gewichtetes Mitteln an/aus. Bei false: Momentanwert-Snapshot pro Intervall.
static bool s_averaging = true;
void setAveragingEnabled(bool enabled) { s_averaging = enabled; }

// Accumulator for 1min average (time-weighted)
static int32_t       s_accumPowerdraw = 0;      // sum(value * duration)
static int32_t       s_accumWattneeded = 0;     // sum(value * duration)
static int32_t       s_accumSolarpower = 0;     // sum(value * duration)
static int32_t       s_accumHeaterPower = 0;    // Daily energy counter (kWh * 1000 for int16)
static int32_t       s_accumTBoiler = 0;        // sum(value * duration) in 0.1°C
static int32_t       s_accumTInlet  = 0;        // sum(value * duration) in 0.1°C
static int32_t       s_accumTOutlet = 0;        // sum(value * duration) in 0.1°C
static int32_t       s_accumTHrod   = 0;        // sum(value * duration) in 0.1°C
static uint32_t      s_accumDuration = 0;       // total duration in seconds
static uint32_t      s_accumCountTBoiler = 0;   // separate Counter (Sensor optional)
static uint32_t      s_accumCountTInlet  = 0;   // separate Counter (Sensor optional)
static uint32_t      s_accumCountTOutlet = 0;   // separate Counter (Sensor optional)
static uint32_t      s_accumCountTHrod   = 0;   // separate Counter (Sensor optional)
static uint32_t      s_accumStartEpoch = 0;
static uint32_t      s_lastSampleEpoch = 0;     // for time-weighted calculation
static uint32_t      s_lastDayEpoch = 0;        // Last day when energy was stored

// Accumulator for 15min downsampling (long-term)
static int32_t       s_accumPowerdraw_long = 0;
static int32_t       s_accumWattneeded_long = 0;
static int32_t       s_accumSolarpower_long = 0;
static int32_t       s_accumHeaterPower_long = 0;
static int32_t       s_accumTBoiler_long = 0;
static int32_t       s_accumTInlet_long  = 0;
static int32_t       s_accumTOutlet_long = 0;
static int32_t       s_accumTHrod_long   = 0;
static uint32_t      s_accumDuration_long = 0;  // total duration in seconds
static uint32_t      s_accumCountTBoiler_long = 0;
static uint32_t      s_accumCountTInlet_long  = 0;
static uint32_t      s_accumCountTOutlet_long = 0;
static uint32_t      s_accumCountTHrod_long   = 0;
static uint32_t      s_accumCount_long = 0;
static uint32_t      s_accumStartEpoch_long = 0;

static constexpr const char *NVS_PARTITION  = "histdata";
static constexpr const char *NVS_NAMESPACE  = "hist";
static constexpr unsigned long SAVE_INTERVAL_MS = 5UL * 60UL * 1000UL;

// Short-term: 576 samples / 8 chunks = 72 samples per chunk
static constexpr size_t CHUNK_SAMPLES_SHORT = 72;
static constexpr size_t NUM_CHUNKS_SHORT    = CAPACITY_SHORT / CHUNK_SAMPLES_SHORT;
static_assert(CAPACITY_SHORT % CHUNK_SAMPLES_SHORT == 0, "CAPACITY_SHORT %% CHUNK_SAMPLES_SHORT");

// Long-term: 1344 samples / 14 chunks = 96 samples per chunk
static constexpr size_t CHUNK_SAMPLES_LONG = 96;
static constexpr size_t NUM_CHUNKS_LONG    = CAPACITY_LONG / CHUNK_SAMPLES_LONG;
static_assert(CAPACITY_LONG % CHUNK_SAMPLES_LONG == 0, "CAPACITY_LONG %% CHUNK_SAMPLES_LONG");

static constexpr uint32_t SCHEMA_VERSION = 11;  // bumped: retention 2d short / 14d long

static void ensurePartition() {
  static bool done = false;
  if (done) return;
  esp_err_t err = nvs_flash_init_partition(NVS_PARTITION);
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase_partition(NVS_PARTITION);
    err = nvs_flash_init_partition(NVS_PARTITION);
  }
  if (err != ESP_OK) Serial.printf("[HIST] init failed: %d\n", (int)err);
  done = true;
}

static bool isValidEpoch(uint32_t epoch, uint32_t nowEpoch) {
  return epoch >= 1600000000 && epoch <= nowEpoch + 86400;
}

void begin() {
  if (!s_mux) s_mux = xSemaphoreCreateMutex();
  ensurePartition();
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, /*ro*/ true, NVS_PARTITION)) return;

  time_t now;
  time(&now);
  const uint32_t nowEpoch = (now >= 1600000000) ? (uint32_t)now : 0;

  const uint32_t schema = p.getUInt("schema", 0);
  // Schema 10 -> 11 only GREW the long-term capacity (672 -> 1344). The 20-byte
  // Sample layout and the short-term layout are unchanged, so we can migrate the
  // old data in place instead of discarding it (see long-term load below).
  const bool migrateFrom10 = (schema == 10);
  if (schema != SCHEMA_VERSION && !migrateFrom10) {
    Serial.printf("[HIST] schema %lu incompatible with %lu, resetting history\n",
                  (unsigned long)schema, (unsigned long)SCHEMA_VERSION);
    webLog("[HIST] schema %lu incompatible with %lu, resetting history",
           (unsigned long)schema, (unsigned long)SCHEMA_VERSION);
    p.end();
    // Wipe the namespace so the next boot starts with the new layout
    Preferences pw;
    if (pw.begin(NVS_NAMESPACE, /*ro*/ false, NVS_PARTITION)) {
      pw.clear();
      pw.end();
    }
    ensurePartition();  // re-init after clear
    return;
  }
  if (migrateFrom10) {
    Serial.println("[HIST] migrating schema 10 -> 11 (long-term 672 -> 1344, data preserved)");
    webLog("[HIST] migrating history schema 10 -> 11 (data preserved)");
  }

  // Load short-term data
  const uint32_t cnt_short = p.getUInt("cnt_s", 0);
  const uint32_t idx_short = p.getUInt("idx_s", 0);
  if (cnt_short <= CAPACITY_SHORT && idx_short < CAPACITY_SHORT) {
    for (size_t c = 0; c < NUM_CHUNKS_SHORT; c++) {
      char key[8];
      snprintf(key, sizeof(key), "ds%u", (unsigned)c);
      p.getBytes(key, &s_buf_short[c * CHUNK_SAMPLES_SHORT], CHUNK_SAMPLES_SHORT * sizeof(Sample));
    }
    s_count_short = cnt_short;
    s_index_short = idx_short;
    // Restore last-sample epoch even without NTP (needed for clock seeding below)
    if (s_count_short > 0) {
      const size_t lastSlot = (s_index_short + CAPACITY_SHORT - 1) % CAPACITY_SHORT;
      if (s_buf_short[lastSlot].epoch >= 1600000000) {
        s_lastSampleEpoch_short = s_buf_short[lastSlot].epoch;
      }
    }
    // Validate: discard corrupt/unplausible samples
    if (s_count_short > 0 && nowEpoch > 0) {
      const size_t startIdx = (s_index_short + CAPACITY_SHORT - s_count_short) % CAPACITY_SHORT;
      size_t valid = 0;
      for (size_t k = 0; k < s_count_short; k++) {
        const size_t i = (startIdx + k) % CAPACITY_SHORT;
        if (!isValidEpoch(s_buf_short[i].epoch, nowEpoch)) {
          s_buf_short[i].epoch = 0;  // mark invalid
        } else {
          valid++;
        }
      }
      if (valid == 0) {
        s_count_short = 0;
        s_index_short = 0;
        s_lastSampleEpoch_short = 0;
      } else {
        const size_t lastSlot = (s_index_short + CAPACITY_SHORT - 1) % CAPACITY_SHORT;
        s_lastSampleEpoch_short = s_buf_short[lastSlot].epoch;
      }
    }
  }

  // Load long-term data.  When migrating from schema 10 the on-disk ring was
  // sized for 672 samples / 7 chunks; the live buffer is now 1344 / 14 chunks.
  const uint32_t cnt_long = p.getUInt("cnt_l", 0);
  const uint32_t idx_long = p.getUInt("idx_l", 0);
  const size_t   oldCapLong    = migrateFrom10 ? 672 : CAPACITY_LONG;
  const size_t   oldChunksLong = migrateFrom10 ? (672 / CHUNK_SAMPLES_LONG) : NUM_CHUNKS_LONG;
  if (cnt_long <= oldCapLong && idx_long < oldCapLong) {
    for (size_t c = 0; c < oldChunksLong; c++) {
      char key[8];
      snprintf(key, sizeof(key), "dl%u", (unsigned)c);
      p.getBytes(key, &s_buf_long[c * CHUNK_SAMPLES_LONG], CHUNK_SAMPLES_LONG * sizeof(Sample));
    }
    if (migrateFrom10) {
      // Re-linearize the old ring (modulus 672) into chronological order at the
      // front of the new buffer.  The upper half (>= 672) is still empty, so we
      // use it as scratch: copy oldest..newest there, then move back to front.
      if (cnt_long > 0) {
        const size_t startIdx = (idx_long + oldCapLong - cnt_long) % oldCapLong;
        const size_t scratch  = CAPACITY_LONG - cnt_long;   // >= 672, no overlap with 0..671
        for (size_t k = 0; k < cnt_long; k++)
          s_buf_long[scratch + k] = s_buf_long[(startIdx + k) % oldCapLong];
        for (size_t k = 0; k < cnt_long; k++)
          s_buf_long[k] = s_buf_long[scratch + k];
        for (size_t k = cnt_long; k < CAPACITY_LONG; k++)
          s_buf_long[k].epoch = 0;
      }
      s_count_long = cnt_long;
      s_index_long = cnt_long % CAPACITY_LONG;  // cnt_long <= 672 < 1344
    } else {
      s_count_long = cnt_long;
      s_index_long = idx_long;
    }
    // Restore last-sample epoch even without NTP (needed for clock seeding below)
    if (s_count_long > 0) {
      const size_t lastSlot = (s_index_long + CAPACITY_LONG - 1) % CAPACITY_LONG;
      if (s_buf_long[lastSlot].epoch >= 1600000000) {
        s_lastSampleEpoch_long = s_buf_long[lastSlot].epoch;
      }
    }
    if (s_count_long > 0 && nowEpoch > 0) {
      const size_t startIdx = (s_index_long + CAPACITY_LONG - s_count_long) % CAPACITY_LONG;
      size_t valid = 0;
      for (size_t k = 0; k < s_count_long; k++) {
        const size_t i = (startIdx + k) % CAPACITY_LONG;
        if (!isValidEpoch(s_buf_long[i].epoch, nowEpoch)) {
          s_buf_long[i].epoch = 0;
        } else {
          valid++;
        }
      }
      if (valid == 0) {
        s_count_long = 0;
        s_index_long = 0;
        s_lastSampleEpoch_long = 0;
      } else {
        const size_t lastSlot = (s_index_long + CAPACITY_LONG - 1) % CAPACITY_LONG;
        s_lastSampleEpoch_long = s_buf_long[lastSlot].epoch;
      }
    }
  }

  p.end();

  // Clock seeding: after a reboot WITHOUT WiFi the RTC starts at 0 and NTP
  // cannot sync. Seed the system clock from the newest saved sample so that
  // time() is approximately valid and History/Energy recording continues.
  // NTP overwrites the clock as soon as WiFi returns.
  if (nowEpoch == 0) {
    const uint32_t seed = (s_lastSampleEpoch_short > s_lastSampleEpoch_long)
                          ? s_lastSampleEpoch_short : s_lastSampleEpoch_long;
    if (seed >= 1600000000) {
      struct timeval tv = { .tv_sec = (time_t)(seed + INTERVAL_SHORT_SEC), .tv_usec = 0 };
      settimeofday(&tv, nullptr);
      Serial.printf("[HIST] no NTP at boot - clock seeded from last sample (epoch %u)\n",
                    (unsigned)seed);
    }
  }

  // Initialize accumulators
  s_accumDuration = 0;
  s_accumStartEpoch = 0;
  s_lastSampleEpoch = 0;
  s_accumDuration_long = 0;
  s_accumStartEpoch_long = 0;
  s_accumCount_long = 0;
  s_lastDayEpoch = 0;
}

void tickSample(int powerdraw, int wattneeded, int solarpower,
                float t_boiler_c, float t_inlet_c, float heater_power, float t_outlet_c, float t_hrod_c) {
  // Obtain current epoch.  When NTP is synced the RTC keeps ticking even
  // without WiFi.  After a cold reboot without WiFi the RTC starts at 0;
  // in that case we extrapolate from the last known-good epoch + millis()
  // so history recording continues with approximate timestamps.
  static uint32_t s_lastGoodEpoch = 0;
  static unsigned long s_lastGoodMs = 0;

  time_t now;
  time(&now);

  uint32_t epoch;
  if (now >= 1700000000) {
    // NTP / RTC time is valid — use it and remember as reference
    epoch = (uint32_t)now;
    s_lastGoodEpoch = epoch;
    s_lastGoodMs    = millis();
  } else if (s_lastGoodEpoch != 0) {
    // NTP not available but we have a previous reference — extrapolate
    epoch = s_lastGoodEpoch + (millis() - s_lastGoodMs) / 1000;
    static unsigned long s_lastFallbackLogMs = 0;
    if (millis() - s_lastFallbackLogMs > 300000) {   // log every 5 min max
      webLog("[Hist] NTP unavailable, using millis() extrapolation");
      s_lastFallbackLogMs = millis();
    }
  } else {
    static unsigned long s_lastSkipLogMs = 0;
    if (millis() - s_lastSkipLogMs > 300000) {
      webLog("[Hist] No NTP reference yet, skipping sample");
      s_lastSkipLogMs = millis();
    }
    return;   // never had NTP — no valid time reference at all
  }

  // Initialize short-term accumulator on first call or after 1min
  if (s_accumDuration == 0) {
    s_accumStartEpoch = epoch;
    s_lastSampleEpoch = epoch;
  }

  // Calculate time delta since last sample (minimum 1s, max 60s)
  uint32_t dt = (epoch > s_lastSampleEpoch) ? (epoch - s_lastSampleEpoch) : 1;
  if (dt > INTERVAL_SHORT_SEC) dt = INTERVAL_SHORT_SEC;  // cap to prevent excessive weight on stale data
  s_lastSampleEpoch = epoch;

  // Accumulate time-weighted values for short-term (1min)
  s_accumPowerdraw += powerdraw * (int32_t)dt;
  s_accumWattneeded += wattneeded * (int32_t)dt;
  s_accumSolarpower += solarpower * (int32_t)dt;
  // heater_power is now daily energy counter (kWh), not time-weighted
  if (heater_power >= 0.0f) {
    s_accumHeaterPower = (int32_t)heater_power;  // Store current energy value
  }
  auto validT = [](float v) { return v > -50.0f && v < 150.0f; };
  if (validT(t_boiler_c)) {
    s_accumTBoiler += (int32_t)roundf(t_boiler_c * 10.0f) * (int32_t)dt;
    s_accumCountTBoiler++;
  }
  if (validT(t_inlet_c)) {
    s_accumTInlet += (int32_t)roundf(t_inlet_c * 10.0f) * (int32_t)dt;
    s_accumCountTInlet++;
  }
  if (validT(t_outlet_c)) {
    s_accumTOutlet += (int32_t)roundf(t_outlet_c * 10.0f) * (int32_t)dt;
    s_accumCountTOutlet++;
  }
  if (validT(t_hrod_c)) {
    s_accumTHrod += (int32_t)roundf(t_hrod_c * 10.0f) * (int32_t)dt;
    s_accumCountTHrod++;
  }
  s_accumDuration += dt;

  // Check if 1 minute has passed
  if (epoch - s_accumStartEpoch < INTERVAL_SHORT_SEC) return;

  // Calculate 1-minute time-weighted average
  Sample s_short{};
  s_short.epoch      = s_accumStartEpoch + INTERVAL_SHORT_SEC / 2;
  s_short.powerdraw  = (int16_t)constrain(s_accumPowerdraw / (int32_t)s_accumDuration, -32767, 32767);
  s_short.wattneeded = (int16_t)constrain(s_accumWattneeded / (int32_t)s_accumDuration, -32767, 32767);
  s_short.solarpower = (int16_t)constrain(s_accumSolarpower / (int32_t)s_accumDuration, -32767, 32767);
  // heater_power is daily energy counter (kWh * 10 for int16), not time-weighted
  s_short.heater_power = (int16_t)constrain(s_accumHeaterPower * 10, -32767, 32767);
  s_short.t_boiler   = (s_accumCountTBoiler > 0)
                       ? (int16_t)constrain(s_accumTBoiler / (int32_t)s_accumDuration, -32767, 32767)
                       : INT16_MIN;
  s_short.t_inlet    = (s_accumCountTInlet > 0)
                       ? (int16_t)constrain(s_accumTInlet  / (int32_t)s_accumDuration,  -32767, 32767)
                       : INT16_MIN;
  s_short.t_outlet   = (s_accumCountTOutlet > 0)
                       ? (int16_t)constrain(s_accumTOutlet / (int32_t)s_accumDuration,  -32767, 32767)
                       : INT16_MIN;
  s_short.t_hrod     = (s_accumCountTHrod > 0)
                       ? (int16_t)constrain(s_accumTHrod   / (int32_t)s_accumDuration,  -32767, 32767)
                       : INT16_MIN;

  // Wenn Averaging deaktiviert: Momentanwerte (zuletzt an tickSample übergeben)
  // statt zeit-gewichtetem Mittel speichern. Epoch/heater_power bleiben unverändert.
  if (!s_averaging) {
    s_short.powerdraw  = (int16_t)constrain(powerdraw,  -32767, 32767);
    s_short.wattneeded = (int16_t)constrain(wattneeded, -32767, 32767);
    s_short.solarpower = (int16_t)constrain(solarpower, -32767, 32767);
    s_short.t_boiler   = validT(t_boiler_c) ? (int16_t)roundf(t_boiler_c * 10.0f) : INT16_MIN;
    s_short.t_inlet    = validT(t_inlet_c)  ? (int16_t)roundf(t_inlet_c  * 10.0f) : INT16_MIN;
    s_short.t_outlet   = validT(t_outlet_c) ? (int16_t)roundf(t_outlet_c * 10.0f) : INT16_MIN;
    s_short.t_hrod     = validT(t_hrod_c)   ? (int16_t)roundf(t_hrod_c   * 10.0f) : INT16_MIN;
  }

  // Also accumulate for long-term (15min downsampling)
  if (s_accumDuration_long == 0) {
    s_accumStartEpoch_long = s_accumStartEpoch;
  }
  // Each 5-min sample represents INTERVAL_SHORT_SEC seconds of time-weighted data
  s_accumPowerdraw_long += s_short.powerdraw * (int32_t)INTERVAL_SHORT_SEC;
  s_accumWattneeded_long += s_short.wattneeded * (int32_t)INTERVAL_SHORT_SEC;
  s_accumSolarpower_long += s_short.solarpower * (int32_t)INTERVAL_SHORT_SEC;
  // heater_power is energy counter, not time-weighted - just copy latest value
  s_accumHeaterPower_long = s_short.heater_power;
  if (s_short.t_boiler != INT16_MIN) {
    s_accumTBoiler_long += s_short.t_boiler * (int32_t)INTERVAL_SHORT_SEC;
    s_accumCountTBoiler_long++;
  }
  if (s_short.t_inlet != INT16_MIN) {
    s_accumTInlet_long += s_short.t_inlet * (int32_t)INTERVAL_SHORT_SEC;
    s_accumCountTInlet_long++;
  }
  if (s_short.t_outlet != INT16_MIN) {
    s_accumTOutlet_long += s_short.t_outlet * (int32_t)INTERVAL_SHORT_SEC;
    s_accumCountTOutlet_long++;
  }
  if (s_short.t_hrod != INT16_MIN) {
    s_accumTHrod_long += s_short.t_hrod * (int32_t)INTERVAL_SHORT_SEC;
    s_accumCountTHrod_long++;
  }
  s_accumDuration_long += (uint32_t)INTERVAL_SHORT_SEC;
  s_accumCount_long++;

  histLock();

  // Store short-term sample
  s_buf_short[s_index_short] = s_short;
  s_index_short = (s_index_short + 1) % CAPACITY_SHORT;
  if (s_count_short < CAPACITY_SHORT) s_count_short++;
  s_lastSampleEpoch_short = s_short.epoch;

  // Check if 15 minutes have passed (15 * 1-minute samples)
  if (s_accumDuration_long >= 15 * 60) {
    Sample s_long{};
    s_long.epoch      = s_accumStartEpoch_long + INTERVAL_LONG_SEC / 2;
    s_long.powerdraw  = (int16_t)constrain(s_accumPowerdraw_long / (int32_t)s_accumDuration_long, -32767, 32767);
    s_long.wattneeded = (int16_t)constrain(s_accumWattneeded_long / (int32_t)s_accumDuration_long, -32767, 32767);
    s_long.solarpower = (int16_t)constrain(s_accumSolarpower_long / (int32_t)s_accumDuration_long, -32767, 32767);
    // heater_power is daily energy counter, not time-weighted - use latest value
    s_long.heater_power = (int16_t)s_accumHeaterPower_long;
    s_long.t_boiler   = (s_accumCountTBoiler_long > 0)
                        ? (int16_t)constrain(s_accumTBoiler_long / (int32_t)s_accumDuration_long, -32767, 32767)
                        : INT16_MIN;
    s_long.t_inlet    = (s_accumCountTInlet_long > 0)
                        ? (int16_t)constrain(s_accumTInlet_long  / (int32_t)s_accumDuration_long,  -32767, 32767)
                        : INT16_MIN;
    s_long.t_outlet   = (s_accumCountTOutlet_long > 0)
                        ? (int16_t)constrain(s_accumTOutlet_long / (int32_t)s_accumDuration_long,  -32767, 32767)
                        : INT16_MIN;
    s_long.t_hrod     = (s_accumCountTHrod_long > 0)
                        ? (int16_t)constrain(s_accumTHrod_long   / (int32_t)s_accumDuration_long,  -32767, 32767)
                        : INT16_MIN;

    // Wenn Averaging deaktiviert: jüngsten Momentanwert-Snapshot (s_short)
    // statt Langzeit-Mittel speichern. Epoch bleibt erhalten.
    if (!s_averaging) {
      s_long.powerdraw  = s_short.powerdraw;
      s_long.wattneeded = s_short.wattneeded;
      s_long.solarpower = s_short.solarpower;
      s_long.t_boiler   = s_short.t_boiler;
      s_long.t_inlet    = s_short.t_inlet;
      s_long.t_outlet   = s_short.t_outlet;
      s_long.t_hrod     = s_short.t_hrod;
    }

    s_buf_long[s_index_long] = s_long;
    s_index_long = (s_index_long + 1) % CAPACITY_LONG;
    if (s_count_long < CAPACITY_LONG) s_count_long++;
    s_lastSampleEpoch_long = s_long.epoch;
    
    // Reset long-term accumulator
    s_accumPowerdraw_long = 0;
    s_accumWattneeded_long = 0;
    s_accumSolarpower_long = 0;
    s_accumHeaterPower_long = 0;
    s_accumTBoiler_long = 0;
    s_accumTInlet_long  = 0;
    s_accumTOutlet_long = 0;
    s_accumTHrod_long   = 0;
    s_accumCountTBoiler_long = 0;
    s_accumCountTInlet_long  = 0;
    s_accumCountTOutlet_long = 0;
    s_accumCountTHrod_long   = 0;
    s_accumCount_long = 0;
    s_accumStartEpoch_long = 0;
  }
  
  histUnlock();

  // Reset short-term accumulator
  s_accumPowerdraw = 0;
  s_accumWattneeded = 0;
  s_accumSolarpower = 0;
  s_accumHeaterPower = 0;
  s_accumTBoiler = 0;
  s_accumTInlet  = 0;
  s_accumTOutlet = 0;
  s_accumTHrod   = 0;
  s_accumDuration = 0;
  s_accumCountTBoiler = 0;
  s_accumCountTInlet  = 0;
  s_accumCountTOutlet = 0;
  s_accumCountTHrod   = 0;
  s_accumStartEpoch = 0;
  s_lastSampleEpoch = 0;
}

size_t saveNow() {
  if (s_count_short == 0 && s_count_long == 0) return 0;
  ensurePartition();
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, /*ro*/ false, NVS_PARTITION)) return 0;
  
  p.putUInt("schema", SCHEMA_VERSION);
  p.putUInt("cnt_s", (uint32_t)s_count_short);
  p.putUInt("idx_s", (uint32_t)s_index_short);
  p.putUInt("cnt_l", (uint32_t)s_count_long);
  p.putUInt("idx_l", (uint32_t)s_index_long);
  
  size_t total = 0;
  
  // Save short-term data
  uint8_t scratch_short[CHUNK_SAMPLES_SHORT * sizeof(Sample)];
  for (size_t c = 0; c < NUM_CHUNKS_SHORT; c++) {
    histLock();
    memcpy(scratch_short, &s_buf_short[c * CHUNK_SAMPLES_SHORT], sizeof(scratch_short));
    histUnlock();
    char key[8];
    snprintf(key, sizeof(key), "ds%u", (unsigned)c);
    total += p.putBytes(key, scratch_short, sizeof(scratch_short));
  }
  
  // Save long-term data
  uint8_t scratch_long[CHUNK_SAMPLES_LONG * sizeof(Sample)];
  for (size_t c = 0; c < NUM_CHUNKS_LONG; c++) {
    histLock();
    memcpy(scratch_long, &s_buf_long[c * CHUNK_SAMPLES_LONG], sizeof(scratch_long));
    histUnlock();
    char key[8];
    snprintf(key, sizeof(key), "dl%u", (unsigned)c);
    total += p.putBytes(key, scratch_long, sizeof(scratch_long));
  }
  
  p.end();
  return total;
}

void tickSave() {
  const unsigned long now = millis();
  if (s_lastSaveMs != 0 && now - s_lastSaveMs < SAVE_INTERVAL_MS) return;
  s_lastSaveMs = now;
  saveNow();
}

size_t count()       { return s_count_short + s_count_long; }
uint32_t lastEpoch() { return s_lastSampleEpoch_short > s_lastSampleEpoch_long ? s_lastSampleEpoch_short : s_lastSampleEpoch_long; }

// ---- HTTP handlers ----------------------------------------------------
// Parse range parameters from query string
static void parseRangeParams(PsychicRequest* req, uint32_t& startEpoch, uint32_t& endEpoch) {
  time_t now; time(&now);
  const uint32_t nowEpoch = (now >= 1700000000) ? (uint32_t)now : 0;
  
  // Default: last 24 hours
  startEpoch = nowEpoch > 86400 ? nowEpoch - 86400 : 0;
  endEpoch = nowEpoch;
  
  // Check for 'range' parameter (1h, 24h, 7d, 30d, 3m)
  if (req->hasParam("range")) {
    String range = req->getParam("range")->value();
    if (range == "1h") {
      startEpoch = nowEpoch > 3600      ? nowEpoch - 3600      : 0;
    } else if (range == "6h") {
      startEpoch = nowEpoch > 21600     ? nowEpoch - 21600     : 0;
    } else if (range == "12h") {
      startEpoch = nowEpoch > 43200     ? nowEpoch - 43200     : 0;
    } else if (range == "24h" || range == "1d") {
      startEpoch = nowEpoch > 86400     ? nowEpoch - 86400     : 0;
    } else if (range == "2d") {
      startEpoch = nowEpoch > 172800    ? nowEpoch - 172800    : 0;
    } else if (range == "7d") {
      startEpoch = nowEpoch > 604800    ? nowEpoch - 604800    : 0;
    } else if (range == "14d") {
      startEpoch = nowEpoch > 1209600   ? nowEpoch - 1209600   : 0;
    } else if (range == "30d") {
      startEpoch = nowEpoch > 2592000   ? nowEpoch - 2592000   : 0;
    } else if (range == "90d" || range == "3m") {
      startEpoch = nowEpoch > 7776000   ? nowEpoch - 7776000   : 0;
    }
  }
  
  // Override with explicit timestamps if provided
  if (req->hasParam("start")) {
    startEpoch = req->getParam("start")->value().toInt();
  }
  if (req->hasParam("end")) {
    endEpoch = req->getParam("end")->value().toInt();
  }
}

// Helper: format a raw 0.1°C value into a JSON value string ("null" or "%.1f")
static void fmtTempJson(char buf[16], int16_t raw) {
  if (raw == INT16_MIN) { strcpy(buf, "null"); }
  else { snprintf(buf, 16, "%.1f", raw / 10.0f); }
}

// GET /api/history?range=24h
// Returns combined data: short-term (5min) for recent + long-term (15min) for older ranges.
static esp_err_t handleHistoryJson(PsychicRequest *req) {
  time_t now; time(&now);
  const uint32_t nowEpoch = (now >= 1700000000) ? (uint32_t)now : 0;

  uint32_t startEpoch, endEpoch;
  parseRangeParams(req, startEpoch, endEpoch);

  PsychicStreamResponse res(req, "application/json");
  res.beginSend();
  res.printf("{\"now_epoch\":%lu,\"interval_sec\":%u,\"points\":[",
             (unsigned long)nowEpoch, (unsigned)INTERVAL_SHORT_SEC);

  // Snapshot ring-buffer state under lock (short window), then stream directly.
  size_t totalShort, startIdxShort, totalLong, startIdxLong;
  uint32_t oldestShortEpoch = UINT32_MAX;
  histLock();
  totalShort    = s_count_short;
  startIdxShort = (s_index_short + CAPACITY_SHORT - s_count_short) % CAPACITY_SHORT;
  totalLong     = s_count_long;
  startIdxLong  = (s_index_long + CAPACITY_LONG - s_count_long) % CAPACITY_LONG;
  for (size_t k = 0; k < totalShort; k++) {
    const size_t i = (startIdxShort + k) % CAPACITY_SHORT;
    const uint32_t e = s_buf_short[i].epoch;
    if (e != 0 && e >= startEpoch && e <= endEpoch && e < oldestShortEpoch) oldestShortEpoch = e;
  }
  histUnlock();

  bool first = true;
  auto emit = [&](const Sample &s) {
    if (s.epoch == 0) return;
    if (s.epoch < startEpoch || s.epoch > endEpoch) return;
    long age = nowEpoch ? ((long)nowEpoch - (long)s.epoch) : 0;
    char tb[16], ti[16], to[16], th[16];
    fmtTempJson(tb, s.t_boiler);
    fmtTempJson(ti, s.t_inlet);
    fmtTempJson(to, s.t_outlet);
    fmtTempJson(th, s.t_hrod);
    res.printf("%s{\"dt_sec\":%ld,\"pd\":%d,\"wn\":%d,\"sp\":%d,\"hp\":%d,\"tb\":%s,\"ti\":%s,\"to\":%s,\"th\":%s}",
               first ? "" : ",", -age, (int)s.powerdraw, (int)s.wattneeded,
               (int)s.solarpower, (int)s.heater_power, tb, ti, to, th);
    first = false;
  };

  // Long-term (older) first, skipping overlaps
  for (size_t k = 0; k < totalLong; k++) {
    const size_t i = (startIdxLong + k) % CAPACITY_LONG;
    const Sample &s = s_buf_long[i];
    if (oldestShortEpoch != UINT32_MAX && s.epoch >= oldestShortEpoch) continue;
    emit(s);
  }
  // Short-term (newer)
  for (size_t k = 0; k < totalShort; k++) {
    const size_t i = (startIdxShort + k) % CAPACITY_SHORT;
    emit(s_buf_short[i]);
  }

  res.print("]}");
  return res.endSend();
}

// Helper: format a raw 0.1°C value into a CSV cell string ("" or "%.1f")
static void fmtTempCsv(char buf[16], int16_t raw) {
  if (raw == INT16_MIN) { buf[0] = '\0'; }
  else { snprintf(buf, 16, "%.1f", raw / 10.0f); }
}

// GET /api/history.csv?range=7d
// Returns combined short-term and long-term data as CSV
static esp_err_t handleHistoryCsv(PsychicRequest *req) {
  uint32_t startEpoch, endEpoch;
  parseRangeParams(req, startEpoch, endEpoch);

  PsychicStreamResponse res(req, "text/csv");
  res.addHeader("Content-Type", "text/csv");
  res.addHeader("Content-Disposition", "attachment; filename=\"power-history.csv\"");
  res.beginSend();
  res.print("Zeitstempel;Powerdraw_W;Wattneeded_W;SolarPower_W;HeaterPower_W;T_Speicher_C;T_Inlet_C;T_Outlet_C;T_Heizstab_C\r\n");

  // Snapshot ring-buffer state under lock (short window), then stream directly.
  size_t totalShort, startIdxShort, totalLong, startIdxLong;
  uint32_t oldestShortEpoch = UINT32_MAX;
  histLock();
  totalShort    = s_count_short;
  startIdxShort = (s_index_short + CAPACITY_SHORT - s_count_short) % CAPACITY_SHORT;
  totalLong     = s_count_long;
  startIdxLong  = (s_index_long + CAPACITY_LONG - s_count_long) % CAPACITY_LONG;
  for (size_t k = 0; k < totalShort; k++) {
    const size_t i = (startIdxShort + k) % CAPACITY_SHORT;
    const uint32_t e = s_buf_short[i].epoch;
    if (e != 0 && e >= startEpoch && e <= endEpoch && e < oldestShortEpoch) oldestShortEpoch = e;
  }
  histUnlock();

  auto emitRow = [&](const Sample &s) {
    if (s.epoch == 0) return;
    if (s.epoch < startEpoch || s.epoch > endEpoch) return;
    time_t t = (time_t)s.epoch;
    struct tm tm;
    localtime_r(&t, &tm);
    char tb[16], ti[16], to[16], th[16];
    fmtTempCsv(tb, s.t_boiler);
    fmtTempCsv(ti, s.t_inlet);
    fmtTempCsv(to, s.t_outlet);
    fmtTempCsv(th, s.t_hrod);
    res.printf("%04d-%02d-%02d %02d:%02d:%02d;%d;%d;%d;%d;%s;%s;%s;%s\r\n",
               tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,
               tm.tm_hour,tm.tm_min,tm.tm_sec,
               (int)s.powerdraw,(int)s.wattneeded,(int)s.solarpower,(int)s.heater_power,tb,ti,to,th);
  };

  // Long-term (older) first, skipping overlaps
  for (size_t k = 0; k < totalLong; k++) {
    const size_t i = (startIdxLong + k) % CAPACITY_LONG;
    const Sample &s = s_buf_long[i];
    if (oldestShortEpoch != UINT32_MAX && s.epoch >= oldestShortEpoch) continue;
    emitRow(s);
  }
  // Short-term (newer)
  for (size_t k = 0; k < totalShort; k++) {
    const size_t i = (startIdxShort + k) % CAPACITY_SHORT;
    emitRow(s_buf_short[i]);
  }

  return res.endSend();
}

// GET /api/history/info - returns storage statistics
static esp_err_t handleHistoryInfo(PsychicRequest *req) {
  time_t now; time(&now);
  const uint32_t nowEpoch = (now >= 1700000000) ? (uint32_t)now : 0;
  
  JsonDocument doc;
  doc["now_epoch"] = nowEpoch;
  doc["short_term"]["capacity"] = CAPACITY_SHORT;
  doc["short_term"]["count"] = s_count_short;
  doc["short_term"]["interval_sec"] = INTERVAL_SHORT_SEC;
  doc["short_term"]["retention_days"] = (CAPACITY_SHORT * INTERVAL_SHORT_SEC) / 86400;
  doc["long_term"]["capacity"] = CAPACITY_LONG;
  doc["long_term"]["count"] = s_count_long;
  doc["long_term"]["interval_sec"] = INTERVAL_LONG_SEC;
  doc["long_term"]["retention_days"] = (CAPACITY_LONG * INTERVAL_LONG_SEC) / 86400;
  doc["total_count"] = count();
  
  String jsonString;
  serializeJson(doc, jsonString);
  return req->reply(200, "application/json", jsonString.c_str());
}

void registerRoutes(PsychicHttpServer &srv) {
  srv.on("/api/history",     HTTP_GET, handleHistoryJson);
  srv.on("/api/history.csv", HTTP_GET, handleHistoryCsv);
  srv.on("/api/history/info", HTTP_GET, handleHistoryInfo);
}

}  // namespace History
