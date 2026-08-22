// =========================================================================
// Pool temperature / ORP history - ring buffer + LittleFS persistence
// See history.h for the rationale. This implementation is a trimmed-down
// port of the Feuchtesteuerung09 history module, adapted for this project's
// sensor set (pool temp + ORP + pump/flow/ely state).
// =========================================================================
#include "history.h"
#include "manual.h"

#include <Preferences.h>
#include <nvs_flash.h>
#include <time.h>
#include <math.h>
#include <memory>

namespace History {

// ----- Static state -------------------------------------------------------
// The ring buffer lives in .bss (zero-initialised). ~52 KB - ESP32-WROOM has
// ~300 KB of DRAM, this is fine.
static Sample        s_buf[CAPACITY];
static size_t        s_count = 0;          // valid samples (<= CAPACITY)
static size_t        s_index = 0;          // next slot to write
static uint32_t      s_lastSampleEpoch = 0;
static unsigned long s_lastSaveMs = 0;

// portMUX so the AsyncTCP task can snapshot state while loop() updates it.
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;

// Tunables --------------------------------------------------------------
static constexpr const char   *NVS_PARTITION      = "histdata";  // see partitions_homekit.csv
static constexpr const char   *NVS_NAMESPACE      = "hist";
static constexpr uint32_t      SAMPLE_INTERVAL_SEC = 15 * 60;  // 15 min
static constexpr unsigned long SAVE_INTERVAL_MS    = 5 * 60 * 1000; // 5 min

// Chunked blob layout in NVS. 1344 samples * 8 B = 10.5 KB total. Two
// chunks of 672 samples (= 5376 B) keep each blob well under the 4 KB
// single-page NVS entry threshold-times-a-bit and make partial rewrites
// cheap when only the tail of the ring changes.
static constexpr size_t CHUNK_SAMPLES = 672;
static constexpr size_t NUM_CHUNKS    = CAPACITY / CHUNK_SAMPLES;
static_assert(CAPACITY % CHUNK_SAMPLES == 0,
              "CAPACITY must be a multiple of CHUNK_SAMPLES");

// Schema version stored alongside the data so older/newer layouts are
// detected and ignored instead of misread. Bumped to 2 after dropping
// the actuator flags / switching to 8-byte samples.
static constexpr uint32_t SCHEMA_VERSION = 2;

// Make sure the histdata partition is initialised. The first boot after a
// fresh flash will find it blank and nvs_flash_init_partition() formats it.
static void ensurePartition() {
  static bool s_initDone = false;
  if (s_initDone) return;
  esp_err_t err = nvs_flash_init_partition(NVS_PARTITION);
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase_partition(NVS_PARTITION);
    err = nvs_flash_init_partition(NVS_PARTITION);
  }
  if (err != ESP_OK) {
    Serial.printf("[HIST] histdata init failed: %d\n", (int)err);
  }
  s_initDone = true;
}

// ----- begin() : load from histdata NVS ----------------------------------
void begin() {
  ensurePartition();
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, /*readOnly*/ true, NVS_PARTITION)) {
    return;   // no namespace yet - normal on first boot
  }

  const uint32_t schema = p.getUInt("schema", 0);
  const uint16_t cap    = p.getUShort("cap", 0);
  const uint16_t ssize  = p.getUShort("ssize", 0);
  const uint32_t cnt    = p.getUInt("count", 0);
  const uint32_t idx    = p.getUInt("index", 0);

  if (schema != SCHEMA_VERSION || cap != CAPACITY || ssize != sizeof(Sample) ||
      cnt > CAPACITY || idx >= CAPACITY) {
    p.end();
    return;   // schema mismatch - treat as empty
  }

  size_t loaded = 0;
  for (size_t c = 0; c < NUM_CHUNKS; c++) {
    char key[8];
    snprintf(key, sizeof(key), "d%u", (unsigned)c);
    const size_t bytes = CHUNK_SAMPLES * sizeof(Sample);
    uint8_t *dst       = reinterpret_cast<uint8_t *>(&s_buf[c * CHUNK_SAMPLES]);
    const size_t got   = p.getBytes(key, dst, bytes);
    if (got == bytes) loaded += CHUNK_SAMPLES;
    // short/missing chunks just leave those slots zeroed (epoch==0 == empty)
  }
  p.end();

  s_count = cnt;
  s_index = idx;
  if (s_count > 0) {
    const size_t lastSlot = (s_index + CAPACITY - 1) % CAPACITY;
    s_lastSampleEpoch = s_buf[lastSlot].epoch;
  }
  (void)loaded;   // loaded count only used for debug
}

// ----- tickSample() : record once every SAMPLE_INTERVAL_SEC --------------
void tickSample(float tempPool, int16_t orpMv) {
  // Need an absolute timestamp to be useful.
  time_t now;
  time(&now);
  if (now < 1700000000) return;       // NTP not synced (pre-2023)

  const uint32_t epoch = (uint32_t)now;
  if (s_lastSampleEpoch != 0 &&
      epoch - s_lastSampleEpoch < SAMPLE_INTERVAL_SEC) return;

  // Quantise temperature to 0.1 °C. Use TEMP_MISSING when the sensor is
  // absent (caller passes NaN) or the value is out of plausible range.
  int16_t tempTenths = TEMP_MISSING;
  if (!isnan(tempPool) && tempPool > -50.0f && tempPool < 125.0f) {
    tempTenths = (int16_t)lroundf(tempPool * 10.0f);
  }

  Sample s{};
  s.epoch      = epoch;
  s.tempTenths = tempTenths;
  s.orpMv      = orpMv;

  portENTER_CRITICAL(&s_mux);
  s_buf[s_index] = s;
  s_index = (s_index + 1) % CAPACITY;
  if (s_count < CAPACITY) s_count++;
  portEXIT_CRITICAL(&s_mux);

  s_lastSampleEpoch = epoch;
}

// ----- saveNow() : write chunked blobs to histdata NVS -------------------
size_t saveNow() {
  if (s_count == 0) return 0;
  ensurePartition();

  Preferences p;
  if (!p.begin(NVS_NAMESPACE, /*readOnly*/ false, NVS_PARTITION)) {
    return 0;
  }

  // Metadata (tiny scalars - no page-align waste).
  p.putUInt  ("schema", SCHEMA_VERSION);
  p.putUShort("cap",    (uint16_t)CAPACITY);
  p.putUShort("ssize",  (uint16_t)sizeof(Sample));
  p.putUInt  ("count",  (uint32_t)s_count);
  p.putUInt  ("index",  (uint32_t)s_index);

  // Snapshot each chunk under the lock (short windows, ~3 KB each).
  uint8_t scratch[CHUNK_SAMPLES * sizeof(Sample)];
  size_t  total = 0;
  for (size_t c = 0; c < NUM_CHUNKS; c++) {
    const uint8_t *src = reinterpret_cast<const uint8_t *>(&s_buf[c * CHUNK_SAMPLES]);
    portENTER_CRITICAL(&s_mux);
    memcpy(scratch, src, sizeof(scratch));
    portEXIT_CRITICAL(&s_mux);
    char key[8];
    snprintf(key, sizeof(key), "d%u", (unsigned)c);
    total += p.putBytes(key, scratch, sizeof(scratch));
  }
  p.end();
  return total;
}

// ----- tickSave() : rate-limited saveNow() -------------------------------
void tickSave() {
  const unsigned long now = millis();
  if (s_lastSaveMs != 0 && now - s_lastSaveMs < SAVE_INTERVAL_MS) return;
  s_lastSaveMs = now;
  saveNow();
}

size_t   count()     { return s_count; }
uint32_t lastEpoch() { return s_lastSampleEpoch; }

// =========================================================================
// HTTP handlers - PsychicStreamResponse writes chunks via esp_http_server
// =========================================================================
// PsychicHttp runs handlers synchronously on a worker-thread pool. Each
// request gets its own stack, so the "per-request heap state + shared_ptr"
// dance we needed under AsyncWebServer is gone: we just stream the data
// straight from s_buf under the portMUX.
//
// We still snapshot (total, startIdx) under the lock so the ring-buffer
// writer (loop() -> tickSample) can't race with us mid-response; the
// samples themselves are read lock-free (atomic 8-byte reads on ESP32
// are fine since we only read each slot once and loop() only writes
// non-current slots).
// =========================================================================

// ----- /api/history (JSON) -----------------------------------------------
// Response shape:
//   {"now_epoch":<nowSec>, "capacity":1344, "interval_sec":900, "points":[
//     {"dt_sec":-<age>, "t":<tempC>, "orp":<mv>}, ...
//   ]}
// "dt_sec" is negative seconds relative to now_epoch (age). Samples with
// epoch==0 or older than CAPACITY*interval_sec are skipped.
static esp_err_t handleHistoryJson(PsychicRequest *req) {
  portENTER_CRITICAL(&s_mux);
  const size_t total    = s_count;
  const size_t startIdx = (s_index + CAPACITY - s_count) % CAPACITY;
  portEXIT_CRITICAL(&s_mux);

  time_t nowSec;
  time(&nowSec);
  const uint32_t nowEpoch = (nowSec >= 1700000000) ? (uint32_t)nowSec : 0;

  PsychicStreamResponse res(req, "application/json");
  res.beginSend();

  res.printf("{\"now_epoch\":%lu,\"capacity\":%u,\"interval_sec\":%u,\"points\":[",
             (unsigned long)nowEpoch,
             (unsigned)CAPACITY,
             (unsigned)SAMPLE_INTERVAL_SEC);

  bool first = true;
  for (size_t k = 0; k < total; k++) {
    const size_t i  = (startIdx + k) % CAPACITY;
    const Sample &s = s_buf[i];
    if (s.epoch == 0) continue;

    long age = 0;
    if (nowEpoch) {
      age = (long)nowEpoch - (long)s.epoch;
      if (age < -60 || age > (long)CAPACITY * (long)SAMPLE_INTERVAL_SEC) continue;
    }

    char tempStr[16];
    if (s.tempTenths == TEMP_MISSING) strcpy(tempStr, "null");
    else snprintf(tempStr, sizeof(tempStr), "%.1f", s.tempTenths / 10.0f);
    char orpStr[16];
    if (s.orpMv == ORP_MISSING) strcpy(orpStr, "null");
    else snprintf(orpStr, sizeof(orpStr), "%d", (int)s.orpMv);

    res.printf("%s{\"dt_sec\":%ld,\"t\":%s,\"orp\":%s}",
               first ? "" : ",", -age, tempStr, orpStr);
    first = false;
  }

  res.print("]}");
  return res.endSend();
}

// ----- /api/history.csv --------------------------------------------------
static esp_err_t handleHistoryCsv(PsychicRequest *req) {
  portENTER_CRITICAL(&s_mux);
  const size_t total    = s_count;
  const size_t startIdx = (s_index + CAPACITY - s_count) % CAPACITY;
  portEXIT_CRITICAL(&s_mux);

  PsychicStreamResponse res(req, "text/csv");
  res.addHeader("Content-Disposition", "attachment; filename=\"pool-history.csv\"");
  res.beginSend();

  res.print("Zeitstempel;Temperatur_C;ORP_mV;Chlor_mgL;pH\r\n");

  for (size_t k = 0; k < total; k++) {
    const size_t i  = (startIdx + k) % CAPACITY;
    const Sample &s = s_buf[i];
    if (s.epoch == 0) continue;

    time_t t = (time_t)s.epoch;
    struct tm tm;
    localtime_r(&t, &tm);

    char tempStr[16];
    if (s.tempTenths == TEMP_MISSING) strcpy(tempStr, "");
    else snprintf(tempStr, sizeof(tempStr), "%.1f", s.tempTenths / 10.0f);
    char orpStr[16];
    if (s.orpMv == ORP_MISSING) strcpy(orpStr, "");
    else snprintf(orpStr, sizeof(orpStr), "%d", (int)s.orpMv);

    // Linearly interpolated manual readings (blank if outside range).
    char clStr[16] = "";
    char phStr[16] = "";
    const float cl = Manual::interpolate(Manual::KIND_CL, s.epoch);
    const float ph = Manual::interpolate(Manual::KIND_PH, s.epoch);
    if (!isnan(cl)) snprintf(clStr, sizeof(clStr), "%.2f", cl);
    if (!isnan(ph)) snprintf(phStr, sizeof(phStr), "%.2f", ph);

    res.printf("%04d-%02d-%02d %02d:%02d:%02d;%s;%s;%s;%s\r\n",
               tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
               tm.tm_hour, tm.tm_min, tm.tm_sec,
               tempStr, orpStr, clStr, phStr);
  }

  return res.endSend();
}

void registerRoutes(PsychicHttpServer &srv) {
  srv.on("/api/history",      HTTP_GET, handleHistoryJson);
  // Two URLs for the CSV: the canonical "/api/history.csv" plus a dot-less
  // alias "/api/history-csv". Some HTTP clients (and the esp_http_server
  // routing layer in some configurations) get confused by an explicit dot
  // extension on a dynamic route, so the alias is the reliable fallback.
  srv.on("/api/history.csv",  HTTP_GET, handleHistoryCsv);
  srv.on("/api/history-csv",  HTTP_GET, handleHistoryCsv);
}

}  // namespace History
