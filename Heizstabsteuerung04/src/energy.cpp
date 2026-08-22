#include "energy.h"
#include <Preferences.h>
#include <time.h>

namespace Energy {

static constexpr const char *NVS_NS = "energy";
static constexpr unsigned long SAVE_INTERVAL_MS = 5UL * 60UL * 1000UL;
static constexpr uint32_t      MAX_DT_MS        = 5000;  // skip if gap too large

// Persistent state
static double        s_cumulWh        = 0.0;   // current month's Wh (fine grained)
static double        s_lifetimeWh     = 0.0;   // total since first start
static uint32_t      s_startEpoch     = 0;     // first integration start (epoch)
static uint16_t      s_curYear        = 0;     // tracked month for rollover (0 = unset)
static uint8_t       s_curMonth       = 0;
static MonthlyChunk  s_chunks[MONTHLY_CHUNKS] = {};  // ring buffer, oldest first

// Volatile
static uint32_t      s_lastTickMs     = 0;
static unsigned long s_lastSaveMs     = 0;
static bool          s_dirty          = false;

// ---- NVS persistence -----------------------------------------------------

static void load() {
  Preferences p;
  if (!p.begin(NVS_NS, /*ro*/ true)) return;
  s_cumulWh    = p.getDouble("cw", 0.0);
  s_lifetimeWh = p.getDouble("lw", 0.0);
  s_startEpoch = p.getUInt  ("se", 0);
  s_curYear    = p.getUShort("cy", 0);
  s_curMonth   = p.getUChar ("cm", 0);
  size_t n = p.getBytes("ch", s_chunks, sizeof(s_chunks));
  if (n != sizeof(s_chunks)) {
    memset(s_chunks, 0, sizeof(s_chunks));
  }
  p.end();
}

void saveNow() {
  Preferences p;
  if (!p.begin(NVS_NS, /*ro*/ false)) return;
  p.putDouble("cw", s_cumulWh);
  p.putDouble("lw", s_lifetimeWh);
  p.putUInt  ("se", s_startEpoch);
  p.putUShort("cy", s_curYear);
  p.putUChar ("cm", s_curMonth);
  p.putBytes ("ch", s_chunks, sizeof(s_chunks));
  p.end();
  s_dirty = false;
  s_lastSaveMs = millis();
}

void begin() {
  load();
}

// ---- Monthly rollover ----------------------------------------------------

static void pushChunk(uint16_t year, uint8_t month, uint32_t wh) {
  // Shift left, append newest at end (ring keeps last MONTHLY_CHUNKS entries)
  for (size_t i = 0; i + 1 < MONTHLY_CHUNKS; i++) s_chunks[i] = s_chunks[i + 1];
  MonthlyChunk &slot = s_chunks[MONTHLY_CHUNKS - 1];
  slot.year  = year;
  slot.month = month;
  slot._pad  = 0;
  slot.wh    = wh;
}

// ---- Public API ----------------------------------------------------------

void tick(int watts) {
  const uint32_t now = millis();
  if (s_lastTickMs == 0) {
    s_lastTickMs = now;
    return;
  }
  uint32_t dt = now - s_lastTickMs;
  s_lastTickMs = now;
  if (dt == 0 || dt > MAX_DT_MS) return;  // skip dubious deltas

  if (watts <= 0) return;  // nothing to integrate

  // Integrate: Wh += W * dt_ms / 3_600_000
  const double inc = (double)watts * (double)dt / 3600000.0;
  s_cumulWh    += inc;
  s_lifetimeWh += inc;
  s_dirty = true;

  // Anchor start epoch on first real integration (needs NTP)
  if (s_startEpoch == 0) {
    time_t t; time(&t);
    if (t > 1700000000) s_startEpoch = (uint32_t)t;
  }

  // Month rollover: check only every 60s to avoid time() calls every loop
  static uint32_t s_lastMonthCheckMs = 0;
  if (now - s_lastMonthCheckMs >= 60000) {
    s_lastMonthCheckMs = now;
    time_t now_t; time(&now_t);
    if (now_t > 1700000000) {
      struct tm tm; localtime_r(&now_t, &tm);
      const uint16_t y = (uint16_t)(tm.tm_year + 1900);
      const uint8_t  m = (uint8_t)(tm.tm_mon + 1);
      if (s_curYear == 0) {
        s_curYear = y; s_curMonth = m;
      } else if (y != s_curYear || m != s_curMonth) {
        pushChunk(s_curYear, s_curMonth, (uint32_t)(s_cumulWh + 0.5));
        s_cumulWh = 0.0;
        s_curYear = y; s_curMonth = m;
        saveNow();
      }
    }
  }
}

void tickSave() {
  if (!s_dirty) return;
  const unsigned long now = millis();
  if (s_lastSaveMs != 0 && now - s_lastSaveMs < SAVE_INTERVAL_MS) return;
  saveNow();
}

void resetAll() {
  s_cumulWh = 0.0;
  s_lifetimeWh = 0.0;
  s_startEpoch = 0;
  s_curYear = 0; s_curMonth = 0;
  memset(s_chunks, 0, sizeof(s_chunks));
  saveNow();
}

void fillStatus(JsonVariant doc) {
  doc["energy_current_wh"]  = (uint32_t)(s_cumulWh + 0.5);
  doc["energy_lifetime_wh"] = (uint32_t)(s_lifetimeWh + 0.5);
  doc["energy_start_epoch"] = s_startEpoch;
  doc["energy_year"]        = s_curYear;
  doc["energy_month"]       = s_curMonth;
  JsonArray arr = doc["energy_history"].to<JsonArray>();
  for (size_t i = 0; i < MONTHLY_CHUNKS; i++) {
    const MonthlyChunk &c = s_chunks[i];
    if (c.year == 0) continue;
    JsonObject o = arr.add<JsonObject>();
    o["year"]  = c.year;
    o["month"] = c.month;
    o["wh"]    = c.wh;
  }
}

}  // namespace Energy
