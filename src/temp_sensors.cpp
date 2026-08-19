#include "temp_sensors.h"
#include "webserver.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

namespace TempSensors {

// ----- Konfiguration -----
static constexpr uint32_t REQUEST_INTERVAL_MS = 10000;  // alle 10s ein neuer Read
static constexpr float    T_MIN_VALID         = -55.0f;  // DS18B20 spec: -55°C to +125°C
static constexpr float    T_MAX_VALID         = 100.0f;
static constexpr float    T_POWER_ON_DEFAULT  = 85.0f;   // DS18B20 power-on scratchpad default
static constexpr const char *NVS_NS           = "temp";

// ----- State -----
// MAX_SENSORS lives in temp_sensors.h (shared with callers of scanList()).

static OneWire           *s_wire = nullptr;
static DallasTemperature *s_dt   = nullptr;

static uint64_t s_boilerRom = 0;
static uint64_t s_inletRom  = 0;
static uint64_t s_outletRom = 0;
static uint64_t s_hrodRom   = 0;

static float s_boilerLast = NAN;
static float s_inletLast  = NAN;
static float s_outletLast = NAN;
static float s_hrodLast   = NAN;

static uint32_t s_lastReadMs = 0;
static volatile bool s_rescanReq = false;  // set by HTTP task, serviced in tick()

// Mutex for thread-safe access to sensor values from HTTP handlers
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Deferred assignment state (set by httpd task, drained by loop task in tick())
volatile bool     g_tempAssignPending = false;
PendingAssign     g_tempPendingAssign;

// ===== ROM <-> Helpers =====================================================

static void romToBytes(uint64_t rom, uint8_t out[8]) {
  for (int i = 0; i < 8; i++) out[i] = (rom >> (8 * i)) & 0xFF;
}
static uint64_t romFromBytes(const uint8_t in[8]) {
  uint64_t r = 0;
  for (int i = 0; i < 8; i++) r |= ((uint64_t)in[i]) << (8 * i);
  return r;
}

void romToHex(uint64_t rom, char out[24]) {
  uint8_t b[8]; romToBytes(rom, b);
  // snprintf into the caller's fixed buffer - zero heap allocation.
  snprintf(out, 24, "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
           b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
}

uint64_t romFromHex(const char *hex) {
  // Parse hex digit pairs directly; separators ('-', ':', ' ') are skipped.
  // No String tokenizer -> no heap allocation.
  uint8_t b[8] = {0};
  int idx = 0;
  int hi = -1;  // pending high nibble
  for (const char *p = hex; *p && idx < 8; p++) {
    const char c = *p;
    int v = -1;
    if      (c >= '0' && c <= '9') v = c - '0';
    else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
    if (v < 0) continue;  // separator
    if (hi < 0) { hi = v; }
    else { b[idx++] = (uint8_t)((hi << 4) | v); hi = -1; }
  }
  return romFromBytes(b);
}

// ===== NVS-Persistenz =======================================================

static void loadMapping() {
  Preferences p;
  if (p.begin(NVS_NS, /*ro*/ true)) {
    s_boilerRom = p.getULong64("boiler", 0);
    s_inletRom  = p.getULong64("inlet",  0);
    s_outletRom = p.getULong64("outlet", 0);
    s_hrodRom   = p.getULong64("hrod",   0);
    p.end();
  }
}

static void saveMapping() {
  Preferences p;
  if (p.begin(NVS_NS, /*ro*/ false)) {
    p.putULong64("boiler", s_boilerRom);
    p.putULong64("inlet",  s_inletRom);
    p.putULong64("outlet", s_outletRom);
    p.putULong64("hrod",   s_hrodRom);
    p.end();
  }
}

// ===== Scan state (simple array, no heap during runtime) ====================

static uint8_t  s_count = 0;
static uint64_t s_roms[MAX_SENSORS];
static float    s_vals[MAX_SENSORS];

static bool readSensorByRomSafe(uint64_t rom, float &out) {
  if (!s_dt || s_count == 0) return false;
  uint8_t addr[8];
  romToBytes(rom, addr);
  float t = s_dt->getTempC(addr);
  out = t;
  if (t == DEVICE_DISCONNECTED_C || t == T_POWER_ON_DEFAULT ||
      t < T_MIN_VALID || t > T_MAX_VALID) {
    return false;
  }
  return true;
}

// ===== Public API ===========================================================

void begin(uint8_t pin) {
  webLog("[Temp] begin() on GPIO%d", pin);
  if (s_dt)   { delete s_dt;   s_dt   = nullptr; }
  if (s_wire) { delete s_wire; s_wire = nullptr; }
  s_count = 0;
  s_rescanReq = false;
  for (uint8_t i = 0; i < MAX_SENSORS; i++) s_vals[i] = NAN;
  s_boilerLast = s_inletLast = s_outletLast = s_hrodLast = NAN;

  s_wire = new OneWire(pin);
  s_dt   = new DallasTemperature(s_wire);

  // Bus recovery before the first scan: a warm reboot (panic/WDT) can leave a
  // sensor mid-transaction holding the bus low, so the very first search finds
  // nothing. Extra reset pulses + settle time release it.
  s_wire->reset();
  delay(10);
  s_wire->reset();
  delay(10);

  s_dt->begin();
  s_dt->setResolution(10);            // 10-bit = 187.5ms conversion, 0.25°C precision
  s_dt->setWaitForConversion(true);   // blocking mode — library handles conversion timing
  webLog("[Temp] DallasTemperature library init done (blocking mode)");
  loadMapping();

  // Initial scan. Up to 2 attempts: if the bus was still settling (or a sensor
  // was mid-transaction at reboot), the first search can find nothing while a
  // second one succeeds. Boot-time only — no runtime auto-rescan.
  for (uint8_t attempt = 0; attempt < 2 && s_count == 0; attempt++) {
    if (attempt > 0) {
      webLog("[Temp] Boot scan found 0 sensors, retrying after bus reset");
      s_wire->reset();
      delay(50);
      s_dt->begin();
      s_dt->setResolution(12);
      s_dt->setWaitForConversion(true);
    }
    uint8_t cnt = s_dt->getDeviceCount();
    webLog("[Temp] getDeviceCount() = %d", cnt);
    for (uint8_t i = 0; i < cnt && s_count < MAX_SENSORS; i++) {
      uint8_t addr[8];
      if (s_dt->getAddress(addr, i)) {
        if (addr[0] == 0x28) {
          s_roms[s_count] = romFromBytes(addr);
          s_vals[s_count] = NAN;
          char hex[24]; romToHex(s_roms[s_count], hex);
          webLog("[Temp]  Sensor[%d] ROM=%s", s_count, hex);
          s_count++;
        } else {
          webLog("[Temp]  Skipping non-DS18B20 (family=%02X)", addr[0]);
        }
      } else {
        webLog("[Temp]  getAddress(%d) FAILED", i);
      }
    }
  }
  webLog("[Temp] Scan complete: %u DS18B20 sensor(s)", (unsigned)s_count);
  s_lastReadMs = millis();
}

void tick() {
  if (!s_dt) return;

  const uint32_t now = millis();

  // Drain deferred sensor assignments from the httpd task (handleTempAssign).
  // uint64_t writes + NVS save must happen on the loop task to avoid torn reads
  // and blocking flash writes from the httpd task.
  if (g_tempAssignPending) {
    g_tempAssignPending = false;
    portENTER_CRITICAL(&s_mux);
    if (g_tempPendingAssign.hasBoiler) s_boilerRom  = g_tempPendingAssign.boilerRom;
    if (g_tempPendingAssign.hasInlet ) s_inletRom   = g_tempPendingAssign.inletRom;
    if (g_tempPendingAssign.hasOutlet) s_outletRom  = g_tempPendingAssign.outletRom;
    if (g_tempPendingAssign.hasHrod  ) s_hrodRom    = g_tempPendingAssign.hrodRom;
    portEXIT_CRITICAL(&s_mux);
    g_tempPendingAssign = PendingAssign();  // clear for next time
    saveMapping();
    webLog("[Temp] Sensor assignment applied (deferred from httpd)");
  }

  // Service a rescan requested from another task (e.g. HTTP /api/temp/scan).
  // Done here so all OneWire bus access + array mutation stays on the loop task.
  if (s_rescanReq) {
    s_rescanReq = false;
    rescan();
  }

  if (s_count == 0) return;  // no sensors found; only boot scan or manual rescan re-enumerates

  // Rate-limit reads
  if (now - s_lastReadMs < REQUEST_INTERVAL_MS) return;
  s_lastReadMs = now;

  // Blocking conversion: 10-bit resolution blocks ~187.5ms.
  s_dt->requestTemperatures();

  // Read all known sensors by ROM address.
  for (uint8_t i = 0; i < s_count && i < MAX_SENSORS; i++) {
    float t = NAN;
    const bool ok = readSensorByRomSafe(s_roms[i], t);
    if (ok) {
      s_vals[i] = t;
    }
    // On failure: keep last good value (s_vals[i] unchanged)
  }

  // Map to roles
  s_boilerLast = NAN; s_inletLast = NAN; s_outletLast = NAN; s_hrodLast = NAN;
  for (uint8_t i = 0; i < s_count; i++) {
    if (!(s_vals[i] > -50.0f && s_vals[i] < 150.0f)) continue;
    if (s_boilerRom && s_roms[i] == s_boilerRom) s_boilerLast = s_vals[i];
    if (s_inletRom  && s_roms[i] == s_inletRom ) s_inletLast  = s_vals[i];
    if (s_outletRom && s_roms[i] == s_outletRom) s_outletLast = s_vals[i];
    if (s_hrodRom   && s_roms[i] == s_hrodRom  ) s_hrodLast   = s_vals[i];
  }
}

uint8_t sensorCount() { return s_count; }

float getBoilerC()    { return s_boilerLast; }
float getInletC()     { return s_inletLast;  }
float getOutletC()    { return s_outletLast; }
float getHeaterRodC() { return s_hrodLast;   }

uint8_t scanList(Found out[], uint8_t maxCount) {
  // Snapshot ROMs/values under the lock (fast), format hex strings outside it.
  portENTER_CRITICAL(&s_mux);
  const uint8_t n = (s_count < maxCount) ? s_count : maxCount;
  for (uint8_t i = 0; i < n; i++) {
    out[i].rom = s_roms[i];
    out[i].current_c = s_vals[i];
  }
  portEXIT_CRITICAL(&s_mux);
  for (uint8_t i = 0; i < n; i++) romToHex(out[i].rom, out[i].romHex);
  return n;
}

void requestRescan() { s_rescanReq = true; }
bool rescanPending() { return s_rescanReq; }

void rescan() {
  if (!s_dt) return;
  webLog("[Temp] Manual rescan triggered");
  s_dt->begin();

  // Build into locals first; commit to the shared arrays under the lock so
  // scanList()/tick() readers (other task) never see a half-updated state.
  uint64_t roms[MAX_SENSORS];
  float    vals[MAX_SENSORS];
  uint8_t  count = 0;

  uint8_t cnt = s_dt->getDeviceCount();
  for (uint8_t i = 0; i < cnt && count < MAX_SENSORS; i++) {
    uint8_t addr[8];
    if (s_dt->getAddress(addr, i) && addr[0] == 0x28) {
      roms[count] = romFromBytes(addr);
      vals[count] = NAN;
      count++;
    }
  }
  webLog("[Temp] Manual rescan complete: %u sensors", (unsigned)count);

  // Read temperatures immediately so scanList returns live values.
  if (count > 0) {
    webLog("[Temp] Reading temperatures after rescan...");
    s_dt->requestTemperatures();  // blocks ~187.5ms (10-bit)
    for (uint8_t i = 0; i < count; i++) {
      uint8_t addr[8];
      romToBytes(roms[i], addr);
      char hex[24]; romToHex(roms[i], hex);
      float t = s_dt->getTempC(addr);
      if (t == DEVICE_DISCONNECTED_C || t == T_POWER_ON_DEFAULT ||
          t < T_MIN_VALID || t > T_MAX_VALID) {
        vals[i] = NAN;
        webLog("[Temp]  Rescan read FAIL[%d] ROM=%s", i, hex);
      } else {
        vals[i] = t;
        webLog("[Temp]  Rescan read OK [%d] ROM=%s T=%.2f C", i, hex, t);
      }
    }
  }

  // Atomic commit of the new sensor set.
  portENTER_CRITICAL(&s_mux);
  for (uint8_t i = 0; i < count; i++) { s_roms[i] = roms[i]; s_vals[i] = vals[i]; }
  s_count = count;
  portEXIT_CRITICAL(&s_mux);

  // Next regular read happens after REQUEST_INTERVAL_MS.
  s_lastReadMs = millis();
}

// Thread-safe ROM getters: uint64_t is 8 bytes on a 32-bit CPU, so an
// unsynchronized read can see a torn value if another task writes mid-read.
// portENTER_CRITICAL is a brief spinlock-disable — sub-microsecond for a copy.
uint64_t boilerRom()    { portENTER_CRITICAL(&s_mux); uint64_t r = s_boilerRom;  portEXIT_CRITICAL(&s_mux); return r; }
uint64_t inletRom()     { portENTER_CRITICAL(&s_mux); uint64_t r = s_inletRom;   portEXIT_CRITICAL(&s_mux); return r; }
uint64_t outletRom()    { portENTER_CRITICAL(&s_mux); uint64_t r = s_outletRom;  portEXIT_CRITICAL(&s_mux); return r; }
uint64_t heaterRodRom() { portENTER_CRITICAL(&s_mux); uint64_t r = s_hrodRom;   portEXIT_CRITICAL(&s_mux); return r; }

// Deferred assignment: HTTP handlers call these, which set the pending struct.
// The actual uint64_t write + NVS save happens in tick() on the loop task.
void assignBoiler(uint64_t rom)    { g_tempPendingAssign.hasBoiler = true; g_tempPendingAssign.boilerRom = rom; g_tempAssignPending = true; }
void assignInlet (uint64_t rom)    { g_tempPendingAssign.hasInlet  = true; g_tempPendingAssign.inletRom  = rom; g_tempAssignPending = true; }
void assignOutlet(uint64_t rom)    { g_tempPendingAssign.hasOutlet = true; g_tempPendingAssign.outletRom = rom; g_tempAssignPending = true; }
void assignHeaterRod(uint64_t rom) { g_tempPendingAssign.hasHrod   = true; g_tempPendingAssign.hrodRom   = rom; g_tempAssignPending = true; }

}  // namespace TempSensors
