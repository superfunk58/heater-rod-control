#include "temp_sensors.h"
#include "webserver.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

namespace TempSensors {

// ----- Konfiguration -----
static constexpr uint32_t REQUEST_INTERVAL_MS = 5000;   // alle 5s ein neuer Read
static constexpr uint32_t CONVERSION_WAIT_MS  = 250;    // 10-bit DS18B20 needs 187.5ms; 250ms margin
static constexpr uint32_t RESCAN_INTERVAL_MS  = 10000;  // auto-rescan every 10s if sensors missing
static constexpr uint8_t  MAX_FAIL_STREAK     = 3;      // declare sensor lost after 3 consecutive fails
static constexpr uint8_t  BUS_RESET_FAILS     = 5;      // trigger bus reset after 5 consecutive fails
static constexpr uint32_t BUS_RESET_DELAY_MS  = 1000;   // delay after bus reset
static constexpr uint32_t ONEWIRE_TIMEOUT_MS  = 500;    // timeout for OneWire operations
static constexpr float    T_MIN_VALID         = -10.0f;
static constexpr float    T_MAX_VALID         = 100.0f;
static constexpr const char *NVS_NS           = "temp";

// ----- State -----
static constexpr uint8_t MAX_SENSORS = 8;

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
static uint32_t s_lastRescanMs = 0;
static uint8_t  s_failStreak[MAX_SENSORS] = {0};
static uint8_t  s_busFailStreak = 0;        // consecutive bus-level failures
static uint32_t s_busResetMs = 0;           // timestamp of last bus reset
static bool     s_busResetPending = false;  // bus reset in progress
static volatile bool s_rescanReq = false;  // set by HTTP task, serviced in tick()

// Mutex for thread-safe access to sensor values from HTTP handlers
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Non-blocking conversion state machine
enum class ConvState : uint8_t { IDLE, WAITING };
static ConvState s_convState = ConvState::IDLE;
static uint32_t  s_convStartMs = 0;

// ===== ROM <-> Helpers =====================================================

static void romToBytes(uint64_t rom, uint8_t out[8]) {
  for (int i = 0; i < 8; i++) out[i] = (rom >> (8 * i)) & 0xFF;
}
static uint64_t romFromBytes(const uint8_t in[8]) {
  uint64_t r = 0;
  for (int i = 0; i < 8; i++) r |= ((uint64_t)in[i]) << (8 * i);
  return r;
}

String romToHex(uint64_t rom) {
  uint8_t b[8]; romToBytes(rom, b);
  // Use snprintf into a fixed buffer - avoids repeated String heap reallocations.
  char buf[24];  // "AA-BB-CC-DD-EE-FF-GG-HH\0" = 23 chars + NUL
  snprintf(buf, sizeof(buf), "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
           b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);
  return String(buf);
}

uint64_t romFromHex(const String &hex) {
  uint8_t b[8] = {0};
  int idx = 0;
  String tok;
  for (size_t i = 0; i < hex.length() && idx < 8; i++) {
    char c = hex[i];
    if (c == '-' || c == ':' || c == ' ') {
      if (tok.length()) { b[idx++] = (uint8_t)strtoul(tok.c_str(), nullptr, 16); tok = ""; }
    } else {
      tok += c;
      if (tok.length() == 2) { b[idx++] = (uint8_t)strtoul(tok.c_str(), nullptr, 16); tok = ""; }
    }
  }
  if (tok.length() && idx < 8) b[idx++] = (uint8_t)strtoul(tok.c_str(), nullptr, 16);
  return romFromBytes(b);
}

// ===== Bus Recovery =======================================================

static void resetBus() {
  if (!s_wire) return;
  webLog("[Temp] Bus RESET triggered (fail streak=%u)", s_busFailStreak);
  s_wire->reset();
  s_busResetMs = millis();
  s_busResetPending = true;
  s_busFailStreak = 0;
  // Clear all sensor failure streaks
  for (uint8_t i = 0; i < MAX_SENSORS; i++) s_failStreak[i] = 0;
  // Force an immediate rescan after the recovery delay: the bus reset may have
  // re-enumerated sensors, so rebuilding the ROM list quickly helps recovery.
  s_lastRescanMs = 0;
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

  // Retry once on a bad read: transient bus noise or a pre-empted scratchpad
  // read can produce a single bad sample. A second immediate read often succeeds
  // without needing a full 3 s conversion cycle.
  for (int attempt = 0; attempt < 2; attempt++) {
    // Measure read duration only to LOG a genuinely slow bus. We must NOT reject
    // the value based on wall-clock time: getTempC() can be preempted by the
    // WiFi task mid-transaction, inflating elapsed well past the threshold even
    // though the reading is perfectly valid. Rejecting it here caused the value
    // to flicker between a real reading and null. A truly hung/disconnected bus
    // is detected by the DEVICE_DISCONNECTED_C / range checks below.
    uint32_t start = millis();
    float t = s_dt->getTempC(addr);
    uint32_t elapsed = millis() - start;
    if (elapsed > ONEWIRE_TIMEOUT_MS) {
      webLog("[Temp] Slow sensor read (%lu ms)", elapsed);
    }

    // Preserve the raw value for logging even if this attempt fails.
    out = t;
    if (t == DEVICE_DISCONNECTED_C || t < T_MIN_VALID || t > T_MAX_VALID) {
      if (attempt == 0) {
        delayMicroseconds(100);  // tiny bus settle before retry
        continue;
      }
      return false;
    }
    return true;
  }
  return false;
}

// ===== Public API ===========================================================

void begin(uint8_t pin) {
  webLog("[Temp] begin() on GPIO%d", pin);
  if (s_dt)   { delete s_dt;   s_dt   = nullptr; }
  if (s_wire) { delete s_wire; s_wire = nullptr; }
  s_count = 0;

  s_wire = new OneWire(pin);
  s_dt   = new DallasTemperature(s_wire);

  s_dt->begin();
  s_dt->setResolution(10);            // 10-bit = 187.5ms conversion, faster + more stable
  s_dt->setWaitForConversion(false);  // NON-BLOCKING — critical for WiFi stability!
  webLog("[Temp] DallasTemperature library init done (async mode)");
  loadMapping();

  // Initial scan
  uint8_t cnt = s_dt->getDeviceCount();
  webLog("[Temp] getDeviceCount() = %d", cnt);
  for (uint8_t i = 0; i < cnt && s_count < MAX_SENSORS; i++) {
    uint8_t addr[8];
    if (s_dt->getAddress(addr, i)) {
      if (addr[0] == 0x28) {
        s_roms[s_count] = romFromBytes(addr);
        s_vals[s_count] = NAN;
        webLog("[Temp]  Sensor[%d] ROM=%s", s_count, romToHex(s_roms[s_count]).c_str());
        s_count++;
      } else {
        webLog("[Temp]  Skipping non-DS18B20 (family=%02X)", addr[0]);
      }
    } else {
      webLog("[Temp]  getAddress(%d) FAILED", i);
    }
  }
  webLog("[Temp] Scan complete: %u DS18B20 sensor(s)", (unsigned)s_count);
  s_lastReadMs = millis();
  s_convState = ConvState::IDLE;
}

void tick() {
  if (!s_dt) return;

  // Service a rescan requested from another task (e.g. HTTP /api/temp/scan).
  // Done here so all OneWire bus access + array mutation stays on the loop task.
  if (s_rescanReq) {
    s_rescanReq = false;
    rescan();
  }

  // Handle bus reset delay - skip reads during recovery period
  if (s_busResetPending) {
    if (millis() - s_busResetMs < BUS_RESET_DELAY_MS) {
      return;  // still in recovery delay
    }
    s_busResetPending = false;
    webLog("[Temp] Bus recovery complete, resuming reads");
  }

  if (s_count == 0) return;  // silent: no sensors configured

  uint32_t now = millis();

  switch (s_convState) {
    case ConvState::IDLE:
      if (now - s_lastReadMs < REQUEST_INTERVAL_MS) return;
      // Issue async conversion request (~2ms for the command, NO 750ms wait)
      s_dt->requestTemperatures();
      s_convState = ConvState::WAITING;
      s_convStartMs = now;
      return;  // let loop() continue immediately — WiFi stays responsive!

    case ConvState::WAITING:
      if (now - s_convStartMs < CONVERSION_WAIT_MS) return;  // not ready yet
      s_convState = ConvState::IDLE;
      s_lastReadMs = now;
      break;  // fall through to read results
  }

  // Read all known sensors by ROM address. Track failure streaks for auto-rescan.
  bool anySensorLost = false;
  uint8_t totalFails = 0;
  for (uint8_t i = 0; i < s_count && i < MAX_SENSORS; i++) {
    float t;
    const bool ok = readSensorByRomSafe(s_roms[i], t);
    if (ok) {
      s_vals[i] = t;
      if (s_failStreak[i] > 0) {
        webLog("[Temp] Sensor %d recovered (T=%.1f)", i, t);
        s_failStreak[i] = 0;
      }
    } else {
      s_vals[i] = NAN;
      if (++s_failStreak[i] >= MAX_FAIL_STREAK) {
        anySensorLost = true;
      }
      totalFails++;
      if (s_failStreak[i] == 1) {
        webLog("[Temp] Sensor %d FAIL (streak=%d, raw=%.2f)", i, s_failStreak[i], t);
      }
    }
  }

  // Trigger bus reset if many sensors fail consecutively
  if (totalFails > 0 && totalFails == s_count) {
    s_busFailStreak++;
    if (s_busFailStreak >= BUS_RESET_FAILS) {
      resetBus();
      return;  // skip further processing this cycle
    }
  } else {
    s_busFailStreak = 0;  // reset bus fail streak if at least one sensor OK
  }

  // Auto-rescan if all sensors are lost (bus may have reset / wiring issue)
  if (anySensorLost && (now - s_lastRescanMs >= RESCAN_INTERVAL_MS)) {
    s_lastRescanMs = now;
    webLog("[Temp] Auto-rescan triggered (sensors lost)");
    rescan();
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

std::vector<Found> scanList() {
  std::vector<Found> out;
  portENTER_CRITICAL(&s_mux);
  out.reserve(s_count);
  for (uint8_t i = 0; i < s_count; i++) {
    Found f;
    f.rom = s_roms[i];
    f.current_c = s_vals[i];
    f.romHex = romToHex(s_roms[i]);
    out.push_back(f);
  }
  portEXIT_CRITICAL(&s_mux);
  return out;
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
  // The driver runs in non-blocking mode (setWaitForConversion(false)), so
  // requestTemperatures() returns before the ~187.5ms conversion finishes and
  // an immediate read would fail. Block for the conversion here (uses delay(),
  // which yields to the WiFi task), then restore async mode for tick().
  if (count > 0) {
    webLog("[Temp] Reading temperatures after rescan...");
    s_dt->setWaitForConversion(true);
    s_dt->requestTemperatures();        // blocks ~187.5ms (10-bit), WiFi stays alive
    s_dt->setWaitForConversion(false);  // restore non-blocking mode for tick()
    for (uint8_t i = 0; i < count; i++) {
      uint8_t addr[8];
      romToBytes(roms[i], addr);
      float t = s_dt->getTempC(addr);
      if (t == DEVICE_DISCONNECTED_C || t < T_MIN_VALID || t > T_MAX_VALID) {
        vals[i] = NAN;
        webLog("[Temp]  Rescan read FAIL[%d] ROM=%s", i, romToHex(roms[i]).c_str());
      } else {
        vals[i] = t;
        webLog("[Temp]  Rescan read OK [%d] ROM=%s T=%.2f C", i, romToHex(roms[i]).c_str(), t);
      }
    }
  }

  // Atomic commit of the new sensor set.
  portENTER_CRITICAL(&s_mux);
  for (uint8_t i = 0; i < count; i++) { s_roms[i] = roms[i]; s_vals[i] = vals[i]; }
  s_count = count;
  portEXIT_CRITICAL(&s_mux);
}

uint64_t boilerRom()    { return s_boilerRom;  }
uint64_t inletRom()     { return s_inletRom;   }
uint64_t outletRom()    { return s_outletRom;  }
uint64_t heaterRodRom() { return s_hrodRom;    }

void assignBoiler(uint64_t rom)    { s_boilerRom  = rom; saveMapping(); }
void assignInlet (uint64_t rom)    { s_inletRom   = rom; saveMapping(); }
void assignOutlet(uint64_t rom)    { s_outletRom  = rom; saveMapping(); }
void assignHeaterRod(uint64_t rom) { s_hrodRom    = rom; saveMapping(); }

}  // namespace TempSensors
