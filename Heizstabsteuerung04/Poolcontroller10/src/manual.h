// =========================================================================
// Manual measurement log - chlorine (mg/L) and pH (0-14)
// =========================================================================
// The user enters lab/strip-test readings via the web UI; we store up to
// MANUAL_CAPACITY entries per kind (chlorine, pH) in the dedicated
// "histdata" NVS partition (so they survive OTA + LittleFS uploads, just
// like the auto-sampled history).
//
// Each entry is a {epoch, value} pair. Values are kept as float to allow
// 0.01 mg/L resolution for chlorine and 0.01 pH resolution.
//
// The chart UI linearly interpolates between adjacent entries to draw a
// continuous line; outside the bounded epoch range we draw nothing.
// =========================================================================
#pragma once

#include <Arduino.h>
#include <PsychicHttp.h>

namespace Manual {

// Per-kind capacity. ~2 years of weekly entries fit in 100 slots; 8 B per
// entry -> 800 B per kind, trivial in NVS.
static constexpr size_t CAPACITY = 100;

enum Kind : uint8_t { KIND_CL = 0, KIND_PH = 1, KIND_COUNT = 2 };

#pragma pack(push, 1)
struct Entry {
  uint32_t epoch;   // Unix seconds; 0 == empty slot
  float    value;   // mg/L (Cl) or 0..14 (pH)
};
#pragma pack(pop)
static_assert(sizeof(Entry) == 8, "Manual::Entry must be 8 bytes");

// Initialises NVS namespace and loads any persisted entries.
void begin();

// Append a new entry (sorted-insert by epoch). If an entry with the same
// epoch already exists for this kind, it is overwritten. Returns true on
// success, false if kind is invalid or the store is full.
bool addEntry(Kind k, uint32_t epoch, float value);

// Remove the entry with the matching epoch (if any). Returns true if
// something was actually removed.
bool removeEntry(Kind k, uint32_t epoch);

// Number of valid entries currently stored for the given kind.
size_t count(Kind k);

// Linear interpolation: returns the interpolated value at the requested
// epoch, or NAN if epoch is outside [first, last] entry of this kind or
// fewer than 1 entries exist.
float interpolate(Kind k, uint32_t epoch);

// Register HTTP routes:
//   GET    /api/manual              -> {"chlorine":[{epoch,value}], "ph":[]}
//   POST   /api/manual              -> body: {"kind":"cl|ph","value":N,"epoch":N?}
//   DELETE /api/manual?kind=cl&epoch=N
void registerRoutes(PsychicHttpServer &srv);

} // namespace Manual
