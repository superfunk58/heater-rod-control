// ============================================================================
// JsonArena: bump allocator over a fixed static buffer for ArduinoJson v7.
// ----------------------------------------------------------------------------
// ArduinoJson v7 JsonDocument always allocates its variant/string pools from
// the heap. On periodic paths (e.g. the 2 s status broadcast) that alloc/free
// churn slowly fragments the ESP32 heap. This allocator serves all pool blocks
// from one fixed buffer instead: deallocate() is a no-op and the whole arena
// is recycled with reset() before the document is rebuilt.
//
// NOT thread-safe — give each task its own arena instance (the loop task and
// the single esp_http_server task must not share one).
// ============================================================================
#pragma once

#include <ArduinoJson.h>
#include <cstring>

class JsonArena : public ArduinoJson::Allocator {
 public:
  JsonArena(void *buf, size_t cap) : buf_(static_cast<uint8_t *>(buf)), cap_(cap) {}

  void reset() { used_ = 0; }

  void *allocate(size_t size) override {
    const size_t total = 8 + ((size + 7) & ~(size_t)7);  // 8-byte size header, 8-byte alignment
    if (used_ + total > cap_) return nullptr;              // -> JsonDocument overflowed()
    uint8_t *hdr = buf_ + used_;
    *reinterpret_cast<uint64_t *>(hdr) = (uint64_t)size;
    used_ += total;
    return hdr + 8;
  }

  void deallocate(void *) override {}  // arena: recycled wholesale via reset()

  void *reallocate(void *ptr, size_t new_size) override {
    if (!ptr) return allocate(new_size);
    uint8_t *hdr = static_cast<uint8_t *>(ptr) - 8;
    const size_t old_size = (size_t)*reinterpret_cast<uint64_t *>(hdr);
    // Grow in place when this is the most recent block (string append path).
    if (hdr + 8 + ((old_size + 7) & ~(size_t)7) == buf_ + used_) {
      const size_t aligned = (new_size + 7) & ~(size_t)7;
      if (hdr + 8 + aligned <= buf_ + cap_) {
        *reinterpret_cast<uint64_t *>(hdr) = (uint64_t)new_size;
        used_ = (size_t)(hdr + 8 + aligned - buf_);
        return ptr;
      }
    }
    void *p = allocate(new_size);
    if (p) memcpy(p, ptr, old_size < new_size ? old_size : new_size);
    return p;
  }

 private:
  uint8_t *buf_;
  size_t cap_;
  size_t used_ = 0;
};
