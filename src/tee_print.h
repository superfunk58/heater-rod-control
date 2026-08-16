// ============================================================================
// TeePrint: Print subclass that mirrors all output to HardwareSerial AND
// the webLog ring buffer. Installed at boot so every Serial.print* call
// also appears in the web UI log window — no flash writes, no LittleFS
// involvement, just a RAM ring buffer pushed via SSE.
//
// Usage: replace `Serial.print*` with `Log.print*` in application code.
// The class accumulates characters until a newline, then stores the
// complete line into the webLog ring buffer (thread-safe via mutex).
// ============================================================================
#pragma once
#include <Arduino.h>

// Forward declaration — implemented in webserver.cpp
// Stores a complete line into the webLog ring buffer (no Serial output).
void webLogBufferLine(const char *line);

class TeePrint : public Print {
 public:
  TeePrint(HardwareSerial &serial) : _serial(serial) {}

  size_t write(uint8_t c) override {
    _serial.write(c);
    _append((char)c);
    return 1;
  }

  size_t write(const uint8_t *buf, size_t size) override {
    _serial.write(buf, size);
    for (size_t i = 0; i < size; i++) _append((char)buf[i]);
    return size;
  }

  // printf with \n auto-appended (matches Serial.println convenience)
  int printf(const char *format, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, format);
    char buf[128];
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len > 0) {
      size_t w = (size_t)len < sizeof(buf) ? (size_t)len : sizeof(buf) - 1;
      write((const uint8_t *)buf, w);
    }
    return len;
  }

  void flush() { _serial.flush(); }

 private:
  HardwareSerial &_serial;
  char _line[128];
  size_t _pos = 0;

  void _append(char c) {
    if (c == '\r') return;
    if (c == '\n') {
      _line[_pos] = '\0';
      if (_pos > 0) webLogBufferLine(_line);
      _pos = 0;
    } else if (_pos < sizeof(_line) - 1) {
      _line[_pos++] = c;
    } else {
      _line[sizeof(_line) - 1] = '\0';
      webLogBufferLine(_line);
      _pos = 0;
      _line[_pos++] = c;
    }
  }
};

// Global instance — use `Log.print*` instead of `Serial.print*`
extern TeePrint Log;
