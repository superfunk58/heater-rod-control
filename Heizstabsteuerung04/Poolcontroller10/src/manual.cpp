// =========================================================================
// Manual measurement log (chlorine + pH) - implementation
// =========================================================================
#include "manual.h"

#include <Preferences.h>
#include <ArduinoJson.h>
#include <math.h>
#include <time.h>
#include <string.h>

namespace Manual {

// ---- State --------------------------------------------------------------
static Entry s_buf[KIND_COUNT][CAPACITY];
static size_t s_count[KIND_COUNT] = {0, 0};

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static constexpr const char *NVS_PARTITION = "histdata";
static constexpr const char *NVS_NAMESPACE = "manual";

static const char *kindKey(Kind k) {
  return (k == KIND_CL) ? "cl" : "ph";
}

// ---- Persistence --------------------------------------------------------
static void persistKind(Kind k) {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, /*ro*/ false, NVS_PARTITION)) return;
  char key[16];
  snprintf(key, sizeof(key), "%s_buf", kindKey(k));
  p.putBytes(key, s_buf[k], sizeof(s_buf[k]));
  snprintf(key, sizeof(key), "%s_n", kindKey(k));
  p.putUInt(key, (uint32_t)s_count[k]);
  p.end();
}

void begin() {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, /*ro*/ true, NVS_PARTITION)) return;
  for (uint8_t k = 0; k < KIND_COUNT; k++) {
    char key[16];
    snprintf(key, sizeof(key), "%s_buf", kindKey((Kind)k));
    p.getBytes(key, s_buf[k], sizeof(s_buf[k]));
    snprintf(key, sizeof(key), "%s_n", kindKey((Kind)k));
    const uint32_t n = p.getUInt(key, 0);
    s_count[k] = (n <= CAPACITY) ? n : 0;
  }
  p.end();
}

// ---- Mutators -----------------------------------------------------------
bool addEntry(Kind k, uint32_t epoch, float value) {
  if (k >= KIND_COUNT || epoch < 1700000000UL || isnan(value)) return false;

  portENTER_CRITICAL(&s_mux);
  Entry *buf  = s_buf[k];
  size_t n    = s_count[k];

  // Overwrite if epoch already present.
  for (size_t i = 0; i < n; i++) {
    if (buf[i].epoch == epoch) {
      buf[i].value = value;
      portEXIT_CRITICAL(&s_mux);
      persistKind(k);
      return true;
    }
  }

  if (n >= CAPACITY) {
    portEXIT_CRITICAL(&s_mux);
    return false;
  }

  // Sorted insert by epoch.
  size_t pos = n;
  while (pos > 0 && buf[pos - 1].epoch > epoch) {
    buf[pos] = buf[pos - 1];
    pos--;
  }
  buf[pos] = {epoch, value};
  s_count[k] = n + 1;
  portEXIT_CRITICAL(&s_mux);

  persistKind(k);
  return true;
}

bool removeEntry(Kind k, uint32_t epoch) {
  if (k >= KIND_COUNT) return false;
  bool removed = false;

  portENTER_CRITICAL(&s_mux);
  Entry *buf = s_buf[k];
  size_t n   = s_count[k];
  for (size_t i = 0; i < n; i++) {
    if (buf[i].epoch == epoch) {
      for (size_t j = i; j + 1 < n; j++) buf[j] = buf[j + 1];
      buf[n - 1] = {0, 0.0f};
      s_count[k] = n - 1;
      removed = true;
      break;
    }
  }
  portEXIT_CRITICAL(&s_mux);

  if (removed) persistKind(k);
  return removed;
}

size_t count(Kind k) { return (k < KIND_COUNT) ? s_count[k] : 0; }

float interpolate(Kind k, uint32_t epoch) {
  if (k >= KIND_COUNT) return NAN;
  portENTER_CRITICAL(&s_mux);
  const Entry *buf = s_buf[k];
  const size_t n   = s_count[k];

  if (n == 0) { portEXIT_CRITICAL(&s_mux); return NAN; }
  if (n == 1) {
    const float v = (buf[0].epoch == epoch) ? buf[0].value : NAN;
    portEXIT_CRITICAL(&s_mux);
    return v;
  }
  if (epoch < buf[0].epoch || epoch > buf[n - 1].epoch) {
    portEXIT_CRITICAL(&s_mux);
    return NAN;
  }
  // Binary search for the bracketing pair.
  size_t lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const size_t mid = (lo + hi) / 2;
    if (buf[mid].epoch <= epoch) lo = mid; else hi = mid;
  }
  const uint32_t e0 = buf[lo].epoch, e1 = buf[hi].epoch;
  const float    v0 = buf[lo].value, v1 = buf[hi].value;
  portEXIT_CRITICAL(&s_mux);

  if (e1 == e0) return v0;
  const float frac = (float)(epoch - e0) / (float)(e1 - e0);
  return v0 + frac * (v1 - v0);
}

// =========================================================================
// HTTP handlers
// =========================================================================
static const char *kindFromStr(const char *s) {
  if (!s) return nullptr;
  if (!strcasecmp(s, "cl") || !strcasecmp(s, "chlorine") || !strcasecmp(s, "chlor"))
    return "cl";
  if (!strcasecmp(s, "ph"))
    return "ph";
  return nullptr;
}

static esp_err_t handleGet(PsychicRequest *req) {
  DynamicJsonDocument doc(8192);
  for (uint8_t k = 0; k < KIND_COUNT; k++) {
    JsonArray arr = doc.createNestedArray((k == KIND_CL) ? "chlorine" : "ph");
    portENTER_CRITICAL(&s_mux);
    const size_t n = s_count[k];
    for (size_t i = 0; i < n; i++) {
      JsonObject o = arr.createNestedObject();
      o["epoch"] = s_buf[k][i].epoch;
      o["value"] = s_buf[k][i].value;
    }
    portEXIT_CRITICAL(&s_mux);
  }
  String out; serializeJson(doc, out);
  return req->reply(200, "application/json", out.c_str());
}

static esp_err_t handlePost(PsychicRequest *req) {
  const String &body = req->body();
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) {
    return req->reply(400, "text/plain", "bad json");
  }
  const char *kindStr = doc["kind"] | "";
  const char *kc = kindFromStr(kindStr);
  if (!kc) return req->reply(400, "text/plain", "kind must be 'cl' or 'ph'");

  if (!doc["value"].is<float>() && !doc["value"].is<int>() &&
      !doc["value"].is<double>()) {
    return req->reply(400, "text/plain", "value missing");
  }
  const float value = doc["value"].as<float>();

  uint32_t epoch = doc["epoch"] | 0UL;
  if (epoch == 0) {
    time_t now; time(&now);
    if (now < 1700000000) return req->reply(503, "text/plain", "ntp not synced");
    epoch = (uint32_t)now;
  }

  const Kind k = (!strcmp(kc, "cl")) ? KIND_CL : KIND_PH;
  if (!addEntry(k, epoch, value)) {
    return req->reply(507, "text/plain", "store full");
  }
  return req->reply(200, "application/json", "{\"ok\":true}");
}

static esp_err_t handleDelete(PsychicRequest *req) {
  const String kindStr = req->getParam("kind") ? req->getParam("kind")->value() : String();
  const String epochStr = req->getParam("epoch") ? req->getParam("epoch")->value() : String();
  const char *kc = kindFromStr(kindStr.c_str());
  if (!kc) return req->reply(400, "text/plain", "kind must be 'cl' or 'ph'");
  const uint32_t epoch = (uint32_t)strtoul(epochStr.c_str(), nullptr, 10);
  if (epoch == 0) return req->reply(400, "text/plain", "epoch missing");
  const Kind k = (!strcmp(kc, "cl")) ? KIND_CL : KIND_PH;
  const bool ok = removeEntry(k, epoch);
  return req->reply(ok ? 200 : 404, "text/plain", ok ? "ok" : "not found");
}

void registerRoutes(PsychicHttpServer &srv) {
  srv.on("/api/manual", HTTP_GET,    handleGet);
  srv.on("/api/manual", HTTP_POST,   handlePost);
  srv.on("/api/manual", HTTP_DELETE, handleDelete);
}

} // namespace Manual
