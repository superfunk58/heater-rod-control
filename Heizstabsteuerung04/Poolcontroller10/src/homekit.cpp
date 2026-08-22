// =========================================================================
// homekit.cpp  -  HomeSpan HAP integration for the Pool Controller
// =========================================================================
// Exposes a bridge with 5 accessories: Pump, Ely A/B, Ely K, Pool-Temp,
// Flow-Contact. Writable switches use Service subclasses (so update() can
// mirror changes into main.cpp globals). Read-only sensors just hold bare
// SpanCharacteristic pointers; homekitLoop() pushes fresh values each tick.
// =========================================================================
#include "homekit.h"
#include "HomeSpan.h"
#include <WiFi.h>
#include <mdns.h>
#include <nvs_flash.h>

// ---- Globals owned by main.cpp ------------------------------------------
extern bool           pumping;
extern bool           elyKcommand;
extern uint8_t        elyABPhase;
extern unsigned long  elyABLastSwitchMs;
extern bool           flowing;
extern float          temp_pool;
extern float          temp_air;
extern const char    *FIRMWARE_VERSION;
extern const char    *DEVICE_HOSTNAME;

static const char *HK_SETUP_CODE = "46637726";   // "466-37-726" in Home.app

// ---- Accessory-info helper ----------------------------------------------
// The 6 characteristics below are mandatory on every HomeKit accessory.
static void info(const char *name, const char *serial, const char *model) {
  new Service::AccessoryInformation();
    new Characteristic::Identify();
    new Characteristic::Name(name);
    new Characteristic::Manufacturer("DIY");
    new Characteristic::SerialNumber(serial);
    new Characteristic::Model(model);
    new Characteristic::FirmwareRevision(FIRMWARE_VERSION);
}

// ---- Writable switch services -------------------------------------------
struct HK_Pump : Service::Switch {
  SpanCharacteristic *on = new Characteristic::On(pumping);
  boolean update() override { pumping = on->getNewVal(); return true; }
  void    loop()   override { if (on->getVal<bool>() != pumping) on->setVal(pumping); }
};

struct HK_ElyAB : Service::Switch {
  SpanCharacteristic *on = new Characteristic::On(elyABPhase != 0);
  boolean update() override {
    const bool want = on->getNewVal();
    if (want && elyABPhase == 0) { elyABPhase = 1; elyABLastSwitchMs = millis(); }
    else if (!want)              { elyABPhase = 0; }
    return true;
  }
  void loop() override {
    const bool running = (elyABPhase != 0);
    if (on->getVal<bool>() != running) on->setVal(running);
  }
};

struct HK_ElyK : Service::Switch {
  SpanCharacteristic *on = new Characteristic::On(elyKcommand);
  boolean update() override { elyKcommand = on->getNewVal(); return true; }
  void    loop()   override { if (on->getVal<bool>() != elyKcommand) on->setVal(elyKcommand); }
};

// ---- Read-only sensor characteristics (no subclass needed) --------------
static SpanCharacteristic *hkTemp    = nullptr;   // CurrentTemperature (pool)
static SpanCharacteristic *hkTempAir = nullptr;   // CurrentTemperature (air)

// Flow exposed as a Lightbulb so the Home app shows a clean ON/OFF tile:
// ON  = water flowing, OFF = no flow. Writes from the Home app are reverted
// in update() because the state is dictated by the physical flow switch.
struct HK_FlowLight : Service::LightBulb {
  SpanCharacteristic *on = new Characteristic::On(flowing);
  boolean update() override { on->setVal(flowing); return true; }
  void    loop()   override { if (on->getVal<bool>() != flowing) on->setVal(flowing); }
};

// =========================================================================
// Public API
// =========================================================================
static bool s_started = false;

void homekitSetup() {
  if (s_started) return;
  s_started = true;

  // Silent log; HAP on port 1201 so it doesn't clash with AsyncWebServer
  // on 80; hand WiFi creds so HomeSpan doesn't start its AP config portal.
  homeSpan.setLogLevel(0);
  homeSpan.setPortNum(1201);
  homeSpan.setPairingCode(HK_SETUP_CODE);
  homeSpan.setWifiCredentials(WiFi.SSID().c_str(), WiFi.psk().c_str());

  // HomeSpan v1.9.1 unconditionally prints a ~100-line boot banner +
  // accessory tree dump from inside begin() and SpanAccessory ctors;
  // setLogLevel(0) only silences runtime messages. Temporarily stopping
  // the UART driver is the simplest way to swallow those prints without
  // patching HomeSpan. Safe here because setup() is single-threaded at
  // this point (main loop + AsyncTCP + HomeSpan tasks aren't running yet).
  Serial.flush();
  Serial.end();

  homeSpan.begin(Category::Bridges, "Pool Controller");

  new SpanAccessory(); info("Pool Controller",  "PC09-BRIDGE", "PoolController v9");
  new SpanAccessory(); info("Pumpe",            "PC09-PUMP",   "Relay");     new HK_Pump();
  new SpanAccessory(); info("Elektrolyse AB",   "PC09-ELYAB",  "Alternator");new HK_ElyAB();
  new SpanAccessory(); info("Elektrolyse K",    "PC09-ELYK",   "Relay");     new HK_ElyK();
  new SpanAccessory(); info("Pool-Temperatur",  "PC09-TEMP",   "DS18B20");
                       new Service::TemperatureSensor();
                       hkTemp = new Characteristic::CurrentTemperature(20.0f);
  new SpanAccessory(); info("Flow",             "PC09-FLOW",   "Reed");     new HK_FlowLight();
  new SpanAccessory(); info("Luft-Temperatur",  "PC09-AIRT",   "DS18B20");
                       new Service::TemperatureSensor();
                       hkTempAir = new Characteristic::CurrentTemperature(20.0f);

  // Restore Serial with the same TX-buffer size as in setup().
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);

  // HomeSpan overwrites the mDNS hostname; restore ours.
  mdns_hostname_set(DEVICE_HOSTNAME);
  Serial.printf("[HK] ready, code %s\n", HK_SETUP_CODE);
}

void homekitLoop() {
  if (!s_started) return;
  homeSpan.poll();

  // Push sensor updates (0.1 C debounce on temp).
  if (hkTemp && temp_pool > -50.0f && temp_pool < 99.0f) {
    if (fabsf(hkTemp->getVal<float>() - temp_pool) >= 0.1f) hkTemp->setVal(temp_pool);
  }
  if (hkTempAir && temp_air > -50.0f && temp_air < 99.0f) {
    if (fabsf(hkTempAir->getVal<float>() - temp_air) >= 0.1f) hkTempAir->setVal(temp_air);
  }
}

void homekitResetPairings() {
  Serial.println("[HK] reset + reboot");
  nvs_handle_t h;
  // HAPSRP holds pairings in HomeSpan >= 1.7; SRP was the legacy name.
  for (const char *ns : { "HAPSRP", "SRP" }) {
    if (nvs_open(ns, NVS_READWRITE, &h) == ESP_OK) {
      nvs_erase_all(h); nvs_commit(h); nvs_close(h);
    }
  }
  delay(50);
  ESP.restart();
}

bool homekitIsPaired() {
  if (!s_started) return false;
  return homeSpan.controllerListBegin() != homeSpan.controllerListEnd();
}

const char *homekitGetSetupCode() { return HK_SETUP_CODE; }
