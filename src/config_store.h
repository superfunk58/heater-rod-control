#pragma once
// Persistent configuration in NVS Preferences (default `nvs` partition).
// Survives reboots and firmware OTA. Erased only by `esptool erase_flash`.
//
// Loaded once at boot, written on demand from MQTT/web handlers.

namespace ConfigStore {

// Load all settings from NVS into the extern globals in main.cpp.
// If a key is missing (first boot), the global keeps its compile-time default.
void load();

// Save all settings to NVS. Call after changes from MQTT or web UI.
// Cheap (NVS uses wear-levelling), but debouncing is still nice on hot paths.
void save();

}  // namespace ConfigStore
