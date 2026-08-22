// ============================================================================
// DS18B20 Temperatursensoren (OneWire-Bus)
// ----------------------------------------------------------------------------
// Vier Sensoren auf gemeinsamem Bus:
//   - "boiler"   : Speicher (Warmwasser-Boiler)
//   - "inlet"    : Heizstab-Zulauf (vor Heizstab)
//   - "outlet"   : Heizstab-Ablauf (nach Heizstab)
//   - "hrod"     : Heizstab-Fühler (direkt am Heizstab)
//
// Sensor-Zuweisung erfolgt über NVS (Namespace "temp"); UI listet alle gefundenen
// ROMs mit Live-Temperatur und erlaubt das Mapping pro Rolle.
//
// Lese-Pattern: non-blocking. tick() ruft alle 3s requestTemperatures() und
// liest 800ms später die Werte aus, ohne den Loop zu blockieren.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <vector>

namespace TempSensors {

struct Found {
  uint64_t rom;          // ROM-ID des Sensors (0 = ungültig)
  float    current_c;    // letzter gelesener Wert (NAN wenn unbekannt)
  String   romHex;       // hex-Darstellung "28-FF-AA-BB-..."
};

// Initialisiert den OneWire-Bus auf dem angegebenen GPIO und lädt das
// Sensor-Mapping aus NVS. Muss in setup() einmal aufgerufen werden.
void begin(uint8_t pin);

// Asynchrones Lese-Pattern. Muss in loop() jeden Durchlauf gerufen werden.
void tick();

// Aktuelle gemittelte Werte. NAN wenn Sensor nicht zugewiesen oder Lesefehler.
float getBoilerC();
float getInletC();
float getOutletC();
float getHeaterRodC();

// Anzahl der aktuell gefundenen Sensoren (kein Heap-Allokation).
uint8_t sensorCount();

// Liste aller aktuell gefundenen Sensoren mit ihren Live-Werten (für UI-Scan).
std::vector<Found> scanList();

// Manuelles Rescan des OneWire-Bus (für Button-Scan).
// ACHTUNG: blockierende OneWire-I/O + mutiert die Sensor-Arrays. Darf NUR aus
// dem Loop-Task (tick-Kontext) aufgerufen werden, nie aus einem HTTP-Handler.
void rescan();

// Thread-sicher: fordert ein Rescan an, das beim nächsten tick() im Loop-Task
// ausgeführt wird. Aus HTTP-Handlern (anderer Task) statt rescan() benutzen.
void requestRescan();

// true solange ein angefordertes Rescan noch nicht vom Loop-Task erledigt wurde.
bool rescanPending();

// Aktuelle Zuweisung lesen (0 = unset).
uint64_t boilerRom();
uint64_t inletRom();
uint64_t outletRom();
uint64_t heaterRodRom();

// Mapping schreiben (NVS persistiert). 0 = Rolle leeren.
void assignBoiler(uint64_t rom);
void assignInlet(uint64_t rom);
void assignOutlet(uint64_t rom);
void assignHeaterRod(uint64_t rom);

// Hilfsfunktionen für ROM <-> Hex-String (z.B. "28-FF-AA-12-...")
String romToHex(uint64_t rom);
uint64_t romFromHex(const String &hex);

}  // namespace TempSensors
