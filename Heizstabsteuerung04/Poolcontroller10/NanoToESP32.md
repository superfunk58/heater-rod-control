# Pin-Migration  ·  Arduino Nano (`Poolcontroller08`) → ESP32 DevKit V1 (`Poolcontroller09`)

Quellen:
- Nano: `../Poolcontroller08-last for Nano/src/main.cpp`
- ESP32: `src/main.cpp`

> **Logik-Pegel:** Nano = 5 V, ESP32 = 3.3 V. Alle Relais-Eingänge müssen
> 3.3 V-kompatibel sein (die meisten „low-level-triggered" 5 V-Relaismodule
> funktionieren, „active HIGH" 5 V-Module eventuell nicht zuverlässig → ggf.
> Pegelwandler oder Open-Drain-Stufe).

---

## 1. Aktoren (Relais-Ausgänge, active HIGH)

| Funktion           | Nano-Pin | ESP32-GPIO | Hinweis                                  |
| ------------------ | -------- | ---------- | ---------------------------------------- |
| Pumpe              | **D5**   | **GPIO23** | Unterste Reihe, freier General-Purpose   |
| Elektrolyse A      | **D8**   | **GPIO22** | zusammen mit Ely-B im Alternator-Paar    |
| Elektrolyse B      | **D7**   | **GPIO21** | zusammen mit Ely-A im Alternator-Paar    |
| Elektrolyse K      | **D6**   | **GPIO19** | vom Zeitplan **nicht** betroffen         |
| Transformator      | **D3**   | **GPIO18** | wird von Ely-A/B angesteuert             |

## 2. Sensoren / Eingänge

| Funktion              | Nano-Pin | ESP32-GPIO | Hinweis                                           |
| --------------------- | -------- | ---------- | ------------------------------------------------- |
| DS18B20 (1-Wire)      | **D2**   | **GPIO17** | 4,7 kΩ Pull-up auf 3V3 (nicht mehr 5 V!)          |
| Flow-Switch           | **D4**   | **GPIO16** | `INPUT_PULLUP`, aktiv-LOW, 200 ms Entprellung     |
| Kalibrier-Taster      | –        | **GPIO4**  | **neu**, `INPUT_PULLUP`, aktiv-LOW                |

## 3. ORP-Sensor (DFRobot ORP-Pro) – analog über ADS1115

| Funktion              | Nano                               | ESP32                                        |
| --------------------- | ---------------------------------- | -------------------------------------------- |
| ORP-Spannung          | `ads.readADC_SingleEnded(3)` (A3)  | `ads.readADC_SingleEnded(3)` (A3)            |
| ADS1115 SDA           | **A4** (HW-I²C)                    | **GPIO25** (Software-I²C, `Wire.begin`)      |
| ADS1115 SCL           | **A5** (HW-I²C)                    | **GPIO26** (Software-I²C, `Wire.begin`)      |

> Die Standard-I²C-Pins des ESP32 (GPIO21/22) werden für die Relais benötigt.
> Daher sitzen SDA/SCL auf der oberen Pin-Reihe (25/26).

## 4. Display (ST7920 128×64 über SPI)

| Funktion    | Nano                                   | ESP32                             |
| ----------- | -------------------------------------- | --------------------------------- |
| Library     | `U8GLIB_ST7920_128X64_1X` (Software-SPI) | `U8G2_ST7920_128X64_F_HW_SPI` (VSPI) |
| CS          | **D9**                                  | **GPIO27**                        |
| MOSI / RW   | **D11**                                 | **GPIO13**                        |
| SCK  / E    | **D12**                                 | **GPIO14**                        |
| MISO        | –                                       | nicht benötigt (`-1` im Aufruf)   |
| SS-Pin-Trick| `pinMode(10,OUTPUT); digitalWrite(10,HIGH);` nötig | entfällt — ESP32 hat kein SPI-Master-SS-Problem |

## 5. Status / Sonstiges

| Funktion         | Nano                   | ESP32                            |
| ---------------- | ---------------------- | -------------------------------- |
| Onboard-Status-LED | D13 (nicht genutzt) | **GPIO2** (blinkt bei WLAN OK)   |
| USB-UART (Log)   | D0/D1 (Hardware-UART) | GPIO1/GPIO3 (UART0, belegt durch CP2102) |
| Reset-Taster     | RESET                  | EN                               |

## 6. Auf dem Nano benutzt, auf dem ESP32 entfällt

| Nano-Pin | Grund                                                                     |
| -------- | ------------------------------------------------------------------------- |
| **D0/D1**| Seriell-Debug läuft auf ESP32 weiterhin über UART0 (GPIO1/3).             |
| **D10**  | Nano brauchte SS=HIGH-Workaround für HW-SPI; auf ESP32 nicht nötig.       |
| **D13**  | War auf Nano ungenutzt (HW-SPI wurde nicht gebraucht).                    |
| **A0-A3**| Wurden auf Nano als Output=LOW geparkt; ADS1115 macht den Analog-Job.     |
| **A6/A7**| Analog-only, nie verwendet.                                               |

## 7. Neu auf dem ESP32 (hat der Nano nicht)

- **WLAN + WebServer (AsyncWebServer)** – Port 80
- **MQTT (PubSubClient)** – Port 1883, Publish-Topic konfigurierbar
- **ArduinoOTA** – Port 3232, mDNS `poolcontroller.local`
- **NTP** – Zeitzone CET/CEST, für den Wochen-Zeitplan
- **Wochenplan Ely A/B** – 7 × 24 Bit, stündliche Freigabe
- **Ely A/B Alternator** – automatischer Wechsel nach `elyABSwitchSecs`
- **LittleFS** – Web-UI-Assets, Pinout-SVG-Export
- **Kalibrier-Taster (GPIO4)** – für ORP-Nullpunkt

## 8. Übersichtstabelle „Ein Blick"

```
     Nano   ESP32     Funktion                  Richtung / Typ
     -------------------------------------------------------------
     D2  -> GPIO17   DS18B20 (1-Wire)           INPUT + PU 4k7
     D3  -> GPIO18   Transformator-Relais       OUT  (active HIGH)
     D4  -> GPIO16   Flow-Schalter              IN   (PULLUP, active LOW)
     D5  -> GPIO23   Pumpen-Relais              OUT  (active HIGH)
     D6  -> GPIO19   Ely-K-Relais               OUT  (active HIGH)
     D7  -> GPIO21   Ely-B-Relais               OUT  (active HIGH)
     D8  -> GPIO22   Ely-A-Relais               OUT  (active HIGH)
     D9  -> GPIO27   LCD-CS                     OUT
     D11 -> GPIO13   LCD-MOSI  (VSPI)           OUT
     D12 -> GPIO14   LCD-SCK   (VSPI)           OUT
     A4  -> GPIO25   ADS1115-SDA                I²C
     A5  -> GPIO26   ADS1115-SCL                I²C
     ---    GPIO4    Kalibrier-Taster (NEU)     IN   (PULLUP)
     ---    GPIO2    Status-LED       (NEU)     OUT
     ---    GPIO1/3  UART0 Debug      (NEU USB) belegt durch CP2102
```

---

### Rewire-Checkliste (physisch umstecken)

1. **3V3-Pegel beachten!** Relaismodul auf Low-Level-Trigger-Typ prüfen
   oder Pegelwandler (z. B. MOSFET-Level-Shifter) einbauen.
2. **4,7 kΩ DS18B20-Pull-up** auf das neue 3V3 (statt 5 V) umhängen.
3. **Flow-Switch**: GND-Seite bleibt, Signal-Ader auf GPIO16.
4. **LCD ST7920**: drei Adern (CS, MOSI, SCK) umstecken. VCC wahlweise auf
   VIN (5 V vom USB) lassen — der ST7920 braucht 5 V, die Logikleitungen
   vertragen 3,3 V-Pegel.
5. **ADS1115**: VDD an 3V3, SDA→GPIO25, SCL→GPIO26, ADDR an GND.
6. **Kalibrier-Taster** (neu) zwischen GPIO4 und GND löten, interner
   Pull-up aktiviert.
7. **GPIO2** ist zugleich Strapping-Pin: beim Boot **nicht** extern
   HIGH ziehen, sonst Boot-Modus verändert.
