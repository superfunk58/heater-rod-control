# Poolcontroller09 – ESP32-WROOM-32 DevKit (30-pin) Pin Assignment

**Board orientation:** USB connector facing **LEFT**.
The row **closer to you** (bottom) gets all actuators, sensors, the flow
switch, and the calibration button so they can all be wired from one
terminal strip under the board. The row **far from you** (top) carries the
ST7920 display (HSPI) and the ADS1115 I²C bus.

Verified layout: USB-corner bottom-row pin = **GPIO15**, far-end bottom-row
pin = **GPIO23** (DOIT / HiLetgo / AZ-Delivery 30-pin).

> A polished one-page PDF rendering of this map is in `Pinout.pdf`
> (regenerate via `scripts/pinout.html` → Chrome headless, see bottom of file).

## Bottom row (USB → far end)

| Pin    | Function                   | Direction    | Notes                                           |
|--------|----------------------------|--------------|-------------------------------------------------|
| 3V3    | 3.3 V out                  | —            | Power rail (use for I²C / 1-wire pull-ups)      |
| GND    | GND                        | —            | Common ground                                   |
| GPIO15 | *reserve*                  | —            | ⚠ strap pin (must be HIGH at boot) – leave open |
| GPIO2  | LINK LED (onboard)         | OUT          | ⚠ strap (must be LOW/float at boot) – driven only after boot |
| GPIO4  | **BUTTON** (calibration)   | INPUT_PULLUP | Button to GND, internal pull-up                 |
| GPIO16 | **FLOW** switch            | INPUT_PULLUP | Active LOW (LOW = flow)                         |
| GPIO17 | **DS18B20** 1-wire data    | IN/OUT       | External 4.7 kΩ pull-up to **3V3**              |
| GPIO5  | *reserve*                  | —            | ⚠ strap pin (must be HIGH at boot) – leave open |
| GPIO18 | **TRANSFORMER** relay      | OUT          | Active HIGH (auto, flow-guarded)                |
| GPIO19 | **ELY_K** relay            | OUT          | Active HIGH                                     |
| GPIO21 | **ELY_B** relay            | OUT          | Active HIGH                                     |
| GPIO3  | *(UART0 RX)*               | —            | ⚠ USB-serial input – do not load                |
| GPIO1  | *(UART0 TX)*               | —            | ⚠ USB-serial output – do not load               |
| GPIO22 | **ELY_A** relay            | OUT          | Active HIGH                                     |
| GPIO23 | **PUMP** relay             | OUT          | Active HIGH                                     |

## Top row (USB → far end)

| Pin    | Function                     | Notes                                          |
|--------|------------------------------|------------------------------------------------|
| VIN    | 5 V in                       | Power from USB or ext. 5 V                     |
| GND    | GND                          |                                                |
| GPIO13 | **ST7920 MOSI** (HSPI_MOSI)  | Display data                                   |
| GPIO12 | *reserve*                    | ⚠ strap (MTDI) – MUST be LOW at boot, leave open |
| GPIO14 | **ST7920 SCK** (HSPI_CLK)    |                                                |
| GPIO27 | **ST7920 CS**                | Idle HIGH                                      |
| GPIO26 | **I²C SCL** (ADS1115)        | 4.7 kΩ pull-up to 3V3                           |
| GPIO25 | **I²C SDA** (ADS1115)        | 4.7 kΩ pull-up to 3V3                           |
| GPIO33 | *reserve*                    | in/out available                               |
| GPIO32 | *reserve*                    | in/out available                               |
| GPIO35 | *reserve*                    | Input-only, no internal pull                   |
| GPIO34 | *reserve*                    | Input-only, no internal pull                   |
| GPIO39 | *reserve* (VN)               | Input-only, ADC1_CH3                           |
| GPIO36 | *reserve* (VP)               | Input-only, ADC1_CH0                           |
| EN     | Reset                        | Pulled HIGH on board                           |

## Voltage compatibility (3.3 V vs. 5 V)

ESP32 IO is **3.3 V only** and (mostly) **not 5 V tolerant** on inputs.

| Device                  | Supply | ESP32 interface                  | Verdict / action                                                |
|-------------------------|--------|----------------------------------|------------------------------------------------------------------|
| ADS1115                 | 3.3 V  | I²C (GPIO25/26)                  | ✅ Native 3.3 V. Pull-ups to 3V3. 0.1875 mV/bit at GAIN_TWOTHIRDS. |
| DS18B20                 | 3.3 V  | 1-wire on GPIO17                 | ✅ 3.3 V fine. **4.7 kΩ pull-up to 3V3** (NOT 5 V).               |
| Flow switch (reed/hall) | 3.3 V  | GPIO16 INPUT_PULLUP              | ✅ Dry contact to GND = ideal. If the sensor actively drives 5 V → level-shift (resistor divider 10k/15k, or 74HCT125). |
| Push-button (cal.)      | —      | GPIO4 to GND                     | ✅ Uses internal pull-up, no external R needed.                   |
| ST7920 128×64 LCD       | **5 V**| SPI on GPIO13/14/27 at 3.3 V     | ⚠ **V_IH_min = 0.7·VDD = 3.5 V at 5 V supply.** 3.3 V signals are marginal. Options: (a) run the LCD at **3.3 V** (many modules work, lower contrast – tune the pot), or (b) power at 5 V with a 3.3→5 V level shifter (74HCT125 / TXS0108E) on SCK, MOSI, CS. |
| 5 V relay modules       | 5 V    | GPIO line at 3.3 V               | ⚠ Common JQC-3FF opto-relay boards often need ≥4 V on IN. Options: (a) use a **3.3 V-logic** relay board, (b) add a 74HCT125 5 V buffer, (c) drive the opto LED directly from the GPIO via a smaller series resistor. Verify datasheet of your relay board. |
| Transformer coil        | 12/24 V AC | relay contact                | ✅ relay isolates the coil from the ESP32.                        |
| 5 V rail / VIN          | 5 V    | DevKit VIN pin                   | ✅ Use a ≥1 A 5 V supply if driving 5 relays + LCD.               |

### Pull-up summary

- **I²C SDA/SCL (GPIO25/26):** 4.7 kΩ each to 3V3.
- **DS18B20 DQ (GPIO17):** 4.7 kΩ to 3V3 (external).
- **FLOW (GPIO16):** internal `INPUT_PULLUP` used – external not needed for a dry contact to GND.
- **BUTTON (GPIO4):** internal `INPUT_PULLUP` used – button wired to GND.

### Strapping pins – none used as loaded outputs

`GPIO0, GPIO2, GPIO5, GPIO12, GPIO15` are left as reserves or used only for
non-load purposes. GPIO2 drives only the onboard LED after boot
completes, so the boot-time requirement (LOW / floating) is preserved.

## Reserve GPIOs still available

| Pin | Type | Row |
|-----|------|-----|
| GPIO32, GPIO33 | in/out | top |
| GPIO34, GPIO35 | input-only | top |
| GPIO36 (VP), GPIO39 (VN) | input-only | top |

## Firmware pin constants

Edit `PIN_*` constants at the top of `src/main.cpp` (section
"PIN MAP - ESP32-WROOM-32 DevKit") if you need to rewire.

## Regenerating the PDF

The polished one-page PDF (`Pinout.pdf`) is produced from
`scripts/pinout.html` by Chrome in headless mode:

```sh
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  --headless=new --disable-gpu --no-pdf-header-footer \
  --print-to-pdf="Pinout.pdf" \
  "file://$(pwd)/scripts/pinout.html"
```

To change a pin label, edit the `#top-data` / `#bot-data` JSON blocks in
`scripts/pinout.html` and rerun the command.
