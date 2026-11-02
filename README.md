# HICS Firmware

ESP32 firmware (PlatformIO / Arduino) for the HICS hydroponic controller. Two boards:

| Env | Source | Role |
|---|---|---|
| `esp32_main` | `src/mainBoard/main.cpp` | Sensors, pumps, auto-dosing, Firebase sync |
| `esp32_lcd` | `src/lcdBoard/lcd_main.cpp` | ILI9341 touch UI, talks to main board over UART |

Wiring schematics are in `sch/`.

## Setup

1. Install [PlatformIO](https://platformio.org/).
2. `cp src/mainBoard/secrets.h.example src/mainBoard/secrets.h` and fill in your Firebase / WiFi values. This file is git-ignored.
3. Build / upload:
   ```
   pio run -e esp32_main -t upload
   pio run -e esp32_lcd -t upload
   ```
