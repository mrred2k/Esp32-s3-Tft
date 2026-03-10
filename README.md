# TENSTAR TS-ESP32-S3 TFT Firmware

Live sensor dashboard for the TENSTAR TS-ESP32-S3 board — pressure history graph, IMU graphs, BLE advertising, motion-aware auto-dim backlight, and NeoPixel rainbow.

---

## Hardware

| Item | Details |
|---|---|
| Board | TENSTAR TS-ESP32-S3 (Adafruit Feather ESP32-S3 TFT clone) |
| SoC | ESP32-S3FH4R2 — 240 MHz, 4 MB Flash, **0 KB PSRAM** (none fitted on this PCB revision; SoC part number implies PSRAM but it is absent — confirmed at runtime) |
| Display | ST7789 1.14" 135×240 SPI, rotation 3 (landscape) |
| IMU | QMI8658 6-axis (accel + gyro) at I2C 0x6B |
| Barometer | BMP280 at I2C 0x77 |
| NeoPixel | 1× WS2812 on GPIO 33, power enable GPIO 21 (internal load switch — not a header pin) |
| BOOT btn | GPIO 0, active LOW |

## Pin Map

```
Display : MOSI=35  SCLK=36  CS=7  DC=39  RST=40  BL=45
I2C     : SDA=42   SCL=41
NeoPixel: DATA=33  PWR_EN=21
```

### Board layout (USB connector at top, viewed from display side)

```
                       ┌──[ USB ]──┐
               RST    ─┤           ├─ 
               3V3    ─┤           ├─  
               3V3    ─┤  ┌─────┐  ├─
               GND    ─┤  │ TFT │  ├─  BAT
  A0  free  GPIO18    ─┤  │     │  ├─  EN
  A1  free  GPIO17    ─┤  └─────┘  ├─  USB (VBUS 5V)
  A2  free  GPIO16    ─┤           ├─   13  GPIO13  LED_BUILTIN (no LED on clone)
  A3  free  GPIO15    ─┤           ├─   12  GPIO12  free
  A4  free  GPIO14    ─┤           ├─   11  GPIO11  free
  A5  free   GPIO8    ─┤           ├─   10  GPIO10  free
 SCK  TFT   GPIO36    ─┤           ├─    9   GPIO9   free
  MO  TFT   GPIO35    ─┤           ├─    6   GPIO6   free
  MI  free  GPIO37    ─┤           ├─    5   GPIO5   free
  RX  free   GPIO1    ─┤           ├─  SCL  GPIO41   → I2C clock
  TX  free   GPIO2    ─┤           ├─  SDA  GPIO42   → I2C data
 DBG  avoid GPIO43    ─┤           ├─  (UART0 TX)
                       └───────────┘
```

**Free GPIOs**: `1, 2, 5, 6, 8, 9, 10, 11, 12, 14, 15, 16, 17, 18, 37`  
**LED_BUILTIN (GPIO 13)**: A physical LED is present on this clone but has a high series resistor — visibly dim. Confirmed via firmware blink test.  
**Internal pins (not on headers)**:
- GPIO 21 — the Adafruit variant names this `TFT_I2C_POWER` (LDO enable). On the TENSTAR clone it is wired directly to the **TFT RST line** (active LOW). Pulling it LOW immediately resets the ST7789 controller (white flash + redraw); it does not cut power to the display or the I2C bus. The firmware drives it HIGH at startup and must not pull it LOW during normal operation.
- GPIO 34 — the Adafruit variant names this `NEOPIXEL_POWER`. On the TENSTAR clone it is **unconnected**. The NeoPixel runs on permanent 3.3 V with no software power switch.

**Adafruit-only hardware (absent on clone)**: STEMMA QT connector (JST SH 4-pin I2C breakout), onboard battery fuel gauge IC (LC709203 or MAX17048 at I2C 0x0B/0x36).  
**Avoid**: GPIO 19/20 (USB D−/D+), GPIO 43/44 (UART0 TX/RX)

---

## Features

### Pressure graph (page 0)
BMP280 polled at 50 Hz, 237-sample ring buffer (≈ 4.7 s history).
Auto-scaled per frame; minimum visible range 0.05 hPa so a stable sensor shows a flat line instead of noise.

### IMU graphs (pages 1 / 2)
Raw I2C driver for QMI8658 — no external library.
All three channels overlay on a shared 54 px plot with a single auto-scale.
- Page 1 — gyro: Gx (red), Gy (green), Gz (cyan) in dps
- Page 2 — accel: Ax (orange), Ay (yellow), Az (magenta) in g

### Sprite-buffered rendering
The graph zone is rendered into a `TFT_eSprite` allocated from **heap** (~30 KB), then pushed as a single SPI burst.
This eliminates the clear→redraw flash that appears when writing to the display directly at 50 Hz.
(PSRAM is absent on this board; TFT_eSPI falls back to heap automatically.)

### BLE Environmental Sensing Service *(BLE builds only — omitted when `-DNO_BLE`)*
Advertises as `ESP32S3-XXXX` (last 4 MAC hex digits).
Notifies Temperature (0x2A6E, 0.01 °C) and Pressure (0x2A6D, 0.1 Pa) at 50 Hz.

### Motion-based auto-dim
Backlight fades from 255 → 8 over 1 s after 2 s of stillness.
Wakes instantly on any motion. No sleep mode — the CPU stays running.

Motion uses accel magnitude deviation from a local EMA baseline, **not** a hardcoded 1.0 g:
```
moving = |aMag - ema| > 0.05 g
```
The EMA (α = 0.02, ~1 s time-constant) only advances while still, so it tracks whatever the sensor's resting bias actually is. **This matters:** the QMI8658 on this board reads ~1.09 g at rest, not 1.0 g — comparing against a fixed 1.0 g constant keeps `moving = true` permanently and the display never dims.

Gyro is intentionally excluded from dim logic — gyro zero-rate noise on this chip is ~15–30 dps, which is unpredictable and would prevent dimming regardless of motion threshold.

### Button
| Press | Action |
|---|---|
| Short press | Cycle graph page: hPa → Gyro → Accel |
| Long press (> 800 ms) | Toggle NeoPixel on / off |

---

## Software

Built with PlatformIO + Arduino framework.

```ini
platform  = espressif32@6.4.0
board     = adafruit_feather_esp32s3_tft
```

### Libraries
- `bodmer/TFT_eSPI` — display + sprite
- `adafruit/Adafruit NeoPixel`
- `adafruit/Adafruit BMP280 Library`
- `ESP32 BLE Arduino`

### QMI8658 raw driver
No library. ~60 lines in `namespace QMI` inside `main.cpp`.
Registers written at init:

| Register | Value | Meaning |
|---|---|---|
| 0x0A CTRL9 | 0xFF | Soft-reset (runs first, 15 ms wait) |
| 0x02 CTRL1 | 0x40 | I2C, address auto-increment |
| 0x03 CTRL2 | 0x23 | Accel ±8 g, ODR 119 Hz |
| 0x04 CTRL3 | 0x22 | Gyro ±512 dps, ODR 119 Hz |
| 0x08 CTRL7 | 0x03 | Enable accel + gyro |

The soft-reset is essential: the QMI8658 has its own power rail and does **not** reset when the ESP32 resets. If a previous firmware left CTRL9 in a mid-command state, the sensor outputs zeros until power-cycled — unless a soft-reset is issued at boot.

12-byte burst read from 0x35 (accel XYZ then gyro XYZ, all little-endian int16).
Scale: accel `/ 4096` → g, gyro `/ 64` → dps.

---

## Flash / RAM

| Build | Flash | RAM (static) |
|---|---|---|
| With BLE | ~98% (1.41 MB) | ~27% |
| NO_BLE (`-DNO_BLE`) | ~58% (835 KB) | ~21% |

Available: 1.44 MB Flash, 374 KB RAM total (no PSRAM).
Large consumers: BLE stack (~580 KB flash when enabled), TFT_eSPI (~120 KB flash),
graph sprite 237×65×2 bytes = ~30 KB heap, IMU ring buffer 6×237×4 bytes = ~5.5 KB heap.

---

## Known Quirks / Gotchas


**QMI8658 at I2C 0x7E (SMBus ARA) — INT1 not on any GPIO**
The I2C scan shows a device at 0x7E. This is not a separate chip — it is the SMBus Alert Response Address (ARA).
The QMI8658's INT1 pin is connected to the I2C SMBA (alert) line, **not** to any ESP32-S3 GPIO.
Hardware-verified with a GPIO scan during live Any-Motion interrupt: zero changes on all 22 scanned free GPIOs.
`gpio_wakeup_enable` cannot be used on this board. Light sleep is not used.

**QMI8658 outputs zeros after ESP32 reset**
The QMI8658 has its own power rail and retains its register state across ESP32 resets.
A previous Any-Motion Detection init left CTRL9 = 0x08 (command never acknowledged), freezing the sensor output at zero.
Fixed by issuing `CTRL9 = 0xFF` (cmdSoftReset) at the start of every `QMI::init()` call.

**Accel magnitude vs. gyro for motion detection**
See *Motion-based auto-dim* above. Short version: use accel, not gyro.

**BMP280 IIR filter as noise suppressor**
`SAMPLING_X1` (single pressure sample, ~2.3 ms) combined with `FILTER_X16` gives the same noise floor as hardware oversampling X16, at full ODR.
The IIR accumulates on the sensor and requires no averaging in firmware.

---

## Upload Ports

```ini
upload_port  = COM4   ; ROM bootloader (1200 bps touch to enter)
monitor_port = COM3   ; CDC serial (normal operation)
monitor_speed = 115200
```

## VS Code Tasks

Run via **Terminal → Run Task** or `Ctrl+Shift+P → Tasks: Run Task`.

| Task | Environment | Transport | Notes |
|---|---|---|---|
| **Build (USB, with BLE)** | `tenstar_esp32s3_tft` | — | Full build including BLE stack. ~98% flash. |
| **Build (OTA, no BLE)** | `tenstar_esp32s3_tft_ota_fast` | — | Strips BLE → ~58% flash. |
| **Upload via USB (no BLE)** | `tenstar_esp32s3_tft_usb_fast` | COM4 (esptool) | **Default (`Ctrl+Shift+B`).** No BLE, ~58% flash. ~25 s. USB cable required. |
| **Upload via USB (with BLE)** | `tenstar_esp32s3_tft` | COM4 (esptool) | Full BLE build, ~98% flash. USB cable required. |
| **Upload via USB + Monitor (no BLE)** | `tenstar_esp32s3_tft_usb_fast` | COM4 → COM3 | Upload then opens serial monitor automatically. |
| **Upload via OTA (no BLE)** | `tenstar_esp32s3_tft_ota_fast` | WiFi 192.168.178.89 | No USB needed. Board must be running and on the network. |
| **Monitor (COM3)** | `tenstar_esp32s3_tft` | COM3 115200 | Serial monitor with `esp32_exception_decoder` filter. USB cable required. |
