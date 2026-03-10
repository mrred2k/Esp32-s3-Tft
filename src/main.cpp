#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "secrets.h"
#ifndef NO_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif

#include <driver/gpio.h>
#include <TFT_eSPI.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BMP280.h>

// ════════════════════════════════════════════════════════════════════════════
// TENSTAR TS-ESP32-S3 TFT  –  Hardware notes
// ════════════════════════════════════════════════════════════════════════════
//
// Board     : TENSTAR TS-ESP32-S3  (Adafruit Feather ESP32-S3 TFT clone)
// Chip      : ESP32-S3FH4R2  –  240 MHz, 4 MB Flash, 2 MB PSRAM
// Display   : ST7789 1.14"  135×240  SPI  (rotation 3 = landscape, USB right)
//   MOSI=35  SCLK=36  CS=7  DC=39  RST=40  BL=45 (LEDC PWM)
//
// I2C bus   : SDA=42  SCL=41
//   0x6B  QMI8658  –  6-axis IMU (accel + gyro)//             config: ±8 g / ±512 dps, ODR ~119 Hz, raw I2C (no library)//   0x77  BMP280   –  barometric pressure + temperature
//   0x7E  SMBus Alert Response Address (ARA) – NOT a separate chip.
//
//   About 0x7E / SMBus ARA:
//   The QMI8658's INT1 pin is connected to the I2C SMBA (alert) line — it is
//   NOT wired to any ESP32-S3 GPIO pin. Hardware-verified: GPIO scan while
//   QMI Any-Motion was armed showed zero changes on all free GPIOs.
//   Wake-from-sleep therefore uses a 2-second timer poll loop: the ESP32-S3
//   sleeps in 2 s bursts, reads the IMU on each wake, and fully wakes only
//   when accel magnitude deviates from baseline by > 0.05 g.
//   The SMBus ARA (0x7E) is visible on the I2C bus and responds correctly.
//
// NeoPixel  : GPIO33 (data)  GPIO21 (power enable, active HIGH)
// BOOT btn  : GPIO0  active LOW
//   Short press : cycle graph page  (0=pressure  1=gyro  2=accel)
//   Long press  : toggle NeoPixel RGB on / off
//   Auto-dim    : backlight drops to 8/255 after 3 s of IMU stillness
//                 motion detected via accel magnitude deviation from 1g (>0.05g)
//                 restores to 255 on first detected movement
// Upload    : COM3=CDC normal, COM4=ROM bootloader (1200bps touch)
//
// ════════════════════════════════════════════════════════════════════════════

// ── Pins ──────────────────────────────────────────────────────────────────────
constexpr uint8_t  I2C_SDA      = 42;
constexpr uint8_t  I2C_SCL      = 41;
constexpr uint8_t  BOOT_BTN_PIN   = 0;     // active LOW
constexpr uint8_t  TFT_BL_GPIO    = 45;
constexpr uint8_t  NEO_PIN        = 33;
constexpr uint8_t  NEO_PWR_PIN    = 21;
constexpr uint8_t  NEO_BRIGHTNESS = 20;    // 0-255; raise when not testing

// ── Timing ────────────────────────────────────────────────────────────────────
constexpr uint32_t HEARTBEAT_MS   = 1000;
constexpr uint32_t DEBOUNCE_MS    = 50;
constexpr uint32_t LONGPRESS_MS   = 800;

// ── Backlight PWM (LEDC) ──────────────────────────────────────────────────────
constexpr uint8_t  BL_LEDC_CH    = 0;
constexpr uint32_t BL_LEDC_FREQ  = 1000;
constexpr uint8_t  BL_LEDC_RES   = 8;       // 0-255
constexpr uint8_t  BL_ACTIVE      = 255;    // full brightness
constexpr uint8_t  BL_DIM         = 8;      // auto-dim level
constexpr uint32_t DIM_TIMEOUT_MS  = 2000;   // ms of stillness before dim starts
constexpr uint32_t BL_FADE_MS      = 1000;   // fade-to-dim duration
bool     blDimmed     = false;
bool     blFading     = false;              // fade in progress
uint32_t blFadeStart  = 0;                 // millis() when fade began
uint32_t lastMotionMs = 0;

// ── Graph pages ───────────────────────────────────────────────────────────────
// 0 = BMP280 pressure   1 = gyro (Gx/Gy/Gz)   2 = accel (Ax/Ay/Az)
uint8_t  graphPage = 0;

// ── IMU ring buffers (6 channels × GRAPH_SAMPLES) ────────────────────────────
// [0]=Ax [1]=Ay [2]=Az  (g)    [3]=Gx [4]=Gy [5]=Gz  (dps)
// Appended at 50 Hz alongside the pressure buffer
float    imuHist[6][237];     // NOTE: must be ≥ GRAPH_SAMPLES (defined later)
uint16_t imuHistCnt = 0;

// ── Display layout (rotation=3 → 240×135) ────────────────────────────────────
//  y=  1-65  pressure graph  (237 samples × 20 ms ≈ 4.7 s history, 50 Hz redraw)
//  y= 68     horizontal divider
//  y= 69-133 info panel — 2 columns, x=120 vertical divider, 5 rows each
//  ┌─────────────────────┬─────────────────────┐
//  │ LEFT  (x=2..119)    │ RIGHT (x=121..237)  │
//  ├─────────────────────┼─────────────────────┤
//  │ y=70  RGB swatch    │ y=70  BL  PWM       │
//  │ y=79  Loop/s        │ y=79  MAC           │
//  │ y=88  Heap %        │ y=88  BLE name      │
//  │ y=97  Temp °C       │ y=97  WiFi IP       │
//  │ y=106 Acc XYZ  (g)  │ y=106 Gyr XYZ (dps)│
//  └─────────────────────┴─────────────────────┘
//  border at 0/134

TFT_eSPI          tft;
TFT_eSprite       graphSpr(&tft);   // off-screen buffer for graph zone — no flash
bool              graphSprOk = false;
Adafruit_NeoPixel neo(1, NEO_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_BMP280   bmp(&Wire);

bool     bmpOk      = false;
bool     neoEnabled = true;
uint16_t neoHue     = 0;
uint32_t heartbeatCounter = 0;
uint32_t lastHeartbeatMs  = 0;
uint32_t loopCount        = 0;
uint32_t lastFps          = 0;
char     macBuf[14];   // "3C0F02DFE430\0"
char     bleBuf[16];   // "ESP32S3-E430\0"

// ── QMI8658 IMU ───────────────────────────────────────────────────────────────
// Raw I2C driver — no library. Chip at 0x6B (WHO_AM_I = 0x05).
// Accel ±8 g  → scale 1/4096 g/LSB
// Gyro  ±512 dps → scale 1/64   dps/LSB
// Both at ODR ~119 Hz; we read every loop (~92 Hz).
namespace QMI {
  constexpr uint8_t ADDR = 0x6B;

  void wreg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
  }

  bool init() {
    // Check WHO_AM_I
    Wire.beginTransmission(ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(ADDR, (uint8_t)1);
    if (!Wire.available() || Wire.read() != 0x05) return false;

    // Soft-reset the sensor before configuring.
    // The QMI8658 does NOT reset when the ESP32 resets (it's a separate chip on the
    // I2C bus with its own power rail). If the previous firmware left CTRL9 in a
    // mid-command state (e.g. 0x08 = cmdConfigureAhmd never acknowledged), the sensor
    // keeps outputting zeros until power is cycled — unless we reset it here.
    wreg(0x0A, 0xFF);  // CTRL9: cmdSoftReset
    delay(15);         // datasheet: ≥10 ms for reset to complete

    wreg(0x02, 0x40);  // CTRL1: I2C, address auto-increment
    wreg(0x03, 0x23);  // CTRL2: accel ±8 g, ODR 119 Hz
    wreg(0x04, 0x22);  // CTRL3: gyro ±512 dps, ODR 119 Hz
    wreg(0x08, 0x03);  // CTRL7: enable accel + gyro
    // Note: INT1 is on SMBA (I2C alert line, 0x7E), not connected to any ESP32-S3 GPIO.
    // Any-Motion Detection (CTRL9 cmd 0x08 / CTRL8 0x02) removed.
    return true;
  }

  // Burst-read accel + gyro (12 bytes from 0x35)
  bool read(float& ax, float& ay, float& az,
            float& gx, float& gy, float& gz) {
    Wire.beginTransmission(ADDR);
    Wire.write(0x35);  // first accel register
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(ADDR, (uint8_t)12);
    if (Wire.available() < 12) return false;
    int16_t raw[6];
    for (int i = 0; i < 6; i++) {
      uint8_t lo = Wire.read(), hi = Wire.read();
      raw[i] = (int16_t)((hi << 8) | lo);
    }
    ax = raw[0] / 4096.0f;  // g
    ay = raw[1] / 4096.0f;
    az = raw[2] / 4096.0f;
    gx = raw[3] / 64.0f;    // dps
    gy = raw[4] / 64.0f;
    gz = raw[5] / 64.0f;
    return true;
  }
} // namespace QMI

bool  imuOk      = false;
float imuAx = 0, imuAy = 0, imuAz = 0;   // g
float imuGx = 0, imuGy = 0, imuGz = 0;   // dps

// ── Pressure graph ────────────────────────────────────────────────────────────
// 237 samples × 20 ms/sample ≈ 4.7 s of history, one sample per pixel column
// IIR X16 replaces oversampling — sensor ODR ~350 Hz, we read at 50 Hz
constexpr uint16_t GRAPH_SAMPLES   = 237;
constexpr uint32_t PRESS_UPDATE_MS = 20;    // 50 Hz
float    pressHistory[GRAPH_SAMPLES];       // oldest→newest, hPa
uint16_t pressCnt          = 0;             // valid samples in buffer
float    lastTemp          = NAN;
uint32_t lastPressUpdateMs = 0;

#ifndef NO_BLE
// ── BLE globals ───────────────────────────────────────────────────────────────
// Environmental Sensing Service  0x181A
//   Temperature  0x2A6E  : int16_t,  unit = 0.01 °C   (e.g. 2350 = 23.50 °C)
//   Pressure     0x2A6D  : uint32_t, unit = 0.1  Pa   (e.g. 1013250 = 101325.0 Pa)
BLECharacteristic* bleTempChar  = nullptr;
BLECharacteristic* blePressChar = nullptr;
bool               bleConnected = false;

class BLEConnCB : public BLEServerCallbacks {
  void onConnect(BLEServer*)     override { bleConnected = true;  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    s->startAdvertising();   // restart adv so a new client can connect
  }
};
#endif  // NO_BLE

// ── Helpers ───────────────────────────────────────────────────────────────────
void setBacklight(uint8_t brightness) {
  ledcWrite(BL_LEDC_CH, brightness);
}

// ── TFT static frame + info-panel labels ─────────────────────────────────────
void tft_init() {
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_DARKGREY);
  tft.drawFastHLine(2, 68, tft.width() - 4, TFT_DARKGREY);

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  // Left column
  tft.setCursor(2, 70); tft.print("RGB:");
  tft.setCursor(2, 79); tft.print("Loop/s:");
  tft.setCursor(2, 88); tft.print("Heap:");
  tft.setCursor(2, 97); tft.print("Temp:");
  // Right column
  tft.setCursor(122, 70); tft.print("BL:");
  tft.setCursor(122, 79); tft.print("MAC:");
#ifndef NO_BLE
  tft.setCursor(122, 88); tft.print("BLE:");
#endif
  tft.setCursor(122, 97); tft.print("WiFi:");
  // Row 5 — IMU
  tft.setCursor(2,   106); tft.print("Acc:");
  tft.setCursor(122, 106); tft.print("Gyr:");
  // Vertical divider between columns
  tft.drawFastVLine(120, 69, 65, TFT_DARKGREY);
}

// ── Pressure graph zone (y=1..65, 65 px tall) ───────────────────────────────
// Rendered into a TFT_eSprite (RAM), then pushed in one SPI burst.
// This eliminates the clear→redraw flash and speeds up from ~18 ms to ~6 ms.
//
// Sprite coords (origin = screen pixel 1,1):
//  y= 0.. 7  label:  current hPa
//  y= 9..10  range annotation
//  y=10..64  line chart (auto-scaled, newest sample at right edge)
void draw_pressure_graph() {
  constexpr uint8_t GY_TOP = 10;            // sprite y: graph body top
  constexpr uint8_t GY_BOT = 64;            // sprite y: graph body bottom (=screen 65)
  constexpr uint8_t GH     = GY_BOT - GY_TOP;  // 54 px

  // ── choose render target (sprite if available, else direct) ───────────────
  if (!graphSprOk) {
    // Fallback: direct draw (slow + flash, but correct)
    tft.fillRect(1, 1, GRAPH_SAMPLES, 65, TFT_BLACK);
    tft.setTextSize(1);
    if (!bmpOk || pressCnt == 0) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setCursor(4, GY_TOP + GH / 2 - 3);
      tft.print(bmpOk ? "no data yet" : "BMP280 err");
    }
    return;
  }

  graphSpr.fillSprite(TFT_BLACK);
  graphSpr.setTextSize(1);

  if (!bmpOk || pressCnt == 0) {
    graphSpr.setTextColor(TFT_RED, TFT_BLACK);
    graphSpr.setCursor(4, GY_TOP + GH / 2 - 4);
    graphSpr.print(bmpOk ? "no data yet" : "BMP280 err");
    graphSpr.pushSprite(1, 1);
    return;
  }

  float curHpa = pressHistory[pressCnt - 1];

  // Current value label (sprite y=0 = screen y=1)
  graphSpr.setTextColor(TFT_CYAN, TFT_BLACK);
  graphSpr.setCursor(4, 0);
  graphSpr.printf("%.2f hPa", curHpa);

  if (pressCnt < 2) {
    graphSpr.pushSprite(1, 1);
    return;
  }

  // Auto-scale: find min/max in buffer
  float minP = pressHistory[0], maxP = pressHistory[0];
  for (uint16_t i = 1; i < pressCnt; i++) {
    if (pressHistory[i] < minP) minP = pressHistory[i];
    if (pressHistory[i] > maxP) maxP = pressHistory[i];
  }
  float range = maxP - minP;
  if (range < 0.05f) range = 0.05f;   // keep visible even when dead still

  // Range annotation — top-right
  graphSpr.setTextColor(TFT_DARKGREY, TFT_BLACK);
  graphSpr.setCursor(152, 0);
  graphSpr.printf("r:%.3f", range);

  // Baseline
  graphSpr.drawFastHLine(0, GY_BOT, GRAPH_SAMPLES, TFT_DARKGREY);

  // Line chart: oldest sample left, newest right
  uint16_t xOff = GRAPH_SAMPLES - pressCnt;   // left-pad if buffer not full
  int16_t px = -1, py = -1;
  for (uint16_t i = 0; i < pressCnt; i++) {
    int16_t x = (int16_t)(xOff + i);           // sprite x (0-based)
    int16_t y = GY_BOT - (int16_t)((pressHistory[i] - minP) / range * GH + 0.5f);
    if (y < GY_TOP) y = GY_TOP;
    if (y > GY_BOT) y = GY_BOT;
    if (px >= 0)
      graphSpr.drawLine(px, py, x, y, TFT_GREEN);
    else
      graphSpr.drawPixel(x, y, TFT_GREEN);
    px = x; py = y;
  }

  graphSpr.pushSprite(1, 1);  // one atomic SPI burst — no visible clear+redraw
}

// ── 3-channel stacked IMU graph (pages 1=gyro, 2=accel) ─────────────────────
// Sprite 237×65.  3 sub-graphs, each 21 px tall, 1 px separator.
// Gyro  colors: Gx=RED   Gy=GREEN  Gz=CYAN
// Accel colors: Ax=ORANGE Ay=YELLOW Az=MAGENTA
// All 3 channels overlay on the same plot — one shared auto-scale.
void draw_imu_graphs(bool isAccel) {
  if (!graphSprOk) return;
  constexpr uint8_t GY_TOP = 10;   // same geometry as pressure graph
  constexpr uint8_t GY_BOT = 64;
  constexpr uint8_t GH     = GY_BOT - GY_TOP;   // 54 px

  static const uint16_t COLORS_G[3] = { TFT_RED,    TFT_GREEN,  TFT_CYAN    };
  static const uint16_t COLORS_A[3] = { TFT_ORANGE, TFT_YELLOW, TFT_MAGENTA };
  static const char*    LBL_G[3]    = { "Gx", "Gy", "Gz" };
  static const char*    LBL_A[3]    = { "Ax", "Ay", "Az" };

  const uint16_t* colors = isAccel ? COLORS_A : COLORS_G;
  const char**    labels = isAccel ? LBL_A    : LBL_G;
  int             base   = isAccel ? 0 : 3;

  graphSpr.fillSprite(TFT_BLACK);
  graphSpr.setTextSize(1);

  // ── Shared scale: find min/max across all 3 channels ─────────────────────
  float minV =  1e9f, maxV = -1e9f;
  for (int ch = 0; ch < 3; ch++) {
    for (uint16_t i = 0; i < imuHistCnt; i++) {
      float v = imuHist[base + ch][i];
      if (v < minV) minV = v;
      if (v > maxV) maxV = v;
    }
  }
  // Minimum visible band so a still sensor doesn't collapse to a dot
  float half = isAccel ? 0.5f : 20.0f;
  float mid  = (maxV + minV) * 0.5f;
  if ((maxV - minV) < half * 2.0f) { minV = mid - half; maxV = mid + half; }
  float range = maxV - minV;

  // ── Label row (top): "Gx +12.3  Gy +0.1  Gz -2.7" in each channel colour ─
  uint8_t lx = 1;
  for (int ch = 0; ch < 3; ch++) {
    graphSpr.setTextColor(colors[ch], TFT_BLACK);
    if (imuHistCnt > 0)
      lx += graphSpr.drawString(
              String(labels[ch]) + String(imuHist[base+ch][imuHistCnt-1], 1),
              lx, 0);
    else
      lx += graphSpr.drawString(String(labels[ch]) + "---", lx, 0);
    lx += 4;  // small gap between labels
  }

  // Range annotation top-right
  graphSpr.setTextColor(TFT_DARKGREY, TFT_BLACK);
  graphSpr.setCursor(152, 0);
  graphSpr.printf("r:%.1f", range);

  // Dim zero-line (at the value that maps to midpoint)
  int16_t zeroY = GY_BOT - (int16_t)((0.0f - minV) / range * GH + 0.5f);
  if (zeroY >= GY_TOP && zeroY <= GY_BOT)
    graphSpr.drawFastHLine(0, zeroY, GRAPH_SAMPLES, tft.color565(40, 40, 40));
  // Hard baseline
  graphSpr.drawFastHLine(0, GY_BOT, GRAPH_SAMPLES, tft.color565(25, 25, 25));

  if (imuHistCnt < 2) {
    graphSpr.pushSprite(1, 1);
    return;
  }

  // ── Draw all 3 lines on the same plot ────────────────────────────────────
  uint16_t xOff = GRAPH_SAMPLES - imuHistCnt;
  for (int ch = 0; ch < 3; ch++) {
    int16_t px = -1, py = -1;
    for (uint16_t i = 0; i < imuHistCnt; i++) {
      int16_t x = (int16_t)(xOff + i);
      int16_t y = GY_BOT - (int16_t)((imuHist[base+ch][i] - minV) / range * GH + 0.5f);
      if (y < GY_TOP) y = GY_TOP;
      if (y > GY_BOT) y = GY_BOT;
      if (px >= 0) graphSpr.drawLine(px, py, x, y, colors[ch]);
      else         graphSpr.drawPixel(x, y, colors[ch]);
      px = x; py = y;
    }
  }
  graphSpr.pushSprite(1, 1);
}

// ── Select and render the active graph page ───────────────────────────────────
void redraw_graph() {
  switch (graphPage) {
    case 0: draw_pressure_graph();    break;
    case 1: draw_imu_graphs(false);   break;  // gyro
    case 2: draw_imu_graphs(true);    break;  // accel
  }
}

// ── I2C scan – serial only, no display ───────────────────────────────────────
void i2c_scan() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.printf("\nI2C scan (SDA=%d SCL=%d)...\n", I2C_SDA, I2C_SCL);
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 120; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;
    count++;
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    uint8_t r00 = Wire.available() ? Wire.read() : 0xFF;
    const char* name = (addr==0x76||addr==0x77) ? "BMP280" :
                       (addr==0x6A||addr==0x6B) ? "QMI8658" : "---";
    Serial.printf("  0x%02X  R[00]=0x%02X  %s\n", addr, r00, name);
  }
  Serial.printf("Found %d device(s)\n\n", count);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  tft_init();

  // Backlight PWM
  ledcSetup(BL_LEDC_CH, BL_LEDC_FREQ, BL_LEDC_RES);
  ledcAttachPin(TFT_BL_GPIO, BL_LEDC_CH);
  ledcWrite(BL_LEDC_CH, BL_ACTIVE);

  // MAC + BLE name (derived, no stack needed)
  uint64_t mac = ESP.getEfuseMac();
  snprintf(macBuf, sizeof(macBuf), "%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);
  snprintf(bleBuf, sizeof(bleBuf), "ESP32S3-%.4s", macBuf + 8);

  Serial.println();
  Serial.println("TENSTAR ESP32-S3 Test Firmware");
  Serial.printf("Chip  : %s @ %d MHz\n", ESP.getChipModel(), getCpuFrequencyMhz());
  Serial.printf("MAC   : %s\n", macBuf);
  Serial.printf("BLE   : %s\n", bleBuf);

  // I2C + BMP280
  i2c_scan();
  bmpOk = bmp.begin(0x77);
  if (bmpOk) {
    // Max speed mode: IIR X16 suppresses noise, no oversampling needed
    // osrs_p=X1 → meas time ~2.3 ms → ODR ~350 Hz; we sample at 50 Hz
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X1,    // temp: compensation only
                    Adafruit_BMP280::SAMPLING_X1,    // pressure: X1, IIR does the work
                    Adafruit_BMP280::FILTER_X16,     // IIR coeff 16 (strongest)
                    Adafruit_BMP280::STANDBY_MS_1);  // 0.5 ms standby → ~350 Hz ODR
    Serial.printf("BMP280 OK\n");
  } else {
    Serial.printf("BMP280 FAIL\n");
  }

  // NeoPixel
  pinMode(NEO_PWR_PIN, OUTPUT);
  digitalWrite(NEO_PWR_PIN, HIGH);
  neo.begin();
  neo.setBrightness(NEO_BRIGHTNESS);
  neo.clear();
  neo.show();

  // QMI8658 IMU (raw I2C driver)
  imuOk = QMI::init();
  Serial.printf("QMI8658: %s\n", imuOk ? "OK" : "FAIL");
  lastMotionMs = millis();   // start the dim timer from now

  // WiFi – connect, then set up OTA
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(bleBuf);                // mDNS name for OTA: <name>.local
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi  : connecting to %s", WIFI_SSID);
  {
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
      delay(250); Serial.print(".");
    }
    Serial.println();
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("WiFi  : %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("WiFi  : not connected (offline)");

  // OTA – firmware upload over WiFi (use PlatformIO upload_port = <name>.local)
  ArduinoOTA.setHostname(bleBuf);
#ifdef OTA_PASSWORD
  ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
  ArduinoOTA.onStart([]() {
    String type = ArduinoOTA.getCommand() == U_FLASH ? "firmware" : "filesystem";
    Serial.printf("\n[OTA] Start: %s\n", type.c_str());
    // Show status in WiFi row and a dim progress bar in the graph zone
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(154, 97);
    tft.print("OTA...      ");
    tft.fillRect(1, 28, GRAPH_SAMPLES, 10, tft.color565(30,30,30));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint16_t w = (uint16_t)((uint32_t)GRAPH_SAMPLES * progress / total);
    tft.fillRect(1, 28, w, 10, TFT_GREEN);
    Serial.printf("[OTA] %u%%\r", progress * 100 / total);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Done — rebooting");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(154, 97);
    tft.print("OTA OK!     ");
  });
  ArduinoOTA.onError([](ota_error_t err) {
    Serial.printf("[OTA] Error [%u]\n", err);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(154, 97);
    tft.print("OTA FAIL    ");
  });
  ArduinoOTA.begin();
  Serial.printf("OTA   : ready as [%s.local]\n", bleBuf);

#ifndef NO_BLE
  // BLE – Environmental Sensing Service (ESS 0x181A)
  BLEDevice::init(bleBuf);
  BLEServer*  bleServer  = BLEDevice::createServer();
  bleServer->setCallbacks(new BLEConnCB());
  BLEService* bleService = bleServer->createService(BLEUUID((uint16_t)0x181A));

  bleTempChar = bleService->createCharacteristic(
      BLEUUID((uint16_t)0x2A6E),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  bleTempChar->addDescriptor(new BLE2902());

  blePressChar = bleService->createCharacteristic(
      BLEUUID((uint16_t)0x2A6D),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  blePressChar->addDescriptor(new BLE2902());

  bleService->start();
  BLEAdvertising* bleAdv = BLEDevice::getAdvertising();
  bleAdv->addServiceUUID(BLEUUID((uint16_t)0x181A));
  bleAdv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("BLE   : advertising as [%s]\n", bleBuf);
#endif  // NO_BLE

  // Static info-panel values (right col: MAC is fixed)
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(150, 79); tft.print(macBuf);

  // Create sprite for graph zone: GRAPH_SAMPLES wide × 65 tall, positioned at (1,1)
  // Uses PSRAM automatically when available (~30 KB for this size)
  graphSprOk = (bool)graphSpr.createSprite(GRAPH_SAMPLES, 65);
  if (!graphSprOk) Serial.println("WARN: graph sprite alloc failed, falling back to direct draw");

  draw_pressure_graph();     // draws "no data yet" until first BMP read

  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  Serial.println("Running...");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  ArduinoOTA.handle();   // must be first; blocks during OTA flash

  // ── BOOT button: short → next graph page, long → toggle NeoPixel ──────────
  static bool     lastBtn   = HIGH;
  static uint32_t pressTime = 0;
  static bool     longFired = false;
  bool btn = digitalRead(BOOT_BTN_PIN);

  if (btn == LOW && lastBtn == HIGH) {                           // leading edge
    pressTime = millis(); longFired = false;
  }
  if (btn == LOW && !longFired &&
      (millis() - pressTime) >= LONGPRESS_MS) {                  // long-press
    neoEnabled = !neoEnabled;
    if (!neoEnabled) { neo.clear(); neo.show(); }
    longFired = true;
    Serial.printf("Neo: %s\n", neoEnabled ? "ON" : "OFF");
  }
  if (btn == HIGH && lastBtn == LOW) {                           // trailing edge
    if (!longFired && (millis() - pressTime) >= DEBOUNCE_MS) {  // short press
      graphPage = (graphPage + 1) % 3;
      lastMotionMs = millis();                                   // reset dim timer
      if (blDimmed) { blDimmed = false; setBacklight(BL_ACTIVE); }
      redraw_graph();
      Serial.printf("Graph page: %d\n", graphPage);
    }
  }
  lastBtn = btn;

  uint32_t now = millis();
  loopCount++;

  // ── NeoPixel rainbow (skipped when disabled) ───────────────────────────────
  uint8_t r = 0, g = 0, b = 0;
  if (neoEnabled) {
    neoHue += 256;
    uint32_t c32 = neo.gamma32(neo.ColorHSV(neoHue));
    neo.setPixelColor(0, c32);
    neo.show();
    r = (c32 >> 16) & 0xFF;
    g = (c32 >>  8) & 0xFF;
    b = (c32       ) & 0xFF;
  }

  // ── Heap ──────────────────────────────────────────────────────────────────
  uint32_t heapFree  = ESP.getFreeHeap();
  uint32_t heapTotal = ESP.getHeapSize();
  uint8_t  heapPct   = (uint8_t)((heapFree * 100UL) / heapTotal);

  // ── IMU read + motion-based auto-dim (every loop, ~92 Hz) ────────────────
  // Motion = accel magnitude deviation from 1 g.
  // Adaptive baseline: EMA tracks the resting |a| value (accounts for sensor
  // bias / calibration offset — e.g. this sensor reads ~1.09g at rest, not 1.0g).
  // EMA only advances while still, so sustained vibration doesn't shift baseline.
  // Threshold 0.05g is well above noise (~0.01g) and catches any real motion.
  // Gyro noise is too unpredictable to use for dimming.
  static float amagEma    = 0.0f;   // 0 → bootstrapped on first sample
  static float lastAMag   = 1.0f;
  static bool  lastMoving = false;
  if (imuOk) {
    QMI::read(imuAx, imuAy, imuAz, imuGx, imuGy, imuGz);
    float aMag = sqrtf(imuAx*imuAx + imuAy*imuAy + imuAz*imuAz);
    if (amagEma == 0.0f) amagEma = aMag;          // bootstrap
    bool  moving = fabsf(aMag - amagEma) > 0.05f;
    if (!moving) amagEma += 0.02f * (aMag - amagEma); // track baseline only when still
    lastAMag = aMag; lastMoving = moving;
    if (moving) {
      lastMotionMs = now;
      // Wake: cancel any fade and go full bright instantly
      if (blDimmed || blFading) {
        blDimmed  = false;
        blFading  = false;
        setBacklight(BL_ACTIVE);
        Serial.printf("[DIM] wake  aMag=%.3f\n", aMag);
      }
    }
    // Start fade when stillness timeout expires
    if (!blDimmed && !blFading && (now - lastMotionMs) >= DIM_TIMEOUT_MS) {
      blFading    = true;
      blFadeStart = now;
      Serial.println("[DIM] fade started");
    }
  }
  // ── Backlight fade step (runs every loop, independent of imuOk) ──────────
  if (blFading) {
    uint32_t elapsed = now - blFadeStart;
    if (elapsed >= BL_FADE_MS) {
      blFading  = false;
      blDimmed  = true;
      setBacklight(BL_DIM);
      Serial.println("[DIM] backlight dimmed");
    } else {
      // Linear interpolation BL_ACTIVE → BL_DIM over BL_FADE_MS
      float t  = (float)elapsed / (float)BL_FADE_MS;
      uint8_t br = (uint8_t)(BL_ACTIVE + t * (int)(BL_DIM - BL_ACTIVE));
      setBacklight(br);
    }
  }



  // ── Info panel (every loop) ───────────────────────────────────────────────
  tft.setTextSize(1);

  // ── Left column ───────────────────────────────────────────────────────────
  // RGB  (y=70)
  if (neoEnabled) {
    tft.fillRect(26, 69, 14, 8, tft.color565(r, g, b));
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(44, 70);
    tft.printf("#%02X%02X%02X ", r, g, b);
  } else {
    tft.fillRect(26, 69, 14, 8, TFT_BLACK);
    tft.drawRect(26, 69, 14, 8, TFT_DARKGREY);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(44, 70);
    tft.print("off      ");
  }

  // Loop/s  (y=79, after "Loop/s:" = 7ch×6=42 → x=2+42=44)
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setCursor(44, 79);
  tft.printf("%-5lu  ", (unsigned long)lastFps);

  // Heap %  (y=88, after "Heap:" = 5ch×6=30 → x=32)
  tft.setTextColor(heapPct < 20 ? TFT_RED : TFT_GREEN, TFT_BLACK);
  tft.setCursor(32, 88);
  tft.printf("%2d%% %3uk ", heapPct, heapFree / 1024);

  // Temp  (y=97, after "Temp:" = 5ch×6=30 → x=32)
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setCursor(32, 97);
  if (isnan(lastTemp))
    tft.print("---     ");
  else
    tft.printf("%.1f \xB0" "C  ", lastTemp);

  // ── Right column ──────────────────────────────────────────────────────────
  // BL / page + sleep countdown  (y=70)
  tft.setCursor(150, 70);
  static const char* PAGE_NAMES[3] = { "hPa", "Gyr", "Acc" };
  tft.setTextColor(blDimmed ? TFT_DARKGREY : TFT_YELLOW, TFT_BLACK);
  tft.printf("%s %s   ", blDimmed ? "DIM" : "ON ", PAGE_NAMES[graphPage]);

  // MAC  (y=79, static — drawn once in setup)

#ifndef NO_BLE
  // BLE  (y=88)
  tft.setTextColor(bleConnected ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
  tft.setCursor(150, 88);
  tft.printf("%-12s%s", bleBuf, bleConnected ? "*" : " ");
#endif

  // WiFi  (y=97, after "WiFi:" = 5ch×6=30 → val x=154)
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(154, 97);
  IPAddress ip = WiFi.localIP();
  if (ip[0] == 0)
    tft.print("---      ");
  else
    tft.printf("%-13s", ip.toString().c_str());

  // IMU row  (y=106)
  // Accel: left col x=26, format %+4.1f × 3  → e.g. "+0.0-0.1+1.0"
  // Gyro:  right col x=146, format %+5.0f × 3 → e.g. "+   0-  12+  128"
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(26, 106);
  if (imuOk)
    tft.printf("%+4.1f%+4.1f%+4.1f", imuAx, imuAy, imuAz);
  else
    tft.print("  ---      ");

  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.setCursor(146, 106);
  if (imuOk)
    tft.printf("%+4.0f%+4.0f%+4.0f", imuGx, imuGy, imuGz);
  else
    tft.print("  ---      ");

  // BMP280 – 50 Hz graph update + BLE notify ──────────────────────────────────
  if (now - lastPressUpdateMs >= PRESS_UPDATE_MS) {
    lastPressUpdateMs = now;
    if (bmpOk) {
      float hpa = bmp.readPressure() / 100.0f;
      lastTemp  = bmp.readTemperature();
      // Shift-left buffer: append newest sample at the end
      if (pressCnt < GRAPH_SAMPLES) {
        pressHistory[pressCnt++] = hpa;
      } else {
        memmove(&pressHistory[0], &pressHistory[1],
                (GRAPH_SAMPLES - 1) * sizeof(float));
        pressHistory[GRAPH_SAMPLES - 1] = hpa;
      }
      // Append IMU sample (6 channels, same 50 Hz rate as pressure)
      {
        float s[6] = { imuAx, imuAy, imuAz, imuGx, imuGy, imuGz };
        if (imuHistCnt < GRAPH_SAMPLES) {
          for (int i = 0; i < 6; i++) imuHist[i][imuHistCnt] = s[i];
          imuHistCnt++;
        } else {
          for (int i = 0; i < 6; i++) {
            memmove(&imuHist[i][0], &imuHist[i][1], (GRAPH_SAMPLES - 1) * sizeof(float));
            imuHist[i][GRAPH_SAMPLES - 1] = s[i];
          }
        }
      }
      redraw_graph();
#ifndef NO_BLE
      // ESS encoding: temp 0.01 °C (int16), pressure 0.1 Pa (uint32)
      int16_t  bleTemp  = (int16_t)(lastTemp * 100.0f);
      uint32_t blePress = (uint32_t)(hpa * 1000.0f);  // hPa → 0.1 Pa
      if (bleTempChar)  {
        bleTempChar->setValue((uint8_t*)&bleTemp, 2);
        if (bleConnected) bleTempChar->notify();
      }
      if (blePressChar) {
        blePressChar->setValue((uint8_t*)&blePress, 4);
        if (bleConnected) blePressChar->notify();
      }
#endif
    } else if (pressCnt == 0) {
      redraw_graph();  // show error / no-data once
    }
  }

  // ── Heartbeat ─────────────────────────────────────────────────────────────
  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastFps = loopCount; loopCount = 0;
    lastHeartbeatMs = now;
    heartbeatCounter++;
    float lastHpa = pressCnt > 0 ? pressHistory[pressCnt - 1] : 0.0f;
    Serial.printf("#%lu | Heap:%d%% | Loop/s:%lu | BL:%s | Pg:%d(%s) | Neo:%s | %.1f\xB0" "C %.2fhPa",
                  (unsigned long)heartbeatCounter,
                  heapPct, (unsigned long)lastFps,
                  blDimmed ? "dim" : "on", graphPage,
                  graphPage == 0 ? "hPa" : graphPage == 1 ? "Gyr" : "Acc",
                  neoEnabled ? "on" : "off", lastTemp, lastHpa);
    if (imuOk) {
      Serial.printf(" | Acc:%+.2f%+.2f%+.2f g | Gyr:%+.0f%+.0f%+.0f dps",
                    imuAx, imuAy, imuAz, imuGx, imuGy, imuGz);
      Serial.printf(" | aMag:%.3f ema:%.3f mv:%d idle:%lums dim:%d",
                    lastAMag, amagEma, (int)lastMoving,
                    (unsigned long)(now - lastMotionMs), (int)blDimmed);
    }
    Serial.println();
  }
}
