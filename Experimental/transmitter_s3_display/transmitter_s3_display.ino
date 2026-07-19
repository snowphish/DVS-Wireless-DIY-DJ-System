/*
  WAVESHARE ESP32-S3-LCD-1.69 DVS TRANSMITTER  [TEST - STAGE 1: core]
  - All-in-one puck: ESP32-S3R8 + onboard QMI8658 IMU + LCD + touch + charger.
  - STAGE 1 is the DVS core only: reads platter rotation from the QMI8658
    gyro Z axis, sends RPM ~500x/sec over ESP-NOW, spin-cal learn-once,
    battery monitor on GPIO1. Display + touch cue points come in stage 2/3
    (see README). Compiles with NO extra libraries (Wire + ESP-NOW only).

  Bring-up: flash, open Serial at 115200, and check the boot line
  "QMI8658 WHO_AM_I=0x05 @ addr 0x6B" - that confirms the IMU before anything
  else. Then spin the platter and watch the receiver's serial for RPM.

  Pins are from Waveshare's demo - verify against the wiki.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ===== Display: LovyanGFX ST7789V2 (stage 2) ==========================
#define DISPLAY_ENABLE 1
#if DISPLAY_ENABLE
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#endif

// ===== Identity =======================================================
#define DECK_ID 1                // second unit: set to 2

// ===== Board pins (Waveshare ESP32-S3-LCD-1.69) - VERIFY ==============
#define I2C_SDA_PIN 11           // shared bus: QMI8658 + CST816T + PCF85063
#define I2C_SCL_PIN 10
#define BATT_ADC_PIN 1           // on-board battery voltage divider (confirmed)
#define BUZZER_PIN  33           // pull LOW when idle or the board heats up
// Display ST7789V2 (SPI) - from Waveshare demo, verify:
#define LCD_MOSI_PIN 7
#define LCD_SCLK_PIN 6
#define LCD_CS_PIN   5
#define LCD_DC_PIN   4
#define LCD_RST_PIN  8
#define LCD_BL_PIN   15
// Touch CST816T (I2C, shares the QMI8658 bus) - from Waveshare demo, verify:
#define TOUCH_ENABLE  1
#define TOUCH_ADDR    0x15
#define TOUCH_INT_PIN 14
#define TOUCH_RST_PIN 13
#define TOUCH_POLL_MS 20         // 50 Hz touch scan
#define CUE_HEARTBEAT_MS 500     // resend cue state so the receiver can expire it

#if DISPLAY_ENABLE
// ST7789V2 240x280 panel. offset_y/invert/rgb_order are the "verify
// visually" params - if the image is shifted or colours look wrong,
// tweak offset_y (try 0 or 20), invert, or rgb_order.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX() {
    { auto c = _bus.config();
      c.spi_host = SPI2_HOST; c.spi_mode = 0;
      c.freq_write = 40000000; c.freq_read = 16000000;
      c.pin_sclk = LCD_SCLK_PIN; c.pin_mosi = LCD_MOSI_PIN;
      c.pin_miso = -1; c.pin_dc = LCD_DC_PIN;
      _bus.config(c); _panel.setBus(&_bus); }
    { auto c = _panel.config();
      c.pin_cs = LCD_CS_PIN; c.pin_rst = LCD_RST_PIN; c.pin_busy = -1;
      c.memory_width = 240; c.memory_height = 320;
      c.panel_width = 240;  c.panel_height = 280;
      c.offset_x = 0; c.offset_y = 20;
      c.readable = false; c.invert = true; c.rgb_order = false;
      _panel.config(c); }
    { auto c = _light.config();
      c.pin_bl = LCD_BL_PIN; c.freq = 12000; c.pwm_channel = 7;
      _light.config(c); _panel.setLight(&_light); }
    setPanel(&_panel);
  }
};
LGFX display;

// Layout: a status strip at the top, then a 2x2 cue pad filling the rest.
// The cue tiles are static, so they're only redrawn when a cue's press
// state changes (a brief SPI blip on tap) - cheap enough to keep shown
// while spinning. The status strip refreshes only when the platter is
// stopped, so it never adds jitter during play.
#define STATUS_H 34
#define DISP_STATUS_MAX_RPM 3.0f
#define DISP_STATUS_MS 400
uint32_t lastStatusDrawMs = 0;
int      shownBatt = -1, shownLink = -1;
#endif

// ===== Touch cue state ================================================
uint16_t cueMask = 0;             // bits 0..3 = quadrants TL,TR,BL,BR
uint16_t cueRendered = 0xFFFF;    // last-drawn mask (force first draw)
uint32_t lastTouchMs = 0;
uint32_t lastCueHeartbeatMs = 0;

// ===== QMI8658 (gyro) =================================================
// Address is 0x6B (SA0 high) or 0x6A (low); boot auto-detects.
#define QMI8658_ADDR_A 0x6B
#define QMI8658_ADDR_B 0x6A
#define QMI8658_REG_WHOAMI  0x00
#define QMI8658_WHOAMI_VAL  0x05
#define QMI8658_REG_CTRL1   0x02   // serial iface: addr auto-increment, endianness
#define QMI8658_REG_CTRL3   0x04   // gyro: full-scale + ODR
#define QMI8658_REG_CTRL7   0x08   // sensor enable (bit0 accel, bit1 gyro)
#define QMI8658_REG_GYR_Z_L 0x3F   // gyro Z low byte; data is little-endian
#define QMI8658_REG_STATUS0 0x2E

// CTRL3: gFS ±1024 dps (bits[6:4]=6), gODR ~896.8 Hz (bits[3:0]=3) -> 0x63.
// Sensitivity = 2^15 / 1024 = 32.0 LSB/dps. ±1024 dps = 170 RPM ceiling.
#define QMI8658_CTRL3_VAL   0x63
#define GYRO_LSB_PER_DPS    32.0f

uint8_t qmiAddr = QMI8658_ADDR_A;   // resolved at boot

// ===== Radio / protocol ==============================================
#define ESPNOW_CHANNEL 11
#define SEND_RATE_HZ 500
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
#define HANDSHAKE_INTERVAL_MS 250
#define HANDSHAKE_TIMEOUT_MS 2000
#define USE_BROADCAST 1
#define ESPNOW_FAST_RATE 1

#define PROTOCOL_VERSION 1
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4
#define MSG_EVENT 5
#define MSG_BUTTONS 6            // 4-quadrant touch cues (like the dicer)
#define EVT_TRIM_LOADED   1
#define EVT_TRIM_MISSING  2
#define EVT_CAL_SAMPLING  3
#define EVT_CAL_LOCKED    4
#define EVT_BATT_VOLTAGE  5
#define EVT_BATT_LOW      6
#define EVT_BATT_CRITICAL 7
#define EVT_BATT_RECOVERED 8

// ===== Battery monitor ================================================
// On-board divider -> GPIO1. Ratio unknown from the wiki; default 2.0 and
// trim against a multimeter (divider = Vbat / Vread).
#define BATT_DIVIDER 2.00f
#define BATT_CHECK_INTERVAL_MS 5000
#define BATT_REPORT_INTERVAL_MS 60000
#define BATT_LOW_RESEND_MS 10000
#define BATT_LOW_MV 3500
#define BATT_CRIT_MV 3200
#define BATT_HYST_MV 100

// ===== Fixed tuning ===================================================
#define DEADZONE_RPM   0.20f
#define RPM_MULTIPLIER 1.00f
#define SMOOTHING      1.00f

// ===== Spin calibration (learn once) ==================================
#define SPIN_CAL_ENABLE 1
#define SPIN_CAL_FORGET 0
#define SPIN_CAL_ARM_MS 120000UL
#define SPIN_CAL_SETTLE_MS 4000UL
#define SPIN_CAL_SAMPLES 5000
#define SPIN_CAL_STABLE_RPM 0.15f
#define SPIN_CAL_ACCEPT_PCT 5.0f

// ===== Zero calibration ===============================================
#define CAL_STABLE_SAMPLES 400
#define CAL_MIN_COMPARE 20
#define CAL_STABLE_DPS 2.0f

uint8_t receiverMAC[] = { 0x84, 0xFC, 0xE6, 0x61, 0x1C, 0x0C };
uint8_t broadcastMAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  uint8_t  version;
  uint8_t  deckId;
  int16_t  rpmCenti;
  int16_t  gyroRaw;
  uint32_t seq;
  uint32_t timestampMicros;
} dvs_packet;

dvs_packet packet;

volatile bool     receiverReady = false;
volatile uint32_t lastReceiverReplyMillis = 0;

float    gyroOffsetZ = 0.0f;
float    filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;

Preferences prefs;
float scaleTrim = 1.0f;
bool  spinCalLearned = false;
bool  spinCalArmed = true;
float spinCalSum = 0.0f;
float spinCalTarget = 0.0f;
int   spinCalCount = 0;
uint32_t spinCalEnterMs = 0;

uint16_t battMilliVolts = 0;
uint8_t  battState = 0;
uint32_t lastBattCheckMillis = 0;
uint32_t lastBattReportMillis = 0;
uint32_t lastBattLowSendMillis = 0;

// ===== QMI8658 low-level ==============================================
static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(qmiAddr);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission(true) == 0;
}
static bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(qmiAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(qmiAddr, len, (uint8_t)true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buffer[i] = Wire.read();
  return true;
}
static bool readGyroZRaw(int16_t *gyroZ) {
  uint8_t b[2] = {};
  if (!readRegisters(QMI8658_REG_GYR_Z_L, b, 2)) return false;
  *gyroZ = (int16_t)((b[1] << 8) | b[0]);   // little-endian: low byte first
  return true;
}

void setupQMI8658() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // Resolve the I2C address by probing WHO_AM_I at both candidates.
  uint8_t id = 0; bool found = false;
  const uint8_t cand[2] = { QMI8658_ADDR_A, QMI8658_ADDR_B };
  for (int a = 0; a < 2 && !found; a++) {
    qmiAddr = cand[a];
    for (int tries = 0; tries < 5; tries++) {
      if (readRegisters(QMI8658_REG_WHOAMI, &id, 1) && id == QMI8658_WHOAMI_VAL) {
        found = true; break;
      }
      delay(20);
    }
  }
  if (!found) {
    while (true) {
      Serial.printf("ERR qmi8658 not found (last id=0x%02X) - check I2C SDA=%d SCL=%d\n",
                    id, I2C_SDA_PIN, I2C_SCL_PIN);
      delay(1000);
    }
  }
  Serial.printf("QMI8658 WHO_AM_I=0x%02X @ addr 0x%02X\n", id, qmiAddr);

  writeRegister(QMI8658_REG_CTRL1, 0x40);          // address auto-increment, little-endian
  writeRegister(QMI8658_REG_CTRL3, QMI8658_CTRL3_VAL); // gyro ±1024 dps, ~896 Hz
  writeRegister(QMI8658_REG_CTRL7, 0x02);          // enable gyro (bit1)
  delay(100);                                      // gyro wake/settle
}

// Wait for a stable (stopped) window, average it -> gyro zero offset.
void autoCalibrateGyroZ() {
  const int32_t stableRawDelta = (int32_t)(CAL_STABLE_DPS * GYRO_LSB_PER_DPS);
  int stableSamples = 0; int32_t sum = 0;
  uint32_t readFails = 0;
  while (stableSamples < CAL_STABLE_SAMPLES) {
    int16_t raw = 0;
    if (!readGyroZRaw(&raw)) {
      if (++readFails % 500 == 1)
        Serial.printf("gyro read FAILING during cal (%lu fails) - check I2C/wiring\n",
                      (unsigned long)readFails);
      delay(2);
      continue;
    }
    if (stableSamples >= CAL_MIN_COMPARE) {
      int32_t mean = sum / stableSamples;
      if (abs((int32_t)raw - mean) > stableRawDelta) { stableSamples = 0; sum = 0; continue; }
    }
    sum += raw; stableSamples++;
    delay(2);
  }
  gyroOffsetZ = (float)sum / (float)stableSamples;
  Serial.printf("calibrated: offset=%.1f\n", gyroOffsetZ);
}

// Bias-corrected RPM at the gyro's NOMINAL scale (before any trim).
// Sign is +(dps/6) for this board's mounting (QMI8658 Z, board flat on the
// platter) - the opposite of the C3 puck's orientation.
static inline float rawGyroToNominalRPM(int16_t gyroRaw) {
  float corrected = (float)gyroRaw - gyroOffsetZ;
  float dps = corrected / GYRO_LSB_PER_DPS;
  return (dps / 6.0f);
}

static inline float nominalToOutputRPM(float nominalRpm) {
  float rpm = nominalRpm * RPM_MULTIPLIER * scaleTrim;
  if (fabsf(rpm) < DEADZONE_RPM) rpm = 0.0f;
  return rpm;
}

void serviceSpinCal(float nominalRpm) {
#if SPIN_CAL_ENABLE
  if (!spinCalArmed) return;
  if (millis() > SPIN_CAL_ARM_MS) { spinCalArmed = false; return; }

  float trimmed = fabsf(nominalRpm) * RPM_MULTIPLIER * scaleTrim;
  float target = 0.0f;
  if      (fabsf(trimmed - 33.3333f) < 33.3333f * SPIN_CAL_ACCEPT_PCT * 0.01f) target = 33.3333f;
  else if (fabsf(trimmed - 45.0f)    < 45.0f    * SPIN_CAL_ACCEPT_PCT * 0.01f) target = 45.0f;

  if (target == 0.0f || (spinCalCount > 0 && target != spinCalTarget)) {
    spinCalCount = 0; spinCalSum = 0.0f; spinCalTarget = 0.0f; spinCalEnterMs = 0;
    return;
  }
  uint32_t now = millis();
  if (spinCalEnterMs == 0) spinCalEnterMs = now;
  if (now - spinCalEnterMs < SPIN_CAL_SETTLE_MS) return;

  if (spinCalCount >= 20 &&
      fabsf(fabsf(nominalRpm) - spinCalSum / spinCalCount) > SPIN_CAL_STABLE_RPM) {
    spinCalCount = 0; spinCalSum = 0.0f; spinCalEnterMs = 0;
    return;
  }
  if (spinCalCount == 0) {
    Serial.printf("spin-cal: settled at ~%.2f, sampling %d s...\n",
                  target, SPIN_CAL_SAMPLES / SEND_RATE_HZ);
    sendEventMessage(EVT_CAL_SAMPLING, target);
  }
  spinCalTarget = target;
  spinCalSum += fabsf(nominalRpm);
  spinCalCount++;
  if (spinCalCount < SPIN_CAL_SAMPLES) return;

  float newTrim = target / ((spinCalSum / spinCalCount) * RPM_MULTIPLIER);
  if (newTrim > 0.90f && newTrim < 1.10f) {
    scaleTrim = newTrim;
    spinCalLearned = true;
    prefs.putFloat("trim", scaleTrim);
    Serial.printf("spin-cal: locked %.2f rpm reference, trim=%.5f (saved)\n", target, scaleTrim);
    sendEventMessage(EVT_CAL_LOCKED, scaleTrim);
  }
  spinCalArmed = false;
#else
  (void)nominalRpm;
#endif
}

// ===== ESP-NOW ========================================================
volatile uint32_t txAckCount = 0, txNoAckCount = 0;
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) txAckCount++; else txNoAckCount++;
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) return;
  dvs_packet in; memcpy(&in, dataPtr, sizeof(in));
  if (in.version != PROTOCOL_VERSION || in.deckId != DECK_ID) return;
  if (in.msgType == MSG_WELCOME || in.msgType == MSG_PING) {
    receiverReady = true;
    lastReceiverReplyMillis = millis();
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { while (true) { Serial.println("ERR espnow_init"); delay(1000); } }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = ESPNOW_CHANNEL; peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { while (true) { Serial.println("ERR add_peer"); delay(1000); } }

#if USE_BROADCAST
  esp_now_peer_info_t bcast = {};
  memcpy(bcast.peer_addr, broadcastMAC, 6);
  bcast.channel = ESPNOW_CHANNEL; bcast.encrypt = false;
  esp_now_add_peer(&bcast);
#endif

#if ESPNOW_FAST_RATE
  esp_now_rate_config_t rateCfg = {};
  rateCfg.phymode = WIFI_PHY_MODE_11G;
  rateCfg.rate = WIFI_PHY_RATE_24M;
  esp_now_set_peer_rate_config(receiverMAC, &rateCfg);
#if USE_BROADCAST
  esp_now_set_peer_rate_config(broadcastMAC, &rateCfg);
#endif
#endif
}

void sendControlMessage(uint8_t msgType) {
  dvs_packet c = {};
  c.msgType = msgType; c.version = PROTOCOL_VERSION; c.deckId = DECK_ID;
  c.seq = sequenceNumber; c.timestampMicros = micros();
  esp_now_send(receiverMAC, (uint8_t *)&c, sizeof(c));
}

void sendEventMessage(uint8_t code, float value) {
  dvs_packet e = {};
  e.msgType = MSG_EVENT; e.version = PROTOCOL_VERSION; e.deckId = DECK_ID;
  e.rpmCenti = code;
  e.timestampMicros = (uint32_t)lroundf(value * 1000000.0f);
  esp_now_send(receiverMAC, (uint8_t *)&e, sizeof(e));
}

void waitForReceiver() {
  uint32_t lastHello = 0, lastReport = 0;
  while (!receiverReady) {
    uint32_t now = millis();
    if (now - lastHello >= HANDSHAKE_INTERVAL_MS) { sendControlMessage(MSG_HELLO); lastHello = now; }
    if (now - lastReport >= 2000) {
      lastReport = now;
      Serial.printf("handshake: HELLO acked=%lu no-ack=%lu\n",
                    (unsigned long)txAckCount, (unsigned long)txNoAckCount);
    }
    delay(10);
  }
}

// ===== Battery monitor ================================================
uint16_t readBatteryMilliVolts() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(BATT_ADC_PIN);
  return (uint16_t)((sum / 8) * BATT_DIVIDER);
}

void serviceBattery() {
  uint32_t now = millis();
  if (now - lastBattCheckMillis < BATT_CHECK_INTERVAL_MS) return;
  lastBattCheckMillis = now;
  battMilliVolts = readBatteryMilliVolts();
  float volts = battMilliVolts / 1000.0f;

  uint8_t newState = 0;
  if      (battMilliVolts < BATT_CRIT_MV) newState = 2;
  else if (battMilliVolts < BATT_LOW_MV)  newState = 1;
  else if (battState != 0 && battMilliVolts < BATT_LOW_MV + BATT_HYST_MV)
    newState = battState;

  if (newState != battState) {
    battState = newState;
    lastBattLowSendMillis = 0;
    if (battState == 0) {
      sendEventMessage(EVT_BATT_RECOVERED, volts);
      Serial.printf("battery recovered: %.2f V\n", volts);
    }
  }
  if (battState != 0 && now - lastBattLowSendMillis >= BATT_LOW_RESEND_MS) {
    lastBattLowSendMillis = now;
    sendEventMessage(battState == 2 ? EVT_BATT_CRITICAL : EVT_BATT_LOW, volts);
    Serial.printf("battery %s: %.2f V\n", battState == 2 ? "CRITICAL" : "LOW", volts);
  }
  if (now - lastBattReportMillis >= BATT_REPORT_INTERVAL_MS) {
    lastBattReportMillis = now;
    sendEventMessage(EVT_BATT_VOLTAGE, volts);
  }
}

// ===== Touch cues (CST816T, stage 3) =================================
// Full cue state sent to the receiver on every change + a heartbeat, so a
// lost packet can't stick a note (state-based, like the dicer). Unicast so
// it retries. Receiver maps to MIDI: note = 60 + (deckId-1)*4 + quadrant.
void sendButtonState() {
  dvs_packet p = {};
  p.msgType = MSG_BUTTONS; p.version = PROTOCOL_VERSION; p.deckId = DECK_ID;
  p.rpmCenti = (int16_t)cueMask;   // quadrant bitmask
  p.gyroRaw = 0;
  p.seq = 0;                       // buttons don't use the DATA seq stream
  p.timestampMicros = micros();
  esp_now_send(receiverMAC, (uint8_t *)&p, sizeof(p));
  lastCueHeartbeatMs = millis();
}

#if TOUCH_ENABLE
static bool touchReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)TOUCH_ADDR, len, (uint8_t)true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void setupTouch() {
  pinMode(TOUCH_RST_PIN, OUTPUT);
  digitalWrite(TOUCH_RST_PIN, LOW);  delay(20);
  digitalWrite(TOUCH_RST_PIN, HIGH); delay(60);   // CST816T reset + boot
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
}

// Returns true + fills x,y if a finger is down.
static bool readTouch(uint16_t *x, uint16_t *y) {
  uint8_t b[6] = {};                 // regs 0x01..0x06
  if (!touchReadRegs(0x01, b, 6)) return false;
  if ((b[1] & 0x0F) == 0) return false;                 // finger count == 0
  *x = ((uint16_t)(b[2] & 0x0F) << 8) | b[3];           // XH, XL
  *y = ((uint16_t)(b[4] & 0x0F) << 8) | b[5];           // YH, YL
  return true;
}

// Maps a touch to a quadrant bitmask (0 in the status strip = no cue).
void serviceTouch() {
  uint32_t now = millis();
  if (now - lastTouchMs >= TOUCH_POLL_MS) {
    lastTouchMs = now;
    uint16_t x = 0, y = 0, newMask = 0;
    if (readTouch(&x, &y) && y >= STATUS_H) {
      uint8_t qx = (x < 120) ? 0 : 1;
      uint8_t qy = (y < (STATUS_H + (280 - STATUS_H) / 2)) ? 0 : 2;
      newMask = 1 << (qx + qy);
    }
    if (newMask != cueMask) { cueMask = newMask; sendButtonState(); }
  }
  if (cueMask == 0 && now - lastCueHeartbeatMs >= CUE_HEARTBEAT_MS) sendButtonState();
}
#endif

// ===== Display rendering (stage 2/3) ==================================
#if DISPLAY_ENABLE
LGFX_Sprite canvas(&display);

void setupDisplay() {
  display.init();
  display.setRotation(0);            // 240 wide x 280 tall
  display.setBrightness(200);
  display.fillScreen(TFT_BLACK);
  canvas.setColorDepth(16);
  canvas.setPsram(true);             // 240x280x2 = 134 KB in PSRAM
  canvas.createSprite(display.width(), display.height());
  canvas.setTextDatum(middle_center);
}

// Simple centered boot line (platter is stopped during boot).
void drawBootMsg(const char *msg) {
  canvas.fillScreen(TFT_BLACK);
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.setTextSize(2);
  canvas.drawString("DVS puck", canvas.width() / 2, 60);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setTextSize(1);
  canvas.drawString(msg, canvas.width() / 2, canvas.height() / 2);
  canvas.pushSprite(0, 0);
}

// Top status strip: deck id, battery voltage, link dot.
static void drawStatusStrip() {
  int w = canvas.width();
  canvas.fillRect(0, 0, w, STATUS_H, TFT_BLACK);
  canvas.setTextSize(2);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(TFT_CYAN, TFT_BLACK);
  canvas.drawString("D" + String(DECK_ID), 6, STATUS_H / 2);
  uint16_t bcol = battState == 2 ? TFT_RED : (battState == 1 ? TFT_ORANGE : TFT_GREEN);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(bcol, TFT_BLACK);
  char buf[12]; snprintf(buf, sizeof(buf), "%.2fV", battMilliVolts / 1000.0f);
  canvas.drawString(buf, w / 2, STATUS_H / 2);
  canvas.fillCircle(w - 16, STATUS_H / 2, 7, receiverReady ? TFT_GREEN : TFT_RED);
  canvas.setTextDatum(middle_center);
}

// One cue tile (q = 0..3 = TL,TR,BL,BR), highlighted when pressed.
static void drawQuad(uint8_t q, bool pressed) {
  int halfW = canvas.width() / 2;
  int gy = STATUS_H, gh = canvas.height() - STATUS_H, halfH = gh / 2;
  int qx = (q & 1) ? halfW : 0;
  int qy = (q & 2) ? gy + halfH : gy;
  uint16_t fill = pressed ? TFT_ORANGE : TFT_NAVY;
  canvas.fillRect(qx + 2, qy + 2, halfW - 4, halfH - 4, fill);
  canvas.drawRect(qx + 2, qy + 2, halfW - 4, halfH - 4, TFT_DARKGREY);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(TFT_WHITE, fill);
  canvas.setTextSize(4);
  canvas.drawString(String(q + 1), qx + halfW / 2, qy + halfH / 2);
}

static void renderCuePad() {
  canvas.fillScreen(TFT_BLACK);
  drawStatusStrip();
  for (uint8_t q = 0; q < 4; q++) drawQuad(q, cueMask & (1 << q));
  canvas.pushSprite(0, 0);
  cueRendered = cueMask;
}

// Cue taps redraw immediately (a brief SPI blip, even while spinning, so
// the tiles react). The status strip refreshes only when stopped, so it
// never adds jitter during play. If cue taps cause audible glitches, the
// next step is moving the sprite push to core 0 with a Wire mutex.
void serviceDisplay() {
  if (cueMask != cueRendered) {
    for (uint8_t q = 0; q < 4; q++)
      if ((cueMask ^ cueRendered) & (1 << q)) drawQuad(q, cueMask & (1 << q));
    canvas.pushSprite(0, 0);
    cueRendered = cueMask;
    return;
  }
  uint32_t now = millis();
  if (now - lastStatusDrawMs < DISP_STATUS_MS) return;
  lastStatusDrawMs = now;
  if (fabsf(filteredRPM) >= DISP_STATUS_MAX_RPM) return;   // no SPI while playing
  int b = battState, l = receiverReady ? 1 : 0;
  static int lastMv = -1;
  if (b == shownBatt && l == shownLink && abs((int)battMilliVolts - lastMv) <= 20) return;
  shownBatt = b; shownLink = l; lastMv = battMilliVolts;
  drawStatusStrip();
  canvas.pushSprite(0, 0);
}
#endif

// ===== setup / loop ===================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("BOOT: dvs transmitter S3-LCD-1.69 (stage 1) alive");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);   // idle low or the passive buzzer heats the LDO
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);

#if DISPLAY_ENABLE
  setupDisplay();
  drawBootMsg("starting...");
#endif

  Serial.println("BOOT: loading prefs...");
  prefs.begin("dvs", false);
#if SPIN_CAL_FORGET
  prefs.remove("trim");
  Serial.println("spin-cal: stored trim erased");
#endif
  float stored = prefs.getFloat("trim", 0.0f);
  if (stored > 0.90f && stored < 1.10f) {
    scaleTrim = stored;
    spinCalLearned = true;
    Serial.printf("spin-cal: using stored trim=%.5f\n", scaleTrim);
  } else {
    Serial.println("spin-cal: no stored trim - spin platter at 0% pitch to learn");
  }
  spinCalArmed = !spinCalLearned;

  Serial.println("BOOT: qmi8658 init...");
#if DISPLAY_ENABLE
  drawBootMsg("gyro init...");
#endif
  setupQMI8658();
  Serial.println("BOOT: qmi8658 OK");
#if TOUCH_ENABLE
  setupTouch();                    // shares the QMI8658 I2C bus (already up)
  Serial.println("BOOT: touch OK");
#endif
  Serial.println("BOOT: espnow init...");
  setupEspNow();
  Serial.println("BOOT: espnow OK, waiting for receiver...");
#if DISPLAY_ENABLE
  drawBootMsg("waiting for receiver...");
#endif
  waitForReceiver();
  Serial.println("BOOT: receiver linked");
  sendEventMessage(spinCalLearned ? EVT_TRIM_LOADED : EVT_TRIM_MISSING, scaleTrim);
  battMilliVolts = readBatteryMilliVolts();
  sendEventMessage(EVT_BATT_VOLTAGE, battMilliVolts / 1000.0f);
  Serial.printf("battery: %.2f V\n", battMilliVolts / 1000.0f);
  Serial.println("BOOT: gyro zero cal (platter stopped)...");
#if DISPLAY_ENABLE
  drawBootMsg("hold still: zeroing gyro");
#endif
  autoCalibrateGyroZ();
  Serial.println("BOOT: running");
#if DISPLAY_ENABLE
  renderCuePad();                  // show the 4-quadrant cue pad
#endif
  nextSendMicros = micros();
}

void loop() {
  serviceBattery();
#if TOUCH_ENABLE
  serviceTouch();
#endif
#if DISPLAY_ENABLE
  serviceDisplay();
#endif
  uint32_t now = micros();
  if ((int32_t)(now - nextSendMicros) < 0) return;
  nextSendMicros += SEND_INTERVAL_US;
  if ((int32_t)(now - nextSendMicros) > (int32_t)SEND_INTERVAL_US) nextSendMicros = now + SEND_INTERVAL_US;

  int16_t gyroRaw = 0;
  if (!readGyroZRaw(&gyroRaw)) return;

  float nominalRpm = rawGyroToNominalRPM(gyroRaw);
  serviceSpinCal(nominalRpm);
  float rpm = nominalToOutputRPM(nominalRpm);
  filteredRPM += (rpm - filteredRPM) * SMOOTHING;

  packet.msgType = MSG_DATA;
  packet.version = PROTOCOL_VERSION;
  packet.deckId = DECK_ID;
  packet.rpmCenti = (int16_t)constrain(lroundf(filteredRPM * 100.0f), -32768, 32767);
  packet.gyroRaw = gyroRaw;
  packet.seq = sequenceNumber++;
  packet.timestampMicros = now;

#if USE_BROADCAST
  esp_now_send(broadcastMAC, (uint8_t *)&packet, sizeof(packet));
#else
  esp_now_send(receiverMAC,  (uint8_t *)&packet, sizeof(packet));
#endif

  if (millis() - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS) {
    receiverReady = false;
    waitForReceiver();
    nextSendMicros = micros();
  }
}
