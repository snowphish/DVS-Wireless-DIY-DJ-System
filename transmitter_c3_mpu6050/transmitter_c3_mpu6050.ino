/*
  ESP32-C3 DVS TRANSMITTER + MPU6050  [TEST: low-battery indicator]
  - Battery voltage sensed on GPIO3 via a 2:1 divider from BAT+ (after the
    power switch): BAT+ -> 100k -> GPIO3 -> 100k -> GND, 100 nF GPIO3->GND.
  - Reports voltage / LOW / CRITICAL to the receiver over MSG_EVENT; the
    receiver blinks its status LED (deck 1 = slow blink, deck 2 = 2 pulses).
  - Reads platter rotation speed from the gyro's Z axis.
  - Sends RPM ~500x/sec to the receiver over ESP-NOW (broadcast).
  - Auto-calibrates the gyro zero at boot -> power-cycle = recalibrate
    (do it with the platter STOPPED).

  No LED, no USB app/serial protocol. A plain Serial.begin is kept for
  optional boot/debug messages (works on any USB mode; open Serial Monitor
  at 115200 if you want to watch it).
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ===== Identity =======================================================
// The puck carries no deck number and no receiver MAC. It broadcasts, and
// the receiver auto-assigns a deck by this puck's (factory, stable) MAC:
// first puck to reach the receiver during its pairing window = deck 1,
// second = deck 2, and a puck that drops and returns reclaims its slot.
// Outgoing packets send deckId 0 ("unassigned"); the receiver ignores it
// and routes by MAC. The assigned deck comes back in WELCOME/PING, for
// serial display only.
#define DECK_ID 0

// ===== MPU6050 (gyro) =================================================
#define SDA_PIN 8
#define SCL_PIN 9
#define MPU6050_ADDR 0x68
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_PWR_MGMT_1  0x6B

// Gyro DLPF: 1 = 188 Hz / ~1.9 ms delay (responsive). 2=98Hz, 3=42Hz.
#define GYRO_DLPF_CFG 0x01
// Gyro full-scale range + matching sensitivity (keep these consistent):
//   0x00=+-250dps/131.0   0x08=+-500dps/65.5   0x10=+-1000dps/32.8   0x18=+-2000dps/16.4
#define GYRO_FS_SEL      0x10     // +-1000 dps => ~5x nominal backspin headroom
#define GYRO_LSB_PER_DPS 32.8f

// ===== Radio / protocol ==============================================
#define ESPNOW_CHANNEL 11
#define SEND_RATE_HZ 500
#define SEND_INTERVAL_US (1000000UL / SEND_RATE_HZ)
#define HANDSHAKE_INTERVAL_MS 250
#define HANDSHAKE_TIMEOUT_MS 2000
#define USE_BROADCAST 1           // broadcast DATA = no ACK/retry = low jitter
// Default ESP-NOW rate is 1 Mbps 802.11b: ~650 us airtime per packet, i.e.
// ~33% channel use per deck at 500 Hz (two decks collide a lot). Legacy
// OFDM 6 Mbps still cuts airtime ~3x but decodes ~15 dB weaker signals
// than the 24 Mbps used before - much better range; 24M was dropping ~40%
// of packets across a room. Only affects our TX side; any ESP32 receives
// it fine. Set to 0 to fall back to the stock rate.
#define ESPNOW_FAST_RATE 1

#define PROTOCOL_VERSION 1
#define MSG_HELLO 1
#define MSG_WELCOME 2
#define MSG_DATA 3
#define MSG_PING 4
// Status events relayed to the receiver's serial monitor - the puck spins
// on battery, so its own USB serial is unreachable during the ritual.
#define MSG_EVENT 5
#define EVT_TRIM_LOADED   1      // boot: using stored trim (value = trim)
#define EVT_TRIM_MISSING  2      // boot: no trim - spin at 0% pitch to learn
#define EVT_CAL_SAMPLING  3      // reference settled, 10 s sampling started
#define EVT_CAL_LOCKED    4      // trim learned and saved (value = trim)
#define EVT_BATT_VOLTAGE  5      // periodic battery report (value = volts)
#define EVT_BATT_LOW      6      // below BATT_LOW_MV (value = volts)
#define EVT_BATT_CRITICAL 7      // below BATT_CRIT_MV (value = volts)
#define EVT_BATT_RECOVERED 8     // back above LOW+HYST (value = volts)

// ===== Battery monitor (TEST) =========================================
// BAT+ (after the power switch, so no drain when off) -> 100k -> GPIO3
// -> 100k -> GND, plus 100 nF from GPIO3 to GND (the ADC needs the low
// impedance). analogReadMilliVolts uses the eFuse factory calibration.
#define BATT_ADC_PIN 3
#define BATT_DIVIDER 2.00f            // (100k+100k)/100k; trim if resistors are off
#define BATT_CHECK_INTERVAL_MS 5000
#define BATT_REPORT_INTERVAL_MS 60000 // periodic voltage EVT to the receiver
#define BATT_LOW_RESEND_MS 10000      // re-shout LOW so the receiver can expire it
#define BATT_LOW_MV 3500              // 16340 Li-ion discharge knee
#define BATT_CRIT_MV 3200
#define BATT_HYST_MV 100              // recover only above LOW+HYST

// ===== Fixed tuning ===================================================
#define DEADZONE_RPM   0.20f
#define RPM_MULTIPLIER 1.00f      // manual trim; usually leave at 1.0 and let
                                  // spin-cal learn the gyro scale instead
#define SMOOTHING      1.00f      // 1.0 = pass-through (lowest latency)

// ===== Spin calibration (gyro scale, per-unit, learned) ===============
// The quartz-locked platter is a precise speed reference: within the first
// 2 min after boot, a stable reading near 33.33 (or 45.00) RPM is taken as
// exactly that speed and the gyro scale trim is derived from it, then saved
// to flash. Do the spin-up at 0% PITCH or it will learn the wrong trim.
// (Zero-offset still comes from the stopped-platter boot cal - bias drifts
// with temperature every boot; scale is a per-chip constant.)
#define SPIN_CAL_ENABLE 1
#define SPIN_CAL_FORGET 0         // flash once with 1 to erase a bad trim
#define SPIN_CAL_ARM_MS 120000UL
#define SPIN_CAL_SETTLE_MS 4000UL // wait after reaching speed: the platter's
                                  // last ~0.1% of spin-up must not be sampled
#define SPIN_CAL_SAMPLES 5000     // ~10 s at 500 Hz
#define SPIN_CAL_STABLE_RPM 0.15f // max wobble around the running mean
#define SPIN_CAL_FIRST_PCT 5.0f   // uncalibrated: full gyro tolerance
#define SPIN_CAL_REFINE_PCT 1.0f  // calibrated: only refine, never re-learn big

// ===== Calibration ====================================================
#define CAL_STABLE_SAMPLES 400
#define CAL_MIN_COMPARE 20
#define CAL_STABLE_DPS 2.0f       // allowed jitter while "stopped"

// No receiver MAC to configure: the puck broadcasts and the receiver pairs
// to us by our MAC. Same binary flashes to every puck.
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
volatile uint8_t  assignedDeck = 0;        // deck the receiver reports (0 = none yet)

float    gyroOffsetZ = 0.0f;
float    filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;

Preferences prefs;
float scaleTrim = 1.0f;           // learned gyro scale correction
bool  spinCalLearned = false;
bool  spinCalArmed = true;
float spinCalSum = 0.0f;
float spinCalTarget = 0.0f;
int   spinCalCount = 0;
uint32_t spinCalEnterMs = 0;      // when the reading entered the target window

uint16_t battMilliVolts = 0;
uint8_t  battState = 0;           // 0 ok, 1 low, 2 critical
uint32_t lastBattCheckMillis = 0;
uint32_t lastBattReportMillis = 0;
uint32_t lastBattLowSendMillis = 0;

// ===== MPU6050 low-level ==============================================
static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission(true) == 0;
}
static bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDR, len, true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buffer[i] = Wire.read();
  return true;
}
static bool readGyroZRaw(int16_t *gyroZ) {
  uint8_t b[2] = {};
  if (!readRegisters(MPU6050_REG_GYRO_ZOUT_H, b, 2)) return false;
  *gyroZ = (int16_t)((b[0] << 8) | b[1]);
  return true;
}

void setupMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) {
    while (true) { Serial.println("ERR mpu_init"); delay(1000); }
  }
  delay(50);
  writeRegister(MPU6050_REG_CONFIG, GYRO_DLPF_CFG);     // low-pass filter
  writeRegister(MPU6050_REG_SMPLRT_DIV, 0x00);          // 1 kHz internal rate
  writeRegister(MPU6050_REG_GYRO_CONFIG, GYRO_FS_SEL);  // full-scale range
  delay(50);
}

// Wait for a stable (stopped) window, average it -> gyro zero offset.
// Restarts the window if it detects motion, so it never zeroes while spinning.
void autoCalibrateGyroZ() {
  const int32_t stableRawDelta = (int32_t)(CAL_STABLE_DPS * GYRO_LSB_PER_DPS);
  int stableSamples = 0; int32_t sum = 0;
  uint32_t readFails = 0;
  while (stableSamples < CAL_STABLE_SAMPLES) {
    int16_t raw = 0;
    if (!readGyroZRaw(&raw)) {
      // Previously a silent infinite loop when I2C died - now it tells you.
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
static inline float rawGyroToNominalRPM(int16_t gyroRaw) {
  float corrected = (float)gyroRaw - gyroOffsetZ;
  float dps = corrected / GYRO_LSB_PER_DPS;
  return -(dps / 6.0f);
}

static inline float nominalToOutputRPM(float nominalRpm) {
  float rpm = nominalRpm * RPM_MULTIPLIER * scaleTrim;
  if (fabsf(rpm) < DEADZONE_RPM) rpm = 0.0f;
  return rpm;
}

// Learn the gyro scale from a quartz-locked platter. Accepts a stable
// ~2 s window near 33.33 or 45.00 RPM within the arming period, computes
// trim = target/measured, and persists it. Runs once per boot at most.
void serviceSpinCal(float nominalRpm) {
#if SPIN_CAL_ENABLE
  if (!spinCalArmed) return;
  if (millis() > SPIN_CAL_ARM_MS) { spinCalArmed = false; return; }

  float trimmed = fabsf(nominalRpm) * RPM_MULTIPLIER * scaleTrim;
  float windowPct = spinCalLearned ? SPIN_CAL_REFINE_PCT : SPIN_CAL_FIRST_PCT;
  float target = 0.0f;
  if      (fabsf(trimmed - 33.3333f) < 33.3333f * windowPct * 0.01f) target = 33.3333f;
  else if (fabsf(trimmed - 45.0f)    < 45.0f    * windowPct * 0.01f) target = 45.0f;

  if (target == 0.0f || (spinCalCount > 0 && target != spinCalTarget)) {
    spinCalCount = 0; spinCalSum = 0.0f; spinCalTarget = 0.0f;
    spinCalEnterMs = 0;                    // left the window -> re-settle
    return;
  }
  // Settle: the platter creeps through its last ~0.1% of spin-up after the
  // reading first enters the window; sampling that ramp biases the trim.
  uint32_t now = millis();
  if (spinCalEnterMs == 0) spinCalEnterMs = now;
  if (now - spinCalEnterMs < SPIN_CAL_SETTLE_MS) return;

  if (spinCalCount >= 20 &&
      fabsf(fabsf(nominalRpm) - spinCalSum / spinCalCount) > SPIN_CAL_STABLE_RPM) {
    spinCalCount = 0; spinCalSum = 0.0f;   // wobble -> restart settle + window
    spinCalEnterMs = 0;
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
    Serial.printf("spin-cal: locked %.2f rpm reference, trim=%.5f (saved)\n",
                  target, scaleTrim);
    sendEventMessage(EVT_CAL_LOCKED, scaleTrim);
  }
  spinCalArmed = false;
#else
  (void)nominalRpm;
#endif
}

// ===== ESP-NOW ========================================================
// Unicast frames (HELLO) are MAC-acked, so the send status tells us
// whether the receiver physically heard us - broadcast always "succeeds".
volatile uint32_t txAckCount = 0, txNoAckCount = 0;
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) txAckCount++; else txNoAckCount++;
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *dataPtr, int len) {
  if (len != sizeof(dvs_packet)) return;
  dvs_packet in; memcpy(&in, dataPtr, sizeof(in));
  if (in.version != PROTOCOL_VERSION) return;
  // WELCOME/PING are unicast to our MAC, so they are for us whatever deck
  // number they carry; adopt the deck the receiver assigned (display only).
  if (in.msgType == MSG_WELCOME || in.msgType == MSG_PING) {
    assignedDeck = in.deckId;
    receiverReady = true;
    lastReceiverReplyMillis = millis();
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Max radio TX power (units 0.25 dBm; the driver clamps to chip limit).
  esp_wifi_set_max_tx_power(84);
  if (esp_now_init() != ESP_OK) { while (true) { Serial.println("ERR espnow_init"); delay(1000); } }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t bcast = {};
  memcpy(bcast.peer_addr, broadcastMAC, 6);
  bcast.channel = ESPNOW_CHANNEL; bcast.encrypt = false;
  if (esp_now_add_peer(&bcast) != ESP_OK) { while (true) { Serial.println("ERR add_peer"); delay(1000); } }

#if ESPNOW_FAST_RATE
  esp_now_rate_config_t rateCfg = {};
  rateCfg.phymode = WIFI_PHY_MODE_11G;
  rateCfg.rate = WIFI_PHY_RATE_6M;
  esp_now_set_peer_rate_config(broadcastMAC, &rateCfg);
#endif
}

void sendControlMessage(uint8_t msgType) {
  dvs_packet c = {};
  c.msgType = msgType; c.version = PROTOCOL_VERSION; c.deckId = DECK_ID;
  c.seq = sequenceNumber; c.timestampMicros = micros();
  esp_now_send(broadcastMAC, (uint8_t *)&c, sizeof(c));
}

// Status event -> receiver serial. rpmCenti carries the event code,
// timestampMicros the value as fixed-point x1e6 (e.g. trim 0.99810).
void sendEventMessage(uint8_t code, float value) {
  dvs_packet e = {};
  e.msgType = MSG_EVENT; e.version = PROTOCOL_VERSION; e.deckId = DECK_ID;
  e.rpmCenti = code;
  e.timestampMicros = (uint32_t)lroundf(value * 1000000.0f);
  esp_now_send(broadcastMAC, (uint8_t *)&e, sizeof(e));
}

void waitForReceiver() {
  uint32_t lastHello = 0, lastReport = 0;
  while (!receiverReady) {
    uint32_t now = millis();
    if (now - lastHello >= HANDSHAKE_INTERVAL_MS) { sendControlMessage(MSG_HELLO); lastHello = now; }
    if (now - lastReport >= 2000) {
      lastReport = now;
      Serial.printf("handshake: HELLO acked=%lu no-ack=%lu (acked but no WELCOME = our RX side is deaf)\n",
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

// Checked every 5 s from loop(). LOW/CRITICAL is re-sent every 10 s while
// it persists, so the receiver can expire the state if we vanish.
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
    newState = battState;                 // hysteresis: hold state near the edge

  if (newState != battState) {
    battState = newState;
    lastBattLowSendMillis = 0;            // announce the new state right away
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

// ===== setup / loop ===================================================
void setup() {
  Serial.begin(115200);
  delay(500);                 // give USB-CDC a moment so early prints show
  Serial.println();
  Serial.println("BOOT: dvs transmitter (test build) alive");
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);  // full 0-2.5 V ADC range

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
  // Scale is learned ONCE, then locked. Re-learning on later boots folded
  // whatever bias drift existed at that moment into the trim (scale and
  // bias are inseparable in a single spin measurement) and corrupted it
  // over time. SPIN_CAL_FORGET re-learns. Note: the residual drift seen
  // at 33 rpm over hours is the gyro's scale-vs-temperature coefficient -
  // no zero-offset scheme can fix that (verified: cooling reverses it,
  // re-zeroing doesn't touch it). Better IMU or temp-comp if it matters.
  spinCalArmed = !spinCalLearned;

  Serial.println("BOOT: mpu6050 init...");
  setupMPU6050();
  Serial.println("BOOT: mpu6050 OK");
  Serial.println("BOOT: espnow init...");
  setupEspNow();
  Serial.println("BOOT: espnow OK, waiting for receiver...");
  waitForReceiver();
  Serial.printf("BOOT: receiver linked, assigned deck %u\n", assignedDeck);
  // Link is up now - tell the receiver's serial our calibration state
  // (the puck's own serial is unreachable once it's on the platter).
  sendEventMessage(spinCalLearned ? EVT_TRIM_LOADED : EVT_TRIM_MISSING,
                   scaleTrim);
  battMilliVolts = readBatteryMilliVolts();
  sendEventMessage(EVT_BATT_VOLTAGE, battMilliVolts / 1000.0f);
  Serial.printf("battery: %.2f V\n", battMilliVolts / 1000.0f);
  Serial.println("BOOT: gyro zero cal (platter stopped)...");
  autoCalibrateGyroZ();       // platter must be stopped at power-on
  Serial.println("BOOT: running");
  nextSendMicros = micros();
}

void loop() {
  serviceBattery();
  uint32_t now = micros();
  int32_t untilSend = (int32_t)(nextSendMicros - now);
  if (untilSend > 0) {
    // Battery: idle instead of busy-spinning between 2 ms sends. delay(1)
    // parks the CPU in the FreeRTOS idle task (WFI). Only when >1.2 ms
    // remains, so the 1 ms tick can never overshoot the send slot.
    if (untilSend > 1200) delay(1);
    return;
  }
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

  esp_now_send(broadcastMAC, (uint8_t *)&packet, sizeof(packet));

  if (millis() - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS) {
    receiverReady = false;
    waitForReceiver();
    nextSendMicros = micros();
  }
}
