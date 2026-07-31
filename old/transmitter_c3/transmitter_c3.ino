/*
  ESP32-C3 DVS TRANSMITTER + MPU6050
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
#define DECK_ID 1                // second unit: set to 2

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
// OFDM 24 Mbps cuts airtime ~6x. Only affects our TX side; any ESP32
// receives it fine. Set to 0 to fall back to the stock rate.
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
#define SPIN_CAL_ACCEPT_PCT 5.0f  // acceptance window (full gyro tolerance)

// ===== Calibration ====================================================
#define CAL_STABLE_SAMPLES 400
#define CAL_MIN_COMPARE 20
#define CAL_STABLE_DPS 2.0f       // allowed jitter while "stopped"

// frank receiver: uint8_t receiverMAC[]  = { 0x84, 0xFC, 0xE6, 0x5F, 0x41, 0x10 };
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
float scaleTrim = 1.0f;           // learned gyro scale correction
bool  spinCalLearned = false;
bool  spinCalArmed = true;
float spinCalSum = 0.0f;
float spinCalTarget = 0.0f;
int   spinCalCount = 0;
uint32_t spinCalEnterMs = 0;      // when the reading entered the target window

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
  while (stableSamples < CAL_STABLE_SAMPLES) {
    int16_t raw = 0;
    if (!readGyroZRaw(&raw)) continue;
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
  float target = 0.0f;
  if      (fabsf(trimmed - 33.3333f) < 33.3333f * SPIN_CAL_ACCEPT_PCT * 0.01f) target = 33.3333f;
  else if (fabsf(trimmed - 45.0f)    < 45.0f    * SPIN_CAL_ACCEPT_PCT * 0.01f) target = 45.0f;

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
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {}

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

// Status event -> receiver serial. rpmCenti carries the event code,
// timestampMicros the value as fixed-point x1e6 (e.g. trim 0.99810).
void sendEventMessage(uint8_t code, float value) {
  dvs_packet e = {};
  e.msgType = MSG_EVENT; e.version = PROTOCOL_VERSION; e.deckId = DECK_ID;
  e.rpmCenti = code;
  e.timestampMicros = (uint32_t)lroundf(value * 1000000.0f);
  esp_now_send(receiverMAC, (uint8_t *)&e, sizeof(e));
}

void waitForReceiver() {
  uint32_t lastHello = 0;
  while (!receiverReady) {
    uint32_t now = millis();
    if (now - lastHello >= HANDSHAKE_INTERVAL_MS) { sendControlMessage(MSG_HELLO); lastHello = now; }
    delay(10);
  }
}

// ===== setup / loop ===================================================
void setup() {
  Serial.begin(115200);

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
  // Scale is a temperature-stable per-chip constant: learn it ONCE, then
  // lock it. Re-arming on later boots let each boot's thermal bias drift
  // get folded into the trim (scale and bias are inseparable in a single
  // spin measurement), slowly corrupting it. SPIN_CAL_FORGET re-learns.
  spinCalArmed = !spinCalLearned;

  setupMPU6050();
  setupEspNow();
  waitForReceiver();
  // Link is up now - tell the receiver's serial our calibration state
  // (the puck's own serial is unreachable once it's on the platter).
  sendEventMessage(spinCalLearned ? EVT_TRIM_LOADED : EVT_TRIM_MISSING,
                   scaleTrim);
  autoCalibrateGyroZ();       // platter must be stopped at power-on
  nextSendMicros = micros();
}

void loop() {
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
