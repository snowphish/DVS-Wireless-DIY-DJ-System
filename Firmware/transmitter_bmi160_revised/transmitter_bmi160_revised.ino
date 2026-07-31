/*
  ESP32-C3 DVS TRANSMITTER + BMI160/BMI120  [TEST]
  - Same firmware as the MPU6050 puck (battery monitor, EVT relay, boot
    diagnostics, handshake ACK counters, spin-cal learn-once) with the gyro
    driver swapped for a Bosch BMI160 or register-compatible BMI120.
  - Wiring per the transmitter schematic:
      BMI160 SDA -> GPIO8, SCL -> GPIO9, CS -> 3V3 (I2C mode),
      SAO -> GND (address 0x68). INT1 -> GPIO4 carries data-ready.
      Battery: BAT+ (after switch) -> 100k -> GPIO3 -> 100k -> GND, 100 nF.
      Battery feeds the C3's 5V/VIN pin (never 3V3).
  - IMU is mounted upside-down on the PCB, so the Z axis is inverted vs the
    MPU6050 puck. rawGyroToNominalRPM has no negation to compensate.
  - Reads platter rotation from the gyro Z axis, sends RPM ~500x/sec over
    ESP-NOW (broadcast). Auto-zeroes the gyro at boot (platter STOPPED).

  Serial at 115200 for boot/debug (any USB mode).
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

// ===== BMI160 / BMI120 (gyro) =========================================
#define SDA_PIN 8
#define SCL_PIN 9
#define INT1_PIN 4               // data-ready from the BMI160 (see DRDY below)
#define BMI160_ADDR 0x68         // SAO tied to GND (0x69 if tied high)

#define BMI160_REG_CHIP_ID    0x00
#define BMI160_CHIP_ID_VAL    0xD1
#define BMI120_CHIP_ID_VAL    0xD3
#define BMI160_REG_PMU_STATUS 0x03
#define BMI160_REG_GYR_Z_L    0x10   // Z low byte; BMI160 data is little-endian
#define BMI160_REG_STATUS     0x1B   // bit 6 = drdy_gyr
#define BMI160_STATUS_DRDY_GYR 0x40
#define BMI160_REG_GYR_CONF   0x42
#define BMI160_REG_GYR_RANGE  0x43
#define BMI160_REG_INT_EN1    0x51
#define BMI160_REG_INT_OUT_CTRL 0x53
#define BMI160_REG_INT_LATCH  0x54
#define BMI160_REG_INT_MAP1   0x56
#define BMI160_REG_CMD        0x7E
#define BMI160_CMD_SOFT_RESET 0xB6
#define BMI160_CMD_GYR_NORMAL 0x15   // gyro -> normal power mode

// ===== Data-ready sync (INT1) =========================================
// The gyro runs at 800 Hz while we send at 500 Hz. Reading on a free-running
// timer means each read lands anywhere inside the sensor's 1.25 ms update
// window, so the sample age jitters by up to a full ODR period. On a velocity
// signal that timing jitter IS velocity noise, and it is worst exactly during
// fast scratches (a 10 rpm/ms slew turns 1 ms of jitter into 10 rpm of error).
// Syncing the read to data-ready removes it.
//
// Bit positions below are taken from the BMI160 register map. Note that
// INT_MAP[1] puts INT1 in the HIGH nibble and INT2 in the low one, which is
// the opposite of the layout INT_OUT_CTRL uses - easy to get backwards.
//
//   INT_EN[1]     b7..b0: reserved | fwm_en | ffull_en | DRDY_EN | low_en |
//                         highg_z_en | highg_y_en | highg_x_en
//   INT_OUT_CTRL  b7..b0: int2_output_en | int2_od | int2_lvl | int2_edge |
//                         INT1_OUTPUT_EN | int1_od | INT1_LVL | int1_edge
//   INT_LATCH     b3..b0: int_latch (0 = non-latched)
//   INT_MAP[1]    b7..b0: INT1_DRDY | int1_fwm | int1_ffull | int1_pmu_trig |
//                         int2_drdy | int2_fwm | int2_ffull | int2_pmu_trig
//
// The bounded wait in waitGyroDataReady() still covers the case where the pin
// itself never arrives (broken/absent INT1 trace): the puck reverts to the
// previous free-running behaviour and reports EVT_DRDY_FALLBACK rather than
// stalling the 500 Hz stream.
#define BMI160_INT_EN1_DRDY   0x10   // INT_EN[1] bit 4 = int_drdy_en
#define BMI160_INT_OUT_INT1   0x0A   // INT1 output_en (b3) + active-high (b1),
                                     // push-pull (od=0), level-triggered
#define BMI160_INT_LATCH_NONE 0x00   // non-latched; pulses on each new sample
#define BMI160_INT_MAP1_DRDY_INT1 0x80  // INT_MAP[1] bit 7 = int1_drdy
#define DRDY_WAIT_TIMEOUT_US  1500UL // > one 800 Hz ODR period, << the 2 ms slot
// Consecutive timeouts before giving up. Kept small deliberately: while we are
// still waiting-and-timing-out the send rate sags (each slot pays the timeout),
// so a board where INT1 does not work should bail out in ~100 ms rather than
// limp for half a second.
#define DRDY_FALLBACK_MISSES  50

// Gyro full-scale range + matching sensitivity (keep these consistent):
//   0x00=+-2000dps/16.384  0x01=+-1000dps/32.768  0x02=+-500dps/65.536
//   0x03=+-250dps/131.072  0x04=+-125dps/262.144
#define GYR_RANGE_SEL    0x01     // +-1000 dps => ~5x nominal backspin headroom
#define GYRO_LSB_PER_DPS 32.768f  // 2^15 / 1000
// GYR_CONF: bwp=normal(0b10 -> bits[5:4]), odr=800 Hz(0x0B -> bits[3:0]).
// 800 Hz ODR, normal-mode filter ~= 230 Hz 3 dB -> low control latency,
// comparable to the MPU6050's 188 Hz DLPF.
#define GYR_CONF_VAL     0x2B

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
#define EVT_CAL_ARMED      9     // remote re-cal armed (value = target rpm)
// Added with the zero-cal / IMU-fault hardening. Older receivers print these
// through their generic "code %u value %.5f" branch, so a mixed pair still
// works - it just shows the raw code instead of a friendly string.
#define EVT_ZERO_BLOCKED  10     // zero-cal refused: platter turning (value = rpm)
#define EVT_ZERO_DONE     11     // zero-cal accepted (value = bias in dps)
#define EVT_IMU_FAULT     12     // IMU init/comm failure (value = fault code)
#define EVT_DRDY_FALLBACK 13     // INT1 data-ready unusable, free-running
#define EVT_ZERO_BIAS_MISMATCH 14 // stored-bias cross-check rejected (value = delta dps)

// IMU fault codes carried by EVT_IMU_FAULT.
#define IMU_FAULT_ID     1       // chip id / WHO_AM_I mismatch
#define IMU_FAULT_CONFIG 2       // a configuration write never acknowledged
#define IMU_FAULT_READ   3       // sustained read failures after a good init

// Receiver -> puck commands (unicast; rpmCenti = command, gyroRaw = arg).
// Old receivers never send these; old pucks ignore unknown msgTypes.
#define MSG_COMMAND 7
#define CMD_SPINCAL 1            // re-arm spin-cal; arg = target rpm x100

// ===== Battery monitor ================================================
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
// The quartz-locked platter is a precise speed reference: when armed from
// the receiver's settings portal (Calibrate button), a stable ~10 s reading
// at the commanded base speed is taken as exactly that speed and the scale
// trim is derived and saved to flash. Calibrate at 0% PITCH or it will
// learn the wrong trim. NO auto-arm at boot - portal only (re-learning per
// boot used to fold thermal bias drift into the trim and corrupt it).
#define SPIN_CAL_ENABLE 1
#define SPIN_CAL_FORGET 0         // flash once with 1 to erase a bad trim
#define SPIN_CAL_ARM_MS 120000UL  // how long a portal calibrate stays armed
#define SPIN_CAL_SETTLE_MS 4000UL // wait after reaching speed: the platter's
                                  // last ~0.1% of spin-up must not be sampled
#define SPIN_CAL_SAMPLES 5000     // ~10 s at 500 Hz
#define SPIN_CAL_STABLE_RPM 0.15f // max wobble around the running mean
#define SPIN_CAL_ACCEPT_PCT 5.0f  // acceptance window (full gyro tolerance)

// ===== Gyro zero calibration ==========================================
#define CAL_STABLE_SAMPLES 400
#define CAL_MIN_COMPARE 20
#define CAL_STABLE_DPS 2.0f       // allowed jitter while "stopped"
// ABSOLUTE gate. "Stable" is not the same as "stopped": a quartz-locked
// platter at constant speed is extremely steady, so the jitter test above
// happily accepts it and learns 200 dps (33 1/3 rpm) as the zero point. The
// puck then reads 0 while spinning and full REVERSE once the platter stops -
// silently, because everything looks plausible until you stop the platter.
// Real bias is small (BMI160 spec +-10 dps, typ +-3); a platter is 200 dps at
// 33 1/3 and 270 at 45. 30 dps separates them by nearly 7x.
#define CAL_MAX_BIAS_DPS 30.0f
// A previously stored good bias is a second net: bias drifts with temperature,
// it does not jump. Refuse a new zero that moved further than this from the
// stored one (and report it) rather than silently adopting it.
#define CAL_BIAS_DELTA_DPS 20.0f
#define CAL_BLOCKED_REPORT_MS 3000UL   // how often to re-shout "platter turning"

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
uint32_t lastHelloMillis = 0;

float    gyroOffsetZ = 0.0f;
float    filteredRPM = 0.0f;
uint32_t sequenceNumber = 0;
uint32_t nextSendMicros = 0;

Preferences prefs;
float scaleTrim = 1.0f;           // learned gyro scale correction
bool  spinCalLearned = false;
bool  spinCalArmed = false;       // armed ONLY via CMD_SPINCAL from the portal
float spinCalSum = 0.0f;
float spinCalTarget = 0.0f;
int   spinCalCount = 0;
uint32_t spinCalEnterMs = 0;      // when the reading entered the target window
uint32_t spinCalArmStartedMs = 0;  // elapsed-time check stays safe across millis() wrap
float spinCalForcedTarget = 0.0f;              // >0 = only lock to this rpm
volatile bool    cmdSpinCalPending  = false;   // set by the radio callback
volatile int16_t cmdSpinCalArgCenti = 0;

uint16_t battMilliVolts = 0;
uint8_t  battState = 0;           // 0 ok, 1 low, 2 critical
uint32_t lastBattCheckMillis = 0;
uint32_t lastBattReportMillis = 0;
uint32_t lastBattLowSendMillis = 0;

// IMU health. A dead IMU used to spin forever in setup() printing to a USB
// port that is unreachable once the puck is on the platter, so the deck simply
// never appeared. Now the puck still pairs and reports the fault instead.
uint8_t  imuFaultCode = 0;        // 0 = healthy, else IMU_FAULT_*
uint32_t lastImuFaultSendMillis = 0;
#define IMU_FAULT_RESEND_MS 5000
#define IMU_READ_FAIL_LIMIT 50     // 100 ms at the 500 Hz motion loop
uint16_t imuReadFailStreak = 0;

// INT1 data-ready sync state.
bool     drdyFallback = false;    // true once INT1 is judged unusable
uint16_t drdyMisses = 0;          // consecutive waits that timed out

// Last accepted zero-offset, mirrored in NVS so a fresh cal can be sanity
// checked against it (see CAL_BIAS_DELTA_DPS).
float    storedBiasRaw = 0.0f;
bool     haveStoredBias = false;

// ===== BMI160 low-level ===============================================
static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg); Wire.write(value);
  return Wire.endTransmission(true) == 0;
}
// Retried configuration write. Returns false instead of hanging forever so
// setup() can bring the radio up and report the fault to the receiver.
static bool requireRegisterWrite(uint8_t reg, uint8_t value, const char *name) {
  for (int tries = 0; tries < 5; tries++) {
    if (writeRegister(reg, value)) return true;
    delay(5);
  }
  Serial.printf("ERR bmi160 write %s (reg 0x%02X) - check I2C/wiring\n", name, reg);
  return false;
}
static bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t len) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BMI160_ADDR, len, true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buffer[i] = Wire.read();
  return true;
}
static bool readGyroZRaw(int16_t *gyroZ) {
  uint8_t b[2] = {};
  if (!readRegisters(BMI160_REG_GYR_Z_L, b, 2)) return false;
  *gyroZ = (int16_t)((b[1] << 8) | b[0]);   // little-endian: low byte first
  return true;
}

// Returns false and sets imuFaultCode instead of hanging, so a puck with a
// dead IMU still pairs and shows up as faulted rather than as "never arrived".
bool setupBMI160() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  // Active-high push-pull when connected. The pull-down makes an older puck
  // with no INT1 wire fail LOW deterministically so the bounded fallback can
  // engage instead of accepting a floating HIGH as "data ready".
  pinMode(INT1_PIN, INPUT_PULLDOWN);

  // Sanity-check the part is a BMI160 or its register-compatible, low-power
  // BMI120 variant (unlike the MPU, chip id is readable from suspend).
  uint8_t id = 0;
  for (int tries = 0; tries < 10; tries++) {
    if (readRegisters(BMI160_REG_CHIP_ID, &id, 1) &&
        (id == BMI160_CHIP_ID_VAL || id == BMI120_CHIP_ID_VAL)) break;
    delay(20);
  }
  if (id != BMI160_CHIP_ID_VAL && id != BMI120_CHIP_ID_VAL) {
    Serial.printf("ERR bmi16x_id (got 0x%02X, want 0x%02X or 0x%02X) - check I2C/wiring, CS->3V3, SAO->GND\n",
                  id, BMI160_CHIP_ID_VAL, BMI120_CHIP_ID_VAL);
    imuFaultCode = IMU_FAULT_ID;
    return false;
  }
  Serial.printf("IMU: %s detected (chip id 0x%02X)\n",
                id == BMI120_CHIP_ID_VAL ? "BMI120" : "BMI160", id);

  bool ok = requireRegisterWrite(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET, "soft reset");
  delay(100);

  // Gyro out of suspend into normal mode (startup can take up to ~80 ms).
  ok = requireRegisterWrite(BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL, "gyro normal mode") && ok;
  delay(100);

  // Confirm the gyro PMU actually reached normal mode (bits [3:2] == 0b01).
  uint8_t pmu = 0;
  for (int tries = 0; tries < 20; tries++) {
    if (readRegisters(BMI160_REG_PMU_STATUS, &pmu, 1) && ((pmu >> 2) & 0x3) == 0x1) break;
    delay(10);
  }
  if (((pmu >> 2) & 0x3) != 0x1) {
    Serial.printf("WARN bmi160 gyro PMU not normal (status=0x%02X)\n", pmu);
    ok = false;
  }

  ok = requireRegisterWrite(BMI160_REG_GYR_RANGE, GYR_RANGE_SEL, "gyro range") && ok; // +-1000 dps
  ok = requireRegisterWrite(BMI160_REG_GYR_CONF, GYR_CONF_VAL, "gyro config") && ok;  // 800 Hz, normal BW

  // Route gyro data-ready to INT1: non-latched, push-pull, active high. A
  // failure here is NOT fatal - the read path falls back to free-running.
  bool drdyOk = requireRegisterWrite(BMI160_REG_INT_OUT_CTRL, BMI160_INT_OUT_INT1, "int out ctrl");
  drdyOk = requireRegisterWrite(BMI160_REG_INT_LATCH, BMI160_INT_LATCH_NONE, "int latch") && drdyOk;
  drdyOk = requireRegisterWrite(BMI160_REG_INT_MAP1, BMI160_INT_MAP1_DRDY_INT1, "int map1") && drdyOk;
  drdyOk = requireRegisterWrite(BMI160_REG_INT_EN1, BMI160_INT_EN1_DRDY, "int en1") && drdyOk;
  if (!drdyOk) {
    Serial.println("WARN bmi160 data-ready setup failed - free-running sampling");
    drdyFallback = true;
  }
  delay(10);

  if (!ok) imuFaultCode = IMU_FAULT_CONFIG;
  return ok;
}

// Block until the BMI160 says at least one unread gyro sample is available.
// If INT1 is already high, consume that pending sample immediately; this
// avoids reading before data-ready without pretending to force a new edge.
// Bounded: if INT1 never asserts (mis-set map bit, missing PCB trace) the puck
// reverts to the previous free-running behaviour and says so once.
static void waitGyroDataReady() {
  if (drdyFallback) return;
  uint32_t start = micros();
  while (digitalRead(INT1_PIN) == LOW) {
    if (micros() - start >= DRDY_WAIT_TIMEOUT_US) {
      if (++drdyMisses >= DRDY_FALLBACK_MISSES) {
        drdyFallback = true;
        Serial.println("WARN INT1 data-ready never asserted - reverting to free-running sampling");
        sendEventMessage(EVT_DRDY_FALLBACK, 0.0f);
      }
      return;
    }
  }
  drdyMisses = 0;
}

// Wait for a genuinely STOPPED platter, average it -> gyro zero offset.
//
// Two independent gates, because either one alone is not enough:
//   1. ABSOLUTE magnitude - the reading must be small enough to be plausible
//      gyro bias. This is the one that matters: without it a steady spin sails
//      straight through the jitter test below and gets learned as zero.
//   2. Jitter around the running mean - rejects vibration and spin-up/down.
// A new offset is additionally cross-checked against the last accepted one,
// since bias drifts slowly with temperature and never jumps.
void autoCalibrateGyroZ() {
  const int32_t stableRawDelta = (int32_t)(CAL_STABLE_DPS * GYRO_LSB_PER_DPS);
  const int32_t maxBiasRaw     = (int32_t)(CAL_MAX_BIAS_DPS * GYRO_LSB_PER_DPS);
  int stableSamples = 0; int32_t sum = 0;
  uint32_t readFails = 0;
  uint32_t lastBlockedMs = 0;
  bool blockedAnnounced = false;

  while (stableSamples < CAL_STABLE_SAMPLES) {
    int16_t raw = 0;
    if (!readGyroZRaw(&raw)) {
      readFails++;
      if (readFails == 1)
        Serial.printf("gyro read FAILING during cal (%lu fails) - check I2C/wiring\n",
                      (unsigned long)readFails);
      if (readFails >= IMU_READ_FAIL_LIMIT) {
        imuFaultCode = IMU_FAULT_READ;
        gyroOffsetZ = haveStoredBias ? storedBiasRaw : 0.0f;
        Serial.println("ERR gyro read failed continuously during zero-cal - entering IMU fault mode");
        sendEventMessage(EVT_IMU_FAULT, (float)imuFaultCode);
        lastImuFaultSendMillis = millis();
        return;
      }
      delay(2);
      continue;
    }
    readFails = 0;

    // Gate 1: too big to be bias -> the platter is turning, however steady
    // the reading looks. Say so on the receiver instead of waiting silently.
    if (labs((long)raw) > maxBiasRaw) {
      stableSamples = 0; sum = 0;
      uint32_t now = millis();
      if (!blockedAnnounced || now - lastBlockedMs >= CAL_BLOCKED_REPORT_MS) {
        lastBlockedMs = now;
        blockedAnnounced = true;
        float dps = (float)raw / GYRO_LSB_PER_DPS;
        Serial.printf("zero-cal BLOCKED: platter is turning (%.1f dps = %.2f rpm) - stop it\n",
                      dps, dps / 6.0f);
        sendEventMessage(EVT_ZERO_BLOCKED, dps / 6.0f);
      }
      delay(20);
      continue;
    }

    // Gate 2: jitter around the running mean (vibration, spin-up ramp).
    if (stableSamples >= CAL_MIN_COMPARE) {
      int32_t mean = sum / stableSamples;
      if (abs((int32_t)raw - mean) > stableRawDelta) { stableSamples = 0; sum = 0; continue; }
    }
    sum += raw; stableSamples++;
    delay(2);
  }

  float candidate = (float)sum / (float)stableSamples;

  // The window passed sample-by-sample; confirm the mean itself is sane too.
  if (fabsf(candidate) > (float)maxBiasRaw) {
    Serial.printf("zero-cal REJECTED: mean %.1f dps is not plausible bias\n",
                  candidate / GYRO_LSB_PER_DPS);
    sendEventMessage(EVT_ZERO_BLOCKED, candidate / GYRO_LSB_PER_DPS / 6.0f);
    if (haveStoredBias) {
      gyroOffsetZ = storedBiasRaw;
      Serial.printf("zero-cal: keeping last known good bias %.1f\n", gyroOffsetZ);
      return;
    }
    gyroOffsetZ = 0.0f;
    return;
  }

  // Cross-check against the last accepted bias. Temperature moves it slowly;
  // a large jump means something is wrong, so keep the stored value.
  if (haveStoredBias) {
    float deltaDps = fabsf(candidate - storedBiasRaw) / GYRO_LSB_PER_DPS;
    if (deltaDps > CAL_BIAS_DELTA_DPS) {
      Serial.printf("zero-cal REJECTED: bias moved %.1f dps from stored %.1f - keeping stored\n",
                    deltaDps, storedBiasRaw / GYRO_LSB_PER_DPS);
      sendEventMessage(EVT_ZERO_BIAS_MISMATCH, deltaDps);
      gyroOffsetZ = storedBiasRaw;
      return;
    }
  }

  gyroOffsetZ = candidate;
  storedBiasRaw = candidate;
  haveStoredBias = true;
  prefs.putFloat("bias", candidate);
  Serial.printf("calibrated: offset=%.1f (%.2f dps)\n",
                gyroOffsetZ, gyroOffsetZ / GYRO_LSB_PER_DPS);
  sendEventMessage(EVT_ZERO_DONE, gyroOffsetZ / GYRO_LSB_PER_DPS);
}

// Bias-corrected RPM at the gyro's NOMINAL scale (before any trim).
static inline float rawGyroToNominalRPM(int16_t gyroRaw) {
  float corrected = (float)gyroRaw - gyroOffsetZ;
  float dps = corrected / GYRO_LSB_PER_DPS;
  return dps / 6.0f;   // no negation: upside-down mount already inverts Z
}

static inline float nominalToOutputRPM(float nominalRpm) {
  float rpm = nominalRpm * RPM_MULTIPLIER * scaleTrim;
  if (fabsf(rpm) < DEADZONE_RPM) rpm = 0.0f;
  return rpm;
}

// Learn the gyro scale from a quartz-locked platter. Accepts a stable
// ~10 s window near 33.33 or 45.00 RPM within the arming period, computes
// trim = target/measured, and persists it. Runs once (until FORGET).
void serviceSpinCal(float nominalRpm) {
#if SPIN_CAL_ENABLE
  if (!spinCalArmed) return;
  if (millis() - spinCalArmStartedMs >= SPIN_CAL_ARM_MS) {
    spinCalArmed = false;
    spinCalForcedTarget = 0.0f;
    Serial.println("spin-cal: timed out before a stable reference was found");
    return;
  }

  float trimmed = fabsf(nominalRpm) * RPM_MULTIPLIER * scaleTrim;
  float target = 0.0f;
  if (spinCalForcedTarget > 0.0f) {
    // Remote re-cal: only lock to the speed the receiver told us to expect.
    if (fabsf(trimmed - spinCalForcedTarget) < spinCalForcedTarget * SPIN_CAL_ACCEPT_PCT * 0.01f)
      target = spinCalForcedTarget;
  }
  else if (fabsf(trimmed - 33.3333f) < 33.3333f * SPIN_CAL_ACCEPT_PCT * 0.01f) target = 33.3333f;
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
  spinCalForcedTarget = 0.0f;
#else
  (void)nominalRpm;
#endif
}

// Receiver-triggered spin-cal re-arm (settings portal button). Resets the
// accumulators, restricts the lock window to the requested speed and
// restarts the arming clock. The calibration itself is the same
// non-blocking 10 s average as at first boot - keep the platter spinning
// at the base speed, 0% pitch.
void serviceCalCommand() {
  if (!cmdSpinCalPending) return;
  cmdSpinCalPending = false;
  spinCalForcedTarget = (cmdSpinCalArgCenti > 4000) ? 45.0f : 33.3333f;
  spinCalSum = 0.0f; spinCalCount = 0; spinCalTarget = 0.0f; spinCalEnterMs = 0;
  spinCalArmStartedMs = millis();
  spinCalArmed = true;
  Serial.printf("spin-cal: RE-ARMED by receiver, target %.2f rpm - spin at 0%% pitch\n",
                spinCalForcedTarget);
  sendEventMessage(EVT_CAL_ARMED, spinCalForcedTarget);
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
  else if (in.msgType == MSG_COMMAND && in.rpmCenti == CMD_SPINCAL) {
    // Just note it; loop() acts on it (keep the radio callback tiny).
    cmdSpinCalArgCenti = in.gyroRaw;
    cmdSpinCalPending = true;
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Max TX power (84 x 0.25 = 21 dBm, driver clamps to chip max ~20 dBm).
  // Bench-tested 8.5 / 15 / 20 dBm at range: RSSI and packet loss both
  // improved monotonically with power on these boards (the small-antenna
  // "overdrive" worry did not materialise), so run flat out.
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
  uint32_t lastReport = 0;
  while (!receiverReady) {
    uint32_t now = millis();
    if (now - lastHelloMillis >= HANDSHAKE_INTERVAL_MS) {
      sendControlMessage(MSG_HELLO);
      lastHelloMillis = now;
    }
    if (now - lastReport >= 2000) {
      lastReport = now;
      Serial.printf("handshake: HELLO acked=%lu no-ack=%lu (acked but no WELCOME = our RX side is deaf)\n",
                    (unsigned long)txAckCount, (unsigned long)txNoAckCount);
    }
    delay(10);
  }
}

// Reconnect without stopping the 500 Hz data stream. The old implementation
// blocked inside waitForReceiver() after a link timeout, making recovery add an
// avoidable discontinuity to the receiver's failover output.
void serviceReceiverLink() {
  uint32_t now = millis();
  if (receiverReady && now - lastReceiverReplyMillis > HANDSHAKE_TIMEOUT_MS) {
    receiverReady = false;
    assignedDeck = 0;
    Serial.println("link: receiver timed out; continuing DATA while re-handshaking");
  }
  if (!receiverReady && now - lastHelloMillis >= HANDSHAKE_INTERVAL_MS) {
    sendControlMessage(MSG_HELLO);
    lastHelloMillis = now;
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

  bool stateChanged = newState != battState;
  if (stateChanged) {
    battState = newState;
    if (battState == 0) {
      sendEventMessage(EVT_BATT_RECOVERED, volts);
      Serial.printf("battery recovered: %.2f V\n", volts);
    }
  }
  if (battState != 0 &&
      (stateChanged || now - lastBattLowSendMillis >= BATT_LOW_RESEND_MS)) {
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
  Serial.println("BOOT: dvs transmitter BMI160 (test build) alive");
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
    Serial.println("spin-cal: no stored trim - use the receiver portal's Calibrate button");
  }
  // Last accepted gyro zero, used to sanity-check this boot's calibration.
  float storedBias = prefs.getFloat("bias", NAN);
  if (isfinite(storedBias) &&
      fabsf(storedBias) <= CAL_MAX_BIAS_DPS * GYRO_LSB_PER_DPS) {
    storedBiasRaw = storedBias;
    haveStoredBias = true;
    Serial.printf("zero-cal: last known good bias %.1f (%.2f dps)\n",
                  storedBiasRaw, storedBiasRaw / GYRO_LSB_PER_DPS);
  }
  // Scale is learned ONCE, then locked (re-learning per boot folds thermal
  // bias drift into the scale trim and corrupts it). SPIN_CAL_FORGET redoes.
  // No auto-arm at boot: calibration is triggered on demand from the
  // receiver's settings portal (CMD_SPINCAL) - see serviceCalCommand().

  // Radio FIRST, so an IMU failure can be reported to the receiver. The puck's
  // own USB serial is unreachable once it is sitting on a platter, so a fault
  // raised before the link exists is a fault nobody ever sees.
  Serial.println("BOOT: espnow init...");
  setupEspNow();
  Serial.println("BOOT: espnow OK");

  Serial.println("BOOT: bmi160 init...");
  bool imuOk = setupBMI160();
  Serial.println(imuOk ? "BOOT: bmi160 OK" : "BOOT: bmi160 FAULT - continuing to report it");

  Serial.println("BOOT: waiting for receiver...");
  waitForReceiver();
  Serial.printf("BOOT: receiver linked, assigned deck %u\n", assignedDeck);
  // Link is up now - tell the receiver's serial our calibration state
  // (the puck's own serial is unreachable once it's on the platter).
  sendEventMessage(spinCalLearned ? EVT_TRIM_LOADED : EVT_TRIM_MISSING,
                   scaleTrim);
  battMilliVolts = readBatteryMilliVolts();
  sendEventMessage(EVT_BATT_VOLTAGE, battMilliVolts / 1000.0f);
  Serial.printf("battery: %.2f V\n", battMilliVolts / 1000.0f);

  if (!imuOk) {
    // Paired but useless: keep the link alive and keep saying why, so the
    // deck shows "IMU FAULT" in the manager rather than just never arriving.
    sendEventMessage(EVT_IMU_FAULT, (float)imuFaultCode);
    lastImuFaultSendMillis = millis();
    Serial.println("BOOT: halted on IMU fault (no DATA will be sent)");
    nextSendMicros = micros();
    return;
  }

  Serial.println("BOOT: gyro zero cal (platter stopped)...");
  autoCalibrateGyroZ();       // platter must be stopped at power-on
  if (imuFaultCode) {
    Serial.println("BOOT: zero-cal aborted on IMU fault (no DATA will be sent)");
    nextSendMicros = micros();
    return;
  }
  Serial.println("BOOT: running");
  nextSendMicros = micros();
}

void loop() {
  // A faulted IMU keeps the link and the fault report alive, nothing else.
  if (imuFaultCode) {
    serviceReceiverLink();
    serviceBattery();
    uint32_t nowMs = millis();
    if (nowMs - lastImuFaultSendMillis >= IMU_FAULT_RESEND_MS) {
      lastImuFaultSendMillis = nowMs;
      sendEventMessage(EVT_IMU_FAULT, (float)imuFaultCode);
    }
    delay(10);
    return;
  }

  serviceCalCommand();
  serviceBattery();
  serviceReceiverLink();
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

  // Align the read to the sensor's own 800 Hz update grid so the sample age
  // does not jitter across the ODR period (that jitter reads as velocity noise
  // during fast moves). Falls back to free-running if INT1 is unusable.
  waitGyroDataReady();

  int16_t gyroRaw = 0;
  if (!readGyroZRaw(&gyroRaw)) {
    if (imuReadFailStreak < IMU_READ_FAIL_LIMIT) imuReadFailStreak++;
    if (imuReadFailStreak >= IMU_READ_FAIL_LIMIT) {
      imuFaultCode = IMU_FAULT_READ;
      Serial.println("ERR gyro read failed continuously - entering IMU fault mode");
      sendEventMessage(EVT_IMU_FAULT, (float)imuFaultCode);
      lastImuFaultSendMillis = millis();
    }
    return;
  }
  imuReadFailStreak = 0;

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

}
