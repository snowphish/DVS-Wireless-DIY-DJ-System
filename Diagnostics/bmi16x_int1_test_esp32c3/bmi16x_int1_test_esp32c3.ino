// ESP32-C3 BMI160/BMI120 INT1 diagnostic
// Serial: 115200 baud
// Wiring: SDA=GPIO8, SCL=GPIO9, INT1=GPIO4, CS=3V3, SA0/SDO=GND

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t SDA_PIN  = 8;
constexpr uint8_t SCL_PIN  = 9;
constexpr uint8_t INT1_PIN = 4;

constexpr uint8_t REG_CHIP_ID     = 0x00;
constexpr uint8_t REG_ERR          = 0x02;
constexpr uint8_t REG_PMU          = 0x03;
constexpr uint8_t REG_GYR_Z_L      = 0x10;
constexpr uint8_t REG_STATUS       = 0x1B;
constexpr uint8_t REG_GYR_CONF     = 0x42;
constexpr uint8_t REG_GYR_RANGE    = 0x43;
constexpr uint8_t REG_INT_EN1      = 0x51;
constexpr uint8_t REG_INT_OUT_CTRL = 0x53;
constexpr uint8_t REG_INT_LATCH    = 0x54;
constexpr uint8_t REG_INT_MAP1     = 0x56;
constexpr uint8_t REG_CMD          = 0x7E;

constexpr uint8_t CMD_RESET        = 0xB6;
constexpr uint8_t CMD_GYR_NORMAL   = 0x15;

uint8_t sensorAddress = 0;
volatile uint32_t risingEdges = 0;

void IRAM_ATTR int1Rising() {
  risingEdges++;
}

bool readBytes(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(sensorAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(sensorAddress, length, true) != length) return false;
  for (uint8_t i = 0; i < length; i++) data[i] = Wire.read();
  return true;
}

bool readReg(uint8_t reg, uint8_t &value) {
  return readBytes(reg, &value, 1);
}

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(sensorAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool findSensor() {
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(addresses[i]);
    uint8_t result = Wire.endTransmission(true);
    Serial.printf("Probe 0x%02X: %s\n", addresses[i], result == 0 ? "ACK" : "no response");
    if (result == 0 && sensorAddress == 0) sensorAddress = addresses[i];
  }
  return sensorAddress != 0;
}

void showReg(const char *name, uint8_t reg, uint8_t expected = 0xFF) {
  uint8_t value = 0;
  if (!readReg(reg, value)) {
    Serial.printf("%-14s read FAILED\n", name);
    return;
  }
  Serial.printf("%-14s 0x%02X", name, value);
  if (expected != 0xFF) Serial.printf("  expected 0x%02X  %s", expected, value == expected ? "OK" : "MISMATCH");
  Serial.println();
}

bool configureSensor() {
  Serial.printf("Soft reset: %s\n", writeReg(REG_CMD, CMD_RESET) ? "ACK" : "FAILED");
  delay(250);

  uint8_t id = 0;
  if (!readReg(REG_CHIP_ID, id)) return false;
  Serial.printf("Chip ID: 0x%02X (%s)\n", id,
                id == 0xD1 ? "BMI160" : id == 0xD3 ? "BMI120" : "unexpected");

  Serial.printf("Gyro normal: %s\n", writeReg(REG_CMD, CMD_GYR_NORMAL) ? "ACK" : "FAILED");
  delay(250);

  // Same gyro and INT1 settings used by the transmitter firmware.
  bool ok = true;
  ok &= writeReg(REG_GYR_RANGE, 0x01);     // +/-1000 dps
  ok &= writeReg(REG_GYR_CONF, 0x2B);      // normal bandwidth, 800 Hz
  ok &= writeReg(REG_INT_OUT_CTRL, 0x0A); // INT1 enabled, push-pull, active high, level
  ok &= writeReg(REG_INT_LATCH, 0x00);    // non-latched
  ok &= writeReg(REG_INT_MAP1, 0x80);     // map data-ready to INT1
  ok &= writeReg(REG_INT_EN1, 0x10);      // enable data-ready interrupt
  delay(50);
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBMI160/BMI120 INT1 diagnostic");
  Serial.println("SDA=GPIO8 SCL=GPIO9 INT1=GPIO4 I2C=100kHz");

  pinMode(INT1_PIN, INPUT_PULLDOWN);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!findSensor()) {
    Serial.println("STOP: no sensor at 0x68 or 0x69");
    return;
  }
  Serial.printf("Using address 0x%02X\n", sensorAddress);

  if (!configureSensor()) {
    Serial.println("STOP: sensor setup failed");
    return;
  }

  Serial.println("\nRegister readback:");
  showReg("ERR_REG", REG_ERR);
  showReg("PMU_STATUS", REG_PMU);
  showReg("GYR_CONF", REG_GYR_CONF, 0x2B);
  showReg("GYR_RANGE", REG_GYR_RANGE, 0x01);
  showReg("INT_OUT_CTRL", REG_INT_OUT_CTRL, 0x0A);
  showReg("INT_LATCH", REG_INT_LATCH, 0x00);
  showReg("INT_MAP1", REG_INT_MAP1, 0x80);
  showReg("INT_EN1", REG_INT_EN1, 0x10);

  attachInterrupt(digitalPinToInterrupt(INT1_PIN), int1Rising, RISING);
  Serial.println("\nReporting once per second...");
}

void loop() {
  static uint32_t lastReport = millis();
  static uint32_t statusReadyCount = 0;
  static uint32_t gpioHighCount = 0;
  static uint32_t gyroReads = 0;
  static int16_t lastGyroZ = 0;

  if (sensorAddress == 0) {
    delay(1000);
    return;
  }

  uint8_t status = 0;
  if (readReg(REG_STATUS, status) && (status & 0x40)) statusReadyCount++;
  if (digitalRead(INT1_PIN) == HIGH) gpioHighCount++;

  // Reading gyro data clears the pending data-ready condition, allowing the
  // next transition to be observed on INT1.
  uint8_t raw[2];
  if (readBytes(REG_GYR_Z_L, raw, 2)) {
    lastGyroZ = (int16_t)((uint16_t(raw[1]) << 8) | raw[0]);
    gyroReads++;
  }

  if (millis() - lastReport >= 1000) {
    noInterrupts();
    uint32_t edges = risingEdges;
    risingEdges = 0;
    interrupts();

    uint8_t error = 0, pmu = 0;
    readReg(REG_ERR, error);
    readReg(REG_PMU, pmu);
    Serial.printf("INT1 edges=%lu, GPIO-high=%lu, STATUS-drdy=%lu, reads=%lu, Z=%d, PMU=0x%02X, ERR=0x%02X\n",
                  (unsigned long)edges, (unsigned long)gpioHighCount,
                  (unsigned long)statusReadyCount, (unsigned long)gyroReads,
                  lastGyroZ, pmu, error);

    if (statusReadyCount > 0 && edges == 0) {
      Serial.println("  DIAG: sensor makes data, but GPIO4 sees no INT1 edge -> interrupt path/output fault");
    } else if (edges > 0) {
      Serial.println("  DIAG: INT1 is working");
    } else {
      Serial.println("  DIAG: neither internal DRDY nor INT1 observed -> sensor/configuration problem");
    }

    statusReadyCount = 0;
    gpioHighCount = 0;
    gyroReads = 0;
    lastReport = millis();
  }

  delayMicroseconds(250);
}
