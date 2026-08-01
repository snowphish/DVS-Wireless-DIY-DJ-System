// ESP32-C3 BMI160/BMI120 startup diagnostic
// Wiring: SDA=GPIO8, SCL=GPIO9, CS=3V3, SA0/SDO=GND (0x68)

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;

constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_ERR      = 0x02;
constexpr uint8_t REG_PMU      = 0x03;
constexpr uint8_t REG_CMD      = 0x7E;
constexpr uint8_t CMD_RESET    = 0xB6;
constexpr uint8_t CMD_GYR_NORM = 0x15;

uint8_t sensorAddress = 0;

bool readReg(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, (uint8_t)1, (uint8_t)true) != 1) return false;
  value = Wire.read();
  return true;
}

bool writeReg(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

void printRegister(const char *name, uint8_t reg) {
  uint8_t value = 0;
  if (readReg(sensorAddress, reg, value)) {
    Serial.printf("  %s = 0x%02X\n", name, value);
  } else {
    Serial.printf("  %s read FAILED\n", name);
  }
}

bool findSensor() {
  for (uint8_t address : { (uint8_t)0x68, (uint8_t)0x69 }) {
    Wire.beginTransmission(address);
    uint8_t result = Wire.endTransmission(true);
    Serial.printf("Probe 0x%02X: %s (I2C result %u)\n",
                  address, result == 0 ? "ACK" : "no response", result);
    if (result == 0 && sensorAddress == 0) sensorAddress = address;
  }
  return sensorAddress != 0;
}

void runStartupTest(uint32_t resetDelayMs, uint32_t gyroDelayMs) {
  Serial.println();
  Serial.printf("=== reset wait %lu ms, gyro wait %lu ms ===\n",
                (unsigned long)resetDelayMs, (unsigned long)gyroDelayMs);

  Serial.printf("Soft reset write: %s\n",
                writeReg(sensorAddress, REG_CMD, CMD_RESET) ? "ACK" : "FAILED");
  delay(resetDelayMs);
  printRegister("CHIP_ID", REG_CHIP_ID);
  printRegister("ERR_REG before command", REG_ERR);
  printRegister("PMU before command", REG_PMU);

  Serial.printf("Gyro-normal command write: %s\n",
                writeReg(sensorAddress, REG_CMD, CMD_GYR_NORM) ? "ACK" : "FAILED");

  const uint32_t checkpoints[] = { 10, 50, 100, 200, 500, 1000 };
  uint32_t elapsed = 0;
  for (uint32_t checkpoint : checkpoints) {
    if (checkpoint > gyroDelayMs) break;
    delay(checkpoint - elapsed);
    elapsed = checkpoint;

    uint8_t pmu = 0, error = 0;
    bool pmuOk = readReg(sensorAddress, REG_PMU, pmu);
    bool errOk = readReg(sensorAddress, REG_ERR, error);
    Serial.printf("  after %4lu ms: PMU=%s0x%02X (gyro mode=%u), ERR=%s0x%02X\n",
                  (unsigned long)elapsed,
                  pmuOk ? "" : "READ_FAIL/", pmu,
                  (pmu >> 2) & 0x03,
                  errOk ? "" : "READ_FAIL/", error);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBMI160/BMI120 startup timing test");
  Serial.println("I2C: SDA=GPIO8, SCL=GPIO9, clock=100 kHz");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!findSensor()) {
    Serial.println("No device found at 0x68 or 0x69. Check power, CS, SDA and SCL.");
    return;
  }

  Serial.printf("Using I2C address 0x%02X\n", sensorAddress);
  printRegister("initial CHIP_ID", REG_CHIP_ID);

  runStartupTest(100, 1000);
  runStartupTest(250, 1000);
  runStartupTest(500, 1000);

  Serial.println("\nTest complete. Gyro mode 1 means normal; mode 0 means suspended.");
}

void loop() {
  delay(1000);
}
