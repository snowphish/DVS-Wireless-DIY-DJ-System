// NodeMCU v3 (ESP8266) BMI160/BMI120 startup diagnostic
//
// NodeMCU D2 (GPIO4) -> BMI SDA
// NodeMCU D1 (GPIO5) -> BMI SCL
// NodeMCU 3V3        -> BMI VCC and CS
// NodeMCU GND        -> BMI GND and SA0/SDO (selects address 0x68)
//
// Use 3.3 V only unless the breakout board explicitly includes regulation
// and level shifting. Open Serial Monitor at 115200 baud.

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t SDA_PIN = 4;  // NodeMCU D2
constexpr uint8_t SCL_PIN = 5;  // NodeMCU D1

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
  if (Wire.requestFrom(address, (uint8_t)1) != 1) return false;
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
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t i = 0; i < sizeof(addresses); i++) {
    const uint8_t address = addresses[i];
    Wire.beginTransmission(address);
    const uint8_t result = Wire.endTransmission(true);
    Serial.printf("Probe 0x%02X: %s (I2C result %u)\n",
                  address, result == 0 ? "ACK" : "no response", result);
    if (result == 0 && sensorAddress == 0) sensorAddress = address;
  }
  return sensorAddress != 0;
}

void runStartupTest(uint32_t resetDelayMs) {
  Serial.println();
  Serial.printf("=== reset wait %lu ms ===\n", (unsigned long)resetDelayMs);

  Serial.printf("Soft reset write: %s\n",
                writeReg(sensorAddress, REG_CMD, CMD_RESET) ? "ACK" : "FAILED");
  delay(resetDelayMs);
  printRegister("CHIP_ID", REG_CHIP_ID);
  printRegister("ERR_REG before command", REG_ERR);
  printRegister("PMU before command", REG_PMU);

  Serial.printf("Gyro-normal command write: %s\n",
                writeReg(sensorAddress, REG_CMD, CMD_GYR_NORM) ? "ACK" : "FAILED");

  const uint16_t checkpoints[] = {10, 50, 100, 200, 500, 1000};
  uint16_t elapsed = 0;
  for (uint8_t i = 0; i < sizeof(checkpoints) / sizeof(checkpoints[0]); i++) {
    const uint16_t checkpoint = checkpoints[i];
    delay(checkpoint - elapsed);
    elapsed = checkpoint;

    uint8_t pmu = 0;
    uint8_t error = 0;
    const bool pmuOk = readReg(sensorAddress, REG_PMU, pmu);
    const bool errOk = readReg(sensorAddress, REG_ERR, error);
    Serial.printf("  after %4u ms: PMU=%s0x%02X (gyro mode=%u), ERR=%s0x%02X\n",
                  elapsed,
                  pmuOk ? "" : "READ_FAIL/", pmu,
                  (pmu >> 2) & 0x03,
                  errOk ? "" : "READ_FAIL/", error);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("NodeMCU ESP8266 BMI160/BMI120 startup test");
  Serial.println("I2C: SDA=D2/GPIO4, SCL=D1/GPIO5, clock=100 kHz");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!findSensor()) {
    Serial.println("No device at 0x68 or 0x69. Check power, CS, SDA and SCL.");
    return;
  }

  Serial.printf("Using I2C address 0x%02X\n", sensorAddress);
  printRegister("initial CHIP_ID", REG_CHIP_ID);

  runStartupTest(100);
  runStartupTest(250);
  runStartupTest(500);

  Serial.println();
  Serial.println("Test complete.");
  Serial.println("Expected working result: PMU gyro mode=1 (normally PMU 0x04). ");
  Serial.println("PMU mode=0 plus ERR bit 0 set indicates an internal fatal error.");
}

void loop() {
  delay(1000);
}
