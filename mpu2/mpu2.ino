#include <Wire.h>

#define MPU_ADDR 0x68

// Gyro scale: default ±250 dps -> 131 LSB/(°/s)
const float GYRO_SENS = 131.0f;
// Accel scale: default ±2g -> 16384 LSB/g
const float ACC_SENS  = 16384.0f;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float pitch = 0, roll = 0;     // filtered angles (deg)
unsigned long lastMs = 0;

static inline void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static inline void readBurst(uint8_t startReg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, count, true);
  for (uint8_t i = 0; i < count; i++) buf[i] = Wire.read();
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // Weak internal pull-ups (since you have no resistors)
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);

  Wire.begin(21, 22);
  Wire.setClock(50000); // slow & stable (try 100k if stable)

  // --- Minimal init ---
  writeReg(0x6B, 0x00); // PWR_MGMT_1: wake up
  writeReg(0x1A, 0x03); // CONFIG: DLPF=3 (~44Hz accel/42Hz gyro)
  // leave default ranges: ±2g, ±250dps

  // --- Quick gyro bias calibration (keep sensor still) ---
  const int N = 500;
  long gxSum = 0, gySum = 0, gzSum = 0;
  for (int i = 0; i < N; i++) {
    uint8_t d[14];
    readBurst(0x3B, 14, d);
    int16_t gx = (int16_t)(d[8]  << 8 | d[9]);
    int16_t gy = (int16_t)(d[10] << 8 | d[11]);
    int16_t gz = (int16_t)(d[12] << 8 | d[13]);
    gxSum += gx; gySum += gy; gzSum += gz;
    delay(2);
  }
  gyroBiasX = gxSum / (float)N;
  gyroBiasY = gySum / (float)N;
  gyroBiasZ = gzSum / (float)N;

  // Seed angles from accel
  uint8_t d[14];
  readBurst(0x3B, 14, d);
  int16_t ax = (int16_t)(d[0] << 8 | d[1]);
  int16_t ay = (int16_t)(d[2] << 8 | d[3]);
  int16_t az = (int16_t)(d[4] << 8 | d[5]);
  float axg = ax / ACC_SENS, ayg = ay / ACC_SENS, azg = az / ACC_SENS;

  // Accel-only angles (deg)
  float pitch_acc = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;      // rotation about XZ plane
  float roll_acc  = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;     // rotation about YZ plane
  pitch = pitch_acc;
  roll  = roll_acc;

  lastMs = millis();
  Serial.println("MPU6050: complementary filter running ✅");
}

void loop() {
  // Read 14 bytes starting at ACCEL_XOUT_H
  uint8_t d[14];
  readBurst(0x3B, 14, d);

  int16_t ax = (int16_t)(d[0] << 8 | d[1]);
  int16_t ay = (int16_t)(d[2] << 8 | d[3]);
  int16_t az = (int16_t)(d[4] << 8 | d[5]);
  int16_t gx = (int16_t)(d[8] << 8 | d[9]);
  int16_t gy = (int16_t)(d[10] << 8 | d[11]);
  int16_t gz = (int16_t)(d[12] << 8 | d[13]);

  // Convert
  float axg = ax / ACC_SENS;
  float ayg = ay / ACC_SENS;
  float azg = az / ACC_SENS;

  float gxdps = (gx - gyroBiasX) / GYRO_SENS; // °/s
  float gydps = (gy - gyroBiasY) / GYRO_SENS;
  // gz unused for pitch/roll

  // dt (seconds)
  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  // Accel-only angles (deg)
  float pitch_acc = atan2f(ayg, sqrtf(axg*axg + azg*azg)) * 180.0f / PI;
  float roll_acc  = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;

  // Integrate gyro (note: convention -> pitch from GyY, roll from GyX)
  float pitch_gyro = pitch + gydps * dt;
  float roll_gyro  = roll  + gxdps * dt;

  // Complementary filter
  const float alpha = 0.98f; // 98% gyro, 2% accel
  pitch = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
  roll  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;

  // Output
  Serial.print("Pitch: "); Serial.print(pitch, 2);
  Serial.print("  Roll: "); Serial.print(roll, 2);
  Serial.print("  |  ax,ay,az: ");
  Serial.print(axg, 2); Serial.print(", ");
  Serial.print(ayg, 2); Serial.print(", ");
  Serial.print(azg, 2);
  Serial.println();

  delay(10); // ~100 Hz loop
}
