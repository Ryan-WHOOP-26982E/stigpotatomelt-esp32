#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SDA_PIN 5
#define SCL_PIN 6
#define H3_ADDR 0x19
#define ACCEL_RADIUS_CM 2.738f

static const uint8_t WHO_AM_I   = 0x0F;
static const uint8_t CTRL_REG1  = 0x20;
static const uint8_t CTRL_REG4  = 0x23;
static const uint8_t OUT_X_L    = 0x28;

float z_bias_counts = 0.0f;
float z_filt_g = 0.0f;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(H3_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(H3_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((uint8_t)H3_ADDR, (uint8_t)1);
  if (n != 1) return false;
  val = Wire.read();
  return true;
}

bool readRegs(uint8_t startReg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(H3_ADDR);
  Wire.write(startReg | 0x80);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom((uint8_t)H3_ADDR, len);
  if (n != len) return false;

  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool readAxesRaw(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t d[6];
  if (!readRegs(OUT_X_L, d, 6)) return false;

  x = (int16_t)((d[1] << 8) | d[0]);
  y = (int16_t)((d[3] << 8) | d[2]);
  z = (int16_t)((d[5] << 8) | d[4]);

  x >>= 4;
  y >>= 4;
  z >>= 4;
  return true;
}

float countsToG(int16_t counts) {
  return counts * 0.195f;   // ±400 g mode
}

float rpmFromG(float g, float radius_cm) {
  float a_cm_s2 = fabsf(g) * 980.665f;
  if (radius_cm <= 0.0f) return 0.0f;

  float omega = sqrtf(a_cm_s2 / radius_cm);
  const float PI_F = 3.14159265358979323846f;
  return omega * 60.0f / (2.0f * PI_F);
}

bool calibrateZBias(int samples) {
  long sumZ = 0;
  int good = 0;

  for (int i = 0; i < samples; i++) {
    int16_t x, y, z;
    if (readAxesRaw(x, y, z)) {
      sumZ += z;
      good++;
    }
    delay(5);
  }

  if (good == 0) return false;
  z_bias_counts = (float)sumZ / good;
  return true;
}

void setupSensor() {
  uint8_t who = 0;
  readReg(WHO_AM_I, who);
  Serial.printf("WHO_AM_I  = 0x%02X (expected 0x32)\n", who);

  // 400 Hz, XYZ enabled, normal mode
  writeReg(CTRL_REG1, 0x37);

  // BDU=1, FS=11 => ±400 g
  writeReg(CTRL_REG4, 0xB0);

  uint8_t r1 = 0, r4 = 0;
  readReg(CTRL_REG1, r1);
  readReg(CTRL_REG4, r4);

  Serial.printf("CTRL_REG1 = 0x%02X\n", r1);
  Serial.printf("CTRL_REG4 = 0x%02X\n", r4);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

  Serial.println("\n=== H3LIS331DL Z-axis RPM test ===");
  setupSensor();

  Serial.println("Hold robot still for Z-bias calibration...");
  delay(1000);

  if (calibrateZBias(200)) {
    Serial.printf("Z bias = %.2f counts\n", z_bias_counts);
  } else {
    Serial.println("Calibration failed");
  }

  Serial.println(" rawZ | z_g_corr | rpm");
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 50) return;   // 20 Hz print
  lastPrint = millis();

  int16_t x, y, z;
  if (!readAxesRaw(x, y, z)) {
    Serial.println("Read failed");
    return;
  }

  float z_corr_counts = z - z_bias_counts;
  float z_g = z_corr_counts * 0.195f;

  // simple low-pass filter for stability
  z_filt_g = 0.8f * z_filt_g + 0.2f * z_g;

  // deadband to suppress fake RPM at rest
  if (fabsf(z_filt_g) < 0.30f) z_filt_g = 0.0f;

  float rpm = rpmFromG(z_filt_g, ACCEL_RADIUS_CM);

  Serial.printf("%5d | %+7.3f | %7.1f\n", z, z_filt_g, rpm);
}