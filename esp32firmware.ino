/* ---------------------------------------------------------
   ESP32-C3 | 4 FSR + MPU6050 CSV @ 10 Hz (final)
   Wiring
     FSR Heel  -> ADC GPIO0
     FSR Arch  -> ADC GPIO1
     FSR Fore  -> ADC GPIO2
     FSR Toe   -> ADC GPIO3
     MPU6050   -> 3V3, GND, SDA=GPIO8, SCL=GPIO9, AD0=GND, INT(optional)=GPIO7

   Output (one line per sample):
     ts_ms,heel,arch,fore,toe,pitch_deg,roll_deg,yaw_deg

   Serial commands:
     'z' or 'Z'  -> re-zero FSR baselines (no load on sensors)
   --------------------------------------------------------- */

#include <Arduino.h>
#include <Wire.h>

/* ---------- FSR config ---------- */
const uint8_t PIN_HEEL = 0;   // ADC1
const uint8_t PIN_ARCH = 1;   // ADC1
const uint8_t PIN_FORE = 2;   // ADC1
const uint8_t PIN_TOE  = 3;   // ADC1

const uint8_t  ADC_BITS   = 12;      // 0..4095
const uint8_t  OVERSAMPLE = 8;       // average N reads each cycle
const float    EMA_ALPHA  = 0.25f;   // 0..1 (higher=faster, lower=smoother)
const uint8_t  SAMPLE_HZ  = 10;      // output rate
const uint32_t DT_MS      = 1000UL / SAMPLE_HZ;

struct FSRSensor {
  uint8_t  pin;
  uint16_t baseline = 0;  // zero offset
  float    ema      = 0;  // smoothed value
};
FSRSensor sHeel{PIN_HEEL}, sArch{PIN_ARCH}, sFore{PIN_FORE}, sToe{PIN_TOE};

static inline uint16_t clampU16(int32_t v) {
  if (v < 0) return 0;
  if (v > 4095) return 4095;
  return (uint16_t)v;
}
uint16_t readADC_oversample(uint8_t pin) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < OVERSAMPLE; i++) acc += analogRead(pin);
  return (uint16_t)(acc / OVERSAMPLE);
}
void zeroBaselines() {
  sHeel.baseline = readADC_oversample(sHeel.pin);
  sArch.baseline = readADC_oversample(sArch.pin);
  sFore.baseline = readADC_oversample(sFore.pin);
  sToe .baseline = readADC_oversample(sToe .pin);
}

/* ---------- MPU6050 (I2C) ---------- */
#define SDA_PIN 8
#define SCL_PIN 9
const uint8_t MPU_ADDR = 0x68;     // AD0=GND
bool mpuOK = false;

bool mpuBegin() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  // Wake up: PWR_MGMT_1 = 0
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  return true;
}

void mpuReadRaw(int16_t& ax,int16_t& ay,int16_t& az,int16_t& gx,int16_t& gy,int16_t& gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                         // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  ax = (Wire.read()<<8) | Wire.read();
  ay = (Wire.read()<<8) | Wire.read();
  az = (Wire.read()<<8) | Wire.read();
  Wire.read(); Wire.read();                 // temp (ignored)
  gx = (Wire.read()<<8) | Wire.read();
  gy = (Wire.read()<<8) | Wire.read();
  gz = (Wire.read()<<8) | Wire.read();
}

/* ---------- setup / loop ---------- */
void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(ADC_BITS);
  // If your signals are large and clip, uncomment per-pin attenuation:
  // analogSetPinAttenuation(PIN_HEEL, ADC_11db);
  // analogSetPinAttenuation(PIN_ARCH, ADC_11db);
  // analogSetPinAttenuation(PIN_FORE, ADC_11db);
  // analogSetPinAttenuation(PIN_TOE,  ADC_11db);

  zeroBaselines();
  // prime EMA with one window
  sHeel.ema = readADC_oversample(PIN_HEEL);
  sArch.ema = readADC_oversample(PIN_ARCH);
  sFore.ema = readADC_oversample(PIN_FORE);
  sToe .ema = readADC_oversample(PIN_TOE);

  // I2C + MPU
  Wire.begin(SDA_PIN, SCL_PIN);
  mpuOK = mpuBegin();
  if (!mpuOK) {
    Serial.println(F("# WARNING: MPU6050 not found on I2C (expect addr 0x68). Check SDA=8 SCL=9 and AD0=GND."));
  }

  // Optional header:
  // Serial.println(F("ts_ms,heel,arch,fore,toe,pitch,roll,yaw"));
}

void loop() {
  // Command: re-zero baselines
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'z' || c == 'Z') {
      zeroBaselines();
      sHeel.ema = readADC_oversample(PIN_HEEL);
      sArch.ema = readADC_oversample(PIN_ARCH);
      sFore.ema = readADC_oversample(PIN_FORE);
      sToe .ema = readADC_oversample(PIN_TOE);
      Serial.println(F("# baselines re-zeroed"));
    }
  }

  static uint32_t nextTick = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick += DT_MS;

  /* ----- FSRs ----- */
  uint16_t rH = readADC_oversample(sHeel.pin);
  uint16_t rA = readADC_oversample(sArch.pin);
  uint16_t rF = readADC_oversample(sFore.pin);
  uint16_t rT = readADC_oversample(sToe .pin);

  uint16_t h = clampU16((int32_t)rH - sHeel.baseline);
  uint16_t a = clampU16((int32_t)rA - sArch.baseline);
  uint16_t f = clampU16((int32_t)rF - sFore.baseline);
  uint16_t t = clampU16((int32_t)rT - sToe .baseline);

  sHeel.ema = (1.0f - EMA_ALPHA) * sHeel.ema + EMA_ALPHA * h;
  sArch.ema = (1.0f - EMA_ALPHA) * sArch.ema + EMA_ALPHA * a;
  sFore.ema = (1.0f - EMA_ALPHA) * sFore.ema + EMA_ALPHA * f;
  sToe .ema = (1.0f - EMA_ALPHA) * sToe .ema + EMA_ALPHA * t;

  /* ----- MPU (tilt from accelerometer only) ----- */
  float pitchDeg = 0, rollDeg = 0, yawDeg = 0; // yaw not computed here
  if (mpuOK) {
    int16_t ax, ay, az, gx, gy, gz;
    mpuReadRaw(ax, ay, az, gx, gy, gz);

    // Convert to 'g' for accel (±2g full scale)
    const float accScale = 16384.0f;
    float axg = ax / accScale;
    float ayg = ay / accScale;
    float azg = az / accScale;

    // Roll (left/right) & Pitch (toe-up/down) in degrees
    rollDeg  = atan2f(ayg, azg) * 57.29578f;
    pitchDeg = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 57.29578f;
    // yawDeg remains 0.0 without gyro+mag fusion
  }

  /* ----- CSV output ----- */
  Serial.print(now); Serial.print(",");
  Serial.print((uint16_t)(sHeel.ema)); Serial.print(",");
  Serial.print((uint16_t)(sArch.ema)); Serial.print(",");
  Serial.print((uint16_t)(sFore.ema)); Serial.print(",");
  Serial.print((uint16_t)(sToe.ema));  Serial.print(",");
  Serial.print(pitchDeg, 2); Serial.print(",");
  Serial.print(rollDeg,  2); Serial.print(",");
  Serial.println(yawDeg,  2);
}