#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#define SDA_PIN 4
#define SCL_PIN 5

// ── MPU6050 ───────────────────────────────────────────────
const uint8_t MPU = 0x68;

// Motion threshold — tune this (units: raw accel LSB)
// 16384 = 1g. A value of ~800 catches light wrist movement.
#define MOTION_THRESHOLD  800
#define NO_MOTION_TIMEOUT 5000   // ms of no motion → sleep MAX30102

// ── MAX30102 ──────────────────────────────────────────────
MAX30105 particleSensor;

const byte RATE_SIZE = 8;
byte   rates[RATE_SIZE];
byte   rateSpot   = 0;
long   lastBeat   = 0;
float  beatsPerMinute;
int    beatAvg;

const byte SPO2_SIZE = 8;
byte spo2Readings[SPO2_SIZE];
byte spo2Spot = 0;
int  spo2Avg;

#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t  spo2;
int8_t   validSPO2;
int32_t  heartRate;
int8_t   validHeartRate;

unsigned long lastSpo2Calc = 0;
#define SPO2_INTERVAL 2000

// ── State Machine ─────────────────────────────────────────
enum SensorState { IDLE, FINGER_CHECK, ACTIVE };
SensorState state = IDLE;

unsigned long lastMotionTime = 0;
bool maxAwake = false;

// ─────────────────────────────────────────────────────────
// MPU6050: put into low-power cycle mode
// Wakes briefly every ~40ms, checks accelerometer, sleeps again.
// LP_WAKE_CTRL = 0x40 → 25Hz wake frequency
// CYCLE bit (bit5 of PWR_MGMT_1) enables this mode.
// ─────────────────────────────────────────────────────────
void initMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (size_t)1, true);
  uint8_t whoami = Wire.read();
  Serial.print("MPU6050 WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  // Full reset first
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x80);   // DEVICE_RESET
  Wire.endTransmission(true);
  delay(200);

  // Wake up normally first, configure, then enable low-power cycle
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);   // Clear sleep/cycle bits
  Wire.endTransmission(true);
  delay(100);

  // Set LP_WAKE_CTRL to 01 (25Hz) in PWR_MGMT_2 (register 0x6C)
  // Also disable gyro axes to save more power (bits 0-2 = 111)
  Wire.beginTransmission(MPU);
  Wire.write(0x6C);
  Wire.write(0b01000111);  // 25Hz wake | disable Gx Gy Gz
  Wire.endTransmission(true);

  // Now enable CYCLE mode in PWR_MGMT_1
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x20);   // CYCLE bit only — no sleep, no reset
  Wire.endTransmission(true);

  Serial.println("MPU6050 in low-power cycle mode (25Hz accel only)");
}

// ─────────────────────────────────────────────────────────
// MPU6050: temporarily exit cycle mode for a full reading
// ─────────────────────────────────────────────────────────
void mpuWakeForRead() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);   // Normal mode
  Wire.endTransmission(true);
  delay(10);          // Let it stabilize
}

void mpuReturnToLowPower() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x20);   // Back to CYCLE mode
  Wire.endTransmission(true);
}

// ─────────────────────────────────────────────────────────
// Returns true if acceleration magnitude exceeds threshold
// compared to baseline (resting = 1g on one axis)
// ─────────────────────────────────────────────────────────
bool motionDetected() {
  mpuWakeForRead();

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (size_t)6, true);  // Only accel, skip temp+gyro

  int16_t AcX = (Wire.read() << 8) | Wire.read();
  int16_t AcY = (Wire.read() << 8) | Wire.read();
  int16_t AcZ = (Wire.read() << 8) | Wire.read();

  mpuReturnToLowPower();

  // Magnitude of acceleration vector
  // At rest: √(0² + 0² + 16384²) ≈ 16384
  // We look for deviation from this resting vector
  long mag = sqrt((long)AcX*AcX + (long)AcY*AcY + (long)AcZ*AcZ);
  long deviation = abs(mag - 16384L);

  return deviation > MOTION_THRESHOLD;
}

// ─────────────────────────────────────────────────────────
// Full MPU6050 read for ACTIVE state printing
// ─────────────────────────────────────────────────────────
void readAndPrintMPU6050() {
  mpuWakeForRead();

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (size_t)14, true);

  int16_t AcX = (Wire.read() << 8) | Wire.read();
  int16_t AcY = (Wire.read() << 8) | Wire.read();
  int16_t AcZ = (Wire.read() << 8) | Wire.read();
  int16_t Tmp = (Wire.read() << 8) | Wire.read();
  int16_t GyX = (Wire.read() << 8) | Wire.read();
  int16_t GyY = (Wire.read() << 8) | Wire.read();
  int16_t GyZ = (Wire.read() << 8) | Wire.read();

  mpuReturnToLowPower();

  float ax   = AcX / 16384.0;
  float ay   = AcY / 16384.0;
  float az   = AcZ / 16384.0;
  float gx   = GyX / 131.0;
  float gy   = GyY / 131.0;
  float gz   = GyZ / 131.0;
  float temp = (Tmp / 340.0) + 36.53;

  Serial.println("── MPU6050 ──────────────────────────────");
  Serial.print("Ac: ");
  Serial.print(ax, 2); Serial.print("g | ");
  Serial.print(ay, 2); Serial.print("g | ");
  Serial.print(az, 2); Serial.print("g  ||  Gy: ");
  Serial.print(gx, 1); Serial.print("°/s | ");
  Serial.print(gy, 1); Serial.print("°/s | ");
  Serial.print(gz, 1); Serial.print("°/s  ||  Temp: ");
  Serial.print(temp, 1); Serial.println("°C");
}

// ─────────────────────────────────────────────────────────
// MAX30102 power control
// ─────────────────────────────────────────────────────────
void initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
    while (1);
  }
  wakeMAX30102();
}

void wakeMAX30102() {
  if (maxAwake) return;

  particleSensor.wakeUp();
  particleSensor.setup(60, 4, 2, 400, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  // Prime the buffer
  for (byte i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available())
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate
  );

  // Reset averages for fresh session
  rateSpot = 0; spo2Spot = 0;
  beatAvg  = 0; spo2Avg  = 0;
  memset(rates, 0, sizeof(rates));
  memset(spo2Readings, 0, sizeof(spo2Readings));

  maxAwake = true;
  Serial.println("[MAX30102] Woken up");
}

void sleepMAX30102() {
  if (!maxAwake) return;
  particleSensor.shutDown();
  maxAwake = false;
  Serial.println("[MAX30102] Shut down");
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  initMPU6050();
  initMAX30102();
  sleepMAX30102();   // Start with MAX30102 off

  Serial.println("System ready. Waiting for motion...");
}

// ─────────────────────────────────────────────────────────
void loop() {
  bool motion = motionDetected();

  // ── STATE: IDLE ───────────────────────────────────────
  if (state == IDLE) {
    if (motion) {
      Serial.println("[STATE] Motion detected → FINGER_CHECK");
      wakeMAX30102();
      lastMotionTime = millis();
      state = FINGER_CHECK;
    }
    return;   // Nothing else to do in IDLE
  }

  // ── Motion keep-alive (shared by FINGER_CHECK & ACTIVE)
  if (motion) lastMotionTime = millis();

  bool noMotionTimeout = (millis() - lastMotionTime > NO_MOTION_TIMEOUT);

  // ── STATE: FINGER_CHECK ───────────────────────────────
  if (state == FINGER_CHECK) {
    if (noMotionTimeout) {
      Serial.println("[STATE] No motion → back to IDLE");
      sleepMAX30102();
      state = IDLE;
      return;
    }

    // Peek at IR to check for finger
    while (!particleSensor.available()) particleSensor.check();
    long irValue = particleSensor.getIR();
    particleSensor.nextSample();

    if (irValue >= 50000) {
      Serial.println("[STATE] Finger detected → ACTIVE");
      state = ACTIVE;
    } else {
      Serial.println("[MAX30102] Waiting for finger...");
    }
    return;
  }

  // ── STATE: ACTIVE ─────────────────────────────────────
  if (state == ACTIVE) {
    // No motion AND no finger → back to IDLE
    if (noMotionTimeout) {
      Serial.println("[STATE] No motion → back to IDLE");
      sleepMAX30102();
      state = IDLE;
      return;
    }

    // ── Shift buffer ────────────────────────────────────
    for (byte i = 25; i < BUFFER_LENGTH; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25]  = irBuffer[i];
    }

    // ── Collect 25 new samples ───────────────────────────
    bool fingerDetected = true;
    for (byte i = 75; i < BUFFER_LENGTH; i++) {
      while (!particleSensor.available()) particleSensor.check();

      long irValue  = particleSensor.getIR();
      long redValue = particleSensor.getRed();

      if (irValue < 50000) {
        fingerDetected = false;
        rateSpot = 0; spo2Spot = 0;
        beatAvg  = 0; spo2Avg  = 0;
        memset(rates, 0, sizeof(rates));
        memset(spo2Readings, 0, sizeof(spo2Readings));
        // Drop back to FINGER_CHECK — MAX30102 stays on
        // (will sleep only when motion also stops)
        Serial.println("[STATE] Finger removed → FINGER_CHECK");
        state = FINGER_CHECK;
        break;
      }

      if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        beatsPerMinute = 60.0 / (delta / 1000.0);

        if (beatsPerMinute > 40 && beatsPerMinute < 180) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }

      redBuffer[i] = redValue;
      irBuffer[i]  = irValue;
      particleSensor.nextSample();
    }

    if (state != ACTIVE) return;   // Finger removed mid-loop

    // ── SpO2 recalc every 2s ─────────────────────────────
    if (millis() - lastSpo2Calc >= SPO2_INTERVAL) {
      lastSpo2Calc = millis();
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, BUFFER_LENGTH, redBuffer,
        &spo2, &validSPO2, &heartRate, &validHeartRate
      );
      if (validSPO2 && spo2 >= 85 && spo2 <= 100) {
        spo2Readings[spo2Spot++] = (byte)spo2;
        spo2Spot %= SPO2_SIZE;
        spo2Avg = 0;
        for (byte x = 0; x < SPO2_SIZE; x++) spo2Avg += spo2Readings[x];
        spo2Avg /= SPO2_SIZE;
      }
    }

    // ── Print ────────────────────────────────────────────
    Serial.println("=========================================");
    readAndPrintMPU6050();
    Serial.println("── MAX30102 ─────────────────────────────");
    Serial.print("Heart Rate : ");
    if (beatAvg > 0) { Serial.print(beatAvg); Serial.println(" bpm"); }
    else Serial.println("Calculating...");
    Serial.print("SpO2       : ");
    if (spo2Avg > 0) { Serial.print(spo2Avg); Serial.println(" %"); }
    else Serial.println("Calculating...");
    Serial.println("=========================================");
  }
}