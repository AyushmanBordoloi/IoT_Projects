#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#define SDA_PIN 4
#define SCL_PIN 5

// ── MPU6050 ───────────────────────────────────────────────
const uint8_t MPU = 0x68;

// ── MAX30102 ──────────────────────────────────────────────
MAX30105 particleSensor;

const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

const byte SPO2_SIZE = 8;
byte spo2Readings[SPO2_SIZE];
byte spo2Spot = 0;
int spo2Avg;

#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2;
int8_t  validSPO2;
int32_t heartRate;
int8_t  validHeartRate;

unsigned long lastSpo2Calc = 0;
#define SPO2_INTERVAL 2000

bool fingerDetected = false;

// ─────────────────────────────────────────────────────────
void initMPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (size_t)1, true);
  uint8_t whoami = Wire.read();
  Serial.print("MPU6050 WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x80);
  Wire.endTransmission(true);
  delay(200);

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(200);

  Serial.println("MPU6050 Ready");
}

void initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
    while (1);
  }

  particleSensor.setup(60, 4, 2, 400, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  for (byte i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available())
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );

  Serial.println("MAX30102 Ready");
}

void readAndPrintMPU6050() {
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

  float ax = AcX / 16384.0;
  float ay = AcY / 16384.0;
  float az = AcZ / 16384.0;
  float gx = GyX / 131.0;
  float gy = GyY / 131.0;
  float gz = GyZ / 131.0;
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
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  initMPU6050();
  initMAX30102();

  Serial.println("Both sensors ready!");
  Serial.println("Place finger on MAX30102 for heart rate and SpO2.");
  Serial.println("=========================================");
}

void loop() {
  // ── Shift buffer ──────────────────────────────────────
  for (byte i = 25; i < BUFFER_LENGTH; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  // ── Collect 25 new samples ────────────────────────────
  fingerDetected = true;
  for (byte i = 75; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    long irValue  = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    // ── Finger check ─────────────────────────────────────
    if (irValue < 50000) {
      fingerDetected = false;

      // Reset MAX30102 averages
      rateSpot = 0;
      spo2Spot = 0;
      beatAvg  = 0;
      spo2Avg  = 0;
      memset(rates, 0, sizeof(rates));
      memset(spo2Readings, 0, sizeof(spo2Readings));
      break; // Break out of sample loop, still reach print section
    }

    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60.0 / (delta / 1000.0);

      if (beatsPerMinute > 40 && beatsPerMinute < 180) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    redBuffer[i] = redValue;
    irBuffer[i]  = irValue;
    particleSensor.nextSample();
  }

  // ── Recalculate SpO2 every 2 seconds (only with finger) 
  if (fingerDetected && millis() - lastSpo2Calc >= SPO2_INTERVAL) {
    lastSpo2Calc = millis();

    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_LENGTH, redBuffer,
      &spo2, &validSPO2,
      &heartRate, &validHeartRate
    );

    if (validSPO2 && spo2 >= 85 && spo2 <= 100) {
      spo2Readings[spo2Spot++] = (byte)spo2;
      spo2Spot %= SPO2_SIZE;

      spo2Avg = 0;
      for (byte x = 0; x < SPO2_SIZE; x++)
        spo2Avg += spo2Readings[x];
      spo2Avg /= SPO2_SIZE;
    }
  }

  // ── Print both sensors together every cycle ───────────
  Serial.println("=========================================");
  readAndPrintMPU6050();
  Serial.println("── MAX30102 ─────────────────────────────");
  if (fingerDetected) {
    Serial.print("Heart Rate : ");
    if (beatAvg > 0) {
      Serial.print(beatAvg); Serial.println(" bpm");
    } else {
      Serial.println("Calculating...");
    }
    Serial.print("SpO2       : ");
    if (spo2Avg > 0) {
      Serial.print(spo2Avg); Serial.println(" %");
    } else {
      Serial.println("Calculating...");
    }
  } else {
    Serial.println("No finger detected. Place finger for readings.");
  }
  Serial.println("=========================================");
}