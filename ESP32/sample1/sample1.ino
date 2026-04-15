#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#define SDA_PIN 4
#define SCL_PIN 5

MAX30105 particleSensor;

// ── Heart Rate ────────────────────────────────────────────
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// ── SpO2 Averaging ────────────────────────────────────────
const byte SPO2_SIZE = 8;
byte spo2Readings[SPO2_SIZE];
byte spo2Spot = 0;
int spo2Avg;

// ── SpO2 Buffers ──────────────────────────────────────────
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2;
int8_t  validSPO2;
int32_t heartRate;
int8_t  validHeartRate;

// ── Timing ────────────────────────────────────────────────
unsigned long lastSpo2Calc = 0;
#define SPO2_INTERVAL 2000  // recalculate SpO2 every 2 seconds

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring.");
    while (1);
  }

  Serial.println("Place finger on sensor.");
  Serial.println("Wait 10 seconds for stable readings.");
  Serial.println("------------------------------------");

  // ── Sensor config ─────────────────────────────────────
  particleSensor.setup(60, 4, 2, 400, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  // ── Fill initial SpO2 buffer ──────────────────────────
  for (byte i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available())
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // ── First SpO2 calculation ────────────────────────────
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH, redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );
}

void loop() {

  // ── Shift SpO2 buffer: drop oldest 25, keep 75 ───────
  for (byte i = 25; i < BUFFER_LENGTH; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  // ── Collect 25 new samples ────────────────────────────
  for (byte i = 75; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    long irValue  = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    // ── Finger check ────────────────────────────────────
    if (irValue < 50000) {
      Serial.println("No finger detected. Place finger on sensor...");

      // Reset averages when finger removed
      rateSpot = 0;
      spo2Spot = 0;
      beatAvg  = 0;
      spo2Avg  = 0;
      memset(rates, 0, sizeof(rates));
      memset(spo2Readings, 0, sizeof(spo2Readings));

      delay(500);
      return;
    }

    // ── Beat detection for Heart Rate ───────────────────
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60.0 / (delta / 1000.0);

      if (beatsPerMinute > 40 && beatsPerMinute < 180) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        // Average last 8 BPM readings
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

  // ── Recalculate SpO2 every 2 seconds ─────────────────
  if (millis() - lastSpo2Calc >= SPO2_INTERVAL) {
    lastSpo2Calc = millis();

    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, BUFFER_LENGTH, redBuffer,
      &spo2, &validSPO2,
      &heartRate, &validHeartRate
    );

    // ── Average SpO2 just like heart rate ────────────────
    if (validSPO2 && spo2 >= 85 && spo2 <= 100) {
      spo2Readings[spo2Spot++] = (byte)spo2;
      spo2Spot %= SPO2_SIZE;

      // Average last 8 SpO2 readings
      spo2Avg = 0;
      for (byte x = 0; x < SPO2_SIZE; x++)
        spo2Avg += spo2Readings[x];
      spo2Avg /= SPO2_SIZE;
    }
  }

  // ── Print Results ─────────────────────────────────────
  Serial.println("===========================");

  Serial.print("Heart Rate : ");
  if (beatAvg > 0) {
    Serial.print(beatAvg);
    Serial.println(" bpm");
  } else {
    Serial.println("Calculating...");
  }

  Serial.print("SpO2       : ");
  if (spo2Avg > 0) {
    Serial.print(spo2Avg);
    Serial.println(" %");
  } else {
    Serial.println("Calculating...");
  }

  Serial.println("===========================");
}