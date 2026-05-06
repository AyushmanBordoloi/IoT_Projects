// ================================================================
// sensors.ino
// MAX30102 and MPU6050 initialisation and read functions
// ================================================================

#include "config.h"


// ================================================================
// SENSOR INIT
// ================================================================

bool initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[MAX30102] NOT FOUND — check SDA=GPIO4, SCL=GPIO5");
    return false;
  }

  // 60mA LED current, 4 samples averaged, mode 2 (red+IR),
  // 100Hz sample rate, 411us pulse width, 4096 ADC range
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x3C);
  particleSensor.setPulseAmplitudeIR(0x3C);
  particleSensor.setPulseAmplitudeGreen(0);   // green LED not needed

  Serial.println("[MAX30102] Initialised OK");
  return true;
}

bool initMPU6050() {
  if (!mpu.begin()) {
    Serial.println("[MPU6050] NOT FOUND — check SDA=GPIO4, SCL=GPIO5");
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("[MPU6050] Initialised OK");
  return true;
}


// ================================================================
// HEART RATE READ
// Beat detection with 8-sample rolling average
// Returns 0.0 if no finger detected
// ================================================================

float readHeartRate() {
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    return 0.0;   // no finger on sensor
  }

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat   = millis();
    currentBPM = 60.0 / (delta / 1000.0);

    if (currentBPM > 20 && currentBPM < 255) {
      hrRates[hrRateSpot++] = (byte)currentBPM;
      hrRateSpot %= HR_RATE_SIZE;

      int sum = 0;
      for (byte x = 0; x < HR_RATE_SIZE; x++) sum += hrRates[x];
      avgBPM = sum / HR_RATE_SIZE;
    }
  }
  return (float)avgBPM;
}


// ================================================================
// SPO2 UPDATE
// Maxim algorithm on 100-sample buffer
// First call fills the buffer (blocking ~1s), then rolling 25-sample updates
// ================================================================

void updateSpO2() {
  if (!spo2BufferReady) {
    Serial.println("[MAX30102] Collecting initial SpO2 buffer...");
    for (int i = 0; i < SPO2_BUFFER_LEN; i++) {
      while (!particleSensor.available()) particleSensor.check();
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
    }
    spo2BufferReady = true;
    Serial.println("[MAX30102] SpO2 buffer ready");
  } else {
    // Shift oldest 25 samples out, collect 25 new ones
    for (byte i = 25; i < SPO2_BUFFER_LEN; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25]  = irBuffer[i];
    }
    for (byte i = 75; i < SPO2_BUFFER_LEN; i++) {
      while (!particleSensor.available()) particleSensor.check();
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
    }
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, SPO2_BUFFER_LEN, redBuffer,
    &spo2Value, &spo2Valid,
    &hrFromAlgo, &hrValid
  );
}

// Returns SpO2 only if valid and in physiological range (85-100%)
// Returns 0.0 otherwise
float getSpO2() {
  if (!spo2Valid) return 0.0;
  if (spo2Value < 85 || spo2Value > 100) return 0.0;
  return (float)spo2Value;
}


// ================================================================
// IMU READ
// Returns all 6 axes + internal temperature from MPU6050
// ================================================================

ImuData readIMU() {
  ImuData data = {0, 0, 0, 0, 0, 0, 0};
  sensors_event_t accel, gyro, tempEvent;
  mpu.getEvent(&accel, &gyro, &tempEvent);

  data.ax   = accel.acceleration.x;
  data.ay   = accel.acceleration.y;
  data.az   = accel.acceleration.z;
  data.gx   = gyro.gyro.x;
  data.gy   = gyro.gyro.y;
  data.gz   = gyro.gyro.z;
  data.temp = tempEvent.temperature;
  return data;
}
