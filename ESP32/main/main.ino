// ================================================================
// main.ino
// Board   : ESP32-C3 Super Mini
// Sensors : MAX30102 (HR, SpO2) + MPU6050 (Accel, Gyro)
// I2C     : SDA -> GPIO4, SCL -> GPIO5
// Protocol: MQTT over WiFi
// Core    : Arduino ESP32 Core 2.0.17
//
// Sketch folder structure:
//   main.ino          <- setup(), loop(), global variable definitions
//   config.h          <- all #defines, ImuData struct, extern declarations
//   sensors.ino       <- MAX30102 and MPU6050 init + read functions
//   payload.ino       <- buildPayload()
//   buffer.ino        <- ring buffer push/flush
//   connectivity.ino  <- WiFi, MQTT, NTP
//   utils.ino         <- LED helpers, I2C scan, watchdog
// ================================================================


// ================================================================
// LIBRARIES — included once here, available to all .ino files
// ================================================================
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_task_wdt.h>

#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "config.h"


// ================================================================
// GLOBAL VARIABLE DEFINITIONS
// Declared as extern in config.h, defined exactly once here
// ================================================================

// Sensor objects
MAX30105          particleSensor;
Adafruit_MPU6050  mpu;

// Network clients
WiFiClient        wifiClient;
PubSubClient      mqttClient(wifiClient);

// MAX30102 heart rate state
const byte        HR_RATE_SIZE = 8;
byte              hrRates[8];
byte              hrRateSpot   = 0;
long              lastBeat     = 0;
float             currentBPM   = 0;
int               avgBPM       = 0;

// MAX30102 SpO2 buffers
uint32_t          irBuffer[SPO2_BUFFER_LEN];
uint32_t          redBuffer[SPO2_BUFFER_LEN];
int32_t           spo2Value       = 0;
int8_t            spo2Valid       = 0;
int32_t           hrFromAlgo      = 0;
int8_t            hrValid         = 0;
bool              spo2BufferReady = false;

// Sensor health flags
bool              max30102Ready = false;
bool              mpu6050Ready  = false;

// Offline ring buffer
String            ringBuffer[BUFFER_SIZE];
int               bufHead   = 0;
int               bufTail   = 0;
int               bufCount  = 0;

// Reconnect timing
unsigned long     lastReconnectAttempt = 0;
unsigned long     reconnectDelay       = 1000;

// Publish timing
unsigned long     lastPublishTime = 0;


// ================================================================
// SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(1500);   // ESP32-C3 needs a moment before Serial is ready
  Serial.println("\n\n=== ESP32-C3 Super Mini — Health Monitor ===");
  Serial.println("Core: 2.0.17 | SDA: GPIO4 | SCL: GPIO5 | LED: GPIO8");

  // LED — GPIO8, active LOW
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  // I2C on GPIO4 (SDA) and GPIO5 (SCL)
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.printf("[I2C] Started on SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA, I2C_SCL);

  // Scan I2C bus — immediately shows if wiring is correct
  scanI2C();

  // Init sensors
  max30102Ready = initMAX30102();
  mpu6050Ready  = initMPU6050();

  // If both fail, halt with rapid LED blink
  if (!max30102Ready && !mpu6050Ready) {
    Serial.println("[ERROR] Both sensors failed. Verify I2C wiring. Halting.");
    while (true) {
      ledBlink(5, 100);
      delay(500);
    }
  }

  // WiFi
  connectWiFi();

  // NTP time sync
  syncTime();

  // MQTT
  setupMQTT();

  // Watchdog — init last, after all blocking setup is done
  initWatchdog();

  Serial.println("=== Setup complete. Entering main loop. ===\n");
  ledOn();   // steady LED = system running
}


// ================================================================
// LOOP
// ================================================================

void loop() {
  // Must reset watchdog every iteration or device reboots
  esp_task_wdt_reset();

  // Keep WiFi and MQTT alive
  checkWiFi();
  checkMQTT();

  // Rate-limit publishing to SAMPLE_INTERVAL_MS (100ms = 10Hz)
  unsigned long now = millis();
  if (now - lastPublishTime < SAMPLE_INTERVAL_MS) return;
  lastPublishTime = now;

  // Read MAX30102
  float hr   = 0.0;
  float spO2 = 0.0;
  if (max30102Ready) {
    updateSpO2();
    hr   = readHeartRate();
    spO2 = getSpO2();
  }

  // Read MPU6050
  ImuData imu = {0, 0, 0, 0, 0, 0, 0};
  if (mpu6050Ready) {
    imu = readIMU();
  }

  // Build JSON payload and publish (or buffer if offline)
  String payload = buildPayload(hr, spO2, imu);
  publishOrBuffer(payload);
}
