// ================================================================
// HEALTH MONITORING FIRMWARE
// Board   : ESP32-C3 Super Mini
// Sensors : MAX30102 (HR, SpO2) + MPU6050 (Accel, Gyro)
// I2C     : SDA -> GPIO4, SCL -> GPIO5
// Protocol: MQTT over WiFi
// Core    : Arduino ESP32 Core 2.0.17
// ================================================================


// ================================================================
// LIBRARIES
// ================================================================
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

// Watchdog — ESP32-C3 specific header
#include <esp_task_wdt.h>

// MAX30102
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// ================================================================
// CONFIG — edit these before flashing
// ================================================================

// WiFi
#define WIFI_SSID            "your_wifi_ssid"
#define WIFI_PASS            "your_wifi_password"

// MQTT broker — set this to your server's local IP
#define MQTT_BROKER          "192.168.1.100"
#define MQTT_PORT            1883
#define MQTT_USER            "esp32_device"
#define MQTT_PASS            "device_secret"
#define MQTT_TOPIC           "health/device01/raw"
#define DEVICE_ID            "device01"

// NTP — IST = UTC+5:30 = 19800 seconds
#define NTP_SERVER           "pool.ntp.org"
#define NTP_OFFSET_SEC       19800

// I2C pins — as per your wiring
#define I2C_SDA              4
#define I2C_SCL              5

// ESP32-C3 Super Mini onboard LED — GPIO8, active LOW
#define LED_PIN              8

// Sampling — publish every 100ms = 10Hz
#define SAMPLE_INTERVAL_MS   100

// Watchdog timeout in seconds
#define WDT_TIMEOUT_S        10

// Offline ring buffer capacity
#define BUFFER_SIZE          100

// SpO2 algorithm buffer length — must be 100
#define SPO2_BUFFER_LEN      100


// ================================================================
// GLOBALS
// ================================================================

// IMU data struct — declared here so all functions can use it
struct ImuData {
  float ax, ay, az;   // accelerometer in m/s^2
  float gx, gy, gz;   // gyroscope in rad/s
  float temp;          // MPU6050 internal temperature in C
};

// Sensor objects
MAX30105          particleSensor;
Adafruit_MPU6050  mpu;

// Network clients
WiFiClient        wifiClient;
PubSubClient      mqttClient(wifiClient);

// ---- MAX30102 heart rate state ----
const byte        HR_RATE_SIZE = 8;
byte              hrRates[HR_RATE_SIZE];
byte              hrRateSpot   = 0;
long              lastBeat     = 0;
float             currentBPM   = 0;
int               avgBPM       = 0;

// ---- MAX30102 SpO2 algorithm buffers ----
uint32_t          irBuffer[SPO2_BUFFER_LEN];
uint32_t          redBuffer[SPO2_BUFFER_LEN];
int32_t           spo2Value       = 0;
int8_t            spo2Valid       = 0;
int32_t           hrFromAlgo      = 0;
int8_t            hrValid         = 0;
bool              spo2BufferReady = false;

// ---- Sensor health flags ----
bool              max30102Ready = false;
bool              mpu6050Ready  = false;

// ---- Offline ring buffer ----
String            ringBuffer[BUFFER_SIZE];
int               bufHead   = 0;
int               bufTail   = 0;
int               bufCount  = 0;

// ---- Reconnect timing ----
unsigned long     lastReconnectAttempt = 0;
unsigned long     reconnectDelay       = 1000;

// ---- Publish timing ----
unsigned long     lastPublishTime = 0;


// ================================================================
// LED HELPERS
// ESP32-C3 Super Mini LED is active LOW (LOW = on, HIGH = off)
// ================================================================

void ledOn()  { digitalWrite(LED_PIN, LOW);  }
void ledOff() { digitalWrite(LED_PIN, HIGH); }

void ledBlink(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    ledOn();  delay(delayMs);
    ledOff(); delay(delayMs);
  }
}


// ================================================================
// I2C BUS SCAN
// Prints all devices found — run this to verify wiring before anything else
// ================================================================

void scanI2C() {
  Serial.println("[I2C] Scanning bus...");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("[I2C] Device at 0x%02X", addr);
      if (addr == 0x57 || addr == 0x56) Serial.print("  <-- MAX30102");
      if (addr == 0x68 || addr == 0x69) Serial.print("  <-- MPU6050");
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C] No devices found — check GPIO4/GPIO5 wiring!");
  } else {
    Serial.printf("[I2C] %d device(s) found\n", found);
  }
}


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
// SENSOR READ
// ================================================================

// Heart rate via beat detection + 8-sample rolling average
float readHeartRate() {
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    // No finger detected
    return 0.0;
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

// SpO2 via Maxim algorithm
// First call fills the 100-sample buffer (takes a moment)
// Subsequent calls do rolling 25-sample updates
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
float getSpO2() {
  if (!spo2Valid) return 0.0;
  if (spo2Value < 85 || spo2Value > 100) return 0.0;
  return (float)spo2Value;
}

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


// ================================================================
// PAYLOAD BUILDER
// ================================================================

String buildPayload(float hr, float spO2, ImuData imu) {
  StaticJsonDocument<512> doc;

  // Use real UTC timestamp if NTP synced, else fallback to millis
  long ts = (time(nullptr) > 1000000000UL)
              ? (long)time(nullptr)
              : (long)millis();

  doc["device_id"] = DEVICE_ID;
  doc["ts"]        = ts;
  doc["hr"]        = round(hr * 10.0) / 10.0;
  doc["spo2"]      = round(spO2 * 10.0) / 10.0;
  doc["ax"]        = round(imu.ax * 10000.0) / 10000.0;
  doc["ay"]        = round(imu.ay * 10000.0) / 10000.0;
  doc["az"]        = round(imu.az * 10000.0) / 10000.0;
  doc["gx"]        = round(imu.gx * 10000.0) / 10000.0;
  doc["gy"]        = round(imu.gy * 10000.0) / 10000.0;
  doc["gz"]        = round(imu.gz * 10000.0) / 10000.0;
  doc["temp"]      = round(imu.temp * 10.0) / 10.0;
  doc["valid"]     = (hr > 0 && spO2 > 0);

  String output;
  serializeJson(doc, output);
  return output;
}


// ================================================================
// OFFLINE RING BUFFER
// ================================================================

void bufferPush(const String& data) {
  if (bufCount >= BUFFER_SIZE) {
    // Buffer full — silently drop oldest reading
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    bufCount--;
  }
  ringBuffer[bufHead] = data;
  bufHead  = (bufHead + 1) % BUFFER_SIZE;
  bufCount++;
  Serial.printf("[BUFFER] Stored offline. Count: %d/%d\n", bufCount, BUFFER_SIZE);
}

void flushBuffer() {
  if (bufCount == 0) return;
  Serial.printf("[BUFFER] Flushing %d readings...\n", bufCount);
  while (bufCount > 0 && mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC, ringBuffer[bufTail].c_str());
    bufTail  = (bufTail + 1) % BUFFER_SIZE;
    bufCount--;
    delay(20);   // small gap to avoid flooding broker
  }
  Serial.println("[BUFFER] Flush complete");
}


// ================================================================
// WIFI
// ================================================================

void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected — IP: %s\n",
                  WiFi.localIP().toString().c_str());
    ledBlink(3, 100);   // 3 quick blinks = connected
  } else {
    Serial.println("\n[WiFi] Failed — will retry in loop");
  }
}

void checkWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("[WiFi] Lost connection — reconnecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Reconnected");
    ledBlink(2, 100);
  }
}


// ================================================================
// MQTT
// ================================================================

void setupMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(10);
}

// Non-blocking reconnect with exponential backoff
// Safe to call every loop iteration
void checkMQTT() {
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }

  unsigned long now = millis();
  if (now - lastReconnectAttempt < reconnectDelay) return;
  lastReconnectAttempt = now;

  Serial.printf("[MQTT] Connecting to %s...\n", MQTT_BROKER);

  if (mqttClient.connect(DEVICE_ID, MQTT_USER, MQTT_PASS)) {
    Serial.println("[MQTT] Connected OK");
    reconnectDelay = 1000;   // reset backoff on success
    flushBuffer();            // send any buffered readings immediately
  } else {
    Serial.printf("[MQTT] Failed (rc=%d) — retry in %lums\n",
                  mqttClient.state(), reconnectDelay);
    reconnectDelay = min(reconnectDelay * 2UL, 30000UL);  // cap at 30s
  }
}

void publishOrBuffer(const String& payload) {
  if (mqttClient.connected()) {
    if (mqttClient.publish(MQTT_TOPIC, payload.c_str())) {
      Serial.println("[MQTT] Published: " + payload);
    } else {
      Serial.println("[MQTT] Publish failed — buffering");
      bufferPush(payload);
    }
  } else {
    bufferPush(payload);
  }
}


// ================================================================
// NTP TIME SYNC
// ================================================================

void syncTime() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[NTP] Skipped — no WiFi");
    return;
  }
  Serial.print("[NTP] Syncing time");
  configTime(NTP_OFFSET_SEC, 0, NTP_SERVER);
  int attempts = 0;
  while (time(nullptr) < 1000000000UL && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (time(nullptr) > 1000000000UL) {
    Serial.println("\n[NTP] Synced OK");
  } else {
    Serial.println("\n[NTP] Sync failed — using millis() as timestamp fallback");
  }
}


// ================================================================
// WATCHDOG — ESP32-C3 compatible for Arduino core 2.0.17
// ================================================================

void initWatchdog() {
  esp_task_wdt_init(WDT_TIMEOUT_S, true);  // true = panic/reboot on timeout
  esp_task_wdt_add(NULL);                   // watch the current (loop) task
  Serial.printf("[WDT] Watchdog armed — %ds timeout\n", WDT_TIMEOUT_S);
}


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

  // Scan I2C — this tells you immediately if wiring is correct
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

  // NTP
  syncTime();

  // MQTT
  setupMQTT();

  // Watchdog — init last, after all blocking setup is done
  initWatchdog();

  Serial.println("=== Setup complete. Entering main loop. ===\n");
  ledOn();   // steady LED on = system running
}


// ================================================================
// LOOP
// ================================================================

void loop() {
  // Must call this every iteration or watchdog reboots the device
  esp_task_wdt_reset();

  // Keep WiFi and MQTT alive
  checkWiFi();
  checkMQTT();

  // Rate-limit publishing to SAMPLE_INTERVAL_MS (100ms = 10Hz)
  unsigned long now = millis();
  if (now - lastPublishTime < SAMPLE_INTERVAL_MS) return;
  lastPublishTime = now;

  // ---- Read MAX30102 ----
  float hr   = 0.0;
  float spO2 = 0.0;
  if (max30102Ready) {
    updateSpO2();
    hr   = readHeartRate();
    spO2 = getSpO2();
  }

  // ---- Read MPU6050 ----
  ImuData imu = {0, 0, 0, 0, 0, 0, 0};
  if (mpu6050Ready) {
    imu = readIMU();
  }

  // ---- Build JSON and send ----
  String payload = buildPayload(hr, spO2, imu);
  publishOrBuffer(payload);
}
