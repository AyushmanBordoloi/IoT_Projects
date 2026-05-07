#ifndef CONFIG_H
#define CONFIG_H

// ================================================================
// config.h
// All #defines, the ImuData struct, and global variable declarations
// Edit WiFi/MQTT settings here before flashing
// ================================================================


// ================================================================
// USER SETTINGS — edit these before flashing
// ================================================================

// WiFi
#define WIFI_SSID            "NoName"
#define WIFI_PASS            "NoNamelol"

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
// IMU DATA STRUCT
// Declared here so every file that includes config.h can use it
// ================================================================

struct ImuData {
  float ax, ay, az;   // accelerometer in m/s^2
  float gx, gy, gz;   // gyroscope in rad/s
  float temp;          // MPU6050 internal temperature in C
};


// ================================================================
// GLOBAL VARIABLE DECLARATIONS
// Defined once in main.ino, declared here with extern so all
// other .ino files can access them without redefinition errors
// ================================================================

#include <WiFi.h>
#include <PubSubClient.h>

extern WiFiClient       wifiClient;
extern PubSubClient     mqttClient;

// MAX30102 heart rate state
extern const byte       HR_RATE_SIZE;
extern byte             hrRates[];
extern byte             hrRateSpot;
extern long             lastBeat;
extern float            currentBPM;
extern int              avgBPM;

// MAX30102 SpO2 buffers
extern uint32_t         irBuffer[];
extern uint32_t         redBuffer[];
extern int32_t          spo2Value;
extern int8_t           spo2Valid;
extern int32_t          hrFromAlgo;
extern int8_t           hrValid;
extern bool             spo2BufferReady;

// Sensor health flags
extern bool             max30102Ready;
extern bool             mpu6050Ready;

// Offline ring buffer
extern String           ringBuffer[];
extern int              bufHead;
extern int              bufTail;
extern int              bufCount;

// Reconnect timing
extern unsigned long    lastReconnectAttempt;
extern unsigned long    reconnectDelay;

// Publish timing
extern unsigned long    lastPublishTime;

#endif
