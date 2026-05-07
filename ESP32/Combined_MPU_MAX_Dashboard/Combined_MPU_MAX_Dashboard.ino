#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi Credentials ──────────────────────────────────────
const char* ssid     = "Rezaur";
const char* password = "12345677";

WebServer server(80);

#define SDA_PIN 4
#define SCL_PIN 5

// ── MPU6050 ───────────────────────────────────────────────
const uint8_t MPU = 0x68;

// ── MPU6050 data globals ──────────────────────────────────
float g_ax, g_ay, g_az;
float g_gx, g_gy, g_gz;
float g_temp;

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
// Web Dashboard HTML
// ─────────────────────────────────────────────────────────
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
  <title>ESP32 Health Monitor</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', sans-serif;
      background: #0f0f1a;
      color: #e0e0e0;
      min-height: 100vh;
      padding: 20px;
    }
    h1 {
      text-align: center;
      color: #00d4ff;
      font-size: 1.6rem;
      margin-bottom: 6px;
      letter-spacing: 1px;
    }
    .subtitle {
      text-align: center;
      font-size: 0.8rem;
      color: #555;
      margin-bottom: 20px;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 14px;
      max-width: 700px;
      margin: 0 auto;
    }
    .card {
      background: #1a1a2e;
      border-radius: 14px;
      padding: 18px;
      border: 1px solid #222244;
    }
    .card.full { grid-column: span 2; }
    .card-title {
      font-size: 0.7rem;
      text-transform: uppercase;
      letter-spacing: 1px;
      color: #556;
      margin-bottom: 10px;
    }
    .big-value {
      font-size: 2.4rem;
      font-weight: 700;
      line-height: 1;
    }
    .unit {
      font-size: 0.9rem;
      color: #778;
      margin-left: 4px;
    }
    .bpm   { color: #ff5577; }
    .spo2  { color: #00d4ff; }
    .temp  { color: #ffaa44; }
    .accel { color: #88ff88; }
    .gyro  { color: #bb88ff; }

    .row3 {
      display: flex;
      justify-content: space-between;
      gap: 8px;
    }
    .mini {
      flex: 1;
      text-align: center;
    }
    .mini .label {
      font-size: 0.65rem;
      color: #556;
      margin-bottom: 4px;
    }
    .mini .val {
      font-size: 1.2rem;
      font-weight: 600;
    }

    .finger-badge {
      display: inline-block;
      padding: 4px 12px;
      border-radius: 20px;
      font-size: 0.75rem;
      font-weight: 600;
      margin-bottom: 12px;
    }
    .finger-on  { background: #0d3320; color: #44ff88; border: 1px solid #44ff88; }
    .finger-off { background: #3a1010; color: #ff4444; border: 1px solid #ff4444; }

    .status-dot {
      display: inline-block;
      width: 8px; height: 8px;
      border-radius: 50%;
      background: #44ff88;
      margin-right: 6px;
      animation: pulse 1.5s infinite;
    }
    @keyframes pulse {
      0%,100% { opacity: 1; }
      50%      { opacity: 0.3; }
    }
    .footer {
      text-align: center;
      margin-top: 18px;
      font-size: 0.7rem;
      color: #333;
    }

    @media (max-width: 480px) {
      .grid { grid-template-columns: 1fr; }
      .card.full { grid-column: span 1; }
    }
  </style>
</head>
<body>
  <h1>⚡ ESP32 Health Monitor</h1>
  <p class="subtitle"><span class="status-dot"></span>Live · updates every second</p>

  <div class="grid">

    <!-- Heart Rate -->
    <div class="card">
      <div class="card-title">❤️ Heart Rate</div>
      <div id="finger-badge" class="finger-badge finger-off">No Finger</div><br/>
      <span class="big-value bpm" id="bpm">--</span>
      <span class="unit">bpm</span>
    </div>

    <!-- SpO2 -->
    <div class="card">
      <div class="card-title">🩸 SpO2</div>
      <br/>
      <span class="big-value spo2" id="spo2">--</span>
      <span class="unit">%</span>
    </div>

    <!-- Temperature -->
    <div class="card">
      <div class="card-title">🌡️ Temperature (MPU6050)</div>
      <br/>
      <span class="big-value temp" id="temp">--</span>
      <span class="unit">°C</span>
    </div>

    <!-- Accelerometer -->
    <div class="card">
      <div class="card-title">📐 Accelerometer</div>
      <div class="row3">
        <div class="mini"><div class="label">X</div><div class="val accel" id="ax">--</div></div>
        <div class="mini"><div class="label">Y</div><div class="val accel" id="ay">--</div></div>
        <div class="mini"><div class="label">Z</div><div class="val accel" id="az">--</div></div>
      </div>
      <div style="text-align:center;font-size:0.65rem;color:#445;margin-top:6px;">g-force</div>
    </div>

    <!-- Gyroscope -->
    <div class="card full">
      <div class="card-title">🔄 Gyroscope</div>
      <div class="row3">
        <div class="mini"><div class="label">X</div><div class="val gyro" id="gx">--</div></div>
        <div class="mini"><div class="label">Y</div><div class="val gyro" id="gy">--</div></div>
        <div class="mini"><div class="label">Z</div><div class="val gyro" id="gz">--</div></div>
      </div>
      <div style="text-align:center;font-size:0.65rem;color:#445;margin-top:6px;">°/s</div>
    </div>

  </div>

  <p class="footer">ESP32-C3 · MPU6050 + MAX30102</p>

  <script>
    async function fetchData() {
      try {
        const res = await fetch('/data');
        const d   = await res.json();

        document.getElementById('bpm').textContent  = d.beatAvg  > 0 ? d.beatAvg  : '--';
        document.getElementById('spo2').textContent = d.spo2Avg  > 0 ? d.spo2Avg  : '--';
        document.getElementById('temp').textContent = d.temp;
        document.getElementById('ax').textContent   = d.ax;
        document.getElementById('ay').textContent   = d.ay;
        document.getElementById('az').textContent   = d.az;
        document.getElementById('gx').textContent   = d.gx;
        document.getElementById('gy').textContent   = d.gy;
        document.getElementById('gz').textContent   = d.gz;

        const badge = document.getElementById('finger-badge');
        if (d.finger) {
          badge.textContent  = '✅ Finger Detected';
          badge.className    = 'finger-badge finger-on';
        } else {
          badge.textContent  = '❌ No Finger';
          badge.className    = 'finger-badge finger-off';
        }
      } catch(e) { console.log('fetch error', e); }
    }

    fetchData();
    setInterval(fetchData, 1000);
  </script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", DASHBOARD_HTML);
}

void handleData() {
  String json = "{";
  json += "\"ax\":"     + String(g_ax, 2) + ",";
  json += "\"ay\":"     + String(g_ay, 2) + ",";
  json += "\"az\":"     + String(g_az, 2) + ",";
  json += "\"gx\":"     + String(g_gx, 1) + ",";
  json += "\"gy\":"     + String(g_gy, 1) + ",";
  json += "\"gz\":"     + String(g_gz, 1) + ",";
  json += "\"temp\":"   + String(g_temp, 1) + ",";
  json += "\"beatAvg\":" + String(beatAvg) + ",";
  json += "\"spo2Avg\":" + String(spo2Avg) + ",";
  json += "\"finger\":"  + String(fingerDetected ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

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

  g_ax   = AcX / 16384.0;
  g_ay   = AcY / 16384.0;
  g_az   = AcZ / 16384.0;
  g_gx   = GyX / 131.0;
  g_gy   = GyY / 131.0;
  g_gz   = GyZ / 131.0;
  g_temp = (Tmp / 340.0) + 36.53;
}

// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  initMPU6050();
  initMAX30102();

  // ── Connect to WiFi ───────────────────────────────────
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("Open dashboard at: http://");
  Serial.println(WiFi.localIP());

  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.begin();

  Serial.println("Both sensors ready!");
  Serial.println("Place finger on MAX30102 for heart rate and SpO2.");
  Serial.println("=========================================");
}

void loop() {
  server.handleClient();  // handle web requests

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

      rateSpot = 0;
      spo2Spot = 0;
      beatAvg  = 0;
      spo2Avg  = 0;
      memset(rates, 0, sizeof(rates));
      memset(spo2Readings, 0, sizeof(spo2Readings));
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
  readAndPrintMPU6050();
}

