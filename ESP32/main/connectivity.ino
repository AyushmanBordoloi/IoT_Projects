// ================================================================
// connectivity.ino
// WiFi connection management, MQTT client, and NTP time sync
// ================================================================

#include "config.h"


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
    ledBlink(3, 100);   // 3 quick blinks = WiFi connected
  } else {
    Serial.println("\n[WiFi] Failed — will retry in loop");
  }
}

// Called every loop — reconnects silently if connection dropped
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
// Safe to call every loop iteration — only attempts after delay expires
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
    flushBuffer();            // immediately send any buffered readings
  } else {
    Serial.printf("[MQTT] Failed (rc=%d) — retry in %lums\n",
                  mqttClient.state(), reconnectDelay);
    reconnectDelay = min(reconnectDelay * 2UL, 30000UL);  // cap at 30s
  }
}

// Publish payload to MQTT, or buffer it if not connected
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
