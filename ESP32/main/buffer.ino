// ================================================================
// buffer.ino
// Offline ring buffer — stores readings when MQTT is unavailable
// and flushes them automatically when connection is restored
// ================================================================

#include "config.h"


// ================================================================
// BUFFER PUSH
// Adds a payload string to the ring buffer
// If full, silently drops the oldest reading to make room
// ================================================================

void bufferPush(const String& data) {
  if (bufCount >= BUFFER_SIZE) {
    // Full — drop oldest
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    bufCount--;
  }
  ringBuffer[bufHead] = data;
  bufHead  = (bufHead + 1) % BUFFER_SIZE;
  bufCount++;
  Serial.printf("[BUFFER] Stored offline. Count: %d/%d\n", bufCount, BUFFER_SIZE);
}


// ================================================================
// BUFFER FLUSH
// Publishes all buffered readings to MQTT in order
// Called automatically when MQTT reconnects
// ================================================================

void flushBuffer() {
  if (bufCount == 0) return;
  Serial.printf("[BUFFER] Flushing %d buffered readings...\n", bufCount);
  while (bufCount > 0 && mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC, ringBuffer[bufTail].c_str());
    bufTail  = (bufTail + 1) % BUFFER_SIZE;
    bufCount--;
    delay(20);   // small gap to avoid flooding broker
  }
  Serial.println("[BUFFER] Flush complete");
}
