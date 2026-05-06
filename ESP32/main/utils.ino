// ================================================================
// utils.ino
// LED helpers, I2C bus scanner, and watchdog initialisation
// ================================================================

#include "config.h"


// ================================================================
// LED HELPERS
// ESP32-C3 Super Mini LED is on GPIO8, active LOW
// LOW  = LED on
// HIGH = LED off
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
// Prints all devices found at boot — immediately confirms wiring
// Expected: MAX30102 at 0x57, MPU6050 at 0x68
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
// WATCHDOG
// ESP32-C3 compatible for Arduino core 2.0.17
// Reboots the device if loop() hangs for WDT_TIMEOUT_S seconds
// ================================================================

void initWatchdog() {
  esp_task_wdt_init(WDT_TIMEOUT_S, true);  // true = panic/reboot on timeout
  esp_task_wdt_add(NULL);                   // watch the current loop task
  Serial.printf("[WDT] Watchdog armed — %ds timeout\n", WDT_TIMEOUT_S);
}
