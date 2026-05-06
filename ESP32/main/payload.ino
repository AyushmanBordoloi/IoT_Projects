// ================================================================
// payload.ino
// Builds the JSON string that gets published to the MQTT broker
// ================================================================

#include "config.h"


// ================================================================
// BUILD PAYLOAD
// Packs all sensor readings into a single JSON string
// Uses real UTC timestamp if NTP synced, else falls back to millis()
// ================================================================

String buildPayload(float hr, float spO2, ImuData imu) {
  StaticJsonDocument<512> doc;

  long ts = (time(nullptr) > 1000000000UL)
              ? (long)time(nullptr)
              : (long)millis();

  doc["device_id"] = DEVICE_ID;
  doc["ts"]        = ts;
  doc["hr"]        = round(hr   * 10.0)    / 10.0;
  doc["spo2"]      = round(spO2 * 10.0)    / 10.0;
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
