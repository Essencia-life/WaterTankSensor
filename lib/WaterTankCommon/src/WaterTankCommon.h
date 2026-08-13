#pragma once

#include <Arduino.h>
#include <time.h>

enum SensorState {
  STATE_OK,
  STATE_TIMEOUT,
  STATE_DEADZONE,
  STATE_DRIFT_ERROR,
  STATE_INIT,
  STATE_OUT_OF_RANGE,
};

constexpr int echoPin = 5;
constexpr int trigPin = 4;
constexpr int cycle = 1000;

constexpr unsigned int distance_deadzone = 22;
constexpr long max_TOF_sens = 30000;

constexpr unsigned int distance_max_depth_watertank = 150;
constexpr unsigned int height_watertank_0percent = 20;
constexpr unsigned int height_watertank_100percent = 120;
constexpr unsigned int distance_watertank_0percent = distance_max_depth_watertank - height_watertank_0percent;
constexpr unsigned int distance_watertank_100percent = distance_max_depth_watertank - height_watertank_100percent;

constexpr unsigned int liter_per_cm = 125;

struct LoRaPayload {
  uint8_t state_watertank_sensor;
  float waterlevel;
};

inline float percentage_watertank(float distance) {
  if (distance >= distance_watertank_0percent) return 0;
  if (distance <= distance_watertank_100percent) return 100;

  long nutzbare_hoehe = distance_watertank_0percent - distance_watertank_100percent;
  float aktuelle_wasserhoehe = distance_watertank_0percent - distance;
  float result = ((aktuelle_wasserhoehe * 100) / nutzbare_hoehe);
  return result;
}

inline String getUtcTimestamp() {
  time_t now = time(nullptr);
  if (now <= 0) {
    return "1970-01-01T00:00:00Z";
  }

  struct tm timeinfo;
  if (gmtime_r(&now, &timeinfo) == nullptr) {
    return "1970-01-01T00:00:00Z";
  }

  char ts[32];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           timeinfo.tm_year + 1900,
           timeinfo.tm_mon + 1,
           timeinfo.tm_mday,
           timeinfo.tm_hour,
           timeinfo.tm_min,
           timeinfo.tm_sec);
  return String(ts);
}

inline String buildTankIngestPayload(uint8_t sensorState, float waterLevelPercent, float levelCm, int signalDbm) {
  String timestamp = getUtcTimestamp();
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"deviceId\":\"essencia-water-tank-01\",\"timestamp\":\"%s\",\"state\":%u,\"waterLevelPercent\":%.1f,\"levelCm\":%.1f,\"signalDbm\":%d}",
           timestamp.c_str(),
           static_cast<unsigned int>(sensorState),
           waterLevelPercent,
           levelCm,
           signalDbm);
  return String(payload);
}
