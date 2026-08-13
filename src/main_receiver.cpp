#include <Arduino.h>
#include <heltec_unofficial.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include "WaterTankCommon.h"

#ifndef SERVER_HOST
#define SERVER_HOST "example.com"
#endif
#ifndef SERVER_API_KEY
#define SERVER_API_KEY "replace-me"
#endif

const char* wifi_ssid = "STARLINK";
unsigned long last_rx_millis = 0;
volatile bool rxFlag = false;
uint8_t rx_sensor_state = STATE_INIT;
float rx_water_level = -1;
String rx_status_text = "Waiting...";
bool wifi_and_time_state_is_ok = false;
bool lora_state_is_ok = true;

#if defined(ESP32)
void IRAM_ATTR rxIsr();
void IRAM_ATTR rxIsr() {
  rxFlag = true;
}
#else
void rxIsr();
void rxIsr() {
  rxFlag = true;
}
#endif

String getStatusText(uint8_t state) {
  switch (state) {
    case STATE_OK:            return "OK";
    case STATE_TIMEOUT:       return "Err (Timeout)";
    case STATE_DEADZONE:      return "OK (Min. Dist)";
    case STATE_DRIFT_ERROR:   return "Err (Drift)";
    case STATE_OUT_OF_RANGE:  return "Err (Out of Range)";
    default:                  return "Starting...";
  }
}

bool pushTankReadingToServer(uint8_t sensorState, float waterLevelPercent, float levelCm, int signalDbm) {
  if (strlen(SERVER_HOST) == 0 || strlen(SERVER_API_KEY) == 0) {
    Serial.println("Server push skipped: SERVER_HOST / SERVER_API_KEY not configured.");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Server push skipped: WiFi disconnected.");
    return false;
  }

  String payload = buildTankIngestPayload(sensorState, waterLevelPercent, levelCm, signalDbm);
  String url = String("https://") + String(SERVER_HOST) + "/api/ingest/lora-tank";

  Serial.printf("Posting to %s\n", url.c_str());
  Serial.printf("Payload: %s\n", payload.c_str());

  WiFiClientSecure client;
  HTTPClient http;
  client.setInsecure();

  if (!http.begin(client, url)) {
    Serial.println("HTTP begin failed for server push.");
    return false;
  }

  http.setTimeout(15000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", SERVER_API_KEY);

  int httpCode = http.POST(payload);
  String response = http.getString();
  http.end();

  if (httpCode >= 200 && httpCode < 300) {
    Serial.printf("Server push OK: HTTP %d\n", httpCode);
    Serial.println(response);
    return true;
  }

  Serial.printf("Server push failed: HTTP %d\n", httpCode);
  Serial.println(response);
  return false;
}

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 18, /* data=*/ 17);

void setup() {
  Serial.begin(115200);
  heltec_setup();

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  int state = radio.begin(868.0);
  if (state == RADIOLIB_ERR_NONE) {
    radio.setSpreadingFactor(10);
    radio.setCodingRate(5);
    radio.setSyncWord(0x12);
  } else {
    Serial.print("LoRa-Fehler beim Starten, Code: ");
    Serial.println(state);
    lora_state_is_ok = false;
  }

  radio.setPacketReceivedAction(rxIsr);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

  WiFi.begin(wifi_ssid);
  int wifi_timeout_ctr = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_timeout_ctr < 20) {
    delay(500);
    wifi_timeout_ctr++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 3600, "pool.ntp.org");

    struct tm timeinfo;
    int time_timeout_ctr = 0;
    while (!getLocalTime(&timeinfo) && time_timeout_ctr < 10) {
      delay(500);
      time_timeout_ctr++;
    }

    if (getLocalTime(&timeinfo)) {
      wifi_and_time_state_is_ok = true;
    }
  }
}

void loop() {
  if (rxFlag) {
    rxFlag = false;
    last_rx_millis = millis();
    Serial.println("Packet detected!");

    LoRaPayload dataIn;
    int state = radio.readData((uint8_t*)&dataIn, sizeof(dataIn));

    if (state == RADIOLIB_ERR_NONE) {
      rx_sensor_state = dataIn.state_watertank_sensor;
      rx_water_level = dataIn.waterlevel;
    }

    if (state == RADIOLIB_ERR_NONE) {
      rx_status_text = getStatusText(rx_sensor_state);
      Serial.printf("\nreceived payload int %i", rx_sensor_state);
      Serial.printf("\nreceived payload float %f", rx_water_level);
      Serial.printf("\nso it is rx_status_text = %s", rx_status_text);

      const int signalDbm = radio.getRSSI();
      const float levelCm = 0.0f;
      const float levelPercent = (rx_water_level > 0.0f) ? rx_water_level : 0.0f;
      pushTankReadingToServer(rx_sensor_state, levelPercent, levelCm, signalDbm);
    }

    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

    if (last_rx_millis > 0 && (millis() - last_rx_millis) > 600000) {
      lora_state_is_ok = false;
      rx_status_text = "??";
      rx_water_level = -1;
    }
  }

  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 12, "Watertank Level (R)");
    u8g2.drawHLine(0, 14, 128);

    u8g2.setCursor(0, 25);
    u8g2.print("STATUS Wtr.Sens: ");
    u8g2.print(rx_status_text);

    u8g2.setCursor(0, 37);
    u8g2.print("LORA: ");
    u8g2.print(lora_state_is_ok ? "OK" : "Err");
    u8g2.setCursor(66, 37);
    u8g2.print("WiFi: ");
    u8g2.print(wifi_and_time_state_is_ok ? "OK" : "Err");

    u8g2.drawHLine(0, 40, 128);

    u8g2.setCursor(0, 51);
    if (rx_sensor_state == STATE_OK || rx_sensor_state == STATE_DEADZONE) {
      u8g2.printf("Water-Level: %5.1f %%", rx_water_level);
    } else {
      u8g2.print("Water-Level: --- %");
    }

    unsigned long age_seconds = (millis() - last_rx_millis) / 1000;
    u8g2.setCursor(48, 61);
    u8g2.printf("age:  %04lu s", age_seconds);

    if (wifi_and_time_state_is_ok == true) {
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        u8g2.setCursor(0, 61);
        u8g2.printf("(%02d:%02d)", timeinfo.tm_hour, timeinfo.tm_min);
      }
    }
  } while (u8g2.nextPage());

  delay(cycle);
}
