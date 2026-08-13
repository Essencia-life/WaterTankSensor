#include <Arduino.h>
#include <heltec_unofficial.h>
#include <U8g2lib.h>
#include "WaterTankCommon.h"

bool toggle_var = true;
String SensorTextPrint = "";
String SensorStatus = "";
SensorState currentSensorState = STATE_INIT;
float distance_filtered = 50.0f;
float watertank_level_percentage = -1.0f;
bool is_first_run = true;
unsigned int err_info_ctr = 1;
unsigned long previousLoRaMillis = 0;
bool lora_state_is_ok = true;
unsigned int last_duration = 0;

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 18, /* data=*/ 17);

void setup() {
  Serial.begin(115200);
  heltec_setup();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

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
}

void loop() {
  unsigned int valid_readings = 0;
  unsigned int ctr_errors = 0;
  float distance_acc = 0;

  for (int i = 0; i < 10 && valid_readings < 5; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigPin, LOW);

    unsigned int duration = pulseIn(echoPin, HIGH, max_TOF_sens);
    last_duration = duration;

    float distance = duration * 0.034 / 2;
    Serial.printf("\nInput-Filter run (%i) was distance %f (cm)", i, distance);
    if (distance != 0) {
      distance_acc += distance;
      valid_readings++;
      delay(200);
    } else {
      ctr_errors++;
      delay(200);
    }
  }

  float distance = 0;
  if (valid_readings > 0) {
    distance = distance_acc / valid_readings;
    Serial.printf("\nDistance_mean = %f ", distance);
  } else {
    distance = 0;
  }

  Serial.printf("\nDistance = %f , distance_filtered %f,", distance, distance_filtered);
  if (distance == 0) {
    currentSensorState = STATE_TIMEOUT;
    SensorStatus = "ERR (Timeout)";
    SensorTextPrint = "Sensor disconnected?";
  } else if (distance <= distance_deadzone) {
    currentSensorState = STATE_DEADZONE;
    SensorStatus = "OK (Min. Distance)";
    SensorTextPrint = "dist>22cm. If Tank full,is ok";
    watertank_level_percentage = percentage_watertank(distance_filtered);
  } else if (distance > distance_max_depth_watertank) {
    currentSensorState = STATE_OUT_OF_RANGE;
    SensorStatus = "ERR (> Max. Distance)";
    SensorTextPrint = "dist. > 150cm";
  } else if ((abs(distance - distance_filtered) > 10) && (is_first_run == false)) {
    currentSensorState = STATE_DRIFT_ERROR;
    SensorStatus = "ERR (high Sens drift)";
    SensorTextPrint = "Lit Opened? close & reboot";
  } else if (distance < distance_max_depth_watertank && distance > distance_deadzone) {
    if (currentSensorState == STATE_INIT) {
      distance_filtered = distance;
      is_first_run = false;
    } else {
      distance_filtered = ((distance_filtered * 0.8) + (distance * 0.2));
    }
    currentSensorState = STATE_OK;
    SensorStatus = "OK";
    SensorTextPrint = String(distance_filtered) + " cm";
    watertank_level_percentage = percentage_watertank(distance_filtered);
  } else {
    currentSensorState = STATE_INIT;
    SensorStatus = " ... starting up";
    SensorTextPrint = "lit closed? cable conected?";
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousLoRaMillis >= 30000) {
    previousLoRaMillis = currentMillis;

    LoRaPayload dataOut;
    dataOut.state_watertank_sensor = currentSensorState;
    dataOut.waterlevel = watertank_level_percentage;

    int lora_tx_state = radio.transmit((uint8_t*)&dataOut, sizeof(dataOut));
    if (lora_tx_state == RADIOLIB_ERR_NONE) {
      lora_state_is_ok = true;
    } else {
      lora_state_is_ok = false;
    }

    Serial.printf("\nLora state: %i", lora_tx_state);
    Serial.print("\nLora payload:");
    Serial.print(dataOut.state_watertank_sensor);
    Serial.print(dataOut.waterlevel);
  }

  toggle_var = !toggle_var;

  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 12, "Wassertank-Sensor (S)");
    u8g2.drawHLine(0, 14, 128);
    u8g2.setCursor(72, 26);
    u8g2.print("LORA: ");
    u8g2.print(lora_state_is_ok ? "OK" : "Err");
    u8g2.setCursor(0, 26);
    u8g2.print("STATUS: ");
    u8g2.print(SensorStatus);

    if (currentSensorState == STATE_OK || currentSensorState == STATE_DEADZONE) {
      u8g2.setCursor(0, 38);
      u8g2.printf("Distance: %6.2f cm", distance_filtered);
      u8g2.setCursor(0, 50);
      u8g2.printf("Water-Level: %5.1f %%", watertank_level_percentage);
      Serial.printf("\nWater-Level %f", watertank_level_percentage);
    } else {
      u8g2.setCursor(0, 38);
      u8g2.print(SensorTextPrint);
      u8g2.setCursor(0, 50);
      switch (err_info_ctr) {
        case 0:
          u8g2.print(SensorStatus);
          break;
        case 1:
          u8g2.print(SensorTextPrint);
          break;
        case 2:
          u8g2.print("?TOF?: ");
          u8g2.print(last_duration);
          u8g2.print(" us");
          break;
        case 3:
          u8g2.print("?Dist?(unfilt.): ");
          u8g2.print(distance);
          u8g2.print(" cm");
          break;
        case 4:
          u8g2.print("?Dist?(tpf): ");
          u8g2.print(distance_filtered);
          u8g2.print(" cm");
          break;
        case 5:
          u8g2.print("percent; " + String(watertank_level_percentage) + " %");
          break;
        case 6:
          u8g2.print("LoRa is ok: ");
          u8g2.print(lora_state_is_ok);
          err_info_ctr = 0;
          break;
        default:
          err_info_ctr = 0;
          break;
      }
    }

    u8g2.setCursor(0, 62);
    u8g2.print("D:");
    u8g2.print(distance);
    u8g2.print(" ->D(f); ");
    u8g2.print(distance_filtered);
  } while (u8g2.nextPage());

  err_info_ctr++;
  delay(cycle);
}
