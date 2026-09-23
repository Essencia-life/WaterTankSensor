#define HELTEC_NO_DISPLAY_INSTANCE
#include <Arduino.h>
#include <heltec_unofficial.h> // Ersetzt Arduino.h, bringt u8g2 und radio mit
#include <U8g2lib.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

// OPEN might-ToDos:
//
// Sender:
//    S.1 rms/better mean-average at Ultrasonic-Sensor-value
//        - currently it is a 5-valid-measures-out-of-10. and the mean of these.
//        - because, ususally sensor returns distance like 83.72, 83.71, 84.01, 84.05, 83.81"
//        - but sometimes also like:  83.72, 69.81, 84.01, 84.05, 83.81"  (so one "Ausreisser" (69,81) is far away from the other values. One value far apart)
//    S.2 water-consumption on waterlevel
//        - there is consumption (unpredictable)
//        - there is two pumps (quite predictable)
//          - one is constantly pouring water (if there is sun) at... xl/hrs
//          - the other is pumping each 30mins water, for max 5 mins.
//        - this two inputs and "one"/or multiple outputs could be used as source for "waterconsumption"
// Receiver:
//    R.1 upload values to server:
//        - needs server (from Ben?)
//    R.2 store values on ESP, if Server-Unavailable?
//        - store as much as usefull? then dump when server is back online?
//
// Server:
//    Serv.1 install a server which is storing data.
//    Serv.2 create averages / comparisions / graphs for display-options
//    Serv.3 provide this "display" views to everybodys-phone.
//
// IDEA: What about getting just an other ESP32 with LORA-Antenna, 
// putting the Same Receiver-SW (may disable wifi) on it, and just put it e.g. in the Fun-Kitchen?
// would be easier to just read the waterlevel via display. no phone / internet / server needed.


enum SensorState {              // JSN-Ultrasonic-Sensor
  STATE_OK,                     // Sensor and Distance: all good.
  STATE_TIMEOUT,                // Sensor could not detect any object. Is Object too far (more than 2,5meters) or Sensor fully Covered and Object laying on the sensor?
  STATE_DEADZONE,               // Kinda okey. waterlevel is close to sensor. Sensor-Deadzone is <22cm...3cm?.
  STATE_DRIFT_ERROR,            // too much sensor-drift in short time (did someone opening/closing the lit or did sensor fallen apart)
  STATE_INIT,                   // (at restart, no valid measure yet)
  //STATE_NEEDS_SERVICE,          // Sensor and System needs manual Service or Reset (unplug power (USB-C-Charger/Powersupply), wait 1 min, replug it. Or Search for further failures, if this did not help)
      // NEEDS SERVICE als status obsolet, weil INIT dann aufleuchtet. Und wenn Init mehr als x-Minuten beim Empfänger registriert wird. dann brauchts service. Sonst darf das system sich hier selbst heilen.
  //STATE_BELOW_PUMP_RESTART_LVL, // Usually pump would have started to pump more water in again, but waterlevel is below this level already
      // dann gäbe es auch ein "above pump stop level".
      // und generell wären die dann alle okey. Und irgendwie könnte man die pumprestartlevel vielleicht anderwo hinterlegen?!
  STATE_OUT_OF_RANGE,           // happens when lit opened, waterlevel seems to be deeper then the tank actually is
};

// Konstanten des ESP-Verkabelung-Ultrasonic-Sensor JSP-SR04T-V33
const int echoPin = 5;
const int trigPin = 4;
const int cycle = 1000; // cycle of calculations/math of ESP-Boards in milliseconds

// Konstanten des JSP-SR04T-Ultraschall-Sensors
const unsigned int distance_deadzone = 22;  // Deadzone des Sensors (bei dem Ultraschallsensor von JSN-SR04T ists wohl 22cm)
const long max_TOF_sens = 30000; // grenze des Sensors (maximale Mess-Reichweite in Mikrosekunden)

// Konstanten der Installation in Essencia Wassertank Sensor. Angaben in Zentimeter
const unsigned int distance_max_depth_watertank = 150; // maximaler, sinnvoller zu messender Wert zwischen Sensor und Tankboden (bzw.) der Wassertank-Tiefe (bis zum Sensor)
const unsigned int height_watertank_0percent = 20;  // bei 10cm (in Essencia) fängt das Wasserrohr an, darunterliegende Wasserstände können nicht gepumpt werden.
const unsigned int height_watertank_100percent = 120; // vom Boden, 120cm (in Essencia) is der Wassertank mit 100% voll betitelt. da der Sensor jedoch 22cm Deadzone hat, ist das mal hier so früh auf 100% definiert. Man müsste den Sensor höher montieren (wie chatgpt/gemini schon gesagt hatte)
const unsigned int distance_watertank_0percent = distance_max_depth_watertank - height_watertank_0percent; // 150 - 10 = 140cm Abstand zum Sensor bei leerem Tank
const unsigned int distance_watertank_100percent = distance_max_depth_watertank - height_watertank_100percent; // 150 - 120 = 30cm Abstand zum Sensor bei vollem Tank

const unsigned int liter_per_cm = 125; // Liter Inhalt pro centimeter: Pi*R*R(dezimeter)*1/10
                               // in Essencia ausmessen! aktuelle Schätzung: 20*20*3,14159/10 = 125

bool toggle_var = true; // toggle var for toggeling output of disance and percentage

// LORA (Funkverbindung)
struct LoRaPayload {
  uint8_t state_watertank_sensor;
  float waterlevel;
  // byte = waterconsumption? (t.b.d.)
  // byte = waterconsumption_overflow_bit? (t.b.d.)
};


// Konstanten der ESP-Ausgabe:
// siehe u8g2.setFont(u8g2_font_6x10_tf); weiter unten im text
#ifdef IS_RECEIVER
// --- EMPFÄNGER CONFIG & VARIABLEN (Muss VOR setup() stehen) ---
#ifndef SERVER_HOST
#define SERVER_HOST "example.com"
#endif
#ifndef SERVER_API_KEY
#define SERVER_API_KEY "replace-me"
#endif

const char* wifi_ssid = "STARLINK";
unsigned long last_rx_millis = 0;
unsigned long last_server_push_millis = 0;
const unsigned long server_push_interval = 5UL * 60UL * 1000UL; // Nur alle 5 Minuten an den Server senden
const unsigned long wifi_reconnect_interval = 10UL * 1000UL;
const unsigned long upload_retry_interval = 10UL * 1000UL;
const size_t upload_queue_capacity = 64;
unsigned long last_wifi_attempt_millis = 0;
unsigned long last_time_sync_attempt_millis = 0;
unsigned long last_upload_attempt_millis = 0;
volatile bool rxFlag = false;
uint8_t rx_sensor_state = STATE_INIT;
float rx_water_level = -1;
String rx_status_text = "Waiting...";
bool wifi_and_time_state_is_ok = false;

struct PendingUpload {
  String payload;
};

PendingUpload upload_queue[upload_queue_capacity];
size_t upload_queue_head = 0;
size_t upload_queue_count = 0;

// FIX 2: Vorwärtsdeklaration für den Compiler und IRAM_ATTR für die ISR auf ESP32
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

String getUtcTimestamp() {
  time_t now = time(nullptr);
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

// Hilfsfunktion zur Textübersetzung der Stati
String getStatusText(uint8_t state) {
  switch(state) {
    case STATE_OK:            return "OK";
    case STATE_TIMEOUT:       return "Err (Timeout)";
    case STATE_DEADZONE:      return "OK (Min. Dist)";
    case STATE_DRIFT_ERROR:   return "Err (Drift)";
    case STATE_OUT_OF_RANGE:  return "Err (Out of Range)";
    default:                  return "Starting...";
  }
}

String buildTankIngestPayload(uint8_t sensorState, float waterLevelPercent, float levelCm, int signalDbm) {
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"deviceId\":\"essencia-water-tank-01\",\"timestamp\":\"%s\",\"state\":%u,\"waterLevelPercent\":%.1f,\"levelCm\":%.1f,\"signalDbm\":%d}",
           getUtcTimestamp().c_str(),
           (unsigned int)sensorState,
           waterLevelPercent,
           levelCm,
           signalDbm);
  return String(payload);
}

bool postPayloadToServer(const String& payload) {
  if (strlen(SERVER_HOST) == 0 || strlen(SERVER_API_KEY) == 0) {
    Serial.println("Server push skipped: SERVER_HOST / SERVER_API_KEY not configured.");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Server push skipped: WiFi disconnected.");
    return false;
  }

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

bool enqueueUpload(const String& payload) {
  if (upload_queue_count >= upload_queue_capacity) {
    Serial.println("Upload queue full; dropping oldest upload.");
    upload_queue[upload_queue_head].payload = "";
    upload_queue_head = (upload_queue_head + 1) % upload_queue_capacity;
    upload_queue_count--;
  }

  size_t tail = (upload_queue_head + upload_queue_count) % upload_queue_capacity;
  upload_queue[tail].payload = payload;
  upload_queue_count++;
  Serial.printf("Upload queued; %u pending.\n", (unsigned int)upload_queue_count);
  return true;
}

bool flushUploadQueue() {
  if (upload_queue_count == 0 || WiFi.status() != WL_CONNECTED) {
    return upload_queue_count == 0;
  }

  unsigned long now = millis();
  if (now - last_upload_attempt_millis < upload_retry_interval) {
    return false;
  }

  while (upload_queue_count > 0 && WiFi.status() == WL_CONNECTED) {
    last_upload_attempt_millis = millis();
    String& payload = upload_queue[upload_queue_head].payload;
    if (!postPayloadToServer(payload)) {
      return false;
    }

    payload = "";
    upload_queue_head = (upload_queue_head + 1) % upload_queue_capacity;
    upload_queue_count--;
    Serial.printf("Upload acknowledged; %u pending.\n", (unsigned int)upload_queue_count);
  }

  return upload_queue_count == 0;
}

bool pushTankReadingToServer(uint8_t sensorState, float waterLevelPercent, float levelCm, int signalDbm) {
  String payload = buildTankIngestPayload(sensorState, waterLevelPercent, levelCm, signalDbm);
  enqueueUpload(payload);
  return flushUploadQueue();
}

void maintainWifiConnection() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    wifi_and_time_state_is_ok = false;
    if (now - last_wifi_attempt_millis >= wifi_reconnect_interval) {
      last_wifi_attempt_millis = now;
      Serial.println("WiFi disconnected; trying to reconnect.");
      WiFi.disconnect();
      WiFi.begin(wifi_ssid);
    }
    return;
  }

  if (!wifi_and_time_state_is_ok &&
      now - last_time_sync_attempt_millis >= wifi_reconnect_interval) {
    last_time_sync_attempt_millis = now;
    configTime(0, 3600, "pool.ntp.org");
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
      wifi_and_time_state_is_ok = true;
      Serial.println("WiFi connected and time synchronized.");
    } else {
      Serial.println("WiFi connected, but time synchronization is still pending.");
    }
  }

  flushUploadQueue();
}

#endif

// Konstanten der LORA- (Long Range Radio Communication ESP)
bool lora_send_waterconsump_ovrflw = false; // overflow-counter for waterconsumtion (integration t.b.d) just needed for reset and receiver-logic.
unsigned long int waterconsump = 0; // water-consumption not integrated yet
unsigned long previousLoRaMillis = 0;           // Speichert den letzten Sendezeitpunkt
const unsigned long lora_send_interval = 30000; // Sendeintervall in Millisekunden (60s wären ziel-wert für konst-betrieb.)
// float acc_usage_today/waterconsumption = 0;      // Auffaddierter Verbrauch / Tag -> bräuchte Uhrzeit. Und will ich den verbrauch hier addieren?
// bool waterconsumption_ovrflw = false;        // auch noch to do für irgendwann.


bool lora_state_is_ok = true;  // Lora state (sender and receiver)
String SensorTextPrint = "";    // Variable für Textausgabe deklariert
String SensorStatus = "";       // Variable für SensorStatus deklariert
SensorState currentSensorState = STATE_INIT;  // Sensorstatus auf Init-State schicken
float distance_filtered = 50.0;  // Globaler Filterwert, init-wert bei 50 cm damit keine 0Divisionen entstehen
float watertank_level_percentage = -1;        // watertank_percentage (-1 init wert == eher unrealistisch bei normalem ablassen, weil das Rohr ja bei 0% ist)
// change watertank level to float or sth. which is 45.3% .. because it is toggeling too much. and it makes too much of a difference on 100 steps.
bool is_first_run = true;       // Flag für Erstinitialisierung des Filters
unsigned int err_info_ctr = 1;


U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 18, /* data=*/ 17);

float percentage_watertank (float distance) 
{
  // Schutz vor Werten außerhalb der definierten Tank-Geometrie
  if (distance >= distance_watertank_0percent) return 0;       // Abstand zu groß -> Tank leer
  if (distance <= distance_watertank_100percent) return 100;   // Abstand zu klein -> Tank voll
  
  // Nutzbarer Bereich (z.B. 140cm - 30cm = 110cm)
  long nutzbare_hoehe = distance_watertank_0percent - distance_watertank_100percent; 
  
  // Aktuelle Wasserhöhe über dem Nullpunkt (z.B. 140cm - 85cm = 55cm)
  float aktuelle_wasserhoehe = distance_watertank_0percent - distance;               
  
  // Erst multiplizieren, dann teilen, um Ganzzahl-Divisionsfehler (0 %) zu vermeiden
  float result = ((aktuelle_wasserhoehe * 100) / nutzbare_hoehe ); 
  return result;
}



void setup() {
  
  // Serial USB serial output baud rate
  Serial.begin(115200);
  
  // ATTENTION // ACHTUNG
  heltec_setup(); // ACHTUNG; NUR AKTIVIEREN, WENN ANTENNEN AN MODUL GEKNÜPFT SIND. SONST DROHT SCHADEN AM CHIP.

  #ifdef IS_SENDER
  // pin Modes of ESP-Controller and PINs for JSN-SR04T-Ultrasonic-Sensor-Board
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  #endif

  // Setup for built-in-Screen of ESP-Controller
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);    // fontsize: 6pix (max. width) + 10pix (max. heigt) for each letter/char

  // LORA Setup
  int state = radio.begin(868.0); // Startet Modul auf 868 MHz (Europa)
  if (state == RADIOLIB_ERR_NONE) {
    radio.setSpreadingFactor(10);
    radio.setCodingRate(5);
    radio.setSyncWord(0x12);      // Dein "Geheimcode", muss beim Empfänger gleich sein
    // PS: Die 4 LoRa-Filter (Sender & Empfänger)
    // 1. Frequenz (z. B. 868.0 MHz): Der physikalische Funkkanal. Sender und Empfänger müssen auf derselben Frequenz arbeiten.
    // 2. Spreading Factor (SF7 bis SF12): Bestimmt die Sendedauer und Signalspreizung. Höherer SF erhöht die Reichweite, senkt aber die Datenrate. Muss exakt übereinstimmen.
    // 3. Bandbreite (BW) & Coding Rate (CR): Die Breite des Funksignals (meist 125 kHz) und das Maß der Fehlerkorrektur (z. B. 4/5). Ohne Übereinstimmung bleibt das Signal für den Empfänger unlesbares Rauschen.
    // 4. Sync Word (Die Netzwerk-ID / 0xE5): 0xE5 für E55ENC1A. Ein Byte im Paket-Header. Der Empfänger filtert damit auf Software-Ebene: Passt das Sync Word nicht zu seiner Konfiguration, verwirft er das Paket sofort.

  } else {
    Serial.print("161: LoRa-Fehler beim Starten, Code: ");
    Serial.println(state);
    lora_state_is_ok = false;
  }

  #ifdef IS_RECEIVER
  // LORA Empfänger-Teil:
  radio.setPacketReceivedAction(rxIsr);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

  // und Wifi versuchen zu aktivieren:
  WiFi.begin(wifi_ssid);
  WiFi.setAutoReconnect(true);
  last_wifi_attempt_millis = millis();
  #endif
}


void loop() {

  #ifdef IS_SENDER

  // Mittelwertbildung über SensorWerte
  unsigned int valid_readings = 0;
  unsigned int ctr_errors = 0;
  float distance_acc = 0; // unsigned long um Überlauf bei der Addition zu vermeiden
  unsigned int last_duration = 0;

  // Max 10 Versuche, um exakt 5 gültige Messwerte zu sammeln
  for (int i = 0; i < 10 && valid_readings < 5; i++) 
  {
    // 1. MESSUNG 
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20); 
    digitalWrite(trigPin, LOW);

    unsigned int duration = pulseIn(echoPin, HIGH, max_TOF_sens);
    last_duration = duration;

    float distance = duration * 0.034 / 2; 
    Serial.printf("\n192: Input-Filter run (%i) was distance %f (cm)", i, distance);
    if (distance != 0)
    {
      distance_acc += distance;
      valid_readings++;
      delay(200); // WICHTIG: Kurze Pause, damit sich das Ultraschall-Echo im Tank legt
    } 
    else 
    {
      ctr_errors++;
      delay(200);
    }
  }

  // 2. MITTELWERT BERECHNEN
  float distance = 0;
  if (valid_readings > 0) 
  {
    distance = distance_acc / valid_readings;
    Serial.printf("\n213: Distance_mean = %f ", distance);
  } 
  else
  {
    distance = 0; // Fallback, falls alle 10 Messungen fehlschlugen
  }
  
  Serial.printf("\n220: Distance = %f , distance_filtered %f,", distance, distance_filtered);
  if (distance == 0) {
    // Sensor somehow disconnected? not sensing anymore!
    currentSensorState = STATE_TIMEOUT;
    SensorStatus = "ERR (Timeout)";
    SensorTextPrint = "Sensor disconnected?";
  } else if (distance <= distance_deadzone) {
    // if out of range, dead-zone Sensor (Tank Full?)
    currentSensorState = STATE_DEADZONE;
    SensorStatus = "OK (Min. Distance)";
    SensorTextPrint = "dist>22cm. If Tank full,is ok"; 
    watertank_level_percentage = percentage_watertank(distance_filtered);
  } else if (distance > distance_max_depth_watertank) {
    // if out of distance, too far away, error
    currentSensorState = STATE_OUT_OF_RANGE;
    SensorStatus = "ERR (> Max. Distance)";
    SensorTextPrint = "dist. > 150cm"; // SensorTextPrint = "dist. > int2str(distance_max_depth_watertank) cm".
  } else if ( ( abs( distance-distance_filtered ) > 10 ) && (is_first_run == false) )
    {
    // Rate-Filter for opening Lit (to have a look or sth.).
    // max accepted change 4cm / cycle
    // would it be better to filter this later? on collected data?
    currentSensorState = STATE_DRIFT_ERROR;
    SensorStatus = "ERR (high Sens drift)";
    SensorTextPrint = "Lit Opened? close & reboot";
  } else if ( distance < distance_max_depth_watertank && distance > distance_deadzone ) // Also Sensor innerhalb der normalen, erwarteten Arbeitsbedingungen
    {
    if ( currentSensorState == STATE_INIT ) 
    {// Lowpass-Filter (90% Altwert, 10% Neuwert)
    // erster Startup Filterwert direkt setzten (Init)
      distance_filtered = distance;
      is_first_run = false;
    } else {
      distance_filtered = ( (distance_filtered * 0.8) + (distance * 0.2) );
    }
    currentSensorState = STATE_OK;
    SensorStatus = "OK";
    SensorTextPrint = String(distance_filtered) + " cm";
    watertank_level_percentage = percentage_watertank(distance_filtered);
  } else {
    currentSensorState = STATE_INIT; // Ist doch wie init...
    SensorStatus = " ... starting up";
    SensorTextPrint = "lit closed? cable conected?";
  }
    
  Serial.printf("\n265: CurrentSensorState = %i ",currentSensorState);
  
  // Wasserstand:
  // vermutlich ist jetzt das meiste integriert.
  // Tests (Restart Sender, Restart Empfänger)
  // Wenn Restart Wassersensor, dann wird neu aufintegriert (Verbrauch vom Tag)
      // Empfänger schaut ob integrierter Tageswert? oder forlaufend Integrierendes? (Überlaufendes) kleiner als letzter Wert ist?
      // Wenn Überlauf und Reset gültig waren (z.B. binnen erwarteten 100 Litern) dann wird überlauf vom Sender beim Empfänger ernstgenommen
      // Wenn Überlauf unerwartet ist, dann wird von einem Restart vom Sender ausgegangen und neu, fortlaufend aufaddiert.
      // Empfänger kennt Tageszeit/Uhrzeit
  // Wenn Restart Empfänger, dann ?? (erstmal Tageswerte für Verbrauch verloren?) Oder man könnte sie sich aus der Cloud/USB-Stick oder so holen.

  // Wasserverbrauch
  // Bei eingeschaltener Pumpe UND Ablauf, wird die Differenz bzw. der Verbrauch währenddessen nicht erfasst. Wäre es sinnvoll "bei steigendem Wasserspiegel" den vorherigen Wasserverbrauch zu verlängern?
  // Kann man die Pumpgeschwindigkeit "eichen" und dann bei geringerer Pumpgeschwindigkeit auf Ablauf Rückschließen?
    // leider gibt es zwei Pumpen. Eine läuft bei Solar licht verfügbar -> Bohrloch -> kleine Pumpgeschwindigkeit
    // zweite Pumpe pumpt heftig, ist ein Dieselgenerator.


  
  // 2.1 LORA zykluszeit abfragen
  unsigned long currentMillis = millis(); // Aktuelle Systemzeit abfragen

  // 2.2. LORA zyklus-Zeit-Vergangen Prüfen, (ob die Differenz zwischen jetzt und der letzten Sendung größer als das Intervall ist
  if (currentMillis - previousLoRaMillis >= lora_send_interval) 
  {
    previousLoRaMillis = currentMillis;
    
    LoRaPayload dataOut;
    dataOut.state_watertank_sensor = currentSensorState;
    dataOut.waterlevel = watertank_level_percentage; // Dein Float-Wert

    // Überträgt einfach den Speicherblock der gesamten Struktur (5 Bytes)
    int lora_tx_state = radio.transmit((uint8_t*)&dataOut, sizeof(dataOut));
    
    if (lora_tx_state == RADIOLIB_ERR_NONE) {
      lora_state_is_ok = true;
    } else {    // Antenne war nicht angeschlossen (im Night-Versuch) und dennoch war state_okey... versteh ich nicht.
      lora_state_is_ok = false;
    }  
    Serial.printf("\n306: Lora state: %i", lora_tx_state);
    Serial.print("\n307: Lora payload:");
    Serial.print(dataOut.state_watertank_sensor);
    Serial.print(dataOut.waterlevel);
  }
  

  toggle_var = !toggle_var; // toggle display with two information distance and percentage

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
    
    //u8g2.setCursor(0, 55); // (max 64)
    if (currentSensorState == STATE_OK or currentSensorState == STATE_DEADZONE)
    {
        u8g2.setCursor(0, 38);
        u8g2.printf("Distance: %6.2f cm", distance_filtered);
        u8g2.setCursor(0, 50); // (max 64)
        u8g2.printf("Water-Level: %5.1f %%", watertank_level_percentage);
        Serial.printf("\n329: Water-Level %f", watertank_level_percentage);
    } 
    else // switch-case for currentSensorState != OK
    {
      u8g2.setCursor(0, 38);
      u8g2.print(SensorTextPrint);
      Serial.println("335: \n... in Error-printout for display");
      u8g2.setCursor(0, 50);
      switch (err_info_ctr)
      {
        case 0: // print currentSensorStatus (is some Error)
          u8g2.print(SensorStatus);
          //err_info_ctr++;
          break;

        case 1: // error solve description
          u8g2.print(SensorTextPrint); 
          //err_info_ctr++;
          break;

        case 2: // raw-TOF-Wert of sensor
          u8g2.print("?TOF?: ");
          u8g2.print(last_duration);
          u8g2.print(" us");
          //err_info_ctr++;
          break;

        case 3: // calculed distance based on TOF:
          u8g2.print("?Dist?(unfilt.): ");
          u8g2.print(distance);
          u8g2.print(" cm");
          //err_info_ctr++;
          break;

        case 4: // distance filtered
          u8g2.print("?Dist?(tpf): ");
          u8g2.print(distance_filtered);
          u8g2.print(" cm");
          //err_info_ctr++;
          break;

        case 5: // theoreth. waterlevel %
          u8g2.print("percent; " + String(watertank_level_percentage) + " %"); // calc waterlevel %
          //err_info_ctr++;
          break;

        case 6: // LoRa-Status (right now only error or ok)
          u8g2.print("LoRa is ok: ");
          u8g2.print(lora_state_is_ok);
          err_info_ctr = 0; // reset err_info_ctr for start over after this last info
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
    
  }
  while ( u8g2.nextPage() );

  err_info_ctr++;
  #endif

  #ifdef IS_RECEIVER
  // --- EMPFÄNGER LOOP (Nur Status & Waterlevel) ---
  maintainWifiConnection();

  // 1. Prüfen, ob ein Paket über den Interrupt registriert wurde
  if (rxFlag) {
    rxFlag = false;
    last_rx_millis = millis();
    Serial.println("Packet detected!");

    LoRaPayload dataIn;
    int state = radio.readData((uint8_t*)&dataIn, sizeof(dataIn));

    if (state == RADIOLIB_ERR_NONE) {
      rx_sensor_state = dataIn.state_watertank_sensor;
      rx_water_level = dataIn.waterlevel; // Ist direkt ein float
    }

    if (state == RADIOLIB_ERR_NONE) {
      rx_sensor_state = rx_sensor_state;
      rx_status_text  = getStatusText(rx_sensor_state);
      Serial.printf("\nreceived payload int %i", rx_sensor_state);
      Serial.printf("\nreceived payload float %f", rx_water_level);
      Serial.printf("\nso it is rx_status_text = %s", rx_status_text);

      const int signalDbm = radio.getRSSI();
      const float levelCm = distance_filtered;
      const float levelPercent = (rx_water_level > 0.0f) ? rx_water_level : 0.0f;

      // Nur alle 5 Minuten an den Server senden, aber das Display weiterhin jede 30s aktualisieren.
      if (millis() - last_server_push_millis >= server_push_interval) {
        last_server_push_millis = millis();
        pushTankReadingToServer(rx_sensor_state, levelPercent, levelCm, signalDbm);
      }
    }
    
    // Radio wieder in den Empfangsmodus versetzen
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);

    if (last_rx_millis > 0 && (millis() - last_rx_millis) > 600000) {
    lora_state_is_ok = false;
    rx_status_text = "??";
    rx_water_level = -1; // Sorgt bei deiner Display-Logik für "-- %"
}
  }

  // noch zu integrieren. Zeitstempel (aus Internet?)
  // display: vergangene Zeit (sek) seit letztem empfangenen Wert (oder seit startup)
  // einbindung ins WLAN.
  // daten mit Zeitstempel in cloud(? wo genau?) packen? oder was?
  // water-Consumtion auch beim Empfänger integrieren?

  // 2. Display aktualisieren

  // Display-Ausgabe optimieren:
  // 1. Reihe Infos (ok, not okey)
  // 2. Reihe Wasserlevel den Tag über verteilt (Kurve)
  // 3. Reihe Wasserverbrauch / std. Balkendiagramm (akkumuliert heute, vsl. heute, vergl. Wochendurchschnitt.)
  // (oder ist das eher die Ausgabe für die Webseite/app?)
  // waterconsumption? (water out?)
  // water in
  // put everything in 1 diagramm? (level + water in & out?)  
  // 128 pix = every 20mins
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 12, "Watertank Level (R)");
    u8g2.drawHLine(0, 14, 128);
    
    // Status-Ausgabe (Info-Zeile 1)
    u8g2.setCursor(0, 25);
    u8g2.print("STATUS Wtr.Sens: ");
    u8g2.print(rx_status_text);


    
    // Status-Ausgabe (Info-Zeile 2)
    u8g2.setCursor(0, 37);
    u8g2.print("LORA: ");
    u8g2.print(lora_state_is_ok? "OK": "Err");
    u8g2.setCursor(66, 37);
    u8g2.print("WiFi: ");
    u8g2.print(wifi_and_time_state_is_ok ? "OK" : "Err");

    u8g2.drawHLine(0, 40, 128);

    // Wasserstand-Ausgabe (Info-Zeile 3)
    u8g2.setCursor(0, 51);
    if (rx_sensor_state == STATE_OK or rx_sensor_state == STATE_DEADZONE)
    {
      //u8g2.print("Water-Level:" + String(rx_water_level) + " %");
      u8g2.printf("Water-Level: %5.1f %%", rx_water_level);
    } else {
      u8g2.print("Water-Level: --- %");
    }

    unsigned long age_seconds = (millis() - last_rx_millis) / 1000;

    u8g2.setCursor(48, 61); // (Info-Zeile 4))
    u8g2.printf("age:  %04lu s", age_seconds);
    
    if (wifi_and_time_state_is_ok == true)
    {
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) 
      {
      u8g2.setCursor(0,61);
      u8g2.printf("(%02d:%02d)", timeinfo.tm_hour, timeinfo.tm_min);
      };
    }
    
  } while ( u8g2.nextPage() );


  // Platzhalter für upload der Werte ins Internet und Co.
  // mit Ben bespreochen:
  //  er schickt nen Server/Adresse.
  //  er kann auch eine konfig zur verfügung stellen /bzgl. Zeit usw)
  //  Er erwartet ein .json
  //  darin:
  //  SensorStatus
  //  WaterLevel (in&)
  //  timestamp (received value)

 
  #endif

  delay(cycle);
  
}