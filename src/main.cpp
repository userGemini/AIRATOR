#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_ESP_EC.h"
#include "GravityTDS.h"
#include <SoftwareSerial.h>

// ============================================================
//  NEXTION — Pure raw Serial2, NO library
// ============================================================
#define NEXTION_RX   16
#define NEXTION_TX   17
#define NEXTION_BAUD 9600

#define COLOR_GREEN  2016
#define COLOR_RED    63488
#define COLOR_WHITE  65535
#define COLOR_YELLOW 65504

bool nextionAvailable = false;
unsigned long lastNextionUpdate = 0;
const unsigned long NEXTION_UPDATE_INTERVAL = 1000;

inline void nxEnd() {
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

void nxCmd(const String &cmd) {
  Serial2.print(cmd);
  nxEnd();
}

void sendNextionGauge(const String &comp, int value) {
  nxCmd(comp + ".val=" + String(value));
  delay(20);
}

void sendNextionText(const String &comp, const String &value) {
  nxCmd(comp + ".txt=\"" + value + "\"");
  delay(20);
}

void sendNextionColor(const String &comp, uint16_t color) {
  nxCmd(comp + ".pco=" + String(color));
  delay(20);
}

bool initNextion() {
  Serial2.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX, NEXTION_TX);
  delay(600);
  nxCmd("bkcmd=0");
  delay(100);
  while (Serial2.available()) Serial2.read();
  Serial.println("Nextion display siap (raw mode).");
  return true;
}

// ============================================================
//  KONFIGURASI WIFI
// ============================================================
const char* WIFI_SSID     = "Lab Robotika AI";
const char* WIFI_PASSWORD = "labrobotm101";

// ============================================================
//  KONFIGURASI BROKER HIVEMQ
// ============================================================
const char* MQTT_HOST      = "eb4e1d970eb448f99b741a9ba4d6eacc.s1.eu.hivemq.cloud";
const int   MQTT_PORT      = 8883;
const char* MQTT_USER      = "AIRATOR";
const char* MQTT_PASS      = "airatorPPNS1";
const char* MQTT_CLIENT_ID = "esp32-airator-01";

// ============================================================
//  TOPIK MQTT
// ============================================================
const char* TOPIC_SENSORS_JSON = "aquascope/sensors/all";
const char* TOPIC_LOCATION     = "aquascope/sensors/location";
const char* TOPIC_VOLT         = "aquascope/power/voltage";
const char* TOPIC_AMP          = "aquascope/power/current";
const char* TOPIC_WATT         = "aquascope/power/watt";
const char* TOPIC_VFD_FREQ     = "aquascope/vfd/freq";
const char* TOPIC_VFD_CMD      = "aquascope/vfd/cmd";
const char* TOPIC_VFD_STATUS   = "aquascope/vfd/status";

// Lokasi node
const double NODE_LAT =  -7.283749;
const double NODE_LNG = 112.805076;

// ============================================================
//  INTERVAL
// ============================================================
const unsigned long SENSOR_INTERVAL_MS = 2000;   // baca sensor + publish JSON
const unsigned long POWER_INTERVAL_MS  = 2000;   // publish V/A/W (masih simulasi)

unsigned long lastSensorSend = 0;
unsigned long lastPowerSend  = 0;

// ============================================================
//  STATE VFD
// ============================================================
bool  vfdRunning = false;
float vfdFreq    = 30.0;

// ============================================================
//  PWM → Konversi 0-50Hz menjadi Voltase 0-10V
// ============================================================
const int   PWM_PIN        = 25;
const int   PWM_FREQ_HZ    = 1000;
const int   PWM_RESOLUTION = 12;
const int   PWM_CHANNEL    = 0;
const int   MAX_DUTY       = (1 << PWM_RESOLUTION) - 1;
const float VFD_FREQ_MAX   = 50.0;
const float VFD_VOLT_MAX   = 10.0;

float currentOutputVoltage = 0.0;

void setOutputVoltage(float voltage) {
  if (voltage < 0.0)          voltage = 0.0;
  if (voltage > VFD_VOLT_MAX) voltage = VFD_VOLT_MAX;
  currentOutputVoltage = voltage;
  int dutyValue = (int)round((voltage / VFD_VOLT_MAX) * MAX_DUTY);
  ledcWrite(PWM_CHANNEL, dutyValue);
  Serial.printf("[VFD PWM] %.2f V | duty %d\n", voltage, dutyValue);
}

void applyFrequencyToOutput() {
  if (!vfdRunning) {
    setOutputVoltage(0.0);
    return;
  }
  float targetVolt = (vfdFreq / VFD_FREQ_MAX) * VFD_VOLT_MAX;
  setOutputVoltage(targetVolt);
}

// ============================================================
//  TOMBOL FISIK & RELAY KENDALI UTAMA
// ============================================================
#define PIN_PB_START 14  // Tombol NO (Hubungkan ke GND)
#define PIN_PB_STOP  27  // Tombol NC (Hubungkan ke GND)
#define PIN_RELAY    32  // Output Relay ke VFD/Beban

// Pusat Logika Sinkronisasi Start/Stop Sistem
void updateVfdState(bool startSystem) {
  vfdRunning = startSystem;
  
  if (vfdRunning) {
    digitalWrite(PIN_RELAY, HIGH); // Aktifkan Relay (Ganti ke LOW jika modul Active-Low)
    applyFrequencyToOutput();      // Keluarkan voltase sesuai setpoint Hz
    Serial.println("[SYSTEM] VFD -> RUNNING (Relay ON)");
  } else {
    digitalWrite(PIN_RELAY, LOW);  // Matikan Relay (Ganti ke HIGH jika modul Active-Low)
    setOutputVoltage(0.0);         // Potong voltase langsung ke 0V
    Serial.println("[SYSTEM] VFD -> STOPPED (Relay OFF)");
  }
}

void checkPhysicalButtons() {
  // Membaca PB START (Normally Open) -> Ditekan = LOW
  if (digitalRead(PIN_PB_START) == LOW && !vfdRunning) {
    delay(50); // Debounce manual singkat
    if (digitalRead(PIN_PB_START) == LOW) {
      updateVfdState(true);
      WiFiClientSecure espClient; // (Inisialisasi referensi untuk objek mqtt)
    }
  }
  
  // Membaca PB STOP (Normally Close) -> Ditekan = Sirkuit Putus = HIGH
  if (digitalRead(PIN_PB_STOP) == HIGH && vfdRunning) {
    delay(50); // Debounce manual singkat
    if (digitalRead(PIN_PB_STOP) == HIGH) {
      updateVfdState(false);
    }
  }
}

/* ============================================================
   ===============      SENSOR AIR ASLI       =============
   ============================================================ */
#define PH_PIN          35
#define DO_PIN          34
#define SALINITY_PIN    39
#define TDS_PIN         36
#define TURBIDITY_PIN_1 21   // RX SoftwareSerial
#define TURBIDITY_PIN_2 22   // TX SoftwareSerial
#define TEMP_PIN        33   // DS18B20

#define PH_MIN 0.0
#define PH_MAX 14.0
#define DO_MIN 0.0
#define DO_MAX 20.0
#define TDS_MIN 0
#define TDS_MAX 1000
#define TURBIDITY_MIN 0.0
#define TURBIDITY_MAX 4000.0
#define EC_MIN 0.0
#define EC_MAX 20.0

GravityTDS tdsSensor;
OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensors(&oneWire);
SoftwareSerial turbiditySerial(TURBIDITY_PIN_1, TURBIDITY_PIN_2);
DFRobot_ESP_EC ec;

float acidVoltage    = 2510;
float neutralVoltage = 1170;
float Tcal1 = 25.0;
float Vsat1_mV = 1300;
float Tcal2 = 30.0;
float Vsat2_mV = 1200;
const float K_TDS = 0.5f;

const float DOsatTable_mgL[41] = {
  14.6,14.2,13.8,13.5,13.1,12.8,12.5,12.1,11.8,11.5,
  11.3,11.0,10.8,10.5,10.3,10.1, 9.9, 9.7, 9.5, 9.3,
  9.1, 8.9, 8.7, 8.6, 8.4, 8.3, 8.1, 8.0, 7.8, 7.7,
  7.6, 7.5, 7.4, 7.3, 7.2, 7.1, 7.0, 6.9, 6.8, 6.7,
  6.6
};

const uint8_t READ_DIRTY[5] = {0x18, 0x05, 0x00, 0x01, 0x0D};

volatile float phValue        = 0.0;
volatile float doValue        = 0.0;
volatile float waterTemp      = 0.0;
volatile float turbidityValue = 0.0;
volatile float tdsValue       = 0.0;
volatile float ecValue        = 0.0;
volatile float salinityValue  = 0.0;

uint32_t readMilliVoltsAvg(int pin, int n = 64, uint32_t d_us = 500) {
  uint64_t sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogReadMilliVolts(pin);
    if (d_us) delayMicroseconds(d_us);
  }
  return (uint32_t)(sum / n);
}

float Vsat_at_T(float T) {
  if (fabs(Tcal2 - Tcal1) < 1e-6) return Vsat1_mV;
  float slope = (Vsat2_mV - Vsat1_mV) / (Tcal2 - Tcal1);
  return Vsat1_mV + slope * (T - Tcal1);
}

float DOsat_fromTable(float T) {
  int ti = constrain((int)roundf(T), 0, 40);
  return DOsatTable_mgL[ti];
}

void readAllSensors() {
  tempSensors.requestTemperatures();
  waterTemp = tempSensors.getTempCByIndex(0);

  float voltage_pH = analogRead(PH_PIN) / 4095.0 * 3300;
  float slope     = (7.0 - 4.0) / ((neutralVoltage - 1500) / 3.0 - (acidVoltage - 1500) / 3.0);
  float intercept = 7.0 - slope * (neutralVoltage - 1500) / 3.0;
  phValue = slope * (voltage_pH - 1500) / 3.0 + intercept;
  phValue = constrain(phValue, PH_MIN, PH_MAX);

  tdsSensor.setTemperature(waterTemp);
  tdsSensor.update();
  tdsValue = constrain(tdsSensor.getTdsValue(), TDS_MIN, TDS_MAX);

  uint32_t mv_ec = readMilliVoltsAvg(SALINITY_PIN, 32);
  ecValue = constrain(ec.readEC((float)mv_ec, waterTemp), EC_MIN, EC_MAX);
  salinityValue = ecValue * K_TDS;

  uint32_t mV = readMilliVoltsAvg(DO_PIN);
  float Vsat_mV   = Vsat_at_T(waterTemp);
  float DOsat_mgL = DOsat_fromTable(waterTemp);
  doValue = (Vsat_mV > 0) ? ((float)mV / Vsat_mV) * DOsat_mgL : 0;
  if (doValue < 0) doValue = 0;
  doValue = constrain(doValue, DO_MIN, DO_MAX);

  turbiditySerial.write(READ_DIRTY, 5);
  delay(50);
  if (turbiditySerial.available() >= 5) {
    uint8_t f[5];
    for (int i = 0; i < 5; i++) f[i] = turbiditySerial.read();
    if (f[0] == 0x18 && f[1] == 0x05 && f[4] == 0x0D) {
      turbidityValue = f[3];
    } else {
      Serial.println("⚠️  Frame turbidity tidak valid");
    }
  }
  turbidityValue = constrain(turbidityValue, TURBIDITY_MIN, TURBIDITY_MAX);

  Serial.printf("[SENSOR] pH=%.2f DO=%.2f Temp=%.1fC Turb=%.0f TDS=%.0f EC=%.2f Sal=%.2f\n",
                phValue, doValue, waterTemp, turbidityValue, tdsValue, ecValue, salinityValue);
}

/* ============================================================
   DAYA LISTRIK — Simulasi
   ============================================================ */
struct SimSensor { float value, jitter, minV, maxV; };
SimSensor voltSim = {220.0, 1.5,  200.0, 240.0};
SimSensor ampSim  = {2.5,   0.15, 0.5,   8.0};

float randomWalk(SimSensor &s) {
  s.value += ((float)random(-1000, 1000) / 1000.0f) * s.jitter;
  if (s.value < s.minV) s.value = s.minV;
  if (s.value > s.maxV) s.value = s.maxV;
  return s.value;
}

// ============================================================
//  WIFI & MQTT
// ============================================================
WiFiClientSecure espClient;
PubSubClient     mqtt(espClient);

void connectWiFi() {
  Serial.print("Menyambungkan ke WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  sendNextionText("tWifi", "Connecting...");
  sendNextionColor("tWifi", COLOR_YELLOW);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi tersambung, IP: ");
  Serial.println(WiFi.localIP());

  sendNextionText("tWifi", "Connected");
  sendNextionColor("tWifi", COLOR_GREEN);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.printf("[MQTT IN] %s -> %s\n", topic, msg.c_str());

  if (strcmp(topic, TOPIC_VFD_CMD) == 0) {
    msg.trim(); msg.toUpperCase();
    if (msg == "START") {
      updateVfdState(true);
      mqtt.publish(TOPIC_VFD_STATUS, "RUNNING");
    } else if (msg == "STOP") {
      updateVfdState(false);
      mqtt.publish(TOPIC_VFD_STATUS, "STOPPED");
    }
  }
  else if (strcmp(topic, TOPIC_VFD_FREQ) == 0) {
    float f = msg.toFloat();
    if (f >= 0 && f <= VFD_FREQ_MAX) {
      vfdFreq = f;
      Serial.printf("Frekuensi VFD diperbarui dari MQTT: %.1f Hz\n", vfdFreq);
      applyFrequencyToOutput();
    }
  }
}

void publishLocation() {
  JsonDocument doc;
  doc["lat"] = NODE_LAT;
  doc["lng"] = NODE_LNG;
  char buf[128];
  serializeJson(doc, buf);
  mqtt.publish(TOPIC_LOCATION, buf);
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Menyambungkan ke broker HiveMQ...");
    sendNextionText("tBroker", "Connecting...");
    sendNextionColor("tBroker", COLOR_YELLOW);

    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println(" tersambung!");
      sendNextionText("tBroker", "Connected");
      sendNextionColor("tBroker", COLOR_GREEN);

      mqtt.subscribe(TOPIC_VFD_CMD);
      mqtt.subscribe(TOPIC_VFD_FREQ);
      mqtt.publish(TOPIC_VFD_STATUS, vfdRunning ? "RUNNING" : "STOPPED");
      publishLocation();
      updateVfdState(vfdRunning); 
    } else {
      Serial.printf(" gagal rc=%d, coba lagi 3 detik...\n", mqtt.state());
      sendNextionText("tBroker", "Disconnected");
      sendNextionColor("tBroker", COLOR_RED);
      delay(3000);
    }
  }
}

void publishSensors() {
  JsonDocument doc;
  doc["ph"]        = round(phValue * 100) / 100.0;
  doc["turbidity"] = round(turbidityValue * 10) / 10.0;
  doc["tds"]       = round(tdsValue);
  doc["salinity"]  = round(salinityValue * 100) / 100.0;
  doc["do"]        = round(doValue * 100) / 100.0;
  doc["temp"]      = round(waterTemp * 10) / 10.0;
  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(TOPIC_SENSORS_JSON, buf, n);
}

void publishPower() {
  float v = randomWalk(voltSim);
  float a = randomWalk(ampSim);
  char bufV[16], bufA[16], bufW[16];
  dtostrf(v,     0, 1, bufV);
  dtostrf(a,     0, 2, bufA);
  dtostrf(v * a, 0, 0, bufW);
  mqtt.publish(TOPIC_VOLT, bufV);
  mqtt.publish(TOPIC_AMP,  bufA);
  mqtt.publish(TOPIC_WATT, bufW);
}

// ============================================================
//  UPDATE NEXTION
// ============================================================
void updateNextionDisplay() {
  if (!nextionAvailable) return;

  sendNextionGauge("jpH",        (int)map((long)(phValue * 10), (long)(PH_MIN * 10), (long)(PH_MAX * 10), 0, 180));
  sendNextionGauge("jDO",        (int)map((long)(doValue * 10), (long)(DO_MIN * 10), (long)(DO_MAX * 10), 0, 180));
  sendNextionGauge("jTemp",      (int)map((long)waterTemp, 0, 50, 0, 180));
  sendNextionGauge("jTDS",       (int)map((long)tdsValue, TDS_MIN, TDS_MAX, 0, 180));
  sendNextionGauge("jTurbidity", (int)map((long)turbidityValue, 0, 1000, 0, 180));
  sendNextionGauge("jSalinity",  (int)map((long)(salinityValue * 10), 0, 200, 0, 180));

  sendNextionText("tpHValue",   String(phValue, 2));
  sendNextionText("tDOValue",   String(doValue, 2));
  sendNextionText("tTempValue", String(waterTemp, 1));
  sendNextionText("tTDSValue",  String(tdsValue, 0));
  sendNextionText("tTurbValue", String(turbidityValue, 0));
  sendNextionText("tSalValue",  String(salinityValue, 2));

  sendNextionText("tVFDFreq",   String(vfdFreq, 1) + " Hz");
  sendNextionText("tVFDStatus", vfdRunning ? "RUNNING" : "STOPPED");
  sendNextionColor("tVFDStatus", vfdRunning ? COLOR_GREEN : COLOR_RED);

  sendNextionText("tVolt", String(voltSim.value, 1) + " V");
  sendNextionText("tAmp",  String(ampSim.value,  2) + " A");
  sendNextionText("tWatt", String(voltSim.value * ampSim.value, 0) + " W");

  if (WiFi.status() == WL_CONNECTED) {
    sendNextionText("tWifi", "Connected");
    sendNextionColor("tWifi", COLOR_GREEN);
  } else {
    sendNextionText("tWifi", "Disconnected");
    sendNextionColor("tWifi", COLOR_RED);
  }

  if (mqtt.connected()) {
    sendNextionText("tBroker", "Connected");
    sendNextionColor("tBroker", COLOR_GREEN);
  } else {
    sendNextionText("tBroker", "Disconnected");
    sendNextionColor("tBroker", COLOR_RED);
  }
}

void checkNextionInput() {
  if (Serial2.available() > 0) {
    String inputData = Serial2.readStringUntil('\n');
    inputData.trim();

    if (inputData.startsWith("cmd:")) {
      String cmd = inputData.substring(4);
      if (cmd == "START") {
        updateVfdState(true);
      } else if (cmd == "STOP") {
        updateVfdState(false);
      }

      if (mqtt.connected()) {
        mqtt.publish(TOPIC_VFD_STATUS, vfdRunning ? "RUNNING" : "STOPPED");
      }
    }
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());

  // --- Konfigurasi Hardware Tombol & Relay ---
  pinMode(PIN_PB_START, INPUT_PULLUP);
  pinMode(PIN_PB_STOP, INPUT_PULLUP);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW); // Awal booting relay mati

  // --- VFD via PWM-to-0-10V ---
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  setOutputVoltage(0.0);

  // --- Inisialisasi sensor air ASLI ---
  analogSetPinAttenuation(DO_PIN, ADC_11db);
  analogSetPinAttenuation(SALINITY_PIN, ADC_11db);
  analogReadResolution(12);

  tdsSensor.setPin(TDS_PIN);
  tdsSensor.setAref(3.3);
  tdsSensor.setAdcRange(4095);
  tdsSensor.begin();

  ec.begin();

  tempSensors.begin();
  tempSensors.setResolution(12);

  turbiditySerial.begin(9600);

  // --- Nextion & WiFi/MQTT ---
  nextionAvailable = initNextion();
  connectWiFi();

  espClient.setInsecure();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected())             connectMQTT();
  mqtt.loop();

  checkNextionInput();
  checkPhysicalButtons();

  unsigned long now = millis();

  if (now - lastSensorSend >= SENSOR_INTERVAL_MS) {
    lastSensorSend = now;
    readAllSensors();   // baca sensor air ASLI
    publishSensors();   // kirim hasil baca ke MQTT
  }

  if (now - lastNextionUpdate >= NEXTION_UPDATE_INTERVAL) {
    lastNextionUpdate = now;
    updateNextionDisplay();
  }

  if (now - lastPowerSend >= POWER_INTERVAL_MS) {
    lastPowerSend = now;
    publishPower();
  }
}