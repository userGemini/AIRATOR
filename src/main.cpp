#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include <DFRobot_ESP_EC.h>
//#include <GravityTDS.h>
#include <SoftwareSerial.h>

/* ====================== KONFIGURASI WIFI ====================== */
const char* WIFI_SSID     = "Lab Robotika AI";
const char* WIFI_PASSWORD = "labrobotm101";

/* ================= KONFIGURASI BROKER HIVEMQ =================
   Sama dengan konfigurasi di dashboard (file HTML), tapi ESP32
   pakai MQTT native (TLS) di port 8883, BUKAN WebSocket (8884). */
const char* MQTT_HOST     = "eb4e1d970eb448f99b741a9ba4d6eacc.s1.eu.hivemq.cloud";
const int   MQTT_PORT     = 8883;                 // TLS, MQTT native
const char* MQTT_USER     = "AIRATOR";
const char* MQTT_PASS     = "airatorPPNS1";
const char* MQTT_CLIENT_ID = "esp32-airator-01";  // harus unik per device

/* ========================= TOPIK MQTT ========================= */
const char* TOPIC_SENSORS_JSON = "aquascope/sensors/all";
const char* TOPIC_LOCATION     = "aquascope/sensors/location";

const char* TOPIC_VOLT = "aquascope/power/voltage";
const char* TOPIC_AMP  = "aquascope/power/current";
const char* TOPIC_WATT = "aquascope/power/watt";

const char* TOPIC_VFD_FREQ   = "aquascope/vfd/freq";
const char* TOPIC_VFD_CMD    = "aquascope/vfd/cmd";
const char* TOPIC_VFD_STATUS = "aquascope/vfd/status";

/* Lokasi node default (boleh diubah / dikirim sekali saat boot) */
const double NODE_LAT = -7.283749;//-7.278811
const double NODE_LNG = 112.805076;//112.793503

/* ==================== INTERVAL PENGIRIMAN ==================== */
const unsigned long SENSOR_INTERVAL_MS = 2000;   // kirim data sensor tiap 2 detik
const unsigned long POWER_INTERVAL_MS  = 2000;   // kirim data daya tiap 2 detik

unsigned long lastSensorSend = 0;
unsigned long lastPowerSend  = 0;

/* ==================== STATE VFD ==================== */
bool   vfdRunning = false;
float  vfdFreq    = 30.0;   // Hz, dikontrol dari website (0-50 Hz)

/* ============================================================
   ===================  PWM → 0-10V  KE VFD  ===================
   Mengubah frekuensi (0-50 Hz, dikirim dari website lewat MQTT)
   menjadi tegangan analog 0-10V via modul PWM-to-Voltage, yang
   kemudian masuk ke input kontrol analog VFD fisik.
   ============================================================ */
const int   PWM_PIN        = 25;        // Pin PWM ESP32 (bisa diganti: 25, 26, 27, 32, 33)
const int   PWM_FREQ_HZ    = 1000;      // Frekuensi PWM 1 kHz, aman untuk modul PWM-to-voltage
const int   PWM_RESOLUTION = 12;        // Resolusi 12-bit: 0 - 4095
const int   PWM_CHANNEL    = 0;         // Kanal PWM ESP32
const int   MAX_DUTY       = (1 << PWM_RESOLUTION) - 1;

const float VFD_FREQ_MAX   = 50.0;      // Hz maksimum dari slider website (samakan dgn HTML)
const float VFD_VOLT_MAX   = 10.0;      // Tegangan maksimum modul PWM-to-Voltage

float currentOutputVoltage = 0.0;

// Set tegangan output 0-10V via PWM (dipanggil internal)
void setOutputVoltage(float voltage) {
  if (voltage < 0.0) voltage = 0.0;
  if (voltage > VFD_VOLT_MAX) voltage = VFD_VOLT_MAX;
  currentOutputVoltage = voltage;

  float dutyRatio = voltage / VFD_VOLT_MAX;
  int dutyValue = round(dutyRatio * MAX_DUTY);
  ledcWrite(PWM_CHANNEL, dutyValue);

  Serial.print("[VFD] Tegangan output: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Duty: ");
  Serial.print(dutyRatio * 100.0, 1);
  Serial.print(" % (");
  Serial.print(dutyValue);
  Serial.println(")");
}

// Konversi frekuensi (Hz, 0-50) -> tegangan (0-10V) lalu kirim ke PWM.
// Kalau motor sedang STOP, tegangan dipaksa 0V apa pun frekuensinya.
void applyFrequencyToOutput() {
  if (!vfdRunning) {
    setOutputVoltage(0.0);
    return;
  }
  float voltage = (vfdFreq / VFD_FREQ_MAX) * VFD_VOLT_MAX;
  setOutputVoltage(voltage);
}

/* ==================== STATE SIMULASI SENSOR ====================
   Pakai "random walk": nilai berjalan pelan-pelan di sekitar
   nilai dasar, supaya grafik tidak melompat-lompat aneh.        */
struct SimSensor {
  float value;
  float jitter;   // besar langkah acak tiap update
  float minV, maxV;
};

SimSensor ph        = {7.20, 0.08, 0.0,   14.0};
SimSensor turbidity  = {4.0,  0.6,  0.0,   50.0};
SimSensor tds        = {380,  15.0, 0.0,   2000.0};
SimSensor salinity   = {0.3,  0.05, 0.0,   40.0};
SimSensor doxy       = {6.5,  0.25, 0.0,   20.0};   // dissolved oxygen
SimSensor temp       = {28.0, 0.3,  0.0,   50.0};

SimSensor voltSim    = {220.0, 1.5,  200.0, 240.0};
SimSensor ampSim     = {2.5,   0.15, 0.5,   8.0};

WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

/* ====================== FUNGSI BANTUAN ====================== */

// Gerakkan nilai secara acak kecil-kecilan (random walk) dan clamp ke rentang
float randomWalk(SimSensor &s) {
  float step = ((float)random(-1000, 1000) / 1000.0) * s.jitter;
  s.value += step;
  if (s.value < s.minV) s.value = s.minV;
  if (s.value > s.maxV) s.value = s.maxV;
  return s.value;
}

void connectWiFi() {
  Serial.print("Menyambungkan ke WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi tersambung, IP: ");
  Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("[MQTT IN] ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(msg);

  if (strcmp(topic, TOPIC_VFD_CMD) == 0) {
    msg.trim();
    msg.toUpperCase();
    if (msg == "START") {
      vfdRunning = true;
      mqtt.publish(TOPIC_VFD_STATUS, "RUNNING");
      applyFrequencyToOutput();          // langsung keluarkan tegangan sesuai freq terakhir
    } else if (msg == "STOP") {
      vfdRunning = false;
      mqtt.publish(TOPIC_VFD_STATUS, "STOPPED");
      applyFrequencyToOutput();          // akan dipaksa ke 0V
    }
  }
  else if (strcmp(topic, TOPIC_VFD_FREQ) == 0) {
    float f = msg.toFloat();
    if (f >= 0 && f <= 60) {
      vfdFreq = f;
      Serial.print("Frekuensi VFD diset ke: ");
      Serial.println(vfdFreq);
      applyFrequencyToOutput();          // update tegangan 0-10V sesuai frekuensi baru
    }
  }
}

void publishLocation() {
  StaticJsonDocument<128> doc;
  doc["lat"] = NODE_LAT;
  doc["lng"] = NODE_LNG;
  char buf[128];
  serializeJson(doc, buf);
  mqtt.publish(TOPIC_LOCATION, buf);
}

void connectMQTT() {
  while (!mqtt.connected()) {
    Serial.print("Menyambungkan ke broker HiveMQ...");
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println(" tersambung!");
      mqtt.subscribe(TOPIC_VFD_CMD);
      mqtt.subscribe(TOPIC_VFD_FREQ);

      // Kirim status & lokasi awal
      mqtt.publish(TOPIC_VFD_STATUS, vfdRunning ? "RUNNING" : "STOPPED");
      publishLocation();
      applyFrequencyToOutput();   // sinkronkan output PWM dgn state terakhir
    } else {
      Serial.print(" gagal, rc=");
      Serial.print(mqtt.state());
      Serial.println(" — coba lagi 3 detik...");
      delay(3000);
    }
  }
}

void publishSensors() {
  StaticJsonDocument<256> doc;
  doc["ph"]        = round(randomWalk(ph) * 100) / 100.0;
  doc["turbidity"] = round(randomWalk(turbidity) * 10) / 10.0;
  doc["tds"]       = round(randomWalk(tds));
  doc["salinity"]  = round(randomWalk(salinity) * 100) / 100.0;
  doc["do"]        = round(randomWalk(doxy) * 100) / 100.0;
  doc["temp"]      = round(randomWalk(temp) * 10) / 10.0;

  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(TOPIC_SENSORS_JSON, buf, n);

  Serial.print("[MQTT OUT] ");
  Serial.print(TOPIC_SENSORS_JSON);
  Serial.print(" -> ");
  Serial.println(buf);
}

void publishPower() {
  float v = randomWalk(voltSim);
  float a = randomWalk(ampSim);
  float w = v * a;

  char bufV[16], bufA[16], bufW[16];
  dtostrf(v, 0, 1, bufV);
  dtostrf(a, 0, 2, bufA);
  dtostrf(w, 0, 0, bufW);

  mqtt.publish(TOPIC_VOLT, bufV);
  mqtt.publish(TOPIC_AMP,  bufA);
  mqtt.publish(TOPIC_WATT, bufW);

  Serial.printf("[MQTT OUT] V=%s A=%s W=%s\n", bufV, bufA, bufW);
}

/* ============================== SETUP ============================== */
void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());      // seed acak yang baik di ESP32

  // Setup PWM untuk modul PWM-to-0-10V (kontrol VFD)
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  setOutputVoltage(0.0);          // pastikan motor diam saat boot

  connectWiFi();

  // Untuk produksi sebaiknya pakai sertifikat CA HiveMQ yang valid,
  // tapi setInsecure() dipakai di sini agar simpel untuk simulasi/uji coba.
  espClient.setInsecure();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

/* ============================== LOOP ============================== */
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqtt.connected()) {
    connectMQTT();
  }
  mqtt.loop();

  unsigned long now = millis();

  if (now - lastSensorSend >= SENSOR_INTERVAL_MS) {
    lastSensorSend = now;
    publishSensors();
  }

  if (now - lastPowerSend >= POWER_INTERVAL_MS) {
    lastPowerSend = now;
    publishPower();
  }
}