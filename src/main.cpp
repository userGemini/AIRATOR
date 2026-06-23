#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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

// Kirim akhiran 3x 0xFF (wajib setiap perintah Nextion)
inline void nxEnd() {
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

// Kirim perintah raw string
void nxCmd(const String &cmd) {
  Serial2.print(cmd);
  nxEnd();
}

// Set property .val (NexGauge)
void sendNextionGauge(const String &comp, int value) {
  nxCmd(comp + ".val=" + String(value));
  delay(20);
}

// Set property .txt (NexText)
void sendNextionText(const String &comp, const String &value) {
  nxCmd(comp + ".txt=\"" + value + "\"");
  delay(20);
}

// Set property .pco (warna teks)
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
const unsigned long SENSOR_INTERVAL_MS = 2000;
const unsigned long POWER_INTERVAL_MS  = 2000;

unsigned long lastSensorSend = 0;
unsigned long lastPowerSend  = 0;

// ============================================================
//  STATE VFD
// ============================================================
bool  vfdRunning = false;
float vfdFreq    = 30.0; // Input keypad (0-50 Hz)

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
  // Rumus: (Frekuensi_Input / 50Hz) * 10V = Output Voltase fisik Pin 25
  float targetVolt = (vfdFreq / VFD_FREQ_MAX) * VFD_VOLT_MAX;
  setOutputVoltage(targetVolt);
}

// ============================================================
//  SIMULASI SENSOR (random walk)
// ============================================================
struct SimSensor {
  float value, jitter, minV, maxV;
};

SimSensor ph        = {7.20,  0.08, 0.0,   14.0};
SimSensor turbidity = {4.0,   0.6,  0.0,   50.0};
SimSensor tds       = {380,   15.0, 0.0, 2000.0};
SimSensor salinity  = {0.3,   0.05, 0.0,   40.0};
SimSensor doxy      = {6.5,   0.25, 0.0,   20.0};
SimSensor temp      = {28.0,  0.3,  0.0,   50.0};
SimSensor voltSim   = {220.0, 1.5,  200.0, 240.0};
SimSensor ampSim    = {2.5,   0.15, 0.5,   8.0};

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
  Serial.printf("[MQTT IN] %s -> %s\n", topic, msg.c_str());

  if (strcmp(topic, TOPIC_VFD_CMD) == 0) {
    msg.trim(); msg.toUpperCase();
    if (msg == "START") {
      vfdRunning = true;
      mqtt.publish(TOPIC_VFD_STATUS, "RUNNING");
      applyFrequencyToOutput();
    } else if (msg == "STOP") {
      vfdRunning = false;
      mqtt.publish(TOPIC_VFD_STATUS, "STOPPED");
      applyFrequencyToOutput();
    }
  }
  else if (strcmp(topic, TOPIC_VFD_FREQ) == 0) {
    float f = msg.toFloat();
    if (f >= 0 && f <= VFD_FREQ_MAX) {
      vfdFreq = f;
      Serial.printf("Frekuensi VFD via MQTT: %.1f Hz\n", vfdFreq);
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
    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
      Serial.println(" tersambung!");
      mqtt.subscribe(TOPIC_VFD_CMD);
      mqtt.subscribe(TOPIC_VFD_FREQ);
      mqtt.publish(TOPIC_VFD_STATUS, vfdRunning ? "RUNNING" : "STOPPED");
      publishLocation();
      applyFrequencyToOutput();
    } else {
      Serial.printf(" gagal rc=%d, coba lagi 3 detik...\n", mqtt.state());
      delay(3000);
    }
  }
}

void publishSensors() {
  JsonDocument doc;
  doc["ph"]        = round(randomWalk(ph)        * 100) / 100.0;
  doc["turbidity"] = round(randomWalk(turbidity)  * 10)  / 10.0;
  doc["tds"]       = round(randomWalk(tds));
  doc["salinity"]  = round(randomWalk(salinity)   * 100) / 100.0;
  doc["do"]        = round(randomWalk(doxy)        * 100) / 100.0;
  doc["temp"]      = round(randomWalk(temp)        * 10)  / 10.0;
  char buf[256];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(TOPIC_SENSORS_JSON, buf, n);
  Serial.printf("[MQTT OUT] %s -> %s\n", TOPIC_SENSORS_JSON, buf);
}

void publishPower() {
  float v = randomWalk(voltSim);
  float a = randomWalk(ampSim);
  char bufV[16], bufA[16], bufW[16];
  dtostrf(v,       0, 1, bufV);
  dtostrf(a,       0, 2, bufA);
  dtostrf(v * a,   0, 0, bufW);
  mqtt.publish(TOPIC_VOLT, bufV);
  mqtt.publish(TOPIC_AMP,  bufA);
  mqtt.publish(TOPIC_WATT, bufW);
  Serial.printf("[MQTT OUT] V=%s A=%s W=%s\n", bufV, bufA, bufW);
}

// ============================================================
//  UPDATE NEXTION — Versi Full TEXT & Gauge Responsif
// ============================================================
void updateNextionDisplay() {
  if (!nextionAvailable) return;

  // --- Gauge ---
  sendNextionGauge("jpH",        (int)map((long)(ph.value       * 10), 0, 100,  0, 180));
  sendNextionGauge("jDO",        (int)map((long)(doxy.value     * 10), 0, 100,  0, 180));
  sendNextionGauge("jTemp",      (int)map((long) temp.value,           0, 40,   0, 180));
  sendNextionGauge("jTDS",       (int)map((long) tds.value,            0, 1000, 0, 180));
  sendNextionGauge("jTurbidity", (int)map((long) turbidity.value,      0, 25,   0, 180));
  sendNextionGauge("jSalinity",  (int)map((long)(salinity.value * 10), 0, 50,   0, 180));

  // --- Tampilan Angka Sensor ke Komponen Tipe TEXT ---
  sendNextionText("tpHValue",   String(ph.value, 1));        
  sendNextionText("tDOValue",   String(doxy.value, 1));      
  sendNextionText("tTempValue", String(temp.value, 1));      
  sendNextionText("tTDSValue",  String(tds.value, 0));       
  sendNextionText("tTurbValue", String(turbidity.value, 0));  
  sendNextionText("tSalValue",  String(salinity.value, 1));   

  // --- Penayangan Frekuensi Aktif & Status VFD ---
  sendNextionText("tVFDFreq",   String(vfdFreq, 1) + " Hz");
  sendNextionText("tVFDStatus", vfdRunning ? "RUNNING" : "STOPPED");
  sendNextionColor("tVFDStatus", vfdRunning ? COLOR_GREEN : COLOR_RED);

  // --- Daya listrik ---
  sendNextionText("tVolt", String(voltSim.value, 1) + " V");
  sendNextionText("tAmp",  String(ampSim.value,  2) + " A");
  sendNextionText("tWatt", String(voltSim.value * ampSim.value, 0) + " W");
}

// ============================================================
//  FUNGSI MEMBACA INPUT KEYPAD DARI NEXTION (SERIAL2)
// ============================================================
void checkNextionInput() {
  if (Serial2.available() > 0) {
    String inputData = Serial2.readStringUntil('\n'); 
    inputData.trim(); 

    // 1. Membaca Perintah START / STOP dari Tombol Nextion
    if (inputData.startsWith("cmd:")) {
      String cmd = inputData.substring(4);
      if (cmd == "START") {
        vfdRunning = true;
        Serial.println("[Nextion IN] Tombol START Ditekan");
      } else if (cmd == "STOP") {
        vfdRunning = false;
        Serial.println("[Nextion IN] Tombol STOP Ditekan");
      }
      applyFrequencyToOutput(); // Terapkan perubahan voltase instan
    }
    
    // 2. Membaca Perintah Angka Frekuensi dari Keypad Nextion
    else if (inputData.startsWith("hz:")) {
      String valueStr = inputData.substring(3); 
      float rawValue = valueStr.toFloat(); 
      
      // KUNCI PERBAIKAN: Jika nilai dari Nextion > 50, berarti itu nilai mentah x10 (misal 450 -> 45.0)
      float newFreq = rawValue;
      if (rawValue > VFD_FREQ_MAX) {
        newFreq = rawValue / 10.0; 
      }
      
      // Validasi rentang aman frekuensi (0 s.d 50 Hz)
      if (newFreq >= 0.0 && newFreq <= VFD_FREQ_MAX) {
        vfdFreq = newFreq; 
        Serial.printf("[Nextion IN] Frekuensi Valid Diterima: %.1f Hz\n", vfdFreq);
        
        // Otomatis running jika user memasukkan frekuensi di atas 0 Hz via Nextion
        if (vfdFreq > 0.0) vfdRunning = true; 
        
        applyFrequencyToOutput(); // Langsung hitung PWM & keluarkan tegangan fisik!
        
        // Sinkronisasi data baru ke MQTT agar dashboard web juga terupdate
        if (mqtt.connected()) {
          char freqBuf[16];
          dtostrf(vfdFreq, 0, 1, freqBuf);
          mqtt.publish(TOPIC_VFD_FREQ, freqBuf);
          mqtt.publish(TOPIC_VFD_STATUS, vfdRunning ? "RUNNING" : "STOPPED");
        }
      } else {
        Serial.printf("[Nextion IN] Error: Nilai %.1f Hz di luar batas aman (0-50Hz)!\n", newFreq);
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

  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  setOutputVoltage(0.0);

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

  // Membaca ketikan keypad Nextion setiap siklus loop
  checkNextionInput(); 

  unsigned long now = millis();

  if (now - lastSensorSend >= SENSOR_INTERVAL_MS) {
    lastSensorSend = now;
    publishSensors();
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