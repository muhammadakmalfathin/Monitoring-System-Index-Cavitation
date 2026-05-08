/**
 * ============================================================
 * SISTEM MONITORING KAVITASI - ESP32-S3
 * Sensor: WPT83G (Water Pressure), MPU6050 (Vibration), ACS712 (Current)
 * Kontrol: Fuzzy-PID → 2x Optocoupler (UP/DOWN speed)
 * Output: Serial JSON → Upload ke GitHub via WiFi
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>
#include <math.h>

// ─── KONFIGURASI WiFi & GitHub ─────────────────────────────
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";
const char* GITHUB_TOKEN  = "ghp_YOUR_GITHUB_TOKEN";
const char* GITHUB_REPO   = "username/repo-kavitasi";
const char* DATA_FILE     = "data/sensor_data.json";
const char* GITHUB_API    = "https://api.github.com";

// ─── PIN DEFINITIONS ───────────────────────────────────────
#define PIN_ACS712        34    // ADC1 - Sensor arus
#define PIN_WPT83G        35    // ADC1 - Sensor tekanan
#define PIN_OPTO_UP       25    // Output - Optocoupler NAIK kecepatan
#define PIN_OPTO_DOWN     26    // Output - Optocoupler TURUN kecepatan
#define PIN_LED_STATUS    2     // LED built-in status

// ─── KONSTANTA SENSOR ──────────────────────────────────────
#define ACS712_SENSITIVITY  0.185f   // V/A untuk ACS712-5A
#define ACS712_ZERO_OFFSET  2.5f     // Tegangan offset (V)
#define ADC_REF_VOLTAGE     3.3f
#define ADC_RESOLUTION      4095.0f
#define WPT83G_MAX_PRESSURE 10.0f    // Bar (sesuaikan dengan sensor)
#define WPT83G_MIN_VOLTAGE  0.5f     // V (output minimum sensor)
#define WPT83G_MAX_VOLTAGE  4.5f     // V (output maksimum sensor)

// ─── PARAMETER KAVITASI ────────────────────────────────────
#define CAVITATION_PRESSURE_THRESHOLD  2.0f    // Bar
#define CAVITATION_VIBRATION_THRESHOLD 2.5f    // g (akselerasi)
#define CAVITATION_CURRENT_THRESHOLD   8.0f    // Ampere
#define VAPOR_PRESSURE_WATER           0.023f  // Bar pada 20°C

// ─── PID PARAMETERS ────────────────────────────────────────
float Kp = 1.2f, Ki = 0.1f, Kd = 0.05f;
float pidIntegral = 0.0f;
float pidPrevError = 0.0f;
float pidSetpoint = 4.0f;   // Target tekanan (Bar)
unsigned long lastPidTime = 0;

// ─── FUZZY MEMBERSHIP PARAMETERS ──────────────────────────
struct FuzzySet { float a, b, c, d; };

FuzzySet errorSets[5] = {
  {-10, -10, -3, -1},   // NB: Negative Big
  {-3,  -2,  -1,  0},   // NS: Negative Small
  {-0.5, 0,  0,  0.5},  // ZE: Zero
  {0,    1,  2,  3},    // PS: Positive Small
  {1,    3, 10, 10}     // PB: Positive Big
};

// ─── TIMING ────────────────────────────────────────────────
unsigned long lastSensorRead  = 0;
unsigned long lastGitHubUpload = 0;
const unsigned long SENSOR_INTERVAL  = 1000;   // 1 detik
const unsigned long UPLOAD_INTERVAL  = 30000;  // 30 detik

// ─── DATA BUFFERS ──────────────────────────────────────────
const int BUFFER_SIZE = 60;
struct SensorReading {
  unsigned long timestamp;
  float pressure;       // Bar
  float accel_x, accel_y, accel_z;  // g
  float vibration_rms;  // g RMS
  float current;        // Ampere
  float cavitationIndex;
  int   cavitationClass;  // 0=None, 1=Incipient, 2=Developed, 3=Supercavitation
  float pidOutput;
  bool  optoUp, optoDown;
};

SensorReading dataBuffer[BUFFER_SIZE];
int bufferHead = 0, bufferCount = 0;

// ─── OBJECTS ───────────────────────────────────────────────
MPU6050 mpu;
String filesha = "";

// ─────────────────────────────────────────────────────────
// FUNGSI FUZZY MEMBERSHIP
// ─────────────────────────────────────────────────────────
float trapezoid(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0f;
  if (x >= b && x <= c) return 1.0f;
  if (x > a && x < b)   return (x - a) / (b - a);
  return (d - x) / (d - c);
}

float triangleMF(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b)           return 1.0f;
  if (x < b)            return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// ─────────────────────────────────────────────────────────
// FUZZY-PID CONTROLLER
// ─────────────────────────────────────────────────────────
float fuzzyPID(float error, float dError) {
  // Fuzzifikasi error
  float muNB = trapezoid(error, errorSets[0].a, errorSets[0].b, errorSets[0].c, errorSets[0].d);
  float muNS = trapezoid(error, errorSets[1].a, errorSets[1].b, errorSets[1].c, errorSets[1].d);
  float muZE = triangleMF(error, -0.5f, 0.0f, 0.5f);
  float muPS = trapezoid(error, errorSets[3].a, errorSets[3].b, errorSets[3].c, errorSets[3].d);
  float muPB = trapezoid(error, errorSets[4].a, errorSets[4].b, errorSets[4].c, errorSets[4].d);

  // Aturan Fuzzy → adaptasi Kp, Ki, Kd
  float kpAdj = 0.0f, kiAdj = 0.0f, kdAdj = 0.0f;

  // Mamdani rule base (simplified)
  kpAdj = muNB * 0.8f + muNS * 0.4f + muZE * 0.0f + muPS * 0.4f + muPB * 0.8f;
  kiAdj = muNB * 0.3f + muNS * 0.2f + muZE * 0.1f + muPS * 0.2f + muPB * 0.3f;
  kdAdj = muNB * 0.1f + muNS * 0.05f + muZE * 0.0f + muPS * 0.05f + muPB * 0.1f;

  float Kp_fuzzy = Kp + kpAdj;
  float Ki_fuzzy = Ki + kiAdj;
  float Kd_fuzzy = Kd + kdAdj;

  // Hitung PID
  unsigned long now = millis();
  float dt = (now - lastPidTime) / 1000.0f;
  if (dt <= 0) dt = 0.001f;

  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -10.0f, 10.0f);

  float output = Kp_fuzzy * error + Ki_fuzzy * pidIntegral + Kd_fuzzy * (dError / dt);
  lastPidTime = now;

  return constrain(output, -10.0f, 10.0f);
}

// ─────────────────────────────────────────────────────────
// BACA SENSOR
// ─────────────────────────────────────────────────────────
float readPressure() {
  int raw = analogRead(PIN_WPT83G);
  float voltage = (raw / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  // Linear interpolasi dari rentang tegangan ke tekanan
  float pressure = (voltage - WPT83G_MIN_VOLTAGE) / 
                   (WPT83G_MAX_VOLTAGE - WPT83G_MIN_VOLTAGE) * WPT83G_MAX_PRESSURE;
  return constrain(pressure, 0.0f, WPT83G_MAX_PRESSURE);
}

float readCurrent() {
  // Oversampling untuk akurasi
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(PIN_ACS712);
    delayMicroseconds(100);
  }
  float avg = sum / 100.0f;
  float voltage = (avg / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  float current = (voltage - ACS712_ZERO_OFFSET) / ACS712_SENSITIVITY;
  return fabs(current);
}

float readVibrationRMS() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accel_x = ax / 16384.0f;
  float accel_y = ay / 16384.0f;
  float accel_z = az / 16384.0f;

  // Hitung magnitude akselerasi
  float magnitude = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z) - 1.0f; // Kurangi gravitasi
  return fabs(magnitude);
}

// ─────────────────────────────────────────────────────────
// KLASIFIKASI & PREDIKSI KAVITASI
// ─────────────────────────────────────────────────────────
// Cavitation Index (Thoma Number σ)
float calculateCavitationIndex(float pressure) {
  // σ = (P_atm + P_static - P_vapor) / (0.5 * ρ * v²)
  // Simplifikasi: gunakan rasio tekanan terhadap tekanan uap
  float sigma = (pressure - VAPOR_PRESSURE_WATER) / 
                max(pressure, 0.001f);
  return constrain(sigma, 0.0f, 10.0f);
}

// Multi-parameter cavitation scoring
float calculateCavitationScore(float pressure, float vibration, float current) {
  float scoreP = 0, scoreV = 0, scoreC = 0;

  // Skor tekanan (rendah = bahaya kavitasi)
  if (pressure < CAVITATION_PRESSURE_THRESHOLD) {
    scoreP = (CAVITATION_PRESSURE_THRESHOLD - pressure) / CAVITATION_PRESSURE_THRESHOLD * 100.0f;
  }

  // Skor vibrasi (tinggi = ada kavitasi)
  if (vibration > 0.5f) {
    scoreV = min((vibration / CAVITATION_VIBRATION_THRESHOLD) * 100.0f, 100.0f);
  }

  // Skor arus (fluktuasi = ada kavitasi)
  if (current > CAVITATION_CURRENT_THRESHOLD * 0.7f) {
    scoreC = min(((current - CAVITATION_CURRENT_THRESHOLD * 0.7f) / 
                  (CAVITATION_CURRENT_THRESHOLD * 0.3f)) * 100.0f, 100.0f);
  }

  // Weighted average
  return scoreP * 0.4f + scoreV * 0.4f + scoreC * 0.2f;
}

int classifyCavitation(float score, float vibration, float pressure) {
  // Klasifikasi berdasarkan skor gabungan
  if (score < 20.0f && vibration < 0.8f && pressure > 3.0f) return 0; // Tidak ada kavitasi
  if (score < 45.0f) return 1;  // Kavitasi awal (Incipient)
  if (score < 75.0f) return 2;  // Kavitasi berkembang (Developed)
  return 3;                      // Superkavitasi (Kritis)
}

const char* cavitationLabel(int cls) {
  switch(cls) {
    case 0: return "NORMAL";
    case 1: return "INCIPIENT";
    case 2: return "DEVELOPED";
    case 3: return "SUPERCAVITATION";
    default: return "UNKNOWN";
  }
}

// ─────────────────────────────────────────────────────────
// KONTROL OPTOCOUPLER
// ─────────────────────────────────────────────────────────
void controlOptocoupler(float pidOutput, bool &optoUp, bool &optoDown) {
  optoUp = false;
  optoDown = false;

  if (pidOutput > 0.5f) {
    // Naikkan kecepatan pompa
    optoUp = true;
    digitalWrite(PIN_OPTO_UP, HIGH);
    digitalWrite(PIN_OPTO_DOWN, LOW);
  } else if (pidOutput < -0.5f) {
    // Turunkan kecepatan pompa
    optoDown = true;
    digitalWrite(PIN_OPTO_UP, LOW);
    digitalWrite(PIN_OPTO_DOWN, HIGH);
  } else {
    // Pertahankan kecepatan
    digitalWrite(PIN_OPTO_UP, LOW);
    digitalWrite(PIN_OPTO_DOWN, LOW);
  }
}

// ─────────────────────────────────────────────────────────
// GITHUB API - GET FILE SHA
// ─────────────────────────────────────────────────────────
String getFileSHA() {
  if (WiFi.status() != WL_CONNECTED) return "";
  HTTPClient http;
  String url = String(GITHUB_API) + "/repos/" + GITHUB_REPO + "/contents/" + DATA_FILE;
  http.begin(url);
  http.addHeader("Authorization", String("token ") + GITHUB_TOKEN);
  http.addHeader("Accept", "application/vnd.github.v3+json");
  int code = http.GET();
  String sha = "";
  if (code == 200) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, http.getString());
    sha = doc["sha"].as<String>();
  }
  http.end();
  return sha;
}

// ─────────────────────────────────────────────────────────
// GITHUB API - UPLOAD DATA
// ─────────────────────────────────────────────────────────
void uploadToGitHub() {
  if (WiFi.status() != WL_CONNECTED) return;

  // Build JSON payload dari buffer
  DynamicJsonDocument doc(8192);
  JsonArray readings = doc.createNestedArray("readings");

  int count = min(bufferCount, BUFFER_SIZE);
  for (int i = 0; i < count; i++) {
    int idx = (bufferHead - count + i + BUFFER_SIZE) % BUFFER_SIZE;
    SensorReading &r = dataBuffer[idx];
    JsonObject obj = readings.createNestedObject();
    obj["ts"]           = r.timestamp;
    obj["pressure"]     = serialized(String(r.pressure, 3));
    obj["accel_x"]      = serialized(String(r.accel_x, 3));
    obj["accel_y"]      = serialized(String(r.accel_y, 3));
    obj["accel_z"]      = serialized(String(r.accel_z, 3));
    obj["vibration"]    = serialized(String(r.vibration_rms, 3));
    obj["current"]      = serialized(String(r.current, 3));
    obj["cav_index"]    = serialized(String(r.cavitationIndex, 3));
    obj["cav_class"]    = r.cavitationClass;
    obj["cav_label"]    = cavitationLabel(r.cavitationClass);
    obj["pid_output"]   = serialized(String(r.pidOutput, 3));
    obj["opto_up"]      = r.optoUp;
    obj["opto_down"]    = r.optoDown;
  }
  doc["updated_at"] = millis();

  String jsonContent;
  serializeJson(doc, jsonContent);

  // Base64 encode konten
  String encoded = base64Encode(jsonContent);
  String sha = getFileSHA();

  // Build GitHub API payload
  DynamicJsonDocument payload(12288);
  payload["message"] = String("Update sensor data ") + String(millis());
  payload["content"] = encoded;
  if (sha.length() > 0) payload["sha"] = sha;

  String payloadStr;
  serializeJson(payload, payloadStr);

  HTTPClient http;
  String url = String(GITHUB_API) + "/repos/" + GITHUB_REPO + "/contents/" + DATA_FILE;
  http.begin(url);
  http.addHeader("Authorization", String("token ") + GITHUB_TOKEN);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/vnd.github.v3+json");

  int code = http.PUT(payloadStr);
  Serial.printf("[GitHub] Upload status: %d\n", code);
  http.end();
}

// Base64 encoding
String base64Encode(const String &input) {
  const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String output = "";
  int i = 0;
  unsigned char a3[3], a4[4];
  int len = input.length();
  const char* data = input.c_str();

  while (i < len) {
    int j = 0;
    while (j < 3 && i < len) { a3[j++] = data[i++]; }
    if (j) {
      a4[0] = (a3[0] & 0xfc) >> 2;
      a4[1] = ((a3[0] & 0x03) << 4) | ((a3[1] & 0xf0) >> 4);
      a4[2] = ((a3[1] & 0x0f) << 2) | ((a3[2] & 0xc0) >> 6);
      a4[3] = a3[2] & 0x3f;
      for (int k = 0; k < j + 1; k++) output += chars[a4[k]];
      while (j++ < 3) output += '=';
    }
  }
  return output;
}

// ─────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== SISTEM MONITORING KAVITASI v1.0 ===");

  // Pin setup
  pinMode(PIN_OPTO_UP,   OUTPUT);
  pinMode(PIN_OPTO_DOWN, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_OPTO_UP,   LOW);
  digitalWrite(PIN_OPTO_DOWN, LOW);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // I2C & MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 tidak terdeteksi!");
  } else {
    Serial.println("[OK] MPU6050 terhubung");
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); // ±4g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  }

  // WiFi
  Serial.printf("[WiFi] Menghubungkan ke %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Terhubung! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WiFi] Gagal terhubung, mode offline");
  }

  lastPidTime = millis();
  Serial.println("[READY] Sistem siap beroperasi\n");
}

// ─────────────────────────────────────────────────────────
// LOOP UTAMA
// ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Baca Sensor setiap 1 detik ──
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    // Baca semua sensor
    float pressure    = readPressure();
    float current     = readCurrent();
    float vibration   = readVibrationRMS();

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float accel_x = ax / 16384.0f;
    float accel_y = ay / 16384.0f;
    float accel_z = az / 16384.0f;

    // Hitung kavitasi
    float cavIndex  = calculateCavitationIndex(pressure);
    float cavScore  = calculateCavitationScore(pressure, vibration, current);
    int   cavClass  = classifyCavitation(cavScore, vibration, pressure);

    // Fuzzy-PID
    float error  = pidSetpoint - pressure;
    float dError = error - pidPrevError;
    float pidOut = fuzzyPID(error, dError);
    pidPrevError = error;

    // Kontrol Optocoupler
    bool optoUp, optoDown;
    controlOptocoupler(pidOut, optoUp, optoDown);

    // Simpan ke buffer
    SensorReading &r = dataBuffer[bufferHead];
    r.timestamp      = now;
    r.pressure       = pressure;
    r.accel_x        = accel_x;
    r.accel_y        = accel_y;
    r.accel_z        = accel_z;
    r.vibration_rms  = vibration;
    r.current        = current;
    r.cavitationIndex = cavIndex;
    r.cavitationClass = cavClass;
    r.pidOutput      = pidOut;
    r.optoUp         = optoUp;
    r.optoDown       = optoDown;

    bufferHead = (bufferHead + 1) % BUFFER_SIZE;
    if (bufferCount < BUFFER_SIZE) bufferCount++;

    // LED status berdasarkan klasifikasi
    digitalWrite(PIN_LED_STATUS, cavClass >= 2 ? HIGH : LOW);

    // Output Serial JSON (untuk debugging)
    Serial.printf("{\"ts\":%lu,\"P\":%.2f,\"V\":%.3f,\"I\":%.2f,\"CI\":%.3f,\"CC\":%d,\"%s\",\"PID\":%.2f,\"UP\":%d,\"DN\":%d}\n",
      now, pressure, vibration, current, cavIndex, cavClass, cavitationLabel(cavClass), pidOut, optoUp, optoDown);
  }

  // ── Upload ke GitHub setiap 30 detik ──
  if (now - lastGitHubUpload >= UPLOAD_INTERVAL) {
    lastGitHubUpload = now;
    Serial.println("[GitHub] Mengupload data...");
    uploadToGitHub();
  }
}
