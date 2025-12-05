/*
  LibaTracker.ino - Versão Consolidada 2025 (179 BYTES)
  VR Tracker ESP32-S3 + ICM-20948
  
  FEATURES:
  - Protocolo V6 com CRC32 (179 bytes FIXOS)
  - Thread Safety (mutexes para IMU e ring buffer)
  - Sequence number atômico (volatile + mutex)
  - AsyncWebServer com WebUI completa
  - Beta Madgwick adaptativo otimizado para cintura (0.008-0.038)
  - ZUPT otimizado (0.35 rad/s)
  - Bias dinâmico de giroscópio melhorado
  - Detecção de caminhada no lugar (esteira)
  - Velocity smoothing anti-jitter
  - Declinação magnética configurável
  - Sistema de slots para perfis
  - Buffer circular para WiFi intermitente
  - Taxa de 400 Hz
  - mDNS (vrtracker.local)
*/

// ================= INCLUDES =================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPIFFS.h>
#include <ICM_20948.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <math.h>
#include <algorithm>
#include "protocol.h"  // Define VRTrackerPacket_V6 (179 bytes)
#include "utils.h"     // Define calculate_crc32

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ================= CONFIGURAÇÕES =================
// WiFi (serão carregados do SPIFFS)
static String wifi_ssid = "";
static String wifi_password = "";
static const char* udpAddress = "192.168.3.18";
static const int udpPort = 5005;

// Access Point para configuração inicial
const char* AP_SSID = "VRTRACKER_SETUP";
const char* AP_PASS = "vrtracker123";
IPAddress apIP(192, 168, 4, 1);
const byte DNS_PORT = 53;

// I2C Pins
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQ 400000

// Timing
#define TARGET_FREQ 400.0f
#define SAMPLE_PERIOD_US (1000000UL / (uint32_t)TARGET_FREQ)
#define UDP_SEND_INTERVAL_MS 3  // ~333 Hz envio
#define PACKET_SIZE VRTRACKER_PACKET_SIZE  // 179 bytes
#define RING_BUFFER_SIZE 256

// Limites de segurança
const float GRAVITY = 9.80665f;
const float MAX_ACCEL_MS2 = 60.0f;
const float MAX_GYRO_RADS = 25.0f;

// SPIFFS paths
const char* CONFIG_PATH = "/tracker_cfg.json";
const char* CALIB_PATH = "/calibration.json";
const char* SLOT_DIR = "/configs";
const int MAX_SLOTS = 10;

// ZUPT
#define ZUPT_WINDOW 20

// Step detection
#define STEP_TIMES_MAX 128

// ================= OBJETOS GLOBAIS =================
ICM_20948_I2C myICM;
WiFiUDP Udp;
AsyncWebServer server(80);
DNSServer dnsServer;

// Mutexes para thread safety
SemaphoreHandle_t imuMutex = NULL;
SemaphoreHandle_t ringMutex = NULL;
SemaphoreHandle_t seqMutex = NULL;  // NOVO: mutex para sequence number

// ================= CALIBRAÇÃO =================
float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
float accel_offset[3] = {0.0f, 0.0f, 0.0f};
float mag_offset[3] = {0.0f, 0.0f, 0.0f};
float mag_scale[3] = {1.0f, 1.0f, 1.0f};

// ================= FILTRO KALMAN 1D =================
struct KalmanFilter {
    float x, p, q, r;
};
KalmanFilter kalman[3] = {
    {0, 1.0f, 0.01f, 0.1f},
    {0, 1.0f, 0.01f, 0.1f},
    {0, 1.0f, 0.01f, 0.1f}
};

float kalman_filter_1d(KalmanFilter &kf, float measurement) {
    kf.p = kf.p + kf.q;
    float k = kf.p / (kf.p + kf.r);
    kf.x = kf.x + k * (measurement - kf.x);
    kf.p = (1.0f - k) * kf.p;
    return kf.x;
}

// ================= ORIENTAÇÃO =================
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float predicted_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// ================= DADOS SENSOR =================
float accel_raw[3] = {0.0f, 0.0f, 0.0f};
float gyro_raw[3] = {0.0f, 0.0f, 0.0f};
float mag_calib[3] = {0.0f, 0.0f, 0.0f};
float temperature = 0.0f;

// ================= VELOCIDADE/POSIÇÃO =================
float velocity[3] = {0.0f, 0.0f, 0.0f};
float velocity_filtered[3] = {0.0f, 0.0f, 0.0f};  // Velocity smoothing
float position[3] = {0.0f, 0.0f, 0.0f};
float height_estimate = 0.0f;

// ================= ATIVIDADE =================
enum ActivityState { STATIONARY = 0, WALKING = 1, RUNNING = 2, JUMPING = 3 };
ActivityState current_activity = STATIONARY;
float beta_adaptive = 0.022f;

// ================= ZUPT =================
float accel_mag_window[ZUPT_WINDOW];
int accel_window_idx = 0;
bool zupt_state = false;
unsigned long zupt_since = 0;
bool zupt_initialized = false;
int zupt_samples_count = 0;

// ================= DETECÇÃO DE PASSOS =================
unsigned long step_times[STEP_TIMES_MAX];
int step_times_head = 0;
int step_times_count = 0;
unsigned long last_step_ms = 0;
float bob_hp = 0.0f;
float bob_env = 0.0f;
float bob_prev_x = 0.0f;

// ================= FLAGS E ESTADOS =================
// MUDANÇA: Substituir std::atomic por volatile + mutex
volatile uint32_t global_sequence_number = 0;

// Função auxiliar para incrementar atomicamente o sequence number
uint32_t get_next_sequence() {
    uint32_t seq = 0;
    if (xSemaphoreTake(seqMutex, (TickType_t)10) == pdTRUE) {
        seq = global_sequence_number++;
        xSemaphoreGive(seqMutex);
    }
    return seq;
}

uint8_t current_activity_state = 0;
bool zupt_active_state = false;
bool mag_calibrated_state = false;
bool mag_interference_state = false;
bool low_battery_state = false;
bool imu_error_state = false;
bool calibration_mode_active = false;
bool step_detected_flag = false;

// ================= PACKET COUNTERS =================
volatile uint32_t packet_count = 0;
volatile uint32_t dropped_packets = 0;

// ================= CONFIGURAÇÃO =================
struct Config {
    float x_range = 3.0f;
    float x_threshold = 0.15f;
    int n_sample_x = 3;
    int n_sample_y = 3;
    float y_max_speed = 5.0f;
    float zupt_accel_var_threshold = 0.30f;
    float zupt_gyro_threshold = 0.35f;  // Otimizado para cintura
    float madgwick_beta = 0.022f;       // Otimizado para cintura
    float velocity_decay = 1.0f;
    float position_decay = 1.0f;
    float max_velocity = 6.0f;
    uint32_t zupt_reset_ms = 10000;
    bool use_magnetometer = true;
    float yaw_correction_speed = 0.8f;
    uint32_t yaw_correction_time_ms = 2000;
    float yaw_correction_gain = 0.02f;
    float bob_hp_alpha = 0.9f;
    float bob_lp_alpha = 0.12f;
    float step_detect_thresh = 0.5f;
    float stride_length = 0.45f;
    int step_min_interval_ms = 200;
    float magnetic_declination_deg = -21.5f;  // São José dos Campos
    float velocity_smooth_alpha = 0.15f;       // Anti-jitter
    float accel_threshold_walk = 15.0f;        // m/s^2
    float gyro_threshold_turn = 50.0f;         // rad/s
    bool enable_deep_sleep = false;
} cfg;

// ================= STRIDE ZONES =================
#define MAX_STRIDE_ZONES 8
struct StrideZone {
    float min_hz;
    float max_hz;
    float stride;
};
StrideZone stride_zones[MAX_STRIDE_ZONES];
int stride_zone_count = 0;

// ================= RING BUFFER =================
struct RingEntry {
    uint16_t len;
    uint8_t data[PACKET_SIZE];
};

struct RingBuffer {
    RingEntry entries[RING_BUFFER_SIZE];
    volatile uint16_t write_idx = 0;
    volatile uint16_t read_idx = 0;
    volatile uint16_t count = 0;
} ring_buffer;

void ring_buffer_push(const uint8_t* packet, uint16_t len) {
    if (len == 0 || len > PACKET_SIZE) return;
    if (xSemaphoreTake(ringMutex, (TickType_t)10) == pdTRUE) {
        if (ring_buffer.count < RING_BUFFER_SIZE) {
            RingEntry &e = ring_buffer.entries[ring_buffer.write_idx];
            e.len = len;
            memcpy(e.data, packet, len);
            ring_buffer.write_idx = (ring_buffer.write_idx + 1) % RING_BUFFER_SIZE;
            ring_buffer.count++;
        } else {
            ring_buffer.read_idx = (ring_buffer.read_idx + 1) % RING_BUFFER_SIZE;
            RingEntry &e = ring_buffer.entries[ring_buffer.write_idx];
            e.len = len;
            memcpy(e.data, packet, len);
            ring_buffer.write_idx = (ring_buffer.write_idx + 1) % RING_BUFFER_SIZE;
        }
        xSemaphoreGive(ringMutex);
    }
}

bool ring_buffer_pop(uint8_t* out_packet, uint16_t &out_len) {
    bool ok = false;
    if (xSemaphoreTake(ringMutex, (TickType_t)10) == pdTRUE) {
        if (ring_buffer.count == 0) {
            xSemaphoreGive(ringMutex);
            return false;
        }
        RingEntry &e = ring_buffer.entries[ring_buffer.read_idx];
        out_len = e.len;
        memcpy(out_packet, e.data, out_len);
        ring_buffer.read_idx = (ring_buffer.read_idx + 1) % RING_BUFFER_SIZE;
        ring_buffer.count--;
        ok = true;
        xSemaphoreGive(ringMutex);
    }
    return ok;
}

// ================= FUNÇÕES AUXILIARES =================
String isoNow() {
    return String("boot+") + String(millis()) + "ms";
}

// ================= STRIDE ZONES =================
void clearStrideZones() {
    stride_zone_count = 0;
    for (int i = 0; i < MAX_STRIDE_ZONES; i++) {
        stride_zones[i].min_hz = 0.0f;
        stride_zones[i].max_hz = 0.0f;
        stride_zones[i].stride = cfg.stride_length;
    }
}

void loadStrideZonesFromJson(DynamicJsonDocument &doc) {
    clearStrideZones();
    if (!doc.containsKey("stride_zones")) return;
    JsonArray arr = doc["stride_zones"].as<JsonArray>();
    int i = 0;
    for (JsonObject obj : arr) {
        if (i >= MAX_STRIDE_ZONES) break;
        stride_zones[i].min_hz = obj["min_hz"] | 0.0f;
        stride_zones[i].max_hz = obj["max_hz"] | 100.0f;
        stride_zones[i].stride = obj["stride"] | cfg.stride_length;
        i++;
    }
    stride_zone_count = i;
}

float lookupStrideForHz(float hz) {
    if (stride_zone_count == 0) return cfg.stride_length;
    for (int i = 0; i < stride_zone_count; i++) {
        if (hz >= stride_zones[i].min_hz && hz < stride_zones[i].max_hz)
            return stride_zones[i].stride;
    }
    return stride_zones[stride_zone_count - 1].stride;
}

// ================= CONFIG LOAD/SAVE =================
void loadConfig() {
    if (!SPIFFS.begin(true)) {
        Serial.println(isoNow() + " [ERR] SPIFFS mount failed");
        return;
    }
    if (!SPIFFS.exists(CONFIG_PATH)) {
        Serial.println(isoNow() + " [CFG] config not found, using defaults");
        return;
    }
    File f = SPIFFS.open(CONFIG_PATH, "r");
    if (!f) {
        Serial.println(isoNow() + " [ERR] failed to open config");
        return;
    }
    DynamicJsonDocument doc(1536);
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        Serial.print(isoNow() + " [ERR] parse error: ");
        Serial.println(err.c_str());
        return;
    }

    // Carregar valores
    cfg.x_range = doc["x_range"] | cfg.x_range;
    cfg.x_threshold = doc["x_threshold"] | cfg.x_threshold;
    cfg.n_sample_x = doc["n_sample_x"] | cfg.n_sample_x;
    cfg.n_sample_y = doc["n_sample_y"] | cfg.n_sample_y;
    cfg.y_max_speed = doc["y_max_speed"] | cfg.y_max_speed;
    cfg.zupt_accel_var_threshold = doc["zupt_accel_var_threshold"] | cfg.zupt_accel_var_threshold;
    cfg.zupt_gyro_threshold = doc["zupt_gyro_threshold"] | cfg.zupt_gyro_threshold;
    cfg.madgwick_beta = doc["madgwick_beta"] | cfg.madgwick_beta;
    cfg.velocity_decay = doc["velocity_decay"] | cfg.velocity_decay;
    cfg.position_decay = doc["position_decay"] | cfg.position_decay;
    cfg.max_velocity = doc["max_velocity"] | cfg.max_velocity;
    cfg.zupt_reset_ms = doc["zupt_reset_ms"] | cfg.zupt_reset_ms;
    cfg.use_magnetometer = doc["use_magnetometer"] | cfg.use_magnetometer;
    cfg.yaw_correction_speed = doc["yaw_correction_speed"] | cfg.yaw_correction_speed;
    cfg.yaw_correction_time_ms = doc["yaw_correction_time_ms"] | cfg.yaw_correction_time_ms;
    cfg.yaw_correction_gain = doc["yaw_correction_gain"] | cfg.yaw_correction_gain;
    cfg.bob_hp_alpha = doc["bob_hp_alpha"] | cfg.bob_hp_alpha;
    cfg.bob_lp_alpha = doc["bob_lp_alpha"] | cfg.bob_lp_alpha;
    cfg.step_detect_thresh = doc["step_detect_thresh"] | cfg.step_detect_thresh;
    cfg.stride_length = doc["stride_length"] | cfg.stride_length;
    cfg.step_min_interval_ms = doc["step_min_interval_ms"] | cfg.step_min_interval_ms;
    cfg.magnetic_declination_deg = doc["magnetic_declination_deg"] | cfg.magnetic_declination_deg;
    cfg.velocity_smooth_alpha = doc["velocity_smooth_alpha"] | cfg.velocity_smooth_alpha;
    cfg.accel_threshold_walk = doc["accel_threshold_walk"] | cfg.accel_threshold_walk;
    cfg.gyro_threshold_turn = doc["gyro_threshold_turn"] | cfg.gyro_threshold_turn;
    cfg.enable_deep_sleep = doc["enable_deep_sleep"] | cfg.enable_deep_sleep;

    if (doc.containsKey("wifi_ssid")) {
        wifi_ssid = doc["wifi_ssid"].as<String>();
    }
    if (doc.containsKey("wifi_password")) {
        wifi_password = doc["wifi_password"].as<String>();
    }

    loadStrideZonesFromJson(doc);
    Serial.println(isoNow() + " [CFG] Loaded config");
}

bool saveConfig() {
    DynamicJsonDocument doc(1536);
    doc["x_range"] = cfg.x_range;
    doc["x_threshold"] = cfg.x_threshold;
    doc["n_sample_x"] = cfg.n_sample_x;
    doc["n_sample_y"] = cfg.n_sample_y;
    doc["y_max_speed"] = cfg.y_max_speed;
    doc["zupt_accel_var_threshold"] = cfg.zupt_accel_var_threshold;
    doc["zupt_gyro_threshold"] = cfg.zupt_gyro_threshold;
    doc["madgwick_beta"] = cfg.madgwick_beta;
    doc["velocity_decay"] = cfg.velocity_decay;
    doc["position_decay"] = cfg.position_decay;
    doc["max_velocity"] = cfg.max_velocity;
    doc["zupt_reset_ms"] = cfg.zupt_reset_ms;
    doc["use_magnetometer"] = cfg.use_magnetometer;
    doc["yaw_correction_speed"] = cfg.yaw_correction_speed;
    doc["yaw_correction_time_ms"] = cfg.yaw_correction_time_ms;
    doc["yaw_correction_gain"] = cfg.yaw_correction_gain;
    doc["bob_hp_alpha"] = cfg.bob_hp_alpha;
    doc["bob_lp_alpha"] = cfg.bob_lp_alpha;
    doc["step_detect_thresh"] = cfg.step_detect_thresh;
    doc["stride_length"] = cfg.stride_length;
    doc["step_min_interval_ms"] = cfg.step_min_interval_ms;
    doc["magnetic_declination_deg"] = cfg.magnetic_declination_deg;
    doc["velocity_smooth_alpha"] = cfg.velocity_smooth_alpha;
    doc["accel_threshold_walk"] = cfg.accel_threshold_walk;
    doc["gyro_threshold_turn"] = cfg.gyro_threshold_turn;
    doc["enable_deep_sleep"] = cfg.enable_deep_sleep;
    doc["wifi_ssid"] = wifi_ssid;
    doc["wifi_password"] = wifi_password;

    if (stride_zone_count > 0) {
        JsonArray arr = doc.createNestedArray("stride_zones");
        for (int i = 0; i < stride_zone_count; i++) {
            JsonObject o = arr.createNestedObject();
            o["min_hz"] = stride_zones[i].min_hz;
            o["max_hz"] = stride_zones[i].max_hz;
            o["stride"] = stride_zones[i].stride;
        }
    }

    File f = SPIFFS.open(CONFIG_PATH, "w");
    if (!f) {
        Serial.println(isoNow() + " [ERR] failed to open config for write");
        return false;
    }
    if (serializeJson(doc, f) == 0) {
        Serial.println(isoNow() + " [ERR] failed to write config");
        f.close();
        return false;
    }
    f.close();
    Serial.println(isoNow() + " [CFG] Saved config");
    return true;
}

// ================= CALIBRAÇÃO LOAD/SAVE =================
bool saveCalibration() {
    DynamicJsonDocument doc(512);
    doc["gyro_offset_0"] = gyro_offset[0];
    doc["gyro_offset_1"] = gyro_offset[1];
    doc["gyro_offset_2"] = gyro_offset[2];
    doc["accel_offset_0"] = accel_offset[0];
    doc["accel_offset_1"] = accel_offset[1];
    doc["accel_offset_2"] = accel_offset[2];
    doc["mag_offset_0"] = mag_offset[0];
    doc["mag_offset_1"] = mag_offset[1];
    doc["mag_offset_2"] = mag_offset[2];
    doc["mag_scale_0"] = mag_scale[0];
    doc["mag_scale_1"] = mag_scale[1];
    doc["mag_scale_2"] = mag_scale[2];

    File f = SPIFFS.open(CALIB_PATH, "w");
    if (!f) {
        Serial.println(isoNow() + " [ERR] failed to open calibration for write");
        return false;
    }
    if (serializeJson(doc, f) == 0) {
        Serial.println(isoNow() + " [ERR] failed to write calibration");
        f.close();
        return false;
    }
    f.close();
    Serial.println(isoNow() + " [CALIB] Saved calibration");
    return true;
}

bool loadCalibration() {
    if (!SPIFFS.exists(CALIB_PATH)) return false;
    File f = SPIFFS.open(CALIB_PATH, "r");
    if (!f) return false;
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    gyro_offset[0] = doc["gyro_offset_0"] | gyro_offset[0];
    gyro_offset[1] = doc["gyro_offset_1"] | gyro_offset[1];
    gyro_offset[2] = doc["gyro_offset_2"] | gyro_offset[2];
    accel_offset[0] = doc["accel_offset_0"] | accel_offset[0];
    accel_offset[1] = doc["accel_offset_1"] | accel_offset[1];
    accel_offset[2] = doc["accel_offset_2"] | accel_offset[2];
    mag_offset[0] = doc["mag_offset_0"] | mag_offset[0];
    mag_offset[1] = doc["mag_offset_1"] | mag_offset[1];
    mag_offset[2] = doc["mag_offset_2"] | mag_offset[2];
    mag_scale[0] = doc["mag_scale_0"] | mag_scale[0];
    mag_scale[1] = doc["mag_scale_1"] | mag_scale[1];
    mag_scale[2] = doc["mag_scale_2"] | mag_scale[2];

    Serial.println(isoNow() + " [CALIB] Loaded calibration");
    return true;
}

#define GYRO_CALIB_SAMPLES 200
#define ACCEL_CALIB_SAMPLES 200

void calibrateGyro() {
    Serial.println(isoNow() + " [CALIB] Start gyro calibration. Keep device still.");
    delay(1500);
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            gx_sum += myICM.gyrX() * 0.01745329252f;
            gy_sum += myICM.gyrY() * 0.01745329252f;
            gz_sum += myICM.gyrZ() * 0.01745329252f;
        }
        delay(10);
    }
    gyro_offset[0] = gx_sum / GYRO_CALIB_SAMPLES;
    gyro_offset[1] = gy_sum / GYRO_CALIB_SAMPLES;
    gyro_offset[2] = gz_sum / GYRO_CALIB_SAMPLES;
    Serial.printf("%s [CALIB] gyro_off = %.6f, %.6f, %.6f\n",
                  isoNow().c_str(), gyro_offset[0], gyro_offset[1], gyro_offset[2]);
    saveCalibration();
}

void calibrateAccel() {
    Serial.println(isoNow() + " [CALIB] Calibrating accel. Place level.");
    delay(1500);
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    for (int i = 0; i < ACCEL_CALIB_SAMPLES; i++) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            ax_sum += myICM.accX() / 1000.0f;
            ay_sum += myICM.accY() / 1000.0f;
            az_sum += myICM.accZ() / 1000.0f;
        }
        delay(10);
    }
    accel_offset[0] = ax_sum / ACCEL_CALIB_SAMPLES;
    accel_offset[1] = ay_sum / ACCEL_CALIB_SAMPLES;
    accel_offset[2] = (az_sum / ACCEL_CALIB_SAMPLES) - 1.0f;
    Serial.printf("%s [CALIB] accel_off = %.6f, %.6f, %.6f g\n",
                  isoNow().c_str(), accel_offset[0], accel_offset[1], accel_offset[2]);
    saveCalibration();
}

// ================= MADGWICK UPDATE =================
void madgwickUpdate(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float dt) {
    if (dt <= 0.0f || dt > 0.05f) dt = 1.0f / TARGET_FREQ;

    // Normalizar acelerômetro
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Magnetômetro
    float mn = sqrtf(mx * mx + my * my + mz * mz);
    bool hasMag = (mn > 0.0001f) && cfg.use_magnetometer;

    if (hasMag) {
        mx /= mn;
        my /= mn;
        mz /= mn;

        // Aplicar declinação magnética no plano XY
        float decl_rad = cfg.magnetic_declination_deg * 0.01745329252f;
        float cos_d = cosf(decl_rad);
        float sin_d = sinf(decl_rad);
        float mx_rot = mx * cos_d - my * sin_d;
        float my_rot = mx * sin_d + my * cos_d;
        mx = mx_rot;
        my = my_rot;
    }

    float _q0 = q0, _q1 = q1, _q2 = q2, _q3 = q3;

    // Pré-cálculos
    float q0q0 = _q0 * _q0;
    float q0q1 = _q0 * _q1;
    float q0q2 = _q0 * _q2;
    float q0q3 = _q0 * _q3;
    float q1q1 = _q1 * _q1;
    float q1q2 = _q1 * _q2;
    float q1q3 = _q1 * _q3;
    float q2q2 = _q2 * _q2;
    float q2q3 = _q2 * _q3;
    float q3q3 = _q3 * _q3;

    // Equações do Madgwick para acelerômetro
    float f1 = 2.0f * (q1q3 - q0q2) - ax;
    float f2 = 2.0f * (q0q1 + q2q3) - ay;
    float f3 = 2.0f * (0.5f - q1q1 - q2q2) - az;

    float s0 = -_q2 * f1 + _q1 * f2;
    float s1 = _q3 * f1 + _q0 * f2 - 4.0f * _q1 * f3;
    float s2 = -_q0 * f1 + _q3 * f2 - 4.0f * _q2 * f3;
    float s3 = _q1 * f1 + _q2 * f2;

    // Magnetômetro opcional
    if (hasMag && mn > 0.05f) {
        float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        float bx = sqrtf(hx * hx + hy * hy);
        float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        float f4 = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2) - mx;
        float f5 = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3) - my;
        float f6 = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2) - mz;

        s0 += -2.0f * bz * _q2 * f4 + 2.0f * bz * _q1 * f5 + 2.0f * bx * _q2 * f6;
        s1 += 2.0f * bz * _q3 * f4 + 2.0f * bz * _q0 * f5 + 2.0f * bx * _q3 * f6 - 4.0f * bz * _q1 * f6;
        s2 += -4.0f * bx * _q2 * f4 + 2.0f * bx * _q0 * f5 + 2.0f * bz * _q3 * f5 +
              2.0f * (bx * _q1 - 2.0f * bz * _q2) * f6;
        s3 += 2.0f * bx * _q1 * f4 + 2.0f * bx * _q2 * f5 + 2.0f * bz * _q0 * f6 - 4.0f * bx * _q3 * f6;
    }

    // Normalizar gradiente
    float s_norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (s_norm > 0.0f) {
        s0 /= s_norm;
        s1 /= s_norm;
        s2 /= s_norm;
        s3 /= s_norm;
    }

    // Taxa de variação do quat a partir do giroscópio
    float qDot0 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    float qDot1 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    float qDot2 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    float qDot3 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    float beta = cfg.madgwick_beta;

    // Correção pelo gradiente
    qDot0 -= beta * s0;
    qDot1 -= beta * s1;
    qDot2 -= beta * s2;
    qDot3 -= beta * s3;

    // Integrar para obter nova orientação
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    float qnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qnorm > 0.0f) {
        q0 /= qnorm;
        q1 /= qnorm;
        q2 /= qnorm;
        q3 /= qnorm;
    }
}

// ================= ROTATE VECTOR =================
void rotateVectorByQuaternion(const float q[4], const float vin[3], float vout[3]) {
    float q0_ = q[0], q1_ = q[1], q2_ = q[2], q3_ = q[3];

    float q0q0 = q0_ * q0_;
    float q1q1 = q1_ * q1_;
    float q2q2 = q2_ * q2_;
    float q3q3 = q3_ * q3_;

    float q0q1 = q0_ * q1_;
    float q0q2 = q0_ * q2_;
    float q0q3 = q0_ * q3_;
    float q1q2 = q1_ * q2_;
    float q1q3 = q1_ * q3_;
    float q2q3 = q2_ * q3_;

    float r00 = 1.0f - 2.0f * (q2q2 + q3q3);
    float r01 = 2.0f * (q1q2 - q0q3);
    float r02 = 2.0f * (q1q3 + q0q2);

    float r10 = 2.0f * (q1q2 + q0q3);
    float r11 = 1.0f - 2.0f * (q1q1 + q3q3);
    float r12 = 2.0f * (q2q3 - q0q1);

    float r20 = 2.0f * (q1q3 - q0q2);
    float r21 = 2.0f * (q2q3 + q0q1);
    float r22 = 1.0f - 2.0f * (q1q1 + q2q2);

    vout[0] = r00 * vin[0] + r01 * vin[1] + r02 * vin[2];
    vout[1] = r10 * vin[0] + r11 * vin[1] + r12 * vin[2];
    vout[2] = r20 * vin[0] + r21 * vin[1] + r22 * vin[2];
}

// ================= ZUPT HELPERS =================
void zuptPushAccelHoriz(const float a_world[3]) {
    float ax = a_world[0], ay = a_world[1];
    float horiz = sqrtf(ax * ax + ay * ay);
    accel_mag_window[accel_window_idx] = horiz;
    accel_window_idx = (accel_window_idx + 1) % ZUPT_WINDOW;
    if (!zupt_initialized) {
        zupt_samples_count++;
        if (zupt_samples_count >= ZUPT_WINDOW) zupt_initialized = true;
    }
}

float zuptComputeVariance() {
    float mean = 0.0f;
    for (int i = 0; i < ZUPT_WINDOW; i++) mean += accel_mag_window[i];
    mean /= ZUPT_WINDOW;
    float var = 0.0f;
    for (int i = 0; i < ZUPT_WINDOW; i++) {
        float d = accel_mag_window[i] - mean;
        var += d * d;
    }
    var /= ZUPT_WINDOW;
    return var;
}

bool isZUPT_local(float gyro_mag) {
    if (!zupt_initialized) return false;
    float var = zuptComputeVariance();
    bool accel_cond = (var < cfg.zupt_accel_var_threshold);
    bool gyro_cond = (gyro_mag < cfg.zupt_gyro_threshold);
    return (accel_cond || gyro_cond);
}

// ================= STEP DETECTION =================
void processBobbing(float a_world_z_nograv) {
    bob_hp = cfg.bob_hp_alpha * (bob_hp + a_world_z_nograv - bob_prev_x);
    bob_prev_x = a_world_z_nograv;
    float rect = fabs(bob_hp);
    bob_env = cfg.bob_lp_alpha * rect + (1.0f - cfg.bob_lp_alpha) * bob_env;
    unsigned long now = millis();
    if (bob_env > cfg.step_detect_thresh &&
        (now - last_step_ms) > (unsigned long)cfg.step_min_interval_ms) {
        last_step_ms = now;
        step_detected_flag = true;
        if (step_times_count < STEP_TIMES_MAX) {
            int idx = (step_times_head + step_times_count) % STEP_TIMES_MAX;
            step_times[idx] = now;
            step_times_count++;
        } else {
            step_times[step_times_head] = now;
            step_times_head = (step_times_head + 1) % STEP_TIMES_MAX;
        }
    }
}

float getStepFrequency() {
    if (step_times_count < 2) return 0.0f;
    int first_idx = step_times_head;
    int last_idx = (step_times_head + step_times_count - 1) % STEP_TIMES_MAX;
    unsigned long t_first = step_times[first_idx];
    unsigned long t_last = step_times[last_idx];
    float duration_s = (t_last > t_first) ? ((t_last - t_first) / 1000.0f) : 0.0f;
    if (duration_s <= 0.0f) return 0.0f;
    float freq = (step_times_count - 1) / duration_s;
    return freq;
}

// ================= YAW HELPERS =================
float getYawFromQuat(float q0_, float q1_, float q2_, float q3_) {
    float num = 2.0f * (q0_ * q3_ + q1_ * q2_);
    float den = 1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_);
    return atan2f(num, den);
}

void applyYawCorrection(float target_yaw, float gain) {
    float cur = getYawFromQuat(q0, q1, q2, q3);
    float delta = target_yaw - cur;
    while (delta > M_PI) delta -= 2.0f * M_PI;
    while (delta < -M_PI) delta += 2.0f * M_PI;
    float corr = delta * gain;

    float ca = cosf(corr * 0.5f);
    float sa = sinf(corr * 0.5f);

    float qr0 = ca, qr1 = 0.0f, qr2 = 0.0f, qr3 = sa;

    float out0 = qr0 * q0 - qr1 * q1 - qr2 * q2 - qr3 * q3;
    float out1 = qr0 * q1 + qr1 * q0 + qr2 * q3 - qr3 * q2;
    float out2 = qr0 * q2 - qr1 * q3 + qr2 * q0 + qr3 * q1;
    float out3 = qr0 * q3 + qr1 * q2 - qr2 * q1 + qr3 * q0;

    float n = sqrtf(out0 * out0 + out1 * out1 + out2 * out2 + out3 * out3);
    if (n > 0.0f) {
        q0 = out0 / n;
        q1 = out1 / n;
        q2 = out2 / n;
        q3 = out3 / n;
    }
}

// ================= VALIDATE IMU =================
bool validate_imu_data(const float* accel, const float* gyro, const float* mag, const float* quat) {
    float accel_mag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (accel_mag > MAX_ACCEL_MS2) return false;
    float gyro_mag = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    if (gyro_mag > MAX_GYRO_RADS) return false;
    float qmag = fabs(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    if (qmag < 0.2f || qmag > 2.0f) return false;
    return true;
}

// ================= MOVEMENT PREDICTION =================
void updateMovementPrediction() {
    float lq[4];
    if (xSemaphoreTake(imuMutex, (TickType_t)5) == pdTRUE) {
        lq[0] = q0;
        lq[1] = q1;
        lq[2] = q2;
        lq[3] = q3;
        xSemaphoreGive(imuMutex);
    } else {
        lq[0] = 1;
        lq[1] = lq[2] = lq[3] = 0;
    }
    predicted_quat[0] = lq[0];
    predicted_quat[1] = lq[1];
    predicted_quat[2] = lq[2];
    predicted_quat[3] = lq[3];
}

// ================= DEAD RECKONING =================
unsigned long moving_since = 0;

void updateDeadReckoning(float dt) {
    if (dt <= 0.0f || dt > 0.05f) return;

    float q_snap[4], a_sensor[3], gx, gy, gz;
    if (xSemaphoreTake(imuMutex, (TickType_t)5) == pdTRUE) {
        q_snap[0] = q0;
        q_snap[1] = q1;
        q_snap[2] = q2;
        q_snap[3] = q3;
        a_sensor[0] = accel_raw[0];
        a_sensor[1] = accel_raw[1];
        a_sensor[2] = accel_raw[2];
        gx = gyro_raw[0];
        gy = gyro_raw[1];
        gz = gyro_raw[2];
        xSemaphoreGive(imuMutex);
    } else
        return;

    float a_world[3];
    rotateVectorByQuaternion(q_snap, a_sensor, a_world);

    zuptPushAccelHoriz(a_world);
    float gyro_mag = sqrtf(gx * gx + gy * gy + gz * gz);
    bool local_zupt = isZUPT_local(gyro_mag);

    float a_world_z_nograv = a_world[2] - GRAVITY;
    processBobbing(a_world_z_nograv);

    float step_freq = getStepFrequency();
    float stride_used = (stride_zone_count > 0) ? lookupStrideForHz(step_freq) : cfg.stride_length;
    float speed_from_steps = stride_used * step_freq;

    // Detectar caminhada no lugar (esteira deslizante)
    bool walking_in_place = false;
    if (step_freq > 0.8f && step_freq < 3.0f) {
        float speed_h = sqrtf(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
        if (speed_h < 0.3f) {
            walking_in_place = true;
        }
    }

    if (local_zupt || walking_in_place) {
        if (getStepFrequency() > 0.1f) {
            float body_forward_sensor[3] = {1.0f, 0.0f, 0.0f};
            float body_forward_world[3];
            rotateVectorByQuaternion(q_snap, body_forward_sensor, body_forward_world);
            float bx = body_forward_world[0], by = body_forward_world[1];
            float norm = sqrtf(bx * bx + by * by);
            float dirx = 0.0f, diry = 0.0f;
            if (norm > 1e-6f) {
                dirx = bx / norm;
                diry = by / norm;
            }
            float speed = speed_from_steps;
            velocity[0] = dirx * speed;
            velocity[1] = diry * speed;
            velocity[2] += (a_world_z_nograv)*dt;
            if (zupt_state) {
                zupt_state = false;
                zupt_since = 0;
            }
        } else {
            if (!zupt_state) {
                zupt_state = true;
                zupt_since = millis();
            }
            velocity[0] = velocity[1] = 0.0f;
            velocity[2] = 0.0f;
        }
    } else {
        float body_forward_sensor[3] = {1.0f, 0.0f, 0.0f};
        float body_forward_world[3];
        rotateVectorByQuaternion(q_snap, body_forward_sensor, body_forward_world);
        float bx = body_forward_world[0], by = body_forward_world[1];
        float norm = sqrtf(bx * bx + by * by);
        float dirx = 0.0f, diry = 0.0f;
        if (norm > 1e-6f) {
            dirx = bx / norm;
            diry = by / norm;
        }
        float speed = speed_from_steps;
        velocity[0] = dirx * speed;
        velocity[1] = diry * speed;
        velocity[2] += (a_world_z_nograv)*dt;
        for (int i = 0; i < 3; i++) {
            if (velocity[i] > cfg.max_velocity) velocity[i] = cfg.max_velocity;
            if (velocity[i] < -cfg.max_velocity) velocity[i] = -cfg.max_velocity;
        }
        if (zupt_state) {
            zupt_state = false;
            zupt_since = 0;
        }
    }

    // Suavizar velocidade (anti-jitter)
    velocity_filtered[0] = cfg.velocity_smooth_alpha * velocity[0] +
                           (1.0f - cfg.velocity_smooth_alpha) * velocity_filtered[0];
    velocity_filtered[1] = cfg.velocity_smooth_alpha * velocity[1] +
                           (1.0f - cfg.velocity_smooth_alpha) * velocity_filtered[1];
    velocity_filtered[2] = cfg.velocity_smooth_alpha * velocity[2] +
                           (1.0f - cfg.velocity_smooth_alpha) * velocity_filtered[2];

    position[0] += velocity_filtered[0] * dt;
    position[1] += velocity_filtered[1] * dt;
    position[2] += velocity_filtered[2] * dt;
    height_estimate = position[2];

    if (zupt_state && (millis() - zupt_since > cfg.zupt_reset_ms)) {
        position[0] = position[1] = position[2] = 0.0f;
    }

    float speed_h = sqrtf(velocity_filtered[0] * velocity_filtered[0] +
                          velocity_filtered[1] * velocity_filtered[1]);
    if (speed_h > cfg.yaw_correction_speed) {
        if (moving_since == 0) moving_since = millis();
        if (millis() - moving_since > cfg.yaw_correction_time_ms) {
            float target_yaw = atan2f(velocity_filtered[1], velocity_filtered[0]);
            applyYawCorrection(target_yaw, cfg.yaw_correction_gain);
        }
    } else {
        moving_since = 0;
    }
}

// ================= ACTIVITY RECOGNITION =================
void updateActivityRecognition() {
    float ax = accel_raw[0], ay = accel_raw[1];
    float horiz_mag = sqrtf(ax * ax + ay * ay);
    const float alpha = 0.1f;
    static float accel_magnitude_filtered = 0.0f;
    accel_magnitude_filtered = alpha * horiz_mag + (1.0f - alpha) * accel_magnitude_filtered;

    if (accel_magnitude_filtered < cfg.x_threshold)
        current_activity = STATIONARY;
    else if (accel_magnitude_filtered < cfg.y_max_speed)
        current_activity = WALKING;
    else
        current_activity = RUNNING;
    
    current_activity_state = (uint8_t)current_activity;
}

void updateAdaptiveBeta() {
    switch (current_activity) {
        case STATIONARY:
            beta_adaptive = 0.008f;
            break;
        case WALKING:
            beta_adaptive = 0.022f;
            break;
        case RUNNING:
            beta_adaptive = 0.038f;
            break;
        case JUMPING:
            beta_adaptive = 0.050f;
            break;
        default:
            beta_adaptive = 0.022f;
            break;
    }
    cfg.madgwick_beta = beta_adaptive;
}

// ================= IMU READ =================
struct IMUSample {
    float ax, ay, az, gx, gy, gz, q0, q1, q2, q3;
};

void readIMUDataToSample(IMUSample &s) {
    if (!myICM.dataReady()) {
        s.ax = accel_raw[0];
        s.ay = accel_raw[1];
        s.az = accel_raw[2];
        s.gx = gyro_raw[0];
        s.gy = gyro_raw[1];
        s.gz = gyro_raw[2];
        s.q0 = q0;
        s.q1 = q1;
        s.q2 = q2;
        s.q3 = q3;
        return;
    }

    myICM.getAGMT();
    float acc_g[3] = {myICM.accX() / 1000.0f, myICM.accY() / 1000.0f, myICM.accZ() / 1000.0f};
    float gyr_dps[3] = {myICM.gyrX(), myICM.gyrY(), myICM.gyrZ()};
    float acc_ms2[3] = {
        (acc_g[0] - accel_offset[0]) * GRAVITY,
        (acc_g[1] - accel_offset[1]) * GRAVITY,
        (acc_g[2] - accel_offset[2]) * GRAVITY
    };

    for (int i = 0; i < 3; i++) acc_ms2[i] = kalman_filter_1d(kalman[i], acc_ms2[i]);

    float gyr_rads[3] = {
        gyr_dps[0] * 0.01745329252f - gyro_offset[0],
        gyr_dps[1] * 0.01745329252f - gyro_offset[1],
        gyr_dps[2] * 0.01745329252f - gyro_offset[2]
    };

    float mag[3] = {
        (myICM.magX() - mag_offset[0]) * mag_scale[0],
        (myICM.magY() - mag_offset[1]) * mag_scale[1],
        (myICM.magZ() - mag_offset[2]) * mag_scale[2]
    };

    float temp = myICM.temp();

    if (xSemaphoreTake(imuMutex, (TickType_t)10) == pdTRUE) {
        accel_raw[0] = acc_ms2[0];
        accel_raw[1] = acc_ms2[1];
        accel_raw[2] = acc_ms2[2];
        gyro_raw[0] = gyr_rads[0];
        gyro_raw[1] = gyr_rads[1];
        gyro_raw[2] = gyr_rads[2];
        mag_calib[0] = mag[0];
        mag_calib[1] = mag[1];
        mag_calib[2] = mag[2];
        temperature = temp;
        xSemaphoreGive(imuMutex);
    }

    s.ax = acc_ms2[0];
    s.ay = acc_ms2[1];
    s.az = acc_ms2[2];
    s.gx = gyr_rads[0];
    s.gy = gyr_rads[1];
    s.gz = gyr_rads[2];
    s.q0 = q0;
    s.q1 = q1;
    s.q2 = q2;
    s.q3 = q3;
}

// ================= PACK TELEMETRY (ATUALIZADO) =================
void packTelemetry(VRTrackerPacket_V6* packet) {
    memset(packet, 0, sizeof(VRTrackerPacket_V6));

    // Cabeçalho
    packet->magic = VRTRACKER_MAGIC_BYTE;
    memcpy(packet->tag, VRTRACKER_TAG, VRTRACKER_TAG_LEN);
    packet->timestamp_us = micros();
    packet->sequence = get_next_sequence();  // MUDANÇA: usar função atômica

    // Flags de status
    packet->status_flags = 0;
    if (zupt_active_state) packet->status_flags |= STATUS_ZUPT_ACTIVE;
    if (step_detected_flag) packet->status_flags |= STATUS_STEP_DETECTED;
    if (mag_calibrated_state) packet->status_flags |= STATUS_MAG_CALIBRATED;
    if (mag_interference_state) packet->status_flags |= STATUS_MAG_INTERFERENCE;
    if (low_battery_state) packet->status_flags |= STATUS_LOW_BATTERY;
    if (imu_error_state) packet->status_flags |= STATUS_ERROR_IMU;
    if (calibration_mode_active) packet->status_flags |= STATUS_CALIBRATION_MODE;

    step_detected_flag = false;  // Reset flag

    packet->activity_state = current_activity_state;
    packet->temperature = temperature;

    // Quaternions (protegidos por mutex)
    if (xSemaphoreTake(imuMutex, (TickType_t)5) == pdTRUE) {
        packet->quat[0] = q0;
        packet->quat[1] = q1;
        packet->quat[2] = q2;
        packet->quat[3] = q3;
        xSemaphoreGive(imuMutex);
    }
    
    packet->pred_quat[0] = predicted_quat[0];
    packet->pred_quat[1] = predicted_quat[1];
    packet->pred_quat[2] = predicted_quat[2];
    packet->pred_quat[3] = predicted_quat[3];

    // Sensor data (protegidos por mutex)
    if (xSemaphoreTake(imuMutex, (TickType_t)5) == pdTRUE) {
        memcpy(packet->accel, accel_raw, sizeof(packet->accel));
        memcpy(packet->gyro, gyro_raw, sizeof(packet->gyro));
        memcpy(packet->mag, mag_calib, sizeof(packet->mag));
        xSemaphoreGive(imuMutex);
    }

    // Velocity & Position (usar velocity_filtered)
    packet->velocity[0] = velocity_filtered[0];
    packet->velocity[1] = velocity_filtered[1];
    packet->velocity[2] = velocity_filtered[2];
    packet->position[0] = position[0];
    packet->position[1] = position[1];
    packet->position[2] = position[2];
    packet->height = height_estimate;

    // Filter params
    packet->beta = beta_adaptive;
    
    float step_freq = getStepFrequency();
    float stride_used = (stride_zone_count > 0) ? lookupStrideForHz(step_freq) : cfg.stride_length;
    packet->speed = stride_used * step_freq;
    packet->step_frequency = step_freq;

    // Direction
    float body_forward_sensor[3] = {1.0f, 0.0f, 0.0f};
    float body_forward_world[3];
    float q_snap[4];
    if (xSemaphoreTake(imuMutex, (TickType_t)5) == pdTRUE) {
        q_snap[0] = q0;
        q_snap[1] = q1;
        q_snap[2] = q2;
        q_snap[3] = q3;
        xSemaphoreGive(imuMutex);
    } else {
        q_snap[0] = 1.0f;
        q_snap[1] = q_snap[2] = q_snap[3] = 0.0f;
    }
    
    rotateVectorByQuaternion(q_snap, body_forward_sensor, body_forward_world);
    float bx = body_forward_world[0], by = body_forward_world[1];
    float norm = sqrtf(bx * bx + by * by);
    if (norm > 1e-6f) {
        packet->dir_x = bx / norm;
        packet->dir_y = by / norm;
    } else {
        packet->dir_x = 0.0f;
        packet->dir_y = 0.0f;
    }

    // Padding (já zerado por memset)
    memset(packet->padding, 0, sizeof(packet->padding));

    // CRC32: calcula sobre TODOS os bytes EXCETO os últimos 4
    packet->crc32 = calculate_crc32((const unsigned char*)packet,
                                     sizeof(VRTrackerPacket_V6) - sizeof(uint32_t));
}

// ================= SLOT MANAGEMENT =================
bool ensureConfigsDir() {
    if (!SPIFFS.exists(SLOT_DIR)) {
        if (!SPIFFS.mkdir(SLOT_DIR)) {
            Serial.println(isoNow() + " [ERR] failed to create configs dir");
            return false;
        }
    }
    return true;
}

String slotPath(int slot) {
    char p[64];
    snprintf(p, sizeof(p), "%s/slot_%d.json", SLOT_DIR, slot);
    return String(p);
}

bool saveSlotFromCurrentConfig(int slot, const char* name) {
    if (slot < 0 || slot >= MAX_SLOTS) return false;
    if (!ensureConfigsDir()) return false;

    DynamicJsonDocument cur(1536);
    File f = SPIFFS.open(CONFIG_PATH, "r");
    if (f) {
        deserializeJson(cur, f);
        f.close();
    }

    DynamicJsonDocument out(1536);
    out["name"] = name ? String(name) : String("slot");
    out["created"] = isoNow();
    out["slot"] = slot;
    JsonObject jcfg = out.createNestedObject("cfg");

    JsonObject curObj = cur.as<JsonObject>();
    for (JsonPair p : curObj) {
        jcfg[p.key().c_str()] = p.value();
    }

    String path = slotPath(slot);
    File fo = SPIFFS.open(path.c_str(), "w");
    if (!fo) return false;
    if (serializeJson(out, fo) == 0) {
        fo.close();
        return false;
    }
    fo.close();
    Serial.printf("%s [SLOT] saved slot %d -> %s\n", isoNow().c_str(), slot, path.c_str());
    return true;
}

bool loadSlotToMainConfig(int slot) {
    if (slot < 0 || slot >= MAX_SLOTS) return false;
    String path = slotPath(slot);
    if (!SPIFFS.exists(path.c_str())) return false;

    File f = SPIFFS.open(path.c_str(), "r");
    if (!f) return false;

    DynamicJsonDocument doc(1536);
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    if (!doc.containsKey("cfg")) return false;
    JsonObject jcfg = doc["cfg"].as<JsonObject>();

    File fo = SPIFFS.open(CONFIG_PATH, "w");
    if (!fo) return false;
    if (serializeJson(jcfg, fo) == 0) {
        fo.close();
        return false;
    }
    fo.close();

    loadConfig();
    Serial.printf("%s [SLOT] loaded slot %d into %s\n", isoNow().c_str(), slot, CONFIG_PATH);
    return true;
}

bool deleteSlotFile(int slot) {
    if (slot < 0 || slot >= MAX_SLOTS) return false;
    String path = slotPath(slot);
    if (!SPIFFS.exists(path.c_str())) return false;
    return SPIFFS.remove(path.c_str());
}

String listSlotsJson() {
    DynamicJsonDocument out(1024);
    JsonArray arr = out.createNestedArray("slots");

    for (int i = 0; i < MAX_SLOTS; i++) {
        String p = slotPath(i);
        if (SPIFFS.exists(p.c_str())) {
            File f = SPIFFS.open(p.c_str(), "r");
            if (!f) continue;

            DynamicJsonDocument d(512);
            if (deserializeJson(d, f) == DeserializationError::Ok) {
                JsonObject obj = arr.createNestedObject();
                obj["slot"] = i;
                obj["name"] = d["name"] | String("");
                obj["created"] = d["created"] | String("");
            }
            f.close();
        }
    }

    String s;
    serializeJson(out, s);
    return s;
}

// ================= WEB HTML =================
const char indexHtml[] PROGMEM = R"html(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>VR Tracker Setup</title>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <style>
    body{font-family:Arial,sans-serif;margin:12px;background:#f5f5f5}
    h1{font-size:20px;color:#333}
    .box{background:#fff;border:1px solid #ddd;padding:12px;margin-bottom:12px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}
    label{display:block;margin-top:8px;font-weight:500;color:#555}
    input[type=text],input[type=password],input[type=number],textarea{width:100%;padding:8px;box-sizing:border-box;border:1px solid #ccc;border-radius:4px;font-size:14px}
    button{background:#007bff;color:#fff;padding:10px 16px;border:none;border-radius:4px;cursor:pointer;font-size:14px;margin-top:8px}
    button:hover{background:#0056b3}
    button.secondary{background:#6c757d}
    button.secondary:hover{background:#5a6268}
    button.danger{background:#dc3545}
    button.danger:hover{background:#c82333}
    table{width:100%;border-collapse:collapse;margin-top:12px}
    td,th{padding:8px;border-bottom:1px solid #eee;text-align:left;font-size:14px}
    th{background:#f8f9fa;font-weight:600}
    pre{background:#f4f4f4;padding:12px;border-radius:4px;overflow-x:auto;font-size:12px}
    .status-ok{color:#28a745}
    .status-warn{color:#ffc107}
    .status-error{color:#dc3545}
  </style>
</head>
<body>
  <h1>🎮 VR Tracker - Setup & Config</h1>
  
  <div class="box">
    <h3>📊 Device Status</h3>
    <pre id="status">Loading...</pre>
    <button onclick="fetchStatus()">🔄 Refresh</button>
  </div>

  <div class="box">
    <h3>📡 WiFi Scan & Connect</h3>
    <button onclick="scanWifi()">🔍 Scan Networks</button>
    <div id="networks"></div>
    <hr>
    <form onsubmit="connectWiFi();return false;">
      <label>SSID</label>
      <input type="text" id="ssid">
      <label>Password</label>
      <input type="password" id="pass">
      <button type="submit">✔️ Connect & Save</button>
    </form>
  </div>

  <div class="box">
    <h3>⚙️ Active Config (tracker_cfg.json)</h3>
    <button onclick="loadActiveCfg()">📥 Load Config</button>
    <button onclick="saveActiveCfg()">💾 Save Config</button>
    <button class="secondary" onclick="resetPose()">🔄 Reset Pose</button>
    <p style="font-size:12px;color:#888">Edit carefully. Invalid values may break tracking.</p>
    <textarea id="activeCfg" rows="20" style="font-family:monospace;font-size:12px"></textarea>
  </div>

  <div class="box">
    <h3>💾 Config Slots (0-9)</h3>
    <div id="slots">Loading slots...</div>
    <hr>
    <form onsubmit="saveSlot();return false;">
      <label>Save to Slot</label>
      <input type="number" id="slotIndex" min="0" max="9" value="0">
      <input type="text" id="slotName" placeholder="Profile name">
      <button type="submit">💾 Save Slot</button>
    </form>
    <form onsubmit="uploadSlot();return false;" enctype="multipart/form-data">
      <label>Upload to Slot</label>
      <input type="file" id="slotFile">
      <input type="number" id="uploadIndex" min="0" max="9" value="0">
      <button type="submit">📤 Upload</button>
    </form>
  </div>

  <div class="box">
    <h3>🔧 Calibration</h3>
    <button onclick="calibrateGyro()">🎯 Calibrate Gyro</button>
    <button onclick="calibrateAccel()">📏 Calibrate Accel</button>
    <p style="font-size:12px;color:#888">Place tracker on flat surface before calibrating</p>
  </div>

<script>
function fetchStatus(){
  fetch('/status').then(r=>r.json()).then(js=>{
    document.getElementById('status').textContent=JSON.stringify(js,null,2);
  }).catch(e=>alert('Error: '+e));
}
function scanWifi(){
  fetch('/scan').then(r=>r.json()).then(js=>{
    let out='<table><tr><th>SSID</th><th>RSSI</th><th></th></tr>';
    js.networks.forEach(n=>{
      out+='<tr><td>'+n.ssid+'</td><td>'+n.rssi+'</td><td><button onclick="fillConnect(\''+n.ssid+'\')">Use</button></td></tr>';
    });
    out+='</table>';
    document.getElementById('networks').innerHTML=out;
  });
}
function fillConnect(ssid){document.getElementById('ssid').value=ssid;}
function saveSlot(){
  const idx=document.getElementById('slotIndex').value;
  const name=document.getElementById('slotName').value;
  fetch('/slot/save?slot='+idx+'&name='+encodeURIComponent(name)).then(r=>r.json()).then(js=>{
    alert(js.msg);loadSlots();
  });
}
function uploadSlot(){
  const file=document.getElementById('slotFile').files[0];
  const idx=document.getElementById('uploadIndex').value;
  if(!file){alert('Choose a file');return;}
  const fr=new FormData();
  fr.append('slot',idx);
  fr.append('file',file);
  fetch('/slot/upload',{method:'POST',body:fr}).then(r=>r.json()).then(js=>{
    alert(js.msg);loadSlots();
  });
}
function loadSlots(){
  fetch('/slot/list').then(r=>r.json()).then(js=>{
    let out='<table><tr><th>Slot</th><th>Name</th><th>Actions</th></tr>';
    js.slots.forEach(s=>{
      out+='<tr><td>'+s.slot+'</td><td>'+s.name+'</td><td>';
      out+='<button onclick="loadSlot('+s.slot+')">Load</button> ';
      out+='<button onclick="downloadSlot('+s.slot+')">Download</button> ';
      out+='<button class="danger" onclick="deleteSlot('+s.slot+')">Delete</button>';
      out+='</td></tr>';
    });
    out+='</table>';
    document.getElementById('slots').innerHTML=out;
  });
}
function loadSlot(i){fetch('/slot/load?slot='+i).then(r=>r.json()).then(js=>{alert(js.msg);loadActiveCfg();});}
function deleteSlot(i){if(confirm('Delete slot '+i+'?'))fetch('/slot/delete?slot='+i).then(r=>r.json()).then(js=>{alert(js.msg);loadSlots();});}
function downloadSlot(i){window.location='/slot/download?slot='+i;}
function connectWiFi(){
  const ss=document.getElementById('ssid').value;
  const pw=document.getElementById('pass').value;
  fetch('/connect',{method:'POST',body:new URLSearchParams({ssid:ss,pass:pw})}).then(r=>r.json()).then(js=>{
    alert(js.msg);
  });
}
function loadActiveCfg(){
  fetch('/config').then(r=>{
    if(!r.ok)throw new Error('HTTP '+r.status);
    return r.json();
  }).then(js=>{
    document.getElementById('activeCfg').value=JSON.stringify(js,null,2);
  }).catch(e=>alert('Error loading config: '+e));
}
function saveActiveCfg(){
  const txt=document.getElementById('activeCfg').value;
  let parsed;
  try{parsed=JSON.parse(txt);}catch(e){alert('Invalid JSON: '+e);return;}
  fetch('/config',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(parsed)
  }).then(r=>r.json()).then(js=>{
    alert(js.msg||'Config saved');
  }).catch(e=>alert('Error saving config: '+e));
}
function resetPose(){
  if(confirm('Reset pose and position?'))
  fetch('/reset_pose',{method:'POST'}).then(r=>r.json()).then(js=>alert(js.msg));
}
function calibrateGyro(){
  if(confirm('Keep device still for gyro calibration'))
  fetch('/calib_gyro',{method:'POST'}).then(r=>r.json()).then(js=>alert(js.msg));
}
function calibrateAccel(){
  if(confirm('Place device level for accel calibration'))
  fetch('/calib_accel',{method:'POST'}).then(r=>r.json()).then(js=>alert(js.msg));
}
window.onload=()=>{fetchStatus();loadSlots();}
</script>
</body>
</html>
)html";

// ================= WEB HANDLERS =================
void handleRoot(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", indexHtml);
}

void handleStatus(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(512);
    doc["uptime_ms"] = millis();
    doc["ip"] = WiFi.localIP().toString();
    doc["ap_mode"] = (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA);
    doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
    doc["buffer_count"] = ring_buffer.count;
    doc["buffer_size"] = RING_BUFFER_SIZE;
    doc["packets_sent"] = packet_count;
    doc["packets_dropped"] = dropped_packets;
    doc["activity"] = current_activity;
    doc["zupt_active"] = zupt_state;
    doc["imu_error"] = imu_error_state;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
}

void handleScan(AsyncWebServerRequest *request) {
    int n = WiFi.scanNetworks();
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.createNestedArray("networks");
    for (int i = 0; i < n; i++) {
        JsonObject it = arr.createNestedObject();
        it["ssid"] = WiFi.SSID(i);
        it["rssi"] = WiFi.RSSI(i);
        it["secure"] = (int)WiFi.encryptionType(i);
    }
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
}

void handleConfigGet(AsyncWebServerRequest *request) {
    if (!SPIFFS.exists(CONFIG_PATH)) {
        request->send(404, "application/json", "{\"msg\":\"no config\"}");
        return;
    }
    File f = SPIFFS.open(CONFIG_PATH, "r");
    if (!f) {
        request->send(500, "application/json", "{\"msg\":\"open failed\"}");
        return;
    }
    String content = f.readString();
    f.close();
    request->send(200, "application/json", content);
}

void handleConfigSet(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    static String bodyBuffer;
    if (index == 0) bodyBuffer = "";
    bodyBuffer += String((char*)data).substring(0, len);
    
    if (index + len == total) {
        DynamicJsonDocument doc(1536);
        DeserializationError err = deserializeJson(doc, bodyBuffer);
        if (err) {
            request->send(400, "application/json", "{\"msg\":\"invalid json\"}");
            return;
        }

        File f = SPIFFS.open(CONFIG_PATH, "w");
        if (!f || serializeJson(doc, f) == 0) {
            if (f) f.close();
            request->send(500, "application/json", "{\"msg\":\"write failed\"}");
            return;
        }
        f.close();
        loadConfig();
        request->send(200, "application/json", "{\"msg\":\"saved\"}");
    }
}

void handleConnect(AsyncWebServerRequest *request) {
    if (!request->hasParam("ssid", true) || !request->hasParam("pass", true)) {
        request->send(400, "application/json", "{\"msg\":\"missing params\"}");
        return;
    }

    String ssid_ = request->getParam("ssid", true)->value();
    String pass = request->getParam("pass", true)->value();

    if (ssid_.length() == 0) {
        request->send(400, "application/json", "{\"msg\":\"ssid empty\"}");
        return;
    }

    Serial.printf("%s [WIFI] Trying to connect to SSID='%s'\n", isoNow().c_str(), ssid_.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_.c_str(), pass.c_str());

    unsigned long start = millis();
    bool ok = false;
    while (millis() - start < 8000) {
        if (WiFi.status() == WL_CONNECTED) {
            ok = true;
            break;
        }
        delay(200);
    }

    if (ok) {
        wifi_ssid = ssid_;
        wifi_password = pass;
        saveConfig();
        request->send(200, "application/json", "{\"msg\":\"connected\"}");
    } else {
        request->send(500, "application/json", "{\"msg\":\"failed\"}");
    }
}

void handleResetPose(AsyncWebServerRequest *request) {
    if (xSemaphoreTake(imuMutex, (TickType_t)50) == pdTRUE) {
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
        position[0] = position[1] = position[2] = 0.0f;
        velocity[0] = velocity[1] = velocity[2] = 0.0f;
        velocity_filtered[0] = velocity_filtered[1] = velocity_filtered[2] = 0.0f;
        xSemaphoreGive(imuMutex);
    }
    Serial.println(isoNow() + " [CMD] Pose reset via web");
    request->send(200, "application/json", "{\"msg\":\"pose reset\"}");
}

void handleCalibGyro(AsyncWebServerRequest *request) {
    calibrateGyro();
    request->send(200, "application/json", "{\"msg\":\"gyro calibrated\"}");
}

void handleCalibAccel(AsyncWebServerRequest *request) {
    calibrateAccel();
    request->send(200, "application/json", "{\"msg\":\"accel calibrated\"}");
}

void handleSlotSave(AsyncWebServerRequest *request) {
    if (!request->hasParam("slot")) {
        request->send(400, "application/json", "{\"msg\":\"missing slot\"}");
        return;
    }
    int slot = request->getParam("slot")->value().toInt();
    String name = request->hasParam("name") ? request->getParam("name")->value() : "slot";
    if (saveSlotFromCurrentConfig(slot, name.c_str())) {
        request->send(200, "application/json", "{\"msg\":\"saved\"}");
    } else {
        request->send(500, "application/json", "{\"msg\":\"save failed\"}");
    }
}

void handleSlotList(AsyncWebServerRequest *request) {
    String s = listSlotsJson();
    request->send(200, "application/json", s);
}

void handleSlotLoad(AsyncWebServerRequest *request) {
    if (!request->hasParam("slot")) {
        request->send(400, "application/json", "{\"msg\":\"missing slot\"}");
        return;
    }
    int slot = request->getParam("slot")->value().toInt();
    if (loadSlotToMainConfig(slot)) {
        request->send(200, "application/json", "{\"msg\":\"loaded\"}");
    } else {
        request->send(500, "application/json", "{\"msg\":\"load failed\"}");
    }
}

void handleSlotDelete(AsyncWebServerRequest *request) {
    if (!request->hasParam("slot")) {
        request->send(400, "application/json", "{\"msg\":\"missing slot\"}");
        return;
    }
    int slot = request->getParam("slot")->value().toInt();
    if (deleteSlotFile(slot)) {
        request->send(200, "application/json", "{\"msg\":\"deleted\"}");
    } else {
        request->send(500, "application/json", "{\"msg\":\"delete failed\"}");
    }
}

void handleSlotDownload(AsyncWebServerRequest *request) {
    if (!request->hasParam("slot")) {
        request->send(404, "application/json", "{\"msg\":\"missing slot\"}");
        return;
    }
    int slot = request->getParam("slot")->value().toInt();
    String p = slotPath(slot);
    if (!SPIFFS.exists(p.c_str())) {
        request->send(404, "application/json", "{\"msg\":\"not found\"}");
        return;
    }
    request->send(SPIFFS, p.c_str(), "application/json");
}

// ================= ACCESS POINT =================
bool startAPPortal() {
    Serial.println(isoNow() + " [AP] starting AP for setup...");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASS);
    delay(500);
    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.printf("%s [AP] SSID: %s  IP: %s\n", isoNow().c_str(), AP_SSID, apIP.toString().c_str());
    return true;
}

void stopAPPortal() {
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
}

void startMDNS() {
    if (MDNS.begin("vrtracker")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println(isoNow() + " [mDNS] started: vrtracker.local");
    } else {
        Serial.println(isoNow() + " [mDNS] failed");
    }
}

// ================= SETUP WEB ROUTES =================
void setupWebServerRoutes() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/scan", HTTP_GET, handleScan);
    server.on("/config", HTTP_GET, handleConfigGet);
    server.on("/config", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL, handleConfigSet);
    server.on("/connect", HTTP_POST, handleConnect);
    server.on("/reset_pose", HTTP_POST, handleResetPose);
    server.on("/calib_gyro", HTTP_POST, handleCalibGyro);
    server.on("/calib_accel", HTTP_POST, handleCalibAccel);
    server.on("/slot/save", HTTP_GET, handleSlotSave);
    server.on("/slot/list", HTTP_GET, handleSlotList);
    server.on("/slot/load", HTTP_GET, handleSlotLoad);
    server.on("/slot/delete", HTTP_GET, handleSlotDelete);
    server.on("/slot/download", HTTP_GET, handleSlotDownload);
    
    // Upload handler
    server.on("/slot/upload", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            request->send(200, "application/json", "{\"msg\":\"uploaded\"}");
        },
        [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            static File uploadFile;
            static int currentSlot = 0;
            
            if (index == 0) {
                if (request->hasParam("slot", true)) {
                    currentSlot = request->getParam("slot", true)->value().toInt();
                }
                String tmpPath = String("/tmp_upload_slot_") + String(currentSlot) + ".json";
                uploadFile = SPIFFS.open(tmpPath.c_str(), "w");
            }
            
            if (uploadFile) {
                uploadFile.write(data, len);
            }
            
            if (final) {
                if (uploadFile) {
                    uploadFile.close();
                }
                String tmpPath = String("/tmp_upload_slot_") + String(currentSlot) + ".json";
                String target = slotPath(currentSlot);
                SPIFFS.remove(target.c_str());
                SPIFFS.rename(tmpPath.c_str(), target.c_str());
            }
        }
    );

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });
}

// ================= IMU TASK =================
void imuTask(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static uint32_t lastMicros = 0;

    // Variáveis para bias dinâmico melhorado
    static unsigned long calm_since = 0;
    static float gyro_bias_accumulator[3] = {0, 0, 0};
    static int bias_sample_count = 0;
    static const int BIAS_WINDOW_SAMPLES = 200;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t now = micros();
        float dt = (lastMicros == 0) ? (1.0f / TARGET_FREQ) : ((now - lastMicros) / 1e6f);
        lastMicros = now;

        IMUSample s;
        readIMUDataToSample(s);
        madgwickUpdate(s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                       mag_calib[0], mag_calib[1], mag_calib[2], dt);

        float local_gyro_mag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);

        // Bias dinâmico com janela deslizante
        if (zupt_initialized && isZUPT_local(local_gyro_mag)) {
            if (calm_since == 0) {
                calm_since = millis();
                bias_sample_count = 0;
                gyro_bias_accumulator[0] = 0;
                gyro_bias_accumulator[1] = 0;
                gyro_bias_accumulator[2] = 0;
            }

            unsigned long calm_duration = millis() - calm_since;

            if (bias_sample_count < BIAS_WINDOW_SAMPLES) {
                gyro_bias_accumulator[0] += s.gx;
                gyro_bias_accumulator[1] += s.gy;
                gyro_bias_accumulator[2] += s.gz;
                bias_sample_count++;
            }

            if (calm_duration > 2000 && bias_sample_count >= BIAS_WINDOW_SAMPLES) {
                float bias_x = gyro_bias_accumulator[0] / bias_sample_count;
                float bias_y = gyro_bias_accumulator[1] / bias_sample_count;
                float bias_z = gyro_bias_accumulator[2] / bias_sample_count;

                const float alpha = 0.002f;
                gyro_offset[0] = (1.0f - alpha) * gyro_offset[0] + alpha * bias_x;
                gyro_offset[1] = (1.0f - alpha) * gyro_offset[1] + alpha * bias_y;
                gyro_offset[2] = (1.0f - alpha) * gyro_offset[2] + alpha * bias_z;

                static unsigned long last_log = 0;
                if (millis() - last_log > 10000) {
                    Serial.printf("%s [BIAS] Gyro offset: [%.6f, %.6f, %.6f]\n",
                                  isoNow().c_str(), gyro_offset[0], gyro_offset[1], gyro_offset[2]);
                    last_log = millis();
                }

                bias_sample_count = 0;
                gyro_bias_accumulator[0] = 0;
                gyro_bias_accumulator[1] = 0;
                gyro_bias_accumulator[2] = 0;
            }
        } else {
            calm_since = 0;
            bias_sample_count = 0;
        }

        updateDeadReckoning(dt);
        updateActivityRecognition();
        updateAdaptiveBeta();
        updateMovementPrediction();
    }
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println("\n=== VR TRACKER CONSOLIDATED 2025 - ESP32-S3 (179 BYTES) ===");
    Serial.println("Features: V6 Protocol | AsyncWeb | 400Hz | ZUPT | Anti-Jitter");

    if (!SPIFFS.begin(true)) {
        Serial.println(isoNow() + " [WARN] SPIFFS mount failed");
    }

    // I2C & IMU
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
    delay(100);
    Serial.println(isoNow() + " [IMU] Initializing ICM-20948...");
    myICM.begin(Wire, false);
    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.print(isoNow() + " [ERR] ICM-20948 status: ");
        Serial.println(myICM.statusString());
        imu_error_state = true;
    } else {
        Serial.println(isoNow() + " [OK] ICM-20948 initialized");
    }

    // Mutexes
    imuMutex = xSemaphoreCreateMutex();
    ringMutex = xSemaphoreCreateMutex();
    seqMutex = xSemaphoreCreateMutex();  // NOVO
    
    if (!imuMutex || !ringMutex || !seqMutex) {
        Serial.println(isoNow() + " [ERR] mutex creation failed");
        while (1) delay(1000);
    }

    ensureConfigsDir();
    loadConfig();
    loadCalibration();

    // WiFi
    if (wifi_ssid.length() == 0) {
        Serial.println(isoNow() + " [WARN] WiFi SSID empty. Starting AP portal.");
        startAPPortal();
    } else {
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
        int wto = 0;
        Serial.print(isoNow() + " [WIFI] Connecting");
        while (WiFi.status() != WL_CONNECTED && wto < 50) {
            delay(200);
            Serial.print(".");
            wto++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n%s [OK] WiFi IP: %s RSSI:%d\n",
                          isoNow().c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
            Udp.begin(udpPort);
            startMDNS();
        } else {
            Serial.println("\n" + isoNow() + " [WARN] WiFi offline - starting AP portal");
            startAPPortal();
        }
    }

    // Web Server
    setupWebServerRoutes();
    server.begin();
    Serial.println(isoNow() + " [WEB] Web server started");

    // IMU Task
    BaseType_t ok = xTaskCreatePinnedToCore(
        imuTask,
        "IMU_TASK", 8192, NULL, 3, NULL, 1
    );
    if (ok != pdPASS) {
        Serial.println(isoNow() + " [ERR] create IMU task failed");
    } else {
        Serial.println(isoNow() + " [OK] IMU task started on core 1");
    }

    Serial.println(isoNow() + " [READY] Tracker ready! Packet size: 179 bytes | Rate: 400 Hz");
    Serial.println("==========================================================");
}

// ================= LOOP =================
void loop() {
    unsigned long now = millis();
    static unsigned long lastSend = 0;
    static unsigned long lastStatus = 0;
    static unsigned long lastWiFiCheck = 0;

    // DNS for AP mode
    dnsServer.processNextRequest();

    // Status log
    if (now - lastStatus >= 10000) {
        lastStatus = now;
        const char* names[] = {"STATIONARY", "WALK", "RUN", "JUMP"};
        const char* ws = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "BUFFER";
        Serial.printf("[%lums] WiFi:%s | Buffer:%d/%d | Packets:%u | Lost:%u\n",
                      millis(), ws, ring_buffer.count, RING_BUFFER_SIZE, packet_count, dropped_packets);
        Serial.printf("Act:%s | Beta:%.3f | ZUPT:%s\n",
                      names[current_activity], beta_adaptive, zupt_state ? "ON" : "OFF");
        if (xSemaphoreTake(imuMutex, (TickType_t)5) == pdTRUE) {
            Serial.printf("Quat: [%.3f,%.3f,%.3f,%.3f]\n", q0, q1, q2, q3);
            Serial.printf("Pos: [%.2f,%.2f,%.2f] Vel: [%.2f,%.2f,%.2f]\n",
                          position[0], position[1], position[2],
                          velocity_filtered[0], velocity_filtered[1], velocity_filtered[2]);
            xSemaphoreGive(imuMutex);
        }
    }

    // UDP Send
    if (millis() - lastSend >= UDP_SEND_INTERVAL_MS) {
        lastSend = millis();
        packet_count++;

        VRTrackerPacket_V6 telemetryPacket;

        // VALIDAÇÃO: Garantir 179 bytes
        if (sizeof(VRTrackerPacket_V6) != VRTRACKER_PACKET_SIZE) {
            Serial.printf("[ERR] Packet size mismatch: %d != %d\n", 
                          sizeof(VRTrackerPacket_V6), VRTRACKER_PACKET_SIZE);
            dropped_packets++;
            goto nextPacket;
        }

        float qtmp[4], accel_tmp[3], gyro_tmp[3], mag_tmp[3];
        if (xSemaphoreTake(imuMutex, (TickType_t)5) == pdTRUE) {
            qtmp[0] = q0;
            qtmp[1] = q1;
            qtmp[2] = q2;
            qtmp[3] = q3;
            accel_tmp[0] = accel_raw[0];
            accel_tmp[1] = accel_raw[1];
            accel_tmp[2] = accel_raw[2];
            gyro_tmp[0] = gyro_raw[0];
            gyro_tmp[1] = gyro_raw[1];
            gyro_tmp[2] = gyro_raw[2];
            mag_tmp[0] = mag_calib[0];
            mag_tmp[1] = mag_calib[1];
            mag_tmp[2] = mag_calib[2];
            xSemaphoreGive(imuMutex);
        } else {
            dropped_packets++;
            goto nextPacket;
        }

        if (!validate_imu_data(accel_tmp, gyro_tmp, mag_tmp, qtmp)) {
            dropped_packets++;
            goto nextPacket;
        }

        packTelemetry(&telemetryPacket);

        if (WiFi.status() == WL_CONNECTED) {
            Udp.beginPacket(udpAddress, udpPort);
            Udp.write((uint8_t*)&telemetryPacket, sizeof(VRTrackerPacket_V6));
            Udp.endPacket();
        } else {
            ring_buffer_push((uint8_t*)&telemetryPacket, sizeof(VRTrackerPacket_V6));
        }
    }

nextPacket:

    // WiFi reconnect & buffer flush
    if (millis() - lastWiFiCheck >= 30000) {
        lastWiFiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println(isoNow() + " [WIFI] Reconnecting...");
            WiFi.reconnect();
        } else {
            uint8_t tmp[PACKET_SIZE];
            uint16_t l;
            int sent = 0;
            while (ring_buffer_pop(tmp, l)) {
                Udp.beginPacket(udpAddress, udpPort);
                Udp.write(tmp, l);
                Udp.endPacket();
                sent++;
                delay(1);
            }
            if (sent) {
                Serial.printf("%s [RECOVERY] sent %d buffered packets\n", isoNow().c_str(), sent);
            }
        }
    }

    delay(1);
}

// ================= END OF FILE =================