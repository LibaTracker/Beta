// protocol.h - VRTracker V6 Protocol Definition (FINAL)
// Este arquivo DEVE ser idêntico em ESP32 e PC!
// VERSÃO: 179 bytes (validada)

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// Configurações do protocolo
#define VRTRACKER_PROTOCOL_VERSION 6
#define VRTRACKER_MAGIC_BYTE       0xAA
#define VRTRACKER_TAG              "VRTRACKER_V6"
#define VRTRACKER_TAG_LEN          12
#define VRTRACKER_PACKET_SIZE      179  // Tamanho FIXO (validado)

// Status flags (bitfield)
#define STATUS_ZUPT_ACTIVE          0x01
#define STATUS_STEP_DETECTED        0x02
#define STATUS_MAG_CALIBRATED       0x04
#define STATUS_MAG_INTERFERENCE     0x08
#define STATUS_LOW_BATTERY          0x10
#define STATUS_ERROR_IMU            0x20
#define STATUS_CALIBRATION_MODE     0x40
#define STATUS_RESERVED             0x80

// Activity states
enum ActivityState {
    ACTIVITY_STATIONARY = 0,
    ACTIVITY_WALKING    = 1,
    ACTIVITY_RUNNING    = 2,
    ACTIVITY_JUMPING    = 3
};

// Estrutura do pacote (packed, little-endian)
// IMPORTANTE: Sem padding automático!
#pragma pack(push, 1)
struct VRTrackerPacket_V6 {
    // ========== HEADER (25 bytes) ==========
    uint8_t  magic;                      // 0xAA (1 byte)
    char     tag[VRTRACKER_TAG_LEN];     // "VRTRACKER_V6" (12 bytes)
    uint64_t timestamp_us;               // Microssegundos desde boot (8 bytes)
    uint32_t sequence;                   // Número de sequência (4 bytes)
    
    // ========== STATUS (2 bytes) ==========
    uint8_t  status_flags;               // Bitfield (1 byte)
    uint8_t  activity_state;             // Estado de atividade (1 byte)
    
    // ========== TEMPERATURA (4 bytes) ==========
    float    temperature;                // Celsius (4 bytes)
    
    // ========== ORIENTAÇÃO (32 bytes) ==========
    float    quat[4];                    // Quaternion atual [w, x, y, z] (16 bytes)
    float    pred_quat[4];               // Quaternion predito [w, x, y, z] (16 bytes)
    
    // ========== DADOS IMU RAW (36 bytes) ==========
    float    accel[3];                   // Aceleração [x, y, z] em m/s² (12 bytes)
    float    gyro[3];                    // Giroscópio [x, y, z] em rad/s (12 bytes)
    float    mag[3];                     // Magnetômetro [x, y, z] em µT (12 bytes)
    
    // ========== MOVIMENTO (28 bytes) ==========
    float    velocity[3];                // Velocidade [x, y, z] em m/s (12 bytes)
    float    position[3];                // Posição [x, y, z] em metros (12 bytes)
    float    height;                     // Altura estimada em metros (4 bytes)
    
    // ========== PARÂMETROS DE FILTRO (8 bytes) ==========
    float    beta;                       // Beta do filtro Madgwick (4 bytes)
    float    stride_length;              // Comprimento de passo em metros (4 bytes)
    
    // ========== STEP DETECTION (16 bytes) ==========
    float    speed;                      // Velocidade calculada de passos (m/s) (4 bytes)
    float    step_frequency;             // Frequência de passos (Hz) (4 bytes)
    float    dir_x;                      // Direção normalizada X (4 bytes)
    float    dir_y;                      // Direção normalizada Y (4 bytes)
    
    // ========== RESERVADO FUTURO (20 bytes) ==========
    float    reserved1;                  // (4 bytes)
    float    reserved2;                  // (4 bytes)
    float    reserved3;                  // (4 bytes)
    float    reserved4;                  // (4 bytes)
    float    reserved5;                  // (4 bytes)
    
    // ========== PADDING E CRC (8 bytes) ==========
    uint8_t  padding[4];                 // Alinhamento (4 bytes)
    uint32_t crc32;                      // CRC32 calculado sobre bytes [0..174] (4 bytes)
};
#pragma pack(pop)

// Verificação de tamanho (em tempo de compilação)
#ifdef __cplusplus
static_assert(sizeof(VRTrackerPacket_V6) == VRTRACKER_PACKET_SIZE, 
              "VRTrackerPacket_V6 size must be exactly 179 bytes!");
#endif

// Layout de bytes (para referência):
// Offset | Campo           | Tamanho | Total Acumulado
// -------|-----------------|---------|----------------
// 0      | magic           | 1       | 1
// 1      | tag             | 12      | 13
// 13     | timestamp_us    | 8       | 21
// 21     | sequence        | 4       | 25
// 25     | status_flags    | 1       | 26
// 26     | activity_state  | 1       | 27
// 27     | temperature     | 4       | 31
// 31     | quat            | 16      | 47
// 47     | pred_quat       | 16      | 63
// 63     | accel           | 12      | 75
// 75     | gyro            | 12      | 87
// 87     | mag             | 12      | 99
// 99     | velocity        | 12      | 111
// 111    | position        | 12      | 123
// 123    | height          | 4       | 127
// 127    | beta            | 4       | 131
// 131    | stride_length   | 4       | 135
// 135    | speed           | 4       | 139
// 139    | step_frequency  | 4       | 143
// 143    | dir_x           | 4       | 147
// 147    | dir_y           | 4       | 151
// 151    | reserved1       | 4       | 155
// 155    | reserved2       | 4       | 159
// 159    | reserved3       | 4       | 163
// 163    | reserved4       | 4       | 167
// 167    | reserved5       | 4       | 171
// 171    | padding         | 4       | 175
// 175    | crc32           | 4       | 179 ✓

#endif // PROTOCOL_H