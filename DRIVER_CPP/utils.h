// ==================== utils.h ====================
#ifndef UTILS_H
#define UTILS_H

#include <cstdint>
#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Calcula CRC32 (IEEE 802.3 polynomial: 0xEDB88320)
 * Compatível com zlib.crc32() do Python
 * 
 * @param data Ponteiro para os dados
 * @param len Tamanho dos dados em bytes
 * @return CRC32 calculado (32 bits)
 */
uint32_t calculate_crc32(const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // UTILS_H
