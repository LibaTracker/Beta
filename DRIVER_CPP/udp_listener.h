// udp_listener.h - Versão Corrigida e Alinhada
#pragma once
#include <stdint.h>
#include <functional>
#include <string>

#include "protocol.h"

#ifdef _WIN32
  #ifdef UDP_LISTENER_EXPORTS
    #define UDP_LISTENER_API __declspec(dllexport)
  #else
    #define UDP_LISTENER_API __declspec(dllimport)
  #endif
#else
  #define UDP_LISTENER_API
#endif

extern "C" {

// Callback type: recebe ponteiro para pacote validado + info de origem
// IMPORTANTE: src_ip pode ser nullptr se não disponível (modo reorder)
typedef void (*udp_payload_cb_t)(const VRTrackerPacket_V6* packet, 
                                  const char* src_ip, 
                                  uint16_t src_port);

// Start UDP listener (retorna true em sucesso)
// bind_ip: IP para bind (use "0.0.0.0" ou nullptr para INADDR_ANY)
// port: porta UDP (geralmente 5005)
UDP_LISTENER_API bool start_udp_listener(const char* bind_ip, uint16_t port);

// Stop listener (limpa recursos, fecha socket)
UDP_LISTENER_API void stop_udp_listener();

// Define callback para receber pacotes validados
// Callback é chamado da thread de reorder, use mutex se necessário
UDP_LISTENER_API void set_udp_payload_callback(udp_payload_cb_t cb);

// Configura jitter buffer target em milissegundos (padrão: 50ms)
// ms: tempo máximo de buffering antes de entregar pacotes fora de ordem
UDP_LISTENER_API void set_udp_jitter_ms(int ms);

// Estatísticas do listener (opcional)
typedef struct {
    uint64_t packets_received;    // Total recebido
    uint64_t packets_dropped;     // CRC fail ou invalid
    uint64_t packets_delivered;   // Entregues ao callback
    uint32_t buffer_size;         // Pacotes no buffer de reorder
    uint32_t last_sequence;       // Último seq entregue
} UDPListenerStats;

UDP_LISTENER_API void get_udp_listener_stats(UDPListenerStats* stats);
UDP_LISTENER_API void reset_udp_listener_stats();

}  // extern "C"