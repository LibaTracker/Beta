// Substitua por este arquivo: udp_listener.cpp
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include "udp_listener.h"
#include "protocol.h"
#include "utils.h"

#include <thread>
#include <atomic>
#include <cstring>
#include <iostream>

// Link com Winsock
#pragma comment(lib, "ws2_32.lib")

// Globals (internos)
static std::thread g_listener_thread;
static std::atomic<bool> g_running(false);
static udp_payload_cb_t g_callback = nullptr; // usa typedef do header
static SOCKET g_sockfd = INVALID_SOCKET;
static uint16_t g_port = 5005;
static std::string g_bind_ip = "0.0.0.0";

// Thread que recebe pacotes UDP
static void udp_receiver_thread() {
    constexpr size_t BUF_SIZE = 2048;
    uint8_t buffer[BUF_SIZE];

    sockaddr_in clientAddr;
    int clientAddrLen = sizeof(clientAddr);

    std::cout << "[UDP Listener] Thread iniciada, aguardando pacotes na porta "
              << g_port << " (bind=" << g_bind_ip << ")..." << std::endl;

    while (g_running.load()) {
        int bytesReceived = recvfrom(
            g_sockfd,
            reinterpret_cast<char*>(buffer),
            (int)BUF_SIZE,
            0,
            reinterpret_cast<sockaddr*>(&clientAddr),
            &clientAddrLen
        );

        if (bytesReceived == SOCKET_ERROR) {
            int error = WSAGetLastError();
            if (error == WSAETIMEDOUT || error == WSAEINTR) {
                continue;
            }
            std::cerr << "[UDP Listener] recvfrom error: " << error << std::endl;
            break;
        }

        if (bytesReceived < (int)sizeof(VRTrackerPacket_V6)) {
            // pacote menor que o esperado, ignorar
            continue;
        }

        // Se o pacote recebido for maior ou igual ao struct, pegamos os primeiros bytes.
        // Calcular CRC sobre os primeiros (VRTRACKER_PACKET_SIZE - 4) bytes
        if (bytesReceived < (int)VRTRACKER_PACKET_SIZE) {
            // Se firmware/enviador sempre manda 179 bytes, pacotes menores descartados
            continue;
        }

        // Verificar magic (offset 0)
        uint8_t magic = buffer[0];
        if (magic != VRTRACKER_MAGIC_BYTE) {
            // Ignorar
            //std::cerr << "[UDP Listener] Magic inválido: 0x" << std::hex << (int)magic << std::dec << std::endl;
            continue;
        }

        // Calcular CRC sobre bytes [0 .. 174]
        size_t data_for_crc_len = VRTRACKER_PACKET_SIZE - sizeof(uint32_t); // 175
        uint32_t calculated = calculate_crc32(reinterpret_cast<const unsigned char*>(buffer), data_for_crc_len);

        // Extrair CRC recebido (últimos 4 bytes little-endian)
        const uint8_t* crc_ptr = buffer + (VRTRACKER_PACKET_SIZE - 4);
        uint32_t received_crc = 0;
        // little-endian safe copy
        std::memcpy(&received_crc, crc_ptr, sizeof(received_crc));

        if (calculated != received_crc) {
            std::cerr << "[UDP Listener] CRC inválido: calculado=0x" 
                      << std::hex << calculated << " recebido=0x" << received_crc << std::dec << std::endl;
            continue;
        }

        // Tudo ok: copiar para struct (packed)
        VRTrackerPacket_V6 pkt;
        std::memcpy(&pkt, buffer, sizeof(VRTrackerPacket_V6));

        // Chamar callback (se registrado) - passar IP e porta
        if (g_callback) {
            const char* src_ip = inet_ntoa(clientAddr.sin_addr);
            uint16_t src_port = ntohs(clientAddr.sin_port);
            g_callback(&pkt, src_ip, src_port);
        }
    }

    std::cout << "[UDP Listener] Thread finalizada." << std::endl;
}

// Funções exportadas (API) - assinatura idêntica à declarada em udp_listener.h
extern "C" {

UDP_LISTENER_API bool start_udp_listener(const char* bind_ip, uint16_t port) {
    if (g_running.load()) {
        std::cerr << "[UDP Listener] Já em execução." << std::endl;
        return false;
    }

    // Setup Winsock
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        std::cerr << "[UDP Listener] WSAStartup falhou." << std::endl;
        return false;
    }

    // Criar socket UDP
    g_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sockfd == INVALID_SOCKET) {
        std::cerr << "[UDP Listener] socket() falhou: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return false;
    }

    // Bind
    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (bind_ip == nullptr || std::strcmp(bind_ip, "") == 0) {
        addr.sin_addr.s_addr = INADDR_ANY;
        g_bind_ip = "0.0.0.0";
    } else {
        addr.sin_addr.s_addr = inet_addr(bind_ip);
        g_bind_ip = bind_ip;
    }

    if (bind(g_sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "[UDP Listener] bind() falhou: " << WSAGetLastError() << std::endl;
        closesocket(g_sockfd);
        g_sockfd = INVALID_SOCKET;
        WSACleanup();
        return false;
    }

    // Timeout opcional de recv (1s) para permitir checking do g_running
    DWORD timeout = 1000;
    setsockopt(g_sockfd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));

    g_port = port;
    g_running = true;
    g_listener_thread = std::thread(udp_receiver_thread);

    std::cout << "[UDP Listener] Iniciado em " << g_bind_ip << ":" << g_port << std::endl;
    return true;
}

UDP_LISTENER_API void stop_udp_listener() {
    if (!g_running.load()) return;

    std::cout << "[UDP Listener] Parando..." << std::endl;
    g_running = false;

    if (g_listener_thread.joinable()) g_listener_thread.join();

    if (g_sockfd != INVALID_SOCKET) {
        closesocket(g_sockfd);
        g_sockfd = INVALID_SOCKET;
    }

    WSACleanup();
    std::cout << "[UDP Listener] Parado." << std::endl;
}

UDP_LISTENER_API void set_udp_payload_callback(udp_payload_cb_t cb) {
    g_callback = cb;
    if (g_callback) std::cout << "[UDP Listener] Callback registrado." << std::endl;
    else std::cout << "[UDP Listener] Callback removido." << std::endl;
}

// Placeholders - implemente se precisar (só para compilar; podem ser expandidos)
UDP_LISTENER_API void set_udp_jitter_ms(int ms) { (void)ms; /* opcional */ }
UDP_LISTENER_API void get_udp_listener_stats(UDPListenerStats* stats) {
    if (!stats) return;
    std::memset(stats, 0, sizeof(*stats));
}
UDP_LISTENER_API void reset_udp_listener_stats() { /* opcional */ }

} // extern "C"
