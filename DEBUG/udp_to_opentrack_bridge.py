#!/usr/bin/env python3
# udp_to_opentrack_bridge.py - ✅ CORRIGIDO 2025
"""
Bridge UDP: recebe VRTRACKER_V6, parseia e encaminha.
Pode ser usado para debug ou integração com OpenTrack.

Uso:
  python udp_to_opentrack_bridge.py --listen 0.0.0.0:5005 --out 127.0.0.1:4242 --verbose
"""
import argparse
import socket
import struct
import sys
import time
import binascii
import math

TAG = b"VRTRACKER_V6"
TAG_LEN = 12  # ✅ CORRIGIDO
HEADER_MAGIC = 0xAA

def parse_v6_packet(pkt, verbose=False):
    """
    Parse completo do pacote VRTRACKER_V6.
    
    Retorna dict com todos os campos ou levanta ValueError.
    """
    min_len = 1 + TAG_LEN + 8 + 4 + 2 + 4 + 29*4 + 4  # 29 floats = 25 + 4 extras
    if len(pkt) < min_len:
        raise ValueError(f"packet too short ({len(pkt)} < {min_len})")

    # Magic
    magic = pkt[0]
    if magic != HEADER_MAGIC:
        raise ValueError(f"bad magic 0x{magic:02x}")

    # Tag
    tag = pkt[1:1+TAG_LEN]
    if tag != TAG:
        raise ValueError(f"bad tag {tag!r}")

    # CRC32
    if len(pkt) < 4:
        raise ValueError("packet too short for CRC")
    crc_expected = struct.unpack_from("<I", pkt, len(pkt)-4)[0]
    payload = pkt[:len(pkt)-4]
    crc_calc = binascii.crc32(payload) & 0xFFFFFFFF
    if crc_calc != crc_expected:
        raise ValueError(f"crc mismatch calc=0x{crc_calc:08x} expected=0x{crc_expected:08x}")

    # ✅ CORRIGIDO: Offset correto
    offset = 1 + TAG_LEN
    ts_us = struct.unpack_from("<Q", pkt, offset)[0]; offset += 8
    seq = struct.unpack_from("<I", pkt, offset)[0]; offset += 4
    zupt = pkt[offset]
    activity = pkt[offset+1]
    offset += 2
    temp = struct.unpack_from("<f", pkt, offset)[0]; offset += 4

    # 29 floats
    floats = []
    for _ in range(29):
        f = struct.unpack_from("<f", pkt, offset)[0]
        floats.append(f)
        offset += 4

    # Extrair campos
    parsed = {
        "ts_us": ts_us,
        "seq": seq,
        "zupt": zupt,
        "activity": activity,
        "temp": temp,
    }
    
    idx = 0
    parsed["quat"]      = floats[idx:idx+4]; idx += 4
    parsed["pred_quat"] = floats[idx:idx+4]; idx += 4
    parsed["accel"]     = floats[idx:idx+3]; idx += 3
    parsed["gyro"]      = floats[idx:idx+3]; idx += 3
    parsed["mag"]       = floats[idx:idx+3]; idx += 3
    parsed["vel"]       = floats[idx:idx+3]; idx += 3
    parsed["pos"]       = floats[idx:idx+3]; idx += 3
    parsed["height"]    = floats[idx]; idx += 1
    parsed["beta"]      = floats[idx]; idx += 1
    parsed["speed"]     = floats[idx]; idx += 1
    parsed["stepfreq"]  = floats[idx]; idx += 1
    parsed["dirx"]      = floats[idx]; idx += 1
    parsed["diry"]      = floats[idx]; idx += 1

    parsed["crc_calc"] = crc_calc
    parsed["crc_expected"] = crc_expected
    parsed["raw_len"] = len(pkt)

    if verbose:
        print(f"[PARSE] len={len(pkt)} ts={ts_us} seq={seq} zupt={zupt} act={activity} temp={temp:.2f}°C")
        q = parsed["quat"]
        print(f"        quat=[{q[0]:.5f}, {q[1]:.5f}, {q[2]:.5f}, {q[3]:.5f}]")
        print(f"        speed={parsed['speed']:.4f} m/s stepfreq={parsed['stepfreq']:.3f} Hz")
        print(f"        pos=[{parsed['pos'][0]:.3f}, {parsed['pos'][1]:.3f}, {parsed['pos'][2]:.3f}]")
        
    return parsed

def quat_to_euler(q):
    """Converte quaternion [w,x,y,z] para Euler [yaw, pitch, roll] em graus."""
    q0, q1, q2, q3 = q
    
    # Yaw (Z-axis)
    siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Pitch (Y-axis)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Roll (X-axis)
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))

def run(listen_addr, out_addr, verbose=False):
    """Loop principal do bridge."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(listen_addr)
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"[BRIDGE] Escutando em {listen_addr[0]}:{listen_addr[1]}")
    print(f"[BRIDGE] Encaminhando para {out_addr[0]}:{out_addr[1]}")
    if verbose:
        print("[BRIDGE] Modo verbose ATIVO\n")
    
    packet_count = 0
    error_count = 0
    
    while True:
        try:
            data, addr = sock.recvfrom(65536)
            packet_count += 1
            
            if verbose:
                print(f"\n[RECV #{packet_count}] {len(data)} bytes de {addr[0]}:{addr[1]}")
            
            try:
                parsed = parse_v6_packet(data, verbose=verbose)
            except Exception as e:
                error_count += 1
                if verbose:
                    print(f"[ERROR] Parse falhou: {e}")
                    print(f"        Raw preview: {data[:60]!r}")
                # Encaminhar mesmo assim (pode ser útil para debug)
                out_sock.sendto(data, out_addr)
                continue

            # Calcular Euler para debug
            yaw, pitch, roll = quat_to_euler(parsed["quat"])
            if verbose:
                print(f"[EULER] yaw={yaw:.2f}° pitch={pitch:.2f}° roll={roll:.2f}°")

            # Encaminhar pacote original
            try:
                out_sock.sendto(data, out_addr)
                if verbose:
                    print(f"[FORWARD] Pacote encaminhado para {out_addr[0]}:{out_addr[1]}")
            except Exception as e:
                print(f"[ERROR] Forward falhou: {e}")
                
            # Stats a cada 100 pacotes
            if not verbose and packet_count % 100 == 0:
                ok = packet_count - error_count
                taxa = 100 * ok / packet_count
                print(f"[STATS] Pacotes: {packet_count} | Erros: {error_count} | Taxa: {taxa:.1f}%")

        except KeyboardInterrupt:
            print("\n\n[BRIDGE] Interrompido pelo usuário")
            print(f"[STATS FINAL] Total: {packet_count} | Erros: {error_count}")
            break
        except Exception as e:
            print(f"[ERROR] Loop principal: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Bridge UDP para VRTRACKER_V6")
    ap.add_argument("--listen", default="0.0.0.0:5005", help="endereço para escutar (ex: 0.0.0.0:5005)")
    ap.add_argument("--out", default="127.0.0.1:4242", help="endereço para encaminhar (ex: 127.0.0.1:4242)")
    ap.add_argument("-v", "--verbose", action="store_true", help="log detalhado")
    args = ap.parse_args()

    def parse_addr(s):
        if ":" not in s:
            return (s, 5005)
        host, port = s.rsplit(":", 1)
        return (host, int(port))

    listen = parse_addr(args.listen)
    out = parse_addr(args.out)
    
    try:
        run(listen, out, verbose=args.verbose)
    except KeyboardInterrupt:
        print("\n[EXIT] Finalizado")
        sys.exit(0)
