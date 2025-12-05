#!/usr/bin/env python3
# collect_mag_udp.py - ✅ CORRIGIDO 2025
# Recebe pacotes UDP do tracker v6, extrai campos mag (mx,my,mz) e grava mag_samples.csv

import socket
import struct
import argparse
import csv
import sys
import time

# ✅ CORRIGIDO: TAG_LEN era 16, agora 12
TAG_LEN = 12  # "VRTRACKER_V6" = 12 caracteres ASCII

def parse_v6_packet(pkt):
    """
    Parse VRTRACKER_V6 packet e extrai magnetômetro.
    
    Estrutura do pacote:
    - 1 byte: 0xAA (magic)
    - 12 bytes: "VRTRACKER_V6" (TAG)
    - 8 bytes: timestamp (uint64_t)
    - 4 bytes: sequence (uint32_t)
    - 1 byte: zupt
    - 1 byte: activity
    - 4 bytes: temperature (float)
    - 100 bytes: 25 floats (quat[4], pred_quat[4], accel[3], gyro[3], mag[3], vel[3], pos[3], height, beta)
    - 16 bytes: 4 floats extras (speed, stepfreq, dirx, diry)
    - 4 bytes: CRC32
    """
    try:
        # ✅ CORRIGIDO: offset era 1 + 16, agora 1 + 12
        offset = 1 + TAG_LEN

        # Extrair campos do header
        ts_us = struct.unpack_from("<Q", pkt, offset)[0]; offset += 8
        seq   = struct.unpack_from("<I", pkt, offset)[0]; offset += 4
        zupt  = pkt[offset]
        activity = pkt[offset + 1]
        offset += 2
        temp  = struct.unpack_from("<f", pkt, offset)[0]; offset += 4

        # Agora temos 25 floats principais + 4 extras = 29 floats total
        floats = []
        for _ in range(29):
            f = struct.unpack_from("<f", pkt, offset)[0]
            floats.append(f)
            offset += 4
        
        # Ordem dos floats:
        # [0:4]   = quat (q0, q1, q2, q3)
        # [4:8]   = pred_quat
        # [8:11]  = accel (ax, ay, az)
        # [11:14] = gyro (gx, gy, gz)
        # [14:17] = mag (mx, my, mz) ← o que queremos!
        # [17:20] = vel
        # [20:23] = pos
        # [23]    = height
        # [24]    = beta
        # [25]    = speed
        # [26]    = stepfreq
        # [27]    = dirx
        # [28]    = diry
        
        idx = 14  # mag começa no índice 14
        mag = floats[idx:idx+3]
        
        return {"ts_us": ts_us, "seq": seq, "mag": mag}
    except Exception as e:
        print(f"[PARSE ERROR] {e}")
        return None

def main():
    p = argparse.ArgumentParser(description="Coleta amostras de magnetômetro via UDP")
    p.add_argument("--host", default="0.0.0.0", help="host to bind (default 0.0.0.0)")
    p.add_argument("--port", type=int, default=5005, help="UDP port (default 5005)")
    p.add_argument("--out", default="mag_samples.csv", help="output CSV file")
    p.add_argument("--samples", type=int, default=500, help="num samples to collect")
    args = p.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((args.host, args.port))
    print(f"[MAG COLLECT] Escutando UDP em {args.host}:{args.port}")
    print(f"[MAG COLLECT] Meta: {args.samples} amostras")
    print("\n⚠️  INSTRUÇÕES:")
    print("  1. Segure o tracker (fora do corpo)")
    print("  2. GIRE LENTAMENTE em todos os eixos")
    print("  3. Faça movimentos de figura-8 no ar")
    print("  4. Tente cobrir todas as orientações\n")
    
    rows = []
    s.settimeout(2.0)
    start_time = time.time()
    
    try:
        while len(rows) < args.samples:
            try:
                data, addr = s.recvfrom(65536)
            except socket.timeout:
                if len(rows) == 0 and (time.time() - start_time) > 10:
                    print("\n[TIMEOUT] Nenhum pacote recebido em 10s.")
                    print("Verifique:")
                    print("  - Tracker está ligado?")
                    print("  - PC e tracker na mesma rede?")
                    print("  - Firewall bloqueando porta 5005?")
                    sys.exit(1)
                continue
                
            parsed = parse_v6_packet(data)
            if not parsed: 
                continue
                
            mx, my, mz = parsed["mag"]
            rows.append((mx, my, mz))
            
            # Progress bar simples
            progress = int(50 * len(rows) / args.samples)
            bar = '█' * progress + '░' * (50 - progress)
            print(f"\r[{bar}] {len(rows)}/{args.samples} ({100*len(rows)//args.samples}%)", end="", flush=True)
            
    except KeyboardInterrupt:
        print("\n[INTERROMPIDO] Salvando amostras coletadas...")
    finally:
        s.close()

    if not rows:
        print("\n[ERRO] Nenhuma amostra coletada!")
        return 1
        
    # Salvar CSV
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        for r in rows:
            w.writerow(r)
    
    print(f"\n✅ [SALVO] {len(rows)} amostras → {args.out}")
    print("\n📋 Próximo passo:")
    print(f"   python mag_calib.py --in {args.out} --out mag_calib.json")
    return 0

if __name__ == "__main__":
    sys.exit(main())
