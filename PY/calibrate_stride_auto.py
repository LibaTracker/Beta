#!/usr/bin/env python3
"""
calibrate_stride_auto.py - Calibração automática (sem interação)
Coleta amostras automaticamente em intervalos de tempo
"""
import socket
import struct
import time
import json
import argparse
from pathlib import Path
import sys

def extract_stepfreq(data):
    """Extrai stepfreq do pacote UDP binário (151 bytes)"""
    if len(data) < 151:
        return None
    try:
        stepfreq = struct.unpack_from('<f', data, 147)[0]
        return stepfreq if stepfreq > 0 else None
    except:
        return None

def collect_samples_auto(sock, duration=5):
    """Coleta amostras por X segundos"""
    samples = []
    start_time = time.time()

    print(f"  📊 Coletando por {duration}s...", flush=True)

    while time.time() - start_time < duration:
        try:
            data, _ = sock.recvfrom(2048)
            freq = extract_stepfreq(data)
            if freq and freq > 0:
                samples.append(freq)
        except socket.timeout:
            continue

    if samples:
        avg = sum(samples) / len(samples)
        print(f"  ✅ {len(samples)} amostras | Média: {avg:.3f} Hz", flush=True)
        return avg
    else:
        print(f"  ⚠️  Nenhuma amostra válida!", flush=True)
        return None

def fit_stride_from_pairs(freqs, speeds):
    """Regressão linear: speed = stride_length * freq"""
    if len(freqs) < 2:
        return None

    n = len(freqs)
    sum_f = sum(freqs)
    sum_s = sum(speeds)
    sum_ff = sum(f*f for f in freqs)
    sum_fs = sum(f*s for f, s in zip(freqs, speeds))

    denom = n * sum_ff - sum_f * sum_f
    if abs(denom) < 1e-9:
        return None

    stride = (n * sum_fs - sum_f * sum_s) / denom
    return max(0.1, stride)

def main():
    parser = argparse.ArgumentParser(description='Calibração automática de passada')
    parser.add_argument('--listen', default='0.0.0.0:5005', help='IP:porta UDP')
    parser.add_argument('--out-config', default='tracker_cfg.json', help='Arquivo de saída')
    parser.add_argument('--duration', type=int, default=8, help='Duração de cada coleta (segundos)')
    parser.add_argument('--speeds', default='0.6,1.0,1.4', help='Velocidades para calibrar (m/s)')
    args = parser.parse_args()

    # Parse listen
    parts = args.listen.split(':')
    host = parts[0]
    port = int(parts[1]) if len(parts) > 1 else 5005

    # Parse speeds
    target_speeds = [float(s.strip()) for s in args.speeds.split(',')]

    print("=" * 70, flush=True)
    print("🎮 CALIBRAÇÃO AUTOMÁTICA DE PASSADA", flush=True)
    print("=" * 70, flush=True)
    print(f"📡 Escutando: {host}:{port}", flush=True)
    print(f"⏱️  Duração por coleta: {args.duration}s", flush=True)
    print(f"🎯 Velocidades alvo: {target_speeds}", flush=True)
    print("", flush=True)
    print("📋 INSTRUÇÕES:", flush=True)
    print("  1. Coloque o tracker na CINTURA", flush=True)
    print("  2. O script coletará automaticamente em intervalos", flush=True)
    print("  3. ALTERNE o ritmo de caminhada:", flush=True)
    for i, speed in enumerate(target_speeds, 1):
        ritmo = 'LENTO' if speed < 0.8 else 'MÉDIO' if speed < 1.2 else 'RÁPIDO'
        print(f"     • Coleta {i}: Ritmo {ritmo} (~{speed} m/s)", flush=True)
    print("", flush=True)
    print("⏳ Aguardando 3 segundos para você se preparar...", flush=True)
    time.sleep(3)

    # Socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.settimeout(0.1)

    freqs = []
    speeds = []

    try:
        for i, target_speed in enumerate(target_speeds, 1):
            ritmo = 'LENTO' if target_speed < 0.8 else 'MÉDIO' if target_speed < 1.2 else 'RÁPIDO'
            print(f"\n🔄 COLETA {i}/{len(target_speeds)} - Alvo: {target_speed} m/s", flush=True)
            print(f"   Caminhe no lugar com ritmo {ritmo}...", flush=True)

            avg_freq = collect_samples_auto(sock, args.duration)

            if avg_freq:
                freqs.append(avg_freq)
                speeds.append(target_speed)
            else:
                print(f"  ⚠️  Coleta {i} falhou! Continue...", flush=True)

            if i < len(target_speeds):
                print(f"  ⏸️  Pausa de 2s antes da próxima coleta...", flush=True)
                time.sleep(2)

    except KeyboardInterrupt:
        print("\n\n⏸️  Interrompido pelo usuário", flush=True)

    sock.close()

    # Processar resultados
    print("\n" + "=" * 70, flush=True)
    print("📊 RESULTADOS DA CALIBRAÇÃO", flush=True)
    print("=" * 70, flush=True)

    if len(freqs) < 2:
        print("❌ [ERRO] Poucas amostras válidas (mínimo 2)", flush=True)
        print("   Certifique-se de que o tracker está enviando dados!", flush=True)
        return 1

    print(f"✅ {len(freqs)} amostras coletadas:", flush=True)
    for f, s in zip(freqs, speeds):
        print(f"   • {f:.3f} Hz → {s:.2f} m/s", flush=True)

    stride = fit_stride_from_pairs(freqs, speeds)

    if stride is None:
        print("❌ [ERRO] Falha no cálculo do stride_length", flush=True)
        return 1

    print(f"\n🎯 STRIDE LENGTH CALCULADO: {stride:.4f} m", flush=True)

    # Salvar configuração
    cfg_path = Path(args.out_config)

    if cfg_path.exists():
        with open(cfg_path, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
    else:
        cfg = {}

    cfg['stride_length'] = round(stride, 4)

    with open(cfg_path, 'w', encoding='utf-8') as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)

    print(f"💾 Configuração salva em: {cfg_path}", flush=True)
    print("\n✅ Calibração concluída com sucesso!", flush=True)
    return 0

if __name__ == '__main__':
    sys.exit(main())
