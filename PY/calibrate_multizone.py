#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# calibrate_multizone.py - ✅ CORRIGIDO 2025
"""
Calibrador multizona para VR Tracker com k-means 1D.

Funcionalidade:
 - Escuta pacotes UDP e extrai step frequency
 - Agrupa leituras em amostras por janela de tempo
 - Executa K-means 1D para criar k zonas de velocidade
 - Salva resultado em JSON pronto para o tracker

Uso:
  python calibrate_multizone.py --k 4 --samples 8 --sample-window 3 --max-speed 1.6
"""
from __future__ import annotations
import argparse
import socket
import sys
import time
import json
import logging
import re
import struct
import binascii
from typing import List, Optional, Tuple
from math import isfinite
from pathlib import Path
import random

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s'
)
logger = logging.getLogger("calibrate_multizone")

# ✅ CORRIGIDO: Constantes do pacote
MAGIC = 0xAA
TAG = b"VRTRACKER_V6"
TAG_LEN = 12  # Era 16!

def extract_stepfreq_from_packet(pkt: bytes) -> Optional[float]:
    """
    Extrai stepfreq do pacote binário VRTRACKER_V6.
    
    Estrutura esperada:
    - 1 byte: 0xAA
    - 12 bytes: "VRTRACKER_V6"
    - 8 bytes: ts (uint64)
    - 4 bytes: seq (uint32)
    - 1 byte: zupt
    - 1 byte: activity
    - 4 bytes: temp (float)
    - 25*4 = 100 bytes: floats principais
    - 4*4 = 16 bytes: extras (speed, stepfreq, dirx, diry)
    - 4 bytes: CRC32
    """
    try:
        min_len = 1 + TAG_LEN + 8 + 4 + 2 + 4 + 29*4 + 4
        if len(pkt) < min_len:
            return None
            
        if pkt[0] != MAGIC:
            return None
            
        if pkt[1:1+TAG_LEN] != TAG:
            return None
        
        # Verificar CRC
        crc_expected = struct.unpack_from("<I", pkt, len(pkt)-4)[0]
        crc_calc = binascii.crc32(pkt[:len(pkt)-4]) & 0xFFFFFFFF
        if crc_calc != crc_expected:
            return None
        
        # ✅ CORRIGIDO: Offset correto
        offset = 1 + TAG_LEN
        offset += 8   # ts
        offset += 4   # seq
        offset += 2   # zupt + activity
        offset += 4   # temp
        offset += 25 * 4  # 25 floats principais
        
        # Agora nos extras: [speed, stepfreq, dirx, diry]
        # stepfreq é o segundo float extra (offset + 4)
        if offset + 8 <= len(pkt):
            stepfreq = struct.unpack_from("<f", pkt, offset + 4)[0]
            if isfinite(stepfreq) and 0.1 < stepfreq < 10.0:  # Filtro de sanidade
                return float(stepfreq)
        
        return None
    except Exception as e:
        logger.debug(f"Erro ao extrair stepfreq: {e}")
        return None

# ---------- KMeans 1D ----------
def kmeans_1d(values: List[float], k: int = 4, max_iters: int = 100, seed: Optional[int] = None) -> Tuple[List[float], List[int]]:
    """
    K-means simples para 1D.
    Retorna (centroids, labels).
    """
    if seed is not None:
        random.seed(seed)

    if not values:
        raise ValueError("values vazio")

    vals = [float(v) for v in values]
    n = len(vals)
    if k <= 0:
        raise ValueError("k deve ser >= 1")
    if k > n:
        k = n

    # Inicializar centroides como k valores aleatórios distintos
    centroids = sorted(random.sample(vals, k))
    labels = [0] * n

    for it in range(max_iters):
        changed = False
        
        # Atribuição
        for i, v in enumerate(vals):
            best_j = min(range(k), key=lambda j: abs(v - centroids[j]))
            if labels[i] != best_j:
                labels[i] = best_j
                changed = True
        
        # Atualizar centroides
        new_centroids = []
        for j in range(k):
            members = [vals[i] for i in range(n) if labels[i] == j]
            if members:
                new_centroids.append(sum(members) / len(members))
            else:
                new_centroids.append(random.choice(vals))
        
        # Ordenar (para 1D é conveniente)
        ordered = sorted((c, idx) for idx, c in enumerate(new_centroids))
        order_map = {old_idx: new_idx for new_idx, (_, old_idx) in enumerate(ordered)}
        centroids = [c for c, _ in ordered]
        labels = [order_map[l] for l in labels]

        if not changed:
            break

    return centroids, labels

# ---------- Agregação de amostras ----------
def aggregate_samples_from_values(timestamps: List[float], values: List[float], window_s: float) -> List[float]:
    """
    Agrupa leituras em janelas de tempo e retorna média por janela.
    """
    if not timestamps or not values or len(timestamps) != len(values):
        return []
        
    items = sorted(zip(timestamps, values), key=lambda x: x[0])
    start = items[0][0]
    current_window = start
    window_vals = []
    aggregated = []
    
    for ts, val in items:
        if ts <= current_window + window_s:
            window_vals.append(val)
        else:
            if window_vals:
                aggregated.append(sum(window_vals) / len(window_vals))
            while ts > current_window + window_s:
                current_window += window_s
            window_vals = [val]
            
    if window_vals:
        aggregated.append(sum(window_vals) / len(window_vals))
        
    return aggregated

# ---------- Construir zonas ----------
def build_stride_zones(centroids: List[float], labels: List[int], values: List[float],
                       min_multiplier: float, max_speed: float) -> List[dict]:
    """
    Constrói zonas de stride a partir dos clusters.
    stride_i = (multiplier_i * max_speed) / centroid_i
    """
    if not centroids:
        return []

    k = len(centroids)
    
    # Para cada cluster, calcular min/max/count
    clusters = []
    for j in range(k):
        members = [v for idx, v in enumerate(values) if labels[idx] == j]
        if members:
            clusters.append({
                'index': j,
                'min': min(members),
                'max': max(members),
                'centroid': centroids[j],
                'count': len(members)
            })
        else:
            clusters.append({
                'index': j,
                'min': centroids[j],
                'max': centroids[j],
                'centroid': centroids[j],
                'count': 0
            })

    # Ordenar por centroid
    clusters = sorted(clusters, key=lambda c: c['centroid'])

    # Gerar multiplicadores entre min_multiplier e 1.0
    if k == 1:
        multipliers = [1.0]
    else:
        multipliers = [min_multiplier + (1.0 - min_multiplier) * (i / (k - 1)) for i in range(k)]

    # Calcular strides
    zones = []
    for i, c in enumerate(clusters):
        centroid = float(c['centroid']) if c['centroid'] else 0.0
        multiplier = multipliers[i]
        desired_speed = multiplier * max_speed
        
        if centroid > 0:
            stride = desired_speed / centroid
        else:
            stride = max_speed / 1.0  # Fallback
            
        zones.append({
            'min_hz': float(c['min']),
            'max_hz': float(c['max']),
            'stride': float(stride),
            'centroid_freq': float(centroid),
            'multiplier': float(multiplier),
            'count': int(c.get('count', 0))
        })
        
    return zones

# ---------- Salvar JSON ----------
def save_output_json(out_path: str, zones: List[dict], meta: dict):
    """Salvar zonas em formato compatível com o tracker."""
    payload = {
        'generated_at': time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        'meta': meta,
        'stride_zones': zones
    }
    try:
        with open(out_path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        logger.info(f'✅ [SAVE] Gravado: {out_path}')
    except Exception as e:
        logger.error(f'❌ [SAVE] Falha: {e}')

# ---------- Listener UDP ----------
def run_listener(listen_host: str, listen_port: int,
                 sample_window: float, target_samples: int,
                 k: int, max_speed: float, out_path: str,
                 min_multiplier: float, timeout_s: float):
    """
    Escuta UDP e coleta amostras agregadas.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((listen_host, listen_port))
    sock.settimeout(1.0)
    
    logger.info(f'🎧 [LISTEN] UDP {listen_host}:{listen_port}')
    logger.info(f'⚙️  [CONFIG] window={sample_window}s samples={target_samples} k={k} max_speed={max_speed} m/s')
    logger.info('\n📋 INSTRUÇÕES:')
    logger.info('  1. Coloque o tracker na cintura')
    logger.info('  2. Caminhe na esteira com RITMOS VARIADOS')
    logger.info('  3. Alterne: lento → médio → rápido → lento...')
    logger.info('  4. Mantenha cada ritmo por ~5 segundos')
    logger.info(f'  5. Continue até coletar {target_samples} amostras\n')

    timestamps = []
    raw_values = []
    start_time = time.time()
    last_report = start_time
    last_packet_time = start_time

    try:
        while True:
            # Timeout global
            if timeout_s and (time.time() - start_time) > timeout_s and len(timestamps) == 0:
                logger.error('❌ [TIMEOUT] Nenhuma leitura recebida')
                break

            try:
                data, addr = sock.recvfrom(4096)
                last_packet_time = time.time()
                
                sf = extract_stepfreq_from_packet(data)
                if sf is not None:
                    timestamps.append(time.time())
                    raw_values.append(float(sf))
                    logger.debug(f'[PKT] {addr} → {sf:.3f} Hz')
                    
            except socket.timeout:
                pass
            except Exception as e:
                logger.warning(f'[PKT] Erro: {e}')

            # Report a cada 1s
            if time.time() - last_report > 1.0:
                aggregated = aggregate_samples_from_values(timestamps, raw_values, sample_window)
                elapsed = int(time.time() - start_time)
                logger.info(f'⏱️  [{elapsed}s] Leituras: {len(raw_values)} | Amostras: {len(aggregated)}/{target_samples}')
                last_report = time.time()

            # Verificar se completou
            aggregated = aggregate_samples_from_values(timestamps, raw_values, sample_window)
            if len(aggregated) >= target_samples:
                logger.info('✅ [DONE] Coleta completa!')
                break

            # Inatividade
            if raw_values and (time.time() - last_packet_time) > 8.0 and len(aggregated) > 0:
                logger.info('⏸️  [DONE] Inatividade detectada')
                break

    finally:
        sock.close()

    if not raw_values:
        logger.error('❌ [EXIT] Nenhum valor coletado')
        return False

    aggregated = aggregate_samples_from_values(timestamps, raw_values, sample_window)
    if not aggregated:
        logger.error('❌ [EXIT] Falha ao agregar amostras')
        return False

    logger.info(f'\n📊 [AGG] Amostras agregadas: {len(aggregated)}')
    logger.info(f'     Min: {min(aggregated):.3f} Hz | Max: {max(aggregated):.3f} Hz | Média: {sum(aggregated)/len(aggregated):.3f} Hz')

    # K-means
    try:
        centroids, labels = kmeans_1d(aggregated, k=k, seed=42)
    except Exception as e:
        logger.error(f'❌ [KMEANS] Falha: {e}')
        return False

    logger.info(f'\n🎯 [KMEANS] Centroides: {[f"{c:.3f}" for c in centroids]}')

    # Construir zonas
    zones = build_stride_zones(centroids, labels, aggregated, 
                                min_multiplier=min_multiplier, max_speed=max_speed)
    
    meta = {
        'listen': f'{listen_host}:{listen_port}',
        'sample_window': sample_window,
        'target_samples': target_samples,
        'k': k,
        'max_speed': max_speed,
        'min_multiplier': min_multiplier,
        'samples_collected': len(aggregated)
    }

    save_output_json(out_path, zones, meta)

    # Log resumo
    logger.info('\n' + '='*70)
    logger.info('  ZONAS GERADAS')
    logger.info('='*70)
    for i, z in enumerate(zones):
        logger.info(f"Zona {i}: {z['min_hz']:.2f}-{z['max_hz']:.2f} Hz → stride={z['stride']:.4f}m (mult={z['multiplier']:.2f}, {z['count']} samples)")
    logger.info('='*70 + '\n')

    logger.info('✅ [FIN] Calibração multizona concluída!')
    logger.info(f'\n📋 Próximos passos:')
    logger.info(f'   1. Copie o conteúdo de {out_path}')
    logger.info(f'   2. Cole em tracker_cfg.json na seção "stride_zones"')
    logger.info(f'   3. Faça upload via Web UI')
    logger.info(f'   4. Reinicie o tracker\n')
    
    return True

# ---------- CLI ----------
def parse_args():
    p = argparse.ArgumentParser(description='Calibração multizona (k-means 1D)')
    p.add_argument('--listen', default='0.0.0.0:5005', help='host:port UDP')
    p.add_argument('--sample-window', type=float, default=3.0, help='janela de agregação (s)')
    p.add_argument('--samples', type=int, default=8, help='quantidade de amostras desejadas')
    p.add_argument('--k', type=int, default=4, help='número de zonas (clusters)')
    p.add_argument('--max-speed', type=float, default=1.6, help='velocidade máxima (m/s)')
    p.add_argument('--out', default='tracker_cfg_out.json', help='arquivo de saída')
    p.add_argument('--min-multiplier', type=float, default=0.6, help='multiplicador mínimo (0-1)')
    p.add_argument('--timeout', type=float, default=120.0, help='timeout global (s)')
    return p.parse_args()

def parse_host_port(s: str) -> Tuple[str, int]:
    if ':' in s:
        host, port = s.rsplit(':', 1)
        return host.strip(), int(port.strip())
    else:
        return s, 5005

def main():
    args = parse_args()
    host, port = parse_host_port(args.listen)

    logger.info('='*70)
    logger.info('  CALIBRATE MULTIZONE - VR TRACKER')
    logger.info('='*70 + '\n')

    try:
        ok = run_listener(
            listen_host=host,
            listen_port=port,
            sample_window=args.sample_window,
            target_samples=args.samples,
            k=args.k,
            max_speed=args.max_speed,
            out_path=args.out,
            min_multiplier=args.min_multiplier,
            timeout_s=args.timeout
        )
        sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        logger.info('\n⏸️  Interrompido pelo usuário')
        sys.exit(2)
    except Exception as e:
        logger.exception(f'❌ Erro fatal: {e}')
        sys.exit(3)

if __name__ == '__main__':
    main()
