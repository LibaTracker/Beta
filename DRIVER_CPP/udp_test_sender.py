#!/usr/bin/env python3
"""
udp_test_sender.py - Envia pacotes VRTrackerPacket_V6 (179 bytes)
Uso:
  python udp_test_sender.py --host 192.168.3.18 --port 5005 --rate 100 --duration 10

Observações:
- Gera CRC32 (zlib.crc32) nos últimos 4 bytes
- CRC calculado sobre todos os bytes EXCETO os últimos 4
- Layout EXATO do protocol.h (179 bytes total)
"""

import socket
import struct
import time
import argparse
import random
import zlib
import math

# Constantes do protocolo
MAGIC = 0xAA
TAG = b"VRTRACKER_V6"
TAG_LEN = 12
PACKET_SIZE = 179  # Tamanho expandido

def build_packet(seq, ts_us,
                 status_flags=1,
                 activity_state=0,
                 temperature=30.0,
                 quat=(1.0, 0.0, 0.0, 0.0),
                 pred_quat=(1.0, 0.0, 0.0, 0.0),
                 accel=(0.0, 0.0, 9.80665),
                 gyro=(0.0, 0.0, 0.0),
                 mag=(30.0, 5.0, -40.0),
                 velocity=(0.0, 0.0, 0.0),
                 position=(0.0, 0.0, 0.0),
                 height=1.7,
                 beta=0.022,
                 stride_length=0.45,
                 speed=0.0,
                 step_frequency=0.0,
                 dir_x=1.0,
                 dir_y=0.0):
    """
    Constrói pacote VRTrackerPacket_V6 de 179 bytes.
    Layout DEVE corresponder exatamente ao protocol.h!
    """
    buf = bytearray()
    
    # ========== HEADER (25 bytes) ==========
    buf += struct.pack("<B", MAGIC)  # magic (1)
    
    # tag (12 bytes fixos)
    tag_bytes = TAG[:TAG_LEN]
    buf += tag_bytes
    if len(tag_bytes) < TAG_LEN:
        buf += b"\x00" * (TAG_LEN - len(tag_bytes))
    
    buf += struct.pack("<Q", ts_us)  # timestamp_us (8)
    buf += struct.pack("<I", seq)    # sequence (4)
    
    # ========== STATUS (2 bytes) ==========
    buf += struct.pack("<B", status_flags)    # status_flags (1)
    buf += struct.pack("<B", activity_state)  # activity_state (1)
    
    # ========== TEMPERATURE (4 bytes) ==========
    buf += struct.pack("<f", float(temperature))
    
    # ========== ORIENTATION (32 bytes) ==========
    buf += struct.pack("<4f", *quat)       # quat[4] (16)
    buf += struct.pack("<4f", *pred_quat)  # pred_quat[4] (16)
    
    # ========== IMU RAW DATA (36 bytes) ==========
    buf += struct.pack("<3f", *accel)  # accel[3] (12)
    buf += struct.pack("<3f", *gyro)   # gyro[3] (12)
    buf += struct.pack("<3f", *mag)    # mag[3] (12)
    
    # ========== MOTION DATA (28 bytes) ==========
    buf += struct.pack("<3f", *velocity)  # velocity[3] (12)
    buf += struct.pack("<3f", *position)  # position[3] (12)
    buf += struct.pack("<f", float(height))  # height (4)
    
    # ========== FILTER PARAMETERS (8 bytes) ==========
    buf += struct.pack("<f", float(beta))           # beta (4)
    buf += struct.pack("<f", float(stride_length))  # stride_length (4)
    
    # ========== STEP DETECTION - NOVOS CAMPOS (16 bytes) ==========
    buf += struct.pack("<f", float(speed))            # speed (4)
    buf += struct.pack("<f", float(step_frequency))   # step_frequency (4)
    buf += struct.pack("<f", float(dir_x))            # dir_x (4)
    buf += struct.pack("<f", float(dir_y))            # dir_y (4)
    
    # ========== RESERVED (20 bytes) ==========
    buf += struct.pack("<5f", 0.0, 0.0, 0.0, 0.0, 0.0)  # reserved1-5
    
    # ========== PADDING (4 bytes) ==========
    buf += b"\x00" * 4
    
    # Verificação de tamanho ANTES do CRC
    expected_len = PACKET_SIZE - 4  # 175 bytes (sem CRC)
    if len(buf) != expected_len:
        raise ValueError(f"Buffer size mismatch: {len(buf)} != {expected_len}")
    
    # ========== CRC32 (4 bytes) ==========
    # zlib.crc32 usa mesmo polinômio do código C++ (IEEE 802.3)
    crc = zlib.crc32(buf) & 0xFFFFFFFF
    buf += struct.pack("<I", crc)
    
    # Verificação final
    if len(buf) != PACKET_SIZE:
        raise ValueError(f"Final packet size wrong: {len(buf)} != {PACKET_SIZE}")
    
    return bytes(buf)


def main():
    parser = argparse.ArgumentParser(description='VRTracker V6 UDP Test Sender')
    parser.add_argument("--host", default="127.0.0.1", help="Target IP")
    parser.add_argument("--port", type=int, default=5005, help="Target port")
    parser.add_argument("--rate", type=float, default=100.0, help="Packets/sec")
    parser.add_argument("--duration", type=float, default=10.0, help="Test duration (sec)")
    parser.add_argument("--jitter", type=float, default=0.0, help="Time jitter (sec)")
    parser.add_argument("--loss", type=float, default=0.0, help="Packet loss probability (0-1)")
    parser.add_argument("--verbose", action='store_true', help="Print every packet")
    args = parser.parse_args()

    addr = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    seq = 0
    interval = 1.0 / args.rate
    t0 = time.time()
    next_send = t0
    
    print(f"Sending to {addr}, rate={args.rate} Hz, duration={args.duration}s")
    print(f"Packet size: {PACKET_SIZE} bytes")
    
    packets_sent = 0
    packets_dropped = 0
    
    try:
        while time.time() - t0 < args.duration:
            now = time.time()
            
            if now >= next_send:
                # Add jitter to timestamp
                jitter = random.uniform(-args.jitter, args.jitter) if args.jitter > 0 else 0.0
                ts_us = int((now + jitter) * 1_000_000)
                
                # Simulate motion (simple sinusoidal)
                angle = (seq % 360) * (math.pi / 180.0) * 0.01
                q = (
                    math.cos(angle),
                    math.sin(angle) * 0.1,
                    0.0,
                    0.0
                )
                
                accel = (
                    0.5 * math.sin(seq * 0.1),
                    0.5 * math.cos(seq * 0.1),
                    9.80665 + 0.3 * math.sin(seq * 0.05)
                )
                
                gyro = (
                    0.01 * math.sin(seq * 0.05),
                    0.01 * math.cos(seq * 0.05),
                    0.005 * math.sin(seq * 0.02)
                )
                
                # Simulate walking
                step_freq = 1.8  # 1.8 Hz = ~108 steps/min (walking)
                stride = 0.45
                speed = step_freq * stride  # ~0.81 m/s
                
                # Forward direction (changes slowly)
                dir_angle = seq * 0.001
                dir_x = math.cos(dir_angle)
                dir_y = math.sin(dir_angle)
                
                velocity = (
                    dir_x * speed,
                    dir_y * speed,
                    0.0
                )
                
                position = (
                    velocity[0] * (now - t0),
                    velocity[1] * (now - t0),
                    0.0
                )
                
                packet = build_packet(
                    seq=seq,
                    ts_us=ts_us,
                    status_flags=0x01,  # ZUPT active
                    activity_state=1,   # Walking
                    temperature=30.0,
                    quat=q,
                    pred_quat=q,
                    accel=accel,
                    gyro=gyro,
                    mag=(30.0, 5.0, -40.0),
                    velocity=velocity,
                    position=position,
                    height=1.75,
                    beta=0.022,
                    stride_length=stride,
                    speed=speed,
                    step_frequency=step_freq,
                    dir_x=dir_x,
                    dir_y=dir_y
                )
                
                # Simulate packet loss
                if random.random() >= args.loss:
                    sock.sendto(packet, addr)
                    packets_sent += 1
                    
                    if args.verbose:
                        print(f"[{packets_sent:5d}] seq={seq:6d} ts={ts_us/1e6:.3f}s "
                              f"speed={speed:.2f}m/s freq={step_freq:.1f}Hz")
                else:
                    packets_dropped += 1
                
                seq = (seq + 1) & 0xFFFFFFFF
                next_send += interval
            else:
                sleep_time = max(0.0, next_send - now)
                if sleep_time > 0:
                    time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    elapsed = time.time() - t0
    print(f"\nDone after {elapsed:.1f}s")
    print(f"Packets sent: {packets_sent}")
    print(f"Packets dropped (simulated): {packets_dropped}")
    print(f"Actual rate: {packets_sent/elapsed:.1f} pps")


if __name__ == "__main__":
    main()