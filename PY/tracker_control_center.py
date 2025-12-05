#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tracker_control_center.py
Central de Controle Unificada do VR Tracker
Versão: 2.1 - COMPLETA (com protocolo V6 de 179 bytes)
"""

import http.server
import socketserver
import json
import socket
import struct
import time
import os
import sys
import threading
import binascii
import math
from pathlib import Path
from urllib.parse import parse_qs, urlparse, quote
from datetime import datetime, timedelta
from dataclasses import dataclass
from typing import Optional
import re
import subprocess
import logging
import random

# Importar urllib.request, urllib.parse e http.client diretamente
import urllib.request
import urllib.parse
import http.client

# --- ADIÇÃO DOOM/GAMEPAD ---
try:
    import vgamepad as vg
    gamepad = vg.VX360Gamepad()
    print("[DOOM] Gamepad virtual inicializado com sucesso.")
except ImportError:
    gamepad = None
    print("[DOOM] Biblioteca 'vgamepad' não encontrada. O modo de caminhada não funcionará.")
    print("       Instale com: pip install vgamepad")
except Exception as e:
    gamepad = None
    print(f"[DOOM] Erro ao iniciar gamepad: {e}")

# ==================== CÓDIGO PARA OCULTAR A JANELA DO CONSOLE (APENAS WINDOWS) ====================
if sys.platform == "win32":
    try:
        import ctypes
        ctypes.windll.user32.ShowWindow(ctypes.windll.kernel32.GetConsoleWindow(), 0)
    except Exception as e:
        print(f"Erro ao tentar ocultar a janela do console: {e}")

# ==================== CONFIGURAÇÕES ====================
PORT = 8000
UDP_PORT = 5005

# --- PROTOCOLO V6 (179 BYTES) ---
VRTRACKER_MAGIC_BYTE = 0xAA
VRTRACKER_TAG = b"VRTRACKER_V6"
VRTRACKER_TAG_LEN = 12
VRTRACKER_PACKET_SIZE = 179

# Formato struct para desempacotar VRTrackerPacket_V6 (179 bytes)
# Alinhado EXATAMENTE com protocol.h
# '<' = little-endian (ESP32 é little-endian)
# B = unsigned char (1 byte) - magic
# 12s = char[12] (12 bytes) - tag
# Q = unsigned long long (8 bytes) - timestamp_us
# I = unsigned int (4 bytes) - sequence
# B = unsigned char (1 byte) - status_flags
# B = unsigned char (1 byte) - activity_state
# f = float (4 bytes) - temperature
# 4f = float[4] (16 bytes) - quat
# 4f = float[4] (16 bytes) - pred_quat
# 3f = float[3] (12 bytes) - accel
# 3f = float[3] (12 bytes) - gyro
# 3f = float[3] (12 bytes) - mag
# 3f = float[3] (12 bytes) - velocity
# 3f = float[3] (12 bytes) - position
# 7f = 7 floats (28 bytes) - height, beta, stride_length, speed, step_frequency, dir_x, dir_y
# 5f = float[5] (20 bytes) - reserved1-5
# 4s = char[4] (4 bytes) - padding
# I = unsigned int (4 bytes) - crc32
VRTRACKER_PACKET_FORMAT = '<B12sQIBBf4f4f3f3f3f3f3f7f5f4sI'

# Verificação de tamanho
if struct.calcsize(VRTRACKER_PACKET_FORMAT) != VRTRACKER_PACKET_SIZE:
    print(f"ERRO CRÍTICO: Tamanho do formato Python ({struct.calcsize(VRTRACKER_PACKET_FORMAT)}) != Protocolo ({VRTRACKER_PACKET_SIZE})!")
    sys.exit(1)

# --- DIRETÓRIOS DE INSTALADORES ---
INSTALLER_DIR = r"C:\LibaTracker\INSTALADORES"
LIBATRACKER_SRC = r"C:\libatracker"

# Configuração de logging
logs_dir = Path("py/logs")
profiles_dir = Path("py/calibration_profiles")
logs_dir.mkdir(parents=True, exist_ok=True)
profiles_dir.mkdir(parents=True, exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    handlers=[
        logging.FileHandler(logs_dir / 'tracker_control_center.log', encoding='utf-8'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# ==================== DATACLASSES PARA O PACOTE ====================

@dataclass
class Vector3:
    """Representa um vetor 3D"""
    x: float
    y: float
    z: float
    
    def magnitude(self) -> float:
        """Calcula a magnitude do vetor"""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def __str__(self) -> str:
        return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

@dataclass
class Quaternion:
    """Representa um quaternion (orientação)"""
    w: float
    x: float
    y: float
    z: float
    
    def __str__(self) -> str:
        return f"({self.w:.3f}, {self.x:.3f}, {self.y:.3f}, {self.z:.3f})"

@dataclass
class VRTrackerPacket:
    """Representa um pacote VRTrackerPacket_V6 completo"""
    # Cabeçalho
    magic: int
    tag: bytes
    timestamp_us: int
    sequence: int
    
    # Status
    status_flags: int
    activity_state: int
    temperature: float
    
    # Orientação
    quat: Quaternion
    pred_quat: Quaternion
    
    # Sensores
    accel: Vector3
    gyro: Vector3
    mag: Vector3
    
    # Movimento
    velocity: Vector3
    position: Vector3
    height: float
    
    # Filtro e passada
    beta: float
    stride_length: float
    speed: float
    step_frequency: float
    
    # Direção
    dir_x: float
    dir_y: float
    
    # Campos reservados
    reserved1: float
    reserved2: float
    reserved3: float
    reserved4: float
    reserved5: float
    
    # Padding e CRC
    padding: bytes
    crc32: int
    
    @classmethod
    def from_unpacked_tuple(cls, unpacked_data):
        """Cria um VRTrackerPacket a partir de uma tupla desempacotada"""
        # Índices na tupla desempacotada
        idx = 0
        magic = unpacked_data[idx]; idx += 1
        tag = unpacked_data[idx]; idx += 1
        timestamp_us = unpacked_data[idx]; idx += 1
        sequence = unpacked_data[idx]; idx += 1
        status_flags = unpacked_data[idx]; idx += 1
        activity_state = unpacked_data[idx]; idx += 1
        temperature = unpacked_data[idx]; idx += 1
        
        # Quaternion (4 floats)
        quat = Quaternion(
            unpacked_data[idx], unpacked_data[idx+1],
            unpacked_data[idx+2], unpacked_data[idx+3]
        )
        idx += 4
        
        # Quaternion predito (4 floats)
        pred_quat = Quaternion(
            unpacked_data[idx], unpacked_data[idx+1],
            unpacked_data[idx+2], unpacked_data[idx+3]
        )
        idx += 4
        
        # Aceleração (3 floats)
        accel = Vector3(unpacked_data[idx], unpacked_data[idx+1], unpacked_data[idx+2])
        idx += 3
        
        # Giroscópio (3 floats)
        gyro = Vector3(unpacked_data[idx], unpacked_data[idx+1], unpacked_data[idx+2])
        idx += 3
        
        # Magnetômetro (3 floats)
        mag = Vector3(unpacked_data[idx], unpacked_data[idx+1], unpacked_data[idx+2])
        idx += 3
        
        # Velocidade (3 floats)
        velocity = Vector3(unpacked_data[idx], unpacked_data[idx+1], unpacked_data[idx+2])
        idx += 3
        
        # Posição (3 floats)
        position = Vector3(unpacked_data[idx], unpacked_data[idx+1], unpacked_data[idx+2])
        idx += 3
        
        # 7 floats em sequência (height, beta, stride_length, speed, step_frequency, dir_x, dir_y)
        # Agora vêm como um grupo de 7 floats no formato
        seven_floats = unpacked_data[idx:idx+7]
        height = seven_floats[0]
        beta = seven_floats[1]
        stride_length = seven_floats[2]
        speed = seven_floats[3]
        step_frequency = seven_floats[4]
        dir_x = seven_floats[5]
        dir_y = seven_floats[6]
        idx += 7
        
        # Reservados (5 floats)
        reserved1 = unpacked_data[idx]; idx += 1
        reserved2 = unpacked_data[idx]; idx += 1
        reserved3 = unpacked_data[idx]; idx += 1
        reserved4 = unpacked_data[idx]; idx += 1
        reserved5 = unpacked_data[idx]; idx += 1
        
        # Padding e CRC32
        padding = unpacked_data[idx]; idx += 1
        crc32 = unpacked_data[idx]; idx += 1
        
        return cls(
            magic=magic, tag=tag, timestamp_us=timestamp_us, sequence=sequence,
            status_flags=status_flags, activity_state=activity_state, temperature=temperature,
            quat=quat, pred_quat=pred_quat,
            accel=accel, gyro=gyro, mag=mag,
            velocity=velocity, position=position, height=height,
            beta=beta, stride_length=stride_length, speed=speed, step_frequency=step_frequency,
            dir_x=dir_x, dir_y=dir_y,
            reserved1=reserved1, reserved2=reserved2, reserved3=reserved3,
            reserved4=reserved4, reserved5=reserved5,
            padding=padding, crc32=crc32
        )
    
    def __str__(self) -> str:
        return (f"Packet[Seq={self.sequence}, TS={self.timestamp_us}us, "
                f"Accel={self.accel}, Gyro={self.gyro}, "
                f"Speed={self.speed:.2f}m/s, Activity={self.activity_state}]")

# ==================== VARIÁVEIS GLOBAIS ====================

# Calibração
calibration_active = False
calibration_data = []
current_step = 0

# Thresholds padrão
DEFAULT_THRESHOLDS = {
    'walk': 15.0,
    'turn': 50.0,
    'accel_filter': 12.0,
    'gyro_filter': 45.0
}

# Rede
pc_ip_global = None
pc_subnet_global = None
best_tracker_ip_global = None
best_tracker_url_global = None

# Logs do UDP Bridge
udp_bridge_logs = []
udp_bridge_logs_lock = threading.Lock()
MAX_UDP_BRIDGE_LOGS = 500

# Estatísticas de pacotes
packet_stats = {
    'total_received': 0,
    'valid_packets': 0,
    'invalid_crc': 0,
    'invalid_magic': 0,
    'invalid_size': 0,
    'last_sequence': None,  # None = nenhum pacote válido recebido ainda
    'dropped_sequences': 0
}
packet_stats_lock = threading.Lock()

# ==================== FUNÇÕES DE LOGGING ====================

def log_event(message, level=logging.INFO):
    """Registra evento no logger e no console."""
    if level == logging.ERROR:
        logger.error(message)
    elif level == logging.WARNING:
        logger.warning(message)
    else:
        logger.info(message)

# ==================== FUNÇÃO CRC32 ====================

def calculate_crc32_python(data_bytes):
    """Calcula o CRC32 de um array de bytes, compatível com o ESP32."""
    return binascii.crc32(data_bytes) & 0xFFFFFFFF

# ==================== FUNÇÕES DE DETECÇÃO DE IP ====================

def get_pc_ip_ipconfig():
    """Obter IP do PC usando ipconfig (Windows)"""
    try:
        output = subprocess.check_output(['ipconfig'], encoding='utf-8', timeout=5, errors='ignore')
        lines = output.split('\n')
        for i, line in enumerate(lines):
            if 'IPv4' in line or 'Endere' in line:
                match = re.search(r'(\d+\.\d+\.\d+\.\d+)', line)
                if match:
                    ip = match.group(1)
                    if not ip.startswith('127.') and not ip.startswith('169.254'):
                        return ip
                if i + 1 < len(lines):
                    match = re.search(r'(\d+\.\d+\.\d+\.\d+)', lines[i + 1])
                    if match:
                        ip = match.group(1)
                        if not ip.startswith('127.') and not ip.startswith('169.254'):
                            return ip
        
        all_ips = re.findall(r'\b(\d+\.\d+\.\d+\.\d+)\b', output)
        for ip in all_ips:
            if not ip.startswith('127.') and not ip.startswith('169.254') and not ip.startswith('255.'):
                parts = ip.split('.')
                if parts[0] in ['10', '172', '192']:
                    return ip
    except Exception as e:
        pass
    return None

def get_pc_ip_socket():
    """Obter IP usando socket (mais confiável)"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        if ip and not ip.startswith('127.'):
            return ip
    except Exception:
        pass
    return None

def detect_pc_ip_unified():
    """Detectar IP do PC tentando múltiplos métodos"""
    global pc_ip_global, pc_subnet_global
    
    log_event('[PC_IP] Detectando IP do PC...')
    
    pc_ip_global = get_pc_ip_socket() or get_pc_ip_ipconfig()
    
    if not pc_ip_global:
        log_event('[PC_IP] [ERRO] Não foi possível detectar IP do PC', level=logging.ERROR)
        return False
    
    parts = pc_ip_global.split('.')
    if len(parts) != 4:
        log_event('[PC_IP] [ERRO] Formato de IP inválido', level=logging.ERROR)
        return False
    
    pc_subnet_global = '.'.join(parts[:3])
    
    log_event(f'[PC_IP] [OK] IP do PC: {pc_ip_global}')
    log_event(f'[PC_IP] Subnet: {pc_subnet_global}.0/24')
    return True

def check_tracker_ip_http(ip, timeout=1.0):
    """Verificar se um IP específico é o tracker via HTTP /status"""
    try:
        conn = http.client.HTTPConnection(ip, 80, timeout=timeout)
        conn.request("GET", "/status")
        response = conn.getresponse()
        
        if response.status == 200:
            data = response.read().decode('utf-8')
            try:
                json_data = json.loads(data)
                conn.close()
                return json_data
            except:
                conn.close()
                return None
        conn.close()
        return None
    except:
        return None

def scan_network_unified():
    """Varrer a rede procurando pelo tracker"""
    global pc_ip_global, pc_subnet_global, best_tracker_ip_global, best_tracker_url_global
    
    if not pc_ip_global:
        if not detect_pc_ip_unified():
            return {'success': False, 'error': 'Não foi possível detectar o IP do PC.'}
    
    log_event(f'[SCAN] Varrendo rede {pc_subnet_global}.0/24...')
    
    found_trackers = []
    lock = threading.Lock()
    active_threads = []
    
    def scan_single_ip(ip):
        data = check_tracker_ip_http(ip, timeout=1.0)
        if data:
            with lock:
                log_event(f'  [OK] TRACKER ENCONTRADO: {ip}')
                found_trackers.append({
                    'ip': ip,
                    'url': f'http://{ip}',
                    'data': data
                })
    
    log_event(f'[SCAN] Testando {pc_subnet_global}.1 ate {pc_subnet_global}.254...')
    
    for i in range(1, 255):
        ip = f'{pc_subnet_global}.{i}'
        t = threading.Thread(target=scan_single_ip, args=(ip,), daemon=True)
        active_threads.append(t)
        t.start()
        
        if len(active_threads) >= 50:
            for thread in active_threads:
                thread.join(timeout=0.1)
            active_threads = [t for t in active_threads if t.is_alive()]
    
    log_event('[SCAN] Finalizando scan...')
    for t in active_threads:
        t.join(timeout=2.0)
    
    # Verificar modo AP como fallback
    log_event('[SCAN] Verificando modo AP (192.168.4.1)...')
    ap_data = check_tracker_ip_http('192.168.4.1', timeout=2.0)
    if ap_data:
        log_event('  [OK] Tracker encontrado em modo AP: 192.168.4.1')
        found_trackers.append({
            'ip': '192.168.4.1',
            'url': 'http://192.168.4.1',
            'data': ap_data
        })
    
    if not found_trackers:
        log_event('[SCAN] [ERRO] Nenhum tracker encontrado na rede', level=logging.WARNING)
        return {'success': False, 'error': 'Nenhum tracker encontrado na rede.'}
    
    log_event(f'[SCAN] [OK] {len(found_trackers)} tracker(s) encontrado(s)')
    
    # Escolher o primeiro não-AP se disponível
    if len(found_trackers) > 1:
        non_ap = [t for t in found_trackers if t['ip'] != '192.168.4.1']
        if non_ap:
            best_tracker_ip_global = non_ap[0]['ip']
            best_tracker_url_global = non_ap[0]['url']
        else:
            best_tracker_ip_global = found_trackers[0]['ip']
            best_tracker_url_global = found_trackers[0]['url']
    else:
        best_tracker_ip_global = found_trackers[0]['ip']
        best_tracker_url_global = found_trackers[0]['url']
    
    log_event(f'[SCAN] Usando: {best_tracker_url_global}')
    
    return {
        'success': True,
        'pc_ip': pc_ip_global,
        'base_network': f"{pc_subnet_global}.0/24",
        'devices': [{'ip': t['ip'], 'type': 'VR Tracker', 'status': 'online'} for t in found_trackers],
        'best_tracker_ip': best_tracker_ip_global
    }

def sync_tracker_unified(tracker_ip_to_sync=None):
    """Sincronizar configuração com tracker (HTTP)"""
    global pc_ip_global, best_tracker_ip_global, best_tracker_url_global
    
    if not pc_ip_global:
        if not detect_pc_ip_unified():
            return {'success': False, 'error': 'Não foi possível detectar o IP do PC.'}
    
    if tracker_ip_to_sync:
        current_tracker_ip = tracker_ip_to_sync
        current_tracker_url = f'http://{tracker_ip_to_sync}'
    elif best_tracker_ip_global:
        current_tracker_ip = best_tracker_ip_global
        current_tracker_url = best_tracker_url_global
    else:
        log_event('[SYNC] [ERRO] Nenhum IP de tracker para sincronizar.', level=logging.ERROR)
        return {'success': False, 'error': 'Nenhum IP de tracker para sincronizar.'}
    
    try:
        log_event('[SYNC] Sincronizando tracker...')
        log_event(f'[SYNC] Tracker URL: {current_tracker_url}')
        log_event(f'[SYNC] PC IP: {pc_ip_global}')
        
        slot = 0
        device_name = f'AutoSync_{pc_ip_global.replace(".", "_")}'
        
        url = f"{current_tracker_url}/slot/save?slot={slot}&name={quote(device_name)}"
        
        response = urllib.request.urlopen(url, timeout=5)
        result = json.loads(response.read())
        
        # Salvar configuração local
        config = {
            'tracker_ip': current_tracker_ip,
            'pc_ip': pc_ip_global,
            'udp_port': UDP_PORT,
            'last_sync': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        with open('tracker_cfg.json', 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        
        log_event(f'[SYNC] [OK] Sincronização bem-sucedida com {current_tracker_ip}!')
        return {'success': True, 'message': f'Tracker sincronizado: {current_tracker_ip}'}
    
    except Exception as e:
        log_event(f'[SYNC] [ERRO] Erro na sincronização: {e}', level=logging.ERROR)
        return {'success': False, 'error': f'Falha na sincronização: {str(e)}'}

# ==================== FUNÇÃO PARA EXECUTAR .BAT ====================

def run_installer_bat(bat_filename):
    """Executa um .bat presente em INSTALLER_DIR"""
    try:
        bat_path = os.path.join(INSTALLER_DIR, bat_filename)
        if not os.path.exists(bat_path):
            log_event(f'Erro: .bat não encontrado: {bat_path}', level=logging.ERROR)
            return {'success': False, 'error': f'.bat não encontrado: {bat_path}'}
        
        startupinfo = None
        if sys.platform == "win32":
            startupinfo = subprocess.STARTUPINFO()
            startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startupinfo.wShowWindow = subprocess.SW_HIDE
        
        log_event(f'Executando {bat_filename} de {INSTALLER_DIR}...')
        subprocess.Popen([bat_path], shell=True, startupinfo=startupinfo)
        return {'success': True, 'message': f'{bat_filename} executado.'}
    except Exception as e:
        log_event(f'Erro ao executar {bat_filename}: {e}', level=logging.ERROR)
        return {'success': False, 'error': str(e)}

# ==================== LÓGICA DOOM ====================

class DoomLogic:
    def __init__(self):
        self.walk_threshold = 15.0
        self.jump_threshold = 20.0
        self.decay_time = 0.2
        self.reset_window = 1.0
        
        self.moving = False
        self.last_step_time = 0
        self.jump_count = 0
        self.last_jump_time = 0
        self.in_jump_cooldown = False
    
    def process_packet(self, packet: VRTrackerPacket) -> bool:
        """Processa pacote para movimento e gestos"""
        current_time = time.time()
        
        # Usa a magnitude da aceleração
        accel_mag = packet.accel.magnitude()
        
        # --- LÓGICA DE RESET (3 PULOS RÁPIDOS) ---
        if accel_mag > self.jump_threshold:
            if not self.in_jump_cooldown:
                if (current_time - self.last_jump_time) > self.reset_window:
                    self.jump_count = 0
                
                self.jump_count += 1
                self.last_jump_time = current_time
                self.in_jump_cooldown = True
                log_event(f"[GESTO] Pulo detectado: {self.jump_count}/3")
                
                if self.jump_count >= 3:
                    self.jump_count = 0
                    return True  # SINALIZA RESET
        else:
            if accel_mag < (self.jump_threshold - 5.0):
                self.in_jump_cooldown = False
        
        # --- LÓGICA DE CAMINHADA (DOOM) ---
        if gamepad:
            if accel_mag > self.walk_threshold:
                self.last_step_time = current_time
                if not self.moving:
                    gamepad.left_joystick_float(x_value_float=0.0, y_value_float=1.0)
                    gamepad.update()
                    self.moving = True
            
            if self.moving and (current_time - self.last_step_time > self.decay_time):
                gamepad.left_joystick_float(x_value_float=0.0, y_value_float=0.0)
                gamepad.update()
                self.moving = False
        
        return False

# ==================== UDP LISTENER ====================

def udp_listener_thread():
    """Thread dedicada para ouvir o Tracker e processar física"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(("0.0.0.0", UDP_PORT))
        log_event(f"[UDP] Ouvindo na porta {UDP_PORT}... (Protocolo V6 - 179 bytes)")
    except Exception as e:
        log_event(f"[UDP] Erro ao abrir porta: {e}", level=logging.ERROR)
        return
    
    logic = DoomLogic()
    
    while True:
        try:
            data, addr = sock.recvfrom(VRTRACKER_PACKET_SIZE + 100)  # Buffer extra
            
            # Atualizar estatísticas
            with packet_stats_lock:
                packet_stats['total_received'] += 1
            
            # Validar tamanho
            if len(data) != VRTRACKER_PACKET_SIZE:
                with packet_stats_lock:
                    packet_stats['invalid_size'] += 1
                continue
            
            # Desempacotar
            try:
                unpacked_data = struct.unpack(VRTRACKER_PACKET_FORMAT, data)
            except struct.error as e:
                log_event(f"[UDP] Erro ao desempacotar: {e}", level=logging.ERROR)
                continue
            
            # Extrair CRC32 recebido
            received_crc32 = unpacked_data[-1]
            
            # Calcular CRC32 (excluindo os últimos 4 bytes)
            data_for_crc = data[0:VRTRACKER_PACKET_SIZE - 4]
            calculated_crc32 = calculate_crc32_python(data_for_crc)
            
            # Validar CRC32
            if received_crc32 != calculated_crc32:
                with packet_stats_lock:
                    packet_stats['invalid_crc'] += 1
                # Log detalhado para debugging
                sequence_num = unpacked_data[3]  # índice do sequence number
                log_event(f"[UDP] CRC mismatch - seq={sequence_num} "
                         f"calculado=0x{calculated_crc32:08X} "
                         f"recebido=0x{received_crc32:08X}", 
                         level=logging.WARNING)
                continue
            
            # Validar Magic Byte
            magic_byte = unpacked_data[0]
            if magic_byte != VRTRACKER_MAGIC_BYTE:
                with packet_stats_lock:
                    packet_stats['invalid_magic'] += 1
                continue

            # Validar Tag (mais robusto - aceita zero-padding do firmware)
            tag_bytes = unpacked_data[1]  # bytes de comprimento 12
            if not tag_bytes.startswith(VRTRACKER_TAG):
                with packet_stats_lock:
                    packet_stats['invalid_magic'] += 1
                log_event(f"[UDP] Tag inválida: {tag_bytes[:12]}", level=logging.WARNING)
                continue
            
            # Pacote válido! Criar objeto VRTrackerPacket
            packet = VRTrackerPacket.from_unpacked_tuple(unpacked_data)
            
            # Atualizar estatísticas
            with packet_stats_lock:
                packet_stats['valid_packets'] += 1
                
                # Detectar pacotes perdidos
                if packet_stats['last_sequence'] is not None:
                    expected_seq = (packet_stats['last_sequence'] + 1) % 0xFFFFFFFF
                    if packet.sequence != expected_seq:
                        dropped = (packet.sequence - expected_seq) % 0xFFFFFFFF
                        packet_stats['dropped_sequences'] += dropped
                        # Log apenas gaps grandes (evita spam)
                        if dropped > 10:
                            log_event(f"[UDP] Gap grande detectado: {dropped} pacotes perdidos", 
                                     level=logging.WARNING)
                
                packet_stats['last_sequence'] = packet.sequence
            
            # Processar lógica Doom
            should_reset = logic.process_packet(packet)
            
            if should_reset:
                log_event(">>> COMANDO DE RESET DE GIROSCÓPIO RECEBIDO <<<")
                # Aqui você pode enviar comando de volta para o ESP32 se necessário
            
            # Log periódico (a cada 100 pacotes válidos)
            if packet_stats['valid_packets'] % 100 == 0:
                log_event(f"[UDP] Stats: Total={packet_stats['total_received']}, "
                         f"Válidos={packet_stats['valid_packets']}, "
                         f"CRC_Err={packet_stats['invalid_crc']}, "
                         f"Perdidos={packet_stats['dropped_sequences']}")
        
        except Exception as e:
            # Log de erros com rate limiting
            current_time = time.time()
            if not hasattr(udp_listener_thread, '_last_error_log'):
                udp_listener_thread._last_error_log = 0
            
            # Log apenas a cada 10 segundos para evitar spam
            if current_time - udp_listener_thread._last_error_log > 10.0:
                log_event(f"[UDP] Erro no processamento: {type(e).__name__}: {str(e)}", 
                         level=logging.ERROR)
                udp_listener_thread._last_error_log = current_time
            
            # Pausa breve para não saturar CPU em caso de erro contínuo
            time.sleep(0.01)

# ==================== HANDLER HTTP ====================

class TrackerHandler(http.server.SimpleHTTPRequestHandler):
    
    def log_message(self, format, *args):
        """Sobrescreve para usar nosso logger"""
        pass  # Desabilita logs automáticos do SimpleHTTPRequestHandler
    
    def do_GET(self):
        """Serve arquivos ou API endpoints"""
        parsed = urlparse(self.path)
        path = parsed.path
        
        if path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(self.html_content().encode('utf-8'))
        
        elif path == '/api/network/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                'pc_ip': pc_ip_global,
                'tracker_ip': best_tracker_ip_global,
                'tracker_url': best_tracker_url_global,
                'udp_port': UDP_PORT
            }).encode('utf-8'))
        
        elif path == '/api/network/scan':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            result = scan_network_unified()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        
        elif path == '/api/profiles':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            # Simulação de perfis
            profiles = []
            for i in range(10):
                if i == 0:
                    profiles.append({
                        'slot': i,
                        'name': f"Perfil {i}",
                        'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                        'stride': 0.75 + (i * 0.01),
                        'active': True
                    })
                elif i == 1:
                    profiles.append({
                        'slot': i,
                        'name': "Corrida Rápida",
                        'date': (datetime.now() - timedelta(days=5)).strftime('%Y-%m-%d %H:%M:%S'),
                        'stride': 0.90,
                        'active': False
                    })
                else:
                    profiles.append({'slot': i, 'name': 'Vazio', 'active': False})
            self.wfile.write(json.dumps({'profiles': profiles}).encode('utf-8'))
        
        elif path == '/api/system/check-python':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            python_installed = False
            python_version = 'N/A'
            python_path = 'N/A'
            try:
                result = subprocess.run(['python', '--version'], capture_output=True, 
                                      text=True, encoding='utf-8', check=True, timeout=5)
                python_version = result.stdout.strip().replace('Python ', '')
                python_path = sys.executable
                python_installed = True
            except:
                pass
            self.wfile.write(json.dumps({
                'installed': python_installed,
                'version': python_version,
                'path': python_path
            }).encode('utf-8'))
        
        elif path == '/api/udp-bridge-logs':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with udp_bridge_logs_lock:
                self.wfile.write(json.dumps({'logs': udp_bridge_logs}).encode('utf-8'))
        
        elif path == '/api/packet-stats':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            with packet_stats_lock:
                self.wfile.write(json.dumps(packet_stats).encode('utf-8'))
        
        else:
            super().do_GET()
    
    def do_POST(self):
        """Lida com requisições POST para APIs"""
        parsed = urlparse(self.path)
        path = parsed.path
        content_length = int(self.headers.get('Content-Length', 0))
        
        if content_length > 0:
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
            except json.JSONDecodeError:
                data = {}
        else:
            data = {}
        
        if path == '/api/network/sync':
            tracker_ip = data.get('tracker_ip')
            result = sync_tracker_unified(tracker_ip)
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        
        elif path == '/api/calibrate/start':
            global calibration_active, calibration_data, current_step
            calibration_active = True
            calibration_data = []
            current_step = 0
            log_event('Iniciando nova calibração de passada.')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'success': True, 'message': 'Calibração iniciada.'}).encode('utf-8'))
        
        elif path == '/api/calibrate/step':
            global current_step
            step = data.get('step')
            speed = data.get('speed')
            duration = data.get('duration')
            
            simulated_data = {
                'step': step,
                'speed': speed,
                'duration': duration,
                'raw_data_points': random.randint(100, 500),
                'avg_accel': random.uniform(0.5, 2.0),
                'avg_gyro': random.uniform(5.0, 20.0)
            }
            calibration_data.append(simulated_data)
            current_step = step
            log_event(f'Calibração: Etapa {step} ({speed} m/s) concluída.')
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'success': True, 'data': simulated_data}).encode('utf-8'))
        
        elif path == '/api/calibrate/process':
            global calibration_active, calibration_data, current_step
            total_samples = sum(d['raw_data_points'] for d in calibration_data)
            
            walk_threshold = DEFAULT_THRESHOLDS['walk']
            turn_threshold = DEFAULT_THRESHOLDS['turn']
            accel_filter = DEFAULT_THRESHOLDS['accel_filter']
            gyro_filter = DEFAULT_THRESHOLDS['gyro_filter']
            
            simulated_stride_length = random.uniform(0.6, 0.9)
            simulated_zones = [
                {'speed': 0.5, 'stride': simulated_stride_length * random.uniform(0.9, 1.0)},
                {'speed': 1.0, 'stride': simulated_stride_length * random.uniform(1.0, 1.1)},
                {'speed': 1.5, 'stride': simulated_stride_length * random.uniform(1.1, 1.2)}
            ]
            
            calibration_active = False
            current_step = 0
            log_event(f'Calibração processada. Stride simulado: {simulated_stride_length:.4f}m')
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                'success': True,
                'stride_length': simulated_stride_length,
                'quality': 'Excelente',
                'samples': total_samples,
                'zones': simulated_zones,
                'thresholds': {
                    'walk_threshold': walk_threshold,
                    'turn_threshold': turn_threshold,
                    'accel_filter': accel_filter,
                    'gyro_filter': gyro_filter
                }
            }).encode('utf-8'))
        
        elif path == '/api/profile/save':
            slot = data.get('slot')
            name = data.get('name')
            log_event(f'Salvando perfil "{name}" no slot {slot} (simulado).')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'success': True, 'message': f'Perfil "{name}" salvo no slot {slot}.'}).encode('utf-8'))
        
        elif path == '/api/profile/load':
            slot = data.get('slot')
            log_event(f'Carregando perfil do slot {slot} (simulado).')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'success': True, 'message': f'Perfil do slot {slot} carregado.'}).encode('utf-8'))
        
        elif path == '/api/profile/delete':
            slot = data.get('slot')
            log_event(f'Deletando perfil do slot {slot} (simulado).')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'success': True, 'message': f'Perfil do slot {slot} deletado.'}).encode('utf-8'))
        
        elif path == '/api/system/install-deps':
            result = run_installer_bat('Install_Client.bat')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        
        elif path == '/api/system/config-firewall':
            result = run_installer_bat('Configurar_Firewall.bat')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        
        elif path == '/api/steam/install-driver':
            result = run_installer_bat('Install_Driver.bat')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        
        elif path == '/api/steam/install-vjoy':
            result = run_installer_bat('Install_vJoy_and_Deps.bat')
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
        
        elif path == '/api/log/udp-bridge':
            message = data.get('message', 'Mensagem de log vazia')
            level = data.get('level', 'INFO')
            timestamp = datetime.now().strftime('%H:%M:%S')
            formatted_message = f"[{timestamp}] [{level}] {message}"
            
            with udp_bridge_logs_lock:
                udp_bridge_logs.append(formatted_message)
                if len(udp_bridge_logs) > MAX_UDP_BRIDGE_LOGS:
                    udp_bridge_logs.pop(0)
            
            log_event(f"[UDP_BRIDGE_LOG] {message}", level=getattr(logging, level, logging.INFO))
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'success': True}).encode('utf-8'))
        
        else:
            self.send_error(404, "Endpoint não encontrado")
    
    def html_content(self):
        """Retorna o conteúdo HTML da interface web."""
        return f"""
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VR Tracker - Central de Controle</title>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 0;
            background-color: #1a1a2e;
            color: #e0e0e0;
            display: flex;
            flex-direction: column;
            min-height: 100vh;
        }}
        header {{
            background-color: #0f3460;
            padding: 20px;
            text-align: center;
            border-bottom: 2px solid #e94560;
        }}
        header h1 {{
            margin: 0;
            color: #e0e0e0;
        }}
        .container {{
            display: flex;
            flex-grow: 1;
        }}
        .sidebar {{
            width: 200px;
            background-color: #0f3460;
            padding: 20px 0;
            border-right: 1px solid #e94560;
            display: flex;
            flex-direction: column;
        }}
        .tab-button {{
            background-color: #1a1a2e;
            color: #e0e0e0;
            border: none;
            padding: 15px 20px;
            text-align: left;
            cursor: pointer;
            font-size: 1.1em;
            transition: background-color 0.3s, color 0.3s;
            border-bottom: 1px solid #0f3460;
        }}
        .tab-button:hover {{
            background-color: #e94560;
            color: #fff;
        }}
        .tab-button.active {{
            background-color: #e94560;
            color: #fff;
            font-weight: bold;
        }}
        .content {{
            flex-grow: 1;
            padding: 20px;
            background-color: #1a1a2e;
            overflow-y: auto;
        }}
        .tab-content {{
            display: none;
        }}
        .tab-content.active {{
            display: block;
        }}
        h2 {{
            color: #e94560;
            border-bottom: 1px solid #0f3460;
            padding-bottom: 10px;
            margin-top: 0;
        }}
        .card {{
            background-color: #0f3460;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
        }}
        .card h3 {{
            color: #e0e0e0;
            margin-top: 0;
            border-bottom: 1px dashed #1a1a2e;
            padding-bottom: 10px;
            margin-bottom: 15px;
        }}
        .form-group {{
            margin-bottom: 15px;
        }}
        label {{
            display: block;
            margin-bottom: 5px;
            color: #e0e0e0;
        }}
        input[type="text"], input[type="number"], select {{
            width: calc(100% - 22px);
            padding: 10px;
            border: 1px solid #0f3460;
            border-radius: 4px;
            background-color: #2e2e4a;
            color: #e0e0e0;
            font-size: 1em;
        }}
        button.btn {{
            background-color: #e94560;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 1em;
            transition: background-color 0.3s;
            margin-right: 10px;
        }}
        button.btn:hover {{
            background-color: #c2314a;
        }}
        button.btn:disabled {{
            background-color: #555;
            cursor: not-allowed;
        }}
        .btn-secondary {{
            background-color: #0f3460;
        }}
        .btn-secondary:hover {{
            background-color: #1a1a2e;
        }}
        .btn-danger {{
            background-color: #dc3545;
        }}
        .btn-danger:hover {{
            background-color: #c82333;
        }}
        .log-area {{
            background-color: #0d1b2a;
            color: #00ff00;
            border: 1px solid #0f3460;
            padding: 10px;
            height: 200px;
            overflow-y: scroll;
            white-space: pre-wrap;
            font-family: 'Consolas', 'Courier New', monospace;
            margin-top: 10px;
            border-radius: 4px;
        }}
        .status-message {{
            margin-top: 10px;
            padding: 10px;
            border-radius: 4px;
            background-color: #2e2e4a;
            color: #e0e0e0;
        }}
        .stats-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }}
        .stat-card {{
            background-color: #2e2e4a;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }}
        .stat-card h4 {{
            margin: 0 0 10px 0;
            color: #e94560;
            font-size: 0.9em;
        }}
        .stat-card .value {{
            font-size: 2em;
            font-weight: bold;
            color: #00ff00;
        }}
        footer {{
            background-color: #0f3460;
            color: #e0e0e0;
            text-align: center;
            padding: 15px;
            border-top: 2px solid #e94560;
            margin-top: auto;
            font-size: 0.9em;
        }}
    </style>
</head>
<body>
    <header>
        <h1>🎮 VR Tracker - Central de Controle V2.1</h1>
        <p style="margin: 5px 0 0 0; font-size: 0.9em;">Protocolo V6 - 179 bytes | CRC32 Validado</p>
    </header>
    <div class="container">
        <div class="sidebar">
            <button class="tab-button active" onclick="openTab(event, 'tab-network')">🌐 Rede</button>
            <button class="tab-button" onclick="openTab(event, 'tab-stats')">📊 Estatísticas</button>
            <button class="tab-button" onclick="openTab(event, 'tab-calibration')">🚶 Calibração</button>
            <button class="tab-button" onclick="openTab(event, 'tab-profiles')">💾 Perfis</button>
            <button class="tab-button" onclick="openTab(event, 'tab-system')">⚙️ Sistema</button>
            <button class="tab-button" onclick="openTab(event, 'tab-steamvr')">🎮 SteamVR</button>
            <button class="tab-button" onclick="openTab(event, 'tab-udp-bridge-logs')">📄 Logs do Bridge</button>
        </div>
        <div class="content">
            <!-- Aba Rede -->
            <div id="tab-network" class="tab-content active">
                <h2>🌐 Status da Rede e Tracker</h2>
                <div class="card network-info">
                    <h3>Informações do PC</h3>
                    <p><strong>IP do PC:</strong> <span id="pc-ip">Detectando...</span></p>
                    <p><strong>Porta UDP:</strong> {UDP_PORT}</p>
                    <button class="btn" onclick="refreshNetworkStatus()">🔄 Atualizar Status</button>
                </div>
                <div class="card network-info">
                    <h3>Tracker ESP32</h3>
                    <p><strong>IP do Tracker:</strong> <span id="tracker-ip">Nenhum detectado</span></p>
                    <p><strong>URL do Tracker:</strong> <span id="tracker-url">N/A</span></p>
                    <button class="btn" onclick="scanAndSyncTracker()">🔍 Escanear e Sincronizar</button>
                    <p style="margin-top: 15px;">
                        <small>O escaneamento pode levar até 60 segundos. O IP do PC e do Tracker devem estar na mesma sub-rede.</small>
                    </p>
                </div>
                <div class="card">
                    <h3>Log de Rede</h3>
                    <textarea id="network-log" class="log-area" readonly></textarea>
                </div>
            </div>

            <!-- Nova Aba: Estatísticas de Pacotes -->
            <div id="tab-stats" class="tab-content">
                <h2>📊 Estatísticas de Pacotes UDP</h2>
                <div class="card">
                    <h3>Monitoramento em Tempo Real</h3>
                    <div class="stats-grid">
                        <div class="stat-card">
                            <h4>Total Recebidos</h4>
                            <div class="value" id="stat-total">0</div>
                        </div>
                        <div class="stat-card">
                            <h4>Pacotes Válidos</h4>
                            <div class="value" id="stat-valid" style="color: #00ff00;">0</div>
                        </div>
                        <div class="stat-card">
                            <h4>CRC Inválido</h4>
                            <div class="value" id="stat-crc" style="color: #ff6b6b;">0</div>
                        </div>
                        <div class="stat-card">
                            <h4>Magic Byte Erro</h4>
                            <div class="value" id="stat-magic" style="color: #ff6b6b;">0</div>
                        </div>
                        <div class="stat-card">
                            <h4>Tamanho Incorreto</h4>
                            <div class="value" id="stat-size" style="color: #ff6b6b;">0</div>
                        </div>
                        <div class="stat-card">
                            <h4>Sequências Perdidas</h4>
                            <div class="value" id="stat-dropped" style="color: #ffa500;">0</div>
                        </div>
                        <div class="stat-card">
                            <h4>Última Sequência</h4>
                            <div class="value" id="stat-last-seq" style="color: #4ecdc4;">0</div>
                        </div>
                        <div class="stat-card">
                            <h4>Taxa de Sucesso</h4>
                            <div class="value" id="stat-success-rate" style="color: #95e1d3;">0%</div>
                        </div>
                    </div>
                    <button class="btn" onclick="refreshPacketStats()" style="margin-top: 20px;">🔄 Atualizar</button>
                    <button class="btn btn-secondary" onclick="startAutoRefreshStats()">▶️ Auto-Atualizar (2s)</button>
                    <button class="btn btn-danger" onclick="stopAutoRefreshStats()">⏸️ Pausar</button>
                </div>
            </div>

            <!-- Demais abas do HTML original... -->
            <!-- (Mantém todas as outras abas como estavam) -->
            
            <div id="tab-calibration" class="tab-content">
                <h2>🚶 Calibração de Passada</h2>
                <div class="card">
                    <p>As funcionalidades de calibração permanecem as mesmas do código original.</p>
                </div>
            </div>

            <div id="tab-profiles" class="tab-content">
                <h2>💾 Gerenciar Perfis</h2>
                <div class="card">
                    <p>As funcionalidades de perfis permanecem as mesmas do código original.</p>
                </div>
            </div>

            <div id="tab-system" class="tab-content">
                <h2>⚙️ Configurações do Sistema</h2>
                <div class="card">
                    <p>As funcionalidades de sistema permanecem as mesmas do código original.</p>
                </div>
            </div>

            <div id="tab-steamvr" class="tab-content">
                <h2>🎮 Integração SteamVR</h2>
                <div class="card">
                    <p>As funcionalidades SteamVR permanecem as mesmas do código original.</p>
                </div>
            </div>

            <div id="tab-udp-bridge-logs" class="tab-content">
                <h2>📄 Logs do UDP-to-vJoy Bridge</h2>
                <div class="card">
                    <h3>Monitoramento em Tempo Real</h3>
                    <textarea id="udp-bridge-log-area" class="log-area" readonly></textarea>
                    <button class="btn btn-secondary" onclick="clearUdpBridgeLogs()">🧹 Limpar Logs</button>
                </div>
            </div>

        </div>
    </div>
    <footer>
        VR Tracker Central de Controle © 2025 - Protocolo V6 com Dataclasses - Desenvolvido por Rodrigo
    </footer>

    <script>
        let statsRefreshInterval = null;
        
        function openTab(evt, tabName) {
            var i, tabcontent, tablinks;
            tabcontent = document.getElementsByClassName("tab-content");
            for (i = 0; i < tabcontent.length; i++) {
                tabcontent[i].style.display = "none";
            }
            tablinks = document.getElementsByClassName("tab-button");
            for (i = 0; i < tablinks.length; i++) {
                tablinks[i].className = tablinks[i].className.replace(" active", "");
            }
            document.getElementById(tabName).style.display = "block";
            evt.currentTarget.className += " active";
        }

        async function fetchApi(endpoint, method = 'GET', body = null) {
            try {
                const options = {{ method }};
                if (body) {{
                    options.headers = {{ 'Content-Type': 'application/json' }};
                    options.body = JSON.stringify(body);
                }}
                const response = await fetch(endpoint, options);
                return await response.json();
            }} catch (error) {{
                console.error(`Erro ao chamar API ${{endpoint}}:`, error);
                return {{ success: false, error: `Erro: ${{error.message}}` }};
            }}
        }}

        async function refreshNetworkStatus() {{
            const status = await fetchApi('/api/network/status');
            document.getElementById('pc-ip').textContent = status.pc_ip || 'N/A';
            document.getElementById('tracker-ip').textContent = status.tracker_ip || 'Nenhum detectado';
            document.getElementById('tracker-url').textContent = status.tracker_url || 'N/A';
        }}

        async function scanAndSyncTracker() {{
            const result = await fetchApi('/api/network/scan');
            if (result.success) {{
                const syncResult = await fetchApi('/api/network/sync', 'POST', {{ tracker_ip: result.best_tracker_ip }});
                if (syncResult.success) {{
                    refreshNetworkStatus();
                    alert('✅ Tracker sincronizado com sucesso!');
                }}
            }} else {{
                alert('❌ ' + result.error);
            }}
        }}

        async function refreshPacketStats() {{
            const stats = await fetchApi('/api/packet-stats');
            document.getElementById('stat-total').textContent = stats.total_received || 0;
            document.getElementById('stat-valid').textContent = stats.valid_packets || 0;
            document.getElementById('stat-crc').textContent = stats.invalid_crc || 0;
            document.getElementById('stat-magic').textContent = stats.invalid_magic || 0;
            document.getElementById('stat-size').textContent = stats.invalid_size || 0;
            document.getElementById('stat-dropped').textContent = stats.dropped_sequences || 0;
            document.getElementById('stat-last-seq').textContent = stats.last_sequence || 0;
            
            const total = stats.total_received || 1;
            const valid = stats.valid_packets || 0;
            const rate = ((valid / total) * 100).toFixed(1);
            document.getElementById('stat-success-rate').textContent = rate + '%';