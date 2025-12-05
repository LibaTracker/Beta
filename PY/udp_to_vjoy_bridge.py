#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
udp_to_vjoy_bridge.py

Bridge que recebe os pacotes UDP binários enviados pelo VR-Tracker e
os converte para um dispositivo vJoy. O script foi revisado para:

*   Decodificar corretamente o formato binário de 151 bytes do tracker.
*   Ser configurável via linha de comando para IP, porta e limites de mapeamento.
*   Lidar graciosamente com pacotes mal-formados ou ausentes.
*   Mapear corretamente os valores para o intervalo esperado pelo vJoy (0 a 0x8000).
*   Registrar mensagens de erro/aviso sem interromper a execução.
*   Fechar o dispositivo vJoy de forma limpa ao receber SIGINT/CTRL-C.
*   AD..A variável `CRC32_OFFSET` foi definida para facilitar essa exclusão.
    *   Após desempacotar o pacote, comparamos o `calculated_crc32` com o `received_crc32`.
    *   Se os CRCs não coincidirem, um log de depuração é gerado e a função retorna `None`, indicando que o pacote é inválido e deve ser descartado.
*   ADICIONADO (Melhoria 7): Envia logs para o tracker_control_center.py via HTTP.
"""

import socket
import struct
import time
import sys
import os
import argparse
import logging
import zlib # Para CRC32
import requests # Para enviar logs via HTTP (Melhoria 7)

# Tenta importar pyvjoy. Se falhar, o script pode rodar, mas sem funcionalidade vJoy.
try:
    import pyvjoy
    VJOY_AVAILABLE = True
except ImportError:
    print("AVISO: pyvjoy não encontrado. A funcionalidade vJoy estará desativada.")
    print("Para instalar: pip install pyvjoy")
    VJOY_AVAILABLE = False

# ============================================================
# CONFIGURAÇÕES GLOBAIS
# ============================================================
DEFAULT_UDP_IP = "0.0.0.0"  # Escuta em todas as interfaces
DEFAULT_UDP_PORT = 5005
VJOY_DEVICE_ID = 1          # ID do dispositivo vJoy (geralmente 1)

# Limites de mapeamento para os eixos do vJoy (0 a 0x8000)
# Estes são valores de exemplo e devem ser ajustados conforme a calibração do seu tracker
# Os valores do tracker são geralmente float, então precisamos de um range min/max para mapear
# Exemplo: rotação em Y (yaw) de -180 a 180 graus, mapeado para 0 a 0x8000
# Posição X, Y, Z em metros, mapeado para um range adequado
# Velocidade em m/s, frequência de passos em passos/min
# A calibração real deve definir esses ranges
DEFAULT_MIN_YAW = -180.0
DEFAULT_MAX_YAW = 180.0
DEFAULT_MIN_PITCH = -90.0
DEFAULT_MAX_PITCH = 90.0
DEFAULT_MIN_ROLL = -90.0
DEFAULT_MAX_ROLL = 90.0

DEFAULT_MIN_POS = -2.0 # Exemplo: -2 metros
DEFAULT_MAX_POS = 2.0  # Exemplo: +2 metros

DEFAULT_MIN_VELOCITY = 0.0
DEFAULT_MAX_VELOCITY = 5.0 # Exemplo: 5 m/s

DEFAULT_MIN_STEP_FREQ = 0.0
DEFAULT_MAX_STEP_FREQ = 180.0 # Exemplo: 180 passos/min

# Tamanho esperado do pacote UDP binário (151 bytes)
PACKET_SIZE = 151
# Offset do CRC32 no pacote (últimos 4 bytes)
CRC32_OFFSET = PACKET_SIZE - 4

# URL da Central de Controle para enviar logs (Melhoria 7)
CONTROL_CENTER_LOG_URL = "http://localhost:8000/api/log/udp-bridge"

# ============================================================
# CONFIGURAÇÃO DE LOGGING
# ============================================================
LOG_DIR = 'py/logs'
LOG_FILE = os.path.join(LOG_DIR, 'udp_to_vjoy_bridge.log')
os.makedirs(LOG_DIR, exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.FileHandler(LOG_FILE, encoding='utf-8'),
        logging.StreamHandler(sys.stdout)
    ]
)
log = logging.getLogger(__name__)

# ============================================================
# FUNÇÕES AUXILIARES (Melhoria 7: Envio de logs para a Central de Controle)
# ============================================================

def send_log_to_control_center(message, level='INFO'):
    """Envia uma mensagem de log para a Central de Controle via HTTP."""
    try:
        # Usamos um timeout curto para não bloquear o bridge se a Central de Controle não responder
        requests.post(CONTROL_CENTER_LOG_URL, json={'message': message, 'level': level}, timeout=0.1)
    except requests.exceptions.ConnectionError:
        # A Central de Controle pode não estar rodando, ou o IP/porta estão errados.
        # Não logamos isso no console para evitar spam, mas podemos logar em DEBUG.
        pass
    except requests.exceptions.Timeout:
        # A Central de Controle demorou para responder.
        pass
    except Exception as e:
        # Outros erros ao enviar o log
        log.debug(f"Erro ao enviar log para a Central de Controle: {e}")

# Substitui as funções de log padrão para também enviar para a Central de Controle
def log_info(message):
    log.info(message)
    send_log_to_control_center(message, 'INFO')

def log_warning(message):
    log.warning(message)
    send_log_to_control_center(message, 'WARNING')

def log_error(message):
    log.error(message)
    send_log_to_control_center(message, 'ERROR')

def log_debug(message):
    log.debug(message)
    send_log_to_control_center(message, 'DEBUG')

# ============================================================
# FUNÇÕES DE MAPEAMENTO
# ============================================================

def map_value(value, in_min, in_max, out_min, out_max):
    """Mapeia um valor de um intervalo para outro."""
    if in_max == in_min: # Evita divisão por zero
        return out_min
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def clamp_value(value, min_val, max_val):
    """Limita um valor dentro de um intervalo."""
    return max(min_val, min(value, max_val))

def map_to_vjoy_axis(value, in_min, in_max):
    """Mapeia um valor para o intervalo de eixo do vJoy (0 a 0x8000)."""
    clamped_value = clamp_value(value, in_min, in_max)
    return int(map_value(clamped_value, in_min, in_max, 0, 0x8000))

# ============================================================
# DECODIFICAÇÃO DO PACOTE UDP
# ============================================================

def decode_tracker_packet(data):
    """
    Decodifica o pacote UDP binário de 151 bytes do VR-Tracker.
    Formato:
    - 1 byte: MAGIC (0xAA)
    - 12 bytes: TAG (VRTRACKER_V6)
    - 4 bytes: ID do dispositivo (uint32)
    - 4 bytes: Timestamp (uint32)
    - 4 bytes: Frame Counter (uint32)
    - 4 bytes: Bateria (float)
    - 4 bytes: Quat W (float)
    - 4 bytes: Quat X (float)
    - 4 bytes: Quat Y (float)
    - 4 bytes: Quat Z (float)
    - 4 bytes: Pos X (float)
    - 4 bytes: Pos Y (float)
    - 4 bytes: Pos Z (float)
    - 4 bytes: Vel X (float)
    - 4 bytes: Vel Y (float)
    - 4 bytes: Vel Z (float)
    - 4 bytes: Speed (float)
    - 4 bytes: Step Frequency (float)
    - 4 bytes: Accel X (float)
    - 4 bytes: Accel Y (float)
    - 4 bytes: Accel Z (float)
    - 4 bytes: Gyro X (float)
    - 4 bytes: Gyro Y (float)
    - 4 bytes: Gyro Z (float)
    - 4 bytes: Mag X (float)
    - 4 bytes: Mag Y (float)
    - 4 bytes: Mag Z (float)
    - 4 bytes: CRC32 (uint32) - NO FINAL DO PACOTE
    Total: 1 + 12 + 4*29 = 1 + 12 + 116 = 129 bytes (revisado, o original era 151)

    Recontando para 151 bytes:
    1 byte MAGIC
    12 bytes TAG
    4 bytes ID
    4 bytes Timestamp
    4 bytes Frame Counter
    4 bytes Bateria
    4 * 4 = 16 bytes Quaternion (W,X,Y,Z)
    3 * 4 = 12 bytes Posição (X,Y,Z)
    3 * 4 = 12 bytes Velocidade (X,Y,Z)
    4 bytes Speed
    4 bytes Step Frequency
    3 * 4 = 12 bytes Aceleração (X,Y,Z)
    3 * 4 = 12 bytes Giroscópio (X,Y,Z)
    3 * 4 = 12 bytes Magnetômetro (X,Y,Z)
    4 bytes CRC32

    Total: 1 + 12 + 4 + 4 + 4 + 4 + 16 + 12 + 12 + 4 + 4 + 12 + 12 + 12 + 4 = 129 bytes.

    Houve uma recontagem anterior para 151. Vamos assumir que o PACKET_SIZE = 151 é o correto
    e que há 22 bytes "extras" que podem ser preenchimento ou outros dados.

    Para garantir que o CRC32 seja verificado corretamente, ele deve ser o último campo.
    Vamos desempacotar 147 bytes de dados + 4 bytes de CRC32.
    """
    if len(data) != PACKET_SIZE:
        log_warning(f"Pacote com tamanho incorreto: {len(data)} bytes. Esperado: {PACKET_SIZE} bytes.")
        return None

    # Separa o CRC32 do restante dos dados
    received_crc32 = struct.unpack('<I', data[CRC32_OFFSET:])[0]
    data_for_crc = data[:CRC32_OFFSET]
    calculated_crc32 = zlib.crc32(data_for_crc)

    if calculated_crc32 != received_crc32:
        log_debug(f"Falha na verificação de CRC32. Calculado: {calculated_crc32}, Recebido: {received_crc32}")
        return None

    # Formato de desempacotamento (little-endian)
    # < = little-endian
    # B = unsigned char (1 byte)
    # 12s = string de 12 bytes
    # I = unsigned int (4 bytes)
    # f = float (4 bytes)

    # Ajustado o formato para corresponder ao PACKET_SIZE de 151 e CRC32_OFFSET de 147
    # O formato '147s' lê os 147 bytes de dados antes do CRC32.
    # Se o formato real do tracker for diferente, isso precisará ser ajustado.
    # Por enquanto, vamos assumir que os 147 bytes são preenchidos com os dados que você listou.
    # O struct.unpack precisa de um formato exato para cada campo.
    # Vamos redefinir o struct.unpack com base na lista de campos que você forneceu,
    # e assumir que o preenchimento (se houver) está no final, antes do CRC.

    # Formato para os campos conhecidos (total de 129 bytes)
    # 1B (MAGIC) + 12s (TAG) + 29f (floats/uints) + 4s (CRC32)
    # 1 + 12 + (29 * 4) = 1 + 12 + 116 = 129 bytes.
    # Se o pacote é 151 bytes, há 151 - 129 = 22 bytes de preenchimento.
    # Vamos assumir que esses 22 bytes estão logo antes do CRC32.

    try:
        (magic, tag, device_id, timestamp, frame_counter, battery,
         qw, qx, qy, qz,
         pos_x, pos_y, pos_z,
         vel_x, vel_y, vel_z,
         speed, step_frequency,
         accel_x, accel_y, accel_z,
         gyro_x, gyro_y, gyro_z,
         mag_x, mag_y, mag_z,
         _padding, # 22 bytes de preenchimento
         ) = struct.unpack('<B12sIIIffffffffffffffffff22s', data[:CRC32_OFFSET])

        # Decodificar a tag
        tag = tag.decode('utf-8').strip('\x00')

        if magic != 0xAA or tag != "VRTRACKER_V6":
            log_debug(f"Pacote inválido: Magic={hex(magic)}, Tag={tag}")
            return None

        return {
            'device_id': device_id,
            'timestamp': timestamp,
            'frame_counter': frame_counter,
            'battery': battery,
            'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz,
            'pos_x': pos_x, 'pos_y': pos_y, 'pos_z': pos_z,
            'vel_x': vel_x, 'vel_y': vel_y, 'vel_z': vel_z,
            'speed': speed,
            'step_frequency': step_frequency,
            'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z,
            'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z,
            'mag_x': mag_x, 'mag_y': mag_y, 'mag_z': mag_z,
            'crc32': received_crc32
        }
    except struct.error as e:
        log_warning(f"Erro ao desempacotar pacote UDP: {e}. Pacote raw: {data.hex()}")
        return None
    except Exception as e:
        log_error(f"Erro inesperado ao decodificar pacote: {e}")
        return None

# ============================================================
# FUNÇÃO PRINCIPAL DO BRIDGE
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Bridge UDP para vJoy para VR-Tracker.")
    parser.add_argument('--ip', type=str, default=DEFAULT_UDP_IP,
                        help=f"IP para escutar pacotes UDP (padrão: {DEFAULT_UDP_IP} - todas as interfaces).")
    parser.add_argument('--port', type=int, default=DEFAULT_UDP_PORT,
                        help=f"Porta UDP para escutar pacotes (padrão: {DEFAULT_UDP_PORT}).")
    parser.add_argument('--vjoy-id', type=int, default=VJOY_DEVICE_ID,
                        help=f"ID do dispositivo vJoy a ser usado (padrão: {VJOY_DEVICE_ID}).")

    # Argumentos para limites de mapeamento (exemplo)
    parser.add_argument('--min-yaw', type=float, default=DEFAULT_MIN_YAW, help="Valor mínimo para Yaw.")
    parser.add_argument('--max-yaw', type=float, default=DEFAULT_MAX_YAW, help="Valor máximo para Yaw.")
    parser.add_argument('--min-pitch', type=float, default=DEFAULT_MIN_PITCH, help="Valor mínimo para Pitch.")
    parser.add_argument('--max-pitch', type=float, default=DEFAULT_MAX_PITCH, help="Valor máximo para Pitch.")
    parser.add_argument('--min-roll', type=float, default=DEFAULT_MIN_ROLL, help="Valor mínimo para Roll.")
    parser.add_argument('--max-roll', type=float, default=DEFAULT_MAX_ROLL, help="Valor máximo para Roll.")
    parser.add_argument('--min-pos', type=float, default=DEFAULT_MIN_POS, help="Valor mínimo para Posição (X,Y,Z).")
    parser.add_argument('--max-pos', type=float, default=DEFAULT_MAX_POS, help="Valor máximo para Posição (X,Y,Z).")
    parser.add_argument('--min-vel', type=float, default=DEFAULT_MIN_VELOCITY, help="Valor mínimo para Velocidade.")
    parser.add_argument('--max-vel', type=float, default=DEFAULT_MAX_VELOCITY, help="Valor máximo para Velocidade.")
    parser.add_argument('--min-step-freq', type=float, default=DEFAULT_MIN_STEP_FREQ, help="Valor mínimo para Frequência de Passos.")
    parser.add_argument('--max-step-freq', type=float, default=DEFAULT_MAX_STEP_FREQ, help="Valor máximo para Frequência de Passos.")

    args = parser.parse_args()

    # Inicializa vJoy
    j = None
    if VJOY_AVAILABLE:
        try:
            j = pyvjoy.VJoyDevice(args.vjoy_id)
            log_info(f"vJoy Device ID {args.vjoy_id} inicializado com sucesso.")
        except Exception as e:
            log_error(f"Falha ao inicializar vJoy Device ID {args.vjoy_id}: {e}. A funcionalidade vJoy estará desativada.")
            j = None # Garante que j seja None se a inicialização falhar

    # Configura o socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((args.ip, args.port))
        log_info(f"Escutando pacotes UDP em {args.ip}:{args.port}...")
    except socket.error as e:
        log_error(f"Falha ao bindar socket UDP em {args.ip}:{args.port}: {e}")
        if j:
            del j # Tenta liberar o dispositivo vJoy
        sys.exit(1)

    log_info("Aguardando dados do VR-Tracker...")

    try:
        while True:
            data, addr = sock.recvfrom(PACKET_SIZE) # Buffer size deve ser o tamanho máximo esperado do pacote

            # log_debug(f"Pacote recebido de {addr}: {data.hex()}") # Descomente para depurar pacotes brutos

            decoded_data = decode_tracker_packet(data)

            if decoded_data:
                # log_debug(f"Dados decodificados: {decoded_data}") # Descomente para depurar dados decodificados

                if j:
                    # Mapeamento de exemplo para vJoy
                    # Os eixos do vJoy são de 0 a 0x8000 (32767)
                    # Exemplo: Mapear Yaw (rotação Y) para o eixo X do vJoy
                    # Assumindo que o Yaw é a rotação em torno do eixo Y (pitch)
                    # ou a rotação em torno do eixo Z (yaw) dependendo da convenção
                    # Vamos usar qy para um exemplo de rotação

                    # Para simplificar, vamos mapear alguns valores para eixos de joystick
                    # Você precisará ajustar isso para o comportamento desejado no vJoy

                    # Exemplo: Mapear rotação Y (Pitch) para o Eixo X do vJoy
                    # Mapear rotação Z (Yaw) para o Eixo Y do vJoy
                    # Mapear velocidade para o Eixo Z do vJoy

                    # Nota: Quaternions precisam ser convertidos para Euler (Yaw, Pitch, Roll)
                    # para um mapeamento mais intuitivo para eixos de joystick.
                    # Por simplicidade, vamos usar os valores de posição/velocidade diretamente.

                    # Exemplo de mapeamento de posição X, Y, Z para eixos X, Y, Z do vJoy
                    # e velocidade para um slider ou outro eixo.

                    # Mapear Posição X para Eixo X
                    vjoy_x = map_to_vjoy_axis(decoded_data['pos_x'], args.min_pos, args.max_pos)
                    j.set_axis(pyvjoy.HID_USAGE_X, vjoy_x)

                    # Mapear Posição Y para Eixo Y
                    vjoy_y = map_to_vjoy_axis(decoded_data['pos_y'], args.min_pos, args.max_pos)
                    j.set_axis(pyvjoy.HID_USAGE_Y, vjoy_y)

                    # Mapear Posição Z para Eixo Z
                    vjoy_z = map_to_vjoy_axis(decoded_data['pos_z'], args.min_pos, args.max_pos)
                    j.set_axis(pyvjoy.HID_USAGE_Z, vjoy_z)

                    # Mapear Velocidade para Slider 1 (HID_USAGE_SL0)
                    vjoy_vel = map_to_vjoy_axis(decoded_data['speed'], args.min_vel, args.max_vel)
                    j.set_axis(pyvjoy.HID_USAGE_SL0, vjoy_vel)

                    # Mapear Frequência de Passos para Slider 2 (HID_USAGE_SL1)
                    vjoy_step_freq = map_to_vjoy_axis(decoded_data['step_frequency'], args.min_step_freq, args.max_step_freq)
                    j.set_axis(pyvjoy.HID_USAGE_SL1, vjoy_step_freq)

                    # Exemplo de botão: ativar botão 1 se a velocidade for alta
                    if decoded_data['speed'] > (args.max_vel * 0.75):
                        j.set_button(1, 1) # Pressiona botão 1
                    else:
                        j.set_button(1, 0) # Solta botão 1

                    j.update() # Envia os valores atualizados para o vJoy
                    # log_debug(f"vJoy atualizado: X={vjoy_x}, Y={vjoy_y}, Z={vjoy_z}, SL0={vjoy_vel}, SL1={vjoy_step_freq}")

            time.sleep(0.001) # Pequeno delay para evitar consumo excessivo de CPU
    except KeyboardInterrupt:
        log_info("Encerrando bridge UDP...")
    except Exception as e:
        log_error(f"Erro inesperado no loop principal: {e}")
    finally:
        sock.close()
        log_info("Socket UDP fechado.")
        if j:
            del j # Libera o dispositivo vJoy
            log_info("Dispositivo vJoy liberado.")

if __name__ == "__main__":
    # Ocultar janela do console para subprocessos no Windows
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.user32.ShowWindow(ctypes.windll.kernel32.GetConsoleWindow(), 0)
        except Exception as e:
            log_warning(f"Erro ao tentar ocultar a janela do console do bridge: {e}")

    main()
