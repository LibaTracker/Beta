import time
import vgamepad as vg

# =================================================================
# CONFIGURAÇÕES
# =================================================================
# Doom / Esteira
DOOM_DECAY_TIME = 0.4       # Segura o 'W' por 400ms após um passo
WALK_THRESHOLD = 10.0       # Threshold menor para esteira deslizante (ajuste se precisar)

# Reset (3 Pulos)
JUMP_THRESHOLD = 18.0       # Aceleração vertical (Y ou Z) para considerar um "pulo"
RESET_WINDOW = 2.0          # Tempo máximo para completar os 3 pulos
JUMPS_REQUIRED = 3          # Quantidade de pulos

class GameLogic:
    def __init__(self):
        try:
            self.gamepad = vg.VX360Gamepad()
            print("[GAME LOGIC] Controle Virtual Xbox 360 inicializado.")
        except Exception as e:
            print(f"[GAME LOGIC] Erro ao criar vgamepad: {e}")
            self.gamepad = None

        # Variáveis de Estado - Doom
        self.last_step_time = 0
        self.is_moving_forward = False

        # Variáveis de Estado - Reset
        self.jump_count = 0
        self.last_jump_time = 0
        self.in_jump_cooldown = False

    def process_data(self, accel_x, accel_y, accel_z, gyro_yaw):
        """
        Chame esta função a cada pacote recebido do tracker.
        Retorna 'True' se um RESET de Yaw foi solicitado.
        """
        current_time = time.time()

        # --- LÓGICA 1: RESET COM 3 PULOS ---
        # Assumindo que o tracker nas costas tem o eixo Y ou Z como vertical.
        # Vamos testar a magnitude total para garantir.
        accel_magnitude = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5

        # Detecta pico de aceleração (Pulo/Agachamento rápido)
        if accel_magnitude > JUMP_THRESHOLD:
            if not self.in_jump_cooldown:
                # Se passou muito tempo desde o último pulo, reseta a contagem
                if (current_time - self.last_jump_time) > RESET_WINDOW:
                    self.jump_count = 0

                self.jump_count += 1
                self.last_jump_time = current_time
                self.in_jump_cooldown = True # Evita contar o mesmo pulo várias vezes
                print(f"[GESTO] Pulo detectado! ({self.jump_count}/{JUMPS_REQUIRED})")

                if self.jump_count >= JUMPS_REQUIRED:
                    self.jump_count = 0
                    return True # SINAL DE RESET ATIVADO!
        else:
            # Reseta o cooldown quando a aceleração baixa
            if accel_magnitude < (JUMP_THRESHOLD - 5.0):
                self.in_jump_cooldown = False

        # --- LÓGICA 2: DOOM (DECAY) ---
        # Detecta passo na esteira (threshold menor pois é deslizante)
        is_stepping = accel_magnitude > WALK_THRESHOLD

        if is_stepping:
            self.last_step_time = current_time
            if not self.is_moving_forward:
                if self.gamepad:
                    # Empurra analógico esquerdo para frente
                    self.gamepad.left_joystick_float(x_value_float=0.0, y_value_float=1.0)
                    self.gamepad.update()
                self.is_moving_forward = True
                # print("Doom: Andando...")

        # Verifica se deve parar (Decay)
        if self.is_moving_forward and (current_time - self.last_step_time > DOOM_DECAY_TIME):
            if self.gamepad:
                # Solta o analógico
                self.gamepad.left_joystick_float(x_value_float=0.0, y_value_float=0.0)
                self.gamepad.update()
            self.is_moving_forward = False
            # print("Doom: Parou.")

        return False # Nenhum reset solicitado
