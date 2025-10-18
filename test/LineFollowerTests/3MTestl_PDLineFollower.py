#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Center-Line PD Follower (inversion-safe) with Discontinuous Line Support
- Enhanced recovery system with timer reset on line detection
- Simple buzzer tones for program states
- Maintains last correction during line loss timeout
- Added parameter adjustment menu
"""

# ===== CONFIGURACIÓN DE PARÁMETROS =====
"""
Valores actuales de las variables para ganancia y control:
- KP: 18 (Ganancia proporcional, ajusta la respuesta a errores grandes)
- KD: 2.0 (Ganancia derivativa, ajusta la respuesta a cambios rápidos en el error)
- BASE_SPEED: 20 (Velocidad base del robot) 
- MAX_SPEED: 80 (Velocidad máxima permitida por rueda)
- MIN_SPEED: 10 (Velocidad mínima permitida por rueda)
- LINE_THRESHOLD: 4 (Umbral para detección de línea)
- LINE_LOSS_TIMEOUT: 0.7 segundos (Tiempo para mantener corrección al perder la línea)
- SEARCH_SPEED: 30 (Velocidad de búsqueda al perder la línea)
- CORRECTION_FILTER_ALPHA: 0.65 (Factor de suavizado para corrección)
- MAX_CORRECTION: 18.0 (Corrección máxima permitida)
- DEAD_ZONE: 1.0 (Zona muerta para evitar correcciones pequeñas)

Cambios recientes:
- Se agregó el motor trasero (`mB`) para mejorar los giros.
- La lógica del motor trasero ajusta su velocidad proporcionalmente al error calculado.
- Se mejoró el guardado de la última dirección de giro.
- Se reparó la máquina de estados y la impresión de estado.
- Se añadió menú de ajuste de parámetros (tecla 'p')
"""

import sys
import select
import tty
import termios
import time

from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_B, SpeedPercent

# ===== CENTER-LINE PD CONFIGURATION =====
KP = 18
KD = 2.0
BASE_SPEED = 20
MAX_SPEED = 80
MIN_SPEED = 10

LINE_THRESHOLD = 14 # Current model use 10
LINE_LOSS_TIMEOUT = 0.7
SEARCH_SPEED = 30

CORRECTION_FILTER_ALPHA = 0.65
MAX_CORRECTION = 18.0
DEAD_ZONE = 0.5

SAMPLE_INTERVAL = 0.02
INVERT_MOTORS = True

cl_L = ColorSensor('in4')
cl_R = ColorSensor('in1')
try:
    cl_L.mode = 'COL-REFLECT'
    cl_R.mode = 'COL-REFLECT'
except Exception:
    pass

mL = LargeMotor(OUTPUT_D)
mR = LargeMotor(OUTPUT_A)
mB = LargeMotor(OUTPUT_B)

sound = Sound()
sound.beep()

def apply_motor_speeds(left_speed, right_speed, rear_speed, invert=INVERT_MOTORS):
    l = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    r = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    b = max(min(rear_speed, MAX_SPEED), -MAX_SPEED)
    if invert:
        l = -l
        r = -r
        # b = -b  DO NOT INVERT REAR MOTOR
    mL.on(SpeedPercent(l))
    mR.on(SpeedPercent(r))
    mB.on(SpeedPercent(b))
    return l, r, b

class CenterLinePDFollower:
    def __init__(self, kp, kd, base_speed, max_speed, min_speed):
        self.kp = float(kp)
        self.kd = float(kd)
        self.base_speed = float(base_speed)
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)
        self.last_error = 0.0
        self.last_time = None
        self.filtered_correction = 0.0
        self.recovery_state = "NORMAL"
        self.recovery_start_time = 0.0
        self.last_known_correction = 0.0
        self.consecutive_black_detections = 0
        self.last_turn_direction = "RIGHT"

    def compute(self, left_value, right_value):
        line_detected = self._is_line_detected(left_value, right_value)
        line_lost = self._is_line_lost(left_value, right_value)

        if line_lost:
            current_time = time.time()
            if self.recovery_state == "NORMAL":
                self.recovery_state = "TIMER_STRAIGHT"
                self.recovery_start_time = current_time
                self.last_known_correction = self.filtered_correction
                self.consecutive_black_detections = 0
                return self._timer_straight_recovery()
            elif self.recovery_state == "TIMER_STRAIGHT":
                if line_detected:
                    self.consecutive_black_detections += 1
                    if self.consecutive_black_detections >= 2:
                        self.recovery_start_time = current_time
                        self.consecutive_black_detections = 0
                else:
                    self.consecutive_black_detections = 0
                if current_time - self.recovery_start_time < LINE_LOSS_TIMEOUT:
                    return self._timer_straight_recovery()
                else:
                    self.recovery_state = "SEARCH_360"
                    return self._start_360_search()
            elif self.recovery_state == "SEARCH_360":
                return self._continue_360_search()
        else:
            if self.recovery_state != "NORMAL":
                self.recovery_state = "NORMAL"
                self.consecutive_black_detections = 0
            return self._normal_center_line_following(left_value, right_value)

    def _is_line_detected(self, left_value, right_value):
        return (left_value <= LINE_THRESHOLD) or (right_value <= LINE_THRESHOLD)

    def _is_line_lost(self, left_value, right_value):
        return (left_value > LINE_THRESHOLD) and (right_value > LINE_THRESHOLD)

    def _timer_straight_recovery(self):
        left_speed = self.base_speed - self.last_known_correction
        right_speed = self.base_speed + self.last_known_correction
        rear_speed = self.base_speed
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        rear_speed = max(min(rear_speed, self.max_speed), -self.max_speed)
        return left_speed, right_speed, rear_speed

    def _normal_center_line_following(self, left_value, right_value):
        now = time.time()
        if self.last_time is None:
            dt = SAMPLE_INTERVAL
        else:
            dt = now - self.last_time
            if dt <= 0.0:
                dt = SAMPLE_INTERVAL
        self.last_time = now
        error = float(right_value) - float(left_value)
        avg = (left_value + right_value) / 2.0
        if avg > LINE_THRESHOLD + 15:
            cruise = min(self.max_speed, self.base_speed + 4)
        elif avg > LINE_THRESHOLD:
            cruise = self.base_speed
        else:
            cruise = max(self.min_speed, self.base_speed - 8)
        P = self.kp * error
        D = self.kd * ((error - self.last_error) / dt) if dt > 0 else 0.0
        self.last_error = error
        correction = P + D
        if abs(error) < DEAD_ZONE:
            correction = 0.0
        correction = max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)
        self.filtered_correction = (CORRECTION_FILTER_ALPHA * self.filtered_correction +
                                    (1.0 - CORRECTION_FILTER_ALPHA) * correction)
        # Guardar la última dirección de giro de forma robusta
        if correction > 1.0:
            self.last_turn_direction = "RIGHT"
        elif correction < -1.0:
            self.last_turn_direction = "LEFT"
        left_speed = cruise - self.filtered_correction
        right_speed = cruise + self.filtered_correction
        rear_speed = cruise - (self.filtered_correction * 0.5)
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
        rear_speed = max(min(rear_speed, self.max_speed), -self.max_speed)
        return left_speed, right_speed, rear_speed

    def _start_360_search(self):
        direction = -1 if self.last_turn_direction == "RIGHT" else 1
        rear_speed = 0
        return direction * SEARCH_SPEED, direction * -SEARCH_SPEED, rear_speed

    def _continue_360_search(self):
        direction = -1 if self.last_turn_direction == "RIGHT" else 1
        rear_speed = 0
        return direction * SEARCH_SPEED, direction * -SEARCH_SPEED, rear_speed

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def parameter_adjustment_menu():
    """Menú para ajustar los parámetros del robot"""
    global KP, KD, BASE_SPEED, MAX_SPEED, MIN_SPEED
    global LINE_LOSS_TIMEOUT, SEARCH_SPEED, CORRECTION_FILTER_ALPHA
    global MAX_CORRECTION, DEAD_ZONE, SAMPLE_INTERVAL, INVERT_MOTORS
    
    while True:
        print("\n" + "="*50)
        print("PARAMETER ADJUSTMENT MENU")
        print("="*50)
        print("1. KP (Proportional Gain): {}".format(KP))
        print("2. KD (Derivative Gain): {}".format(KD))
        print("3. BASE_SPEED: {}".format(BASE_SPEED))
        print("4. MAX_SPEED: {}".format(MAX_SPEED))
        print("5. MIN_SPEED: {}".format(MIN_SPEED))
        print("6. LINE_LOSS_TIMEOUT: {}".format(LINE_LOSS_TIMEOUT))
        print("7. SEARCH_SPEED: {}".format(SEARCH_SPEED))
        print("8. CORRECTION_FILTER_ALPHA: {}".format(CORRECTION_FILTER_ALPHA))
        print("9. MAX_CORRECTION: {}".format(MAX_CORRECTION))
        print("10. DEAD_ZONE: {}".format(DEAD_ZONE))
        print("11. SAMPLE_INTERVAL: {}".format(SAMPLE_INTERVAL))
        print("12. INVERT_MOTORS: {}".format(INVERT_MOTORS))
        print("13. Return to main menu")
        print("-"*50)
        
        try:
            choice = input("Select parameter to change (1-13): ").strip()
            if not choice:
                continue
                
            choice = int(choice)
            
            if choice == 13:
                print("Returning to main menu...")
                break
            elif 1 <= choice <= 12:
                current_value = get_parameter_value(choice)
                new_value = input("Enter new value for parameter {} (current: {}): ".format(
                    get_parameter_name(choice), current_value))
                
                if new_value.strip():
                    try:
                        # Convertir al tipo apropiado
                        if choice in [8, 11]:  # CORRECTION_FILTER_ALPHA y SAMPLE_INTERVAL son floats
                            new_value = float(new_value)
                        elif choice == 12:  # INVERT_MOTORS es booleano
                            if new_value.lower() in ['true', '1', 'yes', 'y']:
                                new_value = True
                            elif new_value.lower() in ['false', '0', 'no', 'n']:
                                new_value = False
                            else:
                                print("Invalid boolean value. Use True/False or 1/0.")
                                continue
                        else:  # Los demás son enteros o floats
                            if '.' in new_value:
                                new_value = float(new_value)
                            else:
                                new_value = int(new_value)
                        
                        set_parameter_value(choice, new_value)
                        print("Parameter {} updated to: {}".format(get_parameter_name(choice), new_value))
                        
                        # Actualizar el controlador PD si es necesario
                        if hasattr('pd_controller', 'pd_controller'):
                            if choice in [1, 2, 3, 4, 5]:
                                pd_controller.kp = KP
                                pd_controller.kd = KD
                                pd_controller.base_speed = BASE_SPEED
                                pd_controller.max_speed = MAX_SPEED
                                pd_controller.min_speed = MIN_SPEED
                        
                    except ValueError:
                        print("Invalid value. Please enter a valid number.")
                else:
                    print("No value entered. Parameter unchanged.")
            else:
                print("Invalid choice. Please select 1-13.")
                
        except (ValueError, KeyboardInterrupt):
            print("Invalid input. Please enter a number between 1-13.")

def get_parameter_name(choice):
    """Obtener el nombre del parámetro basado en la selección"""
    names = {
        1: "KP", 2: "KD", 3: "BASE_SPEED", 4: "MAX_SPEED", 5: "MIN_SPEED",
        6: "LINE_LOSS_TIMEOUT", 7: "SEARCH_SPEED", 8: "CORRECTION_FILTER_ALPHA",
        9: "MAX_CORRECTION", 10: "DEAD_ZONE", 11: "SAMPLE_INTERVAL", 12: "INVERT_MOTORS"
    }
    return names.get(choice, "Unknown")

def get_parameter_value(choice):
    """Obtener el valor actual del parámetro"""
    values = {
        1: KP, 2: KD, 3: BASE_SPEED, 4: MAX_SPEED, 5: MIN_SPEED,
        6: LINE_LOSS_TIMEOUT, 7: SEARCH_SPEED, 8: CORRECTION_FILTER_ALPHA,
        9: MAX_CORRECTION, 10: DEAD_ZONE, 11: SAMPLE_INTERVAL, 12: INVERT_MOTORS
    }
    return values.get(choice, None)

def set_parameter_value(choice, value):
    """Establecer nuevo valor para el parámetro"""
    global KP, KD, BASE_SPEED, MAX_SPEED, MIN_SPEED
    global LINE_LOSS_TIMEOUT, SEARCH_SPEED, CORRECTION_FILTER_ALPHA
    global MAX_CORRECTION, DEAD_ZONE, SAMPLE_INTERVAL, INVERT_MOTORS
    
    if choice == 1:
        KP = value
    elif choice == 2:
        KD = value
    elif choice == 3:
        BASE_SPEED = value
    elif choice == 4:
        MAX_SPEED = value
    elif choice == 5:
        MIN_SPEED = value
    elif choice == 6:
        LINE_LOSS_TIMEOUT = value
    elif choice == 7:
        SEARCH_SPEED = value
    elif choice == 8:
        CORRECTION_FILTER_ALPHA = value
    elif choice == 9:
        MAX_CORRECTION = value
    elif choice == 10:
        DEAD_ZONE = value
    elif choice == 11:
        SAMPLE_INTERVAL = value
    elif choice == 12:
        INVERT_MOTORS = value

old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setcbreak(sys.stdin.fileno())
    pd_controller = CenterLinePDFollower(KP, KD, BASE_SPEED, MAX_SPEED, MIN_SPEED)
    print("=== ENHANCED CENTER-LINE PD FOLLOWER WITH REAR MOTOR ===")
    print("Press 's' start/stop, 'r' reset, 'c' calibrate, 'p' parameters, 'q' quit")
    running = False
    stop_signal = False

    while not stop_signal:
        if is_data():
            c = sys.stdin.read(1)
            if c == 'q':
                stop_signal = True
                sound.beep()
            elif c == 's':
                running = not running
                if running:
                    sound.beep()
                    print("STARTING LINE FOLLOWING...")
                else:
                    print("STOPPING...")
                    apply_motor_speeds(0.0, 0.0, 0.0)
            elif c == 'r':
                pd_controller.last_error = 0.0
                pd_controller.last_time = None
                pd_controller.filtered_correction = 0.0
                pd_controller.recovery_state = "NORMAL"
                pd_controller.consecutive_black_detections = 0
                print("Controller reset")
            elif c == 'c':
                sound.beep()
                print("\n=== SENSOR CALIBRATION ===")
                print("Place robot so line is BELOW sensors and press Enter")
                input()
                centered_L = cl_L.value()
                centered_R = cl_R.value()
                print("Centered - Left: {} Right: {}".format(centered_L, centered_R))
                print("Move robot OFF the line (both sensors on white) and press Enter")
                input()
                off_L = cl_L.value()
                off_R = cl_R.value()
                print("Off line - Left: {} Right: {}".format(off_L, off_R))
                suggested = (min(centered_L, centered_R) + max(off_L, off_R)) / 2.0
                LINE_THRESHOLD = suggested
                print("Suggested LINE_THRESHOLD: {:.0f}".format(suggested))
            elif c == 'p':
                # Entrar al menú de ajuste de parámetros
                running = False
                apply_motor_speeds(0.0, 0.0, 0.0)
                # Restaurar configuración temporal de terminal para input()
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                parameter_adjustment_menu()
                # Volver a modo no bloqueante
                tty.setcbreak(sys.stdin.fileno())
                print("Press 's' start/stop, 'r' reset, 'c' calibrate, 'p' parameters, 'q' quit")

        if running and not stop_signal:
            try:
                left_value = cl_L.value()
                right_value = cl_R.value()
            except Exception:
                left_value = cl_L.reflected_light_intensity
                right_value = cl_R.reflected_light_intensity

            left_speed, right_speed, rear_speed = pd_controller.compute(left_value, right_value)
            applied_l, applied_r, applied_b = apply_motor_speeds(left_speed, right_speed, rear_speed)

            error = float(right_value) - float(left_value)
            avg = (left_value + right_value) / 2.0
            state = pd_controller.recovery_state
            if state == "TIMER_STRAIGHT":
                time_remaining = LINE_LOSS_TIMEOUT - (time.time() - pd_controller.recovery_start_time)
                state += " ({:.1f}s)".format(max(0, time_remaining))
            print("L:{:3d} R:{:3d} | Avg:{:4.1f} | Err:{:6.2f} | ML:{:5.1f} MR:{:5.1f} MB:{:5.1f} | State: {}".format(
                int(left_value), int(right_value), avg, error, applied_l, applied_r, applied_b, state))

        elif not stop_signal:
            apply_motor_speeds(0.0, 0.0, 0.0)

        time.sleep(SAMPLE_INTERVAL)

    apply_motor_speeds(0.0, 0.0, 0.0)
    print("Program terminated")

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    apply_motor_speeds(0.0, 0.0, 0.0)