import sys
import argparse
import time
import signal
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QTimer
from server_ui import Ui_server_ui
from server import Server
import threading
import multiprocessing
from message import Message_Parse
from command import Command
from led import Led
from car import Car
from buzzer import Buzzer


class mywindow(QMainWindow, Ui_server_ui):
    def __init__(self):
        self.app = QApplication(sys.argv)
        super(mywindow, self).__init__()
        self.setupUi(self)

        # App flags
        self.server_running = False

        # Hardware / Server objects
        self.config_task()

        # UI button trigger
        self.Button_Server.clicked.connect(self.toggle_server)

        # Auto-enable server
        self.toggle_server()

        # Safe shutdown handlers
        self.app.lastWindowClosed.connect(self.close_application)
        signal.signal(signal.SIGINT, self.handle_sigint)

        # Timer (UI + async tasks)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_signals)
        self.timer.start(80)

        print("\n========= SYSTEM READY =========")

    # -----------------------------------------------------------
    # INITIAL HARDWARE + SERVER CONFIG
    # -----------------------------------------------------------
    def config_task(self):
        print("[INIT] Initializing modules...")

        self.tcp_server = Server()
        self.command = Command()
        self.led = Led()
        self.car = Car()
        self.buzzer = Buzzer()

        self.cmd_queue = multiprocessing.Queue()
        self.cmd_parser = Message_Parse()

        # Thread flags
        self.cmd_thread = None
        self.cmd_running = False

        self.car_thread = None
        self.car_running = False

        # Car auto mode → 1=Manual 2=Light 3=Line 4=Sonic
        self.car_mode = 1

    # -----------------------------------------------------------
    # UI BUTTON → START / STOP SERVER
    # -----------------------------------------------------------
    def toggle_server(self):
        if not self.server_running:
            print("\n[SERVER] → ENABLED")
            self.label.setText("Server On")
            self.Button_Server.setText("Stop")

            self.tcp_server.start_tcp_servers()
            self.start_cmd_thread()
            self.start_car_thread()

            self.server_running = True

        else:
            print("\n[SERVER] → DISABLED")
            self.label.setText("Server Off")
            self.Button_Server.setText("Start")

            self.stop_cmd_thread()
            self.stop_car_thread()

            self.tcp_server.stop_tcp_servers()
            self.server_running = False

    # -----------------------------------------------------------
    # THREAD : RECEIVING COMMANDS
    # -----------------------------------------------------------
    def start_cmd_thread(self):
        self.cmd_running = True
        self.cmd_thread = threading.Thread(target=self.cmd_loop)
        self.cmd_thread.start()
        print("[THREAD] Command Receiver → STARTED")

    def stop_cmd_thread(self):
        self.cmd_running = False
        if self.cmd_thread:
            self.cmd_thread.join(0.3)
        print("[THREAD] Command Receiver → STOPPED")

    def cmd_loop(self):
        while self.cmd_running:
            cmd_data = self.tcp_server.read_data_from_command_server()

            if cmd_data.qsize() > 0:
                client_addr, full_message = cmd_data.get()
                messages = full_message.strip().split("\n")

                for msg in messages:
                    msg = msg.strip()
                    if not msg:
                        continue

                    print(f"\n[RECEIVED] → {msg}")
                    self.cmd_parser.clear_parameters()
                    self.cmd_parser.parse(msg)

                    self.process_command()

            time.sleep(0.002)

    # -----------------------------------------------------------
    # COMMAND EXECUTION LOGIC
    # -----------------------------------------------------------
    def process_command(self):
        cmd = self.cmd_parser.command_string
        params = self.cmd_parser.int_parameter

        # ---------- MOTOR CONTROL ----------
        if cmd == self.command.CMD_MOTOR:
            try:
                duty = [int(v) for v in params]
                speed = abs(duty[0])  # intensité donnée par l'app

                # =============== Détection rotation ===============
                is_rotation = (duty[0] > 0 and duty[2] < 0) or (duty[0] < 0 and duty[2] > 0)

                if is_rotation:
                    # ---- Rotation droite ----
                    if duty[0] > 0:
                        LF = speed
                        LB = speed
                        RF = -speed
                        RB = -speed
                        print("↪️ Rotation DROITE")

                    # ---- Rotation gauche ----
                    else:
                        LF = -speed
                        LB = -speed
                        RF = speed
                        RB = speed
                        print("↩️ Rotation GAUCHE")

                else:
                    # -------- AVANCER / RECULER --------
                    # inversion car moteurs montés inversés
                    LF, LB, RF, RB = [-v for v in duty]
                    print("⬆️ Mouvement linéaire")

                # ENVOI MOTEURS
                self.car.motor.set_motor_model(LF, LB, RF, RB)
                print(f"[MOTOR] → LF:{LF} LB:{LB} RF:{RF} RB:{RB}")

            except Exception as e:
                print("[ERROR] MOTOR:", e)

        # ---------- MODE SWITCH ----------
        elif cmd == self.command.CMD_MODE:
            try:
                mode = int(params[0])
                self.car_mode = mode
                print(f"[MODE] → {mode}")
            except:
                print("[ERROR] MODE CMD INVALID")

        # ---------- SERVO CONTROL ----------
        elif cmd == self.command.CMD_SERVO:
            print("[SERVO CMD] Raw →", params)

            try:
                servo_id = str(params[0])
                angle = int(params[1])
                self.car.servo.set_servo_pwm(servo_id, angle)
                print(f"[SERVO] → ID={servo_id}, Angle={angle}")
            except Exception as e:
                print("[ERROR] SERVO:", e)

        # ---------- BUZZER ----------
        elif cmd == self.command.CMD_BUZZER:
            try:
                self.buzzer.set_state(params[0])
                print("[BUZZER] →", params[0])
            except:
                print("[ERROR] BUZZER")

    # -----------------------------------------------------------
    # THREAD : AUTO MODE BEHAVIOR
    # -----------------------------------------------------------
    def start_car_thread(self):
        self.car_running = True
        self.car_thread = threading.Thread(target=self.car_loop)
        self.car_thread.start()
        print("[THREAD] Car Auto Mode → STARTED")

    def stop_car_thread(self):
        self.car_running = False
        if self.car_thread:
            self.car_thread.join(0.3)
        print("[THREAD] Car Auto Mode → STOPPED")

    def car_loop(self):
        while self.car_running:
            try:
                if self.car_mode == 2:
                    self.car.mode_light()
                elif self.car_mode == 3:
                    self.car.mode_infrared()
                elif self.car_mode == 4:
                    self.car.mode_ultrasonic()
            except:
                pass

            time.sleep(0.015)

    # -----------------------------------------------------------
    # SAFE EXIT
    # -----------------------------------------------------------
    def close_application(self):
        print("\n[SHUTDOWN] Cleaning...")

        self.stop_cmd_thread()
        self.stop_car_thread()
        self.tcp_server.stop_tcp_servers()

        try:
            self.car.motor.set_motor_model(0, 0, 0, 0)
        except:
            pass

        sys.exit(0)

    def handle_sigint(self, *args):
        self.close_application()

    def check_signals(self):
        if self.app.hasPendingEvents():
            self.app.processEvents()


# -----------------------------------------------------------
# MAIN
# -----------------------------------------------------------
if __name__ == "__main__":
    myshow = mywindow()
    myshow.show()
    sys.exit(myshow.app.exec_())