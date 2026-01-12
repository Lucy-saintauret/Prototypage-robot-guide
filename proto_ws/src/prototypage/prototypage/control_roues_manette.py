#!/usr/bin/python

import rclpy
import time
import serial
from control_mode.srv import RobotMode
from rclpy.node import Node
from prototypage.Motor import *
from prototypage.PCA9685 import PCA9685
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Wheels_Mode(Node):

    def __init__(self):
        super().__init__('control_wheels')
        self.srv = self.create_service(RobotMode, 'robot_mode', self.response_callback)
        # Configuration du port série (remplace '/dev/ttyAMA0' par le port série correct)
        # self.ser = serial.Serial('/dev/ttyUSB1',  baudrate=115200, timeout=1)
        self.ser = None  # Désactivé temporairement
        self.PWM = Motor()
        
        # Initialisation des servos pour la caméra
        self.servo_pwm = PCA9685(0x40, debug=False)
        self.servo_pwm.setPWMFreq(50)
        
        self.get_logger().info("Node control_wheels démarré et prêt !")
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.previous_buttons = [0] * 11
        
        self.axis_left_horizontal = 0
        self.axis_left_vertical = 1
        
        self.axis_right_horizontal = 3  # Joystick droit horizontal (pan - gauche/droite)
        self.axis_right_vertical = 4    # Joystick droit vertical (tilt - haut/bas)
        
        self.axis_LT = 2
        self.axis_RT = 5
        
        self.axis_dpad_vertical = 7
        
        self.button_avancer = 1
        
        self.is_moving = False
        self.land_executed = False
        
        self.max_speed = 2500
        
        # Position initiale des servos de la caméra (90° = centre)
        self.camera_pan = 90   # Servo horizontal (canal 8)
        self.camera_tilt = 90  # Servo vertical (canal 9)
        
        # Initialiser les servos au centre
        self.set_camera_servo(8, self.camera_pan)
        self.set_camera_servo(9, self.camera_tilt)
    
    def set_camera_servo(self, channel, angle):
        """Contrôle un servo de la caméra"""
        angle = max(0, min(180, angle))  # Limiter entre 0 et 180 degrés
        
        if channel == 8:  # Pan (gauche/droite)
            pulse = 2500 - int(angle / 0.09)
        elif channel == 9:  # Tilt (haut/bas)
            pulse = 500 + int(angle / 0.09)
        
        self.servo_pwm.setServoPulse(channel, pulse)
        
    def joy_callback(self, msg):
        # twist = Twist()
        rt_value = msg.axes[self.axis_RT]
        lt_value = msg.axes[self.axis_LT]
        
        turn_value = - msg.axes[self.axis_left_horizontal]
        
        trigger_pressed_rt = (1.0 - rt_value) / 2.0
        trigger_pressed_lt = (1.0 - lt_value) / 2.0
        
        deadzone_trigger = 0.1
        deadzone_turn = 0.15
        
        
        #-----------------------------------------------
        # Marche avant 
        #-----------------------------------------------
        
        
        if trigger_pressed_rt > deadzone_trigger:
            # Calculer la vitesse de base
            base_speed = int(trigger_pressed_rt * self.max_speed)
            base_speed = max(100, min(self.max_speed, base_speed))
            
            # Appliquer la direction si le joystick est poussé
            if abs(turn_value) > deadzone_turn:
                # turn_value: -1 = gauche, +1 = droite
                # On réduit la vitesse d'un côté pour tourner
                
                if turn_value < 0:  # Tourner à GAUCHE
                    # Réduire la vitesse des roues gauches
                    turn_factor = abs(turn_value)
                    left_speed = -base_speed * (1.0 - turn_factor)
                    right_speed = -base_speed
                else:  # Tourner à DROITE
                    # Réduire la vitesse des roues droites
                    turn_factor = turn_value
                    left_speed = -base_speed
                    right_speed = -base_speed * (1.0 - turn_factor)
                
                # setMotorModel(roue_avant_gauche, roue_arriere_gauche, roue_avant_droite, roue_arriere_droite)
                self.PWM.setMotorModel(int(left_speed), int(left_speed), 
                                    int(right_speed), int(right_speed))
                
                if not self.is_moving:
                    self.get_logger().info(f"Avancer avec virage ! Base: {base_speed}, Turn: {turn_value:.2f}")
                    self.is_moving = True
            else:
                # Avancer tout droit
                self.PWM.setMotorModel(-base_speed, -base_speed, -base_speed, -base_speed)
                
                if not self.is_moving:
                    self.get_logger().info(f"Avancer droit ! Vitesse: {base_speed}")
                    self.is_moving = True
        
        #-----------------------------------------------
        # Marche arrière
        #-----------------------------------------------    
        
        elif trigger_pressed_lt > deadzone_trigger:
            # Calculer la vitesse de base
            base_speed = int(trigger_pressed_lt * self.max_speed)
            base_speed = max(100, min(self.max_speed, base_speed))
            
            # Appliquer la direction si le joystick est poussé
            if abs(turn_value) > deadzone_turn:
                # turn_value: -1 = gauche, +1 = droite
                # On réduit la vitesse d'un côté pour tourner
                
                if turn_value < 0:  # Tourner à GAUCHE en reculant
                    # Réduire la vitesse des roues gauches
                    turn_factor = abs(turn_value)
                    left_speed = base_speed * (1.0 - turn_factor)  # Positif pour reculer
                    right_speed = base_speed  # Positif pour reculer
                else:  # Tourner à DROITE en reculant
                    # Réduire la vitesse des roues droites
                    turn_factor = turn_value
                    left_speed = base_speed  # Positif pour reculer
                    right_speed = base_speed * (1.0 - turn_factor)  # Positif pour reculer
                
                # setMotorModel(roue_avant_gauche, roue_arriere_gauche, roue_avant_droite, roue_arriere_droite)
                self.PWM.setMotorModel(int(left_speed), int(left_speed), 
                                    int(right_speed), int(right_speed))
                
                if not self.is_moving:
                    self.get_logger().info(f"Reculer avec virage ! Base: {base_speed}, Turn: {turn_value:.2f}")
                    self.is_moving = True
            else:
                # Reculer tout droit
                self.PWM.setMotorModel(base_speed, base_speed, base_speed, base_speed)  # Positif pour reculer
                
                if not self.is_moving:
                    self.get_logger().info(f"Reculer droit ! Vitesse: {base_speed}")
                    self.is_moving = True
        
        #-----------------------------------------------
        # Aucun trigger pressé : rotation sur place ou arrêt
        #-----------------------------------------------
        
        else:
            # Trigger relâché
            # On peut quand même tourner sur place avec le joystick
            if abs(turn_value) > deadzone_turn:
                # Tourner sur place
                turn_speed = int(abs(turn_value) * 2000)  # Vitesse de rotation sur place
                
                if turn_value < 0:  # Tourner sur place à GAUCHE
                    # Gauche en arrière, droite en avant
                    self.PWM.setMotorModel(turn_speed, turn_speed, -turn_speed, -turn_speed)
                else:  # Tourner sur place à DROITE
                    # Gauche en avant, droite en arrière
                    self.PWM.setMotorModel(-turn_speed, -turn_speed, turn_speed, turn_speed)
                
                self.get_logger().info(f"Rotation sur place: {turn_value:.2f}")
            else:
                # Tout arrêter
                if self.is_moving:
                    self.get_logger().info("Arrêt")
                    self.is_moving = False
                self.PWM.setMotorModel(0, 0, 0, 0)
    
        
        # speed = 1000
        # if trigger_pressed_rt > deadzone_trigger:
        #     # Calculer la vitesse proportionnelle à l'enfoncement du trigger
        #     base_speed = int(trigger_pressed_rt * self.max_speed)
        #     base_speed = max(100, min(self.max_speed, speed))  # Limiter entre 100 et max_speed
        #     
        #     if not self.is_moving:
        #         self.get_logger().info(f"Avancer ! Vitesse: {speed}")
        #         self.is_moving = True
        #     
        #     # Avancer avec la vitesse calculée
        #     self.PWM.setMotorModel(-speed, -speed, -speed, -speed)
        # else:
        #     # Trigger relâché, arrêter
        #     if self.is_moving:
        #         self.get_logger().info("Arrêt")
        #         self.is_moving = False
        #     self.PWM.setMotorModel(0, 0, 0, 0)
        #     
        # if trigger_pressed_lt > deadzone:
        #     # Calculer la vitesse proportionnelle à l'enfoncement du trigger
        #     speed = int(trigger_pressed_lt * self.max_speed)
        #     speed = max(100, min(self.max_speed, speed))  # Limiter entre 100 et max_speed
        #     
        #     if not self.is_moving:
        #         self.get_logger().info(f"Reculer ! Vitesse: {speed}")
        #         self.is_moving = True
        #     
        #     # Avancer avec la vitesse calculée
        #     self.PWM.setMotorModel(speed, speed, speed, speed)
        # else:
        #     # Trigger relâché, arrêter
        #     if self.is_moving:
        #         self.get_logger().info("Arrêt")
        #         self.is_moving = False
        #     self.PWM.setMotorModel(0, 0, 0, 0)
            
        # self.PWM.setMotorModel(0, 0, 0, 0) 
        
        # if msg.buttons[self.button_avancer] == 1 and self.previous_buttons[self.button_avancer] == 0:
        #     if not self.land_executed:
        #         self.land_executed = True
        #         self.is_flying = False  # Mise à jour manuelle de l'état
        #         self.PWM.setMotorModel(-2000, -2000, -2000, -2000) 
        # elif msg.buttons[self.button_avancer] == 0:
        #     if self.land_executed:  # Seulement si on était en mouvement
        #         self.get_logger().info("Arrêt")
        #         self.land_executed = False
        #     self.PWM.setMotorModel(0, 0, 0, 0)  # Arrêter
        
        
        #-----------------------------------------------
        # Contrôle de la caméra avec le joystick droit
        #-----------------------------------------------
        
        # Récupérer les valeurs du joystick droit
        right_horizontal = msg.axes[self.axis_right_horizontal]  # -1 (gauche) à +1 (droite)
        right_vertical = msg.axes[self.axis_right_vertical]      # -1 (bas) à +1 (haut)
        
        deadzone_camera = 0.15
        camera_speed = 2  # Vitesse de déplacement en degrés par update
        
        # Contrôle Pan (gauche/droite)
        if abs(right_horizontal) > deadzone_camera:
            # Inverser si nécessaire selon votre configuration
            self.camera_pan -= right_horizontal * camera_speed
            self.camera_pan = max(0, min(180, self.camera_pan))  # Limiter entre 0 et 180
            self.set_camera_servo(8, self.camera_pan)
        
        # Contrôle Tilt (haut/bas)
        if abs(right_vertical) > deadzone_camera:
            # Inverser si nécessaire selon votre configuration
            self.camera_tilt += right_vertical * camera_speed
            self.camera_tilt = max(0, min(180, self.camera_tilt))  # Limiter entre 0 et 180
            self.set_camera_servo(9, self.camera_tilt)
        
        self.get_logger().info("appuie sur le joystick")
        

    def response_callback(self,request,response) :
        print("in reponse call back")
        mode = request.mode  # Récupérer l'entier depuis la requête
        self.get_logger().info("call back")
        # if "mode0" in mode:
        #     self.mode = True
        #     self.get_logger().info("mode suivi activé")
        #     response.success=True
        # if "mode1" in mode:
        #     self.mode = False
        #     self.get_logger().info("mode manuel activé")
        #     response.success=True
        

        if "mode0" in mode:
            self.get_logger().info("loop auto")
            self.subscription_distance = self.create_subscription(
                Float32,
                '/target/cmd_vel',
                self.for_back,
                10)
            
            self.subscription_position = self.create_subscription(
                Float32,
                '/target/position',
                self.left_right,
                10)
            #self.subscription_position  # prevent unused variable warning

        #mode manuel activé
        if "mode1" in mode:
            while True:
                for_back = -1
                #Lecture des données envoyées sur le port série
                data = self.ser.readline().decode('utf-8').strip()  # Afficher la donnée lue
                print(data, type(data))
                if "forward" in data : 
                    for_back = -1 
                    self.PWM.setMotorModel(for_back*1000, for_back*1000, for_back*1000, for_back*1000)  # Forward
                elif "backward" in data:    
                    for_back = 1   
                    self.PWM.setMotorModel(for_back*1000, for_back*1000, for_back*1000, for_back*1000)  # Backward
                elif "right" in data:
                    self.PWM.setMotorModel(for_back*3500, for_back*3500, for_back*100, for_back*100)  # Right
                
                elif "left" in data:     
                    self.PWM.setMotorModel(for_back*100, for_back*100, for_back*3500, for_back*3500)   # Left
                    
                elif "stop" in data:
                    self.PWM.setMotorModel(0, 0, 0, 0)  # Stop
                    break


            return response       
                    
    def scale_axis(self, value):
        deadzone = 0.1
        
        if abs(value) < deadzone:
            return 0.0
        
        scaled = value * self.max_speed
        scaled = max(-self.max_speed, min(self.max_speed, scaled))
        
        return scaled

    def for_back(self, msg):
        print("C")
        if msg.data > 10:
            speed = abs(msg.data)*500
            if speed > 2000:
                speed = 2000
            self.PWM.setMotorModel(-speed, -speed, -speed, -speed)  # Forward
            time.sleep(1)
        elif msg.data < 10:
            speed = abs(msg.data)*500
            if speed > 2000:
                speed = 2000
            self.PWM.setMotorModel(speed, speed, speed, speed)  # Forward
            time.sleep(1)
        else:
            self.PWM.setMotorModel(0, 0, 0, 0)  # Stop
            time.sleep(1)

    def left_right(self, msg):
        if msg.data > 0 :
            self.PWM.setMotorModel(-3500, -3500, -100, -100)  # Turn left
            time.sleep(1)
        elif msg.data < 0 :
            self.PWM.setMotorModel(-100, -100, -3500, -3500)  # Turn right
            time.sleep(1)
        elif msg.data == 0 :
            self.PWM.setMotorModel(0, 0, 0, 0)  # Stop
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    control_wheels = Wheels_Mode()

    rclpy.spin(control_wheels)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_wheels.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()