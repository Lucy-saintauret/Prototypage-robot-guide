import serial
from prototypage.Motor import *

# Configuration du port série (remplace '/dev/ttyAMA0' par le port série correct)
ser = serial.Serial('/dev/ttyUSB0',  baudrate=115200, timeout=1)
pwm = Motor()
# Lecture des données envoyées sur le port série
while True:
    data = ser.readline().decode('utf-8').strip()  # Afficher la donnée lue
    print(data, type(data))
    if "top" in data :
            print('A')
            pwm.setMotorModel(1000, 1000, 1000, 1000)  # Forward
            time.sleep(1)
    if "back" in data:
            print('B')
            pwm.setMotorModel(-1000, -1000, -1000, -1000)  # Backward
            time.sleep(1)
    else :
        pwm.setMotorModel(0, 0, 0, 0)
