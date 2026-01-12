#!/usr/bin/python

import sys
import rclpy
import serial
from control_mode.srv import RobotMode
from rclpy.node import Node


class Client_Mode(Node):

    def __init__(self):
        super().__init__('client_mode')
        self.cli = self.create_client(RobotMode, '/robot_mode')
        self.ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)
        
        # Attendre que le service soit disponible
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RobotMode.Request()

    def read_uart(self):
        """Lit les données depuis l'UART."""
        if self.ser.in_waiting > 0:  # Vérifie s'il y a des données disponibles
            data = self.ser.readline().decode().strip()  # Lire une ligne de l'UART
            self.get_logger().info(f'Received UART data: {data}')
            return data
        return None

    def send_request(self, mode):
        """Envoie une requête au service avec le mode reçu."""
        self.req.mode = mode
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = Client_Mode()

    while rclpy.ok():
        # Lire les données de l'UART
        uart_data = minimal_client.read_uart()
        
        # Si des données sont reçues, les envoyer en requête
        if uart_data:
            try:
                mode = uart_data  
                future = minimal_client.send_request(mode)
                rclpy.spin_until_future_complete(minimal_client, future)
                
                # Afficher la réponse
                response = future.result()
                if response:
                    minimal_client.get_logger().info('Result of robot_control: %d' % response.success)
            except ValueError:
                minimal_client.get_logger().error('Invalid UART data format')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
