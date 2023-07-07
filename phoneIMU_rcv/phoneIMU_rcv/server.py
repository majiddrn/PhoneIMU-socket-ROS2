import rclpy
from rclpy.node import Node

import socket

class Server(Node):
    def __init__(self):
        super().__init__("server")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', 9999))
        self.server_socket.listen(1)
        self.get_logger().info('Socket server started and listening...')