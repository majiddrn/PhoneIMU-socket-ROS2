import rclpy
from rclpy.node import Node
import socket
import threading

from sensor_msgs.msg import Imu

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', 9999))
        self.server_socket.listen(1)
        self.client_thread = None
        self.get_logger().info('Socket server started and listening...')

    def start_server(self):
        while True:
            client_socket, client_address = self.server_socket.accept()
            self.get_logger().info(f'Accepted connection from {client_address}')
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_thread.start()
            self.client_threads = client_thread

    def handle_client(self, client_socket):
        while True:
            data = client_socket.recv(1024).decode()
            if not data:
                break
            self.get_logger().info(f'Received data: {data}')

        client_socket.close()
        self.get_logger().info('Client disconnected')

def main(args=None):
    rclpy.init(args=args)
    server_node = ServerNode()
    server_node.start_server()
    rclpy.spin(server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
