import rclpy
from rclpy.node import Node
import socket
import threading

from sensor_msgs.msg import Imu

class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((socket.gethostname(), 9999))
        self.server_socket.listen(1)
        self.client_thread = None
        self.get_logger().info('Socket server started and listening...')
        self.start_server()

    def start_server(self):
        self.get_logger().info('waiting')
        while True:
            client_socket, client_address = self.server_socket.accept()
            self.get_logger().info(f'Accepted connection from {client_address}')
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_thread.start()
            self.client_threads = client_thread

    def receive_lines(self, socket):
        received_data = ""
        input_stream = socket.makefile('rb')
        while True:
            try:
                received_data = input_stream.read(2)
                if not received_data:
                    break
                received_string = received_data.decode('utf-8')
                print(received_string)
            except Exception as e:
                raise RuntimeError(e)

    def handle_client(self, client_socket):

        input_stream = client_socket.makefile('rb')
        self.get_logger().info(f'here1')

        while True:
             for line in self.receive_lines(client_socket):
                self.get_logger().info(f'{line}')


        self.server_socket.close()
        self.get_logger().info('Client disconnected')

def main(args=None):
    rclpy.init(args=args)
    server_node = Server()
    rclpy.spin(server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
