import rclpy
from rclpy.node import Node
import socket
import threading
import json

from sensor_msgs.msg import Imu

class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.publisher_imu = self.create_publisher(Imu, 'phone_imu', 10)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((socket.gethostname(), 9999))
        self.server_socket.listen(1)
        self.client_thread = None
        self.get_logger().info('Socket server started and listening...')
        self.start_server()

    def publish(self, json_arr):
        decoded_array = json.loads(json_arr)

        # first gyroscope, second is accelerator and the third is orientation data
        phone_imu_data = Imu()

        phone_imu_data.angular_velocity.x = float(decoded_array['imu_data'][0][0])
        phone_imu_data.angular_velocity.y = float(decoded_array['imu_data'][0][1])
        phone_imu_data.angular_velocity.z = float(decoded_array['imu_data'][0][2])
        
        phone_imu_data.linear_acceleration.x = float(decoded_array['imu_data'][1][0])
        phone_imu_data.linear_acceleration.y = float(decoded_array['imu_data'][1][1])
        phone_imu_data.linear_acceleration.z = float(decoded_array['imu_data'][1][2])

        phone_imu_data.orientation.x = float(decoded_array['imu_data'][2][0])
        phone_imu_data.orientation.y = float(decoded_array['imu_data'][2][1])
        phone_imu_data.orientation.z = float(decoded_array['imu_data'][2][2])

        phone_imu_data.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        phone_imu_data.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        phone_imu_data.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_imu.publish(phone_imu_data)



    def start_server(self):
        self.get_logger().info('waiting')
        while True:
            client_socket, client_address = self.server_socket.accept()
            self.get_logger().info(f'Accepted connection from {client_address}')
            client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_thread.start()
            self.client_threads = client_thread

    def receive_lines(self, sock):
        buffer = ""
        while True:
            data = sock.recv(1024).decode()  # Receive data from the socket
            if not data:
                break
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                yield line

    def handle_client(self, client_socket):
        input_stream = client_socket.makefile('rb')
        self.get_logger().info(f'here1')
    
        for line in self.receive_lines(client_socket):
            self.publish(line)
            self.get_logger().info(f'{line}')
            # print(line)


def main(args=None):
    rclpy.init(args=args)
    server_node = Server()
    rclpy.spin(server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
