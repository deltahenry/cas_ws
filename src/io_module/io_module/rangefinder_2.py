import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import socket
import threading
import time


class TcpRangefinderNode(Node):
    def __init__(self):
        super().__init__('tcp_rangefinder_node')

        # TCP設定
        self.server_address = '192.168.1.100'
        self.port = 64000
        self.message = 'M0\r\n'
        self.frequency = 5.0  # 預設頻率 Hz

        # ROS Topic 設定
        self.create_subscription(Bool, 'start_tcp_stream', self.command_callback, 10)
        self.publisher_ = self.create_publisher(String, 'rangefinder_data', 10)

        # 控制旗標
        self.streaming = False
        self.lock = threading.Lock()

        # 背景執行緒
        self.thread = threading.Thread(target=self.tcp_loop, daemon=True)
        self.thread.start()

        self.get_logger().info('TCP Rangefinder Node has started.')

    def command_callback(self, msg):
        with self.lock:
            self.streaming = msg.data
            self.get_logger().info(f'Streaming: {"Started" if msg.data else "Stopped"}')

    def tcp_loop(self):
        rate = 1.0 / self.frequency
        while rclpy.ok():
            with self.lock:
                if self.streaming:
                    try:
                        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                            s.settimeout(2.0)
                            s.connect((self.server_address, self.port))
                            s.sendall(self.message.encode('ascii'))

                            data = s.recv(256)
                            message = data.decode('ascii').strip()

                            msg = String()
                            msg.data = message
                            self.publisher_.publish(msg)
                            self.get_logger().info(f'Received: {message}')

                    except Exception as e:
                        self.get_logger().warn(f'TCP Error: {e}')

            time.sleep(rate)


def main(args=None):
    rclpy.init(args=args)
    node = TcpRangefinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
