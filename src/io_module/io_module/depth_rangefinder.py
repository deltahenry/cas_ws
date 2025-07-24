import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool,Float32MultiArray
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
        self.publisher_ = self.create_publisher(Float32MultiArray, 'depth_data', 10)

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

                            # 解析數值字串：格式應為 M0,-000005154,-000004835
                            try:
                                parts = message.split(",")
                                if len(parts) == 3 and parts[0].startswith("M"):
                                    value1 = float(parts[1]) / 100.0
                                    value2 = float(parts[2]) / 100.0

                                    # 儲存轉換結果
                                    self.range1 = value1
                                    self.range2 = value2

                                    self.get_logger().info(f'Parsed values: {value1:.2f}, {value2:.2f}')
                                    # 發佈到 ROS Topic
                                    depth_data_msg = Float32MultiArray()
                                    depth_data_msg.data = [value1, value2]
                                    self.publisher_.publish(depth_data_msg)
                                else:
                                    self.get_logger().warn(f'Unexpected message format: {message}')
                            except Exception as e:
                                self.get_logger().error(f'Error parsing message: {e}')

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

