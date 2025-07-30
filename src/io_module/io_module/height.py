import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pycomm3 import CIPDriver


class LRSensorNode(Node):
    def __init__(self):
        super().__init__('lr_sensor_node')
        self.declare_parameter('sensor_ip', '192.168.1.101')
        self.declare_parameter('instance_id', 1)
        self.declare_parameter('read_rate', 0.1)

        self.sensor_ip = self.get_parameter('sensor_ip').get_parameter_value().string_value
        self.instance_id = self.get_parameter('instance_id').get_parameter_value().integer_value
        self.read_rate = self.get_parameter('read_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Int32, 'lr_distance', 10)
        self.timer = self.create_timer(self.read_rate, self.read_sensor_data)
        self.get_logger().info(f'LR Sensor Node started. IP: {self.sensor_ip}, Instance: {self.instance_id}')

    def read_sensor_data(self):
        try:
            with CIPDriver(self.sensor_ip) as client:
                response = client.generic_message(
                    service=0x0E,  # Get_Attribute_Single
                    class_code=0x66,
                    instance=self.instance_id,
                    attribute=0x0325
                )
                if response and hasattr(response, 'value'):
                    raw_bytes = response.value
                    self.get_logger().info(f'Raw bytes: {raw_bytes.hex()}')
                    int_value = int.from_bytes(raw_bytes, byteorder='little')
                    msg = Int32()
                    msg.data = int_value
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published Distance: {int_value} mm')
                else:
                    self.get_logger().warn('No value returned from sensor.')
        except Exception as e:
            self.get_logger().error(f'Failed to read sensor: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LRSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
