import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32MultiArray
from common_msgs.msg import ForkCmd
from pymodbus.client import ModbusTcpClient
import time

#parameters
timer_period = 0.01  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('data_node')

       
        self.init_modubus()  # 初始化Modbus通訊
        self.init_io_port()  # 初始化IO端口

        
        self.fork_cmd_subscriber = self.create_subscription(
            Int32MultiArray,
            'fork_io_cmd',
            self.fork_io_cmd_callback,
            10
        )

    def init_modubus(self):
        """初始化叉車Modbus通訊"""
        # 設備參數
        ip = "192.168.1.10"           # 請換成你的設備 IP
        port = 502                    # Modbus TCP 默認通訊埠
        self.slave_id = 2                   # 你的 Slave ID
        self.register_address = 0X9C60     # 要寫入的暫存器地址
        self.value_to_write = 0        # 寫入的值（16-bit 整數）

        # 建立連線S
        self.client = ModbusTcpClient(ip, port=port)
        self.client.connect()

    def init_io_port(self):
        self.Y0 = 0
        self.Y1 = 0
        self.Y2 = 0
        self.Y3 = 0
        self.Y4 = 0
        self.Y5 = 0
        self.Y6 = 0
        self.Y7 = 0
        self.Y8 = 0
        self.Y9 = 0
        self.Y10 = 0
        self.Y11 = 0
        self.Y12 = 0
        self.Y13 = 0
        self.Y14 = 0
        self.Y15 = 0

    def fork_io_cmd_callback(self, msg: Int32MultiArray):
        print(f"接收到叉車IO命令: {msg.data}")
        """處理叉車IO命令"""
        self.Y0 = msg.data[0]
        self.Y1 = msg.data[1]
        self.Y2 = msg.data[2]
        self.Y3 = msg.data[3]
        
class ForkliftControl(Machine):
    def __init__(self, data_node: DataNode):
        self.data_node = data_node

        
    def encode_outputs(self):
        y_values = [
            self.data_node.Y0, self.data_node.Y1, self.data_node.Y2, self.data_node.Y3,
            self.data_node.Y4, self.data_node.Y5, self.data_node.Y6, self.data_node.Y7,
            self.data_node.Y8, self.data_node.Y9, self.data_node.Y10, self.data_node.Y11,
            self.data_node.Y12, self.data_node.Y13, self.data_node.Y14, self.data_node.Y15
        ]
        value = 0
        for i, y in enumerate(y_values):
            value |= (y & 0x1) << i  # 每個 y 值都應該是 0 或 1
        return value

    def run(self):
        value_to_write = self.encode_outputs()
        print(f"寫入的值: {value_to_write}")
        register_address = self.data_node.register_address
        slave_id = self.data_node.slave_id
        self.data_node.client.write_register(address=register_address, value=value_to_write, slave=slave_id)







def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = ForkliftControl(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.run()
            time.sleep(timer_period)

    except KeyboardInterrupt:
        pass
    finally:
        data.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

# 🏁 若此檔案直接執行，就進入 main()
if __name__ == "__main__":
    main()