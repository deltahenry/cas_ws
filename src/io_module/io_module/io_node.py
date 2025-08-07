import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32MultiArray
from common_msgs.msg import DIDOCmd
from pymodbus.client import ModbusTcpClient
import time
import numpy as np

#parameters
timer_period = 0.01  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('data_node')

       
        self.init_modbus()  # 初始化Modbus通訊
        self.init_io_port()  # 初始化IO端口

        
        self.fork_cmd_subscriber = self.create_subscription(
            Int32MultiArray,
            'fork_io_cmd',
            self.fork_io_cmd_callback,
            10
        )

        self.dido_cmd_subscriber = self.create_subscription(
            DIDOCmd,
            'dido_cmd',
            self.handle_dido_cmd,
            10
        )

        

    def init_modbus(self):
        """初始化叉車Modbus通訊"""
        # 設備參數
        ip = "192.168.1.10"           # 請換成你的設備 IP
        port = 502                    # Modbus TCP 默認通訊埠
        self.slave_id = 2                   # 你的 Slave ID
        self.register_address = 0X9C28     # 要寫入的暫存器地址
        self.value_to_write = 0        # 寫入的值（16-bit 整數）

        # # 建立連線S
        self.client = ModbusTcpClient(ip, port=port)
        self.client.connect()

    def init_io_port(self):
        self.DI_1 = np.zeros(16, dtype=int) # 初始化16個DI端口

        self.DO_1 = np.zeros(16, dtype=int)  # 初始化16個DO端口
        self.DO_2 = np.zeros(16, dtype=int)  # 初始化16個DO端口
        self.DO_3 = np.zeros(16, dtype=int)  # 初始化16個DO端口

    def fork_io_cmd_callback(self, msg: Int32MultiArray):
        print(f"接收到叉車IO命令: {msg.data}")
        """處理叉車IO命令"""
        self.DO_1[0] = msg.data[0]
        self.DO_1[1] = msg.data[1]
        self.DO_1[2] = msg.data[2]
        self.DO_1[3] = msg.data[3]

    def handle_dido_cmd(self, msg: DIDOCmd):
        if not msg.name.startswith("DO"):
            self.get_logger().warn(f"Unknown DO name format: {msg.name}")
            return

        try:
            index = int(msg.name[2:]) - 1  # "DO1" → index 0
            group = index // 16 + 1        # 每16個分為一組
            pin = index % 16               # 第幾個pin

            if group == 1:
                self.DO_1[pin] = int(msg.state)
            elif group == 2:
                self.DO_2[pin] = int(msg.state)
            elif group == 3:
                self.DO_3[pin] = int(msg.state)
            else:
                self.get_logger().warn(f"DO group {group} out of range.")
                return

            self.get_logger().info(f"Set DO{group}[{pin}] = {int(msg.state)}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse DO name '{msg.name}': {e}")


class ForkliftControl(Machine):
    def __init__(self, data_node: DataNode):
        self.data_node = data_node

        
    def encode_outputs(self, do_array):
        value = 0
        for i, bit in enumerate(do_array):
            value |= (bit & 0x1) << i
        return value

    def run(self):
        # 每站的設定（可用清單定義來擴展性更強）
        stations = [
            {"slave_id": 2, "do_array": self.data_node.DO_1, "register_address": 0x9C18},
            {"slave_id": 2, "do_array": self.data_node.DO_2, "register_address": 0x9C20},
            {"slave_id": 2, "do_array": self.data_node.DO_3, "register_address": 0x9C28},
        ]

        for station in stations:
            value = self.encode_outputs(station["do_array"])
            slave_id = station["slave_id"]
            address = station["register_address"]

            result = self.data_node.client.write_register(
                address=address,
                value=value,
                slave=slave_id
            )
            print(f"[Slave {slave_id}] 寫入位址 {hex(address)} 的值: {value} (0b{value:016b})")







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