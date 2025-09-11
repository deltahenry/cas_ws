import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32MultiArray,Bool
from common_msgs.msg import DIDOCmd,MH2State
from pymodbus.client import ModbusTcpClient
import numpy as np

#parameters
timer_period = 0.05  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('data_node')

        self.tcp_connected = False   # 紀錄 TCP 狀態
       
        self.init_modbus()  # 初始化Modbus通訊

        self.init_io_port()  # 初始化IO端口

        self.left_gripper_state = 'open'  # 初始狀態
        self.right_gripper_state = 'open'  # 初始狀態
        self.gripper_state = [0,0] # 初始狀態 [left, right] open=0, close=1, moving=2
    

        self.left_limit_state = 'closed'  # 初始狀態
        self.right_limit_state = 'closed'  # 初始狀態
        self.limit_state = [0,0] # 初始狀態 [left, right] open=0, close=1, moving=2
        
        self.fork_cmd_subscriber = self.create_subscription(
            Int32MultiArray,
            'fork_io_cmd',
            self.fork_io_cmd_callback,
            10
        )

        self.gripper_cmd_subscriber = self.create_subscription(
            Int32MultiArray,
            'gripper_io_cmd',
            self.gripper_cmd_callback,  # 使用同一個回調函數處理不同命令
            10
        )

        self.limit_cmd_subscriber = self.create_subscription(
            Int32MultiArray,
            'limit_io_cmd',
            self.limit_cmd_callback,  # 使用同一個回調函數處理不同命令
            10
        )

        self.laser_cmd_subscriber = self.create_subscription(
            Int32MultiArray,
            'laser_io_cmd',
            self.laser_cmd_callback,  # 使用同一個回調函數處理不同命令
            10
        )

        self.dido_cmd_subscriber = self.create_subscription(
            DIDOCmd,
            'test_dido',
            self.handle_dido_cmd,
            10
        )

        self.MH2_state_publisher = self.create_publisher(
            MH2State,
            'mh2_state',
            10
        )

        self.tcp_status_pub = self.create_publisher(Bool, "tcp_status", 10)  # 加一個 publisher

        self.gripper_state_pub = self.create_publisher(Int32MultiArray, "gripper_state", 10)
        self.limit_state_pub = self.create_publisher(Int32MultiArray, "limit_state", 10)


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
        self.tcp_connected = self.client.connect()

    def ensure_connection(self):
        """確保 TCP 連線，若斷線則嘗試重連"""
        if not self.client.is_socket_open():
            self.tcp_connected = False
            self.get_logger().warn("⚠️ TCP 已斷線，嘗試重新連線...")
            self.tcp_connected = self.client.connect()

            if self.tcp_connected:
                self.get_logger().info("🔄 TCP 重連成功")
            else:
                self.get_logger().error("❌ TCP 重連失敗")
        
        # 發佈 TCP 狀態
        self.tcp_status_pub.publish(Bool(data=self.tcp_connected))

        return self.tcp_connected

    def init_io_port(self):
        
        self.on12_state = np.zeros(16, dtype=int)  # 對應位址 0x0008
        self.on34_state = np.zeros(16, dtype=int)  # 對應位址 0x0009

        self.J12_state = np.zeros(16, dtype=int)  # 對應位址 0x0008
        self.J34_state = np.zeros(16, dtype=int)  # 對應位址 0x0009
        
        self.J1_error = np.zeros(16, dtype=int)  # 對應位址 0x014C
        self.J2_error = np.zeros(16, dtype=int)  # 對應位址 0x014D
        self.J3_error = np.zeros(16, dtype=int)  # 對應位址 0x014E

        self.DI_1 = np.zeros(16, dtype=int)  # 初始化16個DI端口 對應位址 0x9810

        self.DO_1 = np.zeros(16, dtype=int)  # 初始化16個DO端口 對應位址 0x9C18
        self.DO_2 = np.zeros(16, dtype=int)  # 初始化16個DO端口 對應位址 0x9C20
        self.DO_3 = np.zeros(16, dtype=int)  # 初始化16個DO端口 對應位址 0x9C28

    def fork_io_cmd_callback(self, msg: Int32MultiArray):
        print(f"接收到叉車IO命令: {msg.data}")
        """處理叉車IO命令"""
        self.DO_1[0] = msg.data[0]
        self.DO_1[1] = msg.data[1]
        self.DO_1[2] = msg.data[2]
        self.DO_1[3] = msg.data[3]

    def gripper_cmd_callback(self, msg: Int32MultiArray):
        print(f"接收到夾爪IO命令: {msg.data}")
        """處理夾爪IO命令"""
        self.DO_2[0] = msg.data[0]
        self.DO_2[2] = msg.data[1]
        self.DO_2[3] = msg.data[2]
        
        self.DO_2[5] = msg.data[3]
        self.DO_2[7] = msg.data[4]
        self.DO_2[8] = msg.data[5]
    
    def limit_cmd_callback(self, msg: Int32MultiArray):
        print(f"接收到限位IO命令: {msg.data}")
        """處理限位IO命令"""
        self.DO_3[5] = msg.data[0]
        self.DO_3[6] = msg.data[1]
        self.DO_3[7] = msg.data[2]
        self.DO_3[8] = msg.data[3]
        self.DO_3[9] = msg.data[4]

        self.DO_3[10] = msg.data[5]
        self.DO_3[11] = msg.data[6]
        self.DO_3[12] = msg.data[7]
        self.DO_3[13] = msg.data[8]
        self.DO_3[14] = msg.data[9]

    def laser_cmd_callback(self, msg: Int32MultiArray):
        print(f"接收到雷射IO命令: {msg.data}")
        """處理雷射IO命令"""
        self.DO_2[15] = msg.data[0]
        self.DO_3[15] = msg.data[1]

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


class ForkliftControl():
    def __init__(self, data_node: DataNode):
        self.data_node = data_node

    def encode_outputs(self, do_array):
        value = 0
        for i, bit in enumerate(do_array):
            value |= (bit & 0x1) << i
        return value

    def run(self):
        # 每站的設定（可用清單定義來擴展性更強）

        if not self.data_node.ensure_connection():
            self.data_node.get_logger().error("❌ Modbus 斷線，略過 DO 寫入")
            return
    
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
            if result.isError():
                self.data_node.get_logger().error(
                    f"[Slave {slave_id}] 寫入 {hex(address)} 失敗: {result}"
                )
            else:
                self.data_node.get_logger().info(
                    f"[Slave {slave_id}] 寫入 {hex(address)}: {value} (0b{value:016b})"
                )

    def read_di(self):
        """從多個 Modbus 暫存器讀取 DI 狀態到各自陣列"""

        if not self.data_node.ensure_connection():
            self.data_node.get_logger().error("❌ Modbus 斷線，略過 DI 讀取")
            return

        try:
            di_map = [
                {"slave_id": 2,"address": 0x0006, "array": self.data_node.on12_state},
                {"slave_id": 2,"address": 0x0007, "array": self.data_node.on34_state},
                {"slave_id": 2,"address": 0x0008, "array": self.data_node.J12_state},
                {"slave_id": 2,"address": 0x0009, "array": self.data_node.J34_state},
                {"slave_id": 2,"address": 0x014C, "array": self.data_node.J1_error},
                {"slave_id": 2,"address": 0x014D, "array": self.data_node.J2_error},
                {"slave_id": 2,"address": 0x014E, "array": self.data_node.J3_error},
                {"slave_id": 2,"address": 0x9810, "array": self.data_node.DI_1},
            ]

            for item in di_map:
                addr = item["address"]
                arr = item["array"]
                slave_id = item["slave_id"]

                result = self.data_node.client.read_holding_registers(
                    address=addr,
                    count=1,
                    slave=slave_id
                )

                if result.isError():
                    print(f"讀取 DI 位址 {hex(addr)} 失敗: {result}")
                    continue

                reg_value = result.registers[0]
                for i in range(16):
                    arr[i] = (reg_value >> i) & 0x1
                print(f"[Slave {slave_id}] 讀取位址 {hex(addr)} 的值: {reg_value} (0b{reg_value:016b})")
        

            # 更新夾爪狀態
            self.update_gripper_state()
            # 更新限位狀態
            self.update_limit_state()   

        except Exception as e:
            print(f"Modbus DI 讀取異常: {e}")
        on12_state_int = self.bits_to_int(self.data_node.on12_state)
        on34_state_int = self.bits_to_int(self.data_node.on34_state)
        J12_state_int = self.bits_to_int(self.data_node.J12_state)
        J34_state_int = self.bits_to_int(self.data_node.J34_state)
        J1_error_int = self.bits_to_int(self.data_node.J1_error)
        J2_error_int = self.bits_to_int(self.data_node.J2_error)
        J3_error_int = self.bits_to_int(self.data_node.J3_error)
        DI_1_int = self.bits_to_int(self.data_node.DI_1)
        print("DI_1_int",DI_1_int)


        # 發佈 MH2 狀態
        mh2_state = MH2State()
        if J12_state_int == 257 and J34_state_int == 257:
            mh2_state.servo_state = True
        else:
            mh2_state.servo_state = False
        
        # 假設 alarm_code 取自 J1_error 的第一個元素
        mh2_state.alarm_code = 0  # 預設為正常狀態
        
        if J1_error_int != 0 or J2_error_int != 0 or J3_error_int != 0:
            mh2_state.servo_state = False
            mh2_state.alarm_code = J1_error_int  # 使用 J1_error 的值作為警報代碼
            print(f"MH2 狀態異常:J1_error: {J1_error_int}")
            print(f"MH2 狀態異常:J2_error: {J2_error_int}")
            print(f"MH2 狀態異常:J3_error: {J3_error_int}")
        
        self.data_node.MH2_state_publisher.publish(mh2_state)

    def bits_to_int(self,bits):
        value = 0
        for i, bit in enumerate(bits):
            value |= (bit & 1) << i
        return value

    def update_gripper_state(self):
        #N.C. 狀態下為 1，動作時變為 0
        if self.data_node.DI_1[3] == 0 and self.data_node.DI_1[4] == 1:
            self.data_node.left_gripper_state = 'open'
        elif self.data_node.DI_1[3] == 1 and self.data_node.DI_1[4] == 0:
            self.data_node.left_gripper_state = 'close'
        else:
            self.data_node.left_gripper_state = 'moving'

        if self.data_node.DI_1[5] == 0 and self.data_node.DI_1[6] == 1:
            self.data_node.right_gripper_state = 'open'
        elif self.data_node.DI_1[5] == 1 and self.data_node.DI_1[6] == 0:
            self.data_node.right_gripper_state = 'close'
        else:
            self.data_node.right_gripper_state = 'moving'
        # 更新狀態陣列
        state_map = {'open': 0, 'close': 1, 'moving': 2}
        self.data_node.gripper_state[0] = state_map[self.data_node.left_gripper_state]
        self.data_node.gripper_state[1] = state_map[self.data_node.right_gripper_state] 
        # 發佈狀態
        gripper_msg = Int32MultiArray()
        gripper_msg.data = self.data_node.gripper_state
        self.data_node.gripper_state_pub.publish(gripper_msg)
    
    def update_limit_state(self):
        if self.data_node.DI_1[11] == 0 and self.data_node.DI_1[12] == 1:
            self.data_node.left_limit_state = 'close'
        elif self.data_node.DI_1[11] == 1 and self.data_node.DI_1[12] == 0:
            self.data_node.left_limit_state = 'open'
        else:
            self.data_node.left_limit_state = 'moving'

        if self.data_node.DI_1[13] == 0 and self.data_node.DI_1[14] == 1:
            self.data_node.right_limit_state = 'close'
        elif self.data_node.DI_1[13] == 1 and self.data_node.DI_1[14] == 0:
            self.data_node.right_limit_state = 'open'
        else:
            self.data_node.right_limit_state = 'moving'
        # 更新狀態陣列
        state_map = {'open': 0, 'close': 1, 'moving': 2}
        self.data_node.limit_state[0] = state_map[self.data_node.left_limit_state]
        self.data_node.limit_state[1] = state_map[self.data_node.right_limit_state] 
        # 發佈狀態
        limit_msg = Int32MultiArray()
        limit_msg.data = self.data_node.limit_state
        self.data_node.limit_state_pub.publish(limit_msg)



def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = ForkliftControl(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.run()      # DO 控制
            system.read_di()  # 讀取 DI 狀態
            time.sleep(timer_period) #timer period = 50ms

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