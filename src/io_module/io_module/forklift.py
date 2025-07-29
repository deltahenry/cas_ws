import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from common_msgs.msg import ForkCmd
from pymodbus.client import ModbusTcpClient

#parameters
timer_period = 0.5  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('data_node')

        self.mode = "run"  # 狀態信息
        self.speed = "slow"
        self.direction = "up"
        self.distance = 10.0
        self.current_height = 0.0  # 當前高度，初始為0

        self.init_fork_modubus()  # 初始化叉車Modbus通訊
        
        self.fork_cmd_subscriber = self.create_subscription(
            ForkCmd,
            'fork_cmd',
            self.fork_cmd_callback,
            10
        )

        # self.component_control_cmd = "idle"  # 組件控制信息
        self.component_control_cmd = "forklift_control"  # 組件控制信息

    def init_fork_modubus(self):
        """初始化叉車Modbus通訊"""
        # 設備參數
        ip = "192.168.1.10"           # 請換成你的設備 IP
        port = 502                    # Modbus TCP 默認通訊埠
        self.slave_id = 2                   # 你的 Slave ID
        self.register_address = 0X9C60     # 要寫入的暫存器地址
        self.value_to_write = 0        # 寫入的值（16-bit 整數）

        # 建立連線S
        # self.client = ModbusTcpClient(ip, port=port)
        # self.client.connect()

    def fork_cmd_callback(self, msg: ForkCmd):
        print(f"[ForkCmd] 接收到叉車控制命令: {msg}")
        self.mode = msg.mode
        self.speed = msg.speed
        self.direction = msg.direction
        self.distance = msg.distance
        
class ForkliftControlState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    STOPPED = "stopped"
    FAIL = "fail"

class ForkliftControl(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = ForkliftControlState.IDLE  # 初始狀態
        self.data_node = data_node

        states = [
            ForkliftControlState.IDLE.value,
            ForkliftControlState.RUNNING.value,
            ForkliftControlState.STOPPED.value,
            ForkliftControlState.FAIL.value
        ]

        transitions = [
            # 狀態轉換
            {"trigger": "start", "source": ForkliftControlState.IDLE.value, "dest": ForkliftControlState.RUNNING.value},
            {"trigger": "stop", "source": ForkliftControlState.RUNNING.value, "dest": ForkliftControlState.STOPPED.value},
            {"trigger": "fail", "source": ForkliftControlState.RUNNING.value, "dest": ForkliftControlState.FAIL.value},
            {"trigger": "return_to_idle", "source": [ForkliftControlState.RUNNING.value, ForkliftControlState.STOPPED.value, ForkliftControlState.FAIL.value], "dest": ForkliftControlState.IDLE.value}
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = ForkliftControlState(self.state)


    def reset_parameters(self):
        """重置參數"""
        self.data_node.component_control_cmd = "idle"


    def step(self):
        # if self.data_node.state_cmds.get("pause_button", False):
        #     print("[ManualAlignmentFSM] 被暫停中")
        #     return  # 暫停中，不執行
        
        if self.data_node.component_control_cmd == "forklift_control":
            print("[ForkliftControl] 開始manual執行叉車控制任務")
            self.run()
        else:
            print("[ForkliftControl] 未收到叉車控制命令，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

    def encode(self, speed, direction):
        """將速度和方向編碼為 Modbus 寫入值"""
        if speed == "fast":
            y0 = 1
            y1 = 0
        elif speed == "slow":
            y0 = 0
            y1 = 1
        if direction == "up":
            y2 = 1
            y3 = 0
        elif direction == "down":
            y2 = 0
            y3 = 1
        
        value = 2^3*y3 + 2^2*y2 + 2^1*y1 + 2^0*y0
        return value

        # 任務完成或失敗時自動清除任務旗標
    def forklift_controller(self,speed_cmd, direction_cmd, distance_cmd):
        
        tolerance = 5  # 容差範圍

        if self.data_node.current_height > distance_cmd+ tolerance:
            value_to_write = self.encode("slow", "down")
        elif self.data_node.current_height < distance_cmd- tolerance:
            value_to_write = self.encode("fast", "up")
        else:
            value_to_write = 0
        
        register_address = self.data_node.register_address
        slave_id = self.data_node.slave_id

        # send Modbus write command
        print(f"[ForkliftControl] 發送 Modbus 寫入命令: 地址={register_address}, 值={value_to_write}, 從站ID={slave_id}")
        # self.data_node.client.write_register(address=register_address, value=value_to_write, slave=slave_id)

    def run(self):
        if self.state == ForkliftControlState.IDLE.value:
            print("[ForkliftControl] 叉車控制系統處於空閒狀態")
            if self.data_node.mode == "run":
                print("[ForkliftControl] 接收到運行命令，開始運行")
                self.start()
            elif self.data_node.mode == "stop":
                print("[ForkliftControl] 接收到停止命令，停止運行")
                self.stop()

        elif self.state == ForkliftControlState.RUNNING.value:
            print("[ForkliftControl] 叉車控制系統正在運行")
            result=self.forklift_controller(self.data_node.speed, self.data_node.direction, self.data_node.distance)
            if result:
                print("[ForkliftControl] 叉車控制任務完成")
                self.data_node.mode = "stop"
                self.return_to_stop()
            else:
                print("waiting")

        elif self.state == ForkliftControlState.STOPPED.value:
            print("[ForkliftControl] 叉車控制系統已停止")
            if self.data_node.mode == "run":
                print("[ForkliftControl] 接收到運行命令，重新開始運行")
                self.return_to_idle()
            elif self.data_node.mode == "stop":
                print("[ForkliftControl] 接收到停止命令，保持停止狀態")



def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = ForkliftControl(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
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
