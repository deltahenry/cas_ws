import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32,Float32MultiArray
from common_msgs.msg import ForkCmd
from pymodbus.client import ModbusTcpClient

#parameters
timer_period = 0.1  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('data_node')

        self.mode = "stop"  # 狀態信息
        self.speed = "slow"
        self.direction = "up"
        self.distance = 0
        self.current_height = 0  # 當前高度，初始為0

        self.control = 0.0

        self.can_forklift_cmd = True  # 是否可以發送叉車命令

        
        self.init_fork_modubus()  # 初始化叉車Modbus通訊
        
        self.fork_cmd_subscriber = self.create_subscription(
            ForkCmd,
            'fork_cmd',
            self.fork_cmd_callback,
            10
        )
        
        self.height_info_subscriber = self.create_subscription(
            Int32,
            'lr_distance',
            self.height_info_callback,
            10
        )

        self.height_cmd_info_publisher = self.create_publisher(Float32MultiArray, 'height_cmd_info', 10)


    def init_fork_modubus(self):
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

    def fork_cmd_callback(self, msg: ForkCmd):
        if msg.mode == "stop":
            print(f"[ForkCmd] 接收到叉車停止命令: {msg}")
            self.mode = msg.mode
            self.speed = msg.speed
            self.direction = msg.direction
            self.distance = msg.distance
        else:
            if self.can_forklift_cmd:
                self.get_logger().info(f"[ForkCmd] 接收到叉車控制命令: {msg}")
                self.mode = msg.mode
                self.speed = msg.speed
                self.direction = msg.direction
                self.distance = msg.distance
            else:
                print("[ForkCmd] 忽略一般命令，等待當前任務完成") 
        
    def height_info_callback(self,msg: Int32):
        """接收來自LR Sensor的高度信息"""
        self.get_logger().info(f"Received height info: {msg.data} mm")
        self.current_height = msg.data
        
class ForkliftControlState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    STOPPED = "stopped"
    FAIL = "fail"

class ForkliftControl(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = ForkliftControlState.IDLE  # 初始狀態
        self.data_node = data_node
        self.count = 0

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

    def step(self):

        height_info = Float32MultiArray()
        height_info.data = [float(self.data_node.distance), float(self.data_node.current_height),float(self.data_node.control)]
        self.data_node.height_cmd_info_publisher.publish(height_info)

        # if self.data_node.state_cmds.get("pause_button", False):
        #     print("[ManualAlignmentFSM] 被暫停中")
        #     return  # 暫停中，不執行
        
        if self.data_node.mode == "run":
            print("[ForkliftControl] 開始執行叉車控制任務")
            self.run()
            return
        
        else:
            print("[ForkliftControl] 非執行模式，強制回到 IDLE 狀態")
            if self.state != ForkliftControlState.IDLE.value:
                self.return_to_idle()
                self.data_node.can_forklift_cmd = True  # 允許發送新的命令
                register_address = self.data_node.register_address
                slave_id = self.data_node.slave_id
                value_to_write = 0
                self.data_node.client.write_register(address=register_address, value=value_to_write, slave=slave_id)

            return

    def encode(self, speed, direction):
        """將速度和方向編碼為 Modbus 寫入值"""
        if speed == "fast":
            y0 = 1
            y1 = 0
        elif speed == "slow":
            y0 = 0
            y1 = 1
        elif speed == "medium":
            y0 = 0
            y1 = 0
        if direction == "up":
            y2 = 1
            y3 = 0
        elif direction == "down":
            y2 = 0
            y3 = 1
        
        value = (2**3)*y3 + (2**2)*y2 + (2**1)*y1 + (2**0)*y0
        return value

     # 任務完成或失敗時自動清除任務旗標
    def forklift_controller(self,speed_cmd, direction_cmd, distance_cmd):
        
        tolerance = 3  # 容差範圍
        result = "waiting"
        register_address = self.data_node.register_address
        slave_id = self.data_node.slave_id
        # pid 

        if self.data_node.current_height > distance_cmd+ tolerance:
            value_to_write = self.encode(speed_cmd, "down")
            # if (self.data_node.current_height - distance_cmd) < 20:
            #     value_to_write = self.encode("medium", "down")
            # else:
            #     value_to_write = self.encode("medium", "down")
        elif self.data_node.current_height < distance_cmd- tolerance:
            value_to_write = self.encode(speed_cmd, "up")
            
        else:
            value_to_write = 0
            result = "done"

        # send Modbus write command
        print(f"[ForkliftControl] 發送 Modbus 寫入命令: 地址={register_address}, 值={value_to_write}, 從站ID={slave_id}")
        self.data_node.client.write_register(address=register_address, value=value_to_write, slave=slave_id)
        
        return result

    def run(self):

        if self.state == ForkliftControlState.IDLE.value:
            print("[ForkliftControl] 叉車控制系統處於空閒狀態")
            if self.data_node.mode == "run":
                print("[ForkliftControl] 接收到運行命令，開始運行")
                self.start()

        elif self.state == ForkliftControlState.RUNNING.value:
            print("[ForkliftControl] 叉車控制系統正在運行")
            
            result=self.forklift_controller(self.data_node.speed, self.data_node.direction, self.data_node.distance)
            self.data_node.can_forklift_cmd = False  # 禁止發送新的命令，直到任務完成
            self.data_node.control = 1.0
            
            if result == "done":
                print("[ForkliftControl] 叉車控制任務完成")
                self.data_node.mode = "stop"
                self.stop()
                self.data_node.can_forklift_cmd = True  # 任務完成後允許發送新的命令
                self.data_node.control = 0.0
                self.count = 0
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