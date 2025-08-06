import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from common_msgs.msg import StateCmd,TaskCmd,MotionCmd,TaskState,MotionState

#parameters
timer_period = 0.5  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmd ={
            'pause_button': False,
        }

        self.task_cmd = "idle"  # rough align,precise align,pick,assembly

        self.func_cmd = {
            'pick_button': False,
            'push_button': False,
        }
       
        self.depth_data = [500.0,500.0]
        self.point_dist = 1000.0
       
        # 初始化 ROS2 Node
        #subscriber
        super().__init__('data_node')
        self.state_cmd_subscriber = self.create_subscription(
            StateCmd,
            '/state_cmd',
            self.state_cmd_callback,
            10
        )

        self.task_cmd_subscriber = self.create_subscription(
            TaskCmd,
            '/task_cmd',
            self.task_cmd_callback,
            10
        )

        self.motion_state_subscriber = self.create_subscription(
            MotionState,
            "/motion_state",
            self.motion_state_callback,
            10
        )
        
        self.depth_data_subscriber = self.create_subscription(
            Float32MultiArray,
            "/depth_data",
            self.depth_data_callback,
            10
        )

        #publisher
        self.precise_align_state_publisher = self.create_publisher(TaskState, '/task_state_precise_align', 10)
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.detection_cmd_publisher = self.create_publisher(String,'/detection_task',10)

    def state_cmd_callback(self, msg: StateCmd):
        print(f"接收到狀態命令: {msg}")
        # 在這裡可以處理狀態命令
        self.state_cmd = {
            'pause_button': msg.pause_button,
        }

    def task_cmd_callback(self, msg: TaskCmd):
        print(f"接收到任務命令: {msg.mode}")
        # 在這裡可以處理任務命令
        self.task_cmd = msg.mode

    def motion_state_callback(self, msg=MotionState):
        print(f"接收到運動狀態: {msg}")
        # 在這裡可以處理運動狀態
        self.motion_states = {
            'motion_finish': msg.motion_finish,
            'init_finish': msg.init_finish,
            'pull_finish': msg.pull_finish,                 
            'push_finish': msg.push_finish,
            'rough_pos_finish': msg.rough_pos_finish,
            'auto_pos_finish': msg.auto_pos_finish,
            'system_error': msg.system_error
        }

    def depth_data_callback(self, msg: Float32MultiArray):
        print(f"接收到深度數據: {msg.data}")
        # 在這裡可以處理深度數據
        self.depth_data = msg.data      
        # 更新深度數據
        if len(self.depth_data) >= 2:
            self.depth_data[0] = msg.data[0]
            self.depth_data[1] = msg.data[1]        
        else:
            self.get_logger().warn("接收到的深度數據長度不足，無法更新。")


class PreciseAlignState(Enum):
    IDLE = "idle"
    INIT = "init"
    SCREW_DETECT = "screw_detect"
    SCREW_ALIGN = "screw_align"
    BATTERY_DETECT = "battery_detect"
    BATTERY_ALIGN = "battery_align"
    DONE = "done"
    FAIL = "fail"

class PreciseAlignFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = PreciseAlignState.IDLE  # 初始狀態
        self.data_node = data_node
        self.run_mode = "pick"

        states = [
            PreciseAlignState.IDLE.value,
            PreciseAlignState.INIT.value,
            PreciseAlignState.SCREW_DETECT.value,
            PreciseAlignState.SCREW_ALIGN.value,
            PreciseAlignState.BATTERY_DETECT.value,
            PreciseAlignState.BATTERY_ALIGN.value,
            PreciseAlignState.DONE.value,
            PreciseAlignState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': PreciseAlignState.IDLE.value, 'dest': PreciseAlignState.INIT.value},
            {'trigger': 'init_to_screw_detect', 'source': PreciseAlignState.INIT.value, 'dest': PreciseAlignState.SCREW_DETECT.value},
            {'trigger': 'screw_detect_to_screw_align', 'source': PreciseAlignState.SCREW_DETECT.value, 'dest': PreciseAlignState.SCREW_ALIGN.value},
            {'trigger': 'screw_align_to_battery_detect', 'source': PreciseAlignState.SCREW_ALIGN.value, 'dest': PreciseAlignState.BATTERY_DETECT.value},
            {'trigger': 'battery_detect_to_battery_align', 'source': PreciseAlignState.BATTERY_DETECT.value, 'dest': PreciseAlignState.BATTERY_ALIGN.value},
            {'trigger': 'battery_align_to_done', 'source': PreciseAlignState.BATTERY_ALIGN.value, 'dest': PreciseAlignState.DONE.value},
            {'trigger': 'return_to_idle', 'source': '*', 'dest': PreciseAlignState.IDLE.value},
            {'trigger': 'return_to_fail', 'source': '*', 'dest': PreciseAlignState.FAIL.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = PreciseAlignState(self.state)

    def depth_ref(self,run_mode):
        """根據運行模式返回參考深度"""
        if run_mode == "pick":
            return 90.0
        elif run_mode == "push":
            return 0.3

    def reset_parameters(self):
        """重置參數"""
        self.run_mode = "pick"
        self.data_node.depth_data = [600.0, 600.0]
        self.data_node.point_dist = 1000.0
        self.data_node.state_cmd = {
            'pause_button': False,
        }
        self.data_node.func_cmd = {
            'pick_button': False,
            'push_button': False,
        }
        
        

    def step(self):
        if self.data_node.state_cmd.get("pause_button", False):
            print("[PreciseAlignmentFSM] 被暫停中")
        
        elif self.data_node.task_cmd == "precise_align":
            print("[PreciseAlignmentFSM] 開始手動對齊任務")
            self.run()
        else:
            print("[PreciseAlignmentFSM] 手動對齊任務未啟動，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        if self.state == PreciseAlignState.IDLE.value:
            print("[PreciseAlignmentFSM] 等待開始")
            if self.data_node.task_cmd == "precise_align":
                print("[PreciseAlignmentFSM] 開始對齊")
                self.idle_to_init()
            else:
                print("[PreciseAlignmentFSM] 對齊任務未啟動，等待中")
        
        elif self.state == PreciseAlignState.INIT.value:   
            print("[PreciseAlignmentFSM] 初始化階段")
            if self.data_node.func_cmd.get("pick_button", False):
                self.run_mode = "pick"
            elif self.data_node.func_cmd.get("push_button", False):
                self.run_mode = "push"
            else:
                print("[PreciseAlignmentFSM] 未選擇運行模式，等待人為選擇")
                return

        elif self.state == PreciseAlignState.SCREW_DETECT.value:
            print("[PreciseAlignmentFSM] 螺絲檢測階段")
            #open detection task screw
            self.data_node.detection_cmd_publisher.publish(String(data="screw"))
            #get screw detection result -> TF
            if self.data_node.screw_TF  == 123:
                print("[PreciseAlignmentFSM] 螺絲檢測未完成，等待中")
                return
            else:
                print("[PreciseAlignmentFSM] 螺絲檢測完成，進入螺絲對齊階段")
                # 假設檢測完成後進入下一個狀態
                self.screw_detect_to_screw_align()

        elif self.state == PreciseAlignState.SCREW_ALIGN.value:
            print("[PreciseAlignmentFSM] 螺絲對齊階段")
            # 根據螺絲檢測結果進行對齊
            screw_TF = self.data_node.screw_TF






def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = PreciseAlignFSM(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            # 更新狀態發布
            data.precise_align_state_publisher.publish(
                TaskState(mode="precise_align", state=system.state)
            )
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
