import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from common_msgs.msg import StateCmd,MotionState, MotionCmd

#parameters
timer_period = 0.5  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmd ={
            'pause_button': False,
        }

        self.task_cmd = "idle"  # rough align,precise align,pick,assembly
        # self.task_cmd = "rough_align"  # rough align,precise align,pick,assembly

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
        self.rough_align_state_publisher = self.create_publisher(String, '/task_state_rough_align', 10)
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.detection_cmd_publisher = self.create_publisher(String,'/detection_task',10)

    def state_cmd_callback(self, msg: StateCmd):
        print(f"接收到狀態命令: {msg}")
        # 在這裡可以處理狀態命令
        self.state_cmd = {
            'pause_button': msg.pause_button,
        }

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


class RoughAlignState(Enum):
    IDLE = "idle"
    INIT = "init"
    CHECK_DEPTH = "check_depth"
    ROUGH_ALIGN = "rough_align"
    DONE = "done"
    FAIL = "fail"

class RoughAlignFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = RoughAlignState.IDLE  # 初始狀態
        self.data_node = data_node
        self.run_mode = "pick"

        states = [
            RoughAlignState.IDLE.value,
            RoughAlignState.INIT.value,
            RoughAlignState.CHECK_DEPTH.value,
            RoughAlignState.ROUGH_ALIGN.value,
            RoughAlignState.DONE.value,
            RoughAlignState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': RoughAlignState.IDLE.value, 'dest': RoughAlignState.INIT.value},
            {'trigger': 'init_to_check_depth', 'source': RoughAlignState.INIT.value, 'dest': RoughAlignState.CHECK_DEPTH.value},
            {'trigger': 'check_depth_to_rough_align', 'source': RoughAlignState.CHECK_DEPTH.value, 'dest': RoughAlignState.ROUGH_ALIGN.value},
            {'trigger': 'rough_align_to_done', 'source': RoughAlignState.ROUGH_ALIGN.value, 'dest': RoughAlignState.DONE.value},
            {'trigger': 'done_to_fail', 'source': RoughAlignState.DONE.value, 'dest': RoughAlignState.FAIL.value},    
            {'trigger': 'return_to_idle', 'source': '*', 'dest': RoughAlignState.IDLE.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = RoughAlignState(self.state)

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
            print("[RoughAlignmentFSM] 被暫停中")
        
        elif self.data_node.task_cmd == "rough_align":
            print("[RoughAlignmentFSM] 開始手動對齊任務")
            self.run()
        else:
            print("[RoughAlignmentFSM] 手動對齊任務未啟動，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        if self.state == RoughAlignState.IDLE.value:
            print("[RoughAlignmentFSM] 等待開始手動對齊")
            if self.data_node.task_cmd == "rough_align":
                print("[RoughAlignmentFSM] 開始手動對齊")
                self.idle_to_init()
            else:
                print("[RoughAlignmentFSM] 手動對齊任務未啟動，等待中")
        
        elif self.state == RoughAlignState.INIT.value:
            print("[RoughAlignmentFSM] 初始化階段")
            if self.data_node.func_cmd.get("pick_button", False):
                self.run_mode = "pick"
            elif self.data_node.func_cmd.get("push_button", False):
                self.run_mode = "push"
            else:
                print("[RoughAlignmentFSM] 未選擇運行模式，等待人為選擇")
                return
            #open_vision_guide_line(red=True, green=False)
            #open_guide_laser
            self.init_to_check_depth()


        elif self.state == RoughAlignState.CHECK_DEPTH.value:
            print("[RoughAlignmentFSM] 深度檢查階段")
            # 檢查深度數據
            depth_ref = self.depth_ref(self.run_mode)

            if self.data_node.depth_data[0] < depth_ref:
                print("depth_data 小於參考深度，進入粗對齊階段")
                self.check_depth_to_rough_align()
            else:
                print("depth_data 大於或等於參考深度，waiting for human push")
        
        elif self.state == RoughAlignState.ROUGH_ALIGN.value:
            print("[RoughAlignmentFSM] 粗對齊階段")
            #open rough aligh check
            self.data_node.detection_cmd_publisher.publish(String(data='icp_fit'))
            if self.data_node.point_dist < 0.05:  # 假設 0.05 是粗對齊的閾值
                print("粗對齊完成，進入完成階段")
                self.rough_align_to_done()
                #close rough aligh check
                self.data_node.detection_cmd_publisher.publish(String(data='idle'))
            else:
                print("粗對齊未完成，等待人為調整")
        
        elif self.state == RoughAlignState.DONE.value:
            print("[RoughAlignmentFSM] 對齊完成階段")
            #change_vision_guide_line(red=False, green=True, blue=False)
            #open_guide_raser
            
        elif self.state == RoughAlignState.FAIL.value:
            print("[RoughAlignmentFSM] 對齊失敗階段")
            #change_vision_guide_line(red=False, green=False, blue=True)
            #open_guide_raser
            # 這裡可以加入對齊失敗的處理邏輯，例如重試或報錯



def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = RoughAlignFSM(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            # 更新狀態發布
            data.rough_align_state_publisher.publish(String(data=system.state))
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
