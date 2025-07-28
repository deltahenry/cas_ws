import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from common_msgs.msg import StateCmd,MotionState, MotionCmd,JogCmd

#parameters
timer_period = 0.5  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmds ={
            'pause_button': False,
        }

        self.func_cmds = {
            'pick_button': False,
            'push_button': False,
        }
       
        self.depth_data = [500.0,500.0]

        # self.run_cmd = "idle"  # 狀態信息
        self.run_cmd = "component_control"  # 組件控制信息

        # self.component_control_cmd = "idle"  # 組件控制信息
        self.component_control_cmd = "pose_control"

        self.can_accept_pose_cmd = False  # 是否可以接受姿態控制命令
       
        # 初始化 ROS2 Node
        #subscriber
        super().__init__('data_node')
        self.state_info_subscriber = self.create_subscription(
            String,
            '/state_info',
            self.state_info_callback,
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
        self.jog_cmd_subscriber = self.create_subscription(
            JogCmd,
            "/jog_cmd",
            self.jog_cmd_callback,
            10
        )

        #publisher
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)

    def state_info_callback(self, msg: String):
        print(f"接收到狀態信息: {msg.data}")
        self.state_info = msg.data      
    
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

    def jog_cmd_callback(self, msg: JogCmd):
        print(f"接收到JOG控制命令: {msg}")
        # 在這裡可以處理姿態控制命令
        if self.can_accept_pose_cmd:
            if msg.target == "x_axis":
                x_move = msg.direction*msg.distance
                y_move = 0.0
                yaw_move = 0.0
            elif msg.target == "y_axis":
                x_move = 0.0
                y_move = msg.direction*msg.distance
                yaw_move = 0.0
            elif msg.target == "yaw_axis":
                x_move = 0.0
                y_move = 0.0
                yaw_move = msg.direction*msg.angle

            motion = MotionCmd()
            motion.command_type = MotionCmd.TYPE_GOTO_RELATIVE
            motion.pose_data = [x_move, y_move, yaw_move]
            motion.speed = msg.speed
            self.motion_cmd_publisher.publish(motion)

        else:
            self.get_logger().warn("目前不接受姿態控制命令。")

class ComponentControlState(Enum):
    IDLE = "idle"
    INIT = "init"
    POSE_Control = "pose_control"
    VISION_Control = "vision_control"
    CLIPPER_Control = "clipper_control"
    FAIL = "fail"

class ManualAlignFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = ComponentControlState.IDLE  # 初始狀態
        self.data_node = data_node
        self.run_mode = "pick"

        states = [
            ComponentControlState.IDLE.value,
            ComponentControlState.POSE_Control.value,
            ComponentControlState.VISION_Control.value,
            ComponentControlState.CLIPPER_Control.value,
            ComponentControlState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_pose_control', 'source': ComponentControlState.IDLE.value, 'dest': ComponentControlState.POSE_Control.value},
            {'trigger': 'idle_to_vision_control', 'source': ComponentControlState.IDLE.value, 'dest': ComponentControlState.VISION_Control.value},
            {'trigger': 'idle_to_clipper_control', 'source': ComponentControlState.IDLE.value, 'dest': ComponentControlState.CLIPPER_Control.value},
            {'trigger': 'return_to_idle', 'source': '*', 'dest': ComponentControlState.IDLE.value},
            {'trigger': 'to fail', 'source': '*', 'dest': ComponentControlState.FAIL.value}
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = ComponentControlState(self.state)

    def depth_ref(self,run_mode):
        """根據運行模式返回參考深度"""
        if run_mode == "pick":
            return 90.0
        elif run_mode == "push":
            return 0.3

    def reset_parameters(self):
        """重置參數"""
        self.run_mode = "pick"
        self.data_node.state_info = "idle"
        self.data_node.depth_data = [600.0, 600.0]
        self.data_node.state_cmds = {
            'pause_button': False,
        }
        self.data_node.func_cmds = {
            'pick_button': False,
            'push_button': False,
        }

    def step(self):
        if self.data_node.state_cmds.get("pause_button", False):
            print("[ManualAlignmentFSM] 被暫停中")
            return  # 暫停中，不執行
        
        if self.data_node.run_cmd == "component_control":
            print("[Start ComponentControl] 開始組件控制任務")
            self.run()
        else:
            print("[ComponentControl] 未啟動組件控制任務，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        if self.state == ComponentControlState.IDLE.value:
            print("[ComponentControl] 等待開始組件控制")
            if self.data_node.run_cmd == "component_control":
                print("[ComponentControl] 開始組件控制")
                if self.data_node.component_control_cmd == "pose_control":
                    self.idle_to_pose_control()
                elif self.data_node.component_control_cmd == "vision_control":
                    self.idle_to_vision_control()
                elif self.data_node.component_control_cmd == "clipper_control":
                    self.idle_to_clipper_control()
                else:
                    print("[ComponentControl] 未知的組件控制任務，等待中")
            else:
                print("[ComponentControl] 組件控制任務未啟動，等待中")
                
        elif self.state == ComponentControlState.POSE_Control.value:
            print("[ComponentControl] 姿態控制階段")
            if self.data_node.component_control_cmd == "pose_control":
                self.data_node.can_accept_pose_cmd = True  # 可以接受姿態控制命令
            else:
                self.data_node.can_accept_pose_cmd = False
                self.return_to_idle()  # 返回到空閒狀態

        elif self.state == ComponentControlState.VISION_Control.value:
            if self.data_node.component_control_cmd == "vision_control":
                print("[ComponentControl] 進入視覺控制階段")
                # 在這裡添加視覺控制的邏輯
            else:
                print("[ComponentControl] 退出視覺控制階段")
                self.return_to_idle()

        elif self.state == ComponentControlState.CLIPPER_Control.value:
            if self.data_node.component_control_cmd == "clipper_control":
                print("[ComponentControl] 進入夾爪控制階段")
                # 在這裡添加夾爪控制的邏輯
            else:
                print("[ComponentControl] 退出夾爪控制階段")
                self.return_to_idle()
        
            
            
                
        



        



def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = ManualAlignFSM(data)    # FSM 實體

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
