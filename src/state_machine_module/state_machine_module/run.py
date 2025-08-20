import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32
from common_msgs.msg import StateCmd,TaskCmd,MotionCmd,TaskState,ForkCmd,ForkState,Recipe,CurrentPose,ClipperCmd
import numpy as np

#parameters
timer_period = 0.5  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmd ={
            'init_button': False,
            'pause_button': False,
            'run_button': False,
            'stop_button': False,
        }

        self.task_cmd = "idle"  # rough align,precise align,pick,RUN
     
        self.forkstate = "idle"

        self.current_pose = [0.0, 0.0, 0.0]

        self.current_height = 0.0

        self.target_depth = 500.0


       
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
        
        self.depth_data_subscriber = self.create_subscription(
            Float32MultiArray,
            "/depth_data",
            self.depth_data_callback,
            10
        )

        self.height_info_subscriber = self.create_subscription(
            Int32,
            'lr_distance',
            self.height_info_callback,
            10
        )

        self.fork_state_subscriber = self.create_subscription(
            ForkState,
            'fork_state',
            self.fork_state_callback,
            10
        )

        self.recipe_data_subscriber = self.create_subscription(
            Recipe,
            'recipe_data',
            self.recipe_callback,
            10
        )

        self.current_pose_subscriber = self.create_subscription(    
            CurrentPose,
            'current_pose',
            self.current_pose_callback,
            10
        )

        #publisher
        self.RUN_state_publisher = self.create_publisher(TaskState, '/task_state_RUN', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.detection_cmd_publisher = self.create_publisher(String,'/detection_task',10)
        self.fork_cmd_publisher = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.clipper_cmd_publisher = self.create_publisher(ClipperCmd, 'clipper_cmd', 10)
        
    def publish_fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.fork_cmd_publisher.publish(msg)
        self.get_logger().info(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")

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

    def height_info_callback(self,msg: Int32):
        """接收來自LR Sensor的高度信息"""
        self.get_logger().info(f"Received height info: {msg.data} mm")
        self.current_height = msg.data

    def fork_state_callback(self, msg: ForkState):
        self.forkstate = msg.state  # 假設 ForkState 有個 .state 屬性

    def recipe_callback(self, msg: Recipe):
        self.target_mode = msg.mode
        self.target_height = msg.height
        # 在這裡可以添加更多的處理邏輯
        # 例如，根據接收到的 recipe 更新其他狀態或觸發其他操作

    def current_pose_callback(self, msg: CurrentPose):
        """接收當前機器人位置"""
        self.get_logger().info(f"Received current pose: {msg.pose_data}")
        self.current_pose[0] = msg.pose_data[0]
        self.current_pose[1] = msg.pose_data[1]
        self.current_pose[2] = msg.pose_data[2]


class RUNState(Enum):
    IDLE = "idle"
    INIT = "init"
    ROUGH_ALIGN = "rough_align"
    PRECISE_ALIGN = "precise_align"
    ASSEMBLY = "assembly"
    PICK = "pick"
    DONE = "done"
    FAIL = "fail"

class RUNFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = RUNState.IDLE  # 初始狀態
        self.data_node = data_node
        self.motor_cmd_sent = False
        self.send_fork_cmd = False

        states = [
            RUNState.IDLE.value,
            RUNState.INIT.value,       
            RUNState.ROUGH_ALIGN.value,
            RUNState.PRECISE_ALIGN.value,
            RUNState.ASSEMBLY.value,
            RUNState.PICK.value,
            RUNState.DONE.value,
            RUNState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': RUNState.IDLE.value, 'dest': RUNState.INIT.value},
            

            {'trigger': 'fail', 'source': '*', 'dest': RUNState.FAIL.value},
            {'trigger': 'return_to_idle', 'source': '*', 'dest': RUNState.IDLE.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = RUNState(self.state)

    def depth_ref(self,run_mode):
        """根據運行模式返回參考深度"""
        if run_mode == "pick":
            return 90.0
        elif run_mode == "push":
            return 90.0

    def reset_parameters(self):
        """重置參數"""
        self.motor_cmd_sent = False
        self.send_fork_cmd = False
        

    def step(self):
        if self.data_node.state_cmd.get("pause_button", False):
            print("[RUNmentFSM] 被暫停中")
        
        elif self.data_node.task_cmd == "RUN":
            print("[RUNmentFSM] 開始手動對齊任務")
            self.run()
        else:
            print("[RUNmentFSM] 手動對齊任務未啟動，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        push_pose_cmd = [0.0,500.0,0.0]  # 推進階段的目標位置
        back_pose_cmd = [0.0, 0.0, 0.0]  # 回到家位置的目標位置

        if self.state == RUNState.IDLE.value:
            print("[RUNmentFSM] 等待開始")
            if self.data_node.task_cmd == "RUN":
                self.idle_to_init()
                print("[RUNmentFSM] 進入初始化階段")
            
        elif self.state == RUNState.INIT.value:
            print("[RUNmentFSM] 初始化階段")
            self.init_to_push()
        
        elif self.state == RUNState.PUSH.value:
            print("[RUNmentFSM] 推進階段")
            if not self.motor_cmd_sent:
                self.sent_motor_cmd(push_pose_cmd)
                self.motor_cmd_sent = True  # 標記已發送初始化命令
            else:
                print("[RUNmentFSM] 馬達命令已發送，等待完成")
                push_arrive = self.check_pose(push_pose_cmd)
                if push_arrive:
                    print("[RUNmentFSM] 馬達已到達推進位置")
                    self.motor_cmd_sent = False  # 重置標記
                    self.push_to_open_clipper()
                else:
                    print("[RUNmentFSM] 馬達尚未到達推進位置，繼續等待")
        
        elif self.state == RUNState.OPEN_CLIPPER.value:
            print("[RUNmentFSM] 開啟夾爪階段")
            self.send_clipper_cmd("open_clipper")
            time.sleep(10)  # 等待夾爪開啟
            self.open_clipper_to_back_home()

        elif self.state == RUNState.BACK_HOME.value:
            print("[RUNmentFSM] 回到家位置階段")
            if not self.motor_cmd_sent:
                self.sent_motor_cmd(back_pose_cmd)
                self.motor_cmd_sent = True
            else:
                print("[RUNmentFSM] 馬達命令已發送，等待完成")
                back_arrive = self.check_pose(back_pose_cmd)
                if back_arrive:
                    print("[RUNmentFSM] 馬達已到達家位置")
                    self.back_home_to_move_forklift()
                else:
                    print("[RUNmentFSM] 馬達尚未到達家位置，繼續等待")
        
        elif self.state == RUNState.MOVE_FORKLIFT.value:
            print("[RUNmentFSM] 移動叉車階段")
            if not self.send_fork_cmd:
                height_cmd = 100.0
                tolerance = 5.0

                if abs(self.data_node.current_height - height_cmd) < tolerance:
                    print("[RUNmentFSM] 叉車已到達目標高度")
                    self.move_forklift_to_done()
                else:
                    print(f"[RUNmentFSM] 叉車尚未到達目標高度，當前高度: {self.data_node.current_height}, 目標高度: {height_cmd}")
                    self.fork_cmd(mode="run", speed="slow", direction="down", distance= height_cmd)
                    self.send_fork_cmd = True
            else:
                print("[RUNmentFSM] 叉車命令已發送，等待完成")
        
        elif self.state == RUNState.DONE.value:
            print("[RUNmentFSM] 任務完成")
            self.data_node.task_cmd = "idle"
            self.return_to_idle()
        
        elif self.state == RUNState.FAIL.value:
            print("[RUNmentFSM] 任務失敗")
            self.data_node.task_cmd = "idle"
            self.return_to_idle()
        
        else:
            print(f"[RUNmentFSM] 未知狀態: {self.state}")
            self.data_node.task_cmd = "idle"
            self.return_to_idle()

    def fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.data_node.fork_cmd_publisher.publish(msg)
        print(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")

    def sent_motor_cmd(self,pose_cmd):
        """發送馬達初始化命令"""
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_Y_MOVE
        msg.pose_data = [pose_cmd[0], pose_cmd[1], pose_cmd[2]]
        msg.speed = 30.0
        self.data_node.motion_cmd_publisher.publish(msg)
    
    def check_pose(self,pose_cmd):
        print(f"[RUNmentFSM] 檢查Y位置: {pose_cmd[1]}")
        if abs(self.data_node.current_pose[1] - pose_cmd[1]) < 2.0:
            print("馬達已經到位置")
            return True
        else:
            print("馬達尚未到位置")
            return False

    def send_clipper_cmd(self, mode):
        msg = ClipperCmd()
        msg.mode = mode
        self.data_node.clipper_cmd_publisher.publish(msg)
        print(f"[Clipper] Published: {mode}")


def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = RUNFSM(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            # 更新狀態發布
            data.RUN_state_publisher.publish(
                TaskState(mode="RUN", state=system.state)
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