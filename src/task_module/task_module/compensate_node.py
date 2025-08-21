import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String,Float32MultiArray, Int32MultiArray, Int32
from common_msgs.msg import StateCmd,TaskCmd, MotionCmd,MotionState,TaskState,ForkCmd,ForkState,Recipe

#parameters
timer_period = 0.5  # seconds

# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmd ={
            'pause_button': False,
        }

        self.task_cmd = "idle"  # rough align,precise align,pick,assembly
        self.depth_data = [500.0,500.0]
        self.current_height = 0.0
        self.forkstate = "idle"

        self.target_mode = "idle" # idle,pick,assembly  

        self.target_pose = Pose()  # 初始化目標姿態
        self.target_pose.position.x = 0.0
        self.target_pose.position.y = 0.0
        self.target_pose.position.z = 0.0
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = 0.0
        self.target_pose.orientation.w = 1.0

        self.pose_cmd = [0.0,0.0,0.0] #xy yaw(radian) using go_to_pose
       
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
            '/fork_state',
            self.fork_state_callback,
            10
        )

        self.recipe_data_subscriber = self.create_subscription(
            Recipe,
            'recipe_data',
            self.recipe_callback,
            10
        )

        self.target_pose_subscriber = self.create_subscription(
            Pose,
            '/target_arm_pose',
            self.target_pose_callback,
            10
        )


        #publisher
        self.rough_align_state_publisher = self.create_publisher(TaskState, '/task_state_rough_align', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.detection_cmd_publisher = self.create_publisher(String,'/lshape/cmd',10)
        self.fork_cmd_publisher = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.laser_cmd_publisher = self.create_publisher(Int32MultiArray,'/laser_io_cmd',10)

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
            print("接收到的深度數據長度不足，無法更新。")

    def height_info_callback(self,msg: Int32):
        """接收來自LR Sensor的高度信息"""
        print(f"接收到高度信息: {msg.data} mm")
        self.current_height = msg.data
    
    def fork_state_callback(self, msg: ForkState):
        """接收叉車狀態"""
        self.forkstate = msg.state  # 假設 ForkState 有個 .state 屬性

    def recipe_callback(self, msg: Recipe):
        self.target_mode = msg.mode
        # 在這裡可以添加更多的處理邏輯
        # 例如，根據接收到的 recipe 更新其他狀態或觸發其他操作

    def target_pose_callback(self, msg: Pose):
        self.target_pose.position.x = msg.position.x
        self.target_pose.position.y = msg.position.y
        self.target_pose.position.z = msg.position.z
        self.target_pose.orientation.x = msg.orientation.x
        self.target_pose.orientation.y = msg.orientation.y
        self.target_pose.orientation.z = msg.orientation.z
        self.target_pose.orientation.w = msg.orientation.w
        

class CompensateState(Enum):
    IDLE = "idle"
    INIT = "init"
    DETECT = "detect"
    GET_COMPENSATE = "get_compensate"
    MOVE_MOTOR = "move_motor"
    MOVE_FORKLIFT = "move_forklift"
    DONE = "done"
    FAIL = "fail"

class CompensateFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = CompensateState.IDLE  # 初始狀態
        self.data_node = data_node
        self.run_mode = "pick"
        self.send_fork_cmd = False  # 用於控制叉車命令的發送

        states = [
            CompensateState.IDLE.value,
            CompensateState.INIT.value,
            CompensateState.DETECT.value,
            CompensateState.GET_COMPENSATE.value,
            CompensateState.MOVE_MOTOR.value,
            CompensateState.MOVE_FORKLIFT.value,
            CompensateState.DONE.value,
            CompensateState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': CompensateState.IDLE.value, 'dest': CompensateState.INIT.value},
            {'trigger': 'init_to_detection', 'source': CompensateState.INIT.value, 'dest': CompensateState.DETECT.value},
            {'trigger': 'dect_to_get_compensate', 'source': CompensateState.DETECT.value, 'dest': CompensateState.GET_COMPENSATE.value},
            {'trigger': 'get_compensate_to_move_motor', 'source': CompensateState.GET_COMPENSATE.value, 'dest': CompensateState.MOVE_MOTOR.value},
            {'trigger': 'move_motor_to_move_forklift', 'source': CompensateState.MOVE_MOTOR.value, 'dest': CompensateState.MOVE_FORKLIFT.value},
            {'trigger': 'move_forklift_to_done', 'source': CompensateState.MOVE_FORKLIFT.value, 'dest': CompensateState.DONE.value},
            {'trigger': 'fail', 'source': '*', 'dest': CompensateState.FAIL.value},  
            {'trigger': 'return_to_idle', 'source': '*', 'dest': CompensateState.IDLE.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = CompensateState(self.state)

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
            'push_button': True
        }
        self.send_fork_cmd = False  # 用於控制叉車命令的發送
        
    def step(self):
        if self.data_node.state_cmd.get("pause_button", False):
            print("[CompensatementFSM] 被暫停中")
        
        elif self.data_node.task_cmd == "rough_align":
            print("[CompensatementFSM] 開始手動對齊任務")
            self.run()
        else:
            print("[CompensatementFSM] 手動對齊任務未啟動，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        """FSM的主循環"""
        if self.state == CompensateState.IDLE.value:
            self.idle_to_init()
        
        elif self.state == CompensateState.INIT.value:
            print("[CompensatementFSM] 初始化中...")
            # 在這裡可以添加初始化邏輯
            self.init_to_detection()

        elif self.state == CompensateState.DETECT.value:
            print("[CompensatementFSM] 偵測中...")
            # 在這裡可以添加偵測邏輯
            self.dect_to_get_compensate

        elif self.state == CompensateState.GET_COMPENSATE.value:
            #translate the target pose to the robot's coordinate system
            print("[CompensatementFSM] 獲取補償位置中...")
            self.data_node.pose_cmd = self.pose_to_cmd(self.data_node.target_pose)
        
    def pose_to_cmd(pose: Pose):
        # Extract XY position
        x = pose.position.x
        y = pose.position.y

        # Convert quaternion → yaw (Z axis rotation)
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # Yaw (rotation about Z) from quaternion
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        return [x, y, yaw]  
            
    def laser_cmd(self, cmd: str):
        """發送雷射命令"""
        if cmd == "laser_open":
            value = [1,1]
            value = Int32MultiArray(data=value)  # 封裝為 Int32MultiArray
            self.data_node.laser_cmd_publisher.publish(value)
        elif cmd == "laser_close":
            value = [0,0]
            value = Int32MultiArray(data=value)  # 封裝為 Int32MultiArray
            self.data_node.laser_cmd_publisher.publish(value)
        print(f"[CompensatementFSM] 發送雷射命令: {cmd}")

    def fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.data_node.fork_cmd_publisher.publish(msg)
        print(f"[CompensatementFSM] 發送叉車命令: mode={mode}, speed={speed}, direction={direction}, distance={distance}")

def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = CompensateFSM(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            # 更新狀態發布
            data.rough_align_state_publisher.publish(
                TaskState(mode="rough_align", state=system.state)
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
