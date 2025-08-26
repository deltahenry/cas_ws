import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String,Float32MultiArray, Int32MultiArray, Int32
from common_msgs.msg import StateCmd,TaskCmd, MotionCmd,MotionState,TaskState,ForkCmd,ForkState,Recipe,CurrentPose
import copy

#parameters
timer_period = 0.1  # seconds

# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmd ={
            'pause_button': False,
        }

        self.compensate_cmd = "idle"  # 'l_shape','screw'
        self.confirm_compensate = False
        

        self.depth_data = [500.0,500.0]
        self.current_height = 0.0
        self.current_pose = [0.0, 0.0, 0.0]
        self.forkstate = "idle"

        self.target_mode = "idle" # idle,pick,assembly  


        self.pose_cmd = [0.0,0.0,0.0] #xy yaw(radian) using go_to_pose

        #mm
        self.obiect_pose_x = 0.0 
        self.obiect_pose_z = 0.0

        self.golden_pose_x = 258.0 
        self.golden_pose_z = 465.0

       
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
            '/compensate_cmd',
            self.compensate_cmd_callback,
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

        self.object_pose_sub = self.create_subscription(
            Pose,
            '/object_camera_pose',
            self.object_pose_callback,
            10
        )

        self.current_pose_subscriber = self.create_subscription(    
            CurrentPose,
            'current_pose',
            self.current_pose_callback,
            10
        )

        self.confirm_compensate_subscriber = self.create_subscription(String, '/confirm_cmd', self.confirm_callback, 10)
        
        #publisher
        self.rough_align_state_publisher = self.create_publisher(TaskState, '/task_state_rough_align', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.detection_cmd_publisher = self.create_publisher(String,'/lshape/cmd',10)
        self.fork_cmd_publisher = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.laser_cmd_publisher = self.create_publisher(Int32MultiArray,'/laser_io_cmd',10)
        self.pose_pub = self.create_publisher(MotionCmd, "/current_pose_cmd", 10)

    def state_cmd_callback(self, msg: StateCmd):
        print(f"接收到狀態命令: {msg}")
        # 在這裡可以處理狀態命令
        self.state_cmd = {
            'pause_button': msg.pause_button,
        }

    def compensate_cmd_callback(self, msg: TaskCmd):
        print(f"接收到任務命令: {msg.mode}")
        # 在這裡可以處理任務命令
        self.compensate_cmd = msg.mode

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
        
    def current_pose_callback(self, msg: CurrentPose):
        """接收當前機器人位置"""
        self.get_logger().info(f"Received current pose: {msg.pose_data}")
        self.current_pose[0] = float(msg.pose_data[0])
        self.current_pose[1] = float(msg.pose_data[1])
        self.current_pose[2] = float(msg.pose_data[2])

    def object_pose_callback(self, msg: Pose):
        print(f"接收到目標物體姿態: {msg}")
        self.obiect_pose_x = msg.position.x * 1000.0  # m to mm
        self.obiect_pose_z = msg.position.z * 1000.0  # m to mm


    def confirm_callback(self, msg: String):
        print(f"接收到確認命令: {msg.data}")
        if msg.data == "confirm":
            self.confirm_compensate = True
        else:
            self.confirm_compensate = False
            
class CompensateState(Enum):
    IDLE = "idle"
    INIT = "init"
    ROUGH_COMPENSATE = "rough_compensate"
    DEPTH_DETECT = "depth_detect"
    YAW_COMPENSATE = "yaw_compensate"
    VISION_DETECT = "vision_detect"
    XZ_COMPENSATE = "XZ_compensate"
    DONE = "done"
    FAIL = "fail"

class CompensateFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = CompensateState.IDLE  # 初始狀態
        self.data_node = data_node
        self.run_mode = "pick"
        self.send_fork_cmd = False  # 用於控制叉車命令的發送
        self.motor_cmd_sent = False
        self.pose_cmd_to_motion = [0.0,0.0,0.0]


        states = [
            CompensateState.IDLE.value,
            CompensateState.INIT.value,
            CompensateState.ROUGH_COMPENSATE.value,
            CompensateState.DEPTH_DETECT.value,
            CompensateState.YAW_COMPENSATE.value,
            CompensateState.VISION_DETECT.value,
            CompensateState.XZ_COMPENSATE.value,
            CompensateState.DONE.value,
            CompensateState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': CompensateState.IDLE.value, 'dest': CompensateState.INIT.value},
            {'trigger': 'init_to_rough_compensate', 'source': CompensateState.INIT.value, 'dest': CompensateState.ROUGH_COMPENSATE.value},
            {'trigger': 'rough_compensate_to_depth_detect', 'source': CompensateState.ROUGH_COMPENSATE.value, 'dest': CompensateState.DEPTH_DETECT.value},
            {'trigger': 'depth_detect_to_yaw_compensate', 'source': CompensateState.DEPTH_DETECT.value, 'dest': CompensateState.YAW_COMPENSATE.value},
            # {'trigger': 'depth_detect_to_vision_detect', 'source': CompensateState.DEPTH_DETECT.value, 'dest': CompensateState.VISION_DETECT.value},
            # {'trigger': 'yaw_compensate_check', 'source': CompensateState.YAW_COMPENSATE.value, 'dest': CompensateState.DEPTH_DETECT.value},
            {'trigger': 'yaw_compensate_to_vision_detect', 'source': CompensateState.YAW_COMPENSATE.value, 'dest': CompensateState.VISION_DETECT.value},
            {'trigger': 'vision_detect_to_xz_compensate', 'source': CompensateState.VISION_DETECT.value, 'dest': CompensateState.XZ_COMPENSATE.value},
            {'trigger': 'xz_compensate_to_done', 'source': CompensateState.XZ_COMPENSATE.value, 'dest': CompensateState.DONE.value},
            {'trigger': 'done_to_idle', 'source': CompensateState.DONE.value, 'dest': CompensateState.IDLE.value},
            {'trigger': 'fail', 'source': '*', 'dest': CompensateState.FAIL.value},  
            {'trigger': 'return_to_idle', 'source': '*', 'dest': CompensateState.IDLE.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = CompensateState(self.state)

    def reset_parameters(self):
        """重置參數"""
       
        self.motor_cmd_sent = False
        self.data_node.compensate_cmd = "idle"
        self.send_fork_cmd = False  # 用於控制叉車命令的發送

        self.data_node.state_cmd = {
            'pause_button': False,
        }

        
    def step(self):
        if self.data_node.state_cmd.get("pause_button", False):
            print("[CompensatementFSM] 被暫停中")
        
        elif self.data_node.compensate_cmd == "l_shape" or self.data_node.compensate_cmd == "screw":
            print("[CompensatementFSM] 開始compensate")
            self.run()
        else:
            print("[CompensatementFSM] compensate未啟動，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        print(self.data_node.confirm_compensate)
        """FSM的主循環"""
        if self.state == CompensateState.IDLE.value:
            # print("[CompensatementFSM] 空閒中...")
            # 在這裡可以添加空閒邏輯
            if self.data_node.compensate_cmd in ["l_shape", "screw"]:
                self.idle_to_init()
            else:
                print("[CompensatementFSM] 空閒中...")
                return
        
        elif self.state == CompensateState.INIT.value:
            print("[CompensatementFSM] 初始化中...")
            # 在這裡可以添加初始化邏輯
            self.init_to_rough_compensate()

        elif self.state == CompensateState.ROUGH_COMPENSATE.value:
            print("[CompensatementFSM] 粗略補償中...")
            x_compensate =  self.data_node.obiect_pose_x - self.data_node.golden_pose_x
            z_compensate =  self.data_node.obiect_pose_z - self.data_node.golden_pose_z
            
            x_cmd = self.data_node.current_pose[0] + x_compensate
            y_cmd = self.data_node.current_pose[1]            
            yaw_cmd = self.data_node.current_pose[2]
            z_cmd = self.data_node.current_height + z_compensate

            print("x_cmd",x_cmd)
            print("y_cmd",y_cmd)
            print("yaw_cmd",yaw_cmd)
            print("z_cmd",z_cmd)

            if self.data_node.confirm_compensate:
                print("[CompensatementFSM] 使用者確認補償，進行補償動作")
                self.rough_compensate_to_depth_detect()
                self.data_node.confirm_compensate = False  # 重置確認標誌
            else:
                print("[CompensatementFSM] 等待使用者確認補償...")
                
        elif self.state == CompensateState.DEPTH_DETECT.value:
            print("[CompensatementFSM] 深度檢測中...")
            self.depth_detect_to_yaw_compensate()
            # left_depth = copy.deepcopy(self.data_node.depth_data[0])
            # right_depth = copy.deepcopy(self.data_node.depth_data[1])
            # yaw_compensate = math.atan2((right_depth - left_depth), 480.0)  # 480mm是兩雷射間距

            # # 在這裡可以添加深度檢測邏輯
            # if self.data_node.depth_data[0] > 1000.0 or self.data_node.depth_data[1] > 1000.0:
            #     print("too far,waiting")
            # else:
            #     if abs(yaw_compensate) > 0.005  and abs(yaw_compensate) < 0.174:
            #         self.depth_detect_to_yaw_compensate()
            #     else:
            #         print("yaw angle small,skip to vision detect")
            #         self.depth_detect_to_vision_detect()
        
        elif self.state == CompensateState.YAW_COMPENSATE.value:
            left_depth = copy.deepcopy(self.data_node.depth_data[0])
            right_depth = copy.deepcopy(self.data_node.depth_data[1])
            yaw_compensate = math.atan2((right_depth - left_depth), 480.0)  # 480mm是兩雷射間距
            print(f"[CompensatementFSM] 偏航補償角度 (deg): {yaw_compensate*57.2958}")

            x_cmd = self.data_node.current_pose[0]
            y_cmd = self.data_node.current_pose[1]            
            yaw_cmd = self.data_node.current_pose[2] + yaw_compensate
            z_cmd = self.data_node.current_height

            print("x_cmd",x_cmd)
            print("y_cmd",y_cmd)
            print("yaw_cmd(deg)",yaw_cmd*57.2958)
            print("z_cmd",z_cmd)

            #for ui display
            # pose_cmd_to_ui = copy.deepcopy(self.data_node.current_pose) #xy yaw(radian)
            # pose_cmd_to_ui[2]  = pose_cmd_to_ui[2] + yaw_compensate
            # msg = MotionCmd()
            # msg.command_type = MotionCmd.TYPE_GOTO
            # msg.pose_data = [pose_cmd_to_ui[0], pose_cmd_to_ui[1], pose_cmd_to_ui[2]]
            # msg.speed = 10.0
            # self.data_node.pose_pub.publish(msg)
            
            # 等待使用者確認補償
            if not self.data_node.confirm_compensate:
                print("[CompensatementFSM] 等待使用者確認補償...")
                return
            else:
                print("[CompensatementFSM] 使用者確認補償，進行補償動作")
                self.data_node.confirm_compensate = False  # 重置確認標誌
                self.yaw_compensate_to_vision_detect()

            # if self.data_node.confirm_compensate:
            #     print("[CompensatementFSM] 使用者確認補償，進行補償動作")
            #     if not self.motor_cmd_sent:
            #         self.pose_cmd_to_motion = copy.deepcopy(self.data_node.current_pose) #xy yaw(radian)
            #         self.pose_cmd_to_motion[2]  = pose_cmd_to_ui[2] + yaw_compensate
            #         self.sent_motor_cmd(self.pose_cmd_to_motion)
            #         self.motor_cmd_sent = True
            #     else:
            #         if self.check_pose(self.pose_cmd_to_motion):
            #             self.yaw_compensate_check()
            #             self.motor_cmd_sent = False
            #             self.data_node.confirm_compensate = False  # 重置確認標誌
            #         else:
            #             print("[CompensatementFSM] 等待馬達到位...")
            # else:
            #     print("[CompensatementFSM] 等待使用者確認補償...")
               
        elif self.state == CompensateState.VISION_DETECT.value:
            self.vision_detect_to_xz_compensate()
        
        elif self.state == CompensateState.XZ_COMPENSATE.value:
            x_compensate =  self.data_node.obiect_pose_x - self.data_node.golden_pose_x
            z_compensate =  self.data_node.obiect_pose_z - self.data_node.golden_pose_z
            
            x_cmd = self.data_node.current_pose[0] + x_compensate
            y_cmd = self.data_node.current_pose[1]            
            yaw_cmd = self.data_node.current_pose[2]
            z_cmd = self.data_node.current_height + z_compensate

            print("x_cmd",x_cmd)
            print("y_cmd",y_cmd)
            print("yaw_cmd",yaw_cmd)
            print("z_cmd",z_cmd)

            if self.data_node.confirm_compensate:
                print("[CompensatementFSM] 使用者確認補償，進行補償動作")
                self.xz_compensate_to_done()
                self.data_node.confirm_compensate = False  # 重置確認標誌
            else:
                print("[CompensatementFSM] 等待使用者確認補償...")

        elif self.state == CompensateState.DONE.value:
            print("[CompensatementFSM] 補償完成!")
            self.data_node.compensate_cmd = "idle"
            self.return_to_idle()
        
    def pose_to_cmd(self,pose: Pose):
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
            
    def sent_motor_cmd(self,pose_cmd):
        """發送馬達初始化命令"""
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_GOTO
        msg.pose_data = [pose_cmd[0], pose_cmd[1], pose_cmd[2]]
        msg.speed = 10.0
        # self.data_node.motion_cmd_publisher.publish(msg)
    
    def check_pose(self,pose_cmd):
        """檢查馬達是否到位"""
        print(f"檢查馬達位置: 目標 {pose_cmd}, 當前 {self.data_node.current_pose}")  

        if np.allclose(self.data_node.current_pose, pose_cmd, atol=0.05):
            print("馬達已經到位置")
            return True
        else:
            print("馬達尚未到位置")
            return False

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
