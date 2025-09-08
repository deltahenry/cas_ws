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
        self.to_done = False
        

        self.depth_data = [500.0,500.0]
        self.current_height = 0.0
        self.current_pose = [0.0, 0.0, 0.0]
        self.forkstate = "idle"

        self.target_mode = "idle" # idle,pick,assembly  


        self.pose_cmd = [0.0,0.0,0.0] #xy yaw(radian) using go_to_pose

        #mm
        self.get_detection = False
        self.compensate_x = 0.0
        self.compensate_z = 0.0

       
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

        self.compensate_pose_sub = self.create_subscription(
            Float32MultiArray,
            '/compensate_pose',
            self.compensate_pose_callback,
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
        self.compensate_state_publisher = self.create_publisher(TaskState, '/task_state_compensate', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.fork_cmd_publisher = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.laser_cmd_publisher = self.create_publisher(Int32MultiArray,'/laser_io_cmd',10)
        self.ui_pose_publisher = self.create_publisher(Float32MultiArray,'/compensate_pose_cmd',10)

        self.detection_cmd_publisher = self.create_publisher(String,'/detection_cmd',10)

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

    def compensate_pose_callback(self, msg: Float32MultiArray):
        """接收物件位置"""
        print(f"接收到物件位置: {msg.data}")
        if len(msg.data) >= 2:
            self.get_detection = True
            self.compensate_x = msg.data[0]
            self.compensate_z = msg.data[1]
        else:
            print("接收到的物件位置數據長度不足，無法更新。")

    def confirm_callback(self, msg: String):
        print(f"接收到確認命令: {msg.data}")
        if msg.data == "confirm":
            self.confirm_compensate = True
            self.to_done = False
        elif msg.data == "to_done":
            self.to_done = True
            self.confirm_compensate = False
        else:
            self.confirm_compensate = False
            self.to_done = False
            
class CompensateState(Enum):
    IDLE = "idle"
    INIT = "init"
    COMPENSATE_Z_START = "compensate_z_start"
    COMPENSATE_Z_WAIT = "compensate_z_wait"
    COMPENSATE_Z_CHECK = "compensate_z_check"
    COMPENSATE_Z = "compensate_z"
    COMPENSATE_Z_DONE = "compensate_z_done"
    COMPENSATE_X_START = "compensate_x_start"
    COMPENSATE_X_WAIT = "compensate_x_wait"
    COMPENSATE_X_CHECK = "compensate_x_check"
    COMPENSATE_X = "compensate_x"
    COMPENSATE_X_DONE = "compensate_x_done"
    COMPENSATE_YAW_START = "compensate_yaw_start"
    COMPENSATE_YAW_WAIT = "compensate_yaw_wait"
    COMPENSATE_YAW_CHECK = "compensate_yaw_check"
    COMPENSATE_YAW = "compensate_yaw"
    COMPENSATE_YAW_DONE = "compensate_yaw_done"
    COMPENSATE_CHECK_START = "compensate_check_start"
    COMPENSATE_CHECK_WAIT = "compensate_check_wait"
    COMPENSATE_CHECK = "compensate_check"
    DONE = "done"
    FAIL = "fail"

class CompensateFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = CompensateState.IDLE  # 初始狀態
        self.data_node = data_node
        self.send_compensate = False
        self.pose_cmd_to_motion = [0.0,0.0,0.0]
        self.x_cmd = 0.0
        self.y_cmd = 0.0
        self.yaw_cmd = 0.0
        self.z_cmd = 80.0


        states = [
            CompensateState.IDLE.value,
            CompensateState.INIT.value,
            CompensateState.COMPENSATE_Z_START.value,
            CompensateState.COMPENSATE_Z_WAIT.value,
            CompensateState.COMPENSATE_Z_CHECK.value,
            CompensateState.COMPENSATE_Z.value,
            CompensateState.COMPENSATE_Z_DONE.value,
            CompensateState.COMPENSATE_X_START.value,
            CompensateState.COMPENSATE_X_WAIT.value,
            CompensateState.COMPENSATE_X_CHECK.value,
            CompensateState.COMPENSATE_X.value,
            CompensateState.COMPENSATE_X_DONE.value,
            CompensateState.COMPENSATE_YAW_START.value,
            CompensateState.COMPENSATE_YAW_WAIT.value,
            CompensateState.COMPENSATE_YAW_CHECK.value,
            CompensateState.COMPENSATE_YAW.value,
            CompensateState.COMPENSATE_YAW_DONE.value,
            CompensateState.COMPENSATE_CHECK_START.value,
            CompensateState.COMPENSATE_CHECK_WAIT.value,
            CompensateState.COMPENSATE_CHECK.value,
            CompensateState.DONE.value,
            CompensateState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': CompensateState.IDLE.value, 'dest': CompensateState.INIT.value},
            {'trigger': 'init_to_compensate_z_start', 'source': CompensateState.INIT.value, 'dest': CompensateState.COMPENSATE_Z_START.value},
           
            {'trigger': 'compensate_z_start_to_compensate_z_wait', 'source': CompensateState.COMPENSATE_Z_START.value, 'dest': CompensateState.COMPENSATE_Z_WAIT.value},
            {'trigger': 'compensate_z_wait_to_compensate_z_check', 'source': CompensateState.COMPENSATE_Z_WAIT.value, 'dest': CompensateState.COMPENSATE_Z_CHECK.value},
            {'trigger': 'compensate_z_check_to_compensate_z', 'source': CompensateState.COMPENSATE_Z_CHECK.value, 'dest': CompensateState.COMPENSATE_Z.value},
            {'trigger': 'compensate_z_check_to_compensate_z_done', 'source': CompensateState.COMPENSATE_Z_CHECK.value, 'dest': CompensateState.COMPENSATE_Z_DONE.value},
            {'trigger': 'compensate_z_to_compensate_z_done', 'source': CompensateState.COMPENSATE_Z.value, 'dest': CompensateState.COMPENSATE_Z_DONE.value},
            {'trigger': 'compensate_z_done_to_compensate_x_start', 'source': CompensateState.COMPENSATE_Z_DONE.value, 'dest': CompensateState.COMPENSATE_X_START.value},

            {'trigger': 'compensate_x_start_to_compensate_x_wait', 'source': CompensateState.COMPENSATE_X_START.value, 'dest': CompensateState.COMPENSATE_X_WAIT.value},
            {'trigger': 'compensate_x_wait_to_compensate_x_check', 'source': CompensateState.COMPENSATE_X_WAIT.value, 'dest': CompensateState.COMPENSATE_X_CHECK.value},
            {'trigger': 'compensate_x_check_to_compensate_x', 'source': CompensateState.COMPENSATE_X_CHECK.value, 'dest': CompensateState.COMPENSATE_X.value},
            {'trigger': 'compensate_x_check_to_compensate_x_done', 'source': CompensateState.COMPENSATE_X_CHECK.value, 'dest': CompensateState.COMPENSATE_X_DONE.value},
            {'trigger': 'compensate_x_to_compensate_x_done', 'source': CompensateState.COMPENSATE_X.value, 'dest': CompensateState.COMPENSATE_X_DONE.value},
            {'trigger': 'compensate_x_done_to_compensate_yaw_start', 'source': CompensateState.COMPENSATE_X_DONE.value, 'dest': CompensateState.COMPENSATE_YAW_START.value},

            {'trigger': 'compensate_yaw_start_to_compensate_yaw_wait', 'source': CompensateState.COMPENSATE_YAW_START.value, 'dest': CompensateState.COMPENSATE_YAW_WAIT.value},
            {'trigger': 'compensate_yaw_wait_to_compensate_yaw_check', 'source': CompensateState.COMPENSATE_YAW_WAIT.value, 'dest': CompensateState.COMPENSATE_YAW_CHECK.value},
            {'trigger': 'compensate_yaw_check_to_compensate_yaw', 'source': CompensateState.COMPENSATE_YAW_CHECK.value, 'dest': CompensateState.COMPENSATE_YAW.value},
            {'trigger': 'compensate_yaw_check_to_compensate_yaw_done', 'source': CompensateState.COMPENSATE_YAW_CHECK.value, 'dest': CompensateState.COMPENSATE_YAW_DONE.value},
            {'trigger': 'compensate_yaw_to_compensate_yaw_done', 'source': CompensateState.COMPENSATE_YAW.value, 'dest': CompensateState.COMPENSATE_YAW_DONE.value},

            {'trigger': 'compensate_yaw_done_to_compensate_check_start', 'source': CompensateState.COMPENSATE_YAW_DONE.value, 'dest': CompensateState.COMPENSATE_CHECK_START.value},
            {'trigger': 'compensate_check_start_to_compensate_check_wait', 'source': CompensateState.COMPENSATE_CHECK_START.value, 'dest': CompensateState.COMPENSATE_CHECK_WAIT.value},
            {'trigger': 'compensate_check_wait_to_compensate_check', 'source': CompensateState.COMPENSATE_CHECK_WAIT.value, 'dest': CompensateState.COMPENSATE_CHECK.value},
            {'trigger': 'compensate_check_to_done', 'source': CompensateState.COMPENSATE_CHECK.value, 'dest': CompensateState.DONE.value},
            {'trigger': 'compensate_check_to_compensate_x_start', 'source': CompensateState.COMPENSATE_CHECK.value, 'dest': CompensateState.COMPENSATE_X_START.value},

            {'trigger': 'fail', 'source': '*', 'dest': CompensateState.FAIL.value},  
            {'trigger': 'return_to_idle', 'source': '*', 'dest': CompensateState.IDLE.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = CompensateState(self.state)

    def reset_parameters(self):
        """重置參數"""
        self.data_node.compensate_cmd = "idle"

        self.x_cmd = 0.0
        self.y_cmd = 0.0
        self.yaw_cmd = 0.0
        self.z_cmd = 80.0

        self.data_node.get_detection = False

        self.data_node.to_done = False

        self.send_compensate = False

        self.pose_cmd_to_motion = [0.0,0.0,0.0]

        self.target_height = 80.0

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
        TOL_X_MM = 1.0  # 容差值 mm
        TOL_Z_MM = 1.0  # 容差值 mm
        TOL_YAW_RAD = 0.00175  # 容差值 radian (約0.1度)  #0.8mm

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
            self.init_to_compensate_z_start()

        elif self.state == CompensateState.COMPENSATE_Z_START.value:
            print("[CompensatementFSM] 視覺檢測中...")
            self.data_node.detection_cmd_publisher.publish(String(data="start_detect"))
            self.compensate_z_start_to_compensate_z_wait()
        
        elif self.state == CompensateState.COMPENSATE_Z_WAIT.value:
            print("[CompensatementFSM] 等待視覺檢測結果...")
            if self.data_node.get_detection:
                self.compensate_z_wait_to_compensate_z_check()
            else:
                print("[CompensatementFSM] wait for detection...")
                return
        
        elif self.state == CompensateState.COMPENSATE_Z_CHECK.value:
            print("[CompensatementFSM] 視覺檢測結果確認中...")
            if abs(self.data_node.compensate_z) < TOL_Z_MM:
                print("[CompensatementFSM] Z方向補償量過小，跳過Z補償")
                self.compensate_z_check_to_compensate_z_done()
            else:
                self.compensate_z_check_to_compensate_z()

        elif self.state == CompensateState.COMPENSATE_Z.value:
            print("[CompensatementFSM] 補償Z中...")

            if self.data_node.to_done:
                self.compensate_z_to_compensate_z_done()
            else:
                z_compensate =  copy.deepcopy(self.data_node.compensate_z)
                self.x_cmd = self.data_node.current_pose[0] 
                self.y_cmd = self.data_node.current_pose[1]            
                self.yaw_cmd = self.data_node.current_pose[2] 
                self.z_cmd = self.data_node.current_height + z_compensate

                self.data_node.ui_pose_publisher.publish(Float32MultiArray(data=[self.x_cmd,self.y_cmd,self.yaw_cmd*57.2958,self.z_cmd]))

                if self.data_node.confirm_compensate:
                    if not self.send_compensate:
                        self.compensate(self.x_cmd,self.y_cmd,self.yaw_cmd,self.z_cmd)
                        print("[CompensatementFSM] 使用者確認補償，進行補償動作")
                        self.send_compensate = True
                    else:
                        if self.check_compensate_done():
                            print("[CompensatementFSM] 補償完成")
                            self.compensate_z_to_compensate_z_done()
                        else:
                            print("[CompensatementFSM] 補償中，等待完成...")

        elif self.state == CompensateState.COMPENSATE_Z_DONE.value:
            print("[CompensatementFSM] Z方向補償完成，進入X方向補償")
            self.data_node.get_detection = False
            self.data_node.to_done = False
            self.data_node.confirm_compensate = False
            self.send_compensate = False
            self.compensate_z_done_to_compensate_x_start()

        elif self.state == CompensateState.COMPENSATE_X_START.value:
            print("[CompensatementFSM] 視覺檢測中...")
            print("to_done:",self.data_node.to_done)
            self.data_node.detection_cmd_publisher.publish(String(data="start_detect"))
            self.compensate_x_start_to_compensate_x_wait()
        
        elif self.state == CompensateState.COMPENSATE_X_WAIT.value:
            print("[CompensatementFSM] 等待視覺檢測結果...")
            print("to_done:",self.data_node.to_done)
            if self.data_node.get_detection:
                self.compensate_x_wait_to_compensate_x_check()
            else:
                print("[CompensatementFSM] wait for detection...")
                return
            
        elif self.state == CompensateState.COMPENSATE_X_CHECK.value:
            print("[CompensatementFSM] 視覺檢測結果確認中...")
            print("to_done:",self.data_node.to_done)
            if abs(self.data_node.compensate_x) < TOL_X_MM:
                print("[CompensatementFSM] X方向補償量過小，跳過X補償")
                self.compensate_x_check_to_compensate_x_done()
            else:
                self.compensate_x_check_to_compensate_x()
        
        elif self.state == CompensateState.COMPENSATE_X.value:
            print("[CompensatementFSM] 補償X中...")
            print("to_done:",self.data_node.to_done)

            if self.data_node.to_done:
                print('[CompensatementFSM] 使用者跳過X補償')
                self.compensate_x_to_compensate_x_done()
            else:
                x_compensate =  copy.deepcopy(self.data_node.compensate_x)
                self.x_cmd = self.data_node.current_pose[0] + x_compensate
                self.y_cmd = self.data_node.current_pose[1]            
                self.yaw_cmd = self.data_node.current_pose[2] 
                self.z_cmd = self.data_node.current_height 

                self.data_node.ui_pose_publisher.publish(Float32MultiArray(data=[self.x_cmd,self.y_cmd,self.yaw_cmd*57.2958,self.z_cmd]))

                if self.data_node.confirm_compensate:
                    if not self.send_compensate:
                        self.compensate(self.x_cmd,self.y_cmd,self.yaw_cmd,self.z_cmd)
                        print("[CompensatementFSM] 使用者確認補償，進行補償動作")
                        self.send_compensate = True
                    else:
                        if self.check_compensate_done():
                            print("[CompensatementFSM] 補償完成")
                            self.compensate_x_to_compensate_x_done()
                        else:
                            print("[CompensatementFSM] 補償中，等待完成...")
        
        elif self.state == CompensateState.COMPENSATE_X_DONE.value:
            print("[CompensatementFSM] X方向補償完成，進入YAW方向補償")
            self.data_node.get_detection = False
            self.data_node.to_done = False
            self.data_node.confirm_compensate = False
            self.send_compensate = False
            self.compensate_x_done_to_compensate_yaw_start()
        
        elif self.state == CompensateState.COMPENSATE_YAW_START.value:
            print("[CompensatementFSM] 視覺檢測中...")
            self.data_node.detection_cmd_publisher.publish(String(data="start_detect"))
            self.compensate_yaw_start_to_compensate_yaw_wait()
        
        elif self.state == CompensateState.COMPENSATE_YAW_WAIT.value:
            self.compensate_yaw_wait_to_compensate_yaw_check()
        
        elif self.state == CompensateState.COMPENSATE_YAW_CHECK.value:
            left = self.data_node.depth_data[0]
            right = self.data_node.depth_data[1]
            yaw_compensate = math.atan2((right - left), 480.0)

            if abs(left-right) > 30.0:  # >3.5 deg
                print("[CompensatementFSM] 深度感測器數值異常，跳過YAW補償")
                self.fail()

            elif abs(yaw_compensate) < TOL_YAW_RAD:
                print("[CompensatementFSM] YAW方向補償量過小，跳過YAW補償")
                self.compensate_yaw_check_to_compensate_yaw_done()
            
            else:
                self.compensate_yaw_check_to_compensate_yaw()
        
        elif self.state == CompensateState.COMPENSATE_YAW.value:
            print("[CompensatementFSM] 補償YAW中...")

            if self.data_node.to_done:
                self.compensate_yaw_to_compensate_yaw_done()
            else:
                left = self.data_node.depth_data[0]
                right = self.data_node.depth_data[1]
                yaw_compensate = math.atan2((right - left), 480.0) #480 = sensor width

                self.x_cmd = self.data_node.current_pose[0] + 880.0*0.2*yaw_compensate  
                self.y_cmd = self.data_node.current_pose[1]
                self.yaw_cmd = self.data_node.current_pose[2] + 0.3*yaw_compensate  #radian
                self.z_cmd = self.data_node.current_height
                
                print("left:",left," right:",right)
                print("yaw_mea:", f"{math.degrees(-yaw_compensate):.2f}")
                print("current yaw:", f"{math.degrees(self.data_node.current_pose[2]):.2f}")
                yaw_cmd_wo = self.data_node.current_pose[2] + yaw_compensate
                print("yaw_cmd_wo_modify:",yaw_cmd_wo)
                print(" yaw_cmd:",self.yaw_cmd)

                self.data_node.ui_pose_publisher.publish(Float32MultiArray(data=[self.x_cmd,self.y_cmd,self.yaw_cmd*57.2958,self.z_cmd]))

                self.data_node.confirm_compensate = True

                if self.data_node.confirm_compensate:
                    if not self.send_compensate:
                        print("send yaw:",self.yaw_cmd)
                        self.compensate(self.x_cmd,self.y_cmd,self.yaw_cmd,self.z_cmd)
                        print("[CompensatementFSM] 使用者確認補償，進行補償動作")
                        self.send_compensate = True
                    else:
                        if self.check_compensate_done():
                            print("[CompensatementFSM] 補償完成")
                            self.compensate_yaw_to_compensate_yaw_done()
                        else:
                            print("[CompensatementFSM] 補償中，等待完成...")
        
        elif self.state == CompensateState.COMPENSATE_YAW_DONE.value:
            print("[CompensatementFSM] YAW方向補償完成，進入補償結果確認")
            self.data_node.get_detection = False
            self.data_node.to_done = False
            self.data_node.confirm_compensate = False
            self.send_compensate = False
            self.compensate_yaw_done_to_compensate_check_start()
        
        elif self.state == CompensateState.COMPENSATE_CHECK_START.value:
            print("[CompensatementFSM] 視覺檢測中...")
            self.data_node.detection_cmd_publisher.publish(String(data="start_detect"))
            self.compensate_check_start_to_compensate_check_wait()
        
        elif self.state == CompensateState.COMPENSATE_CHECK_WAIT.value:
            print("[CompensatementFSM] 等待視覺檢測結果...")
            if self.data_node.get_detection:
                self.compensate_check_wait_to_compensate_check()
            else:
                print("[CompensatementFSM] wait for detection...")
                return
        
        elif self.state == CompensateState.COMPENSATE_CHECK.value:
            x_compensate =  copy.deepcopy(self.data_node.compensate_x)
            z_compensate =  copy.deepcopy(self.data_node.compensate_z)
            left = self.data_node.depth_data[0]
            right = self.data_node.depth_data[1]
            yaw_compensate = math.atan2((right - left), 480.0)
            print(f"[CompensatementFSM] 最終補償量: X={x_compensate} mm, Z={z_compensate} mm, YAW={yaw_compensate*57.2958} 度")

            if abs(x_compensate) < TOL_X_MM and abs(z_compensate) < TOL_Z_MM and abs(yaw_compensate) < TOL_YAW_RAD:
                print("[CompensatementFSM] 最終補償量過小，補償完成")
                self.compensate_check_to_done()
            else:
                print("[CompensatementFSM] 最終補償量過大，進行補償")
                self.compensate_check_to_compensate_x_start()
                      
        elif self.state == CompensateState.DONE.value:
            print("[CompensatementFSM] 補償完成!")

        elif self.state == CompensateState.FAIL.value:
            print("[CompensatementFSM] 補償失敗，請重新嘗試!")
            if self.data_node.to_done:
                print("[CompensatementFSM] 使用者跳過補償失敗，進入下一步")
                self.data_node.get_detection = False
                self.data_node.to_done = False
                self.data_node.confirm_compensate = False
                self.send_compensate = False
                self.return_to_idle()
            else:
                print("[CompensatementFSM] 補償失敗，等待使用者操作...")


    def compensate(self,x_cmd,y_cmd,yaw_cmd,z_cmd):
        print(f"[CompensatementFSM] 開始補償動作")
        self.pose_cmd_to_motion = [x_cmd,y_cmd,yaw_cmd]
        self.sent_motor_cmd(self.pose_cmd_to_motion)

        self.target_height = float(z_cmd)
        self.fork_cmd("run", 'slow', "down", self.target_height)

    def check_compensate_done(self):
        tolerance = 1.0  # 容差值 mm
        if not self.send_compensate:
            return False
        if not self.check_pose(self.pose_cmd_to_motion):
            print("[CompensatementFSM] 等待馬達到位...")
            return False
        if abs(self.data_node.current_height - self.target_height) > tolerance or self.data_node.forkstate != "idle":
            print("[CompensatementFSM] 馬達到位，等待叉車下降...")
            return False
        print("[CompensatementFSM] 補償完成，進入下一步")
        return True

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
        self.data_node.motion_cmd_publisher.publish(msg)
    
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
            data.compensate_state_publisher.publish(
                TaskState(mode="compensate", state=system.state)
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
