import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from common_msgs.msg import StateCmd,MotionState, MotionCmd,MultipleM
import numpy as np

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
       
        self.depth_data = [100.0,100.0]

        self.state_info = "idle"  # 狀態信息

        self.current_motor_len = [10.0, 0.0, 0.0]
       
        # 初始化 ROS2 Node
        #subscriber
        super().__init__('data_node')
        self.state_info_subscriber = self.create_subscription(
            String,
            '/state_info',
            self.state_info_callback,
            10
        )
        self.depth_data_subscriber = self.create_subscription(
            Float32MultiArray,
            "/depth_data",
            self.depth_data_callback,
            10
        )
        self.motors_info_sub = self.create_subscription(MultipleM,'/multi_motor_info',self.motors_info_callback,10)


        #publisher
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.init_state_info_publisher = self.create_publisher(String, '/init_state_info', 10)

    # def state_cmd_callback(self, msg: StateCmd):
    #     print(f"接收到狀態命令: {msg}")
    #     # 在這裡可以處理狀態命令
    #     self.state_cmds = {
    #         'pause_button': msg.pause_button,
    #     }

    def state_info_callback(self, msg: String):
        print(f"接收到狀態信息: {msg.data}")
        self.state_info = msg.data      
    
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

    def motors_info_callback(self, msg:MultipleM):
        self.current_motor_len = [msg.motor_info[0].fb_position,msg.motor_info[1].fb_position,msg.motor_info[2].fb_position]
        # print("motor_info callback",self.current_motor_len)

class InitState(Enum):
    IDLE = "idle"
    CHECK_DEPTH = "check_depth"
    INIT = "init"
    DONE = "done"
    FAIL = "fail"

class InitFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = InitState.IDLE  # 初始狀態
        self.data_node = data_node

        self.motor_init_sent = False

        states = [
            InitState.IDLE.value,
            InitState.CHECK_DEPTH.value,
            InitState.INIT.value,
            InitState.DONE.value,
            InitState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_check_depth', 'source': InitState.IDLE.value, 'dest': InitState.CHECK_DEPTH.value},
            {'trigger': 'check_depth_to_init', 'source': InitState.CHECK_DEPTH.value, 'dest': InitState.INIT.value},
            {'trigger': 'init_to_done', 'source': InitState.INIT.value, 'dest': InitState.DONE.value},
            {'trigger': 'init_to_fail', 'source': InitState.INIT.value, 'dest': InitState.FAIL.value},
            {'trigger': 'return_to_idle', 'source': '*', 'dest': InitState.IDLE.value}

        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = InitState(self.state)

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
        
        if self.data_node.state_info == "init":
            print("[Run Initialization FSM] 開始初始化任務")
            self.run()
        else:
            print("[Run Initialization FSM] 初始化任務未啟動，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        depth_threshold = 200.0 
        
        if self.state == InitState.IDLE.value:
            print("[Run Initialization FSM] 狀態為 IDLE，等待開始")
            if self.data_node.state_info == "init":
                print("[Run Initialization FSM] 狀態信息為 init，進入 CHECK_DEPTH 狀態")
                self.idle_to_check_depth()
            return
        
        if self.state == InitState.CHECK_DEPTH.value:
            print("[Run Initialization FSM] 檢查深度")
            # 檢查深度是否符合要求
            if self.data_node.depth_data[0] > depth_threshold and self.data_node.depth_data[1] > depth_threshold:
                print("[Run Initialization FSM] 深度檢查通過，進入 INIT 狀態")
                self.check_depth_to_init()
            else:
                print("waiting for depth data to be valid")
                return  # 深度不符合要求，等待下一次檢查
        
        elif self.state == InitState.INIT.value:
            print("[Run Initialization FSM] 執行初始化命令")
            if not self.motor_init_sent:
                # 發送初始化命令
                self.sent_motor_init_cmd()
                self.motor_init_sent = True  # 標記已發送初始化命令
            else:
                print("[Run Initialization FSM] 初始化命令已發送，等待完成")
                motor_init = self.check_motor_init_status()
                if motor_init:
                    print("[Run Initialization FSM] 馬達初始化完成，進入 DONE 狀態")
                    self.init_to_done()
        elif self.state == InitState.DONE.value:
            print("[Run Initialization FSM] 初始化任務完成")
            
    def sent_motor_init_cmd(self):
        """發送馬達初始化命令"""
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_HOME
        msg.pose_data = [0.0, 0.0, 0.0]
        msg.speed = 50.0
        self.data_node.motion_cmd_publisher.publish(msg)
        self.motor_init_sent = True

    def check_motor_init_status(self):
        motor_home_position = [0.0, 0.0, 0.0]  # 假設馬達初始化後的位置
        if np.allclose(self.data_node.current_motor_len, motor_home_position,atol=0.05):
            print("[Run Initialization FSM] 馬達已經回到初始化位置")
            return True
        else:
            print("[Run Initialization FSM] 馬達尚未回到初始化位置")
            return False

        



def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = InitFSM(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            data.init_state_info_publisher.publish(String(data=system.state))
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
