import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32
from common_msgs.msg import StateCmd,TaskState,MotionCmd,ForkCmd,GripperCmd,LimitCmd
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
        # 初始化 ROS2 Node
        #subscriber
        super().__init__('data_node')

        self.state_cmd_subscriber = self.create_subscription(
            StateCmd,
            'state_cmd',
            self.state_cmd_callback,
            10)
        
        self.motion_pub = self.create_publisher(MotionCmd, 'motion_cmd', 10)
        self.fork_pub = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.gripper_pub = self.create_publisher(GripperCmd, 'gripper_cmd', 10)
        self.limit_pub = self.create_publisher(LimitCmd, 'limit_cmd', 10)
        

    def state_cmd_callback(self, msg: StateCmd):
        print(f"接收到狀態命令: {msg}")
        # 在這裡可以處理狀態命令
        self.state_cmd = {
            'init_button': msg.init_button,
            'run_button': msg.run_button,
            'pause_button': msg.pause_button,
            'stop_button': msg.stop_button,
        }
    
class STOPState(Enum):
    IDLE = "idle"
    STOP = "stop"

class STOPFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = STOPState.IDLE
        self.data_node = data_node


        states = [
            STOPState.IDLE.value,
            STOPState.STOP.value,
        ]
        
        transitions = [
            {'trigger': 'return_to_idle', 'source': '*', 'dest': STOPState.IDLE.value},
            {'trigger': 'stop', 'source': '*', 'dest': STOPState.STOP.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = STOPState(self.state)
        print(f"[STOPmentFSM] 狀態更新為: {self.phase}")

    def reset_parameters(self):
        """重置參數"""
        self.data_node.state_cmd = {
            'init_button': False,
            'run_button': False,
            'pause_button': False,
            'stop_button': False,
        }


    def step(self):

        if self.data_node.state_cmd.get("stop_button", False):
            self.run()
            print("[RUNmentFSM] 停止按鈕被按下，進入 STOP 狀態")
        
        else:
            print("[RUNmentFSM] 等待啟動")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        if self.state == STOPState.IDLE.value:
            print("[STOPmentFSM] 空閒狀態")
            # 在空閒狀態下等待啟動命令
            if self.data_node.state_cmd.get("stop_button", False):
                print("[STOPmentFSM] 收到啟動命令，進入 STOP 狀態")
                self.stop()
        
        elif self.state == STOPState.STOP.value:
            print("[STOPmentFSM] 停止狀態")
            # 在停止狀態下執行停止動作
            self.motor_stop()
            self.fork_stop()
            self.gripper_stop()
            self.limit_stop()
            print("[STOPmentFSM] 已發送停止命令")
            # time.sleep(1)  # 停止動作持續一段時間
            # print("[STOPmentFSM] 停止動作完成，返回空閒狀態")
            # self.reset_parameters()  # 重置參數
            # self.return_to_idle()  # 返回到空閒狀態
            
    def motor_stop(self):
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_STOP
        msg.pose_data = [0.0, 0.0, 0.0]  # X, Y, Z, Yaw(deg)
        msg.speed = 0.0
        self.data_node.motion_pub.publish(msg)
    
    def fork_stop(self):
        msg = ForkCmd()
        msg.mode = "stop"
        msg.speed = "slow"
        msg.direction = "stop"
        msg.distance = 0.0
        self.data_node.fork_pub.publish(msg)
    
    def gripper_stop(self):
        msg = GripperCmd()
        msg.mode = "stop_gripper"
        self.data_node.gripper_pub.publish(msg)
    
    def limit_stop(self):
        msg = LimitCmd()
        msg.mode = "stop_limit"
        self.data_node.limit_pub.publish(msg)
    


def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = STOPFSM(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            # # 更新狀態發布
            # data.run_state_publisher.publish(
            #     TaskState(mode="run", state=system.state)
            # )
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