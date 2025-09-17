# -*- coding: utf-8 -*-
"""
Created on Sun Aug 10 23:14:59 2025

@author: USER
"""

import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32,Float32MultiArray,Int32MultiArray
from common_msgs.msg import GripperCmd
from pymodbus.client import ModbusTcpClient
import time
import csv

#parameters
timer_period = 0.1  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('gripper_control')

        self.mode = "open"  
       
        self.gripper_cmd_subscriber = self.create_subscription(
            GripperCmd,
            'gripper_cmd',
            self.gripper_cmd_callback,
            10
        )
        
        self.gripper_io_cmd_publisher = self.create_publisher(Int32MultiArray, 'gripper_io_cmd', 10)
        self.gripper_state_publisher = self.create_publisher(String, 'gripper_control_state', 10)


    def gripper_cmd_callback(self, msg: GripperCmd):
        self.mode = msg.mode
                
class GripperControlState(Enum):
    IDLE = "idle"
    OPENING = "opening"
    OPEN = "open"
    CLOSING = "closing"
    CLOSED = "closed"
    STOP = "stop"
    FAIL = "fail"

class GripperControl(Machine):
    def __init__(self, data_node: DataNode):

        self.phase = GripperControlState.IDLE  # 初始狀態
        self.data_node = data_node

        self.D17 = 0  # 控制right_receipt
        self.D19 = 0  # 控制right_move
        self.D20 = 0  # 控制right_stop
        
        self.D22 = 0  # 控制left_receipt
        self.D24 = 0  # 控制left_move
        self.D25 = 0  # 控制left_stop


        states = [
            GripperControlState.IDLE.value,
            GripperControlState.OPENING.value,
            GripperControlState.OPEN.value, 
            GripperControlState.CLOSING.value,
            GripperControlState.CLOSED.value,
            GripperControlState.STOP.value,
            GripperControlState.FAIL.value
        ]

        transitions = [
            # 狀態轉換
            {'trigger': 'opening', 'source': [GripperControlState.IDLE.value,GripperControlState.CLOSED.value,], 'dest': GripperControlState.OPENING.value},
            {'trigger': 'closing', 'source': [GripperControlState.IDLE.value,GripperControlState.OPEN.value], 'dest': GripperControlState.CLOSING.value},
            {'trigger': 'open_finish', 'source': GripperControlState.OPENING.value, 'dest': GripperControlState.OPEN.value},
            {'trigger': 'close_finish', 'source': GripperControlState.CLOSING.value, 'dest': GripperControlState.CLOSED.value},
            {'trigger': 'stop', 'source': '*', 'dest': GripperControlState.STOP.value},
            {'trigger': 'fail', 'source': '*', 'dest': GripperControlState.FAIL.value},
            {'trigger': 'reset', 'source': '*', 'dest': GripperControlState.IDLE.value}
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = GripperControlState(self.state)

    def step(self):
        
        self.run()
        # if self.data_node.state_cmd.get("pause_button", False):
        #     print("[ManualAlignmentFSM] 被暫停中")
        #     return  # 暫停中，不執行
        
        # if self.data_node.mode == "run":
        #     print("[ForkliftControl] 開始執行叉車控制任務")
        #     self.run()
        #     return
        
        # else:
        #     if self.state != ForkliftControlState.IDLE.value:
        #         print("[ForkliftControl] 非執行模式，強制回到 IDLE 狀態")
        #         self.return_to_idle()
        #         self.data_node.can_forklift_cmd = True  # 允許發送新的命令
        #         print(self.data_node.current_speed, self.data_node.current_direction)
        #         self.forklift_controller("slow","stop", self.data_node.current_height)  # 停止叉車
                
        #     # else:
        #     #     # print("[ForkliftControl] 叉車控制系統已經處於空閒狀態")

            # return

    def encode(self, speed, direction):
        """將速度和方向編碼為 Modbus 寫入值"""
        if speed == "fast":
            y0 = 1
            y1 = 0
        elif speed == "slow":
            y0 = 0
            y1 = 1
        elif speed == "stop":
            y0 = 0
            y1 = 0
        if direction == "up":
            y2 = 1
            y3 = 0
        elif direction == "down":
            y2 = 0
            y3 = 1
        elif direction == "stop":
            y2 = 0
            y3 = 0
        
        # value = (2**3)*y3 + (2**2)*y2 + (2**1)*y1 + (2**0)*y0
        value = [y0, y1, y2, y3]
        value = Int32MultiArray(data=value)  # 封裝為 Int32MultiArray

        return value

    def gripper_controller(self, mode):
        """控制夾爪的開啟和關閉"""
        result = 'waiting'

        if mode == "open":
            value = Int32MultiArray(data=[0, 0, 0, 0, 0, 0])  # 封裝為 Int32MultiArray
            self.data_node.gripper_io_cmd_publisher.publish(value)  #change receipt
            value = Int32MultiArray(data=[0, 1, 0, 0, 1, 0])  # 封裝為 Int32MultiArray
            self.data_node.gripper_io_cmd_publisher.publish(value)#open move
            time.sleep(1)  # 等待夾爪開啟完成
            result = 'done'

        elif mode == "close":
            value = Int32MultiArray(data=[1, 0, 0, 1, 0, 0])  # 封裝為 Int32MultiArray
            self.data_node.gripper_io_cmd_publisher.publish(value)#change receipt
            value = Int32MultiArray(data=[1, 1, 0, 1, 1, 0])  # 封裝為 Int32MultiArray
            self.data_node.gripper_io_cmd_publisher.publish(value)  #close move
            time.sleep(1)  # 等待夾爪關閉完成
            result = 'done'

        elif mode == "stop":
            value = Int32MultiArray(data=[0, 0, 1, 0, 0, 1])    # 封裝為 Int32MultiArray
            self.data_node.gripper_io_cmd_publisher.publish(value)  #stop move
            time.sleep(1)  # 等待夾爪停止完成
            result = 'done'
        
        return result
            



    def run(self):
        """執行狀態機的邏輯"""
        if self.state == GripperControlState.IDLE.value:
            print("[GripperControl] 狀態機處於空閒狀態")
            if self.data_node.mode == "open_gripper":
                print("[GripperControl] 開始開啟夾爪")
                self.opening()
            elif self.data_node.mode == "close_gripper":
                print("[GripperControl] 開始關閉夾爪")
                self.closing()
            return
        
        elif self.state == GripperControlState.OPENING.value:
            print("[GripperControl] 狀態機正在開啟夾爪")
            result = self.gripper_controller("open")
            
            if result == 'done':
                print("[GripperControl] 夾爪開啟完成")
                self.open_finish()
            else:
                print("[GripperControl] 夾爪開啟失敗")
                self.fail()
        
        elif self.state == GripperControlState.OPEN.value:
            print("[GripperControl] 狀態機處於夾爪開啟狀態")
            if self.data_node.mode == "close_gripper":
                print("[GripperControl] 開始關閉夾爪")
                self.closing()
            elif self.data_node.mode == "stop_gripper":
                print("[GripperControl] 停止夾爪操作")
                self.stop()
            return
        
        elif self.state == GripperControlState.CLOSING.value:
            print("[GripperControl] 狀態機正在關閉夾爪")
            result = self.gripper_controller("close")
            
            if result == 'done':
                print("[GripperControl] 夾爪關閉完成")
                self.close_finish()
            else:
                print("[GripperControl] 夾爪關閉失敗")
                self.fail()
        
        elif self.state == GripperControlState.CLOSED.value:
            print("[GripperControl] 狀態機處於夾爪關閉狀態")
            if self.data_node.mode == "open_gripper":
                print("[GripperControl] 開始開啟夾爪")
                self.opening()
            elif self.data_node.mode == "stop_gripper":
                print("[GripperControl] 停止夾爪操作")
                self.stop()
            return

        elif self.state == GripperControlState.STOP.value:
            print("[GripperControl] 狀態機處於停止狀態")
            result = self.gripper_controller("stop")
            if result == 'done':
                print("[GripperControl] 夾爪已停止")
                self.reset()
            else:
                print("[GripperControl] 停止夾爪失敗")
                self.fail()
        
        elif self.state == GripperControlState.FAIL.value:
            print("[GripperControl] 狀態機處於失敗狀態")
            # 在失敗狀態下，可以選擇重置或其他操作
            self.reset()
            return




def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = GripperControl(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            data.gripper_state_publisher.publish(String(data=system.state))
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