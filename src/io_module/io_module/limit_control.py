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
from common_msgs.msg import LimitCmd
from pymodbus.client import ModbusTcpClient
import time
import csv

#parameters
timer_period = 0.1  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('Limit_control')

        self.mode = "idle"  
        self.limit_state = [0,0] # 初始狀態 [left, right] open=0, close=1, moving=2
       
        self.limit_cmd_subscriber = self.create_subscription(
            LimitCmd,
            'limit_cmd',
            self.limit_cmd_callback,
            10
        )
        self.limit_state_subscriber = self.create_subscription(
            Int32MultiArray,
            'limit_state',
            self.limit_state_callback,
            10
        )
        
        self.limit_io_cmd_publisher = self.create_publisher(Int32MultiArray, 'limit_io_cmd', 10)
        self.limit_state_publisher = self.create_publisher(String, 'limit_control_state', 10)


    def limit_cmd_callback(self, msg: LimitCmd):
        self.mode = msg.mode

    def limit_state_callback(self, msg: Int32MultiArray):
        print(f"接收到限位狀態: {msg.data}")
        if len(msg.data) >= 2:
            self.limit_state[0] = msg.data[0]  # 左限位狀態
            self.limit_state[1] = msg.data[1]  # 右限位狀態
        else:
            self.get_logger().warn("接收到的限位狀態長度不足，無法更新。")
                
class LimitControlState(Enum):
    IDLE = "idle"
    OPENING = "opening"
    OPEN = "open"
    CLOSING = "closing"
    CLOSED = "closed"
    STOP = "stop"
    FAIL = "fail"

class LimitControl(Machine):
    def __init__(self, data_node: DataNode):

        self.phase = LimitControlState.IDLE  # 初始狀態
        self.data_node = data_node

        states = [
            LimitControlState.IDLE.value,
            LimitControlState.OPENING.value,
            LimitControlState.OPEN.value, 
            LimitControlState.CLOSING.value,
            LimitControlState.CLOSED.value,
            LimitControlState.STOP.value,
            LimitControlState.FAIL.value
        ]

        transitions = [
            # 狀態轉換
            {'trigger': 'opening', 'source': [LimitControlState.IDLE.value,LimitControlState.CLOSED.value,], 'dest': LimitControlState.OPENING.value},
            {'trigger': 'closing', 'source': [LimitControlState.IDLE.value,LimitControlState.OPEN.value], 'dest': LimitControlState.CLOSING.value},
            {'trigger': 'open_finish', 'source': LimitControlState.OPENING.value, 'dest': LimitControlState.OPEN.value},
            {'trigger': 'close_finish', 'source': LimitControlState.CLOSING.value, 'dest': LimitControlState.CLOSED.value},
            {'trigger': 'stop', 'source': '*', 'dest': LimitControlState.STOP.value},
            {'trigger': 'fail', 'source': '*', 'dest': LimitControlState.FAIL.value},
            {'trigger': 'reset', 'source': '*', 'dest': LimitControlState.IDLE.value}
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = LimitControlState(self.state)

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

            # return夾爪


    def Limit_controller(self, mode):
        """控制Limit的開啟和關閉"""
        result = 'waiting'

        if mode == "open":
            value = Int32MultiArray(data=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0])  # 封裝為 Int32MultiArray
            self.data_node.limit_io_cmd_publisher.publish(value)  #change receipt
            value = Int32MultiArray(data=[1, 0, 1, 0, 0, 1, 0, 1, 0, 0])  # 封裝為 Int32MultiArray
            self.data_node.limit_io_cmd_publisher.publish(value)#open move
            
            if self.data_node.limit_state == [0,0]:  # 假設 [0,0] 表示夾爪已完全開啟
                result = 'done'
            else:
                print(f"[LimitControl] 目前限位狀態: {self.data_node.limit_state}, 繼續等待開啟完成...")
                result = 'waiting'  # 繼續等待夾爪開啟完成

        elif mode == "close":
            value = Int32MultiArray(data=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # 封裝為 Int32MultiArray
            self.data_node.limit_io_cmd_publisher.publish(value)#change receipt
            value = Int32MultiArray(data=[0, 0, 1, 0, 0, 0, 0, 1, 0, 0])  # 封裝為 Int32MultiArray
            self.data_node.limit_io_cmd_publisher.publish(value)  #close move

            if self.data_node.limit_state == [1,1]:  # 假設 [1,1] 表示夾爪已完全關閉
                result = 'done'
            else:
                print(f"[LimitControl] 目前限位狀態: {self.data_node.limit_state}, 繼續等待關閉完成...")
                result = 'waiting'  # 繼續等待夾爪關閉完成

        elif mode == "stop":
            value = Int32MultiArray(data=[0, 0, 0, 1, 0, 0, 0, 0, 1, 0])    # 封裝為 Int32MultiArray
            self.data_node.limit_io_cmd_publisher.publish(value)  #stop move
            value = Int32MultiArray(data=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])    # 封裝為 Int32MultiArray
            self.data_node.limit_io_cmd_publisher.publish(value)  #stop move
            time.sleep(1)  # 等待Limit停止完成
            result = 'done'
        
        return result
            



    def run(self):
        """執行狀態機的邏輯"""
        if self.state == LimitControlState.IDLE.value:
            print("[LimitControl] 狀態機處於空閒狀態")
            if self.data_node.mode == "open_limit":
                print("[LimitControl] 開始開啟Limit")
                self.opening()
            elif self.data_node.mode == "close_limit":
                print("[LimitControl] 開始關閉Limit")
                self.closing()
            else:
                print("[LimitControl] 保持空閒狀態")
            return
        
        elif self.state == LimitControlState.OPENING.value:
            print("[LimitControl] 狀態機正在開啟Limit")
            result = self.Limit_controller("open")
            
            if result == 'done':
                print("[LimitControl] Limit開啟完成")
                self.open_finish()

            elif result == 'waiting':
                if self.data_node.mode == "stop_limit":
                    print("[LimitControl] 停止Limit操作")
                    self.stop()
                else:
                    print("[LimitControl] Limit正在開啟中...")

            else:
                print("[LimitControl] Limit開啟失敗")
                self.fail()
        
        elif self.state == LimitControlState.OPEN.value:
            print("[LimitControl] 狀態機處於Limit開啟狀態")
            if self.data_node.mode == "close_limit":
                print("[LimitControl] 開始關閉Limit")
                self.closing()
            elif self.data_node.mode == "stop_limit":
                print("[LimitControl] 停止Limit操作")
                self.stop()
            return
        
        elif self.state == LimitControlState.CLOSING.value:
            print("[LimitControl] 狀態機正在關閉Limit")
            result = self.Limit_controller("close")
            
            if result == 'done':
                print("[LimitControl] Limit關閉完成")
                self.close_finish()
            
            elif result == 'waiting':
                if self.data_node.mode == "stop_limit":
                    print("[LimitControl] 停止Limit操作")
                    self.stop()
                else:
                    print("[LimitControl] Limit正在關閉中...")

            else:
                print("[LimitControl] Limit關閉失敗")
                self.fail()
        
        elif self.state == LimitControlState.CLOSED.value:
            print("[LimitControl] 狀態機處於Limit關閉狀態")
            if self.data_node.mode == "open_limit":
                print("[LimitControl] 開始開啟Limit")
                self.opening()
            elif self.data_node.mode == "stop_limit":
                print("[LimitControl] 停止Limit操作")
                self.stop()
            return

        elif self.state == LimitControlState.STOP.value:
            print("[LimitControl] 狀態機處於停止狀態")
            result = self.Limit_controller("stop")
            if result == 'done':
                print("[LimitControl] Limit已停止")
                self.reset()
            else:
                print("[LimitControl] 停止Limit失敗")
                self.fail()
        
        elif self.state == LimitControlState.FAIL.value:
            print("[LimitControl] 狀態機處於失敗狀態")
            # 在失敗狀態下，可以選擇重置或其他操作
            self.reset()
            return




def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = LimitControl(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            data.limit_state_publisher.publish(String(data=system.state))
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