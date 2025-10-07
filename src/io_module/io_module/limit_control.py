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
timer_period = 0.05  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('Limit_control')

        self.mode = "idle"  
        self.left_limit_state = 3 #open=0, close=1, moving=2, idle=3
        self.right_limit_state = 3 #open=0, close=1, moving=2, idle=3
       
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
        # note!!!!! left = data[1], right = data[0]
        if len(msg.data) >= 2:
            self.left_limit_state = msg.data[0]  # 左限位狀態
            self.right_limit_state = msg.data[1]  # 右限位狀態
            print(f"更新後的左限位狀態: {self.left_limit_state}, 右限位狀態: {self.right_limit_state}")
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

        self.wait_start_time = None
        self.TIMEOUT = 15.0  # 最大等待秒數

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
            {'trigger': 'to_OPEN', 'source': LimitControlState.IDLE.value, 'dest': LimitControlState.OPEN.value},
            {'trigger': 'to_CLOSED', 'source': LimitControlState.IDLE.value, 'dest': LimitControlState.CLOSED.value},
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


    def Limit_controller(self, mode):
        """控制Limit的開啟和關閉"""
        result = 'waiting'

        if mode == "open":
            if self.data_node.left_limit_state == 0 and self.data_node.right_limit_state == 0:  # 假設 [0,0] 表示夾爪已完全開啟
                print("Limit已完全開啟")
                value = Int32MultiArray(data=[0, 0, 0, 1, 0, 0, 0, 0, 1, 0])    # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)  #stop move
                result = 'done'
            
            elif self.data_node.left_limit_state == 0 and self.data_node.right_limit_state != 0: # left open, right moving
                print("left open, right moving")
                value = Int32MultiArray(data=[0, 0, 0, 1, 0, 1, 0, 1, 0, 0])  # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)#open move
            
            elif self.data_node.left_limit_state != 0 and self.data_node.right_limit_state == 0: # left moving, right open
                print("left moving, right open")
                value = Int32MultiArray(data=[1, 0, 1, 0, 0, 0, 0, 0, 1, 0])  # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)#open move
            
            else:
                print("[LimitControl] Limit正在開啟中...")
                value = Int32MultiArray(data=[1, 0, 1, 0, 0, 1, 0, 1, 0, 0])  # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)#open move

        elif mode == "close":
            if self.data_node.left_limit_state == 1 and self.data_node.right_limit_state == 1:  # 假設 [1,1] 表示夾爪已完全關閉
                print("Limit已完全關閉")
                value = Int32MultiArray(data=[0, 0, 0, 1, 0, 0, 0, 0, 1, 0])    # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)  #stop move
                result = 'done'
            
            elif self.data_node.left_limit_state == 1 and self.data_node.right_limit_state != 1: # left close, right moving
                print("left close, right moving")
                value = Int32MultiArray(data=[0, 0, 0, 1, 0, 0, 0, 1, 0, 0])  # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)#close move
            
            elif self.data_node.left_limit_state != 1 and self.data_node.right_limit_state == 1: # left moving, right close
                print("left moving, right close")
                value = Int32MultiArray(data=[0, 0, 1, 0, 0, 0, 0, 0, 1, 0])  # 封裝為 Int32MultiArray
                # value = Int32MultiArray(data=[0, 0, 0, 1, 0, 0, 0, 0, 1, 0])   #test
                self.data_node.limit_io_cmd_publisher.publish(value)#close move
            
            else:
                print("[LimitControl] Limit正在關閉中...")
                # value = Int32MultiArray(data=[0, 0, 0, 1, 0, 0, 0, 1, 0, 0])   #test
                value = Int32MultiArray(data=[0, 0, 1, 0, 0, 0, 0, 1, 0, 0])  # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)#close move

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
            
            if self.data_node.left_limit_state == 0 and self.data_node.right_limit_state == 0: # left open, right open
                self.to_OPEN()
            
            elif self.data_node.left_limit_state == 1 and self.data_node.right_limit_state == 1: # left close, right close
                self.to_CLOSED()

            else:
                print("[LimitControl] 狀態機處於空閒狀態")

                if self.data_node.mode == "open_limit":
                    print("[LimitControl] 開始開啟Limit")
                    value = Int32MultiArray(data=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0])  # 封裝為 Int32MultiArray
                    self.data_node.limit_io_cmd_publisher.publish(value)#change receipt
                    self.opening()

                elif self.data_node.mode == "close_limit":
                    print("[LimitControl] 開始關閉Limit")
                    value = Int32MultiArray(data=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # 封裝為 Int32MultiArray
                    self.data_node.limit_io_cmd_publisher.publish(value)#change receipt
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
                self.wait_start_time = None  # 重置等待時間

            elif result == 'waiting':
                if self.wait_start_time is None:
                    self.wait_start_time = time.time()

                elapsed = time.time() - self.wait_start_time

                if self.data_node.mode == "stop_limit":
                    print("[LimitControl] 停止Limit操作")
                    self.stop()
                    self.wait_start_time = None

                elif elapsed < self.TIMEOUT:
                    print(f"[LimitControl] Limit正在開啟中... ({elapsed:.2f}s)")

                else:
                    print("[LimitControl] Limit開啟超時，操作失敗")
                    self.fail()
                    self.wait_start_time = None
        
        elif self.state == LimitControlState.OPEN.value:
            print("[LimitControl] 狀態機處於Limit開啟狀態")
            if self.data_node.mode == "close_limit":
                print("[LimitControl] 開始關閉Limit")
                value = Int32MultiArray(data=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0])  # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)#change receipt
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
                self.wait_start_time = None  # 重置等待時間

            elif result == 'waiting':
                if self.wait_start_time is None:
                    self.wait_start_time = time.time()

                elapsed = time.time() - self.wait_start_time
                if self.data_node.mode == "stop_limit":
                    print("[LimitControl] 停止Limit操作")
                    self.stop()
                    self.wait_start_time = None
                elif elapsed < self.TIMEOUT:
                    print(f"[LimitControl] Limit正在關閉中... ({elapsed:.2f}s)")
                else:
                    print("[LimitControl] Limit關閉超時，操作失敗")
                    self.fail()
                    self.wait_start_time = None
        
        elif self.state == LimitControlState.CLOSED.value:
            print("[LimitControl] 狀態機處於Limit關閉狀態")
            if self.data_node.mode == "open_limit":
                print("[LimitControl] 開始開啟Limit")
                value = Int32MultiArray(data=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0])  # 封裝為 Int32MultiArray
                self.data_node.limit_io_cmd_publisher.publish(value)#change receipt
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

                if self.data_node.mode == "open_limit":
                    print("[LimitControl] 開始開啟Limit")
                    self.reset()

                elif self.data_node.mode == "close_limit":
                    print("[LimitControl] 開始關閉Limit")
                    self.reset()
                
                else:
                    print("[LimitControl] 等待新的命令")

            else:
                print("[LimitControl] 停止Limit失敗")
                self.fail()
        
        elif self.state == LimitControlState.FAIL.value:
            print("[LimitControl] 狀態機處於失敗狀態")
            result = self.Limit_controller("stop")

            if self.data_node.mode == "reset_limit":
                print("[LimitControl] 重置狀態機到空閒狀態")
                self.reset()
            else:
                print("[LimitControl] 等待重置命令")




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