import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32
from common_msgs.msg import StateCmd,TaskCmd,MotionCmd,TaskState,Recipe
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

        self.target_mode = "idle"  # 目標模式，初始為空閒
     
        self.rough_align_state = "idle"  # rough align task state
        self.precise_align_state = "idle"  # precise align task state
        self.pick_state = "idle"  # pick task state
        self.assembly_state = "idle"  # assembly task state
       
        # 初始化 ROS2 Node
        #subscriber
        super().__init__('data_node')

        self.state_cmd_subscriber = self.create_subscription(
            StateCmd,
            '/state_cmd',
            self.state_cmd_callback,
            10
        )

        self.rough_align_state_subscriber = self.create_subscription(
            TaskState,
            '/task_state_rough_align',
            self.task_state_rough_align_callback,
            10
        )

        self.precise_align_state_subscriber = self.create_subscription(
            TaskState,
            '/task_state_precise_align',
            self.task_state_precise_align_callback,
            10
        )

        self.pick_state_subscriber = self.create_subscription(
            TaskState,
            '/task_state_pick',
            self.task_state_pick_callback,
            10
        )

        self.assembly_state_subscriber = self.create_subscription(
            TaskState,
            '/task_state_assembly',
            self.task_state_assembly_callback,
            10
        )

        self.recipe_subscriber = self.create_subscription(
            Recipe,
            'recipe_data',
            self.recipe_callback,
            10
        )

        self.run_state_publisher = self.create_publisher(
            TaskState,
            '/task_state_run',
            10
        )

        self.task_cmd_publisher = self.create_publisher(
            TaskCmd,
            '/task_cmd',
            10
        )

    def state_cmd_callback(self, msg: StateCmd):
        print(f"接收到狀態命令: {msg}")
        # 在這裡可以處理狀態命令
        self.state_cmd = {
            'init_button': msg.init_button,
            'run_button': msg.run_button,
            'pause_button': msg.pause_button,
            'stop_button': msg.stop_button,
        }

    def task_state_rough_align_callback(self, msg: TaskState):
        print(f"接收到粗對齊任務狀態: {msg}")
        # 在這裡可以處理粗對齊任務狀態
        self.rough_align_state = msg.state  

    def task_state_precise_align_callback(self, msg: TaskState):
        print(f"接收到精細對齊任務狀態: {msg}")
        # 在這裡可以處理精細對齊任務狀態
        self.precise_align_state = msg.state
    
    def task_state_pick_callback(self, msg: TaskState):
        print(f"接收到拾取任務狀態: {msg}")
        # 在這裡可以處理拾取任務狀態
        self.pick_state = msg.state
    
    def task_state_assembly_callback(self, msg: TaskState):
        print(f"接收到組裝任務狀態: {msg}")
        # 在這裡可以處理組裝任務狀態
        self.assembly_state = msg.state
    
    def recipe_callback(self, msg: Recipe):
        self.target_mode = msg.mode

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
            {'trigger': 'init_to_rough_align', 'source': RUNState.INIT.value, 'dest': RUNState.ROUGH_ALIGN.value},
            {'trigger': 'rough_align_to_precise_align', 'source': RUNState.ROUGH_ALIGN.value, 'dest': RUNState.PRECISE_ALIGN.value},
            {'trigger': 'precise_align_to_pick', 'source': RUNState.PRECISE_ALIGN.value, 'dest': RUNState.PICK.value},
            {'trigger': 'pick_to_assembly', 'source': RUNState.PICK.value, 'dest': RUNState.ASSEMBLY.value},
            {'trigger': 'assembly_to_done', 'source': RUNState.ASSEMBLY.value, 'dest': RUNState.DONE.value},
            {'trigger': 'precise_align_to_assembly', 'source': RUNState.PRECISE_ALIGN.value, 'dest': RUNState.ASSEMBLY.value},
            {'trigger': 'assembly_to_pick', 'source': RUNState.ASSEMBLY.value, 'dest': RUNState.PICK.value},
            {'trigger': 'pick_to_done', 'source': RUNState.PICK.value, 'dest': RUNState.DONE.value},
            {'trigger': 'done_to_idle', 'source': RUNState.DONE.value, 'dest': RUNState.IDLE.value},
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
        self.data_node.state_cmd = {
            'init_button': False,
            'run_button': False,
            'pause_button': False,
            'stop_button': False,
        }
        self.data_node.task_cmd = "idle"  # 重置任務命令


    def step(self):
        if self.data_node.state_cmd.get("pause_button", False):
            print("[RUNmentFSM] 被暫停中")

        elif self.data_node.state_cmd.get("stop_button", False):
            print("[RUNmentFSM] 被停止中")
            self.send_task_cmd("idle")  # 發送任務命令回到空閒狀態
            self.reset_parameters()
            self.return_to_idle()  # 返回到空閒狀態
        
        elif self.data_node.state_cmd.get("run_button", False):
            print("[RUNmentFSM] 被啟動中")
            self.run()
        else:
            print("[RUNmentFSM] 等待啟動")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):
        print(self.data_node.target_mode)

        if self.state == RUNState.IDLE.value:
            print("[RUNmentFSM] 等待啟動")
            if self.data_node.state_cmd.get('run_button', False):
                self.idle_to_init()
                print("[RUNmentFSM] 狀態轉換到 INIT")
            else:
                print("[RUNmentFSM] 等待啟動")
        
        elif self.state == RUNState.INIT.value:
            print("[RUNmentFSM] 初始化階段")

            print("[RUNmentFSM] 發送任務命令: rough_align")
            self.send_task_cmd("rough_align")

            print("[RUNmentFSM] 狀態轉換到 ROUGH_ALIGN")
            self.init_to_rough_align()

        elif self.state == RUNState.ROUGH_ALIGN.value:
            print("[RUNmentFSM] 粗對齊階段")

            if self.data_node.rough_align_state == "done":
                print("[RUNmentFSM] 粗對齊任務完成")
                print("[RUNmentFSM] 發送任務命令: precise_align")
                self.send_task_cmd("precise_align")
                print("[RUNmentFSM] 狀態轉換到 PRECISE_ALIGN")
                self.rough_align_to_precise_align()
            elif self.data_node.rough_align_state == "fail":
                print("[RUNmentFSM] 粗對齊任務失敗")
                self.fail()
            else:
                print("[RUNmentFSM] 等待粗對齊任務完成")

        elif self.state == RUNState.PRECISE_ALIGN.value:
            print("[RUNmentFSM] 精細對齊階段")

            if self.data_node.precise_align_state == "done":
                print("[RUNmentFSM] 精細對齊任務完成")
                if self.data_node.target_mode == "assembly":
                    print("[RUNmentFSM] 目標模式為組裝，直接跳過拾取階段")
                    self.send_task_cmd("assembly")
                    self.precise_align_to_assembly()
                elif self.data_node.target_mode == "pick":
                    print("[RUNmentFSM] 目標模式為拾取，進入拾取階段")
                    self.send_task_cmd("pick")
                    self.precise_align_to_pick()
                else:
                    print("[RUNmentFSM] 未知目標模式，無法進行精細對齊後的操作")
                    self.fail()
            elif self.data_node.precise_align_state == "fail":
                print("[RUNmentFSM] 精細對齊任務失敗")
                self.fail()
            else:
                print("[RUNmentFSM] 等待精細對齊任務完成")
        
        elif self.state == RUNState.PICK.value:
            print("[RUNmentFSM] 拾取階段")


            if self.data_node.pick_state == "done":
                print("[RUNmentFSM] 拾取任務完成")
                print("[RUNmentFSM] 發送任務命令: done")
                self.send_task_cmd("done")
                print("[RUNmentFSM] 狀態轉換到 DONE")
                self.pick_to_done()
            elif self.data_node.pick_state == "fail":
                print("[RUNmentFSM] 拾取任務失敗")
                self.fail()
            else:
                print("[RUNmentFSM] 等待拾取任務完成")
        
        elif self.state == RUNState.ASSEMBLY.value:
            print("[RUNmentFSM] 組裝階段")


            if self.data_node.assembly_state == "done":
                print("[RUNmentFSM] 組裝任務完成")
                print("[RUNmentFSM] 發送任務命令: done")
                self.send_task_cmd("done")
                print("[RUNmentFSM] 狀態轉換到 DONE")
                self.assembly_to_done()
            elif self.data_node.assembly_state == "fail":
                print("[RUNmentFSM] 組裝任務失敗")
                self.fail()
            else:
                print("[RUNmentFSM] 等待組裝任務完成")
        
        elif self.state == RUNState.DONE.value:
            print("[RUNmentFSM] 任務完成")
            # 在這裡可以添加任務完成後的處理邏輯
        
        elif self.state == RUNState.FAIL.value:
            print("[RUNmentFSM] 任務失敗")
            # 在這裡可以添加任務失敗後的處理邏輯
            print("[RUNmentFSM] 發送任務命令: idle")
            self.send_task_cmd("idle")
            print("[RUNmentFSM] 狀態轉換到 IDLE")
            self.return_to_idle()

    def send_task_cmd(self, task):
        """發送任務命令"""
        msg = TaskCmd()
        msg.mode = task
        self.data_node.task_cmd_publisher.publish(msg)
        print(f"[RUNmentFSM] 發送任務命令: {task}")



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
            data.run_state_publisher.publish(
                TaskState(mode="run", state=system.state)
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