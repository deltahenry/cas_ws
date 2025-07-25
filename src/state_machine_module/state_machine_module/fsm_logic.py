import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_msgs.msg import StateCmd, MotionState, MotionCmd

#parameters
timer_period = 0.5  # seconds



# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.mode_cmd = {
            'auto': True,
            'manual': False,
        }
        self.state_cmd = {
            'run': False,
            'pause': False,
            'stop': False,
            'reselect': False,
            'init_finished': True,  # 用於檢查 init 是否完成
        }
        self.task_cmd = {
            'connect': False,
            'init': False,            
            'rough_align': False,
            'precise_align': False,
            'pick': False,
            'assem': False,
        }

        # 初始化 ROS2 Node
        super().__init__('data_node')
        #subscriber
        self.mode_cmd_subscriber = self.create_subscription(
            String,
            '/mode_cmd',
            self.mode_cmd_callback,
            10
        )
        self.state_cmd_subscriber = self.create_subscription(
            StateCmd,
            '/state_cmd',
            self.state_cmd_callback,
            10
        )
        self.task_trigger_subscriber = self.create_subscription(
            String,
            '/task_trigger',
            self.task_trigger_callback,
            10
        )
        self.init_state_info_subscriber = self.create_subscription(
            String,
            '/init_state_info',
            self.init_state_info_callback,
            10
        )

        #publisher
        self.state_info_publisher = self.create_publisher(String, '/state_info', 10)
    
    def mode_cmd_callback(self, msg: String):
        print(f"接收到模式命令: {msg.data}")
        # 僅當 init 完成後才能進入 manual 模式
        if msg.data == "manual":
            if self.state_cmd.get('init_finished', False):  
                self.mode_cmd['auto'] = False
                self.mode_cmd['manual'] = True
                print("[模式切換] 切換為 manual 模式")
            else:
                print("[警告] 初始化尚未完成，無法切換為 manual 模式")
        
        elif msg.data == "auto":
            self.mode_cmd['auto'] = True
            self.mode_cmd['manual'] = False
            print("[模式切換] 切換為 auto 模式")

    def state_cmd_callback(self, msg: StateCmd):
        print(f"接收到狀態命令: {msg}")
        # 在這裡可以處理狀態命令
        self.state_cmd = {
            'run': msg.run_button,
            'pause': msg.pause_button,
        }

    def task_trigger_callback(self, msg: String):
        print(f"接收到任務觸發: {msg.data}")
        # 在這裡可以處理任務觸發邏輯
        if msg.data == "connect":
            self.task_cmd['connect'] = True
        elif msg.data == "init":
            self.task_cmd['init'] = True
        elif msg.data == "rough_align":
            self.task_cmd['rough_align'] = True
        elif msg.data == "precise_align":
            self.task_cmd['precise_align'] = True
        elif msg.data == "pick":
            self.task_cmd['pick'] = True
        elif msg.data == "assem":
            self.task_cmd['assem'] = True

    def init_state_info_callback(self, msg: String):
        print(f"接收到初始化狀態信息: {msg.data}")
        # 在這裡可以處理初始化狀態信息
        if msg.data == "done":
            self.state_cmd['init_finished'] = True
            print("[初始化] 初始化任務已完成")
        else:
            self.state_cmd['init_finished'] = False
            print("[初始化] 初始化任務尚未完成")

class FSMState(Enum):
    START = 'start'
    CONNECT = 'connect'
    INIT = 'init'
    IDLE = 'idle'
    ROUGH_ALIGN = 'rough_align'
    PRECISE_ALIGN = 'precise_align'
    PICK = 'pick'
    ASSEMBLE = 'assemble'
    ALARM = 'alarm'

class FSMStateMachine(Machine):
    def __init__(self, data_node: DataNode):
        
        self.phase = FSMState.START
        self.node = data_node

        self.fsm_flags = {
            "waiting_run": False,
            "pause_mode": False
        }

        self.legal_tasks_by_state = {
            'start': ['connect'],
            'connect': ['init'],
            'init': [],
            'idle': ['rough_align', 'precise_align', 'pick', 'assem'],            
            'rough_align': ['precise_align'],
            'precise_align': ['pick', 'assem'],
        }

        states = [
            FSMState.START.value,
            FSMState.CONNECT.value,
            FSMState.INIT.value,
            FSMState.IDLE.value,
            FSMState.ROUGH_ALIGN.value,
            FSMState.PRECISE_ALIGN.value,
            FSMState.PICK.value,
            FSMState.ASSEMBLE.value,
            FSMState.ALARM.value
        ]
        
        transitions = [
            {'trigger': 'start_to_connect', 'source': FSMState.START.value, 'dest': FSMState.CONNECT.value},
            {'trigger': 'connect_to_init', 'source': FSMState.CONNECT.value, 'dest': FSMState.INIT.value},
            {'trigger': 'init_to_idle', 'source': FSMState.INIT.value, 'dest': FSMState.IDLE.value},
            {'trigger': 'idle_to_rough_align', 'source': FSMState.IDLE.value, 'dest': FSMState.ROUGH_ALIGN.value},
            {'trigger': 'rough_align_to_precise_align', 'source': FSMState.ROUGH_ALIGN.value, 'dest': FSMState.PRECISE_ALIGN.value},
            {'trigger': 'precise_align_to_pick', 'source': FSMState.PRECISE_ALIGN.value, 'dest': FSMState.PICK.value},
            {'trigger': 'pick_to_assemble', 'source': FSMState.PICK.value, 'dest': FSMState.ASSEMBLE.value},
            {'trigger': 'reset_to_start', 'source': '*', 'dest': FSMState.START.value},
            {'trigger': 'alarm', 'source': '*', 'dest': FSMState.ALARM.value},
            {'trigger': 'idle_to_precise_align', 'source': FSMState.IDLE.value, 'dest': FSMState.PRECISE_ALIGN.value},
            {'trigger': 'idle_to_pick', 'source': FSMState.IDLE.value, 'dest': FSMState.PICK.value},    
            {'trigger': 'idle_to_assemble', 'source': FSMState.IDLE.value, 'dest': FSMState.ASSEMBLE.value},
            {'trigger': 'return_to_idle', 'source': '*', 'dest': FSMState.IDLE.value},
            
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = FSMState(self.state)
    
    def clear_illegal_tasks(self, current_state):
        legal_tasks = self.legal_tasks_by_state.get(current_state, [])
        for task in self.node.task_cmd:
            if self.node.task_cmd[task] and task not in legal_tasks:
                print(f"[FSM] 狀態 {current_state} 不允許任務 {task}，清除")
                self.node.task_cmd[task] = False

def fsm_logic(system: FSMStateMachine, data: DataNode):
    system_alarm = False
    # 全局錯誤判斷
    if system_alarm and not system.state == 'alarm':
        print("[FSM] 系統錯誤，觸發 alarm")
        system.alarm()
        return
    
    # Manual Mode 重新選擇按鈕處理
    if data.mode_cmd.get("manual",False) and data.state_cmd.get("pause", False):
        # 如果是 manual 模式且按下了重新選擇按鈕
        print("[FSM] Manual 模式下觸發 Reselect，重置任務回到 Idle")
        system.return_to_idle()  # 或是 system.manual_align_to_idle() 視 FSM 結構而定

    # # 全局 pause 判斷
    # if data.state_cmd.get("pause", False):
    #     if not system.fsm_flags.get("pause_mode", False):
    #         print("[FSM] 被暫停，進入 Pause 模式")
    #         system.fsm_flags["pause_mode"] = True
    #     else:
    #         print("[FSM] 已經在 Pause 模式中，等待解除")
    #     return

    # if system.fsm_flags.get("pause_mode", False):
    #     if not data.state_cmd.get("run_button", False):
    #         print("[FSM] Pause 模式中，尚未接收到 Run")
    #         return
    #     else:
    #         print("[FSM] 接收到 Run，解除 Pause 模式")
    #         system.fsm_flags["pause_mode"] = False

    # ======================== FSM 狀態邏輯 ========================
    if system.state == 'start':
        print("[FSM] 系統啟動，觸發 start_to_connect")
        system.clear_illegal_tasks('start')
        system.start_to_connect()

    elif system.state == 'connect':
        print("[FSM] 連接中...")
        system.clear_illegal_tasks('connect')
        connect_states = True  # TODO: 真實檢查條件
        if connect_states:
            print("[FSM] 連接成功，等待初始化按鈕")
            if data.task_cmd.get("init", False):
                print("[FSM] 接收到初始化按鈕，觸發 connect_to_init")
                system.connect_to_init()
            else:
                print("[FSM] 等待初始化按鈕被按下")
        else:
            print("[FSM] 連接失敗，保持在 connect 狀態")

    elif system.state == 'init':
        print("[FSM] 初始化中...")
        system.clear_illegal_tasks('init')
        if data.state_cmd.get("init_finished", False):
            print("[FSM] 初始化完成，觸發 init_to_idle")
            system.init_to_idle()
        else:
            print("[FSM] 初始化未完成，保持在 init 狀態")

    elif system.state == 'idle':
        system.clear_illegal_tasks('idle')
        if data.mode_cmd.get("auto",False):
            print("[FSM] 自動模式，等待使用者按下 Run")
            if data.state_cmd.get('run', False):
                print("[FSM] 接收到 Run，觸發 idle_to_rough_align")
                system.idle_to_rough_align()
            else:
                print("[FSM] 等待 Run 按鈕被按下")
        else:
            print("[FSM] 手動模式，等待使用者按下按鈕")
            data.task_cmd['rough_align'] = True  # test
            if data.task_cmd.get('rough_align', False):
                print("[FSM] 接收到手動對齊任務，觸發 idle_to_rough_align")
                system.idle_to_rough_align()
            elif data.task_cmd.get('precise_align', False):
                print("[FSM] 接收到手動精確對齊任務，觸發 idle_to_precise_align")
                system.idle_to_precise_align()
            elif data.task_cmd.get('pick', False):
                print("[FSM] 接收到手動拾取任務，觸發 idle_to_pick")
                system.idle_to_pick()
            elif data.task_cmd.get('assem', False):
                print("[FSM] 接收到手動組裝任務，觸發 idle_to_assemble")
                system.idle_to_assemble()
            else:
                print("[FSM] 等待手動對齊任務被觸發")

    elif system.state == 'rough_align':
        system.clear_illegal_tasks('rough_align')
        print("[FSM] 粗對齊中...")

        
        # system.manual_alignment_to_auto_alignment()

    # elif system.state == 'auto_alignment':
    #     print("[FSM] 自動對位完成，觸發 auto_alignment_to_auto_pick")
    #     system.auto_alignment_to_auto_pick()

    # elif system.state == 'auto_pick':
    #     print("[FSM] 自動拾取完成，觸發 auto_pick_to_auto_assemble")
    #     system.auto_pick_to_auto_assemble()

    # elif system.state == 'auto_assemble':
    #     print("[FSM] 自動組裝完成，觸發 auto_assemble_to_idle")
    #     system.auto_assemble_to_idle()

    elif system.state == 'alarm':
        if not data.motion_states.get("system_error", False):
            print("[FSM] 系統錯誤已解決，觸發 reset_to_start")
            system.reset_to_start()
        else:
            print("[FSM] 系統仍有錯誤，保持在 alarm 狀態")



        

def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = FSMStateMachine(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            fsm_logic(system,data)
            print(f"[現在狀態] {system.state}")
            publish_msg = String()
            publish_msg.data = system.state
            system.node.state_info_publisher.publish(publish_msg)
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
