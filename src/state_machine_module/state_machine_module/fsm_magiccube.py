import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_msgs.msg import ButtonCommand, MotionState, MotionCmd

#parameters
timer_period = 0.5  # seconds



# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        #init button_cmd
        self.button_cmds = {
            'stop_button': False,
            'init_button': False,
            'reselect_button': False,
            'pull_button': False,
            'push_button': False,
            'debug_button': False
        }

        #init motion_state
        self.motion_states = {
            'motion_finish': False,
            'init_finish': False,
            'pull_finish': False,                 
            'push_finish': False,
            'rough_pos_finish': False,
            'auto_pos_finish': False,
            'system_error': False
        }

        # 初始化 ROS2 Node
        #subscriber
        super().__init__('data_node')
        self.button_cmd_subscriber = self.create_subscription(
            ButtonCommand,
            "/button_cmd",
            self.button_cmd_callback,
            10
        )
        self.motion_state_subscriber = self.create_subscription(
            MotionState,
            "/motion_state",
            self.motion_state_callback,
            10
        )

        #publisher
        self.button_cmd_publisher = self.create_publisher(ButtonCommand, '/button_cmd', 10)
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)

    def button_cmd_callback(self, msg=ButtonCommand):
        print(f"接收到按鈕命令: {msg}")
        self.button_cmds = {
            'stop_button': msg.stop_button,           
            'init_button': msg.init_button,
            'reselect_button': msg.reselect_button, 
            'pull_button': msg.pull_button,
            'push_button': msg.push_button,
            'debug_button': msg.debug_button
        }
    
    def motion_state_callback(self, msg=MotionState):
        print(f"接收到運動狀態: {msg}")
        # 在這裡可以處理運動狀態
        self.motion_states = {
            'motion_finish': msg.motion_finish,
            'init_finish': msg.init_finish,
            'pull_finish': msg.pull_finish,                 
            'push_finish': msg.push_finish,
            'rough_pos_finish': msg.rough_pos_finish,
            'auto_pos_finish': msg.auto_pos_finish,
            'system_error': msg.system_error
        }

    def rewrite_button_cmd(self,button_name,value):
        if button_name in self.button_cmds:
            self.button_cmds[button_name] = value
            self.button_cmd_publisher.publish(
                ButtonCommand(
                    stop_button=self.button_cmds['stop_button'],
                    init_button=self.button_cmds['init_button'],
                    reselect_button=self.button_cmds['reselect_button'],
                    pull_button=self.button_cmds['pull_button'],
                    push_button=self.button_cmds['push_button'],
                    debug_button=self.button_cmds['debug_button']
                )
            )
        else:
            print(f"按鈕名稱 {button_name} 不存在。")

    def rewrite_motion_state(self,motion_name,value):
        if motion_name in self.motion_states:
            self.motion_states[motion_name] = value
            self.motion_state_publisher.publish(
                MotionState(
                    motion_finish=self.motion_states['motion_finish'],
                    init_finish=self.motion_states['init_finish'],
                    pull_finish=self.motion_states['pull_finish'],                 
                    push_finish=self.motion_states['push_finish'],
                    rough_pos_finish=self.motion_states['rough_pos_finish'],
                    auto_pos_finish=self.motion_states['auto_pos_finish'],
                    system_error=self.motion_states['system_error']
                )
            )
        else:
            print(f"運動狀態名稱 {motion_name} 不存在。")

    def publish_motion_cmd(self, command: str,pose_data=[345.0,0.0,0.0], speed=0.5):
        msg = MotionCmd()
        if command == 'home':
            msg.command_type = MotionCmd.TYPE_HOME
            self.motion_cmd_publisher.publish(msg)

        elif command == 'goto':
            msg.command_type = MotionCmd.TYPE_GOTO
            msg.pose_data = pose_data 
            self.motion_cmd_publisher.publish(msg)
            
        elif command == 'goto_relative':
            msg.command_type = MotionCmd.TYPE_GOTO_RELATIVE

        else:
            print(f"未知的運動命令: {command}")
            return
        print(f"發佈運動命令: {command}")

class ConnectState(Enum):
    IDLE = 'idle'
    RUN = 'run'
    DONE = 'done'
    FAIL = 'fail'

class InitState(Enum):
    IDLE = 'idle'
    RUN = 'run'
    DONE = 'done'
    FAIL = 'fail'

class FSMState(Enum):
    START = 'start'
    CONNECT = 'connect'
    INIT = 'init'
    IDLE = 'idle'
    WARNING = 'warning'
    ERROR = 'error'

class InitStateMachine(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = InitState.IDLE
        self.data_node = data_node

        states = [
            InitState.IDLE.value,
            InitState.RUN.value,
            InitState.DONE.value,
            InitState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_run', 'source': InitState.IDLE.value, 'dest': InitState.RUN.value},
            {'trigger': 'run_to_done', 'source': InitState.RUN.value, 'dest': InitState.DONE.value},
            {'trigger': 'run_to_fail', 'source': InitState.RUN.value, 'dest': InitState.FAIL.value},
            {'trigger': 'reset_to_idle', 'source': '*', 'dest': InitState.IDLE.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = InitState(self.state)

    def run(self):
        if self.phase == InitState.IDLE:
            self.data_node.publish_motion_cmd(command='home')  # 發佈初始化命令
            print("publish init command...")
            self.idle_to_run()
            return InitState.IDLE.value
        
        elif self.phase == InitState.RUN:
            # 模擬初始化過程
            if self.data_node.motion_states["init_finish"]:
                self.run_to_done()  # 模擬初始化完成
            else:
                print("正在初始化...")
            return InitState.RUN.value
        
        elif self.phase == InitState.DONE:
            print("已經完成初始化。")
            self.reset_to_idle()  # 重置狀態機以便下次使用
            return InitState.DONE.value
        
        elif self.phase == InitState.FAIL:
            print("初始化失敗，請檢查系統。")
            self.reset_to_idle()  # 重置狀態機以便下次使用
            return InitState.FAIL.value

class FSMStateMachine(Machine):
    def __init__(self, data_node: DataNode):
        
        self.phase = FSMState.START
        self.data_node = data_node
        self.init_state_machine = InitStateMachine(data_node)

        states = [
            FSMState.START.value,   
            FSMState.CONNECT.value,
            FSMState.INIT.value,
            FSMState.IDLE.value,
            FSMState.WARNING.value,
            FSMState.ERROR.value
        ]
        transitions = [
            {'trigger': 'start_to_connect', 'source': FSMState.START.value, 'dest': FSMState.CONNECT.value},
            {'trigger': 'connect_to_init', 'source': FSMState.CONNECT.value, 'dest': FSMState.INIT.value},
            {'trigger': 'init_to_idle', 'source': FSMState.INIT.value, 'dest': FSMState.IDLE.value},
            {'trigger': 'init_to_start', 'source': FSMState.INIT.value, 'dest': FSMState.START.value},
            {'trigger': 'warning', 'source': '*', 'dest': FSMState.WARNING.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = FSMState(self.state)

def fsm_logic(system: FSMStateMachine,data: DataNode):
    if system.state == 'start':
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        else:
            print("🚀 [FSM] 系統啟動，觸發 start_to_connect")
            system.start_to_connect()
    
    elif system.state == 'connect':     
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        else:
            print("🔗 [FSM] 連接中，觸發 connect_to_init")
            system.connect_to_init()

    elif system.state == 'init':
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        else:
            states = system.init_state_machine.run()
            if states == InitState.DONE.value:
                print("✅ [FSM] 初始化完成，觸發 init_to_idle")
                system.init_to_start()
            elif states == InitState.FAIL.value:
                print("❌ [FSM] 初始化失敗，請檢查系統。")
                system.warning()
            else:
                print(f"🔧 [FSM] 初始化狀態: {states}")

        

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
