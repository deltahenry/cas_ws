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

        #init button_cmd
        self.state_cmd = {
            'init_button': False,
            'run_button': False,   
            'pause_button': False,
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
        self.state_cmd_subscriber = self.create_subscription(
            StateCmd,
            "/state_cmd",
            self.state_cmd_callback,
            10
        )

        #publisher
        self.state_info_publisher = self.create_publisher(StateCmd, '/state_cmd', 10)
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)

    def state_cmd_callback(self, msg:StateCmd):
        print(f"接收到按鈕命令: {msg}")
        self.state_cmd = {
            'init_button': msg.init_button,
            'run_button': msg.run_button,   
            'pause_button': msg.pause_button,
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

    def publish_motion_cmd(self, command: str,pose_data, speed):
        msg = MotionCmd()
        if command == 'home':
            msg.command_type = MotionCmd.TYPE_HOME
            self.motion_cmd_publisher.publish(msg)

        elif command == 'goto':
            msg.command_type = MotionCmd.TYPE_GOTO
            msg.pose_data = pose_data 
            msg.speed = speed
            self.motion_cmd_publisher.publish(msg)
            
        elif command == 'goto_relative':
            msg.command_type = MotionCmd.TYPE_GOTO_RELATIVE

        else:
            print(f"未知的運動命令: {command}")
            return
        print(f"發佈運動命令: {command}")


class FSMState(Enum):
    START = 'start'
    CONNECT = 'connect'
    INIT = 'init'
    IDLE = 'idle'
    Manual_Alignment = 'manual_alignment'
    Auto_Alignment = 'auto_alignment'
    Auto_Pick = 'auto_pick'
    Auto_Assemble = 'auto_assemble'
    Alarm = 'alarm'

class FSMStateMachine(Machine):
    def __init__(self, data_node: DataNode):
        
        self.phase = FSMState.START
        self.data_node = data_node

        self.fsm_flags = {
            "waiting_run": False,
            "pause_mode": False
        }

        self.manual_alignment_time = 0.0

        states = [
            FSMState.START.value,
            FSMState.CONNECT.value,
            FSMState.INIT.value,
            FSMState.IDLE.value,
            FSMState.Manual_Alignment.value,
            FSMState.Auto_Alignment.value,
            FSMState.Auto_Pick.value,
            FSMState.Auto_Assemble.value,
            FSMState.Alarm.value,
        ]
        
        transitions = [
            {'trigger': 'start_to_connect', 'source': FSMState.START.value, 'dest': FSMState.CONNECT.value},
            {'trigger': 'connect_to_init', 'source': FSMState.CONNECT.value, 'dest': FSMState.INIT.value},
            {'trigger': 'init_to_idle', 'source': FSMState.INIT.value, 'dest': FSMState.IDLE.value},
            {'trigger': 'idle_to_manual_alignment', 'source': FSMState.IDLE.value, 'dest': FSMState.Manual_Alignment.value},
            # {'trigger': 'manual_alignment_to_auto_alignment', 'source': FSMState.Manual_Alignment.value, 'dest': FSMState.Auto_Alignment.value},
            {'trigger': 'manual_alignment_to_idle', 'source': FSMState.Manual_Alignment.value, 'dest': FSMState.IDLE.value},
            {'trigger': 'auto_alignment_to_auto_pick', 'source': FSMState.Auto_Alignment.value, 'dest': FSMState.Auto_Pick.value},
            {'trigger': 'auto_pick_to_auto_assemble', 'source': FSMState.Auto_Pick.value, 'dest': FSMState.Auto_Assemble.value},
            {'trigger': 'auto_assemble_to_idle', 'source': FSMState.Auto_Assemble.value, 'dest': FSMState.IDLE.value},
            {'trigger': 'alarm', 'source': '*', 'dest': FSMState.Alarm.value},
            {'trigger': 'reset_to_start', 'source': '*', 'dest': FSMState.START.value}
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = FSMState(self.state)


def fsm_logic(system: FSMStateMachine, data: DataNode):
    # 全局錯誤判斷
    if data.motion_states.get("system_error", False):
        print("[FSM] 系統錯誤，觸發 alarm")
        system.alarm()
        return

    # 全局 pause 判斷
    if data.state_cmd.get("pause_button", False):
        system.fsm_flags["pause_mode"] = True
        print("[FSM] 停止執行，等待下一次 Run")
        return

    if system.fsm_flags.get("pause_mode", False):
        if not data.state_cmd.get("run_button", False):
            print("[FSM] Pause 模式中，尚未接收到 Run")
            return
        else:
            print("[FSM] 接收到 Run，解除 Pause 模式")
            system.fsm_flags["pause_mode"] = False

    # ======================== FSM 狀態邏輯 ========================

    if system.state == 'start':
        print("[FSM] 系統啟動，觸發 start_to_connect")
        system.start_to_connect()

    elif system.state == 'connect':
        print("[FSM] 連接中...")
        connect_states = True  # TODO: 真實檢查條件
        if connect_states:
            print("[FSM] 連接成功，等待初始化按鈕")
            if data.state_cmd.get('init_button', False):
                print("[FSM] 觸發 connect_to_init")
                system.connect_to_init()
            else:
                print("[FSM] 等待初始化按鈕被按下")
        else:
            print("[FSM] 連接失敗，保持在 connect 狀態")

    elif system.state == 'init':
        print("[FSM] 初始化中...")
        init_states = True  # TODO: 真實檢查條件
        if init_states:
            print("[FSM] 初始化完成，觸發 init_to_idle")
            system.init_to_idle()
        else:
            print("[FSM] 初始化未完成，保持在 init 狀態")

    elif system.state == 'idle':
        print("[FSM] 系統待機中，等待使用者按下 Run")
        if data.state_cmd.get('run_button', False):
            print("[FSM] 觸發 idle_to_manual_alignment")
            system.idle_to_manual_alignment()
        else:
            print("[FSM] 尚未接收到 Run")

    elif system.state == 'manual_alignment':
        print("[FSM] 手動對位中...")
        if system.manual_alignment_time < 5.0:  # 假設手動對位需要 5 秒
            system.manual_alignment_time += timer_period
            print(f"[FSM] 手動對位進行中，已經等待 {system.manual_alignment_time:.1f} 秒")
        else:
            print("[FSM] 手動對位完成，觸發 manual_alignment_to_idle")
            system.manual_alignment_to_idle()
            system.manual_alignment_time = 0.0
        
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
