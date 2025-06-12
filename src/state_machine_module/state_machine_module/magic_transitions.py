import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_msgs.msg import ButtonCommand, MotionState


#parameters
timer_period = 0.5  # seconds

# 狀態與轉移
states = [
    'start','initial', 'idle', 'rough_pos', 'auto_pos',
    'push', 'pull', 'warn', 'manual'
]

transitions = [
    {'trigger': 'push_init_button', 'source': 'start', 'dest': 'initial'},
    {'trigger': 'initial_finish', 'source': 'initial', 'dest': 'idle'},
    {'trigger': 'start', 'source': 'idle', 'dest': 'rough_pos'},
    {'trigger': 'rough_finish', 'source': 'rough_pos', 'dest': 'auto_pos'},
    {'trigger': 'push', 'source': 'auto_pos', 'dest': 'push'},
    {'trigger': 'pull', 'source': 'auto_pos', 'dest': 'pull'},
    {'trigger': 'push_finish', 'source': 'push', 'dest': 'idle'},
    {'trigger': 'pull_finish', 'source': 'pull', 'dest': 'idle'},
    {'trigger': 'warning', 'source': 'start', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'initial', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'idle', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'rough_pos', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'auto_pos', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'push', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'pull', 'dest': 'warn'},
    {'trigger': 'debug', 'source': 'warn', 'dest': 'manual'},
    {'trigger': 'debug_finish', 'source': 'manual', 'dest': 'idle'},
]

# 🎨 畫圖邏輯
def init_plot():
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 8))
    return fig, ax

def update_plot(ax, transitions, current_state, pos=None):
    G = nx.DiGraph()
    for t in transitions:
        G.add_edge(t['source'], t['dest'], label=t['trigger'])

    if pos is None:
        pos = nx.spring_layout(G, seed=42)

    ax.clear()
    node_colors = ['lightgreen' if n == current_state else 'lightgray' for n in G.nodes]
    nx.draw(G, pos, ax=ax, with_labels=True, node_size=2500, node_color=node_colors,
            font_size=12, font_weight='bold', edgecolors='black')

    edge_labels = {(t['source'], t['dest']): t['trigger'] for t in transitions}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='darkblue', ax=ax)

    ax.set_title(f"pushbly FSM - Current State: {current_state}", fontsize=16)
    ax.axis('off')
    plt.pause(1.0)
    return pos

# 🌟 裝飾器
def with_plot(func):
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if not hasattr(self, 'fig') or not hasattr(self, 'ax'):
            self.fig, self.ax = init_plot()
        result = func(self, *args, **kwargs)
        wrapper.pos = update_plot(self.ax, transitions, self.state, pos=getattr(wrapper, 'pos', None))
        return result
    wrapper.pos = None
    return wrapper


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

# 🧠 State Machine 實作
class AssemblySystem:
    def __init__(self,data_node: DataNode):
        self.data_node = data_node
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='start', auto_transitions=False)
        self.a = 0
        self.machine.on_enter_start(self.on_enter_start)
        self.machine.on_exit_start(self.on_exit_start)

        self.machine.on_enter_initial(self.on_enter_initial)
        self.machine.on_exit_initial(self.on_exit_initial)

        self.machine.on_enter_idle(self.on_enter_idle)
        self.machine.on_exit_idle(self.on_exit_idle)

        self.current_step = 0

    #start
    def on_enter_start(self):
        print("進入[狀態] start：系統啟動。")

    def on_exit_start(self):
        print("退出[狀態] start：系統啟動。")

    #initial
    def on_enter_initial(self):
        print("進入[狀態] initial：系統初始化。")
        self.data_node.rewrite_button_cmd('init_button', False)  # 重置初始化按鈕狀態
        
    def on_exit_initial(self):
        print("退出[狀態] initial：系統初始化。")

    #idle
    def on_enter_idle(self):
        print("進入[狀態] idle：系統待命。")

    def on_exit_idle(self):
        print("退出[狀態] idle：系統待命。")

    # rough_pos
    def on_enter_rough_pos(self):
        print("進入[狀態] rough_pos：系統進入粗定位。")

    def on_exit_rough_pos(self):
        print("退出[狀態] rough_pos：系統進入粗定位。")
    
    # auto_pos
    def on_auto_pos(self):
        print("進入[狀態] auto_pos：系統進入自動定位。")

    def on_exit_auto_pos(self):
        print("退出[狀態] auto_pos：系統進入自動定位。")

    # push
    def on_enter_push(self):
        print("進入[狀態] push：系統進入組裝。")

    def on_exit_push(self):
        print("退出[狀態] push：系統進入組裝。")
        self.data_node.rewrite_button_cmd('push_button',False)
        self.data_node.rewrite_motion_state('rough_pos_finish', False)  # 重置自動對準完成狀態
        self.data_node.rewrite_motion_state('auto_pos_finish', False)  # 重置自動對準完成狀態
        self.data_node.rewrite_motion_state('push_finish', False)  # 重置推進完成狀態
    
    # pull
    def on_enter_pull(self):
        print("進入[狀態] pull：系統進入拾取。")
        
    def on_exit_pull(self):
        print("退出[狀態] pull：系統進入拾取。")
        self.data_node.rewrite_button_cmd('pull_button',False)
        self.data_node.rewrite_motion_state('rough_pos_finish', False)  # 重置自動對準完成狀態
        self.data_node.rewrite_motion_state('auto_pos_finish', False)
        self.data_node.rewrite_motion_state('pull_finish', False)  # 重置拾取完成狀態

    # warn
    def on_enter_warn(self):
        print("進入[狀態] warn：系統進入警告狀態。")
  
    def on_exit_warn(self):
        print("退出[狀態] warn：系統進入警告狀態。")
   
    # manual
    def on_enter_manual(self):
        print("進入[狀態] manual：系統進入手動模式。")

  
    def on_exit_manual(self):
        print("退出[狀態] manual：系統進入手動模式。")
        self.data_node.rewrite_button_cmd('debug_button', False)  # 重置除錯按鈕狀態
        self.data_node.rewrite_motion_state('system_error', False)  # 重置系統錯誤狀態




@with_plot
def fsm_logic(system: AssemblySystem,data: DataNode):

    # 狀態持續時的行為（每次都執行）
    if system.state == 'start':
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        elif data.button_cmds['init_button']:           
            print("🔘 [FSM] 按下初始化按鈕，觸發 push_init_button")
            system.push_init_button()
    
    elif system.state == 'initial':
        print("🔄 [FSM] 系統初始化中，執行 initial 邏輯...")
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        elif data.motion_states['init_finish']:
            print("✅ [FSM] 初始化完成，觸發 initial_finish")
            system.initial_finish()
        else:
            print("🛠️ [FSM] 等待初始化完成...")

    elif system.state == 'idle':            
        print("🕒 [FSM] 系統待命中，執行 idle 邏輯...")
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        elif data.button_cmds['pull_button']:
            print("🔘 [FSM] 按下拾取器按鈕，觸發 start")
            system.start()
        elif data.button_cmds['push_button']:
            print("🔘 [FSM] 按下推進器按鈕，觸發 start")
            system.start()
        else:
            print("🛑 [FSM] 等待按鈕命令...")

    elif system.state == 'rough_pos':
        print("🔧 [FSM] 系統進入粗定位，執行 rough_pos 邏輯...")
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        elif data.motion_states['rough_pos_finish']:
            print("✅ [FSM] 粗定位完成，觸發 rough_finish")
            system.rough_finish()
        else:
            print("🛠️ [FSM] 等待粗定位完成...")
    
    elif system.state == 'auto_pos':
        print("🔍 [FSM] 系統進入自動定位，執行 auto_pos 邏輯...")
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        elif data.button_cmds['push_button']:
            if data.motion_states['auto_pos_finish']:
                print("✅ [FSM] 自動定位完成，觸發 push 或 pull")
                system.push()
        elif data.button_cmds['pull_button']:
            if data.motion_states['auto_pos_finish']:
                print("✅ [FSM] 自動定位完成，觸發 push 或 pull")
                system.pull()
        else:
            print("🛠️ [FSM] 等待自動定位完成...")

    elif system.state == 'push':
        print("🔧 [FSM] 系統進入組裝，執行 push 邏輯...")
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        elif data.motion_states['push_finish']:
            print("✅ [FSM] 組裝完成，觸發 push_finish")
            system.push_finish()
        else:
            print("🛠️ [FSM] 等待組裝完成...")

    elif system.state == 'pull':
        print("🔧 [FSM] 系統進入拾取，執行 pull 邏輯...")
        if data.motion_states["system_error"]:
            print("⚠️ [FSM] 系統錯誤，觸發 warning")
            system.warning()
        elif data.motion_states['pull_finish']:
            print("✅ [FSM] 拾取完成，觸發 pull_finish")
            system.pull_finish()
        else:
            print("🛠️ [FSM] 等待拾取完成...")

    elif system.state == 'warn':
        print("⚠️ [FSM] 系統進入警告狀態，執行 warn 邏輯...")
        if data.button_cmds['debug_button']:
            print("🔘 [FSM] 按下除錯按鈕，觸發 debug")
            system.debug()
        else:
            print("🛑 [FSM] 等待除錯按鈕命令...")

    elif system.state == 'manual':
        print("🛠️ [FSM] 系統進入手動模式，執行 manual 邏輯...")
        if data.button_cmds['init_button']:
            print("🔘 [FSM] 按下初始化按鈕，觸發 debug_finish")
            system.debug_finish()
        else:
            print("🛑 [FSM] 等待初始化按鈕命令...")

def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = AssemblySystem(data)    # FSM 實體

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
