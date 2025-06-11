import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps

# 狀態與轉移
states = [
    'start','initial', 'idle', 'rough_pos', 'auto_pos',
    'assem', 'pick', 'warn', 'manual'
]

transitions = [
    {'trigger': 'push_init_button', 'source': 'start', 'dest': 'initial'},
    {'trigger': 'initial_finish', 'source': 'initial', 'dest': 'idle'},
    {'trigger': 'start', 'source': 'idle', 'dest': 'rough_pos'},
    {'trigger': 'rough_finish', 'source': 'rough_pos', 'dest': 'auto_pos'},
    {'trigger': 'push', 'source': 'auto_pos', 'dest': 'assem'},
    {'trigger': 'pull', 'source': 'auto_pos', 'dest': 'pick'},
    {'trigger': 'push_finish', 'source': 'assem', 'dest': 'idle'},
    {'trigger': 'pull_finish', 'source': 'pick', 'dest': 'idle'},
    {'trigger': 'warning', 'source': 'idle', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'rough_pos', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'auto_pos', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'assem', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'pick', 'dest': 'warn'},
    {'trigger': 'debug', 'source': 'warn', 'dest': 'manual'},
    {'trigger': 'debug_finish', 'source': 'manual', 'dest': 'idle'},
]

demo_sequence = [
    'initial_finish',
    'start',
    'rough_finish',
    'push',
    'warning',
    'debug',
    'debug_finish',
    'start',
    'rough_finish',
    'pull',
    'pull_finish'
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

    ax.set_title(f"Assembly FSM - Current State: {current_state}", fontsize=16)
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

# 🧠 State Machine 實作
class AssemblySystem:
    def __init__(self):
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='initial', auto_transitions=False)
        self.a = 0
        self.machine.on_enter_idle(self.on_enter_idle)
        self.machine.on_exit_idle(self.on_exit_idle)
        self.current_step = 0

    @with_plot
    def step(self):
        if self.current_step >= len(demo_sequence):
            return False
        trigger = demo_sequence[self.current_step]
        print(f"[➡️ 轉移] 呼叫觸發器：{trigger}")
        if hasattr(self, trigger):
            getattr(self, trigger)()
            self.current_step += 1
            return True
        return False

    def on_enter_idle(self):
        while 1:
            print("進入[狀態] idle：系統待命。")
            self.a += 1
            print(self.a)
            break

    def on_exit_idle(self):
        print("退出[狀態] idle：系統待命。")

# ✅ 新增 main() 函式
def main():
    system = AssemblySystem()
    while True:
        print(f"[現在狀態] {system.state}")
        if not system.step():
            print("✅ 流程完成。")
            break
        time.sleep(1)
    plt.ioff()
    plt.show()

# 🏁 若此檔案直接執行，就進入 main()
if __name__ == "__main__":
    main()
