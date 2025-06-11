import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from common_msgs.msg import ButtonCommand, MotionState

# 狀態與轉移定義同你之前一樣
states = ['start', 'initial', 'idle', 'rough_pos', 'auto_pos', 'assem', 'pick', 'warn', 'manual']
transitions = [
    {'trigger': 'push_init_button', 'source': 'start', 'dest': 'initial'},
    {'trigger': 'initial_finish', 'source': 'initial', 'dest': 'idle'},
    {'trigger': 'start', 'source': 'idle', 'dest': 'rough_pos'},
    {'trigger': 'rough_finish', 'source': 'rough_pos', 'dest': 'auto_pos'},
    {'trigger': 'push', 'source': 'auto_pos', 'dest': 'assem'},
    {'trigger': 'pull', 'source': 'auto_pos', 'dest': 'pick'},
    {'trigger': 'push_finish', 'source': 'assem', 'dest': 'idle'},
    {'trigger': 'pull_finish', 'source': 'pick', 'dest': 'idle'},
    # 把 warning 分拆成多條 transition
    {'trigger': 'warning', 'source': 'idle', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'rough_pos', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'auto_pos', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'assem', 'dest': 'warn'},
    {'trigger': 'warning', 'source': 'pick', 'dest': 'warn'},
    {'trigger': 'debug', 'source': 'warn', 'dest': 'manual'},
    {'trigger': 'debug_finish', 'source': 'manual', 'dest': 'idle'},
]

class AssemblyFSM:
    def __init__(self):
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='start', auto_transitions=False)

        # 初始化繪圖
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.pos = None
        self.draw_fsm()

    def draw_fsm(self):
        G = nx.DiGraph()
        for t in transitions:
            G.add_edge(t['source'], t['dest'], label=t['trigger'])

        if self.pos is None:
            self.pos = nx.spring_layout(G, seed=42)

        self.ax.clear()
        node_colors = ['lightgreen' if n == self.state else 'lightgray' for n in G.nodes]
        nx.draw(G, self.pos, ax=self.ax, with_labels=True, node_size=2500, node_color=node_colors,
                font_size=12, font_weight='bold', edgecolors='black')

        edge_labels = {(t['source'], t['dest']): t['trigger'] for t in transitions}
        nx.draw_networkx_edge_labels(G, self.pos, edge_labels=edge_labels, font_color='darkblue', ax=self.ax)

        self.ax.set_title(f"Assembly FSM - Current State: {self.state}", fontsize=16)
        self.ax.axis('off')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def trigger_callback(self, msg):
        trigger_name = msg.data.strip()
        self.get_logger().info(f"📩 收到 trigger: {trigger_name}")

        if hasattr(self, trigger_name):
            try:
                getattr(self, trigger_name)()
                self.get_logger().info(f"✅ 轉移成功，目前狀態：{self.state}")
                self.draw_fsm()  # 轉移成功就更新圖
            except Exception as e:
                self.get_logger().error(f"⚠️ 轉移失敗：{e}")
        else:
            self.get_logger().warn(f"⚠️ 找不到 trigger：{trigger_name}")

class ROS2Node(Node):
    def __init__(self):
        super().__init__('ros2_node')
        self

        # 訂閱 trigger topic

        self.button_cmd_publisher = self.create_publisher(ButtonCommand, "/button_cmd", 10)
        self.motion_state_publisher = self.create_publisher(MotionState, "/motion_state", 10)

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
    def button_cmd_callback(self, msg=ButtonCommand):
        print(f"📩 收到 ButtonCommand: {msg}")
        # 在這裡處理 ButtonCommand 的邏輯
        # 例如，根據 msg.command 觸發相應的狀態轉移
    def motion_state_callback(self, msg=MotionState):
       print(f"📩 收到 MotionState: {msg}")
        # 在這裡處理 MotionState 的邏輯
        # 例如，根據 msg.state 觸發相應的狀態轉移

def main(args=None):
    rclpy.init(args=args)
    fsm = AssemblyFSM()
    node = ROS2Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
