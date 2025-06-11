import rclpy
from rclpy.node import Node
import threading
import time
from yasmin import Machine, State

# ----- FSM Definition -----
class SimpleFSM:
    def __init__(self):
        self.states = [
            State(name="idle", on_enter=self.on_idle),
            State(name="working", on_enter=self.on_working),
        ]

        self.transitions = [
            ["start", "idle", "working"],
            ["reset", "working", "idle"],
        ]

        self.machine = Machine(states=self.states, transitions=self.transitions, initial="idle")

    def on_idle(self):
        print("[FSM] Entered IDLE state")

    def on_working(self):
        print("[FSM] Entered WORKING state")

    def run(self):
        while True:
            time.sleep(3)
            if self.machine.state == "idle":
                self.machine.start()
            else:
                self.machine.reset()

# ----- ROS 2 Node Definition -----
class MyROSNode(Node):
    def __init__(self):
        super().__init__('my_threaded_node')
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('[ROS2] Timer tick!')

def ros2_thread():
    rclpy.init()
    node = MyROSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def fsm_thread():
    fsm = SimpleFSM()
    fsm.run()

# ----- Main Multithread Launcher -----
if __name__ == "__main__":
    t1 = threading.Thread(target=ros2_thread)
    t2 = threading.Thread(target=fsm_thread)

    t1.start()
    t2.start()

    t1.join()
    t2.join()