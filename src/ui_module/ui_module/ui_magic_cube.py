import sys
import rclpy
from rclpy.node import Node
from common_msgs.msg import ButtonCommand, MotionState

from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel,
    QVBoxLayout, QGridLayout
)
from PySide6.QtCore import QTimer

class RosPublisherNode(Node):
    def __init__(self):
        super().__init__('gui_button_publisher')
        self.button_cmd_publisher = self.create_publisher(ButtonCommand, "/button_cmd", 10)
        self.motion_state_publisher = self.create_publisher(MotionState, "/motion_state", 10)

        self.button_cmd_subscription = self.create_subscription(
            ButtonCommand, "/button_cmd", self.button_cmd_callback, 10)
        self.motion_state_subscription = self.create_subscription(
            MotionState, "/motion_state", self.motion_state_callback, 10)

        self.latest_button_cmd = None
        self.latest_motion_state = None

    def publish_all_button_states(self, button_states):
        msg = ButtonCommand()
        msg.stop_button = button_states["Stop"]
        msg.init_button = button_states["Init"]
        msg.reselect_button = button_states["Reselect"]
        msg.pull_button = button_states["Pull"]
        msg.push_button = button_states["Push"]
        msg.debug_button = button_states["Debug"]

        self.button_cmd_publisher.publish(msg)
        self.get_logger().info("Published ButtonCommand message")

    def publish_motion_state(self, motion_flags):
        msg = MotionState()
        msg.motion_finish = motion_flags["Motion Finish"]
        msg.init_finish = motion_flags["Init Finish"]
        msg.pull_finish = motion_flags["Pull Finish"]
        msg.push_finish = motion_flags["Push Finish"]
        msg.rough_pos_finish = motion_flags["Rough Pos Finish"]
        msg.auto_pos_finish = motion_flags["Auto Pos Finish"]
        msg.system_error = motion_flags["System Error"]
        self.motion_state_publisher.publish(msg)
        self.get_logger().info("Published MotionState message")

    def button_cmd_callback(self, msg):
        self.latest_button_cmd = msg

    def motion_state_callback(self, msg):
        self.latest_motion_state = msg

    def get_latest_msgs(self):
        return self.latest_button_cmd, self.latest_motion_state

class ControlUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("ROS 2 System Control Panel")
        self.setFixedSize(500, 500)
        self.ros_node = ros_node

        self.button_states = {
            "Stop": False,
            "Init": False,
            "Reselect": False,
            "Pull": False,
            "Push": False,
            "Debug": False,
        }

        self.motion_flags = {
            "Motion Finish": False,
            "Init Finish": False,
            "Pull Finish": False,
            "Push Finish": False,
            "Rough Pos Finish": False,
            "Auto Pos Finish": False,
            "System Error": False,
        }

        self.button_widgets = {}
        self.motion_widgets = {}

        self.init_ui()

        # 每 200ms 檢查是否有新的 ROS 訊息
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_ros_messages)
        self.timer.start(200)

    def init_ui(self):
        layout = QVBoxLayout()
        grid = QGridLayout()

        # ButtonCommand 控制區
        for i, name in enumerate(self.button_states.keys()):
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(self.make_button_handler(name))
            self.button_widgets[name] = btn
            grid.addWidget(btn, i // 2, i % 2)
        layout.addLayout(grid)

        # MotionState 控制區
        for name in self.motion_flags.keys():
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(self.make_motion_handler(name))
            self.motion_widgets[name] = btn
            layout.addWidget(btn)

        self.status_label = QLabel("ROS Publishing Enabled")
        layout.addWidget(self.status_label)
        self.setLayout(layout)

    def make_button_handler(self, name):
        def handler():
            self.button_states[name] = self.button_widgets[name].isChecked()
            self.ros_node.publish_all_button_states(self.button_states)
        return handler

    def make_motion_handler(self, name):
        def handler():
            self.motion_flags[name] = self.motion_widgets[name].isChecked()
            self.ros_node.publish_motion_state(self.motion_flags)
            self.status_label.setText(f"Published {name}: {self.motion_flags[name]}")
        return handler

    def check_ros_messages(self):
        button_msg, motion_msg = self.ros_node.get_latest_msgs()

        if button_msg:
            self.update_ui_from_button_cmd(button_msg)
            self.ros_node.latest_button_cmd = None  # 清除

        if motion_msg:
            self.update_ui_from_motion_state(motion_msg)
            self.ros_node.latest_motion_state = None  # 清除

    def update_ui_from_button_cmd(self, msg=ButtonCommand):
        mapping = {
            "Stop": msg.stop_button,
            "Init": msg.init_button,
            "Reselect": msg.reselect_button,
            "Pull": msg.pull_button,
            "Push": msg.push_button,
            "Debug": msg.debug_button,
        }
        for key, val in mapping.items():
            self.button_states[key] = val
            self.button_widgets[key].setChecked(val)

    def update_ui_from_motion_state(self, msg=MotionState):
        mapping = {
            "Motion Finish": msg.motion_finish,
            "Init Finish": msg.init_finish,
            "Pull Finish": msg.pull_finish,
            "Push Finish": msg.push_finish,
            "Rough Pos Finish": msg.rough_pos_finish,
            "Auto Pos Finish": msg.auto_pos_finish,
            "System Error": msg.system_error,

        }
        for key, val in mapping.items():
            self.motion_flags[key] = val
            self.motion_widgets[key].setChecked(val)

def main():
    rclpy.init()
    ros_node = RosPublisherNode()

    app = QApplication(sys.argv)
    window = ControlUI(ros_node)
    window.show()

    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
