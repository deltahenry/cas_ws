# ui_forklift.py
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer
from common_msgs.msg import GripperCmd
from std_msgs.msg import Int32

class GripperController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.ui.OpenGripper.clicked.connect(lambda: self.send_gripper_cmd("open_gripper"))
        self.ui.CloseGripper.clicked.connect(lambda: self.send_gripper_cmd("close_gripper"))
        self.ui.StopGripper.clicked.connect(lambda: self.send_gripper_cmd("stop_gripper"))
        self.ui.ResetGripper.clicked.connect(lambda: self.send_gripper_cmd("reset_gripper"))

    def send_gripper_cmd(self, mode):
        msg = GripperCmd()
        msg.mode = mode

        self.ros_node.gripper_cmd_publisher.publish(msg)
        print(f"[Gripper] Published: {mode}")
    

    def on_touch_buttons(self, button):
        button.setStyleSheet("""
        QPushButton {
            background-color: rgba(11, 118, 160, 0.3);
            border: 1px solid #0B76A0;
            color: white;
        }
        """)
        QTimer.singleShot(200, lambda: button.setStyleSheet("""
        QPushButton {
            color: white;
        }
        """))
