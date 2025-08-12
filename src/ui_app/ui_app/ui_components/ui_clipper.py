# ui_forklift.py
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer
from common_msgs.msg import ClipperCmd
from std_msgs.msg import Int32

class ClipperController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.ui.OpenClipper.clicked.connect(lambda: self.send_clipper_cmd("open_clipper"))
        self.ui.CloseClipper.clicked.connect(lambda: self.send_clipper_cmd("close_clipper"))
        self.ui.StopClipper.clicked.connect(lambda: self.send_clipper_cmd("stop_clipper"))
        self.ui.ResetClipper.clicked.connect(lambda: self.send_clipper_cmd("reset_clipper"))

    def send_clipper_cmd(self, mode):
        msg = ClipperCmd()
        msg.mode = mode

        self.ros_node.clipper_cmd_publisher.publish(msg)
        print(f"[Clipper] Published: {mode}")
    

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
