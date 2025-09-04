# ui_forklift.py
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer
from common_msgs.msg import LimitCmd
from std_msgs.msg import Int32

class LimitController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.ui.LimitOpen.clicked.connect(lambda: self.send_limit_cmd("open_limit"))
        self.ui.LimitClose.clicked.connect(lambda: self.send_limit_cmd("close_limit"))
        self.ui.LimitStop.clicked.connect(lambda: self.send_limit_cmd("stop_limit"))
        

    def send_limit_cmd(self, cmd: str):
        msg = LimitCmd()
        msg.mode = cmd
        self.ros_node.limit_cmd_publisher.publish(msg)
        if cmd == "open_limit":
            print("[UI] 發布 LimitCmd: 開啟")
        elif cmd == "close_limit":
            print("[UI] 發布 LimitCmd: 關閉")
        elif cmd == "stop_limit":
            print("[UI] 發布 LimitCmd: 停止")


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
