from PySide6.QtCore import QTimer
from common_msgs.msg import JogCmd

class MotorController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        # Connect UI buttons to jog commands
        self.ui.ControlUpCP.clicked.connect(lambda: self.send_jog_cmd("y_axis", 1.0))
        self.ui.ControlDownCP.clicked.connect(lambda: self.send_jog_cmd("y_axis", -1.0))
        self.ui.ControlLeftCP.clicked.connect(lambda: self.send_jog_cmd("x_axis", -1.0))
        self.ui.ControlRightCP.clicked.connect(lambda: self.send_jog_cmd("x_axis", 1.0))
        self.ui.YawPlusCP.clicked.connect(lambda: self.send_jog_cmd("yaw", 1.0))
        self.ui.YawMinusCP.clicked.connect(lambda: self.send_jog_cmd("yaw", -1.0))

        self.ui.MotorResetButton.clicked.connect(lambda: self.on_touch_buttons(self.ui.MotorResetButton))
        self.ui.ControlUpCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlUpCP))
        self.ui.ControlDownCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlDownCP))
        self.ui.ControlLeftCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlLeftCP))
        self.ui.ControlRightCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlRightCP))
        self.ui.YawPlusCP.clicked.connect(lambda: self.on_touch_controls(self.ui.YawPlusCP))
        self.ui.YawMinusCP.clicked.connect(lambda: self.on_touch_controls(self.ui.YawMinusCP))

    def send_jog_cmd(self, axis, direction):
        msg = JogCmd()
        msg.target = axis
        msg.direction = direction
        msg.distance = 5.0
        msg.speed = 50.0
        self.ros_node.jog_cmd_publisher.publish(msg)
        print(f"[Motor] Sent JogCmd: axis={axis}, direction={direction}")

    def on_touch_controls(self, button):
        button.setStyleSheet("""
        QPushButton {
            background-color: rgba(11, 118, 160, 0.3);
            border: 1px solid #0B76A0;
            color: white;
        }
        """)
        QTimer.singleShot(200, lambda: button.setStyleSheet("""
        QPushButton {
            background-color: transparent;
            border: none;
            color: white;
        }
        """))

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
