# ui_forklift.py
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer
from common_msgs.msg import ForkCmd

class ForkliftController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        # Connect UI buttons
        self.ui.LiftUp.clicked.connect(lambda: self.send_fork_cmd("up"))
        self.ui.LowerDown.clicked.connect(lambda: self.send_fork_cmd("down"))

        self.ui.LiftUp.clicked.connect(lambda: self.on_touch_buttons(self.ui.LiftUp))
        self.ui.LowerDown.clicked.connect(lambda: self.on_touch_buttons(self.ui.LowerDown))
        

    def send_fork_cmd(self, direction):
        mode_button = self.ui.buttonGroup_2.checkedButton()
        speed_button = self.ui.buttonGroup.checkedButton()

        mode = mode_button.text().lower() if mode_button else "run"
        speed = speed_button.text().lower() if speed_button else "slow"
        distance = float(self.ui.SliderLift.value())

        self.publish_fork_cmd(mode, speed, direction, distance)



    # def send_command(self):
    #     mode = self.mode_combo.currentText()
    #     speed = self.speed_combo.currentText()
    #     direction = self.direction_combo.currentText()
    #     distance = float(self.current_distance)
    #     self.ros_node.publish_cmd(mode, speed, direction, distance)
    #     self.height_cmd_label.setText(f"Height Cmd: {distance} mm")

    def publish_fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.ros_node.fork_cmd_publisher.publish(msg)
        print(f"[ForkLift] Published: {mode}, {speed}, {direction}, {distance}")


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
