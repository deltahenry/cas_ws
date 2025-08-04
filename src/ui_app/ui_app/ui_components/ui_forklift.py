# ui_forklift.py
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer
from common_msgs.msg import ForkCmd
from std_msgs.msg import Int32

class ForkliftController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.ui.SliderLift.valueChanged.connect(self.on_slider_changed)


        self.current_height = 0 

        # Connect UI buttons
        self.ui.LiftUp.clicked.connect(self.lift_up_10mm)
        self.ui.LowerDown.clicked.connect(self.lower_down_10mm)

        self.ui.SendForkliftCommand.clicked.connect(self.handle_send_forklift_command)
        self.ui.StopForkliftButton.clicked.connect(self.handle_stop_forklift_command)

    


    def send_fork_cmd(self, direction):
        mode_button = self.ui.buttonGroup_2.checkedButton()
        speed_button = self.ui.buttonGroup.checkedButton()

        mode = mode_button.text().lower() if mode_button else "run"
        speed = speed_button.text().lower() if speed_button else "slow"
        distance = float(self.ui.SliderLift.value())

        self.publish_fork_cmd(mode, speed, direction, distance)

    def update_height_display(self, height):
        self.current_height = height
        self.ui.currentHeight.setText(f"{height} mm")  



    def publish_fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.ros_node.fork_cmd_publisher.publish(msg)
        print(f"[ForkLift] Published: {mode}, {speed}, {direction}, {distance}")

    def handle_send_forklift_command(self):
        # Send a ForkCmd with mode="run"
        self.send_fork_cmd_from_height_command(mode="run")

    def handle_stop_forklift_command(self):
        # Send a pure STOP message
        self.publish_fork_cmd(mode="stop", speed="", direction="", distance=0.0)


    def send_fork_cmd_from_height_command(self, mode):
        speed_button = self.ui.buttonGroup.checkedButton()
        speed = speed_button.text().lower() if speed_button else "slow"

        # Use fixed direction from buttons
        if self.ui.LiftUp.isChecked():
            direction = "up"
        elif self.ui.LowerDown.isChecked():
            direction = "down"
        else:
            direction = "stop"

        raw_text = self.ui.HeightCommand.text()
        print(f"[DEBUG] Raw HeightCommand text: '{raw_text}'")
        cleaned_text = raw_text.lower().replace("mm", "").strip()

        try:
            target_distance = float(cleaned_text)
        except ValueError:
            print("[Forklift] Invalid HeightCommand value")
            return

        self.publish_fork_cmd(mode, speed, direction, target_distance)


    def on_slider_changed(self, value):
        self.ui.HeightCommand.setText(str(value))


    def lift_up_10mm(self):
        self.on_touch_buttons(self.ui.LiftUp)

        new_value = min(self.ui.SliderLift.maximum(), self.ui.SliderLift.value() + 10)
        self.ui.SliderLift.setValue(new_value)  # This updates the HeightCommand text too

        # Immediately publish the command
        self.publish_adjust_command("up", new_value)

    def lower_down_10mm(self):
        self.on_touch_buttons(self.ui.LowerDown)

        new_value = max(self.ui.SliderLift.minimum(), self.ui.SliderLift.value() - 10)
        self.ui.SliderLift.setValue(new_value)

        self.publish_adjust_command("down", new_value)

    def publish_adjust_command(self, direction, distance):
        speed_button = self.ui.buttonGroup.checkedButton()
        speed = speed_button.text().lower() if speed_button else "slow"
        self.publish_fork_cmd("run", speed, direction, float(distance))





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