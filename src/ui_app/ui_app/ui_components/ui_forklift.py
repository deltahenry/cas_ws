# ui_forklift.py
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import QTimer
from common_msgs.msg import ForkCmd
from std_msgs.msg import Int32

class ForkliftController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.current_height = 0 

        # self.ui.SliderLift.valueChanged.connect(self.on_slider_changed)
        self.ui.InputDistance.textChanged.connect(self.on_input_changed)

        # Connect UI buttons
        self.ui.LiftUp.clicked.connect(self.lift_up_10mm)
        self.ui.LowerDown.clicked.connect(self.lower_down_10mm)

        self.ui.SendForkliftCommand.clicked.connect(self.handle_send_forklift_command)
        self.ui.StopForkliftButton.clicked.connect(self.handle_stop_forklift_command)

    def publish_fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.ros_node.fork_cmd_publisher.publish(msg)
        print(f"[ForkLift] Published: {mode}, {speed}, {direction}, {distance}")

    def update_height_display(self, height):
        self.current_height = height
        self.ui.currentHeight.setText(f"{height:.2f}")  
        self.ui.zPose.setText(f"{height:.2f}") # cartesian pose: z
        self.ui.zVisionLabel.setText(f"{height:.2f}")

    def lift_up_10mm(self):
        self.on_touch_buttons(self.ui.LiftUp)

        self.publish_fork_cmd("run", self.get_speed(), "up", float(min(1500, self.current_height + 10.0)))  # send only +10mm

        # Disable the button for 5 seconds
        self.disable_buttons_temporarily(5000)


    def lower_down_10mm(self):
        self.on_touch_buttons(self.ui.LowerDown)

        self.publish_fork_cmd("run", self.get_speed(), "down", float(max(80, self.current_height - 10.0)))  # send only -10mm

        self.disable_buttons_temporarily(5000)


    def get_speed(self):
        speed_button = self.ui.buttonGroup.checkedButton()
        return speed_button.text().lower() if speed_button else "slow"

    def disable_buttons_temporarily(self, ms):
        # Disable functionality
        self.ui.LiftUp.setEnabled(False)
        self.ui.LowerDown.setEnabled(False)

        # Add disabled style without overwriting existing design
        self.ui.LiftUp.setStyleSheet(self.ui.LiftUp.styleSheet() + "opacity: 0.5;")
        self.ui.LowerDown.setStyleSheet(self.ui.LowerDown.styleSheet() + "opacity: 0.5;")

        # Re-enable after cooldown
        QTimer.singleShot(ms, lambda: self.enable_buttons())

    def enable_buttons(self):
        self.ui.LiftUp.setEnabled(True)
        self.ui.LowerDown.setEnabled(True)

        # Remove only the opacity we added
        original_style_up = self.ui.LiftUp.styleSheet().replace("opacity: 0.5;", "")
        original_style_down = self.ui.LowerDown.styleSheet().replace("opacity: 0.5;", "")

        self.ui.LiftUp.setStyleSheet(original_style_up)
        self.ui.LowerDown.setStyleSheet(original_style_down)



    def handle_send_forklift_command(self):
        # Send a ForkCmd with mode="run"
        self.send_fork_cmd_from_height_command()

    def handle_stop_forklift_command(self):
        # Send a pure STOP message
        self.publish_fork_cmd(mode="stop", speed="", direction="", distance=0.0)


    def send_fork_cmd_from_height_command(self):
        mode = "run"
        speed = self.get_speed()

        # Read target height from UI
        raw_text = self.ui.HeightCommand.text()
        print(f"[DEBUG] Raw HeightCommand text: '{raw_text}'")
        cleaned_text = raw_text.lower().replace("mm", "").strip()

        try:
            target_distance = float(cleaned_text)
        except ValueError:
            print("[Forklift] Invalid HeightCommand value")
            return

        # Auto decide direction based on current height
        current_height = self.current_height  # Use your own member directly
        if target_distance > current_height:
            direction = "up"
        elif target_distance < current_height:
            direction = "down"
        else:
            self.handle_stop_forklift_command()
            return

        self.publish_fork_cmd(mode, speed, direction, target_distance)



    # def on_slider_changed(self, value):
    #         self.ui.HeightCommand.setText(f"{value:.2f}")

    def on_input_changed(self, value):
        try:
            num = float(value)  # convert string to float
            self.ui.HeightCommand.setText(f"{num:.2f}")
        except ValueError:
            # handle non-numeric input gracefully
            self.ui.HeightCommand.setText("")


    def publish_adjust_command(self, direction, distance):
            self.publish_fork_cmd("run", self.get_speed(), direction, float(distance))


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
