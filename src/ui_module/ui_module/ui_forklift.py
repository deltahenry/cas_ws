import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton, QComboBox, QLabel, QSlider, QHBoxLayout
)
from PySide6.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from common_msgs.msg import ForkCmd
from std_msgs.msg import Int32


class ForkliftControlNode(Node):
    def __init__(self):
        super().__init__('forklift_control_node')
        self.current_height = 0  # mm
        self.gui = None  # GUI reference

        self.publisher_ = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.height_info_subscriber = self.create_subscription(
            Int32,
            'lr_distance',
            self.height_info_callback,
            10
        )

    def publish_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")

    def height_info_callback(self, msg: Int32):
        self.current_height = msg.data
        if self.gui:
            self.gui.update_height(msg.data)
        self.get_logger().info(f"Received height info: {msg.data} mm")


class ForkliftControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Forklift Control")
        self.step = 10  # mm per step
        self.adjust_direction = None
        self.current_distance = 120  # initial value

        layout = QVBoxLayout()

        # --- Height Display ---
        self.height_label = QLabel("Current Height: 0 mm")
        self.height_label.setStyleSheet("font-size: 18px;")
        layout.addWidget(self.height_label)

        self.height_cmd_label = QLabel("Height Cmd: 120 mm")
        self.height_cmd_label.setStyleSheet("font-size: 18px;")
        layout.addWidget(self.height_cmd_label)

        # --- Mode ---
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["run"])
        layout.addWidget(QLabel("Mode"))
        layout.addWidget(self.mode_combo)

        # --- Speed ---
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["fast", "slow", "medium"])
        layout.addWidget(QLabel("Speed"))
        layout.addWidget(self.speed_combo)

        # --- Direction ---
        self.direction_combo = QComboBox()
        self.direction_combo.addItems(["up", "down"])
        # layout.addWidget(QLabel("Direction"))
        # layout.addWidget(self.direction_combo)

        # --- Distance Slider ---
        layout.addWidget(QLabel("Distance (manual)"))
        self.distance_slider = QSlider(Qt.Horizontal)
        self.distance_slider.setMinimum(80)
        self.distance_slider.setMaximum(1500)
        self.distance_slider.setValue(self.current_distance)
        self.distance_slider.valueChanged.connect(self.update_distance)
        layout.addWidget(self.distance_slider)

        # --- Up/Down Buttons ---
        button_layout = QHBoxLayout()
        self.up_button = QPushButton("↑")
        self.down_button = QPushButton("↓")
        for btn in [self.up_button, self.down_button]:
            btn.setMinimumHeight(60)
            btn.setStyleSheet("font-size: 24px;")
        button_layout.addWidget(self.up_button)
        button_layout.addWidget(self.down_button)
        layout.addLayout(button_layout)

        self.up_button.pressed.connect(self.start_increase)
        self.up_button.released.connect(self.stop_adjustment)
        self.down_button.pressed.connect(self.start_decrease)
        self.down_button.released.connect(self.stop_adjustment)

        self.adjust_timer = QTimer()
        self.adjust_timer.timeout.connect(self.adjust_distance)

        # --- Send Manual Command Button ---
        self.send_button = QPushButton("Send Command")
        self.send_button.setMinimumHeight(60)
        self.send_button.setStyleSheet("font-size: 20px; color: white; background-color: green;")
        self.send_button.clicked.connect(self.send_command)
        layout.addWidget(self.send_button)

        # --- STOP Button ---
        self.stop_button = QPushButton("STOP")
        self.stop_button.setMinimumHeight(60)
        self.stop_button.setStyleSheet("font-size: 20px; color: white; background-color: red;")
        self.stop_button.clicked.connect(self.send_stop_command)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)

    def update_distance(self, value):
        self.current_distance = value
        self.height_cmd_label.setText(f"Height Cmd: {value} mm")

    def update_height(self, height):
        self.height_label.setText(f"Current Height: {height} mm")

    def get_current_height(self):
        return self.ros_node.current_height

    def start_increase(self):
        self.adjust_direction = "up"
        self.adjust_timer.start(100)

    def start_decrease(self):
        self.adjust_direction = "down"
        self.adjust_timer.start(100)

    def stop_adjustment(self):
        self.adjust_timer.stop()
        self.adjust_direction = None

    def adjust_distance(self):
        current_height = self.get_current_height()

        if self.adjust_direction == "up":
            distance = min(1500, current_height + self.step)
        elif self.adjust_direction == "down":
            distance = max(80, current_height - self.step)
        else:
            return

        mode = self.mode_combo.currentText()
        speed = self.speed_combo.currentText()
        direction = self.adjust_direction
        self.ros_node.publish_cmd(mode, speed, direction, float(distance))
        self.height_cmd_label.setText(f"Height Cmd: {distance} mm")

    def send_command(self):
        mode = self.mode_combo.currentText()
        speed = self.speed_combo.currentText()
        direction = self.direction_combo.currentText()
        distance = float(self.current_distance)
        self.ros_node.publish_cmd(mode, speed, direction, distance)
        self.height_cmd_label.setText(f"Height Cmd: {distance} mm")

    def send_stop_command(self):
        self.ros_node.publish_cmd("stop", "", "", 0.0)


def main(args=None):
    rclpy.init(args=args)
    ros_node = ForkliftControlNode()

    app = QApplication(sys.argv)
    gui = ForkliftControlGUI(ros_node)
    ros_node.gui = gui
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    exit_code = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
