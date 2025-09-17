import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit
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
        self.gui = None

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
        self.get_logger().info(
            f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}"
        )

    def height_info_callback(self, msg: Int32):
        self.current_height = msg.data
        if self.gui:
            self.gui.update_height(msg.data)


class ForkliftControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Forklift Control")
        self.step = 2  # 每次移動量 2mm
        self.current_distance = 0

        layout = QVBoxLayout()

        # --- 高度顯示 ---
        self.height_label = QLabel("現在高度: 0 mm")
        self.height_label.setAlignment(Qt.AlignCenter)
        self.height_label.setStyleSheet("font-size: 20px;")
        layout.addWidget(self.height_label)

        self.height_cmd_label = QLabel("高度命令: 0 mm")
        self.height_cmd_label.setAlignment(Qt.AlignCenter)
        self.height_cmd_label.setStyleSheet("font-size: 20px;")
        layout.addWidget(self.height_cmd_label)

        # --- 輸入框 + 發布命令 ---
        input_layout = QHBoxLayout()
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText("輸入高度 mm")
        self.input_field.setFixedHeight(40)
        input_layout.addWidget(self.input_field)

        self.publish_button = QPushButton("發布命令")
        self.publish_button.setMinimumHeight(40)
        self.publish_button.setStyleSheet("font-size: 18px; background-color: green; color: white;")
        self.publish_button.clicked.connect(self.send_command)
        input_layout.addWidget(self.publish_button)
        layout.addLayout(input_layout)

        # --- 上下微調按鈕 ---
        btn_layout = QHBoxLayout()
        self.up_button = QPushButton("↑")
        self.down_button = QPushButton("↓")
        for btn in [self.up_button, self.down_button]:
            btn.setMinimumHeight(60)
            btn.setStyleSheet("font-size: 24px;")
        btn_layout.addWidget(self.up_button)
        btn_layout.addWidget(self.down_button)
        layout.addLayout(btn_layout)

        self.up_button.clicked.connect(self.increase_and_send)
        self.down_button.clicked.connect(self.decrease_and_send)

        # --- STOP 按鈕 ---
        self.stop_button = QPushButton("STOP")
        self.stop_button.setMinimumHeight(60)
        self.stop_button.setStyleSheet("font-size: 20px; background-color: red; color: white;")
        self.stop_button.clicked.connect(self.send_stop_command)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)

    def update_height(self, height):
        self.height_label.setText(f"現在高度: {height} mm")

    def send_command(self):
        """由輸入框 Key in 值來發布"""
        text = self.input_field.text().strip()
        if not text:
            return
        try:
            value = int(text)
            self.current_distance = value
            self.height_cmd_label.setText(f"高度命令: {value} mm")
            self.ros_node.publish_cmd("run", "slow", "up", float(value))
        except ValueError:
            print("⚠️ 請輸入有效的數字")

    def send_stop_command(self):
        self.ros_node.publish_cmd("stop", "slow", "down", 0.0)

    def increase_and_send(self):
        self.current_distance += self.step
        self.height_cmd_label.setText(f"高度命令: {self.current_distance} mm")
        self.ros_node.publish_cmd("run", "slow", "up", float(self.current_distance))

    def decrease_and_send(self):
        self.current_distance = max(0, self.current_distance - self.step)
        self.height_cmd_label.setText(f"高度命令: {self.current_distance} mm")
        self.ros_node.publish_cmd("run", "slow", "down", float(self.current_distance))


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


if __name__ == "__main__":
    main()
