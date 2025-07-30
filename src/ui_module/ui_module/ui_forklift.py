import sys
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QComboBox, QLabel, QSlider, QHBoxLayout
from PySide6.QtCore import Qt

import rclpy
from rclpy.node import Node
from common_msgs.msg import ForkCmd  # 你的消息定義，請確認名稱與訊息內容

class ForkliftControlNode(Node):
    def __init__(self):
        super().__init__('forklift_control_node')
        self.publisher_ = self.create_publisher(ForkCmd, 'fork_cmd', 10)

    def publish_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")

class ForkliftControlGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Forklift Control")

        layout = QVBoxLayout()

        # 模式選擇 (run / stop)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["run", "stop"])
        layout.addWidget(QLabel("Mode"))
        layout.addWidget(self.mode_combo)

        # 速度選擇 (fast / slow)
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["fast", "slow","medium"])
        layout.addWidget(QLabel("Speed"))
        layout.addWidget(self.speed_combo)

        # 方向選擇 (up / down)
        self.direction_combo = QComboBox()
        self.direction_combo.addItems(["up", "down"])
        layout.addWidget(QLabel("Direction"))
        layout.addWidget(self.direction_combo)

        # 距離滑桿
        self.distance_slider = QSlider(Qt.Horizontal)
        self.distance_slider.setMinimum(80)
        self.distance_slider.setMaximum(200)
        self.distance_slider.setValue(120)
        layout.addWidget(QLabel("Distance"))
        layout.addWidget(self.distance_slider)

        # 發送按鈕
        self.send_button = QPushButton("Send Command")
        self.send_button.clicked.connect(self.send_command)
        layout.addWidget(self.send_button)

        self.setLayout(layout)

    def send_command(self):
        mode = self.mode_combo.currentText()
        speed = self.speed_combo.currentText()
        direction = self.direction_combo.currentText()
        distance = float(self.distance_slider.value())
        self.ros_node.publish_cmd(mode, speed, direction, distance)

def main(args=None):
    rclpy.init(args=args)
    ros_node = ForkliftControlNode()

    app = QApplication(sys.argv)
    gui = ForkliftControlGUI(ros_node)
    gui.show()

    # 使用 Qt timer，定期執行 rclpy.spin_once 避免阻塞 Qt loop
    from PySide6.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # 每10毫秒執行一次

    exit_code = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
