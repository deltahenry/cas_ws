#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
from std_msgs.msg import String
from common_msgs.msg import TaskCmd, MotionCmd

class CompensateUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "compensate_ui")
        QWidget.__init__(self)

        self.pose_label = QLabel("目標位置: (尚未獲取)")
        self.detect_btn = QPushButton("🔍 偵測")
        self.confirm_btn = QPushButton("✅ 確認補償")

        layout = QVBoxLayout()
        layout.addWidget(self.pose_label)
        layout.addWidget(self.detect_btn)
        layout.addWidget(self.confirm_btn)
        self.setLayout(layout)

        # Publisher
        self.task_cmd_pub = self.create_publisher(TaskCmd, "/compensate_cmd", 10)
        self.confirm_pub = self.create_publisher(String, "/confirm_cmd", 10)

        # Subscriber
        self.pose_sub = self.create_subscription(MotionCmd, "/current_pose_cmd", self.pose_callback, 10)

        # 按鈕事件
        self.detect_btn.clicked.connect(self.send_detect_cmd)
        self.confirm_btn.clicked.connect(self.send_confirm_cmd)

    def send_detect_cmd(self):
        msg = TaskCmd()
        msg.mode = "l_shape"   # 依需求也可以改成 "screw"
        self.task_cmd_pub.publish(msg)
        self.pose_label.setText("目標位置: 偵測中...")

    def send_confirm_cmd(self):
        msg = String()
        msg.data = "confirm"
        self.confirm_pub.publish(msg)
        self.pose_label.setText("目標位置: 已確認，執行中...")

    def pose_callback(self, msg: MotionCmd):
        x, y, yaw = msg.pose_data
        self.pose_label.setText(
            f"目標位置: x={x:.2f}, y={y:.2f}, yaw={yaw*57.29:.2f}, speed={msg.speed:.1f}"
        )


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    ui = CompensateUI()
    ui.show()

    from threading import Thread
    executor_thread = Thread(target=rclpy.spin, args=(ui,), daemon=True)
    executor_thread.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
