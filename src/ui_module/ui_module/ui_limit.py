#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from threading import Thread
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QMainWindow
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from common_msgs.msg import TaskCmd, LimitCmd   # ✅ 匯入 LimitCmd

class IntegratedUI(QWidget):
    def __init__(self, ros_node: Node):
        super().__init__()
        self.ros_node = ros_node

        # ========== Compensate ==========
        self.pose_label = QLabel("目標位置: (尚未獲取)")

        self.detect_btn = QPushButton("🔍 偵測")
        self.cancel_btn = QPushButton("❌ 取消偵測")
        self.confirm_btn = QPushButton("✅ 確認補償")
        self.to_done_btn = QPushButton("✅ To Done")

        # ========== Limit Controller ==========
        self.limit_open_btn = QPushButton("🚦 開啟 Limit")
        self.limit_close_btn = QPushButton("🛑 關閉 Limit")
        self.limit_stop_btn = QPushButton("⏹ 停止 Limit")

        # Publisher
        self.task_cmd_pub = self.ros_node.create_publisher(TaskCmd, "/compensate_cmd", 10)
        self.confirm_pub = self.ros_node.create_publisher(String, "/confirm_cmd", 10)
        self.limit_pub = self.ros_node.create_publisher(LimitCmd, "/limit_cmd", 10)   # ✅ 使用 LimitCmd

        # Subscriber
        self.pose_sub = self.ros_node.create_subscription(
            Float32MultiArray, "/compensate_pose_cmd", self.pose_callback, 10
        )

        # ========== Layout ==========
        layout = QVBoxLayout()
        layout.addWidget(QLabel("=== 補償控制 ==="))
        layout.addWidget(self.pose_label)
        layout.addWidget(self.detect_btn)
        layout.addWidget(self.cancel_btn)
        layout.addWidget(self.confirm_btn)
        layout.addWidget(self.to_done_btn)

        layout.addWidget(QLabel("=== Limit 控制 ==="))
        layout.addWidget(self.limit_open_btn)
        layout.addWidget(self.limit_close_btn)
        layout.addWidget(self.limit_stop_btn)

        self.setLayout(layout)

        # 綁定事件
        self.detect_btn.clicked.connect(lambda: self.send_detect_cmd("l_shape"))
        self.cancel_btn.clicked.connect(lambda: self.send_detect_cmd("stop"))
        self.confirm_btn.clicked.connect(lambda: self.send_confirm_cmd("confirm"))
        self.to_done_btn.clicked.connect(lambda: self.send_confirm_cmd("to_done"))

        self.limit_open_btn.clicked.connect(lambda: self.send_limit_cmd("open_limit"))
        self.limit_close_btn.clicked.connect(lambda: self.send_limit_cmd("close_limit"))
        self.limit_stop_btn.clicked.connect(lambda: self.send_limit_cmd("stop_limit"))

    # ===== Compensate 功能 =====
    def send_detect_cmd(self, cmd: str):
        msg = TaskCmd()
        msg.mode = cmd
        self.task_cmd_pub.publish(msg)
        if cmd == "l_shape":
            self.pose_label.setText("目標位置: 偵測中...")
        elif cmd == "stop":
            self.pose_label.setText("目標位置: 偵測已取消")
        print(f"[UI] 發布偵測指令: {cmd}")

    def send_confirm_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.confirm_pub.publish(msg)
        if cmd == "confirm":
            self.pose_label.setText("目標位置: 已確認，執行中...")
        elif cmd == "to_done":
            self.pose_label.setText("目標位置: 強制完成補償")
        print(f"[UI] 發布確認指令: {cmd}")

    # ===== Limit Controller 功能 =====
    def send_limit_cmd(self, cmd: str):
        msg = LimitCmd()
        msg.mode = cmd
        self.limit_pub.publish(msg)
        if cmd == "open_limit":
            print("[UI] 發布 LimitCmd: 開啟")
        elif cmd == "close_limit":
            print("[UI] 發布 LimitCmd: 關閉")
        elif cmd == "stop_limit":
            print("[UI] 發布 LimitCmd: 停止")

    def pose_callback(self, msg: Float32MultiArray):
        x, y, yaw, z = msg.data
        self.pose_label.setText(
            f"目標位置: (x: {x:.2f}, y: {y:.2f}, yaw(deg): {yaw:.2f}, z: {z:.2f})"
        )

def main():
    rclpy.init()
    ros_node = Node("integrated_ui_node")

    app = QApplication(sys.argv)
    window = QMainWindow()
    window.setWindowTitle("整合控制介面")
    ui = IntegratedUI(ros_node)
    window.setCentralWidget(ui)
    window.resize(400, 500)
    window.show()

    # ROS2 executor 放在背景執行緒
    def ros_spin():
        rclpy.spin(ros_node)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    sys.exit(app.exec())

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
