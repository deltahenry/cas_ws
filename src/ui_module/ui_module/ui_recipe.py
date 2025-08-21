#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox
)
import rclpy
from rclpy.node import Node
from common_msgs.msg import Recipe  # 你的自訂 msg


class RecipePublisher(Node):
    def __init__(self):
        super().__init__('recipe_ui_node')
        self.publisher_ = self.create_publisher(Recipe, 'recipe_cmd', 10)

    def publish_recipe(self, mode: str, height: float, depth: float):
        msg = Recipe()
        msg.mode = mode
        msg.height = height
        msg.depth = depth
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg}")


class RecipeUI(QWidget):
    def __init__(self, node: RecipePublisher):
        super().__init__()
        self.node = node
        self.setWindowTitle("Recipe Publisher UI")

        layout = QVBoxLayout()

        # Mode 下拉選單
        mode_layout = QHBoxLayout()
        mode_label = QLabel("Mode:")
        self.mode_box = QComboBox()
        self.mode_box.addItems(["pick", "assembly"])
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_box)
        layout.addLayout(mode_layout)

        # Height 輸入
        height_layout = QHBoxLayout()
        height_label = QLabel("Height:")
        self.height_input = QLineEdit()
        self.height_input.setPlaceholderText("輸入 height (float)")
        height_layout.addWidget(height_label)
        height_layout.addWidget(self.height_input)
        layout.addLayout(height_layout)

        # Depth 輸入
        depth_layout = QHBoxLayout()
        depth_label = QLabel("Depth:")
        self.depth_input = QLineEdit()
        self.depth_input.setPlaceholderText("輸入 depth (float)")
        depth_layout.addWidget(depth_label)
        depth_layout.addWidget(self.depth_input)
        layout.addLayout(depth_layout)

        # Publish 按鈕
        self.publish_button = QPushButton("Publish Recipe")
        self.publish_button.clicked.connect(self.publish_recipe)
        layout.addWidget(self.publish_button)

        self.setLayout(layout)

    def publish_recipe(self):
        try:
            mode = self.mode_box.currentText()
            height = float(self.height_input.text())
            depth = float(self.depth_input.text())
            self.node.publish_recipe(mode, height, depth)
        except ValueError:
            print("請輸入正確的數字 (height, depth)")


def main(args=None):
    rclpy.init(args=args)
    node = RecipePublisher()

    app = QApplication(sys.argv)
    ui = RecipeUI(node)
    ui.show()

    # ROS2 與 Qt 整合
    from PySide6.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
