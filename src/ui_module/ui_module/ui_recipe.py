#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QStackedWidget, QGridLayout
)
from PySide6.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from common_msgs.msg import Recipe  # 你的自訂 msg


# Pick (2x9，高度獨立)
pick_heights = {
    "C1": [944.0, 824.0, 704.0, 586.0, 465.0, 344.0, 226.0, 113.0, 113.0],
    "C2": [944.0, 824.0, 704.0, 586.0, 465.0, 344.0, 226.0, 113.0, 113.0],
}

# Assembly (4x9，高度獨立)
assembly_heights = {
    "C1": [944.0, 824.0, 704.0, 586.0, 465.0, 344.0, 226.0, 113.0, 113.0],
    "C2": [944.0, 824.0, 704.0, 586.0, 465.0, 344.0, 226.0, 113.0, 113.0],
    "C3": [944.0, 824.0, 704.0, 586.0, 465.0, 344.0, 226.0, 113.0, 113.0],
    "C4": [944.0, 824.0, 704.0, 586.0, 465.0, 344.0, 226.0, 113.0, 113.0],
}


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

        self.selected_button = None
        self.selected_mode = None
        self.selected_height = None
        self.selected_depth = None

        main_layout = QVBoxLayout()

        # Mode 選擇
        mode_layout = QHBoxLayout()
        mode_label = QLabel("Mode:")
        self.mode_box = QComboBox()
        self.mode_box.addItems(["pick", "assembly"])
        self.mode_box.currentTextChanged.connect(self.switch_page)
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_box)
        main_layout.addLayout(mode_layout)

        # Stack: Pick Page & Assembly Page
        self.stack = QStackedWidget()
        self.pick_page = self.create_grid_page("pick", pick_heights, fixed_depth=500.0)
        self.assembly_page = self.create_grid_page("assembly", assembly_heights, fixed_depth=600.0)
        self.stack.addWidget(self.pick_page)
        self.stack.addWidget(self.assembly_page)
        main_layout.addWidget(self.stack)

        # Save 按鈕
        self.save_button = QPushButton("Save Recipe")
        self.save_button.clicked.connect(self.save_recipe)
        main_layout.addWidget(self.save_button)

        self.setLayout(main_layout)
        self.switch_page("pick")  # 預設顯示 Pick 頁面

    def create_grid_page(self, mode, height_dict, fixed_depth):
        page = QWidget()
        layout = QGridLayout(page)

        for c, (col_name, heights) in enumerate(height_dict.items()):
            for r, h in enumerate(heights):
                btn = QPushButton(f"{col_name}R{r+1}\nH={h}, D={fixed_depth}")
                btn.setCheckable(True)
                btn.clicked.connect(
                    lambda checked, h=h, d=fixed_depth, b=btn, m=mode: self.select_cell(m, h, d, b)
                )
                layout.addWidget(btn, r, c)

        return page

    def switch_page(self, mode):
        if mode == "pick":
            self.stack.setCurrentIndex(0)
        else:
            self.stack.setCurrentIndex(1)

    def select_cell(self, mode, height, depth, button):
        if self.selected_button:
            self.selected_button.setChecked(False)
        self.selected_button = button
        self.selected_mode = mode
        self.selected_height = height
        self.selected_depth = depth
        print(f"選擇 {mode} -> H={height}, D={depth}")

    def save_recipe(self):
        if self.selected_mode is None:
            print("請先選擇格子")
            return
        self.node.publish_recipe(self.selected_mode, self.selected_height, self.selected_depth)


def main(args=None):
    rclpy.init(args=args)
    node = RecipePublisher()

    app = QApplication(sys.argv)
    ui = RecipeUI(node)
    ui.show()

    # ROS2 與 Qt 整合
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
