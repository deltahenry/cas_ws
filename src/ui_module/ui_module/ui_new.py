#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from threading import Thread
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QTextEdit, QSplitter, QPushButton, QLabel, QComboBox, QStackedWidget, QDoubleSpinBox,
    QButtonGroup, QLineEdit, QGroupBox, QSizePolicy, QStackedLayout
)
from PySide6.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray,Int32,Bool,Int32MultiArray
from common_msgs.msg import Recipe, TaskCmd, StateCmd, TaskState, LimitCmd,GripperCmd,ForkCmd,MotionCmd,MotionState,CurrentPose,ForkState,MH2State

from PySide6.QtGui import QPixmap, QImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from uros_interface.srv import ESMCmd


# -------------------
# 高度設定
pick_heights = {"C1": [944.0,824.0,704.0,586.0,465.0,344.0,226.0,113.0,113.0]}
assembly_heights = {"C1": [944.0,824.0,704.0,586.0,465.0,344.0,226.0,113.0,113.0]}

# ROS2 Publisher Node
class RecipePublisher(Node):
    def __init__(self):
        super().__init__('recipe_ui_node')
        self.recipe_publisher = self.create_publisher(Recipe, 'recipe_cmd', 10)
        self.task_cmd_publisher = self.create_publisher(TaskCmd, 'task_cmd', 10)
        self.state_cmd_publisher = self.create_publisher(StateCmd, 'state_cmd', 10)
        self.limit_pub = self.create_publisher(LimitCmd, '/limit_cmd', 10)
        self.gripper_pub = self.create_publisher(GripperCmd, '/gripper_cmd', 10)
        self.pass_pub = self.create_publisher(Float32MultiArray, '/compensate_pose_cmd', 10)
        self.confirm_pub = self.create_publisher(String, '/confirm_cmd', 10)
        self.compensate_task_pub = self.create_publisher(TaskCmd, '/compensate_cmd', 10)
        self.debug_pub = self.create_publisher(Bool,'/debug_cmd', 10)
        self.servo_pub = self.create_publisher(String, "/servo_cmd", 10)
        self.light_pub = self.create_publisher(String, 'light_cmd', 10)

        # Async service client
        self.cli = self.create_client(ESMCmd, '/esm_command')
        self.waiting_for_result = False

    def publish_recipe(self, mode, height, depth):
        msg = Recipe()
        msg.mode = mode
        msg.height = height
        msg.depth = depth
        self.recipe_publisher.publish(msg)
        self.get_logger().info(f"Published Recipe: {msg}")


# -------------------
# UI1: RecipeUI
class RecipeUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.selected_button = None
        self.selected_mode = None
        self.selected_height = None
        self.selected_depth = None
        self.selected_name = None
        self.log_callback = None

        self.pick_depth = 500.0
        self.assembly_depth = 600.0

        main_layout = QVBoxLayout()

        # Mode 選擇
        mode_layout = QHBoxLayout()
        mode_label = QLabel("Mode:")
        self.mode_box = QComboBox()
        self.mode_box.addItems(["pick","assembly"])
        self.mode_box.currentTextChanged.connect(self.switch_page)
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_box)
        main_layout.addLayout(mode_layout)

        # 手動輸入深度
        depth_layout = QHBoxLayout()
        depth_label = QLabel("Depth:")
        self.depth_spin = QDoubleSpinBox()
        self.depth_spin.setRange(0, 2000)   # 假設深度範圍
        self.depth_spin.setValue(self.pick_depth)
        self.depth_spin.valueChanged.connect(self.update_depth)
        depth_layout.addWidget(depth_label)
        depth_layout.addWidget(self.depth_spin)
        main_layout.addLayout(depth_layout)

        # Stack
        self.stack = QStackedWidget()
        self.pick_page = self.create_grid_page("pick", pick_heights, lambda: self.pick_depth)
        self.assembly_page = self.create_grid_page("assembly", assembly_heights, lambda: self.assembly_depth)
        self.stack.addWidget(self.pick_page)
        self.stack.addWidget(self.assembly_page)
        main_layout.addWidget(self.stack)

        # Save
        self.save_button = QPushButton("Save Recipe")
        self.save_button.clicked.connect(self.save_recipe)
        main_layout.addWidget(self.save_button)
        self.setLayout(main_layout)
        self.switch_page("pick")

    def create_grid_page(self, mode, height_dict, depth_func):
        page = QWidget()
        layout = QGridLayout(page)
        col_name, heights = list(height_dict.items())[0]
        for r,h in enumerate(heights):
            name = f"{col_name}R{r+1}"
            btn = QPushButton(f"{name}\nH={h:.1f}, D={depth_func():.1f}")
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, h=h, d_func=depth_func, b=btn, m=mode, n=name: self.select_cell(m,h,d_func(),b,n))
            layout.addWidget(btn, r, 0)
        return page

    def switch_page(self, mode):
        if mode=="pick":
            self.stack.setCurrentIndex(0)
            self.depth_spin.setValue(self.pick_depth)
        else:
            self.stack.setCurrentIndex(1)
            self.depth_spin.setValue(self.assembly_depth)

    def select_cell(self, mode, height, depth, button, name):
        if self.selected_button: 
            self.selected_button.setChecked(False)
        self.selected_button = button
        self.selected_mode = mode
        self.selected_height = height
        self.selected_depth = depth
        self.selected_name = name
        if self.log_callback:
            self.log_callback(1,f"👉 選擇 {mode} -> {name}, Depth={depth:.1f}")

    def update_depth(self, val):
        """當手動輸入深度時更新"""
        if self.mode_box.currentText() == "pick":
            self.pick_depth = val
        else:
            self.assembly_depth = val
        # 如果已經選擇了某個 cell，要更新顯示
        if self.selected_button:
            self.selected_button.setText(f"{self.selected_name}\nH={self.selected_height:.1f}, D={val:.1f}")
            self.selected_depth = val

    def save_recipe(self):
        if not self.selected_mode:
            if self.log_callback: self.log_callback(1,"⚠ 請先選擇格子")
            return
        if self.selected_mode=="pick":
            self.selected_depth = self.pick_depth
            self.update_depth(self.pick_depth)
        else:
            self.selected_depth = self.assembly_depth
            self.update_depth(self.assembly_depth)
        self.node.publish_recipe(self.selected_mode,self.selected_height,self.selected_depth)
        if self.log_callback:
            self.log_callback(1,f"💾 已儲存: Mode={self.selected_mode},櫃體={self.selected_name}, Depth={self.selected_depth:.1f}")

# -------------------
# UI2: Task UI
class UI2(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        self.DI_subscriber = self.node.create_subscription(Int32MultiArray,'di_state',self.update_di_state,10)

        layout = QVBoxLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(5,5,5,5)
        
        top_layout = QHBoxLayout()
        top_layout.setSpacing(10)

        self.init_btn = QPushButton("INIT")
        self.run_btn = QPushButton("RUN")
        for b in [self.init_btn, self.run_btn]:
            self.enlarge_button(b)
            top_layout.addWidget(b)

        # self.init_btn.clicked.connect(lambda: self.send_state_cmd("init"))
        self.run_btn.clicked.connect(lambda: self.send_state_cmd("run"))

        layout.addLayout(top_layout)

        self.task_group = QButtonGroup(self)
        self.task_group.setExclusive(False)
        self.task_widgets = {}
        self.current_task = None

        task_names = ["rough_align","precise_align","pick","assembly"]
        for name in task_names:
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.toggled.connect(lambda checked, n=name: self.on_task_toggled(n, checked))
            self.enlarge_button(btn)  # 放大按鈕
            layout.addWidget(btn)

            log = QTextEdit()
            log.setReadOnly(True)
            log.setFixedHeight(50)  # 縮小 log
            layout.addWidget(log)

            self.task_group.addButton(btn)
            self.task_widgets[name] = {"button": btn, "log": log}

        # Pause / Stop 並排
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.pause_btn = QPushButton("pause")
        self.pause_btn.setCheckable(True)
        self.pause_state = False
        self.pause_btn.clicked.connect(lambda: self.toggle_state_button("pause"))
        self.enlarge_button(self.pause_btn)
        btn_layout.addWidget(self.pause_btn)

        self.stop_btn = QPushButton("stop")
        self.stop_btn.setCheckable(True)
        self.stop_state = False
        self.stop_btn.clicked.connect(lambda: self.toggle_state_button("stop"))
        self.enlarge_button(self.stop_btn)
        btn_layout.addWidget(self.stop_btn)

        layout.addLayout(btn_layout)

        self.setLayout(layout)

    def enlarge_button(self, btn, width=150, height=50, font_size=18):
        btn.setStyleSheet(f"""
            font-size: {font_size}px;
        """)
        btn.setMinimumSize(width, height)
        btn.setMaximumSize(width*2, height*2)  # 防止樣式縮小


        # 訂閱任務 topic
        self.node.create_subscription(TaskState, 'task_state_rough_align', self.update_rough_align_log, 10)
        self.node.create_subscription(TaskState, 'task_state_precise_align', self.update_precise_align_log, 10)
        self.node.create_subscription(TaskState, 'task_state_pick', self.update_pick_log, 10)
        self.node.create_subscription(TaskState, 'task_state_assembly', self.update_assembly_log, 10)

    def on_task_toggled(self, task_name, checked):
        log_widget = self.task_widgets[task_name]["log"]

        if checked:
            if self.current_task == task_name:
                self.send_task_cmd("idle")
                log_widget.append(f"[UI2] {task_name} cancelled -> idle")
                self.task_widgets[task_name]["button"].setChecked(False)
                self.current_task = None
            else:
                self.send_task_cmd(task_name)
                log_widget.append(f"[UI2] {task_name} started")
                self.current_task = task_name
                # 取消其他任務
                for other, widgets in self.task_widgets.items():
                    if other != task_name:
                        widgets["button"].setChecked(False)
        else:
            if self.current_task == task_name:
                self.send_task_cmd("idle")
                log_widget.append(f"[UI2] {task_name} cancelled -> idle")
                self.current_task = None

    def send_task_cmd(self, task_name):
        msg = TaskCmd()
        msg.mode = task_name
        self.node.task_cmd_publisher.publish(msg)

    def update_di_state(self, msg: Int32MultiArray):
        di_states = msg.data
        estop_state = di_states[15]  # 假設第16個元素是 E-STOP 狀態
        if estop_state == 0 and not self.stop_state:
            self.set_state("stop", True, source="estop")
        # elif estop_state == 1 and self.stop_state:
        #     self.set_state("stop", False, source="estop")

    # --- UI/外部事件觸發 ---
    def toggle_state_button(self, flag):
        if flag == "stop":
            self.set_state("stop", not self.stop_state, source="ui")
        elif flag == "pause":
            self.set_state("pause", not self.pause_state, source="ui")
    
    def set_state(self, flag: str, state: bool, source="ui"):
        """統一處理 stop/pause 狀態更新"""
        if flag == "stop":
            if self.stop_state == state:
                return
            self.stop_state = state
            btn = self.stop_btn
        elif flag == "pause":
            if self.pause_state == state:
                return
            self.pause_state = state
            btn = self.pause_btn
        else:
            return

        # 更新 UI
        btn.setChecked(state)
        btn.setStyleSheet("background-color: red; color: white;" if state else "")

        # 如果來源是 UI，才發布 ROS cmd
        if source == "ui":
            self.send_state_cmd(flag)

        # log
        for widgets in self.task_widgets.values():
            widgets["log"].append(f"[UI2] {flag} {'ON' if state else 'OFF'} (from {source})")

    def send_state_cmd(self, flag):
        msg = StateCmd()
        msg.init_button = False
        msg.run_button = False
        msg.pause_button = False
        msg.stop_button = False
        if flag == "init": msg.init_button = True
        elif flag == "run": msg.run_button = True
        elif flag == "pause": msg.pause_button = self.pause_state
        elif flag == "stop": msg.stop_button = self.stop_state
        self.node.state_cmd_publisher.publish(msg)

    # def toggle_state_button(self, flag):
    #     if flag == "pause":
    #         self.pause_state = not self.pause_state
    #         is_on = self.pause_state
    #         btn = self.pause_btn
    #     else:
    #         self.stop_state = not self.stop_state
    #         is_on = self.stop_state
    #         btn = self.stop_btn

    #     btn.setStyleSheet("background-color: red; color: white;" if is_on else "")
    #     self.send_state_cmd(flag)
    #     for widgets in self.task_widgets.values():
    #         widgets["log"].append(f"[UI2] {flag} {'ON' if is_on else 'OFF'}")

    def update_task_log(self, task_name, msg: TaskState):
        log_widget = self.task_widgets[task_name]["log"]
        mode = getattr(msg, "mode", "unknown")
        state = getattr(msg, "state", "running")
        log_widget.append(f"[Topic] {mode} -> {state}")

    def update_rough_align_log(self, msg: TaskState):
        self.update_task_log("rough_align", msg)
    
    def update_precise_align_log(self, msg: TaskState):
        self.update_task_log("precise_align", msg)
    
    def update_pick_log(self, msg: TaskState):
        self.update_task_log("pick", msg)
    
    def update_assembly_log(self, msg: TaskState):
        self.update_task_log("assembly", msg)

# -------------------
# UI3: Compensate / Limit
class UI3(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.state = "idle"
        self.current_yaw = 0.0  # 存放 UI 回傳 yaw 值

        self.servo_state = False  # 真實狀態 (True=ON, False=OFF, None=未知)

        self.debug_state = False  # 除錯模式

        # ---------------- 補償控制區 ----------------
        compensation_group = QGroupBox("補償控制")
        compensation_layout = QVBoxLayout()

        self.state_label = QLabel("補償狀態: idle")
        self.pose_label = QLabel("目標位置: (尚未獲取)")

        # 偵測 & 取消偵測
        self.detect_btn = QPushButton("🔍 偵測")
        self.cancel_btn = QPushButton("❌ 取消偵測")
        detect_layout = QHBoxLayout()
        detect_layout.addWidget(self.detect_btn)
        detect_layout.addWidget(self.cancel_btn)

        # 補償 Z / X
        # Z補償
        # self.z_input = QLineEdit()
        # self.z_input.setPlaceholderText("輸入Z值")
        # self.z_btn = QPushButton("✅ Z補償")
        # self.z_btn.setFixedWidth(100)
        # z_layout = QHBoxLayout()
        # z_layout.addWidget(QLabel("Z補償值:"))
        # z_layout.addWidget(self.z_input)
        # z_layout.addWidget(self.z_btn)  # 按鈕貼右側
        self.z_label = QLabel("Z補償值: 0.00")
        self.z_label.setFixedWidth(200)
        self.z_btn = QPushButton("✅ Z補償")
        self.z_btn.setFixedWidth(100)

        z_layout = QHBoxLayout()
        z_layout.setContentsMargins(0, 0, 0, 0)
        z_layout.addWidget(self.z_label)
        z_layout.addStretch(1)       # 將按鈕推到右側
        z_layout.addWidget(self.z_btn)

        # X補償
        # self.x_input = QLineEdit()
        # self.x_input.setPlaceholderText("輸入X值")
        # self.x_btn = QPushButton("✅ X補償")
        # self.x_btn.setFixedWidth(100)

        # x_layout = QHBoxLayout()
        # x_layout.addWidget(QLabel("X補償值:"))
        # x_layout.addWidget(self.x_input)
        # x_layout.addWidget(self.x_btn)
        self.x_label = QLabel("X補償值: 0.00")
        self.x_label.setFixedWidth(200)
        self.x_btn = QPushButton("✅ X補償")
        self.x_btn.setFixedWidth(100)

        x_layout = QHBoxLayout()
        x_layout.setContentsMargins(0, 0, 0, 0)
        x_layout.addWidget(self.x_label)
        x_layout.addStretch(1)       # 將按鈕推到右側
        x_layout.addWidget(self.x_btn)

        # Yaw補償 (前方顯示 UI 回傳 yaw)
        self.yaw_label = QLabel("Yaw補償值: 0.00")
        self.yaw_label.setFixedWidth(200)
        self.yaw_btn = QPushButton("✅ Yaw補償")
        self.yaw_btn.setFixedWidth(100)

        yaw_layout = QHBoxLayout()
        yaw_layout.setContentsMargins(0, 0, 0, 0)
        yaw_layout.addWidget(self.yaw_label)
        yaw_layout.addStretch(1)       # 將按鈕推到右側
        yaw_layout.addWidget(self.yaw_btn)

        # Pass / To Done
        self.pass_btn = QPushButton("▶ Pass")
        self.to_done_btn = QPushButton("✅ To Done")
        task_layout = QHBoxLayout()
        task_layout.addWidget(self.pass_btn)
        task_layout.addSpacing(20)
        task_layout.addWidget(self.to_done_btn)

        # 補償控制區整合
        compensation_layout.addWidget(self.state_label)
        compensation_layout.addWidget(self.pose_label)
        compensation_layout.addLayout(detect_layout)
        compensation_layout.addLayout(z_layout)
        compensation_layout.addLayout(x_layout)
        compensation_layout.addLayout(yaw_layout)
        compensation_layout.addLayout(task_layout)

        compensation_group.setLayout(compensation_layout)
        compensation_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        # ---------------- 元件控制區 ----------------
        component_group = QGroupBox("元件控制")
        component_layout = QVBoxLayout()

        # 元件切換按鈕
        self.motor_btn = QPushButton("馬達控制")
        self.fork_btn = QPushButton("叉車控制")
        self.gripper_btn = QPushButton("夾具控制")
        self.limit_btn = QPushButton("限位控制")
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.motor_btn)
        btn_layout.addWidget(self.fork_btn)
        btn_layout.addWidget(self.gripper_btn)
        btn_layout.addWidget(self.limit_btn)
        component_layout.addLayout(btn_layout)

        # 元件控制內容區 (QStackedLayout)
        self.component_stack = QStackedLayout()
        self.motor_widget = QLabel("馬達控制區 (待實作)")
        self.fork_widget = QLabel("叉車控制區 (待實作)")
        self.gripper_widget = QLabel("夾具控制區 (待實作)")

        # Limit 控制頁面
        self.limit_widget = QWidget()
        limit_layout = QVBoxLayout()
        self.limit_open_btn = QPushButton("🚦 開啟 Limit")
        self.limit_close_btn = QPushButton("🛑 關閉 Limit")
        self.limit_stop_btn = QPushButton("⏹ 停止 Limit")
        self.limit_reset_btn = QPushButton("🔄 重置 Limit")
        self.limit_status_label = QLabel("狀態: --")
        self.limit_status_label.setFixedHeight(20)  # 高度固定 20px
        self.limit_status_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.limit_status_label.setStyleSheet("font-size: 14px;")  # 調整字體大小
        limit_layout.addWidget(self.limit_status_label)  # 加到 layout 裡
        limit_layout.addWidget(self.limit_open_btn)
        limit_layout.addWidget(self.limit_close_btn)
        limit_layout.addWidget(self.limit_stop_btn)
        limit_layout.addWidget(self.limit_reset_btn)
        self.limit_widget.setLayout(limit_layout)

        # Gripper 控制頁面
        self.gripper_widget = QWidget()
        gripper_layout = QVBoxLayout()
        self.gripper_open_btn = QPushButton("🚦 開啟 Gripper")
        self.gripper_close_btn = QPushButton("🛑 關閉 Gripper")
        self.gripper_stop_btn = QPushButton("⏹ 停止 Gripper")
        self.gripper_reset_btn = QPushButton("🔄 重置 Gripper")
        self.gripper_status_label = QLabel("狀態: --")
        self.gripper_status_label.setFixedHeight(20)  # 高度固定 20px
        self.gripper_status_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.gripper_status_label.setStyleSheet("font-size: 14px;")  # 調整字體大小
        gripper_layout.addWidget(self.gripper_status_label)
        gripper_layout.addWidget(self.gripper_open_btn)
        gripper_layout.addWidget(self.gripper_close_btn)
        gripper_layout.addWidget(self.gripper_stop_btn)
        gripper_layout.addWidget(self.gripper_reset_btn)
        self.gripper_widget.setLayout(gripper_layout)

        self.fork_widget = QWidget()
        fork_layout = QVBoxLayout()

        # 水平排列：目前高度 + 高度命令
        height_display_layout = QHBoxLayout()
        self.fork_current_label = QLabel("目前高度: - mm")
        self.fork_current_label.setStyleSheet("font-size: 14px;")
        self.fork_cmd_label = QLabel("高度命令: - mm")
        self.fork_cmd_label.setStyleSheet("font-size: 14px;")
        self.fork_state_label = QLabel("狀態: --")
        self.fork_state_label.setStyleSheet("font-size: 14px;")
        height_display_layout.addWidget(self.fork_current_label)
        height_display_layout.addSpacing(20)  # 間距
        height_display_layout.addWidget(self.fork_cmd_label)
        height_display_layout.addSpacing(20)  # 間距
        height_display_layout.addWidget(self.fork_state_label)
        fork_layout.addLayout(height_display_layout)

        # 高度輸入 + 發布
        height_input_layout = QHBoxLayout()
        self.fork_height_input = QLineEdit()
        self.fork_height_input.setPlaceholderText("高度輸入 (mm)")
        self.fork_publish_btn = QPushButton("✅ 發布命令")
        self.fork_publish_btn.setMinimumHeight(40)
        height_input_layout.addWidget(QLabel("輸入高度(mm):"))
        height_input_layout.addWidget(self.fork_height_input)
        height_input_layout.addWidget(self.fork_publish_btn)
        fork_layout.addLayout(height_input_layout)

        # 上下控制按鈕
        move_layout = QHBoxLayout()
        self.fork_up_btn = QPushButton("⬆ 上升+2mm")
        self.fork_down_btn = QPushButton("⬇ 下降-2mm")
        for btn in [self.fork_up_btn, self.fork_down_btn]:
            btn.setMinimumHeight(30)
            btn.setStyleSheet("font-size: 16px;")
        move_layout.addWidget(self.fork_up_btn)
        move_layout.addWidget(self.fork_down_btn)
        fork_layout.addLayout(move_layout)

        # STOP 按鈕
        self.fork_stop_btn = QPushButton("STOP")
        self.fork_stop_btn.setMinimumHeight(50)
        self.fork_stop_btn.setStyleSheet("font-size: 18px; background-color: red; color: white;")
        fork_layout.addWidget(self.fork_stop_btn)

        self.fork_widget.setLayout(fork_layout)

        self.fork_publish_btn.clicked.connect(self.fork_send_command)
        self.fork_up_btn.clicked.connect(self.fork_increase)
        self.fork_down_btn.clicked.connect(self.fork_decrease)
        self.fork_stop_btn.clicked.connect(self.fork_stop)

        # 內部變數
        self.fork_current_height = 0.0  # 目前高度
        self.fork_current_distance = 0.0  # 記錄命令送出的高度
        self.fork_step = 2.0  # 每次微調的高度 (mm)

        # 馬達控制頁面
        self.motor_widget = QWidget()
        motor_layout = QVBoxLayout()

        # --- 上方 Home / Y+ / Stop + Debug --- 
        top_layout = QGridLayout()

        # Servo button (一開始顯示未知狀態)
        self.servo_btn = QPushButton("⚙️ Servo ?")
        self.servo_btn.setFixedSize(60, 30)
        self.servo_btn.setStyleSheet("background-color: gray; font-size: 12px;")
        self.servo_btn.clicked.connect(self.toggle_servo)

        self.home_btn = QPushButton("🏠 Home")
        self.debug_btn = QPushButton("🐞 Debug")
        self.stop_btn = QPushButton("🛑 Stop")
        self.y_plus_btn = QPushButton("Y+")

        for btn in [self.home_btn, self.debug_btn, self.stop_btn]:
            btn.setFixedSize(60, 30)
            btn.setStyleSheet("font-size: 12px;")

        self.y_plus_btn.setFixedSize(40, 40)
        self.y_plus_btn.setStyleSheet("font-size: 16px;")

        # 右上 Servo 在 Home 右方
        top_layout.addWidget(self.servo_btn, 0, 0, alignment=Qt.AlignLeft | Qt.AlignTop)
        # 左上 Home
        top_layout.addWidget(self.home_btn, 0, 0, alignment=Qt.AlignRight | Qt.AlignTop)
        # Debug 在 Home 下方
        top_layout.addWidget(self.debug_btn, 0, 2, alignment=Qt.AlignLeft | Qt.AlignTop)
        # 右上 Stop
        top_layout.addWidget(self.stop_btn, 0, 2, alignment=Qt.AlignRight | Qt.AlignTop)
        # Y+ 置中
        top_layout.addWidget(self.y_plus_btn, 0, 1, alignment=Qt.AlignCenter)

        motor_layout.addLayout(top_layout)

        # --- 下方中央 XY 控制 + 步距下拉 ---
        xy_layout = QGridLayout()
        self.x_minus_btn = QPushButton("X-")
        self.x_plus_btn = QPushButton("X+")
        self.y_minus_btn = QPushButton("Y-")
        self.step_combo = QComboBox()
        self.step_combo.setFixedSize(80, 30)
        self.step_combo.setStyleSheet("font-size: 12px;")
        self.step_combo.addItem("0.1mm/0.1°", (0.1, 0.1, 0.1))
        self.step_combo.addItem("1mm/0.5°", (1.0, 1.0, 0.5))
        self.step_combo.addItem("5mm/1°", (5.0, 5.0, 1.0))
        self.step_combo.addItem("10mm/5°", (10.0, 10.0, 5.0))

        for btn in [self.x_minus_btn, self.x_plus_btn, self.y_minus_btn]:
            btn.setFixedSize(40, 40)
            btn.setStyleSheet("font-size: 16px;")

        xy_layout.addWidget(self.x_minus_btn, 0, 0, alignment=Qt.AlignCenter)
        xy_layout.addWidget(self.step_combo, 0, 1, alignment=Qt.AlignCenter)
        xy_layout.addWidget(self.x_plus_btn, 0, 2, alignment=Qt.AlignCenter)
        xy_layout.addWidget(self.y_minus_btn, 1, 1, alignment=Qt.AlignCenter)

        # Yaw 按鈕左下 / 右下
        self.yaw_plus_btn = QPushButton("⟲")
        self.yaw_minus_btn = QPushButton("⟳")
        for btn in [self.yaw_plus_btn, self.yaw_minus_btn]:
            btn.setFixedSize(30, 30)
            btn.setStyleSheet("font-size: 18px;")
        xy_layout.addWidget(self.yaw_minus_btn, 1, 0, alignment=Qt.AlignLeft | Qt.AlignBottom)
        xy_layout.addWidget(self.yaw_plus_btn, 1, 2, alignment=Qt.AlignRight | Qt.AlignBottom)

        motor_layout.addLayout(xy_layout)

        # --- Y 軸輸入 + 發布 ---
        y_input_layout = QHBoxLayout()
        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("輸入Y軸位置 (mm)")
        self.y_publish_btn = QPushButton("✅ 發布Y軸命令")
        self.y_publish_btn.setMinimumHeight(40)
        y_input_layout.addWidget(QLabel("Y軸位置:"))
        y_input_layout.addWidget(self.y_input)
        y_input_layout.addWidget(self.y_publish_btn)
        motor_layout.addLayout(y_input_layout)

        self.motor_widget.setLayout(motor_layout)

        # 事件綁定
        self.y_publish_btn.clicked.connect(self.motor_y_send_command)
        self.x_plus_btn.clicked.connect(lambda: self.motor_x_move("x+", self.step_combo.currentData()[0]))
        self.x_minus_btn.clicked.connect(lambda: self.motor_x_move("x-", self.step_combo.currentData()[0]))
        self.y_plus_btn.clicked.connect(lambda: self.motor_y_move("y+", self.step_combo.currentData()[1]))
        self.y_minus_btn.clicked.connect(lambda: self.motor_y_move("y-", self.step_combo.currentData()[1]))
        self.yaw_plus_btn.clicked.connect(lambda: self.motor_yaw_move("yaw+", self.step_combo.currentData()[2]))
        self.yaw_minus_btn.clicked.connect(lambda: self.motor_yaw_move("yaw-", self.step_combo.currentData()[2]))
        self.home_btn.clicked.connect(self.motor_home)
        self.stop_btn.clicked.connect(self.motor_stop)
        self.debug_btn.clicked.connect(self.toggle_debug_state)

        # 將元件頁面加入堆疊
        self.component_stack.addWidget(self.motor_widget)
        self.component_stack.addWidget(self.fork_widget)
        self.component_stack.addWidget(self.gripper_widget)
        self.component_stack.addWidget(self.limit_widget)


        component_layout.addLayout(self.component_stack)
        component_group.setLayout(component_layout)
        component_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 按鈕事件綁定，切換頁面
        self.motor_btn.clicked.connect(lambda: self.component_stack.setCurrentWidget(self.motor_widget))
        self.fork_btn.clicked.connect(lambda: self.component_stack.setCurrentWidget(self.fork_widget))
        self.gripper_btn.clicked.connect(lambda: self.component_stack.setCurrentWidget(self.gripper_widget))
        self.limit_btn.clicked.connect(lambda: self.component_stack.setCurrentWidget(self.limit_widget))

        # ---------------- 主 Layout ----------------
        main_layout = QVBoxLayout()
        main_layout.addWidget(compensation_group, stretch=2)  # 補償控制區較大
        main_layout.addWidget(component_group, stretch=3)
        self.setLayout(main_layout)

        # Publisher
        self.task_cmd_pub = node.compensate_task_pub
        self.confirm_pub = node.confirm_pub
        self.pass_pub = node.pass_pub
        self.limit_pub = node.limit_pub
        self.fork_pub = node.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.motion_pub = node.create_publisher(MotionCmd, 'motion_cmd', 10)
        self.debug_pub = node.debug_pub
        
        # Subscriber
        node.create_subscription(Float32MultiArray, "/ui_target_pose", self.target_pose_callback, 10)
        node.create_subscription(Float32MultiArray, "/ui_compensate_value", self.compensate_value_callback, 10)
        node.create_subscription(TaskState, '/task_state_compensate', self.compensate_state_callback, 10)
        node.create_subscription(Int32, 'lr_distance', self.update_fork_current_height, 10)
        node.create_subscription(ForkState, 'fork_state', self.update_fork_state, 10)
        node.create_subscription(String, 'gripper_control_state', self.update_gripper_state, 10)
        node.create_subscription(String, 'limit_control_state', self.update_limit_state, 10)
        node.create_subscription(Bool, 'debug_mode_state', self.update_debug_state, 10)
        node.create_subscription(MH2State,'mh2_state', self.update_servo_state, 10)
        node.create_subscription(MotionState, 'motion_state', self.update_motion_state, 10)

        # 事件
        self.detect_btn.clicked.connect(lambda: self.send_detect_cmd("l_shape"))
        self.cancel_btn.clicked.connect(lambda: self.send_detect_cmd("stop"))

        self.z_btn.clicked.connect(lambda: self.send_z_comfirm("confirm"))
        self.x_btn.clicked.connect(lambda: self.send_x_comfirm("confirm"))
        self.yaw_btn.clicked.connect(lambda: self.send_yaw_comfirm("confirm"))

        self.limit_open_btn.clicked.connect(lambda: self.send_limit_cmd("open_limit"))
        self.limit_close_btn.clicked.connect(lambda: self.send_limit_cmd("close_limit"))
        self.limit_stop_btn.clicked.connect(lambda: self.send_limit_cmd("stop_limit"))
        self.limit_reset_btn.clicked.connect(lambda: self.send_limit_cmd("reset_limit"))

        self.gripper_open_btn.clicked.connect(lambda: self.send_gripper_cmd("open_gripper"))
        self.gripper_close_btn.clicked.connect(lambda: self.send_gripper_cmd("close_gripper"))
        self.gripper_stop_btn.clicked.connect(lambda: self.send_gripper_cmd("stop_gripper"))
        self.gripper_reset_btn.clicked.connect(lambda: self.send_gripper_cmd("reset_gripper"))


        self.pass_btn.clicked.connect(self.publish_pass_once)
        self.to_done_btn.clicked.connect(lambda: self.send_confirm_cmd("to_done"))

    # ========= Callback / 功能 =========
    def update_motion_state(self, msg: MotionState):
        if msg.motion_finish:
            self.step_combo.setStyleSheet("background-color: lightgreen; font-size: 12px;")
        elif msg.motion_finish == False:
            self.step_combo.setStyleSheet("background-color: lightgray; font-size: 12px;")

    def toggle_servo(self):
        """UI 點擊事件 -> 發布 servo_cmd，UI 狀態由 mh2_state 回報決定"""
        msg = String()
        if self.servo_state:  # 如果現在是 ON -> 發 OFF
            msg.data = "servo off"
        else:                 # 如果現在是 OFF/未知 -> 發 ON
            msg.data = "servo on"
        self.node.servo_pub.publish(msg)

    def update_servo_state(self, msg: MH2State):
        """更新 Servo 狀態 (由 Controller 發佈的狀態決定)"""
        self.servo_state = msg.servo_state
        if self.servo_state:
            self.servo_btn.setText("⚙️ Servo ON")
            self.servo_btn.setStyleSheet("background-color: lightgreen; font-size: 12px;")
        else:
            self.servo_btn.setText("⚙️ Servo OFF")
            self.servo_btn.setStyleSheet("background-color: lightgray; font-size: 12px;")

    # ---------------- 更新 yaw 值 ----------------    
    def compensate_value_callback(self, msg: Float32MultiArray):
        x, y, yaw, z = msg.data
        self.x_label.setText(f"X補償值: {x:.2f}")
        self.z_label.setText(f"Z補償值: {z:.2f}")
        self.yaw_label.setText(f"Yaw補償值: {yaw:.2f}")
        self.current_yaw = yaw  # 更新目前 yaw 值

    def compensate_state_callback(self, msg: TaskState):
        self.state = msg.state
        self.state_label.setText(f"狀態: {self.state}")

    def publish_pass_once(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]
        self.pass_pub.publish(msg)

    def send_detect_cmd(self, cmd: str):
        msg = TaskCmd()
        msg.mode = cmd
        self.task_cmd_pub.publish(msg)
        if cmd == "l_shape":
            self.pose_label.setText("目標位置: 偵測中...")
        elif cmd == "stop":
            self.pose_label.setText("目標位置: 偵測已取消")

    # def send_z_pose_cmd(self, z_text: str):
    #     try:
    #         z = float(z_text)
    #     except ValueError:
    #         self.pose_label.setText("Z補償: 輸入錯誤")
    #         return
    #     msg = Float32MultiArray()
    #     msg.data = [0.0, z]
    #     self.pose_cmd_pub.publish(msg)

    # def send_x_pose_cmd(self, x_text: str):
    #     try:
    #         x = float(x_text)
    #     except ValueError:
    #         self.pose_label.setText("X補償: 輸入錯誤")
    #         return
    #     msg = Float32MultiArray()
    #     msg.data = [x, 0.0]
    #     self.pose_cmd_pub.publish(msg)

    def send_z_comfirm(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.confirm_pub.publish(msg)

    def send_x_comfirm(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.confirm_pub.publish(msg)
    
    def send_yaw_comfirm(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.confirm_pub.publish(msg)

    def send_confirm_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.confirm_pub.publish(msg)
        if cmd == "to_done":
            self.pose_label.setText("目標位置: 強制完成補償")

    def update_limit_state(self, msg: String):
        self.limit_status_label.setText(f"狀態: {msg.data}")

    def send_limit_cmd(self, cmd: str):
        msg = LimitCmd()
        msg.mode = cmd
        self.limit_pub.publish(msg)

    def update_gripper_state(self, msg: String):
        self.gripper_status_label.setText(f"狀態: {msg.data}")

    def send_gripper_cmd(self, cmd: str):
        msg = GripperCmd()
        msg.mode = cmd
        self.node.gripper_pub.publish(msg)

    def target_pose_callback(self, msg: Float32MultiArray):
        x, y, yaw, z = msg.data
        self.pose_label.setText(f"目標位置: (x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}, z: {z:.2f})")      

    # 更新目前高度顯示
    def update_fork_current_height(self, msg: Int32):
        self.fork_current_label.setText(f"目前高度: {msg.data:.0f} mm")
        self.fork_current_height = float(msg.data)

    # 更新叉車狀態顯示
    def update_fork_state(self, msg: ForkState):
        self.fork_state_label.setText(f"狀態: {msg.state}") 

    # 發布輸入高度命令
    def fork_send_command(self):
        try:
            distance = float(self.fork_height_input.text())
            distance = max(50.0, min(1000.0, distance))  # 限制範圍在 50-1000 mm
        except ValueError:
            self.fork_cmd_label.setText("高度命令: 輸入錯誤")
            return
        self.fork_current_distance = distance
        self.fork_cmd_label.setText(f"高度命令: {distance:.0f} mm")
        msg = ForkCmd()
        msg.mode = "run"
        msg.speed = "slow"
        msg.direction = "up" if distance >= self.fork_current_distance else "down"
        msg.distance = distance
        self.fork_pub.publish(msg)

    # 上升 2mm
    def fork_increase(self):
        # 根據目前高度增加 step
        new_distance = min(1000.0, self.fork_current_height + self.fork_step)
        new_distance = max(50.0, new_distance)  # 確保不低於 50mm
        self.fork_current_distance = new_distance
        self.fork_cmd_label.setText(f"高度命令: {new_distance:.0f} mm")
        
        msg = ForkCmd()
        msg.mode = "run"
        msg.speed = "slow"
        msg.direction = "up"
        msg.distance = new_distance
        self.fork_pub.publish(msg)

    # 下降 2mm
    def fork_decrease(self):
        new_distance = max(50.0, self.fork_current_height - self.fork_step)
        new_distance = min(1000.0, new_distance)  # 確保不高於 1000mm
        self.fork_current_distance = new_distance
        self.fork_cmd_label.setText(f"高度命令: {new_distance:.0f} mm")
        
        msg = ForkCmd()
        msg.mode = "run"
        msg.speed = "slow"
        msg.direction = "down"
        msg.distance = new_distance
        self.fork_pub.publish(msg)
    # STOP
    def fork_stop(self):
        msg = ForkCmd()
        msg.mode = "stop"
        msg.speed = "slow"
        msg.direction = "stop"
        msg.distance = 0.0
        self.fork_pub.publish(msg)

    def motor_x_move(self, dir, step):
        if dir == "x+":
            delta = step   
        elif dir == "x-":
            delta = -step 
        else:
            delta = 0.0

        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_GOTO_RELATIVE
        msg.pose_data = [delta, 0.0, 0.0]  # X, Y, Z, Yaw(deg)
        msg.speed = 5.0
        self.motion_pub.publish(msg)
    
    def motor_y_move(self, dir, step):
        if dir == "y+":
            delta = step   
        elif dir == "y-":
            delta = -step 
        else:
            delta = 0.0

        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_GOTO_RELATIVE
        msg.pose_data = [0.0, delta, 0.0]  # X, Y, Z, Yaw(deg)
        msg.speed = 5.0
        self.motion_pub.publish(msg)
    
    def motor_yaw_move(self, dir, step):
        if dir == "yaw+":
            delta_yaw = step   # 每次旋轉 1度
        elif dir == "yaw-":
            delta_yaw = -step
        else:
            delta_yaw = 0.0

        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_GOTO_RELATIVE
        msg.pose_data = [0.0, 0.0,delta_yaw]  # X, Y, Z, Yaw(deg)
        msg.speed = 5.0
        self.motion_pub.publish(msg)

    def motor_y_send_command(self):
        try:
            y = float(self.y_input.text())
        except ValueError:
            return
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_Y_MOVE
        msg.pose_data = [0.0, y, 0.0]  # X, Y, Z, Yaw(deg)
        msg.speed = 20.0
        self.motion_pub.publish(msg)

    def motor_home(self):
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_HOME
        msg.pose_data = [0.0, 0.0, 0.0]  # X, Y, Z, Yaw(deg)
        msg.speed = 5.0
        self.motion_pub.publish(msg)
    
    def motor_stop(self):
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_STOP
        msg.pose_data = [0.0, 0.0, 0.0]  # X, Y, Z, Yaw(deg)
        msg.speed = 0.0
        self.motion_pub.publish(msg)

    def update_debug_state(self, msg: Bool):
        self.debug_state = msg.data
        self.update_button_style()

    def toggle_debug_state(self):
        """按鈕被點擊時切換狀態"""
        self.debug_state = not self.debug_state
        self.update_button_style()
        # 發布新的狀態
        msg = Bool()
        msg.data = self.debug_state
        self.debug_pub.publish(msg)

    def update_button_style(self):
        """根據 debug_state 改變按鈕外觀"""
        if self.debug_state:
            self.debug_btn.setStyleSheet(
                "font-size: 14px; font-weight: bold; background-color: green; color: white;"
            )
            self.debug_btn.setChecked(True)
        else:
            self.debug_btn.setStyleSheet(
                "font-size: 14px; font-weight: bold; background-color: lightgray; color: black;"
            )
            self.debug_btn.setChecked(False)

# ---------------- 視覺 Overlay ----------------
class VisualOverlay(QWidget):
    def __init__(self, node):
        super().__init__()
        self.setAttribute(Qt.WA_TransparentForMouseEvents)  # 允許滑鼠穿透
        self.setStyleSheet("background-color: rgba(0,0,0,200); color: white; font-size: 24px;")
        self.bridge = CvBridge()
        node.create_subscription(Image, '/visual_image', self.update_image, 10)
        self.label = QLabel("等待影像...", self)
        self.label.setAlignment(Qt.AlignCenter)

    def resizeEvent(self, event):
        self.label.setGeometry(0, 0, self.width(), self.height())

    def update_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)
            self.label.setPixmap(QPixmap.fromImage(qt_img).scaled(
                self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception as e:
            print("Image update error:", e)

# ---------------- Main Integrated UI ----------------
class IntegratedUI(QWidget):
    def __init__(self,node):
        super().__init__()
        
        self.light_pub = node.light_pub

        node.create_subscription(CurrentPose, '/current_pose', self.update_current_pose, 10)
        node.create_subscription(Float32MultiArray, 'depth_data', self.update_depth, 10)   
        node.create_subscription(Int32, 'lr_distance', self.update_height, 10)

        self.resize(1280, 799)
        self.setWindowTitle("Integrated UI")

        # --- 上方三 log ---
        log_layout = QHBoxLayout()
        self.log1 = QTextEdit(); self.log1.setReadOnly(True)
        # self.log2 = QTextEdit(); self.log2.setReadOnly(True)

        # log2 改成容器
        self.log2 = QWidget()
        log2_layout = QVBoxLayout()
        log2_layout.setContentsMargins(2, 2, 2, 2)
        log2_layout.setSpacing(2)

        # log3 改成容器
        self.log3 = QWidget()
        log3_layout = QVBoxLayout()
        log3_layout.setContentsMargins(2, 2, 2, 2)
        log3_layout.setSpacing(2)

        self.current_pose_label = QLabel("📍 Position:\nX: -- mm Y: -- mm Yaw: -- °")
        self.depth_label = QLabel("📏 Depth:\nLeft: -- mm Right: -- mm")
        self.height_label = QLabel("⬆️ Height:\nZ: -- mm")

        for lbl in [self.current_pose_label, self.depth_label, self.height_label]:
            lbl.setStyleSheet("font-size: 12px; font-family: Consolas;")  # 字體稍微縮小

        log3_layout.addWidget(self.current_pose_label)
        log3_layout.addWidget(self.depth_label)
        log3_layout.addWidget(self.height_label)
        self.log3.setLayout(log3_layout)

        # 設定可調寬度
        self.log3.setMinimumWidth(450)   # 最小寬度
        self.log3.setMaximumWidth(450)   # 最大寬度，可根據需要調整

        log_layout.addWidget(self.log1)
        log_layout.addWidget(self.log2)
        log_layout.addWidget(self.log3)

        # --- 下方 splitter ---
        self.splitter = QSplitter(Qt.Horizontal)

        self.ui1 = RecipeUI(node)
        self.ui1.log_callback = self.append_log
        self.ui2 = UI2(node)
        self.ui3 = UI3(node)

        # 包裝 UI1/UI2 到 container
        self.ui12_container = QWidget()
        container_layout = QHBoxLayout()
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(0)
        container_layout.addWidget(self.ui1)
        container_layout.addWidget(self.ui2)
        self.ui12_container.setLayout(container_layout)
        self.ui12_container.setFixedSize(800, 600)  # 固定左側大小

        # 添加到 splitter
        self.splitter.addWidget(self.ui12_container)
        self.splitter.addWidget(self.ui3)
        self.splitter.setSizes([800, 480])  # 左右比例

        # --- 視覺 overlay ---
        self.visual_overlay = VisualOverlay(node)
        self.visual_overlay.setParent(self.ui12_container)  # 指定 parent
        self.visual_overlay.setGeometry(0, 0, 800, 600)
        self.visual_overlay.setVisible(False)  # 初始隱藏

        # # --- 切換按鈕 ---
        # self.toggle_visual_btn = QPushButton("切換視覺畫面")
        # self.toggle_visual_btn.setCheckable(True)
        # self.toggle_visual_btn.clicked.connect(self.toggle_visual)

        # --- 切換按鈕 ---
        self.toggle_visual_btn = QPushButton("切換視覺畫面")
        self.toggle_visual_btn.setCheckable(True)
        self.toggle_visual_btn.clicked.connect(self.toggle_visual)

        # --- 新增光源按鈕 ---
        self.toggle_light_btn = QPushButton("開啟光源")
        self.toggle_light_btn.setCheckable(True)
        self.toggle_light_btn.clicked.connect(self.toggle_light)  # 你需要定義 toggle_light 函式

        # --- 按鈕水平佈局 ---
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.toggle_visual_btn)
        button_layout.addWidget(self.toggle_light_btn)

        # --- 主 Layout ---
        main_layout = QVBoxLayout()
        main_layout.addLayout(log_layout, 2)
        main_layout.addLayout(button_layout)
        main_layout.addWidget(self.splitter, 8)
        self.setLayout(main_layout)

    def toggle_visual(self, checked):
        self.visual_overlay.setVisible(checked)

    def toggle_light(self, checked):
        msg = String()
        if checked:
            msg.data = "light on"
            self.toggle_light_btn.setText("close光源")
        else:
            msg.data = "light off"
            self.toggle_light_btn.setText("open光源")
        
        self.light_pub.publish(msg)


    def append_log(self, log_id, text):
        if log_id == 1: self.log1.append(text)
        elif log_id == 2: self.log2.append(text)
        elif log_id == 3: self.log3.append(text)

    def update_current_pose(self, msg: CurrentPose):
        print("Received CurrentPose:", msg.pose_data)
        x = float(msg.pose_data[0])
        y = float(msg.pose_data[1])
        yaw = float(msg.pose_data[2] * 57.2958)
        #show in log3
        # 只更新對應區塊
        self.current_pose_label.setText(f"📍 Position:\nX: {x:.2f} mm Y: {y:.2f} mm Yaw: {yaw:.2f}°")
        # self.depth_label.setText(f"📏 Depth:\nLeft: {left_depth:.2f} mm\nRight: {right_depth:.2f} mm")

    def update_depth(self,msg: Float32MultiArray):
        left_depth = float(msg.data[0])
        right_depth = float(msg.data[1])
        self.depth_label.setText(f"📏 Depth:\nL: {left_depth:.2f} mm\nR: {right_depth:.2f} mm")

    def update_height(self,msg: Int32):
        z_height = float(msg.data)
        self.height_label.setText(f"⬆️ Height:\nZ: {z_height:.2f} mm")
        
# -------------------
def main(args=None):
    rclpy.init(args=args)
    node = RecipePublisher()

    app = QApplication(sys.argv)
    window = IntegratedUI(node)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
