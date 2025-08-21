from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPixmap
from common_msgs.msg import JogCmd, MotionCmd, MultipleM, MH2State
from uros_interface.srv import ESMCmd
from rclpy.client import Client
from std_msgs.msg import String


'''Use this if needs to be scaled down'''

# import os
# os.environ["QT_SCALE_FACTOR"] = "0.95"

# from PySide6.QtGui import QGuiApplication
# from PySide6.QtCore import Qt
# QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
#     Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
# )

class MotorController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.cli: Client = self.ros_node.create_client(ESMCmd, '/esm_command')
        self.waiting_for_result = False

        

        self._light_colors = {
            "red_on":    "#FF4D4D",
            "red_off":   "#660000",
            "yellow_on": "#FFEB3B",
            "yellow_off":"#666633",
            "green_on":  "#6FCF53",
            "green_off": "#336633",
        }

        self._set_lights(on_red=False, on_yellow=False, on_green=False)

        self.num: float | None = None
        self.speed: float | None = None

        # def _prepare_lamp(lbl):
        #     lbl.setPixmap(QPixmap())                # ensure no image covers the bg
        #     lbl.setText("")                         # no text overlay
        #     # lbl.setAttribute(Qt.WA_StyledBackground, True)
        #     # lbl.setFixedSize(size, size)            # consistent circle size (optional)
        #     # lbl.setStyleSheet(f"border-radius:{size//2}px;")  # circle


        #     lbl.setStyleSheet("border-radius:10px;")

        # # prepare once
        # _prepare_lamp(self.ui.RedSignal)
        # _prepare_lamp(self.ui.YellowSignal)
        # _prepare_lamp(self.ui.GreenSignal)

        # Disable buttons until service is ready
        # self.ui.ServoON.setEnabled(False)
        # self.ui.ServoOFF.setEnabled(False)

        # # Poll for readiness (non-blocking)
        # self._srv_timer = QTimer()
        # self._srv_timer.timeout.connect(self._check_srv_ready)
        # self._srv_timer.start(200)  # check 5x/sec

        # Connect UI buttons to jog commands
        # Buttons: pass only the SIGN; we’ll compute distance/angle inside.

        self.ui.ControlUpCP.setAutoRepeat(True)
        self.ui.ControlUpCP.setAutoRepeatDelay(300)
        self.ui.ControlUpCP.setAutoRepeatInterval(80) 

        self.ui.ControlDownCP.setAutoRepeat(True)
        self.ui.ControlDownCP.setAutoRepeatDelay(300)
        self.ui.ControlDownCP.setAutoRepeatInterval(80) 

        self.ui.ControlLeftCP.setAutoRepeat(True)
        self.ui.ControlLeftCP.setAutoRepeatDelay(300)
        self.ui.ControlLeftCP.setAutoRepeatInterval(80) 

        self.ui.ControlRightCP.setAutoRepeat(True)
        self.ui.ControlRightCP.setAutoRepeatDelay(300)
        self.ui.ControlRightCP.setAutoRepeatInterval(80) 

        # self.ui.RedSignal.setStyleSheet("background-color: red")
        # self.ui.YellowSignal.setStyleSheet("background-color: yellow")

        # self.ui.GreenSignal.setStyleSheet("background-color: green")


        self.ui.ControlUpCP.clicked.connect(  lambda: self.send_jog_cmd("y_axis",  +1))
        self.ui.ControlDownCP.clicked.connect(lambda: self.send_jog_cmd("y_axis",  -1))
        self.ui.ControlLeftCP.clicked.connect(lambda: self.send_jog_cmd("x_axis",  -1))
        self.ui.ControlRightCP.clicked.connect(lambda: self.send_jog_cmd("x_axis", +1))
        self.ui.YawPlusCP.clicked.connect(    lambda: self.send_jog_cmd("yaw_axis", +1))
        self.ui.YawMinusCP.clicked.connect(   lambda: self.send_jog_cmd("yaw_axis", -1))
        


        self.ui.HomeMotor.clicked.connect(self.send_init_cmd)

        self.ui.ControlUpCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlUpCP))
        self.ui.ControlDownCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlDownCP))
        self.ui.ControlLeftCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlLeftCP))
        self.ui.ControlRightCP.clicked.connect(lambda: self.on_touch_controls(self.ui.ControlRightCP))
        self.ui.YawPlusCP.clicked.connect(lambda: self.on_touch_controls(self.ui.YawPlusCP))
        self.ui.YawMinusCP.clicked.connect(lambda: self.on_touch_controls(self.ui.YawMinusCP))

        self.ui.HomeMotor.clicked.connect(lambda: self.on_touch_buttons(self.ui.HomeMotor))
        self.ui.StopMotor.clicked.connect(lambda: self.on_touch_buttons(self.ui.StopMotor))

        self.ui.ServoON.clicked.connect(lambda: self.call_servo(True))  
        self.ui.ServoOFF.clicked.connect(lambda: self.call_servo(False))  

        self.ui.HomeY.clicked.connect(lambda: self.send_y_motor_cmd("home_y"))
        self.ui.ReadyY.clicked.connect(lambda: self.send_y_motor_cmd("ready_y"))
        self.ui.AssemblyY.clicked.connect(lambda: self.send_y_motor_cmd("assembly_y"))

        self.ui.InputMotorDistance.textChanged.connect(self.on_distance_changed)
        self.ui.InputMotorSpeed.textChanged.connect(self.on_speed_changed)

        self.ui.SendYCommand.clicked.connect(self.send_y_command)


        self.motor_distance_state = 0  # 0: first, 1: second, 2: third
        self.selected_distance = 1.0
        self.selected_deg = 0.5
        self.ui.MotorChooseDistance.setText("1 mm - 0.5°")

        self.ui.MotorChooseDistance.clicked.connect(self.on_motor_distance_clicked)


    def on_motor_distance_clicked(self):
        self.motor_distance_state = (self.motor_distance_state + 1) % 3

        if self.motor_distance_state == 0:
            self.selected_distance = 1.0
            self.selected_deg = 0.5
            self.ui.MotorChooseDistance.setText("1 mm - 0.5°")
        elif self.motor_distance_state == 1:
            self.selected_distance = 5.0
            self.selected_deg = 1.0
            self.ui.MotorChooseDistance.setText("5 mm - 1°")
        elif self.motor_distance_state == 2:
            self.selected_distance = 10.0
            self.selected_deg = 5.0
            self.ui.MotorChooseDistance.setText("10 mm - 5°")

        print(f"[UI] Motor distance set to {self.selected_distance} mm, {self.selected_deg}°")


    def send_jog_cmd(self, axis, sign):
        # sign is +1 or -1 (int); keep distance/angle signed as you requested
        msg = JogCmd()
        msg.target = axis
        msg.direction = float(1 if sign >= 0 else -1)  # exactly ±1.0

        if axis == "yaw_axis":
            msg.angle = self.selected_deg     # ±0.5 / ±1 / ±5
            msg.distance = 0.0
        else:
            msg.distance = self.selected_distance  # ±1 / ±5 / ±10
            msg.angle = 0.0

        msg.speed = 5.0
        self.ros_node.jog_cmd_publisher.publish(msg)
        print(f"[Motor] JogCmd -> target={msg.target}, dir={msg.direction}, dist={msg.distance}, ang={msg.angle}")

    def send_init_cmd(self):
        m1 = 0.0
        m2 = 0.0
        m3 = 0.0
        speed = 5.0

        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_HOME
        msg.pose_data = [m1, m2, m3]
        msg.speed = speed
        self.ros_node.motion_cmd_publisher.publish(msg)
        print(f"[Home]: \n Command Type: {msg.command_type} \n Pose Data: {msg.pose_data} \n Speed: {msg.speed}")

    # def on_mh2_state(self, msg: MH2State):
    #     """Called from Qt thread (via singleShot)."""
    #     self._apply_servo_ui(msg.servo_state)
    #     self._apply_alarm_ui(msg.alarm_code)

    # def _apply_servo_ui(self, is_on: bool):
    #     # Keep your original styles; just add a semi-transparent overlay
    #     if is_on:
    #         self.ui.ServoON.setStyleSheet(
    #             self.ui.ServoON.styleSheet() + "\nQPushButton { border: 2px solid yellow; }"
    #         )
    #         self.ui.ServoOFF.setStyleSheet(self.ui.ServoOFF.styleSheet())
    #     else:
    #         self.ui.ServoOFF.setStyleSheet(
    #             self.ui.ServoOFF.styleSheet() + "\nQPushButton { border: 2px solid yellow; }"
    #         )
    #         self.ui.ServoON.setStyleSheet(self.ui.ServoON.styleSheet())

    # def _apply_alarm_ui(self, code: int):
    #     if code == 0:
    #         # normal: no change, keep default
    #         self.ui.AlarmButton.setStyleSheet(self.ui.AlarmButton.styleSheet())
    #     else:
    #         # highlight the alarm button to show it's active
    #         self.ui.AlarmButton.setStyleSheet(
    #             self.ui.AlarmButton.styleSheet() + "\nQPushButton { border: 2px solid yellow; }"
    #         )

    def _on_mh2_state_ui(self, servo_on: bool, alarm_code: int):
        btn = self.ui.ServoONOFFButton
        btn.blockSignals(True)
        btn.setChecked(servo_on)
        btn.setText("Servo ON" if servo_on else "Servo OFF")
        btn.blockSignals(False)
        btn.setEnabled(True)

        # Alarm UI
        if hasattr(self.ui, "AlarmButton"):
            self.ui.AlarmButton.setText(f"Alarm: {alarm_code}")
            # optional coloring: 0 = ok (green-ish), else = warn (yellow) / error (red)
            if alarm_code == 0:
                self.ui.AlarmButton.setStyleSheet("background-color: black; color: white;")   # green
            else:
                self.ui.AlarmButton.setStyleSheet("background-color: #FFEB3B; color:black;")   # yellow (or red)
        else:
            print("[UI] AlarmButton not found - check your .ui objectName")

    def call_servo(self, on=True):
        if not self.cli.service_is_ready():
            self.ros_node.get_logger().warn('Service not available')
            return

        request = ESMCmd.Request()
        request.servo_status = on
        request.mode = 4
        request.speed_limit = 50
        request.lpf = 10

        future = self.cli.call_async(request)
        self.waiting_for_result = True

        def handle_response(fut):
            self.waiting_for_result = False
            if fut.result() is not None:
                self.ros_node.get_logger().info(f"Service {'ON' if on else 'OFF'} call succeeded")
            else:
                self.ros_node.get_logger().error(f"Service {'ON' if on else 'OFF'} call failed")

        future.add_done_callback(handle_response)

    # def send_y_motor_cmd(self, flag):
    #     msg = String()
    #     msg.data = flag

    #     self.ros_node.y_motor_cmd_publisher.publish(msg)
    #     print(f"[UI] Sent YMotor Cmd String: {msg.data}")

    def current_pose(self, x: float, y: float, yaw_deg: float):
        """
        Slot to handle current pose updates from ROS and show them on the UI.
        Connected in MainWindow:
            self.current_pose_update.connect(self.motor_controller.current_pose)
        """
        # Defensive: make sure the labels exist and have setText
        if hasattr(self.ui, "xPos"):
            self.ui.xPos.setText(f"{x:.2f}")  
        if hasattr(self.ui, "yPos"):
            self.ui.yPos.setText(f"{y:.2f}")
        if hasattr(self.ui, "yawPos"):
            self.ui.yawPos.setText(f"{yaw_deg:.2f}")  # degrees with 2 decimal places


    def send_y_motor_cmd(self, flag):
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_Y_MOVE

        if flag == "home_y":
            print("[UI] Sending YMotor Cmd: Home")
            msg.pose_data = [0.0, 0.0, 0.0]  # Y軸歸零

        elif flag == "ready_y":
            print("[UI] Sending YMotor Cmd: Ready")
            msg.pose_data = [0.0, 500.0, 0.0]

        elif flag == "assembly_y":
            print("[UI] Sending YMotor Cmd: Assembly")
            msg.pose_data = [0.0, 1100.0, 0.0]

        else:
            print(f"[UI] Unknown YMotor command: {flag}")
            return
        
        msg.speed = 20.0  # 可以根據需要調整速度
        self.ros_node.motion_cmd_publisher.publish(msg)
        print(f"[YMotor] Command Type: {msg.command_type}, Pose Data: {msg.pose_data}, Speed: {msg.speed}")

    def on_distance_changed(self, text: str):
        try:
            self.num = float(text)
        except ValueError:
            self.num = 0.0

    def on_speed_changed(self, text: str):
        text = text.strip()
        if text == "":
            self.speed = None    # nothing entered yet
        else:
            try:
                self.speed = float(text)
            except ValueError:
                self.speed = None

    def send_y_command(self):
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_Y_MOVE

        y_val = self.num if self.num is not None else 0.0
        spd   = self.speed if self.speed is not None else 10.0

        msg.pose_data = [0.0, y_val, 0.0]   
        msg.speed = spd

        self.ros_node.motion_cmd_publisher.publish(msg)
        print(f"[YMotor] Command Type: {msg.command_type}, Pose Data: {msg.pose_data}, Speed: {msg.speed}")

    

    def update_gui(self):
        # 每次更新也可以做其他檢查或顯示狀態
        if self.ros_node.waiting_for_result:
            self.status_label.setText("Waiting for service response...")
        else:
            self.status_label.setText("Ready")


    def on_motor_info(self, m1: float, m2: float, m3: float):
        self.ui.MotorM1.setText(f"M1: {m1:.2f}")
        self.ui.MotorM2.setText(f"M2: {m2:.2f}")
        self.ui.MotorM3.setText(f"M3: {m3:.2f}")


    def _paint_light(self, lbl, on_color, off_color, is_on):
        color = on_color if is_on else off_color
        lbl.setStyleSheet(f"background-color:{color}; border-radius:25px;")



    def _set_lights(self, on_red: bool, on_yellow: bool, on_green: bool):
        c = self._light_colors
        self._paint_light(self.ui.RedSignal,    c["red_on"],    c["red_off"],    on_red)
        self._paint_light(self.ui.YellowSignal, c["yellow_on"], c["yellow_off"], on_yellow)
        self._paint_light(self.ui.GreenSignal,  c["green_on"],  c["green_off"],  on_green)
        

    def apply_motion_state(self, init_done: bool, motion_done: bool):
        if init_done and motion_done:
            r, y, g = False, False, True
        elif init_done and not motion_done:
            r, y, g = False, True,  False
        elif not init_done and not motion_done:
            r, y, g = True,  False, False
        else:
            r, y, g = False, True,  False
        self._set_lights(r, y, g)  # now on GUI thread



    def on_touch_controls(self, button):
        button.setStyleSheet("""
        QPushButton {
            background-color: rgba(11, 118, 160, 0.3);
            border: 1px solid #0B76A0;
            color: white;
        }
        """)
        QTimer.singleShot(200, lambda: button.setStyleSheet("""
        QPushButton {
            background-color: transparent;
            border: none;
            color: white;
        }
        """))

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
