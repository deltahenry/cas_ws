from PySide6.QtCore import QTimer
from common_msgs.msg import JogCmd, MotionCmd
from uros_interface.srv import ESMCmd
from rclpy.client import Client
from std_msgs.msg import String

class MotorController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.cli: Client = self.ros_node.create_client(ESMCmd, '/esm_command')
        self.waiting_for_result = False

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

        msg.speed = 20.0
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

    def send_y_motor_cmd(self, flag):
        msg = String()
        msg.data = flag

        self.ros_node.y_motor_cmd_publisher.publish(msg)
        print(f"[UI] Sent YMotor Cmd String: {msg.data}")



    def update_gui(self):
        # 每次更新也可以做其他檢查或顯示狀態
        if self.ros_node.waiting_for_result:
            self.status_label.setText("Waiting for service response...")
        else:
            self.status_label.setText("Ready")



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
