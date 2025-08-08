from PySide6.QtCore import QTimer
from common_msgs.msg import JogCmd, MotionCmd
from uros_interface.srv import ESMCmd
from rclpy.client import Client

class MotorController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        self.cli: Client = self.ros_node.create_client(ESMCmd, '/esm_command')
        self.service_ready = False

        # Disable buttons until service is ready
        self.ui.ServoON.setEnabled(False)
        self.ui.ServoOFF.setEnabled(False)

        # Poll for readiness (non-blocking)
        self._srv_timer = QTimer()
        self._srv_timer.timeout.connect(self._check_srv_ready)
        self._srv_timer.start(200)  # check 5x/sec

        # Connect UI buttons to jog commands
        self.ui.ControlUpCP.clicked.connect(lambda: self.send_jog_cmd("y_axis", 1.0))
        self.ui.ControlDownCP.clicked.connect(lambda: self.send_jog_cmd("y_axis", -1.0))
        self.ui.ControlLeftCP.clicked.connect(lambda: self.send_jog_cmd("x_axis", -1.5))
        self.ui.ControlRightCP.clicked.connect(lambda: self.send_jog_cmd("x_axis", 1.5))
        self.ui.YawPlusCP.clicked.connect(lambda: self.send_jog_cmd("yaw", 0.5))
        self.ui.YawMinusCP.clicked.connect(lambda: self.send_jog_cmd("yaw", -0.5))

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

    def _check_srv_ready(self):
        if self.cli.wait_for_service(timeout_sec=0.0):
            self.service_ready = True
            self.ui.ServoON.setEnabled(True)
            self.ui.ServoOFF.setEnabled(True)
            self.ros_node.get_logger().info('/esm_command is READY')
            self._srv_timer.stop()
        else:
            self.ros_node.get_logger().info('Waiting for /esm_command...')



    def send_jog_cmd(self, axis, direction):
        msg = JogCmd()
        msg.target = axis
        msg.direction = direction
        msg.distance = 5.0
        msg.speed = 50.0
        self.ros_node.jog_cmd_publisher.publish(msg)
        print(f"[Motor] Sent JogCmd: axis={axis}, direction={direction}")

    def send_init_cmd(self):
        m1 = 0.0
        m2 = 0.0
        m3 = 0.0
        speed = 50.0

        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_HOME
        msg.pose_data = [m1, m2, m3]
        msg.speed = speed
        self.ros_node.motion_cmd_publisher.publish(msg)

    def call_servo(self, on=True):
        if not self.cli.wait_for_service(timeout_sec=0.2):
            self.ros_node.get_logger().warn('Service not available')
            return

        req = ESMCmd.Request()
        req.servo_status = on
        req.mode = 4
        req.speed_limit = 50
        req.lpf = 10

        future = self.cli.call_async(req)
        self.waiting_for_result = True

        def handle_response(fut):
            self.waiting_for_result = False
            exc = fut.exception()
            if exc is not None:
                self.ros_node.get_logger().error(f"Service {'ON' if on else 'OFF'} failed: {exc}")
                return
            self.ros_node.get_logger().info(f"Service {'ON' if on else 'OFF'} call succeeded")

        future.add_done_callback(handle_response)




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
