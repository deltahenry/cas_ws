import sys
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,QLineEdit, QHBoxLayout
from PySide6.QtCore import QTimer, Qt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uros_interface.srv import ESMCmd
from common_msgs.msg import MotionCmd,MultipleM, SingleM,StateCmd
from std_msgs.msg import Bool


class RosNode(Node):
    def __init__(self):
        super().__init__('ros_gui_node')

        # Publisher for testing
        self.mode_cmd_publisher = self.create_publisher(String, '/mode_cmd', 10)
        self.state_cmd_publisher = self.create_publisher(StateCmd, '/state_cmd', 10)
        self.task_trigger_publisher = self.create_publisher(String, '/task_trigger', 10)

        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)

        self.tcp_stream_pub = self.create_publisher(Bool, '/start_tcp_stream', 10)

        # Subscriber for testing
        self.test_sub = self.create_subscription(String, '/gui_test', self.test_callback, 10)
        self.motors_info_sub = self.create_subscription(MultipleM,'/multi_motor_info',self.motors_info_callback,10)

        # Async service client
        self.cli = self.create_client(ESMCmd, '/esm_command')
        self.waiting_for_result = False

        # 初始化機器人模型的電機長度
        self.current_motor_len = [10.0, 0.0, 0.0]

        # 初始化狀態命令
        self.state_cmd = {
            'init_button': False,
            'run_button': False,   
            'pause_button': False,
            'reselect_button': False,
        }

    def test_callback(self, msg):
        self.get_logger().info(f"[GUI SUBSCRIBER] Received: {msg.data}")

    def motors_info_callback(self, msg:MultipleM):
        self.current_motor_len = [msg.motor_info[0].fb_position,msg.motor_info[1].fb_position,msg.motor_info[2].fb_position]
        print("motor_info callback",self.current_motor_len)

    def call_servo(self, on=True):
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service not available')
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
                self.get_logger().info(f"Service {'ON' if on else 'OFF'} call succeeded")
            else:
                self.get_logger().error(f"Service {'ON' if on else 'OFF'} call failed")

        future.add_done_callback(handle_response)


class MyGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()

        # GUI 定時更新（非阻塞 ROS）
        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self.update_gui)
        self.frame_timer.start(500)

        self.tcp_stream_on = False  # 用來記錄狀態


    def init_ui(self):
        self.setWindowTitle("ROS2 Non-blocking GUI")
        self.setGeometry(100, 100, 400, 300)
        layout = QVBoxLayout()

        self.status_label = QLabel("Press button to send service", self)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # TCP Stream 切換按鈕
        self.toggle_tcp_btn = QPushButton("Start TCP Stream")
        self.toggle_tcp_btn.clicked.connect(self.toggle_tcp_stream)
        layout.addWidget(self.toggle_tcp_btn)   

        # Servo ON/OFF Buttons
        self.btn_on = QPushButton("Servo ON", self)
        self.btn_on.clicked.connect(lambda: self.ros_node.call_servo(True))
        layout.addWidget(self.btn_on)

        self.btn_off = QPushButton("Servo OFF", self)
        self.btn_off.clicked.connect(lambda: self.ros_node.call_servo(False))
        layout.addWidget(self.btn_off)

        # 新增按鈕
        self.run_button = QPushButton("Run")
        self.run_button.clicked.connect(self.send_run_cmd)
        layout.addWidget(self.run_button)

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self.send_pause_cmd)
        layout.addWidget(self.pause_button)

        self.init_button = QPushButton("Init")
        self.init_button.clicked.connect(self.send_init_task_cmd)
        layout.addWidget(self.init_button)

        self.auto_button = QPushButton("Auto")
        self.auto_button.clicked.connect(self.send_auto_mode_cmd)
        layout.addWidget(self.auto_button)

        self.manual_button = QPushButton("Manual")
        self.manual_button.clicked.connect(self.send_manual_mode_cmd)
        layout.addWidget(self.manual_button)

        self.setLayout(layout)
    
    def publish_motor_cmd(self):
        
        m1 = float(self.m1_input.text())
        m2 = float(self.m2_input.text())
        m3 = float(self.m3_input.text())/57.2958  # Convert degrees to radians
        speed = float(self.speed_input.text())

        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_GOTO
        msg.pose_data = [m1, m2, m3]
        msg.speed = speed
        self.ros_node.motion_cmd_publisher.publish(msg)

    def toggle_tcp_stream(self):
        self.tcp_stream_on = not self.tcp_stream_on
        msg = Bool()
        msg.data = self.tcp_stream_on
        self.ros_node.tcp_stream_pub.publish(msg)

        if self.tcp_stream_on:
            self.toggle_tcp_btn.setText("Stop TCP Stream")
        else:
            self.toggle_tcp_btn.setText("Start TCP Stream")

    def send_init_task_cmd(self):
        msg = String()
        msg.data = "init"
        self.ros_node.task_trigger_publisher.publish(msg)
    
    def send_run_cmd(self):
        msg = StateCmd()
        self.ros_node.state_cmd['run_button'] = True
        msg.init_button = self.ros_node.state_cmd['init_button']
        msg.run_button = self.ros_node.state_cmd['run_button']
        msg.pause_button = self.ros_node.state_cmd['pause_button']
        self.ros_node.state_cmd_publisher.publish(msg)
    
    def send_pause_cmd(self):
        msg = StateCmd()
        self.ros_node.state_cmd['pause_button'] = True
        msg.init_button = self.ros_node.state_cmd['init_button']
        msg.run_button = self.ros_node.state_cmd['run_button']
        msg.pause_button = self.ros_node.state_cmd['pause_button']
        self.ros_node.state_cmd_publisher.publish(msg)

    def send_auto_mode_cmd(self):
        msg = String()
        msg.data = "auto"
        self.ros_node.mode_cmd_publisher.publish(msg)
    
    def send_manual_mode_cmd(self):
        msg = String()
        msg.data = "manual"
        self.ros_node.mode_cmd_publisher.publish(msg)

    # def send_init_cmd(self):
        
    #     m1 = 0.0
    #     m2 = 0.0
    #     m3 = 0.0
    #     speed = 50.0

    #     msg = MotionCmd()
    #     msg.command_type = MotionCmd.TYPE_HOME
    #     msg.pose_data = [m1, m2, m3]
    #     msg.speed = speed
    #     self.ros_node.motion_cmd_publisher.publish(msg)
            
    def update_gui(self):
        # 每次更新也可以做其他檢查或顯示狀態
        if self.ros_node.waiting_for_result:
            self.status_label.setText("Waiting for service response...")
        else:
            self.status_label.setText("Ready")
    


def main():
    rclpy.init()

    ros_node = RosNode()
    app = QApplication(sys.argv)
    gui = MyGUI(ros_node)
    gui.show()

    # ROS spin with QTimer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    ros_timer.start(10)  # ROS 每 10 ms 處理一次 callback

    exit_code = app.exec()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
