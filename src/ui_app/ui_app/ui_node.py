import sys
from datetime import date
import cv2
from PySide6.QtCore import Qt, QEvent, QTimer, Signal
from PySide6.QtGui import QIcon, QImage, QPixmap
from PySide6.QtWidgets import QApplication, QMainWindow

from ui_app.ui_magic_cube import Ui_MainWindow # Import my design made in Qt Designer (already in .py)
import ui_app.resources_rc  # this includes the images and icons

#ui components
from ui_app.ui_components.ui_forklift import ForkliftController
from ui_app.ui_components.ui_motor import MotorController

#Vision
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# for ROS2
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from common_msgs.msg import StateCmd, ForkCmd, JogCmd

class ROSNode(Node):
    def __init__(self):
        super().__init__('ui_node')

        self.mode_cmd = {
            'auto': True,
            'manual': False,
        }

        self.task_cmd = {
            'connect': False,
            'init': False,            
            'rough_align': False,
            'precise_align': False,
            'pick': False,
            'assem': False,
        }

        self.menu_cmd = {
            'main_page': True,
            'component_control': False
        }
        self.forklift_controller = None

        # Subscriber to the same topic
        # self.subscription = self.create_subscription(StateCmd, '/state_cmd', self.ros_message_callback, 10)

        # mode cmd
        self.mode_cmd_publisher = self.create_publisher(String, '/mode_cmd', 10)

        # publisher using custom StateCmd message
        self.state_cmd_publisher = self.create_publisher(StateCmd, '/state_cmd', 10)

        #task cmd
        self.task_cmd_publisher = self.create_publisher(String, '/task_cmd', 10)

        #fork lift
        self.fork_cmd_publisher = self.create_publisher(ForkCmd, '/fork_cmd', 10)

        self.height_info_subscriber = self.create_subscription(
            Int32,
            'lr_distance',
            self.height_info_callback,
            10
        )

        #jog control
        self.jog_cmd_publisher = self.create_publisher(JogCmd, '/jog_cmd', 10)

        #menu publishers
        self.component_control_publisher = self.create_publisher(String, '/run_cmd', 10)

        #vision publish
        self.vision_control_publisher = self.create_publisher(String, '/detection_task', 10)

        self.bridge = CvBridge()

        self.color_image_subscriber = self.create_subscription(
        Image,
        '/color_image',
        self.image_callback,
        10
        )

        # self.detection_task_subscriber = self.create_subscription(
        #     String,
        #     '/detection_task',
        #     self.detection_task_callback,
        #     10
        # )


        self.detection_task_callback_ui = None
        self.image_update_callback = None


    def height_info_callback(self, msg: Int32):
        self.get_logger().info(f"Received height info: {msg.data} mm")

        if self.forklift_controller:
            self.forklift_controller.update_height_display(msg.data)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # Pass to Qt
        if self.image_update_callback:
            self.image_update_callback(cv_img)

    def detection_task_callback(self, msg: String):
        if self.detection_task_callback_ui:
            self.detection_task_callback_ui(msg.data)




class MainWindow(QMainWindow):
    ros_msg_received = Signal(str)  # Thread-safe signal to update UI

    def __init__(self, ros_node):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        #Start ROS2 Node
        self.ros_node = ros_node

        self.ros_node.image_update_callback = self.update_image
        self.ros_node.detection_task_callback_ui = self.update_detection_mode


        #motor ui
        self.motor_controller = MotorController(self.ui, self.ros_node)

        #forklift ui
        self.forklift_controller = ForkliftController(self.ui, self.ros_node)
        self.ros_node.forklift_controller = self.forklift_controller
        
        

        # Connect Qt signal to UI handler
        self.ros_msg_received.connect(self.handle_ros_message)

        # connect buttons to send data to another node (ROS2)
        self.ui.RunButton.clicked.connect(lambda: self.send_state_cmd("run"))
        self.ui.StopButton.clicked.connect(lambda: self.send_state_cmd("stop"))
        self.ui.AutoResetButton.clicked.connect(lambda: self.send_state_cmd("reset"))

        self.ui.AutoButton.toggled.connect(self.on_auto_toggled)
        self.ui.ManualButton.toggled.connect(self.on_manual_toggled)


        self.ui.RoughAlignButton.clicked.connect(lambda: self.send_task_cmd("rough align"))
        self.ui.PreciseAlignButton.clicked.connect(lambda: self.send_task_cmd("precise align"))
        self.ui.PickButton.clicked.connect(lambda: self.send_task_cmd("pick"))
        self.ui.AssemblyButton.clicked.connect(lambda: self.send_task_cmd("assembly"))

        #VISION
        # self.ui.VisionOption.clicked.connect(lambda: self.send_detection_task(""))
        self.ui.VisionOne.toggled.connect(lambda checked: checked and self.send_vision_cmd("screw"))
        self.ui.VisionTwo.toggled.connect(lambda checked: checked and self.send_vision_cmd("l_shape"))
        self.ui.VisionThree.toggled.connect(lambda checked: checked and self.send_vision_cmd("icp_fit"))



        '''menu buttons'''

        #main page - publisher


        #component control - publisher
        self.ui.ComponentControlButton.clicked.connect(lambda: self.send_menu_cmd("component_control"))

        # Get Today's date
        today = date.today()
        formatted_date = today.strftime("%m/%d/%Y")
        self.ui.DateInput.setText(formatted_date)

        #By Default
        self.ui.ListOptionsWidget.setVisible(False)

        # Touchscreen style handlers in Main Page - Auto
        self.ui.RunButton.clicked.connect(lambda: self.on_touch_buttons(self.ui.RunButton))
        self.ui.StopButton.clicked.connect(lambda: self.on_touch_buttons(self.ui.StopButton))
        self.ui.AutoResetButton.clicked.connect(lambda: self.on_touch_buttons(self.ui.AutoResetButton))

        # self.ui.RecordDataButton.clicked.connect(lambda: self.on_record_data_clicked(self.ui.RecordDataButton))
        self.ui.ClipperButtonOnOff.toggled.connect(lambda: self.update_clipper_state(self.ui.ClipperButtonOnOff))

        #INIT or Stop
        self.ui.INITBefore.clicked.connect(self.change_to_action_buttons)

        #Auto or Manual
        self.ui.ChooseAutoButton.clicked.connect(lambda: self.choose_auto_or_manual(0))
        self.ui.ChooseManualButton.clicked.connect(lambda: self.choose_auto_or_manual(1))

        #choose your component control
        self.ui.ChooseMotor.clicked.connect(self.choose_motor)
        self.ui.ChooseVision.clicked.connect(self.choose_vision)
        self.ui.ChooseClipper.clicked.connect(self.choose_clipper)
        self.ui.ChooseForklift.clicked.connect(self.choose_forklift)

        #Vision
        self.ui.VisionTextInComponentControl.setFixedSize(640, 480)

        #Main Page, go to other pages
        self.ui.MainPageButton.clicked.connect(self.change_to_main_page)
        self.ui.ProductionRecordButton.clicked.connect(self.change_to_production_record_page)
        self.ui.ComponentControlButton.clicked.connect(self.change_to_component_control_page)
        self.ui.LogsButton.clicked.connect(self.change_to_logs_page)
        self.ui.SystemSettingsButton.clicked.connect(self.change_to_system_settings_page)

        # Main Page, Auto and Manual options
        self.ui.AutoButton.clicked.connect(self.change_to_auto_page)
        self.ui.ManualButton.clicked.connect(self.change_to_manual_page)

        #Component Control, List Menu
        self.ui.HamburgerMenu.clicked.connect(self.toggle_menu)

        self.ui.MotorOption.clicked.connect(lambda: self.component_control_switch_page("Motor", 0))
        # self.ui.VisionOption.clicked.connect(lambda: self.component_control_switch_page("Vision", 1))
        self.ui.VisionOption.clicked.connect(lambda: self.component_control_switch_page("Vision", 1))
        self.ui.ClipperOption.clicked.connect(lambda: self.component_control_switch_page("Clipper", 2))
        self.ui.ForkliftOption.clicked.connect(lambda: self.component_control_switch_page("Forklift", 3))

        

    def update_image(self, cv_img):
        qt_img = convert_cv_to_qt(cv_img)
        pixmap = QPixmap.fromImage(qt_img)
        self.ui.VisionTextInComponentControl.setPixmap(pixmap)  # Replace with your QLabel name

    def update_detection_mode(self, mode):
        print(f"[UI] Detection mode is now: {mode}")
        # Optionally update label or status bar


    #ROS2 Menu
    def send_menu_cmd(self, flag):
        msg = String()

        # msg.main_page = False
        msg.data = flag
        
        print(f"[DEBUG] Publishing MenuCmd: {msg}")
        self.ros_node.component_control_publisher.publish(msg)
        print(f"[UI] Sent MenuCmd: {flag}")


    #ROS2 Flag
    def send_state_cmd(self, flag):
        msg = StateCmd()

        # Reset all flags
        msg.pause_button = False
        msg.init_button = False
        msg.run_button = False
        msg.reset_button = False

        if flag == "init":
            msg.init_button = True
        elif flag == "run":
            msg.run_button = True
        elif flag == "stop":
            msg.pause_button = True
        elif flag == "reset":
            msg.reset_button = True

        print(f"[DEBUG] Publishing StateCmd: {msg}")
        self.ros_node.state_cmd_publisher.publish(msg)
        print(f"[UI] Sent StateCmd: {flag}")

    def send_mode_cmd(self, flag):
        msg = String()
        msg.data = flag

        self.ros_node.mode_cmd_publisher.publish(msg)
        print(f"[UI] Sent ModeCmd: {msg.data}")


    def send_task_cmd(self, flag):
        msg = String()
        msg.data = flag
        
        self.ros_node.task_cmd_publisher.publish(msg)
        print(f"[UI] Sent TaskCmd: {msg.data}")

    def send_jog_cmd(self, axis, direction):
        msg = JogCmd()
        msg.target = axis
        msg.direction = direction  # 1.0 for + direction, -1.0 for - direction
        msg.distance = 5.0         # fixed distance
        msg.speed = 50.0           # fixed speed
        self.ros_node.jog_cmd_publisher.publish(msg)
        print(f"[UI] Sent JogCmd: axis={axis}, direction={direction}")

    def update_clipper_state(self, button):
        if button.isChecked():
            button.setText("Clipper ON")
        else:
            button.setText("Clipper OFF")

    def send_vision_cmd(self, mode):
        print(f"[DEBUG] Trying to publish: {mode}")  

        if hasattr(self, 'last_vision_mode') and self.last_vision_mode == mode:
            return
        self.last_vision_mode = mode
        msg = String()
        msg.data = mode
        self.ros_node.vision_control_publisher.publish(msg)
        print(f"[UI] Sent VisionCmd: {mode}")

    
    #Principal Menu - StackedWidget
    def change_to_main_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(0)

    def change_to_component_control_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(1)

    def change_to_production_record_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(2)

    def change_to_logs_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(3)

    def change_to_system_settings_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(4)

    #INIT or Stop
    def change_to_action_buttons(self):
        self.ui.AutoAndManualStackedWidget.setCurrentIndex(1)

    #choose Auto or Manual
    def choose_auto_or_manual(self, autoOrManualIndex):
        self.ui.AutoAndManualStackedWidget.setCurrentIndex(2)
        self.ui.ActionButtons.setCurrentIndex(autoOrManualIndex)

        if autoOrManualIndex == 0:
            self.ui.AutoButton.setChecked(True)

        elif autoOrManualIndex == 1:
            self.ui.ManualButton.setChecked(True)
    
    #choose your component control
    def choose_motor(self):
        self.ui.ComponentControlStackedWidget.setCurrentIndex(1)
        self.ui.ChangeComponentControlStackedWidget.setCurrentIndex(0)
        self.ui.MotorStartedButton.setText("Motor")

    def choose_vision(self):
        self.ui.ComponentControlStackedWidget.setCurrentIndex(1)
        self.ui.ChangeComponentControlStackedWidget.setCurrentIndex(1)
        self.ui.MotorStartedButton.setText("Vision")


    def choose_clipper(self):
        self.ui.ComponentControlStackedWidget.setCurrentIndex(1)
        self.ui.ChangeComponentControlStackedWidget.setCurrentIndex(2)
        self.ui.MotorStartedButton.setText("Clipper")


    def choose_forklift(self):
        self.ui.ComponentControlStackedWidget.setCurrentIndex(1)
        self.ui.ChangeComponentControlStackedWidget.setCurrentIndex(3)
        self.ui.MotorStartedButton.setText("Forklift")



    # Auto or Manual Buttons
    def change_to_auto_page(self):
        self.ui.ActionButtons.setCurrentIndex(0)
    
    def change_to_manual_page(self):
        self.ui.ActionButtons.setCurrentIndex(1)
        

    #
    def on_auto_toggled(self, checked):
        if checked:
            self.send_mode_cmd("auto")

    def on_manual_toggled(self, checked):
        if checked:
            self.send_mode_cmd("manual")


    #Component Control - Motor
    def toggle_menu(self):
        self.ui.ListOptionsWidget.setVisible(not self.ui.ListOptionsWidget.isVisible())

    def component_control_switch_page(self, name, index):
        self.ui.MotorStartedButton.setText(name)
        self.ui.ChangeComponentControlStackedWidget.setCurrentIndex(index)
        self.ui.ListOptionsWidget.setVisible(False)


    #Vision


    # def activate_vision_view(self):
        # Switch to Vision page
        # self.choose_vision()

        # Optional: Manually refresh or send trigger to vision system
        # print("[UI] Vision page activated — ready to receive image")

        # Optionally send a ROS signal to start detection (if needed)
        # self.send_detection_task("start")  # <-- if you want to publish to /detection_task



    def on_touch_controls(self, button):
        # 1. Visual press feedback
        button.setStyleSheet("""
        QPushButton {
            background-color: rgba(11, 118, 160, 0.3); /* soft blue overlay */
            border: 1px solid #0B76A0;
            color: white;
        }
        """)
        
        # 2. Reset after 200ms
        QTimer.singleShot(200, lambda: button.setStyleSheet("""
        QPushButton {
            background-color: transparent;
            border: none;
            color: white;
        }
        """))

    def on_touch_buttons(self, button):
        # 1. Visual press feedback
        button.setStyleSheet("""
        QPushButton {
            background-color: rgba(11, 118, 160, 0.3); /* soft blue overlay */
            border: 1px solid #0B76A0;
            color: white;
        }
        """)
        
        # 2. Reset after 200ms
        QTimer.singleShot(200, lambda: button.setStyleSheet("""
        QPushButton {
            color: white;
        }
        """))


    def on_ros_message(self, text):
        self.ros_msg_received.emit(text)

    def handle_ros_message(self, text):
        print("ROS message:", text)
        # Update UI labels here if needed

    def closeEvent(self, event):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

    def move_to_second_screen_and_fullscreen(self):
        screens = QApplication.screens()
        if len(screens) > 1:
            second_screen = screens[1]  # Use the actual second screen
            second_geom = second_screen.geometry()
            self.setGeometry(second_geom)  # Move and resize in one step
            self.showFullScreen()
            print("Moved to second screen and fullscreen")
        else:
            self.showMaximized()
            print("Only one screen, maximized")



def convert_cv_to_qt(cv_img):
    rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb_image.shape
    bytes_per_line = ch * w
    return QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)



def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    ros_node = ROSNode()

    app = QApplication(sys.argv)

    window = MainWindow(ros_node)

    # Delay moving and fullscreen until after show()
    QTimer.singleShot(100, window.move_to_second_screen_and_fullscreen)

    threading.Thread(target=ros_spin, args=(ros_node,), daemon=True).start()

    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
