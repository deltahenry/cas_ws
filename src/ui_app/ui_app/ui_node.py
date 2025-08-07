import sys
from datetime import date
import cv2
from PySide6.QtCore import Qt, QEvent, QTimer, Signal
from PySide6.QtGui import QIcon, QImage, QPixmap
from PySide6.QtWidgets import QApplication, QMainWindow, QButtonGroup, QScroller

from ui_app.ui_magic_cube import Ui_MainWindow # Import my design made in Qt Designer (already in .py)
import ui_app.resources_rc  # this includes the images and icons

#ui components
from ui_app.ui_components.ui_forklift import ForkliftController
from ui_app.ui_components.ui_motor import MotorController
from ui_app.ui_components.ui_dido import DIDOController

#Vision
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# for ROS2
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from common_msgs.msg import StateCmd, ForkCmd, JogCmd, ComponentCmd, TaskCmd, TaskState, DIDOCmd

class ROSNode(Node):
    def __init__(self):
        super().__init__('ui_node')


        self.mode_cmd = {
            'auto': True,
            'manual': False,
            'component_control': False,
            
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

        # publisher
        self.mode_cmd_publisher = self.create_publisher(String, '/mode_cmd', 10)

        self.state_cmd_publisher = self.create_publisher(StateCmd, '/state_cmd', 10)

        self.component_cmd_publisher = self.create_publisher(ComponentCmd, "/component_control_cmd", 10)

        self.task_cmd_publisher = self.create_publisher(TaskCmd, '/task_cmd', 10)

        self.fork_cmd_publisher = self.create_publisher(ForkCmd, '/fork_cmd', 10)

        self.jog_cmd_publisher = self.create_publisher(JogCmd, '/jog_cmd', 10)

        self.component_control_publisher = self.create_publisher(String, '/run_cmd', 10)

        self.vision_control_publisher = self.create_publisher(String, '/detection_task', 10)

        self.dido_control_publisher = self.create_publisher(DIDOCmd, '/test_dido', 10)


        # subscriber

        self.height_info_subscriber = self.create_subscription(
            Int32,
            'lr_distance',
            self.height_info_callback,
            10
        )

        self.color_image_subscriber = self.create_subscription(
            Image,
            '/color_image',
            self.image_callback,
            10
        )

        # self.task_state_subscriber = self.create_subscription(
        #     TaskState,
        #     '/task_state',
        #     self.task_state_callback,
        #     10
        # )


        self.bridge = CvBridge()

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

        QScroller.grabGesture(self.ui.ScrollAreaDIDO.viewport(), QScroller.LeftMouseButtonGesture)


        #motor ui
        self.motor_controller = MotorController(self.ui, self.ros_node)

        #forklift ui
        self.forklift_controller = ForkliftController(self.ui, self.ros_node)
        self.ros_node.forklift_controller = self.forklift_controller

        # DI/DO UI Controller
        self.dido_controller = DIDOController(self.ui, self.ros_node)
        

        # Get Today's date
        today = date.today()
        formatted_date = today.strftime("%m/%d/%Y")
        self.ui.DateInput.setText(formatted_date)
        

        # Connect Qt signal to UI handler
        self.ros_msg_received.connect(self.handle_ros_message)

        # auto
        self.ui.AutoButton.toggled.connect(self.on_auto_toggled)

        self.ui.INITButton.clicked.connect(lambda: self.send_state_cmd("init"))
        self.ui.RunButton.clicked.connect(lambda: self.send_state_cmd("run"))

        self.ui.AutoPauseButton.toggled.connect(self.on_pause_toggled)
        self.ui.AutoStopButton.clicked.connect(lambda: self.send_state_cmd("stop"))

        # manual
        self.ui.ManualButton.toggled.connect(self.on_manual_toggled)

        self.ui.RoughAlignButton.toggled.connect(lambda checked: self.on_task_toggled("rough_align", checked))
        self.ui.PreciseAlignButton.toggled.connect(lambda checked: self.on_task_toggled("precise_align", checked))
        self.ui.PickButton.toggled.connect(lambda checked: self.on_task_toggled("pick", checked))
        self.ui.AssemblyButton.toggled.connect(lambda checked: self.on_task_toggled("assembly", checked))

        self.ui.ManualPauseButton.toggled.connect(self.on_pause_toggled)
        self.ui.ManualStopButton.clicked.connect(lambda: self.send_state_cmd("stop"))

        # vision fix
        self.ui.VisionOne.toggled.connect(lambda checked: self.on_vision_toggled("screw", checked))
        self.ui.VisionTwo.toggled.connect(lambda checked: self.on_vision_toggled("l_shape", checked))
        self.ui.VisionThree.toggled.connect(lambda checked: self.on_vision_toggled("icp_fit", checked))


        # vision
        #self.ui.VisionOne.toggled.connect(lambda checked: checked and self.send_vision_cmd("screw"))
        #self.ui.VisionTwo.toggled.connect(lambda checked: checked and self.send_vision_cmd("l_shape"))
        #self.ui.VisionThree.toggled.connect(lambda checked: checked and self.send_vision_cmd("icp_fit"))

        # self.ui.VisionTextInComponentControl.setFixedSize(640, 480)


        # component control - publisher
        # self.ui.ComponentControlButton.toggled.connect(self.on_component_control_toggled)

        # by default
        self.ui.ListOptionsWidget.setVisible(False)

        # Touchscreen style in Main Page - Auto
        self.ui.RunButton.clicked.connect(lambda: self.on_touch_buttons(self.ui.RunButton))
        self.ui.AutoStopButton.clicked.connect(lambda: self.on_touch_buttons(self.ui.AutoStopButton))

        # self.ui.RecordDataButton.clicked.connect(lambda: self.on_record_data_clicked(self.ui.RecordDataButton))
        self.ui.ClipperButtonOnOff.toggled.connect(lambda: self.update_clipper_state(self.ui.ClipperButtonOnOff))

        #choose your component control
        self.ui.ChooseMotor.clicked.connect(self.choose_motor)
        self.ui.ChooseVision.clicked.connect(self.choose_vision)
        self.ui.ChooseClipper.clicked.connect(self.choose_clipper)
        self.ui.ChooseForklift.clicked.connect(self.choose_forklift)
        self.ui.ChooseDIDO.clicked.connect(self.choose_DIDO)

        # menu
        self.ui.MainPageButton.toggled.connect(self.change_to_main_page)
        self.ui.ComponentControlButton.toggled.connect(self.change_to_component_control_page)
        self.ui.ProductionRecordButton.clicked.connect(self.change_to_production_record_page)
        self.ui.LogsButton.clicked.connect(self.change_to_logs_page)
        self.ui.SystemSettingsButton.clicked.connect(self.change_to_system_settings_page)

        # Main Page - Auto and Manual options
        self.ui.AutoButton.clicked.connect(self.change_to_auto_page)
        self.ui.ManualButton.clicked.connect(self.change_to_manual_page)

        #Component Control, List Menu
        self.ui.HamburgerMenu.clicked.connect(self.toggle_menu)

        self.ui.MotorOption.clicked.connect(lambda: self.component_control_switch_page("Motor", 0))
        self.ui.VisionOption.clicked.connect(lambda: self.component_control_switch_page("Vision", 1))
        self.ui.ClipperOption.clicked.connect(lambda: self.component_control_switch_page("Clipper", 2))
        self.ui.ForkliftOption.clicked.connect(lambda: self.component_control_switch_page("Forklift", 3))
        self.ui.DIDOOption.clicked.connect(lambda: self.component_control_switch_page("DI/DO", 4))

        for btn in self.ui.ManualButtons.buttons():
            btn.toggled.connect(lambda checked, b=btn: self.on_manual_button_toggled(b, checked))

        for btn in self.ui.VisionButtonGroup.buttons():
            btn.toggled.connect(lambda checked, b=btn: self.on_vision_button_toggled(b, checked))
        




    def update_circle_off_style(self):
        self.ui.OneCircleOff.setStyleSheet("""
            QPushButton {
                background-color: #0B76A0;
                border: none;
                border-radius: 15px;
                max-width: 30px;
                min-height: 30px;
                max-height: 30px;
                padding: 0;
            }
        """)

        self.ui.OneCircleOn.setStyleSheet("""
            QPushButton {
                background-color: white;
                border: none;
                border-radius: 15px;
                max-width: 30px;
                min-height: 30px;
                max-height: 30px;
                padding: 0;
            }
        """)

        self.ui.DO1Widget.setStyleSheet("""
            QWidget {
                background-color: #0B76A0;
                border: none;
                border-radius: 24px;
            }
        """)

    def update_circle_on_style(self):
        self.ui.OneCircleOff.setStyleSheet("""
            QPushButton {
                background-color: white;
                border: none;
                border-radius: 15px;
                max-width: 30px;
                min-height: 30px;
                max-height: 30px;
                padding: 0;
            }
        """)

        self.ui.OneCircleOn.setStyleSheet("""
            QPushButton {
                background-color: #000000;
                border: none;
                border-radius: 15px;
                max-width: 30px;
                min-height: 30px;
                max-height: 30px;
                padding: 0;
            }
        """)

        self.ui.DO1Widget.setStyleSheet("""
            QWidget {
                background-color: #000000;
                border: none;
                border-radius: 24px;
            }
        """)

        

    def update_image(self, cv_img):
        qt_img = convert_cv_to_qt(cv_img)
        pixmap = QPixmap.fromImage(qt_img)


        current_index = self.ui.ParentStackedWidgetToChangeMenuOptions.currentIndex()

        if current_index == 0:  # Main Page
            self.ui.VisionText.setPixmap(pixmap)
            self.ui.VisionTextInComponentControl.clear()
        elif current_index == 1:  # Component Control
            self.ui.VisionTextInComponentControl.setPixmap(pixmap)
            self.ui.VisionText.clear()
        else:
            self.ui.VisionText.clear()
            self.ui.VisionTextInComponentControl.clear()
            

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


    def on_pause_toggled(self, checked):
        if checked:
            # Button is pressed → machine should pause
            self.send_state_cmd("pause")
            print("[UI] Pause engaged")
        else:
            # Button released → machine should resume running
            self.send_state_cmd("run")
            print("[UI] Pause released, resuming run")

    def on_task_toggled(self, task_name, checked):
        if checked:
            self.send_task_cmd(task_name)
            print(f"[UI] {task_name} started")
        else:
            # Since buttons are exclusive, unchecking means no task is active
            # Optional: send a stop/idle command
            self.send_task_cmd("idle")
            print(f"[UI] {task_name} stopped")


    #ROS2 Flag
    def send_state_cmd(self, flag):
        msg = StateCmd()

        # Reset all flags
        msg.init_button = False
        msg.run_button = False
        msg.pause_button = False
        msg.stop_button = False

        if flag == "init":
            msg.init_button = True
        elif flag == "run":
            msg.run_button = True
        elif flag == "pause":
            msg.pause_button = True
        elif flag == "stop":
            msg.stop_button = True

        print(f"[DEBUG] Publishing StateCmd: {msg}")
        self.ros_node.state_cmd_publisher.publish(msg)
        print(f"[UI] Sent StateCmd: {flag}")

    def send_mode_cmd(self, flag):
        msg = String()
        msg.data = flag

        self.ros_node.mode_cmd_publisher.publish(msg)
        print(f"[UI] Sent ModeCmd: {msg.data}")

    def send_component_cmd(self, flag):
        msg = ComponentCmd()
        msg.mode = flag

        self.ros_node.component_cmd_publisher.publish(msg)
        print(f"[UI] Sent ComponentCmd: {msg.mode}")


    def send_task_cmd(self, flag):
        msg = TaskCmd()
        msg.mode = flag
        
        self.ros_node.task_cmd_publisher.publish(msg)
        print(f"[UI] Sent TaskCmd: {msg.mode}")

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

    #vision fix
    def on_vision_toggled(self, vision_name, checked):
        if checked:
            self.send_vision_cmd(vision_name)
            print(f"[UI] {vision_name} started")
        else:
            # Since buttons are exclusive, unchecking means no task is active
            # Optional: send a stop/idle command
            self.send_vision_cmd("idle")
            print(f"[UI] {vision_name} stopped")

    def send_vision_cmd(self, mode):
        print(f"[DEBUG] Trying to publish: {mode}")  

        if hasattr(self, 'last_vision_mode') and self.last_vision_mode == mode:
            return
        self.last_vision_mode = mode
        msg = String()
        msg.data = mode
        self.ros_node.vision_control_publisher.publish(msg)
        print(f"[UI] Sent VisionCmd: {mode}")

    
    def on_manual_button_toggled(self, button, checked):
        if checked:
            for other in self.ui.ManualButtons.buttons():
                if other != button:
                    other.setChecked(False)
    
    def on_vision_button_toggled(self, button, checked):
        if checked:
            for other in self.ui.VisionButtonGroup.buttons():
                if other != button:
                    other.setCheckable(False)


    #Principal Menu - StackedWidget
    def change_to_main_page(self, checked):
        if checked:
            # Buttons for Main Page reset
            main_page_buttons = [
                self.ui.AutoPauseButton,

                self.ui.RoughAlignButton,
                self.ui.PreciseAlignButton,
                self.ui.PickButton,
                self.ui.AssemblyButton,
                self.ui.ManualPauseButton
            ]
            self.reset_buttons_and_machine(main_page_buttons, send_pause=True)

            self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(0)

    def change_to_component_control_page(self, checked):
        if checked:
            component_control_buttons = [
                 self.ui.ClipperButtonOnOff,
                 self.ui.VisionOne,
                 self.ui.VisionTwo,
                 self.ui.VisionThree,

                 self.ui.buttonGroup
            ]
            self.reset_buttons_and_machine(component_control_buttons, send_pause=True)

            self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(1)
            self.ui.ComponentControlStackedWidget.setCurrentIndex(0)
            self.ui.MiddleStackedWidget.setCurrentIndex(0)
            self.send_mode_cmd("component_control")

    def reset_buttons_and_machine(self, buttons_or_groups, send_pause=False):
        for item in buttons_or_groups:
            if isinstance(item, QButtonGroup):
                # Temporarily disable exclusivity to allow all buttons to uncheck
                item.setExclusive(False)
                for btn in item.buttons():
                    btn.setChecked(False)
                item.setExclusive(True)
            else:
                # Normal single button
                item.setChecked(False)

        if send_pause:
            msg = StateCmd()
            msg.init_button = False
            msg.run_button = False
            msg.pause_button = True
            msg.stop_button = False
            self.ros_node.state_cmd_publisher.publish(msg)

            

    def change_to_production_record_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(2)

    def change_to_logs_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(3)

    def change_to_system_settings_page(self):
        self.ui.ParentStackedWidgetToChangeMenuOptions.setCurrentIndex(4)

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

    def choose_DIDO(self):
        self.ui.ComponentControlStackedWidget.setCurrentIndex(1)
        self.ui.ChangeComponentControlStackedWidget.setCurrentIndex(4)
        self.ui.MiddleStackedWidget.setCurrentIndex(1)
        self.ui.MotorStartedButton.setText("DI/DO")


    # Auto or Manual Buttons
    def change_to_auto_page(self):
        self.ui.ActionButtons.setCurrentIndex(0)
    
    def change_to_manual_page(self):
        self.ui.ActionButtons.setCurrentIndex(1)
        

    
    def on_auto_toggled(self, checked):
        if checked:
            self.send_mode_cmd("auto")

    def on_manual_toggled(self, checked):
        if checked:
            self.send_mode_cmd("manual")

    # def on_dido_toggled(self, checked):
    #     self.send_test_dido("on" if checked else "off")

    # def on_component_control_toggled(self, checked):
    #     if checked:
            

    # def send_test_dido(self, pin: str, state: str):
    #     msg = String()
    #     msg.data = f"{pin}:{state}"
    #     self.ros_node.dido_control_publisher.publish(msg)


    #Component Control - Motor
    def toggle_menu(self):
        self.ui.ListOptionsWidget.setVisible(not self.ui.ListOptionsWidget.isVisible())

    def component_control_switch_page(self, name, index):
        self.ui.MotorStartedButton.setText(name)
        self.ui.ChangeComponentControlStackedWidget.setCurrentIndex(index)
        self.ui.ListOptionsWidget.setVisible(False)

        if name == "DI/DO":
            self.send_component_cmd("dido_control")
            self.ui.MiddleStackedWidget.setCurrentIndex(1)
        elif name == "Motor":
            self.send_component_cmd("pose_control")
            self.ui.MiddleStackedWidget.setCurrentIndex(0)
        elif name == "Vision":
            self.send_component_cmd("vision_control")
            self.ui.MiddleStackedWidget.setCurrentIndex(0)
        elif name == "Clipper":
            self.send_component_cmd("cliper_control")
            self.ui.MiddleStackedWidget.setCurrentIndex(0)
        elif name == "Forklift":
            self.send_component_cmd("forklift_control")
            self.ui.MiddleStackedWidget.setCurrentIndex(0)


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

