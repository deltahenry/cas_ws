from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import QTimer, QThread, QObject, Signal
from std_msgs.msg import String
import cv2
import time 

# def convert_cv_to_qt(cv_img):
#     # rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
#     h, w, ch = cv_img.shape
#     bytes_per_line = ch * w
#     return QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888).copy()

class VisionController(QObject):
    image_ready = Signal(QPixmap)

    def __init__(self, ui, ros_node):
        super().__init__()  

        self.ui = ui
        self.ros_node = ros_node

        self.thread = QThread()
        self.moveToThread(self.thread)

        self.image_ready.connect(self.update_ui_display)
        self.thread.started.connect(self.start_timer) 
        
        self.thread.start()


    def start_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_latest_frame)
        self.timer.start(40) # 25 FPS

    # def process_latest_frame(self):
    #     if self.ros_node.latest_qt_img is not None:
    #         qt_img = self.ros_node.latest_qt_img
    #         self.ros_node.latest_qt_img = None
            
    #         # Process image on background thread
    #         pixmap = QPixmap.fromImage(qt_img)
            
    #         # Emit signal to update UI (thread-safe)
    #         self.image_ready.emit(pixmap)


    def process_latest_frame(self):
        with self.ros_node.image_lock:
            if self.ros_node.latest_ros_msg is not None:
                msg = self.ros_node.latest_ros_msg
                self.ros_node.latest_ros_msg = None
            else:
                return
        
        try:
            cv_img = self.ros_node.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Direct conversion to pixmap (skips QImage step)
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888).copy()
            pixmap = QPixmap.fromImage(qt_img)
            
            self.image_ready.emit(pixmap)
            
        except Exception as e:
            print(f"Failed to process image: {e}")
    
    def update_ui_display(self, pixmap):
        # This runs on main thread - safe for UI updates
        current_index = self.ui.ParentStackedWidgetToChangeMenuOptions.currentIndex()
        if current_index == 0:
            self.ui.VisionText.setPixmap(pixmap)
            self.ui.VisionTextInComponentControl.clear()
        elif current_index == 1:
            self.ui.VisionTextInComponentControl.setPixmap(pixmap)
            self.ui.VisionText.clear()
        else:
            self.ui.VisionText.clear()
            self.ui.VisionTextInComponentControl.clear()
            
    
    def update_detection_mode(self, mode):
        print(f"[UI] Detection mode is now: {mode}")