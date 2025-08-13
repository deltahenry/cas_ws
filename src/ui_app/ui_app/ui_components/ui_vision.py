#NOT USED YET
#ui_vision.py

from PySide6.QtGui import QImage, QPixmap
from std_msgs.msg import String
import cv2

def _cv_to_qimage(cv_img):
    rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)

class VisionController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node
        self._last_mode = None

        # Wire the buttons here (and remove vision wiring from MainWindow)
        self._vision_buttons = [
            (self.ui.VisionOne,   "screw"),
            (self.ui.VisionTwo,   "l_shape"),
            (self.ui.VisionThree, "icp_fit"),
        ]
        for btn, mode in self._vision_buttons:
            btn.setCheckable(True)
            btn.toggled.connect(lambda checked, m=mode: self.on_vision_toggled(m, checked))
            btn.clicked.connect(lambda checked, b=btn: self._on_vision_clicked(b, checked))

    # Slots that MainWindow signals will call:
    def update_image(self, cv_img):
        qimg = _cv_to_qimage(cv_img)
        pixmap = QPixmap.fromImage(qimg)
        idx = self.ui.ParentStackedWidgetToChangeMenuOptions.currentIndex()
        if idx == 0:
            self.ui.VisionText.setPixmap(pixmap)
            self.ui.VisionTextInComponentControl.clear()
        elif idx == 1:
            self.ui.VisionTextInComponentControl.setPixmap(pixmap)
            self.ui.VisionText.clear()
        else:
            self.ui.VisionText.clear()
            self.ui.VisionTextInComponentControl.clear()

    def update_mode(self, mode: str):
        print(f"[Vision] detection mode -> {mode}")

    # Button logic:
    def on_vision_toggled(self, vision_name: str, checked: bool):
        mode = vision_name if checked else f"{vision_name}_off"
        self._send_vision_cmd(mode)

    def _on_vision_clicked(self, btn, checked: bool):
        if checked:
            for other, _ in self._vision_buttons:
                if other is not btn and other.isChecked():
                    other.setChecked(False)

    # Publish:
    def _send_vision_cmd(self, mode: str):
        if getattr(self, "_last_mode", None) == mode:
            return
        self._last_mode = mode
        msg = String()
        msg.data = mode
        self.ros_node.vision_control_publisher.publish(msg)
        print(f"[Vision] cmd: {mode}")
