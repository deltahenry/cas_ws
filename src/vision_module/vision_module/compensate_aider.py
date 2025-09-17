#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

# -------------------------------
# Mode 1: Hardcode your line coordinates here
use_hardcode = True   # set to False if you want to click instead

line1_start = (262, 159)
line1_end   = (262, 230)

line2_start = (262, 230)
line2_end   = (352, 230)
# -------------------------------

clicked_points = []  # for mouse mode

def mouse_callback(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at: ({x}, {y})")
        clicked_points.append((x, y))
        if len(clicked_points) > 4:
            clicked_points = clicked_points[-4:]

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.pub = self.create_publisher(Image, '/visual_image', 10)
        self.bridge = CvBridge()

        # RealSense 初始化
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        # 可以用 mouse mode 調整線段
        cv2.namedWindow("RealSense Click Adjust")
        cv2.setMouseCallback("RealSense Click Adjust", mouse_callback)

        self.timer = self.create_timer(0.03, self.timer_callback)  # 30 ms -> ~33Hz

    def timer_callback(self):
        global clicked_points
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())

        if use_hardcode:
            # --- Always draw from hardcoded coordinates ---
            cv2.line(color_image, line1_start, line1_end, (0, 0, 255), 2)  # red
            cv2.line(color_image, line2_start, line2_end, (0, 255, 0), 2)  # green
        else:
            # --- Draw from mouse clicks ---
            for pt in clicked_points:
                cv2.circle(color_image, pt, 5, (255, 255, 0), -1)
            if len(clicked_points) >= 2:
                cv2.line(color_image, clicked_points[0], clicked_points[1], (0, 0, 255), 2)
            if len(clicked_points) >= 4:
                cv2.line(color_image, clicked_points[2], clicked_points[3], (0, 255, 0), 2)

        # 發布影像
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.pub.publish(msg)

        # 可選：在調整模式下仍然顯示視窗
        if not use_hardcode:
            cv2.imshow("RealSense Click Adjust", color_image)
            cv2.waitKey(1)

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
