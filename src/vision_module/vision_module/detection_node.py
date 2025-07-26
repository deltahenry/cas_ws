import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import glob
from std_msgs.msg import String
from ament_index_python.packages import get_package_prefix

from vision_module.screw_detector import ScrewDetector
from vision_module.l_shape_detector import LShapeDetector
from vision_module.icp_fitter import ICPFITTER
class RealSenseVision(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('RealSense Detection Node Started')

        # 任務指令訂閱者
        self.subscription = self.create_subscription(
            String,
            '/detection_task',
            self.task_callback,
            10
        )

        # 狀態旗標
        self.screw_active = False
        self.lshape_active = False
        self.icp_fit_active = False

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # 螺絲孔模板
        self.templates = self.load_templates('template')
        self.template_size = self.templates[0].shape[::-1]

        # L型線模板
        self.l_templates = self.load_templates('template_l_shape')

        # 初始化螺絲檢測器
        self.screw_detector = ScrewDetector(self.templates)

        # 初始化L型線檢測器
        self.l_shape_dector = LShapeDetector(self.l_templates)

        # 初始化ICP配準器
        self.icp_fitter = ICPFITTER()


    def task_callback(self, msg):
        task = msg.data.lower()
        if task == 'screw':
            self.screw_active = True
            self.lshape_active = False
            self.icp_fit_active = False
            self.get_logger().info("螺絲檢測已啟動")
        elif task == 'l_shape':
            self.screw_active = False
            self.lshape_active = True
            self.icp_fit_active = False
            self.get_logger().info("L型線檢測已啟動")
        elif task == 'icp_fit':
            self.screw_active = False
            self.lshape_active = False
            self.icp_fit_active = True
            self.get_logger().info("ICP配準已啟動")
        else:
            self.screw_active = False
            self.lshape_active = False
            self.icp_fit_active = False
            self.get_logger().warn(f"未知任務: {task}")

    def load_templates(self, folder_name):
        prefix = get_package_prefix('vision_module')
        folder_path = os.path.join(prefix, 'lib', 'vision_module', folder_name)
        # script_dir = os.path.dirname(os.path.abspath(__file__))
        # folder_path = os.path.join(script_dir, folder_name)
        paths = sorted(glob.glob(os.path.join(folder_path, "*.png")))

        templates = []
        for path in paths:
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                templates.append(img)
            else:
                self.get_logger().warn(f"Failed to load: {path}")
        if not templates:
            raise RuntimeError(f"No template found in folder: {folder_name}")
        return templates


def main():
    rclpy.init()
    node = RealSenseVision()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            frames = node.pipeline.wait_for_frames()
            aligned_frames = node.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            

            if node.screw_active:
                screw_results = node.screw_detector.detect(color_image, depth_frame)
                print(f"Detected {len(screw_results)} screws")

            if node.lshape_active:
                l_results = node.l_shape_dector.detect_l_shape_lines(color_image)
                for i, (cx, cy) in enumerate(l_results):
                    cv2.circle(color_image, (cx, cy), 5, (0, 255, 255), -1)
                    cv2.putText(color_image, f"L{i+1}", (cx + 5, cy - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            if node.icp_fit_active:
                dist = node.icp_fitter.icp_fit(color_image, depth_image, depth_intrin)
                print(f"ICP Distance: {dist:.4f}" if dist is not None else "ICP fitting failed")


            cv2.imshow("RealSense Detection", color_image)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()