import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import glob
from std_msgs.msg import String
from ament_index_python.packages import get_package_prefix

class RealSenseVision(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('RealSense Detection Node Started')

        # 狀態旗標
        self.screw_active = False
        self.lshape_active = False

        # 任務指令訂閱者
        self.subscription = self.create_subscription(
            String,
            '/detection_task',
            self.task_callback,
            10
        )

        # 螺絲孔模板
        self.templates = self.load_templates('template')
        self.template_size = self.templates[0].shape[::-1]
        self.match_threshold = 0.65
        self.scales = [0.8, 0.9, 1.0, 1.1, 1.2]
        self.roi_boxes = [
            (362, 266, 397, 399),
            (902, 261, 938, 408)
        ]

        # L型線模板
        self.l_templates = self.load_templates('template_l_shape')
        self.l_match_threshold = 0.7
        self.l_roi_boxes = [
            (319, 423, 379, 502),
            (895, 429, 949, 505)
        ]

        # RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

    def task_callback(self, msg):
        if msg.data == "screw":
            self.screw_active = True
            self.lshape_active = False
            self.get_logger().info("🟢 任務切換：螺絲孔偵測")
        elif msg.data == "lshape":
            self.screw_active = False
            self.lshape_active = True
            self.get_logger().info("🟢 任務切換：L 型線偵測")
        else:
            self.screw_active = False
            self.lshape_active = False
            self.get_logger().info("⚪ 任務暫停：停止偵測")

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

    def detect_screw_holes(self, color_frame, depth_frame):
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        all_boxes = []

        for roi in self.roi_boxes:
            x1, y1, x2, y2 = roi
            roi_gray = gray[y1:y2, x1:x2]

            for tpl0 in self.templates:
                h0, w0 = tpl0.shape
                for s in self.scales:
                    W, H = int(w0 * s), int(h0 * s)
                    if W < 5 or H < 5:
                        continue
                    tpl = cv2.resize(tpl0, (W, H))
                    result = cv2.matchTemplate(roi_gray, tpl, cv2.TM_CCOEFF_NORMED)
                    ys, xs = np.where(result >= self.match_threshold)

                    for y, x in zip(ys, xs):
                        global_x1 = x + x1
                        global_y1 = y + y1
                        global_x2 = global_x1 + W
                        global_y2 = global_y1 + H
                        all_boxes.append([global_x1, global_y1, global_x2, global_y2, float(result[y, x])])

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        top4 = sorted(all_boxes, key=lambda b: -b[4])[:4]

        for i, (x1, y1, x2, y2, score) in enumerate(top4):
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            depth = depth_frame.get_distance(cx, cy)
            if depth == 0 or depth > 1.0:
                print(f"⚠️ 孔位 {i+1}: 無效或距離過遠 (depth={depth:.3f}m)")
                continue
            X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cx, cy], depth)
            print(f"孔位 {i+1}: X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m")

            cv2.rectangle(color_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(color_frame, (cx, cy), 4, (0, 0, 255), -1)
            label = f"{depth:.3f}m"
            cv2.putText(color_frame, label, (cx + 5, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def angle_between(self, v1, v2):
        unit_v1 = v1 / np.linalg.norm(v1)
        unit_v2 = v2 / np.linalg.norm(v2)
        angle = np.degrees(np.arccos(np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)))
        return angle

    def detect_l_shape_lines(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        results = []

        for (x1, y1, x2, y2) in self.l_roi_boxes:
            roi = edges[y1:y2, x1:x2]
            lines = cv2.HoughLinesP(roi, 1, np.pi / 180, threshold=40, minLineLength=20, maxLineGap=10)

            if lines is not None:
                for i in range(len(lines)):
                    for j in range(i+1, len(lines)):
                        x11, y11, x12, y12 = lines[i][0]
                        x21, y21, x22, y22 = lines[j][0]

                        vec1 = np.array([x12 - x11, y12 - y11])
                        vec2 = np.array([x22 - x21, y22 - y21])
                        angle = self.angle_between(vec1, vec2)

                        if 80 <= angle <= 100:
                            x11g, y11g = x11 + x1, y11 + y1
                            x12g, y12g = x12 + x1, y12 + y1
                            x21g, y21g = x21 + x1, y21 + y1
                            x22g, y22g = x22 + x1, y22 + y1

                            cv2.line(frame, (x11g, y11g), (x12g, y12g), (255, 255, 0), 2)
                            cv2.line(frame, (x21g, y21g), (x22g, y22g), (255, 255, 0), 2)

                            cx = (x11g + x12g + x21g + x22g) // 4
                            cy = (y11g + y12g + y21g + y22g) // 4
                            results.append((cx, cy))
        return results

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

            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())

            if node.screw_active:
                node.detect_screw_holes(color_image, depth_frame)

            if node.lshape_active:
                l_results = node.detect_l_shape_lines(color_image)
                for i, (cx, cy) in enumerate(l_results):
                    cv2.circle(color_image, (cx, cy), 5, (0, 255, 255), -1)
                    cv2.putText(color_image, f"L{i+1}", (cx + 5, cy - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

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