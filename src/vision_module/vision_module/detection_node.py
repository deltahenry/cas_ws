import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import glob
import math
from std_msgs.msg import String
from ament_index_python.packages import get_package_prefix

# ⬇️ 加入 tf2 座標廣播
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transformations as tf_transformations
from transforms3d.euler import quat2euler


from vision_module.screw_detector import ScrewDetector
from vision_module.l_shape_detector import LShapeDetector
from vision_module.icp_fitter import ICPFITTER

def remove_duplicate_detections(results, threshold=20):
    filtered = []
    for r in results:
        if 'u' not in r or 'v' not in r:
            continue
        is_duplicate = False
        for f in filtered:
            dist = math.sqrt((r['u'] - f['u'])**2 + (r['v'] - f['v'])**2)
            if dist < threshold:
                is_duplicate = True
                break
        if not is_duplicate:
            filtered.append(r)
    return filtered

class RealSenseVision(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('RealSense Detection Node Started')

        self.subscription = self.create_subscription(
            String,
            '/detection_task',
            self.task_callback,
            10
        )

        self.screw_active = False
        self.lshape_active = False
        self.icp_fit_active = False

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.templates = self.load_templates('template')
        self.template_size = self.templates[0].shape[::-1]

        self.l_templates = self.load_templates('template_l_shape')

        self.screw_detector = ScrewDetector(self.templates)
        self.l_shape_dector = LShapeDetector(self.l_templates)
        self.icp_fitter = ICPFITTER()
        self.last_avg_xyz = None
        self.last_avg_uv = None

        # ⬇️ 初始化 TF 廣播器
        self.br = TransformBroadcaster(self)

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

    def broadcast_screw_tf(self, idx, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f'screw_{idx}'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)

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

                unique_results = []
                for r in screw_results:
                    is_duplicate = False
                    for u in unique_results:
                        if abs(r['u'] - u['u']) < 15 and abs(r['v'] - u['v']) < 15 and abs(r['Z'] - u['Z']) < 0.01:
                            is_duplicate = True
                            break
                    if not is_duplicate:
                        unique_results.append(r)

                filtered_results = sorted(unique_results, key=lambda x: x['Z'])[:4]

                if len(filtered_results) == 4:
                    sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
                    sum_u, sum_v = 0, 0

                    sum_roll, sum_pitch, sum_yaw = 0.0, 0.0, 0.0

                    for i, screw in enumerate(filtered_results):
                        idx = i + 1
                        x, y, z = screw['X'], screw['Y'], screw['Z']
                        u, v = screw['u'], screw['v']
                        print(f"🟢 孔位 {idx}: (u={u}, v={v}), X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")

                        node.broadcast_screw_tf(idx, x, y, z)

                        sum_x += x
                        sum_y += y
                        sum_z += z
                        sum_u += u
                        sum_v += v

                        x1, y1, x2, y2 = screw['bbox']
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(color_image, (u, v), 4, (0, 0, 255), -1)
                        label = f"{z:.3f}m"
                        cv2.putText(color_image, label, (u + 5, v + 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                        qx = screw.get('qx', 0.0)
                        qy = screw.get('qy', 0.0)
                        qz = screw.get('qz', 0.0)
                        qw = screw.get('qw', 1.0)

                        # transforms3d 的輸入順序為 (w, x, y, z)
                        roll, pitch, yaw = quat2euler([qw, qx, qy, qz])

                        sum_roll += roll
                        sum_pitch += pitch
                        sum_yaw += yaw

                    avg_x = sum_x / 4
                    avg_y = sum_y / 4
                    avg_z = sum_z / 4
                    avg_u = int(sum_u / 4)
                    avg_v = int(sum_v / 4)
                    avg_yaw = sum_yaw / 4

                    node.last_avg_xyz = (avg_x, avg_y, avg_z)
                    node.last_avg_uv = (avg_u, avg_v, avg_yaw)
                else:
                    node.get_logger().warn(f"⚠️ 僅偵測到 {len(filtered_results)} 顆螺絲孔，使用上次資料")

                if node.last_avg_xyz is not None and node.last_avg_uv is not None:
                    avg_x, avg_y, avg_z = node.last_avg_xyz
                    avg_u, avg_v, avg_yaw = node.last_avg_uv

                    tf_text = f"Center (Avg): X={avg_x:.3f} Y={avg_y:.3f} Z={avg_z:.3f}"
                    cv2.putText(color_image, tf_text, (30, 40), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (255, 255, 0), 2)

                    cv2.circle(color_image, (avg_u, avg_v), 6, (0, 255, 255), -1)
                    cv2.putText(color_image, "Center", (avg_u + 10, avg_v - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    length = 40
                    x_end_u = int(avg_u + length * np.cos(avg_yaw))
                    x_end_v = int(avg_v + length * np.sin(avg_yaw))
                    y_end_u = int(avg_u - length * np.sin(avg_yaw))
                    y_end_v = int(avg_v + length * np.cos(avg_yaw))

                    cv2.arrowedLine(color_image, (avg_u, avg_v), (x_end_u, x_end_v), (0, 0, 255), 2)
                    cv2.arrowedLine(color_image, (avg_u, avg_v), (y_end_u, y_end_v), (0, 255, 0), 2)

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
