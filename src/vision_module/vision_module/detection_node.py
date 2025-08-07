import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
import os, glob
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_prefix
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transformations as tf_transformations
from scipy.spatial.transform import Rotation as R

from vision_module.screw_detector import ScrewDetector
from vision_module.l_shape_detector import LShapeDetector
from vision_module.icp_fitter import ICPFITTER

def remove_duplicate_detections(results, threshold=15):
    filtered = []
    for r in results:
        if 'u' not in r or 'v' not in r:
            continue
        is_duplicate = any(
            abs(r['u'] - f['u']) < threshold and abs(r['v'] - f['v']) < threshold
            for f in filtered
        )
        if not is_duplicate:
            filtered.append(r)
    return filtered

class RealSenseVision(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.get_logger().info('✅ RealSense Detection Node Started')
        self.subscription = self.create_subscription(String, '/detection_task', self.task_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/screw_center_pose', 10)

        self.screw_active = self.lshape_active = self.icp_fit_active = False

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        source_root = os.path.dirname(os.path.abspath(__file__))
        template_path = os.path.join(source_root, 'template2_shape')
        self.screw_detector = ScrewDetector(template_path)

        self.l_templates = self.load_templates('template2_shape')
        self.l_shape_dector = LShapeDetector(self.l_templates)
        self.icp_fitter = ICPFITTER()

        self.br = TransformBroadcaster(self)
        self.prev_screw_results = []
        self.prev_avg_center = None
        self.prev_avg_pose = None
        self.frame_count = 0
        # ✅ 加入角度記憶變數
        self.prev_yaw = 0.0
        self.prev_pitch = 0.0
        self.prev_roll = 1.5
        

    def load_templates(self, folder_name, augment=False):
        source_root = os.path.dirname(os.path.abspath(__file__))
        folder_path = os.path.join(source_root, folder_name)
        print(f"📂 嘗試讀取模板資料夾：{folder_path}")
        if not os.path.exists(folder_path):
            print(f"❌ 錯誤：資料夾不存在 - {folder_path}")
            return []

        paths = sorted(glob.glob(os.path.join(folder_path, "*.png")))
        if len(paths) == 0:
            print(f"⚠️ 警告：找不到任何 .png 模板在資料夾 {folder_path}")

        templates = []
        for path in paths:
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                templates.append(img)
            else:
                print(f"⚠️ 無法讀取圖片：{path}")

        print(f"✅ 成功載入 {len(templates)} 張模板圖片")
        return templates

    def task_callback(self, msg):
        task = msg.data.lower()
        self.screw_active = task == 'screw'
        self.lshape_active = task == 'l_shape'
        self.icp_fit_active = task == 'icp_fit'
        self.get_logger().info(f"🔄 任務切換: {task}")

    def broadcast_screw_tf(self, idx, x, y, z, quat=None):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f'screw_{idx}'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        if quat is None:
            quat = [0, 0, 0, 1]  # default: no rotation

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.br.sendTransform(t)

    def publish_avg_pose(self, x, y, z, quat):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pose_pub.publish(msg)
        

def main():
    rclpy.init()
    node = RealSenseVision()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            frames = node.pipeline.wait_for_frames()
            aligned = node.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            if node.screw_active:
                screw_results = []
                if node.frame_count % 10 == 0:
                    screw_results = node.screw_detector.detect(color_image, depth_frame)
                    screw_results = remove_duplicate_detections(screw_results)
                    screw_results = sorted(screw_results, key=lambda r: r['Z'])[:4]
                    if len(screw_results) == 4:
                        node.prev_screw_results = screw_results
                else:
                    screw_results = node.prev_screw_results

                if len(screw_results) == 4:
                    avg_x = avg_y = avg_z = 0.0
                    avg_u = avg_v = 0
                    quats = []

                    dx = screw_results[2]['u'] - screw_results[0]['u']
                    dy = screw_results[2]['v'] - screw_results[0]['v']
                    yaw_rad = np.arctan2(dy, dx)
                    yaw_deg = np.degrees(yaw_rad)
                    quat = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)

                    r = R.from_quat(quat)
                    yaw_deg, pitch_deg, roll_deg = r.as_euler('zyx', degrees=True)

                    # ✅ 保留顯示，不做跳過
                    for i, r_ in enumerate(screw_results):
                        u, v = r_['u'], r_['v']
                        X, Y, Z = r_['X'], r_['Y'], r_['Z']
                        quats.append(quat)
                        node.broadcast_screw_tf(i + 1, X, Y, Z, quat)
                        avg_x += X
                        avg_y += Y
                        avg_z += Z
                        avg_u += u
                        avg_v += v

                        cv2.circle(color_image, (u, v), 6, (0, 255, 0), -1)
                        cv2.putText(color_image, f"X={X:.2f} Y={Y:.2f} Z={Z:.2f}", (u + 10, v - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                        print(f"🟢 螺絲 {i+1}: (u={u}, v={v}), X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")

                    # ✅ 用三點建立平面方向，求出 R 與四元數
                    v0 = np.array([screw_results[0]['X'], screw_results[0]['Y'], screw_results[0]['Z']])
                    v1 = np.array([screw_results[1]['X'], screw_results[1]['Y'], screw_results[1]['Z']])
                    v2 = np.array([screw_results[2]['X'], screw_results[2]['Y'], screw_results[2]['Z']])
                    v3 = np.array([screw_results[3]['X'], screw_results[3]['Y'], screw_results[3]['Z']])
                    
                    x_axis = v3 - v2
                    x_axis = x_axis / np.linalg.norm(x_axis)
                    temp_vec = v0 - v2
                    z_axis = np.cross(x_axis, temp_vec)
                    z_axis = z_axis / np.linalg.norm(z_axis)
                    y_axis = np.cross(z_axis, x_axis)  # ✅ 正交右手系

                    R_mat = np.column_stack((x_axis, y_axis, z_axis))
                    quat = tf_transformations.quaternion_from_matrix(
                        np.vstack((np.hstack((R_mat, np.array([[0], [0], [0]]))), [0, 0, 0, 1]))
                    )

                    r = R.from_quat(quat)
                    yaw_deg, pitch_deg, roll_deg = r.as_euler('zyx', degrees=True)

                    print(f"✅ 四元數: {quat}")
                    print(f"✅ 尤拉角: Yaw={yaw_deg:.2f}°, Pitch={pitch_deg:.2f}°, Roll={roll_deg:.2f}°")

                    avg_x /= 4
                    avg_y /= 4
                    avg_z /= 4
                    center_u = avg_u // 4
                    center_v = avg_v // 4

                    print(f"[INFO] Orientation: Yaw={yaw_deg:.1f}°, Pitch={pitch_deg:.1f}°, Roll={roll_deg:.1f}°")

                    node.prev_avg_center = (center_u, center_v)
                    node.prev_avg_pose = (avg_x, avg_y, avg_z, yaw_deg, pitch_deg, roll_deg)

                    node.publish_avg_pose(avg_x, avg_y, avg_z, quat)  # ✅ 呼叫正確方法

                    node.prev_yaw = yaw_deg
                    node.prev_pitch = pitch_deg
                    node.prev_roll = roll_deg

                # ✅ 顯示上次有效結果（即使目前沒偵測到）
                if node.prev_avg_center and node.prev_avg_pose:
                    center_u, center_v = node.prev_avg_center
                    avg_x, avg_y, avg_z, yaw_deg, pitch_deg, roll_deg = node.prev_avg_pose
                    text1 = f"Avg X={avg_x:.2f} Y={avg_y:.2f} Z={avg_z:.2f}"
                    text2 = f"Yaw={yaw_deg:.1f}deg Pitch={pitch_deg:.1f}deg Roll={roll_deg:.1f}deg"
                    cv2.circle(color_image, (int(center_u), int(center_v)), 8, (0, 255, 255), -1)
                    cv2.putText(color_image, text1, (center_u - 100, center_v - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(color_image, text2, (center_u - 100, center_v + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            if node.lshape_active:
                results = node.l_shape_dector.detect_l_shape_lines(color_image)
                for cx, cy in results:
                    cv2.circle(color_image, (cx, cy), 5, (0, 255, 255), -1)

            if node.icp_fit_active:
                dist = node.icp_fitter.icp_fit(color_image, depth_image, depth_intrin)
                print(f"ICP Distance: {dist:.4f}" if dist else "ICP fitting failed")

            cv2.imshow("RealSense Detection", color_image)
            if cv2.waitKey(1) == ord('q'):
                break

            node.frame_count += 1

    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
