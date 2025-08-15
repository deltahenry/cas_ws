#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rclpy
import numpy as np
import pyrealsense2 as rs
from datetime import datetime

from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge


class RealSenseCaptureNode(Node):
    """
    - 按 'q' 關閉視窗並優雅退出（在 main() 的 while 迴圈）
    - 僅支援 /capture_cmd = 'snap'：擷取對齊後 RGB + 點雲 → 發布 + 存檔（PNG 與彩色 PLY）
    """

    def __init__(self):
        super().__init__('realsense_capture_node')

        # ===== 參數 =====
        self.declare_parameter('color_width', 1280)
        self.declare_parameter('color_height', 720)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('save_dir', 'captures')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('preview', True)         # 是否開啟預覽視窗

        self.color_w = int(self.get_parameter('color_width').value)
        self.color_h = int(self.get_parameter('color_height').value)
        self.depth_w = int(self.get_parameter('depth_width').value)
        self.depth_h = int(self.get_parameter('depth_height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.save_dir = str(self.get_parameter('save_dir').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.preview = bool(self.get_parameter('preview').value)

        os.makedirs(self.save_dir, exist_ok=True)

        # ===== RealSense 初始化 =====
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.color_w, self.color_h, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.depth_w, self.depth_h, rs.format.z16, self.fps)

        # 對齊 depth->color
        self.align = rs.align(rs.stream.color)
        # 點雲工具
        self.pc = rs.pointcloud()

        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info("✅ RealSense pipeline started")
        except Exception as e:
            self.get_logger().error(f"❌ 無法啟動 RealSense：{e}")
            raise

        # ===== ROS I/O =====
        self.rgb_pub = self.create_publisher(Image, '/realsense/rgb', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/realsense/points', 10)
        self.cmd_sub = self.create_subscription(String, '/capture_cmd', self.cmd_callback, 10)

        # cv bridge
        self.bridge = CvBridge()

        # 預覽和快取
        self.last_color_frame = None
        self.last_depth_frame = None
        self.last_color_image = None
        self.frame_count = 0

        if self.preview:
            cv2.namedWindow("RealSense Detection", cv2.WINDOW_NORMAL)

        self.get_logger().info("📸 擷取指令：ros2 topic pub --once /capture_cmd std_msgs/String \"{data: 'snap'}\"")

    # ===== 指令回呼 =====
    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'snap':
            self.get_logger().info("📩 收到 'snap'，擷取 RGB + 點雲")
            self.capture_once()
        else:
            self.get_logger().warn("⚠️ 僅支援 'snap' 指令，已忽略其他內容")

    # ===== 擷取一次（使用最新快取的對齊幀；若無則現抓） =====
    def capture_once(self):
        try:
            # 若還沒快取到，現抓一組
            if self.last_color_frame is None or self.last_depth_frame is None:
                for _ in range(3):
                    frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                self.last_color_frame = aligned.get_color_frame()
                self.last_depth_frame = aligned.get_depth_frame()
                if not self.last_color_frame or not self.last_depth_frame:
                    self.get_logger().warn("⚠️ 無法取得 color/depth frame")
                    return
                self.last_color_image = np.asanyarray(self.last_color_frame.get_data())

            stamp_ros = self.get_clock().now().to_msg()
            stamp_text = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

            # === 發布/存檔 RGB ===
            img_msg = self.bridge.cv2_to_imgmsg(self.last_color_image, encoding='bgr8')
            img_msg.header.stamp = stamp_ros
            img_msg.header.frame_id = self.frame_id
            self.rgb_pub.publish(img_msg)

            img_path = os.path.join(self.save_dir, f'{stamp_text}_color.png')
            cv2.imwrite(img_path, self.last_color_image)
            self.get_logger().info(f"💾 已存檔 RGB：{img_path}")

            # === 發布/存檔 點雲 ===
            self.pc.map_to(self.last_color_frame)
            points = self.pc.calculate(self.last_depth_frame)

            # 1) 發布 XYZ 點雲
            vtx = np.asanyarray(points.get_vertices())  # (N,) 結構化陣列，含 x/y/z
            xyz = np.asarray(vtx).view(np.float32).reshape(-1, 3)
            valid = xyz[:, 2] != 0.0
            xyz = xyz[valid]

            header = Header()
            header.stamp = stamp_ros
            header.frame_id = self.frame_id
            cloud_msg = point_cloud2.create_cloud_xyz32(header, xyz)
            self.cloud_pub.publish(cloud_msg)

            # 2) 存成彩色 PLY
            ply_path = os.path.join(self.save_dir, f'{stamp_text}_cloud.ply')
            points.export_to_ply(ply_path, self.last_color_frame)
            self.get_logger().info(f"💾 已存檔 點雲(PLY)：{ply_path}")

            self.get_logger().info("✅ 擷取完成")

        except Exception as e:
            self.get_logger().error(f"❌ 擷取失敗：{e}")

    def destroy_node(self):
        # 停止 RealSense 與關視窗
        try:
            self.pipeline.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        self.get_logger().info("🛑 RealSense pipeline stopped & 視窗已關閉")
        super().destroy_node()


def main():
    rclpy.init()
    node = RealSenseCaptureNode()
    try:
        while rclpy.ok():
            # 讓 /capture_cmd 能即時觸發
            rclpy.spin_once(node, timeout_sec=0.01)

            # 預覽視窗：抓最新一幀、顯示、並更新快取（提供 snap 使用）
            if node.preview:
                try:
                    frames = node.pipeline.wait_for_frames()
                    aligned = node.align.process(frames)
                    color_frame = aligned.get_color_frame()
                    depth_frame = aligned.get_depth_frame()
                    if not color_frame or not depth_frame:
                        continue

                    node.last_color_frame = color_frame
                    node.last_depth_frame = depth_frame
                    color_image = np.asanyarray(color_frame.get_data())
                    node.last_color_image = color_image

                    # 顯示視窗（按 'q' 離開）
                    cv2.imshow("RealSense Detection", color_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        node.get_logger().info("👋 按下 'q'，離開程式。")
                        break

                    node.frame_count += 1
                except Exception as e:
                    node.get_logger().warn(f"預覽更新失敗：{e}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
