import open3d as o3d
import numpy as np
import cv2
from std_msgs.msg import String

golden_path = "golden_sample.pcd"

class ICPFITTER:
    def __init__(self):
        self.golden_pcd = o3d.io.read_point_cloud(golden_path)
        print(f"✅ 成功載入 golden sample: {golden_path}")

    def icp_fit(self,color_image,depth_image,depth_intrin):

        if self.golden_pcd is None:
            print("⚠️ 無法載入 golden sample，請先拍攝並儲存")
            return None

        current_pcd = self.rs_to_pointcloud(color_image, depth_image, depth_intrin)
       
        if current_pcd is None:
            print("⚠️ 無法建立當前點雲，請檢查影像資料")
            return None

        dist = self.compare_pcd_distance(self.golden_pcd, current_pcd)
        
        return dist

    def cmd_callback(self, msg: String):
        cmd = msg.data.lower()
        if cmd == 'a':
            self.get_logger().info("📸 拍攝並儲存 golden sample")
            pcd = self.rs_to_pointcloud(self.color_image, self.depth_image, self.Cam.depth_intrin)
            if pcd is None:
                self.get_logger().warn("無影像資料，無法建立 golden sample")
                return
            o3d.io.write_point_cloud(golden_path, pcd)
            self.golden_pcd = pcd
            self.get_logger().info(f"✅ 成功儲存: {golden_path}")

        elif cmd == 'q':
            self.get_logger().info("🔚 收到結束指令，關閉節點")
            self.destroy_node()
            cv2.destroyAllWindows()

        else:
            self.get_logger().warn(f"❓ 未知指令: {cmd}。可用 'a' 建立 golden sample，'q' 結束節點")

    def rs_to_pointcloud(self, color, depth, intrin):
        if color is None or depth is None:
            return None
        h, w = depth.shape
        fx, fy = intrin.fx, intrin.fy
        cx, cy = intrin.ppx, intrin.ppy
        depth_scale = 0.001

        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth * depth_scale
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
        colors = color.reshape(-1, 3).astype(np.float32) / 255.0

        valid = (z.reshape(-1) > 0) & (z.reshape(-1) < 1.2)
        points = points[valid]
        colors = colors[valid]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    def compare_pcd_distance(self, pcd1, pcd2):
        if pcd1 is None or pcd2 is None:
            return float('inf')
        dists = pcd1.compute_point_cloud_distance(pcd2)
        if len(dists) == 0:
            return float('inf')
        return np.mean(dists)

