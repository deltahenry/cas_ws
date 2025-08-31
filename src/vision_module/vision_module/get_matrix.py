import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import os
from datetime import datetime


class PoseToMatrixNode(Node):
    def __init__(self):
        super().__init__('pose_to_matrix_node')

        # 訂閱 Pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/object_camera_pose',
            self.pose_callback,
            10
        )

        # CSV 檔案
        self.csv_file = os.path.join(os.getcwd(), "pose_matrices.csv")
        self.get_logger().info(f"Saving 4x4 matrices to {self.csv_file}")

        # 如果檔案不存在，先寫入表頭
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                header = ["timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)]
                writer.writerow(header)

    def pose_callback(self, msg: Pose):
        # 位置
        tx, ty, tz = msg.position.x, msg.position.y, msg.position.z

        # 四元素
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        # 四元素 → 旋轉矩陣
        r = R.from_quat([qx, qy, qz, qw])
        R_mat = r.as_matrix()

        # 建立 4x4 變換矩陣
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = [tx, ty, tz]

        # 印出來
        self.get_logger().info(f"4x4 matrix:\n{T}")

        # 存到 CSV
        with open(self.csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            row = [datetime.now().isoformat()] + T.flatten().tolist()
            writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToMatrixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
