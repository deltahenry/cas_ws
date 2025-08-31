#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Transform
from std_msgs.msg import String,Int32
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import time
import json

from .object_frame_algorithms import ObjectFrameCompensation
from .coordinate_transforms import normalize_quaternion, calculate_pose_difference
import numpy as np

from scipy.spatial.transform import Rotation as R
import math
from datetime import datetime
import pandas as pd


class ObjectFrameCompensationNode(Node):
    def __init__(self):
        super().__init__('object_frame_compensation_node')
        
        # 參數聲明
        self.declare_node_parameters()
        
        # TF相關
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 訂閱者
        self.object_pose_sub1 = self.create_subscription(
            Pose,
            '/object_camera_pose',
            self.object_pose_callback,
            10
        )

        self.object_pose_sub2 = self.create_subscription(
            Pose,
            '/object_camera_pose',
            self.get_matrix_callback,
            10
        )

        self.ob_in_world_x = 0.0
        self.ob_in_world_y = 0.0
        self.ob_in_world_z = 0.0
        self.ob_in_world_qx = 0.0
        self.ob_in_world_qy = 0.0
        self.ob_in_world_qz = 0.0
        self.ob_in_world_qw = 1.0
        
        self.current_arm_sub = self.create_subscription(
            Pose,
            '/current_arm_pose',
            self.current_arm_callback,
            10
        )
        
        # 發布者 - 主要輸出
        self.target_arm_pub = self.create_publisher(
            Pose,
            '/target_arm_pose',
            10
        )
        
        # 發布者 - 除錯資訊
        self.object_world_pub = self.create_publisher(
            Pose,
            '/object_world_pose',
            10
        )
        
        self.gripper_object_pub = self.create_publisher(
            Pose,
            '/gripper_object_relationship',
            10
        )
        
        self.comparison_pub = self.create_publisher(
            String,
            '/method_comparison',
            10
        )
        
        # 服務
        self.recalculate_srv = self.create_service(
            Empty,
            '/recalculate_gripper_relationship',
            self.recalculate_service_callback
        )
        
        # 內部狀態
        self.current_arm_pose = None
        self.compensation_calculator = None
        self.parameters_initialized = False
        
        # 效能監控
        self.RESPONSE_TIME_LIMIT_MS = 50.0
        
        # 初始化補償計算器
        self.initialize_compensation_calculator()
        
        self.get_logger().info('Object Frame Compensation Node 已啟動')
        self.get_logger().info('使用物件座標系轉換方法')
    
    def declare_node_parameters(self):
        """聲明所有 ROS2 參數"""
        
        # Camera-in-Hand 轉換參數
        self.declare_parameter('camera_in_hand_tf.translation.x', 0.0)
        self.declare_parameter('camera_in_hand_tf.translation.y', 0.0)
        self.declare_parameter('camera_in_hand_tf.translation.z', 0.0)
        self.declare_parameter('camera_in_hand_tf.rotation.x', 0.0)
        self.declare_parameter('camera_in_hand_tf.rotation.y', 0.0)
        self.declare_parameter('camera_in_hand_tf.rotation.z', 0.0)
        self.declare_parameter('camera_in_hand_tf.rotation.w', 1.0)
        
        # Golden Sample 手臂位置參數（世界座標系）
        self.declare_parameter('golden_arm_pose.position.x', 0.0)
        self.declare_parameter('golden_arm_pose.position.y', 0.0)
        self.declare_parameter('golden_arm_pose.position.z', 0.0)
        self.declare_parameter('golden_arm_pose.orientation.x', 0.0)
        self.declare_parameter('golden_arm_pose.orientation.y', 0.0)
        self.declare_parameter('golden_arm_pose.orientation.z', 0.0)
        self.declare_parameter('golden_arm_pose.orientation.w', 1.0)
        
        # Golden Sample 物件位置參數（相機座標系）
        self.declare_parameter('golden_object_camera_pose.position.x', 0.0)
        self.declare_parameter('golden_object_camera_pose.position.y', 0.0)
        self.declare_parameter('golden_object_camera_pose.position.z', 0.0)
        self.declare_parameter('golden_object_camera_pose.orientation.x', 0.0)
        self.declare_parameter('golden_object_camera_pose.orientation.y', 0.0)
        self.declare_parameter('golden_object_camera_pose.orientation.z', 0.0)
        self.declare_parameter('golden_object_camera_pose.orientation.w', 1.0)
        
        # 節點行為參數
        self.declare_parameter('auto_calculate_on_startup', True)
        self.declare_parameter('publish_debug_info', False)
        self.declare_parameter('enable_method_comparison', False)
        self.declare_parameter('comparison_threshold_mm', 1.0)
        self.declare_parameter('comparison_threshold_deg', 1.0)
    
    def load_parameters(self):
        """載入 ROS2 參數"""
        try:
            # 載入相機到手臂的轉換
            self.camera_in_hand_tf = Transform()
            self.camera_in_hand_tf.translation.x = self.get_parameter('camera_in_hand_tf.translation.x').get_parameter_value().double_value
            self.camera_in_hand_tf.translation.y = self.get_parameter('camera_in_hand_tf.translation.y').get_parameter_value().double_value
            self.camera_in_hand_tf.translation.z = self.get_parameter('camera_in_hand_tf.translation.z').get_parameter_value().double_value
            self.camera_in_hand_tf.rotation.x = self.get_parameter('camera_in_hand_tf.rotation.x').get_parameter_value().double_value
            self.camera_in_hand_tf.rotation.y = self.get_parameter('camera_in_hand_tf.rotation.y').get_parameter_value().double_value
            self.camera_in_hand_tf.rotation.z = self.get_parameter('camera_in_hand_tf.rotation.z').get_parameter_value().double_value
            self.camera_in_hand_tf.rotation.w = self.get_parameter('camera_in_hand_tf.rotation.w').get_parameter_value().double_value
            
            # 載入 Golden Sample 手臂位置
            self.golden_arm_pose = Pose()
            self.golden_arm_pose.position.x = self.get_parameter('golden_arm_pose.position.x').get_parameter_value().double_value
            self.golden_arm_pose.position.y = self.get_parameter('golden_arm_pose.position.y').get_parameter_value().double_value
            self.golden_arm_pose.position.z = self.get_parameter('golden_arm_pose.position.z').get_parameter_value().double_value
            self.golden_arm_pose.orientation.x = self.get_parameter('golden_arm_pose.orientation.x').get_parameter_value().double_value
            self.golden_arm_pose.orientation.y = self.get_parameter('golden_arm_pose.orientation.y').get_parameter_value().double_value
            self.golden_arm_pose.orientation.z = self.get_parameter('golden_arm_pose.orientation.z').get_parameter_value().double_value
            self.golden_arm_pose.orientation.w = self.get_parameter('golden_arm_pose.orientation.w').get_parameter_value().double_value
            
            # 載入 Golden Sample 物件位置
            self.golden_object_camera_pose = Pose()
            self.golden_object_camera_pose.position.x = self.get_parameter('golden_object_camera_pose.position.x').get_parameter_value().double_value
            self.golden_object_camera_pose.position.y = self.get_parameter('golden_object_camera_pose.position.y').get_parameter_value().double_value
            self.golden_object_camera_pose.position.z = self.get_parameter('golden_object_camera_pose.position.z').get_parameter_value().double_value
            self.golden_object_camera_pose.orientation.x = self.get_parameter('golden_object_camera_pose.orientation.x').get_parameter_value().double_value
            self.golden_object_camera_pose.orientation.y = self.get_parameter('golden_object_camera_pose.orientation.y').get_parameter_value().double_value
            self.golden_object_camera_pose.orientation.z = self.get_parameter('golden_object_camera_pose.orientation.z').get_parameter_value().double_value
            self.golden_object_camera_pose.orientation.w = self.get_parameter('golden_object_camera_pose.orientation.w').get_parameter_value().double_value
            
            # 載入控制參數
            self.auto_calculate = self.get_parameter('auto_calculate_on_startup').get_parameter_value().bool_value
            self.publish_debug = self.get_parameter('publish_debug_info').get_parameter_value().bool_value
            self.enable_comparison = self.get_parameter('enable_method_comparison').get_parameter_value().bool_value
            self.comparison_threshold_mm = self.get_parameter('comparison_threshold_mm').get_parameter_value().double_value
            self.comparison_threshold_deg = self.get_parameter('comparison_threshold_deg').get_parameter_value().double_value
            
            # 正規化四元數
            self._normalize_loaded_quaternions()
            
            self.parameters_initialized = True
            self.get_logger().info('參數載入成功')
            return True
            
        except Exception as e:
            self.get_logger().error(f'參數載入失敗: {str(e)}')
            return False
    
    def _normalize_loaded_quaternions(self):
        """正規化載入的四元數參數"""
        
        # 正規化相機轉換四元數
        q = normalize_quaternion(np.array([
            self.camera_in_hand_tf.rotation.x,
            self.camera_in_hand_tf.rotation.y,
            self.camera_in_hand_tf.rotation.z,
            self.camera_in_hand_tf.rotation.w
        ]))
        self.camera_in_hand_tf.rotation.x = q[0]
        self.camera_in_hand_tf.rotation.y = q[1]
        self.camera_in_hand_tf.rotation.z = q[2]
        self.camera_in_hand_tf.rotation.w = q[3]
        
        # 正規化 Golden Sample 手臂位置四元數
        q = normalize_quaternion(np.array([
            self.golden_arm_pose.orientation.x,
            self.golden_arm_pose.orientation.y,
            self.golden_arm_pose.orientation.z,
            self.golden_arm_pose.orientation.w
        ]))
        self.golden_arm_pose.orientation.x = q[0]
        self.golden_arm_pose.orientation.y = q[1]
        self.golden_arm_pose.orientation.z = q[2]
        self.golden_arm_pose.orientation.w = q[3]
        
        # 正規化 Golden Sample 物件位置四元數
        q = normalize_quaternion(np.array([
            self.golden_object_camera_pose.orientation.x,
            self.golden_object_camera_pose.orientation.y,
            self.golden_object_camera_pose.orientation.z,
            self.golden_object_camera_pose.orientation.w
        ]))
        self.golden_object_camera_pose.orientation.x = q[0]
        self.golden_object_camera_pose.orientation.y = q[1]
        self.golden_object_camera_pose.orientation.z = q[2]
        self.golden_object_camera_pose.orientation.w = q[3]
    
    def initialize_compensation_calculator(self):
        """初始化補償計算器"""
        try:
            # 載入參數
            if not self.parameters_initialized and not self.load_parameters():
                self.get_logger().error('參數載入失敗，無法初始化補償計算器')
                return
            
            # 建立補償計算器
            self.compensation_calculator = ObjectFrameCompensation(
                self.camera_in_hand_tf,
                self.golden_arm_pose,
                self.golden_object_camera_pose
            )
            
            # 發布除錯資訊
            if self.publish_debug:
                self.gripper_object_pub.publish(
                    self.compensation_calculator.gripper_in_object_pose
                )
            
            self.get_logger().info('補償計算器初始化完成')
            self.get_logger().info(f'夾具-物件關係已計算: '
                                  f'x={self.compensation_calculator.gripper_in_object_pose.position.x:.3f}, '
                                  f'y={self.compensation_calculator.gripper_in_object_pose.position.y:.3f}, '
                                  f'z={self.compensation_calculator.gripper_in_object_pose.position.z:.3f}')
                                  
        except Exception as e:
            self.get_logger().error(f'補償計算器初始化失敗: {e}')
    
    def object_pose_callback(self, msg):
        """處理物件位置輸入並計算補償"""
        
        start_time = time.time()
        
        if self.current_arm_pose is None:
            self.get_logger().warn('尚未接收到手臂當前位置，使用 Golden Sample 手臂位置')
            current_arm = self.golden_arm_pose
        else:
            current_arm = self.current_arm_pose
            
        if self.compensation_calculator is None:
            self.get_logger().error('補償計算器未初始化')
            return
            
        try:
            if self.enable_comparison:
                # 進行方法比較
                comparison = self.compensation_calculator.compare_with_traditional_method(
                    msg, current_arm
                )
                
                # 使用新方法的結果
                target_pose = comparison['new_method']['target_pose']
                object_world_pose = comparison['new_method']['object_world']
                
                # 發布比較結果
                self._publish_comparison_results(comparison)
                
                # 記錄比較資訊
                diff = comparison['differences']
                self.get_logger().info(
                    f'[方法比較] 位置差異: {diff["position_distance_mm"]:.2f}mm, '
                    f'角度差異: {diff["angle_diff_deg"]:.2f}°, '
                    f'顯著差異: {diff["significant_difference"]}'
                )
                
            else:
                # 只使用物件座標系方法
                target_pose, object_world_pose, debug_info = self.compensation_calculator.calculate_target_arm_pose(
                    msg, current_arm
                )
            
            # 發布主要結果
            self.target_arm_pub.publish(target_pose)
            
            # 發布除錯資訊
            if self.publish_debug:
                self.object_world_pub.publish(object_world_pose)
                self.gripper_object_pub.publish(
                    self.compensation_calculator.gripper_in_object_pose
                )

            self.ob_in_world_x = object_world_pose.position.x
            self.ob_in_world_y = object_world_pose.position.y
            self.ob_in_world_z = object_world_pose.position.z
            self.ob_in_world_qx = object_world_pose.orientation.x
            self.ob_in_world_qy = object_world_pose.orientation.y
            self.ob_in_world_qz = object_world_pose.orientation.z
            self.ob_in_world_qw = object_world_pose.orientation.w

            
            # 效能監控
            end_time = time.time()
            duration_ms = (end_time - start_time) * 1000
            
            if duration_ms > self.RESPONSE_TIME_LIMIT_MS:
                self.get_logger().warning(
                    f'響應時間 {duration_ms:.2f} ms 超過限制 {self.RESPONSE_TIME_LIMIT_MS} ms'
                )
            
            # 記錄結果
            self.get_logger().info(
                f'[物件座標系方法] 計算目標手臂位置: '
                f'x={target_pose.position.x:.3f}, '
                f'y={target_pose.position.y:.3f}, '
                f'z={target_pose.position.z:.3f}, '
                f'耗時: {duration_ms:.2f}ms'
            )
                                  
        except Exception as e:
            self.get_logger().error(f'物件座標系補償計算失敗: {e}')
    
    def current_arm_callback(self, msg):
        """更新手臂當前位置"""
        self.current_arm_pose = msg
        self.get_logger().debug(f'收到手臂位置: x={msg.position.x:.3f}, y={msg.position.y:.3f}, z={msg.position.z:.3f}')
    
    def recalculate_service_callback(self, request, response):
        """重新計算夾具-物件關係的服務"""
        try:
            self.initialize_compensation_calculator()
            self.get_logger().info('夾具-物件關係重新計算完成')
        except Exception as e:
            self.get_logger().error(f'重新計算失敗: {e}')
        
        return response
    
    def get_matrix_callback(self, msg:Pose):

        if self.current_arm_pose is None:
            self.get_logger().warn('尚未接收到手臂當前位置')
            return
        else:
            all_data = []

            # === 加入時間戳 ===
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            all_data.append([f"Timestamp: {timestamp}"])

            #object to camera HTM
            # 位置
            obcf_x, obcf_y, obcf_z = msg.position.x, msg.position.y, msg.position.z
            # 四元素
            obcf_qx, obcf_qy, obcf_qz, obcf_qw = (
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )
            # 四元素 → 旋轉矩陣
            obcf_r = R.from_quat([obcf_qx, obcf_qy, obcf_qz, obcf_qw])
            Robcf_mat = obcf_r.as_matrix()

            # 建立 4x4 變換矩陣
            Tobcf = np.eye(4)
            Tobcf[:3, :3] = Robcf_mat
            Tobcf[:3, 3] = [obcf_x, obcf_y, obcf_z]

            self.get_logger().info(f"T_ob_in_cf:\n{Tobcf}")

            yaw_deg, pitch_deg, roll_deg = self.euler_zyx_from_R(Robcf_mat)
            self.get_logger().info(f"Yaw: {yaw_deg:.2f} deg, Pitch: {pitch_deg:.2f} deg, Roll: {roll_deg:.2f} deg")
            self.append_matrix_with_label(all_data, "T_ob_in_cf", Tobcf, (yaw_deg, pitch_deg, roll_deg))

            #eye in hand HTM
            # 位置
            cfarm_x, cfarm_y, cfarm_z = 0.0 , 0.249, 0.544
            # 四元素
            cfarm_qx, cfarm_qy, cfarm_qz, cfarm_qw = (
                -0.707,
                0.0,
                0.0,
                0.707,
            )
            # 四元素 → 旋轉矩陣
            cfarm_r = R.from_quat([cfarm_qx, cfarm_qy, cfarm_qz, cfarm_qw])
            Rcfarm_mat = cfarm_r.as_matrix()   
            # 建立 4x4 變換矩陣
            Tcfarm = np.eye(4)
            Tcfarm[:3, :3] = Rcfarm_mat
            Tcfarm[:3, 3] = [cfarm_x, cfarm_y, cfarm_z]
            
            self.get_logger().info(f"T_cf_in_arm:\n{Tcfarm}")

            yaw_deg, pitch_deg, roll_deg = self.euler_zyx_from_R(Rcfarm_mat)
            self.get_logger().info(f"Yaw: {yaw_deg:.2f} deg, Pitch: {pitch_deg:.2f} deg, Roll: {roll_deg:.2f} deg")
            self.append_matrix_with_label(all_data, "T_cf_in_arm", Tcfarm, (yaw_deg, pitch_deg, roll_deg))


            #ob in world from node
            # 位置
            tx, ty, tz = self.ob_in_world_x, self.ob_in_world_y, self.ob_in_world_z
            # 四元素
            qx, qy, qz, qw = (
                self.ob_in_world_qx,
                self.ob_in_world_qy,
                self.ob_in_world_qz,
                self.ob_in_world_qw,
            )
            r = R.from_quat([qx, qy, qz, qw])
            R_mat = r.as_matrix()   
            # 建立 4x4 變換矩陣
            Tobwf_node = np.eye(4)
            Tobwf_node[:3, :3] = R_mat
            Tobwf_node[:3, 3] = [tx, ty, tz]
            
            self.get_logger().info(f"T_ob_in_world_node:\n{Tobwf_node}")

            yaw_deg, pitch_deg, roll_deg = self.euler_zyx_from_R(R_mat)
            self.get_logger().info(f"Yaw: {yaw_deg:.2f} deg, Pitch: {pitch_deg:.2f} deg, Roll: {roll_deg:.2f} deg")
            self.append_matrix_with_label(all_data, "T_ob_in_world_node", Tobwf_node, (yaw_deg, pitch_deg, roll_deg))


            #current arm HTM in world
            # 位置
            armwf_x = self.current_arm_pose.position.x 
            armwf_y = self.current_arm_pose.position.y 
            armwf_z = self.current_arm_pose.position.z
            # 四元素
            armwf_qx, armwf_qy, armwf_qz, armwf_qw = (
                self.current_arm_pose.orientation.x,
                self.current_arm_pose.orientation.y,
                self.current_arm_pose.orientation.z,
                self.current_arm_pose.orientation.w,
            )
            # 四元素 → 旋轉矩陣
            armwf_r = R.from_quat([armwf_qx, armwf_qy, armwf_qz, armwf_qw])
            Rarmwf_mat = armwf_r.as_matrix()   
            # 建立 4x4 變換矩陣
            Tarmwf = np.eye(4)
            Tarmwf[:3, :3] = Rarmwf_mat
            Tarmwf[:3, 3] = [armwf_x, armwf_y, armwf_z]     
            self.get_logger().info(f"T_arm_in_world:\n{Tarmwf}")
            yaw_deg, pitch_deg, roll_deg = self.euler_zyx_from_R(Rarmwf_mat)
            self.get_logger().info(f"Yaw: {yaw_deg:.2f} deg, Pitch: {pitch_deg:.2f} deg, Roll: {roll_deg:.2f}) deg")
            self.append_matrix_with_label(all_data, "T_arm_in_world", Tarmwf, (yaw_deg, pitch_deg, roll_deg))


            #計算 T_ob_in_world
            Tobwf = Tarmwf @ Tcfarm @ Tobcf
            self.get_logger().info(f"T_ob_in_world:\n{Tobwf}")
            R_ob_in_world = Tobwf[:3, :3]
            yaw_deg, pitch_deg, roll_deg = self.euler_zyx_from_R(R_ob_in_world)
            self.get_logger().info(f"Yaw: {yaw_deg:.2f} deg, Pitch: {pitch_deg:.2f} deg, Roll: {roll_deg:.2f} deg") 
            self.append_matrix_with_label(all_data, "T_ob_in_world", Tobwf, (yaw_deg, pitch_deg, roll_deg))

            # === 存到 CSV (追加模式) ===
            df = pd.DataFrame(all_data)
            df.to_csv("all_matrices.csv", index=False, header=False, mode="a")
            self.get_logger().info("矩陣 + Euler 已追加到 all_matrices.csv")


    def append_matrix_with_label(self,matrix_list, label, matrix, euler_zyx):
        """將矩陣加上標籤，並加上對應 Euler 角"""
        matrix_list.append([label])  # 標籤
        for row in matrix:
            matrix_list.append(row.tolist())  # 每一列轉成 list
        # Euler angles
        yaw_deg, pitch_deg, roll_deg = euler_zyx
        matrix_list.append([f"Euler_ZYX (deg): Yaw={yaw_deg:.2f}, Pitch={pitch_deg:.2f}, Roll={roll_deg:.2f}"])
        matrix_list.append([])  # 空行分隔


    def euler_zyx_from_R(self,R):
        """R = [x̂ ŷ ẑ], return (yaw, pitch, roll) deg, ZYX order."""
        sy = -float(R[2,0])
        sy = max(-1.0, min(1.0, sy))
        pitch = math.asin(sy)
        if abs(sy) < 0.999999:
            roll  = math.atan2(float(R[2,1]), float(R[2,2]))
            yaw   = math.atan2(float(R[1,0]), float(R[0,0]))
        else:
            roll  = math.atan2(-float(R[1,2]), float(R[1,1]))
            yaw   = 0.0
        return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))


    def _publish_comparison_results(self, comparison):
        """發布方法比較結果"""
        try:
            # 構建比較資料
            comparison_data = {
                'timestamp': time.time(),
                'new_method': {
                    'position': {
                        'x': comparison['new_method']['target_pose'].position.x,
                        'y': comparison['new_method']['target_pose'].position.y,
                        'z': comparison['new_method']['target_pose'].position.z
                    },
                    'orientation': {
                        'x': comparison['new_method']['target_pose'].orientation.x,
                        'y': comparison['new_method']['target_pose'].orientation.y,
                        'z': comparison['new_method']['target_pose'].orientation.z,
                        'w': comparison['new_method']['target_pose'].orientation.w
                    }
                },
                'traditional_method': {
                    'position': {
                        'x': comparison['traditional_method']['target_pose'].position.x,
                        'y': comparison['traditional_method']['target_pose'].position.y,
                        'z': comparison['traditional_method']['target_pose'].position.z
                    },
                    'orientation': {
                        'x': comparison['traditional_method']['target_pose'].orientation.x,
                        'y': comparison['traditional_method']['target_pose'].orientation.y,
                        'z': comparison['traditional_method']['target_pose'].orientation.z,
                        'w': comparison['traditional_method']['target_pose'].orientation.w
                    }
                },
                'differences': {
                    'position_diff_mm': comparison['differences']['position_diff_mm'].tolist(),
                    'position_distance_mm': comparison['differences']['position_distance_mm'],
                    'angle_diff_deg': comparison['differences']['angle_diff_deg'],
                    'significant_difference': comparison['differences']['significant_difference']
                }
            }
            
            # 發布 JSON 字串
            msg = String()
            msg.data = json.dumps(comparison_data, indent=2)
            self.comparison_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'發布比較結果失敗: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectFrameCompensationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中斷信號，正在關閉節點...')
    except Exception as e:
        node.get_logger().error(f'節點運行時發生錯誤: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()