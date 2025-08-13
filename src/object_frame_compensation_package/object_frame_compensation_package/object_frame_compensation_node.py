#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Transform
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import time
import json

from .object_frame_algorithms import ObjectFrameCompensation
from .coordinate_transforms import normalize_quaternion, calculate_pose_difference
import numpy as np


class ObjectFrameCompensationNode(Node):
    def __init__(self):
        super().__init__('object_frame_compensation_node')
        
        # 參數聲明
        self.declare_node_parameters()
        
        # TF相關
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 訂閱者
        self.object_pose_sub = self.create_subscription(
            Pose,
            '~/object_camera_pose',
            self.object_pose_callback,
            10
        )
        
        self.current_arm_sub = self.create_subscription(
            Pose,
            '~/current_arm_pose',
            self.current_arm_callback,
            10
        )
        
        # 發布者 - 主要輸出
        self.target_arm_pub = self.create_publisher(
            Pose,
            '~/target_arm_pose',
            10
        )
        
        # 發布者 - 除錯資訊
        self.object_world_pub = self.create_publisher(
            Pose,
            '~/object_world_pose',
            10
        )
        
        self.gripper_object_pub = self.create_publisher(
            Pose,
            '~/gripper_object_relationship',
            10
        )
        
        self.comparison_pub = self.create_publisher(
            String,
            '~/method_comparison',
            10
        )
        
        # 服務
        self.recalculate_srv = self.create_service(
            Empty,
            '~/recalculate_gripper_relationship',
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