#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Pose, Transform, TransformStamped
from .coordinate_transforms import (
    pose_to_transform_matrix, transform_matrix_to_pose,
    multiply_transform_matrices, invert_transform_matrix,
    transform_pose, normalize_quaternion
)


class ObjectFrameCompensation:
    """
    物件座標系補償演算法實作
    
    新方法核心概念：
    1. Golden Sample 階段：建立夾具與物件的固定相對關係
    2. 執行階段：使用物件座標系轉換計算目標手臂位置
    """
    
    def __init__(self, camera_in_hand_tf, golden_arm_pose, golden_object_camera_pose):
        self.camera_in_hand_tf = camera_in_hand_tf
        self.golden_arm_pose = golden_arm_pose
        self.golden_object_camera_pose = golden_object_camera_pose
        
        # 計算固定的夾具-物件關係
        self.gripper_in_object_pose = self._calculate_gripper_object_relationship()
        
        # 用於比較的傳統方法實例
        self.traditional_method = TraditionalCompensation(
            camera_in_hand_tf, golden_arm_pose, golden_object_camera_pose
        )
    
    def _calculate_gripper_object_relationship(self):
        """
        計算夾具相對於物件座標系的固定位置
        
        步驟：
        1. 將 Golden Sample 物件位置從相機座標系轉換到世界座標系
        2. 建立物件座標系
        3. 計算手臂位置相對於物件座標系的位置
        
        Returns:
            geometry_msgs/Pose: 夾具在物件座標系中的固定位置
        """
        # 步驟1: 物件從相機座標系轉換到世界座標系
        golden_object_world = self._transform_camera_to_world(
            self.golden_object_camera_pose,
            self.golden_arm_pose,  # 使用 golden sample 時的手臂位置
            self.camera_in_hand_tf
        )
        
        # 步驟2: 計算夾具相對於物件座標系的位置
        # gripper_in_object = object_world^(-1) * gripper_world
        gripper_in_object = self._calculate_relative_pose(
            self.golden_arm_pose,    # 夾具位置（世界座標系）
            golden_object_world      # 物件位置（世界座標系）
        )
        
        return gripper_in_object
    
    def calculate_target_arm_pose(self, current_object_camera_pose, current_arm_pose):
        """
        基於新的物件位置計算目標手臂位置
        
        新方法流程：
        1. 將當前物件位置轉換到世界座標系
        2. 使用固定的夾具-物件關係計算目標手臂位置
        
        Args:
            current_object_camera_pose: 當前物件在相機座標系的位置
            current_arm_pose: 當前手臂位置（世界座標系）
            
        Returns:
            tuple: (target_arm_pose, current_object_world, debug_info)
        """
        # 步驟1: 將當前物件位置轉換到世界座標系
        current_object_world = self._transform_camera_to_world(
            current_object_camera_pose,
            current_arm_pose,  # 使用當前手臂位置
            self.camera_in_hand_tf
        )
        
        # 步驟2: 使用固定的夾具-物件關係計算目標手臂位置
        # target_arm = current_object_world * gripper_in_object
        target_arm_pose = self._transform_object_to_world(
            self.gripper_in_object_pose,  # 夾具在物件座標系中的位置
            current_object_world          # 新的物件座標系原點
        )
        
        # 產生除錯資訊
        debug_info = {
            'method': 'object_frame',
            'current_object_world': current_object_world,
            'gripper_in_object': self.gripper_in_object_pose,
            'calculation_steps': [
                '1. 物件相機座標 → 世界座標',
                '2. 應用固定夾具-物件關係',
                '3. 計算目標手臂世界座標'
            ]
        }
        
        return target_arm_pose, current_object_world, debug_info
    
    def compare_with_traditional_method(self, current_object_camera_pose, current_arm_pose):
        """
        與傳統方法進行比較
        
        Returns:
            dict: 包含兩種方法結果的比較資訊
        """
        # 新方法計算
        new_target, new_object_world, new_debug = self.calculate_target_arm_pose(
            current_object_camera_pose, current_arm_pose
        )
        
        # 傳統方法計算
        traditional_target, traditional_debug = self.traditional_method.calculate_compensation(
            current_object_camera_pose, current_arm_pose
        )
        
        # 計算差異
        position_diff = np.array([
            new_target.position.x - traditional_target.position.x,
            new_target.position.y - traditional_target.position.y,
            new_target.position.z - traditional_target.position.z
        ])
        
        position_distance = np.linalg.norm(position_diff)
        
        # 計算四元數角度差異
        q1 = np.array([new_target.orientation.x, new_target.orientation.y,
                       new_target.orientation.z, new_target.orientation.w])
        q2 = np.array([traditional_target.orientation.x, traditional_target.orientation.y,
                       traditional_target.orientation.z, traditional_target.orientation.w])
        
        dot_product = np.abs(np.dot(q1, q2))
        dot_product = min(1.0, dot_product)
        angle_diff_rad = 2 * np.arccos(dot_product)
        angle_diff_deg = np.degrees(angle_diff_rad)
        
        comparison = {
            'new_method': {
                'target_pose': new_target,
                'object_world': new_object_world,
                'debug_info': new_debug
            },
            'traditional_method': {
                'target_pose': traditional_target,
                'debug_info': traditional_debug
            },
            'differences': {
                'position_diff_mm': position_diff * 1000,  # 轉換為毫米
                'position_distance_mm': position_distance * 1000,
                'angle_diff_deg': angle_diff_deg,
                'significant_difference': position_distance > 0.001 or angle_diff_deg > 1.0
            }
        }
        
        return comparison
    
    def _transform_camera_to_world(self, object_camera_pose, arm_world_pose, camera_in_hand_tf):
        """
        將物件從相機座標系轉換到世界座標系
        
        轉換鏈: Camera → Hand → World
        """
        # 計算手臂在世界座標系的轉換矩陣
        T_arm_world = pose_to_transform_matrix(arm_world_pose)
        
        # 計算相機在世界座標系的轉換矩陣
        T_camera_hand = self._transform_to_matrix(camera_in_hand_tf)
        T_camera_world = multiply_transform_matrices(T_arm_world, T_camera_hand)
        
        # 將物件從相機座標系轉換到世界座標系
        T_object_camera = pose_to_transform_matrix(object_camera_pose)
        T_object_world = multiply_transform_matrices(T_camera_world, T_object_camera)
        
        object_world_pose = transform_matrix_to_pose(T_object_world)
        return object_world_pose
    
    def _transform_object_to_world(self, gripper_in_object_pose, object_world_pose):
        """
        將夾具位置從物件座標系轉換到世界座標系
        
        gripper_world = object_world * gripper_in_object
        """
        T_object_world = pose_to_transform_matrix(object_world_pose)
        T_gripper_object = pose_to_transform_matrix(gripper_in_object_pose)
        
        T_gripper_world = multiply_transform_matrices(T_object_world, T_gripper_object)
        
        gripper_world_pose = transform_matrix_to_pose(T_gripper_world)
        return gripper_world_pose
    
    def _calculate_relative_pose(self, target_pose, reference_pose):
        """
        計算 target 相對於 reference 的相對位置
        
        relative_pose = reference_pose^(-1) * target_pose
        """
        T_reference = pose_to_transform_matrix(reference_pose)
        T_target = pose_to_transform_matrix(target_pose)
        
        T_reference_inv = invert_transform_matrix(T_reference)
        T_relative = multiply_transform_matrices(T_reference_inv, T_target)
        
        relative_pose = transform_matrix_to_pose(T_relative)
        return relative_pose
    
    def _transform_to_matrix(self, transform):
        """將 Transform 轉換為 4x4 變換矩陣"""
        temp_pose = Pose()
        temp_pose.position.x = transform.translation.x
        temp_pose.position.y = transform.translation.y
        temp_pose.position.z = transform.translation.z
        temp_pose.orientation = transform.rotation
        
        return pose_to_transform_matrix(temp_pose)


class TraditionalCompensation:
    """
    傳統直接補償方法實作（用於比較）
    """
    
    def __init__(self, camera_in_hand_tf, golden_arm_pose, golden_object_camera_pose):
        self.camera_in_hand_tf = camera_in_hand_tf
        self.golden_arm_pose = golden_arm_pose
        self.golden_object_camera_pose = golden_object_camera_pose
    
    def calculate_compensation(self, current_object_camera_pose, current_arm_pose):
        """
        傳統直接補償方法
        
        流程：
        1. 計算物件在相機座標系中的位移
        2. 將位移轉換到手臂座標系
        3. 應用位移補償到 Golden Sample 手臂位置
        """
        # 步驟1: 計算物件位移（相機座標系）
        object_displacement_camera = self._pose_difference(
            current_object_camera_pose, self.golden_object_camera_pose
        )
        
        # 步驟2: 將位移轉換到手臂座標系
        # 需要使用相機到手臂轉換的逆變換
        camera_to_hand_tf_inv = self._invert_transform(self.camera_in_hand_tf)
        object_displacement_hand = transform_pose(object_displacement_camera, camera_to_hand_tf_inv)
        
        # 步驟3: 應用補償到 Golden Sample 手臂位置
        target_arm_pose = self._pose_add(self.golden_arm_pose, object_displacement_hand)
        
        debug_info = {
            'method': 'traditional',
            'object_displacement_camera': object_displacement_camera,
            'object_displacement_hand': object_displacement_hand,
            'calculation_steps': [
                '1. 計算物件相機座標位移',
                '2. 位移轉換到手臂座標系',
                '3. 應用位移補償'
            ]
        }
        
        return target_arm_pose, debug_info
    
    def _pose_difference(self, pose1, pose2):
        """計算兩個 Pose 之間的差異"""
        diff_pose = Pose()
        diff_pose.position.x = pose1.position.x - pose2.position.x
        diff_pose.position.y = pose1.position.y - pose2.position.y
        diff_pose.position.z = pose1.position.z - pose2.position.z
        
        # 旋轉差異使用相對四元數
        T1 = pose_to_transform_matrix(pose1)
        T2 = pose_to_transform_matrix(pose2)
        T2_inv = invert_transform_matrix(T2)
        T_diff = multiply_transform_matrices(T1, T2_inv)
        
        diff_pose_full = transform_matrix_to_pose(T_diff)
        diff_pose.orientation = diff_pose_full.orientation
        
        return diff_pose
    
    def _pose_add(self, pose_base, pose_offset):
        """將位置偏移量加到基準位置上"""
        T_base = pose_to_transform_matrix(pose_base)
        T_offset = pose_to_transform_matrix(pose_offset)
        T_result = multiply_transform_matrices(T_base, T_offset)
        
        return transform_matrix_to_pose(T_result)
    
    def _invert_transform(self, transform):
        """反轉變換"""
        T = self._transform_to_matrix(transform)
        T_inv = invert_transform_matrix(T)
        
        # 轉換回 Transform 格式
        transform_inv = Transform()
        transform_inv.translation.x = T_inv[0, 3]
        transform_inv.translation.y = T_inv[1, 3]
        transform_inv.translation.z = T_inv[2, 3]
        
        # 從旋轉矩陣提取四元數
        from scipy.spatial.transform import Rotation
        r = Rotation.from_matrix(T_inv[:3, :3])
        q = r.as_quat()  # [x, y, z, w]
        
        transform_inv.rotation.x = q[0]
        transform_inv.rotation.y = q[1]
        transform_inv.rotation.z = q[2]
        transform_inv.rotation.w = q[3]
        
        return transform_inv
    
    def _transform_to_matrix(self, transform):
        """將 Transform 轉換為矩陣"""
        temp_pose = Pose()
        temp_pose.position.x = transform.translation.x
        temp_pose.position.y = transform.translation.y
        temp_pose.position.z = transform.translation.z
        temp_pose.orientation = transform.rotation
        
        return pose_to_transform_matrix(temp_pose)