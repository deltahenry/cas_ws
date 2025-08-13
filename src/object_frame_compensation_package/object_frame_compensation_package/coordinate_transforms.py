#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Pose, Transform, TransformStamped
from scipy.spatial.transform import Rotation
SCIPY_AVAILABLE = False  # Disable scipy to avoid compatibility issues


def pose_to_transform_matrix(pose):
    """
    將 Pose 轉換為 4x4 齊次變換矩陣
    
    Args:
        pose: geometry_msgs/Pose
    
    Returns:
        numpy.array: 4x4 變換矩陣
    """
    # 提取位置
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    
    # 提取四元數並轉換為旋轉矩陣
    q = np.array([pose.orientation.x, pose.orientation.y, 
                  pose.orientation.z, pose.orientation.w])
    
    # 正規化四元數
    q = normalize_quaternion(q)
    
    # 轉換為旋轉矩陣
    if SCIPY_AVAILABLE:
        r = Rotation.from_quat(q)  # scipy 使用 [x, y, z, w] 格式
        R = r.as_matrix()
    else:
        R = quaternion_to_rotation_matrix(q)
    
    # 構建 4x4 齊次變換矩陣
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    
    return T


def transform_matrix_to_pose(matrix):
    """
    將 4x4 變換矩陣轉換為 Pose
    
    Args:
        matrix: numpy.array - 4x4 變換矩陣
    
    Returns:
        geometry_msgs/Pose
    """
    pose = Pose()
    
    # 提取位置
    pose.position.x = float(matrix[0, 3])
    pose.position.y = float(matrix[1, 3])
    pose.position.z = float(matrix[2, 3])
    
    # 提取旋轉矩陣並轉換為四元數
    R = matrix[0:3, 0:3]
    
    if SCIPY_AVAILABLE:
        r = Rotation.from_matrix(R)
        q = r.as_quat()  # [x, y, z, w]
    else:
        q = rotation_matrix_to_quaternion(R)
    
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    
    return pose


def multiply_transform_matrices(T1, T2):
    """
    矩陣相乘 result = T1 * T2
    
    Args:
        T1, T2: numpy.array - 4x4 變換矩陣
    
    Returns:
        numpy.array - 4x4 結果矩陣
    """
    return np.dot(T1, T2)


def invert_transform_matrix(T):
    """
    計算 4x4 變換矩陣的逆矩陣
    
    Args:
        T: numpy.array - 4x4 變換矩陣
    
    Returns:
        numpy.array - 4x4 逆變換矩陣
    """
    # 對於齊次變換矩陣，可以使用高效的逆變換公式
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    
    # 旋轉矩陣的逆等於其轉置
    R_inv = R.T
    
    # 計算逆平移
    t_inv = -R_inv @ t
    
    # 構建逆變換矩陣
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv
    
    return T_inv


def transform_pose(pose, transform):
    """
    使用變換對 Pose 進行座標轉換
    
    Args:
        pose: geometry_msgs/Pose
        transform: geometry_msgs/Transform
    
    Returns:
        geometry_msgs/Pose - 變換後的 Pose
    """
    # 將 Transform 轉換為矩陣
    T_transform = transform_to_matrix(transform)
    
    # 將 Pose 轉換為矩陣
    T_pose = pose_to_transform_matrix(pose)
    
    # 執行變換
    T_result = multiply_transform_matrices(T_transform, T_pose)
    
    # 轉換回 Pose
    return transform_matrix_to_pose(T_result)


def transform_to_matrix(transform):
    """
    將 Transform 轉換為 4x4 變換矩陣
    
    Args:
        transform: geometry_msgs/Transform
    
    Returns:
        numpy.array: 4x4 變換矩陣
    """
    # 創建臨時 Pose
    temp_pose = Pose()
    temp_pose.position.x = transform.translation.x
    temp_pose.position.y = transform.translation.y
    temp_pose.position.z = transform.translation.z
    temp_pose.orientation = transform.rotation
    
    return pose_to_transform_matrix(temp_pose)


def normalize_quaternion(q):
    """
    正規化四元數
    
    Args:
        q: numpy.array - [x, y, z, w]
    
    Returns:
        numpy.array - 正規化後的四元數
    """
    norm = np.linalg.norm(q)
    if norm > 1e-8:
        return q / norm
    else:
        return np.array([0, 0, 0, 1])  # 單位四元數


def quaternion_to_rotation_matrix(q):
    """
    將四元數轉換為旋轉矩陣（手動實作，當 scipy 不可用時）
    
    Args:
        q: numpy.array - [x, y, z, w]
    
    Returns:
        numpy.array - 3x3 旋轉矩陣
    """
    x, y, z, w = q[0], q[1], q[2], q[3]
    
    # 正規化
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm > 0:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
    
    # 計算旋轉矩陣元素
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    
    R = np.array([
        [1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy)],
        [2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx)],
        [2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy)]
    ])
    
    return R


def rotation_matrix_to_quaternion(R):
    """
    將旋轉矩陣轉換為四元數（手動實作）
    
    Args:
        R: numpy.array - 3x3 旋轉矩陣
    
    Returns:
        numpy.array - 四元數 [x, y, z, w]
    """
    trace = np.trace(R)
    
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2  # s = 4 * w
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * x
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * y
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * z
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    return normalize_quaternion(np.array([x, y, z, w]))


def calculate_pose_difference(pose1, pose2):
    """
    計算兩個 Pose 之間的差異
    
    Args:
        pose1, pose2: geometry_msgs/Pose
    
    Returns:
        dict: 包含位置和角度差異的字典
    """
    # 位置差異
    pos_diff = np.array([
        pose1.position.x - pose2.position.x,
        pose1.position.y - pose2.position.y,
        pose1.position.z - pose2.position.z
    ])
    
    pos_distance = np.linalg.norm(pos_diff)
    
    # 角度差異
    q1 = np.array([pose1.orientation.x, pose1.orientation.y,
                   pose1.orientation.z, pose1.orientation.w])
    q2 = np.array([pose2.orientation.x, pose2.orientation.y,
                   pose2.orientation.z, pose2.orientation.w])
    
    # 計算四元數角度差異
    dot_product = np.abs(np.dot(normalize_quaternion(q1), normalize_quaternion(q2)))
    dot_product = min(1.0, dot_product)
    angle_diff_rad = 2 * np.arccos(dot_product)
    angle_diff_deg = np.degrees(angle_diff_rad)
    
    return {
        'position_difference': pos_diff,
        'position_distance': pos_distance,
        'angle_difference_rad': angle_diff_rad,
        'angle_difference_deg': angle_diff_deg
    }


def validate_transform_matrix(T):
    """
    驗證變換矩陣的有效性
    
    Args:
        T: numpy.array - 4x4 變換矩陣
    
    Returns:
        tuple: (is_valid, error_message)
    """
    if T.shape != (4, 4):
        return False, f"矩陣尺寸錯誤: {T.shape}, 應為 (4, 4)"
    
    # 檢查底部行是否為 [0, 0, 0, 1]
    expected_bottom = np.array([0, 0, 0, 1])
    if not np.allclose(T[3, :], expected_bottom, atol=1e-6):
        return False, f"底部行錯誤: {T[3, :]}, 應為 {expected_bottom}"
    
    # 檢查旋轉部分是否為正交矩陣
    R = T[0:3, 0:3]
    should_be_identity = R @ R.T
    identity = np.eye(3)
    
    if not np.allclose(should_be_identity, identity, atol=1e-6):
        return False, "旋轉矩陣不是正交矩陣"
    
    # 檢查行列式是否為 1（右手座標系）
    det = np.linalg.det(R)
    if not np.isclose(det, 1.0, atol=1e-6):
        return False, f"旋轉矩陣行列式錯誤: {det}, 應為 1.0"
    
    return True, "變換矩陣有效"


def create_transform_from_translation_rotation(translation, rotation_quat):
    """
    從平移和旋轉四元數創建變換矩陣
    
    Args:
        translation: array-like - [x, y, z]
        rotation_quat: array-like - [x, y, z, w]
    
    Returns:
        numpy.array - 4x4 變換矩陣
    """
    T = np.eye(4)
    
    # 設置平移
    T[0:3, 3] = translation
    
    # 設置旋轉
    q_norm = normalize_quaternion(np.array(rotation_quat))
    if SCIPY_AVAILABLE:
        r = Rotation.from_quat(q_norm)
        T[0:3, 0:3] = r.as_matrix()
    else:
        T[0:3, 0:3] = quaternion_to_rotation_matrix(q_norm)
    
    return T


def interpolate_poses(pose1, pose2, t):
    """
    在兩個 Pose 之間進行插值
    
    Args:
        pose1, pose2: geometry_msgs/Pose
        t: float - 插值參數 (0-1)
    
    Returns:
        geometry_msgs/Pose - 插值結果
    """
    if not (0 <= t <= 1):
        raise ValueError(f"插值參數 t={t} 必須在 [0, 1] 範圍內")
    
    result = Pose()
    
    # 位置線性插值
    result.position.x = (1-t) * pose1.position.x + t * pose2.position.x
    result.position.y = (1-t) * pose1.position.y + t * pose2.position.y
    result.position.z = (1-t) * pose1.position.z + t * pose2.position.z
    
    # 四元數球面線性插值 (SLERP)
    q1 = normalize_quaternion(np.array([pose1.orientation.x, pose1.orientation.y,
                                       pose1.orientation.z, pose1.orientation.w]))
    q2 = normalize_quaternion(np.array([pose2.orientation.x, pose2.orientation.y,
                                       pose2.orientation.z, pose2.orientation.w]))
    
    q_interp = slerp_quaternion(q1, q2, t)
    
    result.orientation.x = q_interp[0]
    result.orientation.y = q_interp[1]
    result.orientation.z = q_interp[2]
    result.orientation.w = q_interp[3]
    
    return result


def slerp_quaternion(q1, q2, t):
    """
    四元數球面線性插值
    
    Args:
        q1, q2: numpy.array - 正規化四元數 [x, y, z, w]
        t: float - 插值參數 (0-1)
    
    Returns:
        numpy.array - 插值結果四元數
    """
    dot_product = np.dot(q1, q2)
    
    # 如果點積為負，選擇較短路徑
    if dot_product < 0.0:
        q2 = -q2
        dot_product = -dot_product
    
    # 如果四元數非常接近，使用線性插值
    if dot_product > 0.9995:
        result = q1 + t * (q2 - q1)
        return normalize_quaternion(result)
    
    # 使用標準 SLERP
    theta = np.arccos(abs(dot_product))
    sin_theta = np.sin(theta)
    
    factor1 = np.sin((1.0 - t) * theta) / sin_theta
    factor2 = np.sin(t * theta) / sin_theta
    
    result = factor1 * q1 + factor2 * q2
    return normalize_quaternion(result)