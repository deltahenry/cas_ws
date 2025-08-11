import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from common_msgs.msg import MotionCmd,MultipleM, SingleM
from collections import deque
from model_module.magic_cube import RobotModel  # 自訂 model.py 模組
from copy import deepcopy
import numpy as np
from std_msgs.msg import Float32MultiArray
import math


time_period = 0.04  # Timer 的時間間隔，單位為秒
batch_size = 10  # 每次發送的 batch 大小

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        # 機器人模型
        self.robot_model = RobotModel()
        # 初始化 Queue
        self.trajectory_queue = deque()

        # ROS 2 介面
        #Subscriber
        self.create_subscription(MotionCmd, '/motion_cmd', self.motion_cmd_callback, 10)
        self.motors_info_sub = self.create_subscription(MultipleM,'/multi_motor_info',self.motors_info_callback,10)
        #Publisher
        self.motor_cmd_publisher = self.create_publisher(Float32MultiArray, '/motor_position_ref', 10)
        # self.done_pub = self.create_publisher(Bool, '/motion_finished', 10)

        # Timer，每次發送 1 筆指令（從 queue 中）
        self.timer = self.create_timer(time_period, self.send_next_batch)

        # 初始化狀態
        self.motion_finished = True
        self.init_finished = False
        
        self.check_home = False
        self.check_position = False
        
        self.current_motor_len = [0.0, 0.0, 0.0]
        self.current_cartesian_pose = [0.0,0.0,0.0]  # 初始 cartesian pose x y yaw
        self.last_sent_joint_command = [0.0, 0.0, 0.0]  # 上次發送的關節指令
        self.get_logger().info('MotionController ready.')

    #--motion command callback--
    def motion_cmd_callback(self, msg=MotionCmd):
        self.get_logger().info(f"Received motion command: {msg.command_type}")

        if  self.motion_finished:
            if msg.command_type == MotionCmd.TYPE_HOME:
                self.move_home()  # 回到機器人原點
                self.motion_finished = False  # 收到指令後，將 motion_finished 設為 False

            elif msg.command_type == MotionCmd.TYPE_GOTO:
                self.move_p(msg.pose_data,msg.speed)  # 移動到指定的 cartesian 位置
                self.motion_finished = False  # 收到指令後，將 motion_finished 設為 False

            elif msg.command_type == MotionCmd.TYPE_GOTO_RELATIVE:
                # 相對移動到指定的 cartesian 位置
                relative_pose = [
                    self.current_cartesian_pose[0] + msg.pose_data[0],
                    self.current_cartesian_pose[1] + msg.pose_data[1],
                    self.current_cartesian_pose[2] + msg.pose_data[2]/57.29  # 將角度轉換為弧度
                ]
                print(f"Relative pose: {relative_pose}")
                self.move_p(relative_pose, msg.speed)
                self.motion_finished = False  # 收到指令後，將 motion_finished 設為 False
            else:
                self.get_logger().error(f"Unknown command type: {msg.command_type}")
        else:
            self.get_logger().warn("Cannot accept new motion command while in motion.")
            

    def motors_info_callback(self, msg:MultipleM):
        self.current_motor_len = [msg.motor_info[0].fb_position,msg.motor_info[1].fb_position,msg.motor_info[2].fb_position]
        print("motor_info callback",self.current_motor_len)


    #--motion function--
    # back to motor home position
    def move_home(self):
        self.get_logger().info("Received set_home command")
        home_pose = self.robot_model.home_position  # 機器人模型的 home position
       
        # 1. Inverse kinematics
        joint_target = self.robot_model.inverse_kinematics(home_pose)  # 使用機器人模型的逆運動學計算關節目標位置
       
        # 2. Interpolate trajectory
        current_joint_pos = self.current_motor_len
        home_trajectory = self.interpolate_joint_space(current_joint_pos, joint_target,vdes=10.0)  # 使用插值函數計算關節軌跡
       
        # 3. Add to trajectory queue
        self.load_trajectory(home_trajectory)  # 將軌跡載入 queue
        
        #check if motors are in home position
        self.check_home = True

    def move_p(self,cartesian_pos_cmd,speed):
        self.get_logger().info("Received set_cartesian_position command")

        in_workspace = self.robot_model.check_workspace_limits(cartesian_pos_cmd)
        
        print("Target position:", cartesian_pos_cmd)

        if in_workspace:

            # 1. 插值 Cartesian trajectory
            cartesian_trajectory = self.interpolate_cartesian(self.current_cartesian_pose, cartesian_pos_cmd, speed)

            # 2. 對每一點做 Inverse Kinematics，轉成 joint trajectory
            joint_trajectory = []
            for cartesian_point in cartesian_trajectory:
                joint_point = self.robot_model.inverse_kinematics(cartesian_point)
                # joint_point = -self.robot_model.inverse_kinematics(cartesian_point)  # 注意: reverse sign
                joint_trajectory.append(joint_point)

            # 3. 載入 joint_trajectory 進隊列（會自動切成 batch）
            self.load_trajectory(joint_trajectory)

            # 4. 紀錄目標位置，用於校驗用
            self.cartesian_pos_cmd = cartesian_pos_cmd  # 記錄目標位置
            self.check_position = True

        else:
            self.get_logger().error("Target position is out of workspace limits.")
            
    def interpolate_cartesian(self, start_pose, end_pose, vdes):
        dt = time_period / batch_size
        ramp_ratio = 0.2
        rot_weight = 588.0  # <<< 你可以根據實際結構調整

        def angle_diff(a, b):
            return (b - a + math.pi) % (2 * math.pi) - math.pi

        # 差值計算
        d_raw = [
            e - s if i < 2 else angle_diff(s, e)
            for i, (s, e) in enumerate(zip(start_pose, end_pose))
        ]

        # 加入角度等效距離
        d_weighted = [
            d_raw[0],
            d_raw[1],
            d_raw[2] * rot_weight
        ]

        # 計算總距離
        total_dist = sum((x**2 for x in d_weighted))**0.5
        step_dist = vdes * dt
        total_steps = max(int(total_dist / step_dist), 1)

        ramp_steps = max(int(total_steps * ramp_ratio), 1)
        mid_steps = total_steps - 2 * ramp_steps

        # 梯形速度 profile
        accel_profile = [(vdes * (i + 1) / ramp_steps) for i in range(ramp_steps)]
        constant_profile = [vdes] * mid_steps
        decel_profile = [(vdes * (ramp_steps - i) / ramp_steps) for i in range(ramp_steps)]
        speed_profile = accel_profile + constant_profile + decel_profile

        # 積分為距離比例
        cumulative_dist = 0.0
        dist_list = []
        for v in speed_profile:
            cumulative_dist += v * dt
            dist_list.append(cumulative_dist)

        ratios = [d / cumulative_dist for d in dist_list]

        # 插補
        trajectory = [
            [start_pose[i] + r * d_raw[i] for i in range(len(d_raw))]
            for r in ratios
        ]

        return trajectory

    def interpolate_joint_space(self, start_pose, end_pose, vdes):
        dt = time_period / batch_size

        ramp_ratio = 0.2  # 每邊加減速比例 (20%)
        d = [e - s for s, e in zip(start_pose, end_pose)]

        total_dist = sum((x**2 for x in d))**0.5
        step_dist = vdes * dt
        total_steps = max(int(total_dist / step_dist), 1)

        ramp_steps = max(int(total_steps * ramp_ratio), 1)
        mid_steps = total_steps - 2 * ramp_steps

        # -- 梯形速度 profile --
        # 前段加速：從 0 線性增至 vdes
        accel_profile = [(vdes * (i + 1) / ramp_steps) for i in range(ramp_steps)]

        # 中段恆速
        constant_profile = [vdes] * mid_steps

        # 後段減速：從 vdes 線性降至 0
        decel_profile = [(vdes * (ramp_steps - i) / ramp_steps) for i in range(ramp_steps)]

        speed_profile = accel_profile + constant_profile + decel_profile

        # -- 將速度積分為累積距離 --
        dist_acc = 0.0
        distances = []
        for speed in speed_profile:
            dist_acc += speed * dt
            distances.append(dist_acc)

        # 正規化為比例
        normalized = [d / dist_acc for d in distances]

        # 插值位置
        trajectory = [
            [start_pose[i] + ratio * d[i] for i in range(len(d))]
            for ratio in normalized
        ]

        return trajectory

    def load_trajectory(self, trajectory):
        """接收一條 joint 軌跡（List of [M1, M2, M3]），分批後放入 queue"""
        batches = self.split_into_batches(trajectory, batch_size=10)
        self.trajectory_queue.clear()
        self.trajectory_queue.extend(batches)

    # # 批次分割工具
    # 將 trajectory 分割成多個 batch
    def split_into_batches(self, trajectory, batch_size):
        batches = []
        for i in range(0, len(trajectory), batch_size):
            batch = trajectory[i:i+batch_size]
            # 補齊最後一包
            if len(batch) < batch_size:
                last_point = deepcopy(batch[-1])
                while len(batch) < batch_size:
                    batch.append(deepcopy(last_point))
            batches.append(batch)
        return batches

    # Timer callback：每次送一 batch（10 點）
    def send_next_batch(self):
        # 情況 1：如果有軌跡資料（送出一個 batch）
        if self.trajectory_queue:
            batch = self.trajectory_queue.popleft()
            self.send_motor_command(batch)
            self.last_sent_joint_command = batch[-1]
            return

        # 情況 2：還沒初始化，等待馬達抵達 [0, 0, 0]
        if not self.init_finished:
            print("Waiting for motor initialization...")
            if np.allclose(self.current_motor_len, [0, 0, 0], atol=0.05):
                self.init_finished = True
            return

        # 情況 3：初始化完成，且沒有軌跡，判斷是否到達目標
        if np.allclose(self.current_motor_len, self.last_sent_joint_command, atol=0.05):
            # print("Motors have arrived.")
            self.motion_finished = True

            # 狀態處理：是否是回 Home 或 GOTO
            if self.check_home:
                # print("Motor is in home position.")
                self.current_cartesian_pose = self.robot_model.home_position
                self.check_home = False

            elif self.check_position:
                # print("Motor is in target position.")
                self.current_cartesian_pose = self.cartesian_pos_cmd
                self.check_position = False
        else:
            # 尚未到達，持續追蹤命令
            # print("Tracking...")
            self.send_motor_command([self.last_sent_joint_command] * 10)

    # 發送馬達命令
    def send_motor_command(self, batch):
        # motor_positions 可能是 list of float 或 np.array
        flat_positions = [val for joint in batch for val in joint]  # 展平為一維

        msg = Float32MultiArray()
        msg.data = flat_positions
        self.motor_cmd_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
