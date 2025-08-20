# motion_controller_with_velocity.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from common_msgs.msg import MotionCmd, MultipleM, MotionState, CurrentPose
from geometry_msgs.msg import Pose
from collections import deque
from copy import deepcopy
import numpy as np
import math
from PySide6.QtCore import QTimer

from model_module.magic_cube import RobotModel  # custom robot model


time_period = 0.04
batch_size = 10


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        self.robot_model = RobotModel()
        self.trajectory_queue = deque()

        # ROS interfaces
        self.create_subscription(MotionCmd, '/motion_cmd', self.motion_cmd_callback, 10)
        self.create_subscription(MultipleM, '/multi_motor_info', self.motors_info_callback, 10)
        self.motor_cmd_publisher = self.create_publisher(Float32MultiArray, '/motor_position_ref', 10)
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.current_cartesian_pose_publisher = self.create_publisher(CurrentPose, '/current_pose', 10)
        self.current_arm_pose_publisher = self.create_publisher(Pose, '/current_arm_pose', 10)

        # Timer
        self.timer = self.create_timer(time_period, self.send_next_batch)

        # State
        self.motion_finished = True
        self.init_finished = True
        self.current_motor_len = [-10.0, 0.0, 0.0]
        self.current_cartesian_pose = [0.0, 0.0, 0.0]
        self.last_sent_joint_command = [0.0, 0.0, 0.0]
        self.cartesian_pos_cmd = None
        self.check_home = False
        self.check_position = False

        # Velocity mode state
        self.velocity_mode_active = False
        self.velocity_vector = [0.0, 0.0, 0.0]

        self.get_logger().info("MotionController ready.")

    # -------------------- ROS Callbacks --------------------
    def motion_cmd_callback(self, msg: MotionCmd):

        if msg.command_type == MotionCmd.TYPE_HOME:
            if self.motion_finished:
                self.move_home()
                self.motion_finished = False
        
        elif msg.command_type == MotionCmd.TYPE_GOTO:
            if self.motion_finished:
                self.move_p(msg.pose_data, msg.speed)
                self.motion_finished = False
        
        elif msg.command_type == MotionCmd.TYPE_GOTO_RELATIVE:
            if self.motion_finished:
                relative_pose = [
                    self.current_cartesian_pose[0] + msg.pose_data[0],
                    self.current_cartesian_pose[1] + msg.pose_data[1],
                    self.current_cartesian_pose[2] + msg.pose_data[2]/57.29
                ]
                self.move_p(relative_pose, msg.speed)
                self.motion_finished = False
        
        elif msg.command_type == MotionCmd.TYPE_Y_MOVE:
            if self.motion_finished:
                abs_y = msg.pose_data[1]
                relative_pose = [
                    self.current_cartesian_pose[0],
                    abs_y,
                    self.current_cartesian_pose[2]
                ]
                self.move_p(relative_pose, msg.speed)
                self.motion_finished = False
        
        elif msg.command_type == MotionCmd.TYPE_VELOCITY:
            if msg.pose_data[0] == -100.0 and self.velocity_mode_active:
                # Stop velocity mode
                self.velocity_mode_active = False
                self.get_logger().info("Velocity mode stopped.")
                self.motion_finished = True
            else:
                self.velocity_mode_active = True
                self.velocity_vector = msg.pose_data  # [vx, vy, vyaw]
        
        else:
            self.get_logger().error(f"Unknown command type: {msg.command_type}")

    def motors_info_callback(self, msg: MultipleM):
        self.current_motor_len = [
            msg.motor_info[0].fb_position,
            msg.motor_info[1].fb_position,
            msg.motor_info[2].fb_position
        ]

    # -------------------- Motion Functions --------------------
    def move_home(self):
        home_pose = self.robot_model.home_position
        joint_target = self.robot_model.inverse_kinematics(home_pose)
        home_trajectory = self.interpolate_joint_space(self.current_motor_len, joint_target, vdes=10.0)
        self.load_trajectory(home_trajectory)
        self.check_home = True

    def move_p(self, cartesian_pos_cmd, speed):
        if not self.robot_model.check_workspace_limits(cartesian_pos_cmd):
            self.get_logger().error("Target position out of workspace limits.")
            return

        cartesian_trajectory = self.interpolate_cartesian(self.current_cartesian_pose, cartesian_pos_cmd, speed)
        joint_trajectory = [self.robot_model.inverse_kinematics(pt) for pt in cartesian_trajectory]
        self.load_trajectory(joint_trajectory)
        self.cartesian_pos_cmd = cartesian_pos_cmd
        self.check_position = True

    def move_velocity(self, vel_x=0.0, vel_y=0.0, vel_yaw=0.0):
        dt = time_period / batch_size
        current_pose = deepcopy(self.current_cartesian_pose)
        batch_cartesian = []

        for _ in range(batch_size):
            next_pose = [
                current_pose[0] + vel_x * dt,
                current_pose[1] + vel_y * dt,
                current_pose[2] + vel_yaw * dt
            ]
            batch_cartesian.append(next_pose)
            current_pose = next_pose  # update for next iteration

        batch_joints = [self.robot_model.inverse_kinematics(pose) for pose in batch_cartesian]
        self.send_motor_command(batch_joints)

        # 更新 internal pose state 讓下一輪從這裡開始
        self.current_cartesian_pose = current_pose
        self.motion_finished = False

    # -------------------- Interpolation Utilities --------------------
    def interpolate_cartesian(self, start_pose, end_pose, vdes):
        dt = time_period / batch_size
        rot_weight = 588.0

        def angle_diff(a, b):
            return (b - a + math.pi) % (2 * math.pi) - math.pi

        d_raw = [
            e - s if i < 2 else angle_diff(s, e)
            for i, (s, e) in enumerate(zip(start_pose, end_pose))
        ]

        total_dist = sum((x**2 for x in d_raw))**0.5
        step_dist = vdes * dt
        total_steps = max(int(total_dist / step_dist), 1)

        trajectory = [
            [start_pose[i] + (j+1)/total_steps * d_raw[i] for i in range(3)]
            for j in range(total_steps)
        ]
        return trajectory

    def interpolate_joint_space(self, start_pose, end_pose, vdes):
        dt = time_period / batch_size
        d = [e - s for s, e in zip(start_pose, end_pose)]
        total_dist = sum(x**2 for x in d)**0.5
        step_dist = vdes * dt
        total_steps = max(int(total_dist / step_dist), 1)
        trajectory = [
            [start_pose[i] + (j+1)/total_steps*d[i] for i in range(3)]
            for j in range(total_steps)
        ]
        return trajectory

    # -------------------- Queue & Motor --------------------
    def load_trajectory(self, trajectory):
        batches = self.split_into_batches(trajectory, batch_size)
        self.trajectory_queue.clear()
        self.trajectory_queue.extend(batches)

    def split_into_batches(self, trajectory, batch_size):
        batches = []
        for i in range(0, len(trajectory), batch_size):
            batch = trajectory[i:i+batch_size]
            if len(batch) < batch_size:
                last_point = deepcopy(batch[-1])
                while len(batch) < batch_size:
                    batch.append(deepcopy(last_point))
            batches.append(batch)
        return batches

    def send_motor_command(self, batch):
        flat_positions = [val for joint in batch for val in joint]
        msg = Float32MultiArray()
        msg.data = flat_positions
        self.motor_cmd_publisher.publish(msg)

    # -------------------- Timer --------------------
    def send_next_batch(self):
        # Publish motion state
        self.motion_state_publisher.publish(MotionState(
            motion_finish=self.motion_finished,
            init_finish=self.init_finished,
        ))

        # Publish current Cartesian pose
        x, y, yaw = self.current_cartesian_pose
        self.current_cartesian_pose_publisher.publish(CurrentPose(pose_data=[x, y, yaw*57.29]))

        # Publish arm Pose
        arm_pose = Pose()
        arm_pose.position.x = x
        arm_pose.position.y = y
        arm_pose.position.z = 0.0
        arm_pose.orientation.w = 1.0
        self.current_arm_pose_publisher.publish(arm_pose)

        # If trajectory queue has data, send next batch
        if self.trajectory_queue:
            batch = self.trajectory_queue.popleft()
            self.send_motor_command(batch)
            self.last_sent_joint_command = batch[-1]
            return

        # Handle velocity mode
        if self.velocity_mode_active:
            print("Velocity mode active, sending velocity command")
            vx, vy, vyaw = self.velocity_vector
            self.move_velocity(vx, vy, vyaw)
            return

        # Check if motors reached last target
        if np.allclose(self.current_motor_len, self.last_sent_joint_command, atol=0.05):
            self.motion_finished = True
            if self.check_home:
                self.current_cartesian_pose = self.robot_model.home_position
                self.check_home = False
            elif self.check_position:
                self.current_cartesian_pose = self.cartesian_pos_cmd
                self.check_position = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
