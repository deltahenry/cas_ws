import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from common_msgs.msg import MotionCmd
from collections import deque
from model_module.magic_cube import RobotModel  # 自訂 model.py 模組
from copy import deepcopy


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
        self.create_subscription()
        
        #Publisher
        # self.motor_pub = self.create_publisher(JointPosition, '/motor_position_ref', 10)
        # self.done_pub = self.create_publisher(Bool, '/motion_finished', 10)

        # Timer，每次發送 1 筆指令（從 queue 中）
        self.timer = self.create_timer(1, self.send_next_point)

        self.get_logger().info('MotionController ready.')

    #--motion command callback--
    def motion_cmd_callback(self, msg=MotionCmd):
        if msg.command_type == MotionCmd.TYPE_HOME:
            self.move_home()  # 回到機器人原點

        else:
            self.get_logger().error(f"Unknown command type: {msg.command_type}")


    #--motion function--
    # back to motor home position
    def move_home(self):
        self.get_logger().info("Received set_home command")
        pose = self.robot_model.home_position  # 機器人模型的 home position
        # 1. Inverse kinematics
        joint_target = self.robot_model.inverse_kinematics(pose)
        # 2. Interpolate trajectory
        current_joint_pos = self.robot_model.forward_kinematics(pose)  # 取得目前關節位置
        home_trajectory = self.interpolate_joint_space(current_joint_pos, joint_target,vdes=0.1)  # 使用插值函數計算關節軌跡
        # 3. Add to trajectory queue
        self.trajectory_queue.extend(home_trajectory)


    #--interpolate function--
    
    # interpolate the cartesian trajectory
    def interpolate_cartesian(start_pose, end_pose, vdes):
        cartesian_trajectory = end_pose - start_pose
        return cartesian_trajectory
    
    # interpolate the joint command
    def interpolate_joint_space(joint_start_pose, joint_end_pose, vdes):
        dM1_len, dM2_len, dM3_len = joint_end_pose[0] - joint_start_pose[0], \
                                    joint_end_pose[1] - joint_start_pose[1], \
                                    joint_end_pose[2] - joint_start_pose[2]
        
        max_dist = max(abs(dM1_len), abs(dM2_len), abs(dM3_len))
        
        steps = max(int(max_dist / vdes), 1)

        M1_step, M2_step, M3_step = dM1_len / steps, dM2_len / steps, dM3_len / steps
        return [[joint_start_pose[0] + i *M1_step, joint_start_pose[1] + i * M2_step, joint_start_pose[2] + i * M3_step] for i in range(1, steps + 1)]


        # # 批次分割工具
    
    # 將 trajectory 分割成多個 batch
    def split_into_batches(self, trajectory, batch_size=10):
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
    def send_next_point(self):
        if not self.trajectory_queue:
            return

        batch = self.trajectory_queue.popleft()
        for joint in batch:
            self.send_motor_command(joint)
            self.current_joint_state = joint  # 更新目前 joint state

        # 發送完成標記（你也可以設成延遲最後一包再送）
        if not self.trajectory_queue:
            self.done_pub.publish(Bool(data=True))


    # 發送馬達命令
    def send_motor_command(self, joint_angles,joint_velocity,joint_torque):
        msg = JointPosition()
        msg.joint_position = joint_angles
        self.motor_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
