import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from common_msgs.msg import PoseCommand
from collections import deque
from model_module.magic_cube import RobotModel  # 自訂 model.py 模組

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        # 機器人模型
        self.robot_model = RobotModel()

        # 初始化 Queue
        self.trajectory_queue = deque()

        # ROS 2 介面
        
        #Subscriber
        self.create_subscription(PoseCommand, '/position_cmd', self.pose_callback, 10)
        
        #Publisher
        # self.motor_pub = self.create_publisher(JointPosition, '/motor_position_ref', 10)
        # self.done_pub = self.create_publisher(Bool, '/motion_finished', 10)

        # Timer，每次發送 1 筆指令（從 queue 中）
        self.timer = self.create_timer(1, self.send_next_point)

        self.get_logger().info('MotionController ready.')

    # 姿態目標處理
    def pose_callback(self, msg):
        self.get_logger().info("Received pose command")
        
        if msg.type == 'P':
            self.move_p(msg.pose)
        elif msg.type == 'L':
            self.move_l(msg.pose)
        elif msg.type == 'H':
            self.move_home(msg.pose)


    #--motion function--
    # back to motor home position
    # def move_home(self, msg):
    #     self.get_logger().info("Received set_home command")

    #     target_joint = msg.joint_position
    #     trajectory = self.robot_model.interpolate_joint(self.current_joint_state, target_joint, 30)

    #     if not self.robot_model.check_trajectory_limits(trajectory):
    #         self.get_logger().error("Home path violates joint limits")
    #         return

    #     batches = self.split_into_batches(trajectory, 10)
    #     for batch in batches:
    #         self.trajectory_queue.append(batch)

    # point to point
    def move_p(self, target_pose):

        # 1. Inverse kinematics
        joint_target = self.robot_model.inverse_kinematics(target_pose)
        print(joint_target)

        # # 2. Check joint limits
        # if not self.robot_model.check_joint_limits(joint_target):
        #     self.get_logger().error("Joint target exceeds limits!")
        #     return

        # # 3. Trajectory interpolation (joint space)
        # joint_traj = self.robot_model.interpolate_joint_space(self.current_joint_pos, joint_target)
        
        # # 4. Load into queue
        # self.trajectory_queue.extend(joint_traj)
    # linear move in cartesian space
    # def move_l(self, target_pose):
    #     # 1. Cartesian interpolation (e.g., 50 steps from current_pose to target_pose)
    #     cartesian_path = self.robot_model.interpolate_cartesian(self.current_pose, target_pose)

    #     # 2. Inverse Kinematics for each Cartesian point
    #     joint_traj = []
    #     for pose in cartesian_path:
    #         joint = self.robot_model.inverse_kinematics(pose)
    #         if joint is None or not self.robot_model.check_joint_limits(joint):
    #             self.get_logger().error("IK failed or out of joint range")
    #             return
    #         joint_traj.append(joint)

    #     # 3. Load into queue
    #     self.trajectory_queue.extend(joint_traj)


    # #--interpolate function--
    # # interpolate the cartesian trajectory
    # def interpolate_cartesian(start_pose, end_pose, steps):
    #     cartesian_trajectory = end_pose - start_pose
    #     return cartesian_trajectory
    # # interpolate the joint command
    # def interpolate_joint_space(joint_start_pose, joint_end_pose, steps):
    #     joint_trajectory = joint_end_pose - joint_start_pose
    #     return joint_trajectory


    # # 批次分割工具
    # def split_into_batches(self, trajectory, batch_size):
    #     return [trajectory[i:i+batch_size] for i in range(0, len(trajectory), batch_size)]


    # Timer callback：每次送一 batch（10 點）
    def send_next_point(self):
        print("123")
        self.move_p(target_pose=[345.0,0.0,0.0])

        # if not self.trajectory_queue:
        #     return

        # batch = self.trajectory_queue.popleft()
        # for joint in batch:
        #     self.send_motor_command(joint)
        #     self.current_joint_state = joint  # 更新目前 joint state

        # # 發送完成標記（你也可以設成延遲最後一包再送）
        # if not self.trajectory_queue:
        #     self.done_pub.publish(Bool(data=True))


    # # 發送馬達命令
    # def send_motor_command(self, joint_angles,joint_velocity,joint_torque):
    #     msg = JointPosition()
    #     msg.joint_position = joint_angles
    #     self.motor_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
