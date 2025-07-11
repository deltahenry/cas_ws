import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from common_msgs.msg import MotionCmd
from collections import deque
from model_module.magic_cube import RobotModel  # 自訂 model.py 模組
from copy import deepcopy
import numpy as np
from std_msgs.msg import Float32MultiArray


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
        self.current_cartesian_pose = self.robot_model.home_position  # 初始 cartesian pose
        self.last_sent_joint_command = [0.0, 0.0, 0.0]  # 上次發送的關節指令


        self.get_logger().info('MotionController ready.')

    #--motion command callback--
    def motion_cmd_callback(self, msg=MotionCmd):
        self.get_logger().info(f"Received motion command: {msg.command_type}")
        
        self.motion_finished = False  # 收到指令後，將 motion_finished 設為 False

        if msg.command_type == MotionCmd.TYPE_HOME:
            self.move_home()  # 回到機器人原點

        elif msg.command_type == MotionCmd.TYPE_GOTO:
            self.move_p(msg.pose_data,msg.speed)  # 移動到指定的 cartesian 位置

        else:
            self.get_logger().error(f"Unknown command type: {msg.command_type}")


    #--motion function--
    # back to motor home position
    def move_home(self):
        self.get_logger().info("Received set_home command")
        home_pose = self.robot_model.home_position  # 機器人模型的 home position
       
        # 1. Inverse kinematics
        joint_target = self.robot_model.inverse_kinematics(home_pose)  # 使用機器人模型的逆運動學計算關節目標位置
       
        # 2. Interpolate trajectory
        current_joint_pos = self.current_motor_len
        home_trajectory = self.interpolate_joint_space(current_joint_pos, joint_target,vdes=1)  # 使用插值函數計算關節軌跡
       
        # 3. Add to trajectory queue
        self.load_trajectory(home_trajectory)  # 將軌跡載入 queue
        
        #check if motors are in home position
        self.check_home = True

    def move_p(self,cartesian_pos_cmd,speed):
        self.get_logger().info("Received set_cartesian_position command")

        # 1. 插值 Cartesian trajectory
        self.current_cartesian_pose = [0.0, 0.0, 0.0]  # 可根據實際狀況更新
        self.cartesian_pos_cmd = cartesian_pos_cmd  # 記錄目標位置

        cartesian_trajectory = self.interpolate_cartesian(self.current_cartesian_pose, cartesian_pos_cmd, speed)

        # 2. 對每一點做 Inverse Kinematics，轉成 joint trajectory
        # joint_trajectory = []
        # for cartesian_point in cartesian_trajectory:
        #     joint_point = self.robot_model.inverse_kinematics(cartesian_point)
        #     joint_trajectory.append(joint_point)

        # 3. 載入 joint_trajectory 進隊列（會自動切成 batch）
        self.load_trajectory(cartesian_trajectory)

        # 4. 紀錄目標位置，用於校驗用
        self.cartesian_pos_cmd = cartesian_pos_cmd
        self.check_position = True

    #--interpolate function--
    def interpolate_cartesian(self, start_pose, end_pose, vdes):
        dt = time_period / batch_size
        
        ramp_ratio=0.3 
        ramp_speed=0.5

        # 位移向量與總長度
        d = [e - s for s, e in zip(start_pose, end_pose)]
        total_dist = sum((x**2 for x in d))**0.5
        step_dist = vdes * dt
        total_steps = max(int(total_dist / step_dist), 1)

        # 計算各區段步數
        ramp_steps = max(int(total_steps * ramp_ratio), 1)
        mid_steps = total_steps - 2 * ramp_steps

        # 建立速度 profile
        speed_profile = (
            [ramp_speed * vdes] * ramp_steps +
            [vdes] * mid_steps +
            [ramp_speed * vdes] * ramp_steps
        )

        # 將每步速度轉為累積距離
        dist_acc = 0.0
        distances = []
        for speed in speed_profile:
            dist_acc += speed * dt
            distances.append(dist_acc)

        # 正規化為比例 [0 ~ 1]
        normalized = [d / dist_acc for d in distances]

        # 根據比例插值每一點
        trajectory = [
            [start_pose[i] + ratio * d[i] for i in range(len(d))]
            for ratio in normalized
        ]

        return trajectory
    
    # def interpolate_cartesian(self, start_pose, end_pose, vdes):
    #     dt = time_period / batch_size
    #     d = [e - s for s, e in zip(start_pose, end_pose)]
    #     total_dist = sum((x**2 for x in d))**0.5
    #     step_dist = vdes * dt
    #     total_steps = max(int(total_dist / step_dist), 1)

    #     # 速度 profile：前10% + 中間 + 後10%
    #     slow_steps = max(int(total_steps * 0.1), 1)
    #     fast_steps = total_steps - 2 * slow_steps
    #     speed_profile = [0.5 * vdes] * slow_steps + [vdes] * fast_steps + [0.5 * vdes] * slow_steps

    #     # 實際總距離需累加，轉為每一步「累積比率」
    #     dist_acc = 0.0
    #     distances = []
    #     for s in speed_profile:
    #         delta = s * dt
    #         dist_acc += delta
    #         distances.append(dist_acc)

    #     # 正規化 → 把最後一步對應到 1.0（總長度）
    #     normalized = [d / dist_acc for d in distances]

    #     # 根據比例產生每個點
    #     trajectory = []
    #     for ratio in normalized:
    #         point = [start_pose[i] + ratio * d[i] for i in range(len(d))]
    #         trajectory.append(point)

    #     return trajectory
    


    # interpolate the joint commandself.motion_finished = False
    def interpolate_joint_space(self,joint_start_pose, joint_end_pose, vdes):
        dM1_len, dM2_len, dM3_len = joint_end_pose[0] - joint_start_pose[0], \
                                    joint_end_pose[1] - joint_start_pose[1], \
                                    joint_end_pose[2] - joint_start_pose[2]
        
        max_dist = max(abs(dM1_len), abs(dM2_len), abs(dM3_len))
        
        steps = max(int(max_dist / vdes), 1)

        M1_step, M2_step, M3_step = dM1_len / steps, dM2_len / steps, dM3_len / steps
        return [[joint_start_pose[0] + i *M1_step, joint_start_pose[1] + i * M2_step, joint_start_pose[2] + i * M3_step] for i in range(1, steps + 1)]


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
        if not self.trajectory_queue:

            # self.current_motor_len = [-135.0, -136.0, 0.0]  # 模擬目前 joint state

            if self.check_home:
                if  np.allclose(self.current_motor_len, self.robot_model.home_position, atol=0.05):  # 假設這是 home position
                    self.get_logger().info("Motors are in home position.")

                    self.current_cartesian_pose = self.robot_model.home_position  # 更新目前 cartesian pose

                    self.init_finished = True
                    self.check_home = False
                    self.motion_finished = True

                else:
                    self.get_logger().warning("Motors are not in home position yet.")
                    self.send_motor_command([self.last_sent_joint_command] * 10)


            elif self.check_position:
                if np.allclose(self.current_cartesian_pose, self.cartesian_pos_cmd, atol=0.05):  # 假設這是目標 cartesian position
                    self.get_logger().info("Motors are in target position.")

                    self.current_cartesian_pose = self.cartesian_pos_cmd  # 更新目前 cartesian pose

                    self.check_position = False
                    self.motion_finished = True

                else:
                    self.get_logger().warning("Motors are not in target position yet.")
                    self.send_motor_command([self.last_sent_joint_command] * 10)

            return
        
        # 如果有 trajectory，取出一個 batch
        else:
            batch = self.trajectory_queue.popleft()
            self.send_motor_command(batch)
            self.last_sent_joint_command = batch[-1]


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
