import rclpy
from rclpy.node import Node
from model_module.magic_cube import RobotModel  # 自訂 model.py 模組

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')

        # 機器人模型
        self.robot_model = RobotModel()

        # # Timer，每次發送 1 筆指令（從 queue 中）
        self.timer = self.create_timer(1, self.send_next_point)
        self.get_logger().info('MotionController ready.')

    # point to point
    def move_p(self, target_pose, acc = 10, vel=5 , mode=6):

        # 1. Inverse kinematics
        joint_target = self.robot_model.inverse_kinematics(target_pose)
        print(joint_target)

        return


    # Timer callback：每次送一 batch（10 點）
    def send_next_point(self):
        print("123")
        self.move_p(target_pose=[345.0,0.0,0.0])


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
