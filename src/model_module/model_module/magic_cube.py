import rclpy
from rclpy.node import Node
import numpy as np
import math

class RobotModel:
    def __init__(self):
        self.magic_cube_model()
    
    #model information
    def magic_cube_model(self):
        
        #link length
        #joint position/definition
        self.real_motor_home_position = [436.5, 525.5, 0.0] #m1 len ...

        #Relative Position definition
        self.P_J1MC = np.array([[-250],[-318],[0]])
        self.P_J2MC = np.array([[250],[318],[0]])

        #Tool Frame

        return
    
    def forward_kinematics(self, joint_angles):
        current_pose = joint_angles
        return current_pose

    def inverse_kinematics(self, target_pose):
        sin = math.sin
        cos = math.cos
        sqrt = math.sqrt
        
        joint_length = []
        
        x, y, yaw = target_pose
        # for x, y, yaw in target_pose:
        J1_x = x + cos(yaw)*self.P_J1MC[0,0] - sin(yaw)*self.P_J1MC[1,0]
        J1_y = 0 + sin(yaw)*self.P_J1MC[0,0] - cos(yaw)*self.P_J1MC[1,0]
        M1 = J1_x + sqrt(205.5**2-(abs(J1_y)-abs(self.P_J1MC[1,0]))**2)-self.real_motor_home_position[0]

        J2_x = x + cos(yaw)*self.P_J2MC[0,0] - sin(yaw)*self.P_J2MC[1,0]
        J2_y = 0 + sin(yaw)*self.P_J2MC[0,0] - cos(yaw)*self.P_J2MC[1,0]
        M2 = J2_x - sqrt(205.5**2-(abs(J2_y)-abs(self.P_J2MC[1,0]))**2)-self.real_motor_home_position[1]

        M3 = y #need to change
        joint_length = [M1,M2,M3]

        return joint_length


    def check_joint_limits(self, joint_angles):
        pass

    #for velocity control
    def compute_jacobian(self, joint_angles):
        pass


    #for dynamic control(torque)
    def compute_gravity(self, joint_angles):
        pass





def main(args=None):
    robot_model = RobotModel()
    print(robot_model.inverse_kinematics(target_pose=[345.0,0.0,0.0]))

if __name__ == '__main__':
    main()