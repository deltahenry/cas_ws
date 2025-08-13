import rclpy
from rclpy.node import Node
import numpy as np
import math

class RobotModel:
    def __init__(self):
        self.magic_cube_model()
    
    #model information
    def magic_cube_model(self):

        self.home_position = [0.0, 0.0, 0.0] #robot home position
        #link length

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
        
        joint_length = []
        
        x, y, yaw = target_pose
        # for x, y, yaw in target_pose:
        M1 = -400.0 * -sin(yaw) + x
        M2 = -880.0 * -sin(yaw) + x
        M3 = y #need to change
        joint_length = [-M3,-M2,-M1]

        return joint_length

    # def tcp_transform(self, x_move, yaw_move, delta_y):

    #     Xp_x = x_move*math.cos(yaw_move)
    #     Xp_y = -x_move*math.sin(yaw_move) + delta_y

    #     Xcc_x = delta_y*math.sin(yaw_move) + x_move*math.cos(yaw_move)
    #     Xcc_y = -delta_y*math.cos(yaw_move) + self.x_move*math.sin(yaw_move) + delta_y

    #     m = (Xcc_y - Xp_y) / (Xcc_x - Xp_x) if (Xcc_x - Xp_x) != 0 else float('inf')  # slope

    #     Xc_y = 0
    #     Xc_x = (Xc_y - Xcc_y)/ m + Xcc_x if m != 0 else Xcc_x  # x coordinate of the center point
    #     clipper_len = math.sqrt((Xc_x-Xcc_x)**2 + (Xc_y-Xcc_y)**2)  # distance from the center point to the clipper

    #     Xc_x = Xc_x + 345.0  # offset for the center point

    #     return Xc_x, clipper_len, yaw_move

    def check_workspace_limits(self, target_pose):
        sin = math.sin
        cos = math.cos
        x, y, yaw = target_pose

        if yaw<=0:
            if -880.0*-sin(yaw) + x>-139.5:
                return True
            else:
                return False
        else:
            if -880.0*sin(yaw) + x <160.5:
                return True
            else:
                return False

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