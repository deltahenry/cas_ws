
class RobotModel:
    def __init__(self):
        self.n_joints = 6
        self.joint_limits = [...]
        self.tool_offset = [...]
        self.home_position = [...]

    def magic_cube_model(self):
        #model information
        #link length
        #joint position/definition
        pass
    
    def forward_kinematics(self, joint_angles):
        current_pose = joint_angles
        return current_pose
    
    def inverse_kinematics(self, target_pose):
        joint_angles = target_pose
        return joint_angles

    def check_joint_limits(self, joint_angles):
        pass

    #for velocity control
    def compute_jacobian(self, joint_angles):
        pass
    
    #for dynamic control(torque)
    def compute_gravity(self, joint_angles):
        pass

