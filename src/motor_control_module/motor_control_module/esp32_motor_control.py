from uros_interface.msg import Joint2DArr,JointArr
from common_msgs.msg import MultipleM,SingleM
from std_msgs.msg import Float32MultiArray,String
from rclpy.node import Node
import rclpy
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from uros_interface.srv import ESMCmd


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Async service client
        self.cli = self.create_client(ESMCmd, '/esm_command')
        self.waiting_for_result = False

        # Publishers
        self.esp_control_publisher = self.create_publisher(Joint2DArr,'/theta_command',10)
        self.motors_info_publisher = self.create_publisher(MultipleM,'/multi_motor_info',10) 

        #Subscribers
        self.pos_ref_sub = self.create_subscription(Float32MultiArray, '/motor_position_ref',self.motor_cmd_callback , 10) #motor_node
        self.esp_info_subscriber = self.create_subscription(JointArr, '/theta_feedback', self.esp_info_callback ,qos_profile)
        self.servo_cmd_subscriber = self.create_subscription(String, '/servo_cmd', self.servo_cmd_callback , 10)
    
        # Timer
        self.timer = self.create_timer(2, self.timer_callback)  #20Hz       


    def motor_cmd_callback(self,msg:Float32MultiArray):
        pos_ref_queue = np.reshape(msg.data,(10,3))
        self.pub_esp_cmd(pos_ref_queue)

    def pub_esp_cmd(self,pos_ref_queue):
        msg = Joint2DArr()
        msg.theta_2d_arr = [JointArr() for _ in range(10)]
        for i in range (10):
            print([pos_ref_queue[i][0],pos_ref_queue[i][1],pos_ref_queue[i][2],0.0])
            msg.theta_2d_arr[i].theta_arr = [float(pos_ref_queue[i][0]),float(pos_ref_queue[i][1]),float(pos_ref_queue[i][2]),0.0]
        print("Publishing ESP command:", msg.theta_2d_arr)
        self.esp_control_publisher.publish(msg)

    def esp_info_callback(self,msg:JointArr):
        motors_info = MultipleM()
        motors_info.quantity = 3
        motors_info.motor_info = [SingleM() for _ in range(motors_info.quantity)]
        motors_info.motor_info[0].id = 1
        motors_info.motor_info[0].fb_position = float(msg.theta_arr[0])
        motors_info.motor_info[1].id = 2
        motors_info.motor_info[1].fb_position = float(msg.theta_arr[1])
        motors_info.motor_info[2].id = 3
        motors_info.motor_info[2].fb_position = float(msg.theta_arr[2])
        self.motors_info_publisher.publish(motors_info)
        # print("esp_pub")
    
    def timer_callback(self):
        """This runs at 20Hz."""
        # print("motor_control")

    def servo_cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "servo on":
            self.get_logger().info("Received servo ON command")
            self.call_servo(on=True)
        elif cmd == "servo off":
            self.get_logger().info("Received servo OFF command")
            self.call_servo(on=False)
        else:
            self.get_logger().warn(f"Unknown servo command: {msg.data}")

    def call_servo(self, on=True):
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service not available')
            return

        request = ESMCmd.Request()
        request.servo_status = on
        request.mode = 4
        request.speed_limit = 50
        request.lpf = 10

        future = self.cli.call_async(request)
        self.waiting_for_result = True

        def handle_response(fut):
            self.waiting_for_result = False
            if fut.result() is not None:
                self.get_logger().info(f"Service {'ON' if on else 'OFF'} call succeeded")
            else:
                self.get_logger().error(f"Service {'ON' if on else 'OFF'} call failed")

        future.add_done_callback(handle_response)


def main(args=None):
    rclpy.init(args=args)
    ui_node = MotorController()
    rclpy.spin(ui_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()