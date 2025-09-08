#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
from .vision import compensate_cabinet, compensate_cabinet_test


class VisionCompensationNode(Node):
    def __init__(self):
        super().__init__('vision_compensation_node')
        
        self.image_now = None
        
        # Subscribers
        self.detection_cmd_subscriber = self.create_subscription(
            String,
            '/detection_cmd',
            self.detection_cmd_callback,
            10
        )
        
        self.camera_image_subscriber = self.create_subscription(
            Image,
            '/camera_image',
            self.camera_image_callback,
            10
        )
        
        # Publisher
        self.compensate_pose_publisher = self.create_publisher(
            Float32MultiArray,
            '/compensate_pose',
            10
        )
        
        self.get_logger().info('Vision compensation node initialized')
        
    def ros_image_to_numpy(self, ros_image):
        """Convert ROS Image message to numpy array without cv_bridge"""
        height = ros_image.height
        width = ros_image.width
        
        # Convert bytes to numpy array
        if ros_image.encoding == "bgr8":
            channels = 3
            dtype = np.uint8
        elif ros_image.encoding == "mono8":
            channels = 1
            dtype = np.uint8
        else:
            raise ValueError(f"Unsupported encoding: {ros_image.encoding}")
        
        # Convert from bytes to numpy array
        np_array = np.frombuffer(ros_image.data, dtype=dtype)
        
        if channels == 1:
            cv_image = np_array.reshape((height, width))
        else:
            cv_image = np_array.reshape((height, width, channels))
        
        return cv_image

    def camera_image_callback(self, msg: Image):
        """Store current image from camera"""
        try:
            self.image_now = self.ros_image_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            
    def detection_cmd_callback(self, msg: String):
        """Handle detection commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        if command == "start_detect":
            self.handle_start_detect()
        elif command == "start_detect_0":
            self.handle_start_detect_0()
        elif command == "start_test":
            self.handle_start_test()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            
    def handle_start_detect(self):
        """Handle start_detect command"""
        if self.image_now is None:
            self.get_logger().error('No current image available')
            return
            
        golden_path = os.path.expanduser('~/image_sample2D_golden(x=0,z=112).png')
        
        if not os.path.exists(golden_path):
            self.get_logger().error(f'Golden sample not found: {golden_path}')
            return
            
        try:
            x, z = compensate_cabinet(golden_path, self.image_now)
            self.publish_compensation(x, z)
            self.get_logger().info(f'Detect compensation calculated: X={x:.2f}, Z={z:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in detect compensation calculation: {str(e)}')
            
    def handle_start_detect_0(self):
        """Handle start_detect_0 command"""
        golden_path = os.path.expanduser('~/image_sample2D_golden(x=0,z=112).png')
        test_path = os.path.expanduser('~/image_sample2D_golden(x=0,z=112).png')  # Same as golden
        
        if not os.path.exists(golden_path):
            self.get_logger().error(f'Golden sample not found: {golden_path}')
            return
            
        if not os.path.exists(test_path):
            self.get_logger().error(f'Test image not found: {test_path}')
            return
            
        try:
            x, z = compensate_cabinet_test(golden_path, test_path)
            self.publish_compensation(x, z)
            self.get_logger().info(f'Detect_0 compensation calculated: X={x:.2f}, Z={z:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in detect_0 compensation calculation: {str(e)}')
            
    def handle_start_test(self):
        """Handle start_test command"""
        golden_path = os.path.expanduser('~/image_sample/2D_golden(x=0,z=112).png')
        test_path = os.path.expanduser('~/image_sample/2D(x=25,z=117).png')
        
        if not os.path.exists(golden_path):
            self.get_logger().error(f'Golden sample not found: {golden_path}')
            return
            
        if not os.path.exists(test_path):
            self.get_logger().error(f'Test image not found: {test_path}')
            return
            
        try:
            x, z = compensate_cabinet_test(golden_path, test_path)
            self.publish_compensation(x, z)
            self.get_logger().info(f'Test compensation calculated: X={x:.2f}, Z={z:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in test compensation calculation: {str(e)}')
            
    def publish_compensation(self, x: float, z: float):
        """Publish compensation values"""
        msg = Float32MultiArray()
        msg.data = [float(x), float(z)]
        
        self.compensate_pose_publisher.publish(msg)
        
    def compensate_pose_callback(self, msg: Float32MultiArray):
        """Example callback for receiving compensation pose"""
        if len(msg.data) >= 2:
            X, Z = msg.data[:2]
            self.get_logger().info(f"Received compensation: X={X:.2f}, Z={Z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    
    node = VisionCompensationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()