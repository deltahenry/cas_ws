#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class RealSenseCameraNode(Node):
    def __init__(self):
        super().__init__('realsense_camera_node')
        
        self.declare_parameter('camera.model', 'D435')
        self.declare_parameter('camera.count', 1)
        self.declare_parameter('camera.fps', 15)
        
        self.camera_model = self.get_parameter('camera.model').get_parameter_value().string_value
        self.camera_count = self.get_parameter('camera.count').get_parameter_value().integer_value
        self.fps = self.get_parameter('camera.fps').get_parameter_value().integer_value
        
        self.publisher = self.create_publisher(Image, '/camera_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.setup_camera()
        
    def setup_camera(self):
        try:
            # Try to import pyrealsense2, but don't fail if not available
            try:
                import pyrealsense2 as rs
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, self.fps)
                self.pipeline.start(self.config)
                self.has_realsense = True
                self.get_logger().info(f'RealSense {self.camera_model} camera initialized with {self.fps} FPS')
            except ImportError:
                self.has_realsense = False
                self.get_logger().warn('pyrealsense2 not available, using dummy camera')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {str(e)}')
            self.has_realsense = False
            
    def numpy_to_ros_image(self, cv_image):
        """Convert numpy array to ROS Image message without cv_bridge"""
        ros_image = Image()
        
        if len(cv_image.shape) == 3:
            height, width, channels = cv_image.shape
            if channels == 3:
                ros_image.encoding = "bgr8"
            elif channels == 4:
                ros_image.encoding = "bgra8"
        elif len(cv_image.shape) == 2:
            height, width = cv_image.shape
            channels = 1
            ros_image.encoding = "mono8"
        else:
            raise ValueError("Unsupported image format")
        
        ros_image.height = height
        ros_image.width = width
        ros_image.step = width * channels
        ros_image.is_bigendian = False
        ros_image.data = cv_image.tobytes()
        
        return ros_image
            
    def timer_callback(self):
        try:
            if self.has_realsense:
                import pyrealsense2 as rs
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    return
                    
                color_image = np.asanyarray(color_frame.get_data())
            else:
                # Create dummy image for testing
                color_image = np.zeros((480, 640, 3), dtype=np.uint8)
                # Add some pattern for testing
                color_image[200:280, 280:360] = [0, 255, 0]  # Green rectangle
            
            ros_image = self.numpy_to_ros_image(color_image)
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            self.publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error capturing frame: {str(e)}')
            
    def destroy_node(self):
        if hasattr(self, 'pipeline') and hasattr(self, 'has_realsense') and self.has_realsense:
            try:
                self.pipeline.stop()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = RealSenseCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()