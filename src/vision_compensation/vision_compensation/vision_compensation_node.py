#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
from datetime import datetime
from .vision import compensate_cabinet_with_visualization, compensate_cabinet_test_with_visualization


class VisionCompensationNode(Node):
    def __init__(self):
        super().__init__('vision_compensation_node')
        
        self.image_now = None
        self.show_display = True  # Enable image display
        self.window_name = "Vision Compensation - Current Image"
        
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
        
        # Publishers
        self.compensate_pose_publisher = self.create_publisher(
            Float32MultiArray,
            '/compensate_pose',
            10
        )
        
        self.image_golden_with_corner_publisher = self.create_publisher(
            Image,
            '/image_golden_with_corner',
            10
        )
        
        self.image_now_with_corner_publisher = self.create_publisher(
            Image,
            '/image_now_with_corner',
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
    
    def numpy_to_ros_image(self, cv_image):
        """Convert numpy array to ROS Image message"""
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
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"
        
        return ros_image

    def camera_image_callback(self, msg: Image):
        """Store current image from camera"""
        try:
            self.image_now = self.ros_image_to_numpy(msg)
            
            # Display current image
            if self.show_display and self.image_now is not None:
                self.display_current_image()
                
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
            
    def display_current_image(self):
        """Display current image using OpenCV"""
        if self.image_now is not None:
            try:
                # Create display window
                cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
                
                # Display image
                cv2.imshow(self.window_name, self.image_now)
                cv2.waitKey(1)  # Non-blocking wait
                
            except Exception as e:
                self.get_logger().error(f'Error displaying image: {str(e)}')
            
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
        elif command == "save_image":
            self.handle_save_image()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            
    def handle_start_detect(self):
        """Handle start_detect command"""
        if self.image_now is None:
            self.get_logger().error('No current image available')
            return
            
        golden_path = os.path.expanduser('~/image_sample/2D_golden(x=0,z=112).png')
        
        if not os.path.exists(golden_path):
            self.get_logger().error(f'Golden sample not found: {golden_path}')
            return
            
        try:
            x, z, vis_golden, vis_current = compensate_cabinet_with_visualization(golden_path, self.image_now)
            
            # Apply calibration coefficients
            x_out = x * 0.53
            z_out = z * 0.18
            
            self.publish_compensation(x_out, z_out)
            self.publish_visualization_images(vis_golden, vis_current)
            self.get_logger().info(f'Detect compensation calculated: X={x_out:.2f}, Z={z_out:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in detect compensation calculation: {str(e)}')
            
    def handle_start_detect_0(self):
        """Handle start_detect_0 command"""
        golden_path = os.path.expanduser('~/image_sample/2D_golden(x=0,z=112).png')
        test_path = os.path.expanduser('~/image_sample/2D_golden(x=0,z=112).png')  # Same as golden
        
        if not os.path.exists(golden_path):
            self.get_logger().error(f'Golden sample not found: {golden_path}')
            return
            
        if not os.path.exists(test_path):
            self.get_logger().error(f'Test image not found: {test_path}')
            return
            
        try:
            x, z, vis_golden, vis_current = compensate_cabinet_test_with_visualization(golden_path, test_path)
            
            # Apply calibration coefficients
            x_out = x * 0.53
            z_out = z * 0.18
            
            self.publish_compensation(x_out, z_out)
            self.publish_visualization_images(vis_golden, vis_current)
            self.get_logger().info(f'Detect_0 compensation calculated: X={x_out:.2f}, Z={z_out:.2f}')
            
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
            x, z, vis_golden, vis_current = compensate_cabinet_test_with_visualization(golden_path, test_path)
            
            # Apply calibration coefficients
            x_out = x * 0.53
            z_out = z * 0.18
            
            self.publish_compensation(x_out, z_out)
            self.publish_visualization_images(vis_golden, vis_current)
            self.get_logger().info(f'Test compensation calculated: X={x_out:.2f}, Z={z_out:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in test compensation calculation: {str(e)}')
            
    def handle_save_image(self):
        """Handle save_image command"""
        if self.image_now is None:
            self.get_logger().error('No current image available to save')
            return
            
        try:
            # Generate timestamp filename: YYYY_MMDD_HHMMSS.png
            timestamp = datetime.now().strftime('%Y_%m%d_%H%M%S')
            filename = f"{timestamp}.png"
            
            # Save image to current working directory
            success = cv2.imwrite(filename, self.image_now)
            
            if success:
                self.get_logger().info(f'Image saved successfully: {filename}')
            else:
                self.get_logger().error(f'Failed to save image: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Error saving image: {str(e)}')
            
    def publish_compensation(self, x: float, z: float):
        """Publish compensation values"""
        msg = Float32MultiArray()
        msg.data = [float(x), float(z)]
        
        self.compensate_pose_publisher.publish(msg)
        
    def publish_visualization_images(self, vis_golden, vis_current):
        """Publish visualization images with corner annotations"""
        try:
            if vis_golden is not None:
                golden_ros_image = self.numpy_to_ros_image(vis_golden)
                self.image_golden_with_corner_publisher.publish(golden_ros_image)
                
            if vis_current is not None:
                current_ros_image = self.numpy_to_ros_image(vis_current)
                self.image_now_with_corner_publisher.publish(current_ros_image)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing visualization images: {str(e)}')
        
    def compensate_pose_callback(self, msg: Float32MultiArray):
        """Example callback for receiving compensation pose"""
        if len(msg.data) >= 2:
            X, Z = msg.data[:2]
            self.get_logger().info(f"Received compensation: X={X:.2f}, Z={Z:.2f}")
            
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        if self.show_display:
            cv2.destroyAllWindows()
        super().destroy_node()


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

