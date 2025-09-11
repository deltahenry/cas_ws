#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class ShowImageNode(Node):
    def __init__(self):
        super().__init__('show_image_node')
        
        self.golden_image = None
        self.current_image = None
        self.window_name = "Vision Compensation - Golden (Top) / Current (Bottom)"
        
        # Subscribers
        self.golden_subscriber = self.create_subscription(
            Image,
            '/image_golden_with_corner',
            self.golden_image_callback,
            10
        )
        
        self.current_subscriber = self.create_subscription(
            Image,
            '/image_now_with_corner',
            self.current_image_callback,
            10
        )
        
        self.get_logger().info('Show image node initialized')
        
    def ros_image_to_numpy(self, ros_image):
        """Convert ROS Image message to numpy array"""
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
            # Convert grayscale to BGR for consistent display
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        else:
            cv_image = np_array.reshape((height, width, channels))
        
        return cv_image
    
    def golden_image_callback(self, msg: Image):
        """Store golden image with corner annotations"""
        try:
            self.golden_image = self.ros_image_to_numpy(msg)
            self.update_display()
        except Exception as e:
            self.get_logger().error(f'Error converting golden image: {str(e)}')
    
    def current_image_callback(self, msg: Image):
        """Store current image with corner annotations"""
        try:
            self.current_image = self.ros_image_to_numpy(msg)
            self.update_display()
        except Exception as e:
            self.get_logger().error(f'Error converting current image: {str(e)}')
    
    def update_display(self):
        """Update the dual image display"""
        if self.golden_image is not None and self.current_image is not None:
            try:
                # Resize images to same width for consistent display
                height_golden, width_golden = self.golden_image.shape[:2]
                height_current, width_current = self.current_image.shape[:2]
                
                # Use the smaller width for consistency
                target_width = min(width_golden, width_current)
                
                # Resize golden image
                aspect_golden = height_golden / width_golden
                target_height_golden = int(target_width * aspect_golden)
                golden_resized = cv2.resize(self.golden_image, (target_width, target_height_golden))
                
                # Resize current image
                aspect_current = height_current / width_current
                target_height_current = int(target_width * aspect_current)
                current_resized = cv2.resize(self.current_image, (target_width, target_height_current))
                
                # Create combined image (top: golden, bottom: current)
                combined_image = np.vstack([golden_resized, current_resized])
                
                # Add text labels
                label_height = 30
                label_image = np.zeros((label_height, target_width, 3), dtype=np.uint8)
                cv2.putText(label_image, "Golden Sample", (10, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                label_image2 = np.zeros((label_height, target_width, 3), dtype=np.uint8)
                cv2.putText(label_image2, "Current Image", (10, 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Combine with labels
                final_image = np.vstack([label_image, golden_resized, label_image2, current_resized])
                
                # Display the combined image
                cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
                cv2.imshow(self.window_name, final_image)
                cv2.waitKey(1)  # Non-blocking wait
                
            except Exception as e:
                self.get_logger().error(f'Error updating display: {str(e)}')
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ShowImageNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()