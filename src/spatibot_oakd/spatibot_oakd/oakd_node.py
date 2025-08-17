#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class OAKDNode(Node):
    def __init__(self):
        super().__init__('oakd_node')
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Create QoS profile for better image streaming
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to both raw and compressed images
        self.rgb_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.rgb_callback,
            qos_profile
        )
        
        self.rgb_compressed_sub = self.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed',
            self.rgb_compressed_callback,
            qos_profile
        )
        
        # Create publishers for both formats
        self.rgb_processed_pub = self.create_publisher(
            Image, 
            '/oakd/rgb/processed/image_raw', 
            qos_profile
        )
        
        self.rgb_processed_compressed_pub = self.create_publisher(
            CompressedImage,
            '/oakd/rgb/processed/image_raw/compressed',
            qos_profile
        )
        
        # Counter for debugging
        self.callback_counter = 0
        self.process_counter = 0
        
        # Create timer for status updates
        self.create_timer(5.0, self.print_status)
        
        self.get_logger().info('OAK-D Node initialized - Waiting for images...')

    def print_status(self):
        """Print debug status every 5 seconds"""
        self.get_logger().info(f'Status - Callbacks received: {self.callback_counter}, Images processed: {self.process_counter}')

    def rgb_callback(self, msg):
        """Handle raw image format"""
        self.callback_counter += 1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_and_publish(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {str(e)}')

    def rgb_compressed_callback(self, msg):
        """Handle compressed image format"""
        self.callback_counter += 1
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_and_publish(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {str(e)}')

    def process_and_publish(self, cv_image, header):
        """Process image and publish in both raw and compressed formats"""
        try:
            # Add your image processing here
            # For example: Edge detection
            processed_image = cv2.Canny(cv_image, 100, 200)
            # Convert back to BGR for color display
            processed_image = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)

            # Publish raw format
            raw_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            raw_msg.header = header
            self.rgb_processed_pub.publish(raw_msg)

            # Publish compressed format
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "bgr8; jpeg compressed bgr8"
            # Use high quality JPEG compression
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
            result, encoded = cv2.imencode('.jpg', processed_image, encode_param)
            if result:
                compressed_msg.data = np.array(encoded).tobytes()
                self.rgb_processed_compressed_pub.publish(compressed_msg)

            self.process_counter += 1
            if self.process_counter % 30 == 0:
                self.get_logger().info(f'Successfully processed and published image #{self.process_counter}')

        except Exception as e:
            self.get_logger().error(f'Error in process_and_publish: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OAKDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 