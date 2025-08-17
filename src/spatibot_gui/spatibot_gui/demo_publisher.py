#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header

class DemoPublisher(Node):
    def __init__(self):
        super().__init__('demo_publisher')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create mock data
        self.create_mock_image()
        self.create_mock_scan()
        
        # Setup publishers
        self.setup_publishers()
        
        # Setup timers
        self.create_timer(0.1, self.publish_odometry)  # 10 Hz
        self.create_timer(0.033, self.publish_image)   # 30 Hz
        self.create_timer(0.1, self.publish_scan)      # 10 Hz
        self.create_timer(1.0, self.publish_path)      # 1 Hz
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.get_logger().info("Demo publisher started")
    
    def create_mock_image(self):
        """Create a mock camera image"""
        # Create a simple test pattern
        self.mock_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add some colored rectangles
        cv2.rectangle(self.mock_image, (100, 100), (200, 200), (0, 255, 0), -1)  # Green
        cv2.rectangle(self.mock_image, (300, 100), (400, 200), (255, 0, 0), -1)  # Blue
        cv2.rectangle(self.mock_image, (200, 300), (300, 400), (0, 0, 255), -1)  # Red
        
        # Add text
        cv2.putText(self.mock_image, "Demo Camera Feed", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    def create_mock_scan(self):
        """Create mock laser scan data"""
        self.scan_ranges = []
        for i in range(360):
            angle = np.radians(i)
            # Create a simple room-like environment
            if i < 90 or i > 270:  # Front and back
                range_val = 2.0 + 0.5 * np.sin(angle * 3)
            else:  # Sides
                range_val = 1.5 + 0.3 * np.cos(angle * 2)
            
            # Add some noise
            range_val += np.random.normal(0, 0.05)
            self.scan_ranges.append(max(0.1, range_val))
    
    def setup_publishers(self):
        """Setup ROS 2 publishers"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.image_pub = self.create_publisher(Image, '/oakd/rgb/preview/image_raw', qos_profile)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
    
    def publish_image(self):
        """Publish mock camera image"""
        try:
            # Add timestamp to image
            img_with_time = self.mock_image.copy()
            timestamp = self.get_clock().now().nanoseconds
            cv2.putText(img_with_time, f"Time: {timestamp}", (50, 450), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Convert to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(img_with_time, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")
    
    def publish_odometry(self):
        """Publish mock odometry data"""
        # Update robot position based on velocity
        dt = 0.1  # 10 Hz
        self.robot_x += self.linear_vel * np.cos(self.robot_theta) * dt
        self.robot_y += self.linear_vel * np.sin(self.robot_theta) * dt
        self.robot_theta += self.angular_vel * dt
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Set pose
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (simple 2D)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(self.robot_theta / 2.0)
        odom_msg.pose.pose.orientation.w = np.cos(self.robot_theta / 2.0)
        
        # Set velocity
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom_msg)
    
    def publish_scan(self):
        """Publish mock laser scan data"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "laser_frame"
        
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * np.pi
        scan_msg.angle_increment = 2.0 * np.pi / len(self.scan_ranges)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        scan_msg.ranges = self.scan_ranges
        
        self.scan_pub.publish(scan_msg)
    
    def publish_path(self):
        """Publish mock navigation path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        # Create a simple circular path
        num_points = 20
        radius = 2.0
        center_x = self.robot_x + 3.0
        center_y = self.robot_y + 2.0
        
        for i in range(num_points):
            angle = 2.0 * np.pi * i / num_points
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = center_x + radius * np.cos(angle)
            pose_stamped.pose.position.y = center_y + radius * np.sin(angle)
            pose_stamped.pose.position.z = 0.0
            
            # Set orientation
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = np.sin(angle / 2.0)
            pose_stamped.pose.orientation.w = np.cos(angle / 2.0)
            
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    
    demo_publisher = DemoPublisher()
    
    try:
        rclpy.spin(demo_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        demo_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 