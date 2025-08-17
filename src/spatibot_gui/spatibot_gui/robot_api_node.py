#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import yaml
import os
import threading
import time
import subprocess
import cv2
import numpy as np
import base64
from datetime import datetime
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, asdict
from pathlib import Path
from cv_bridge import CvBridge

# ROS 2 imports
from geometry_msgs.msg import PoseStamped, Twist, Pose
from sensor_msgs.msg import BatteryState, Image
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

# Service imports
from spatibot_interfaces.srv import ExecuteCommand

# TurtleBot4 Navigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult

# LLM imports
import google.generativeai as genai


@dataclass
class Waypoint:
    """Represents a waypoint with label and pose"""
    label: str
    x: float
    y: float
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = 'map'

    def to_pose_stamped(self) -> PoseStamped:
        """Convert to ROS PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.x = self.qx
        pose.pose.orientation.y = self.qy
        pose.pose.orientation.z = self.qz
        pose.pose.orientation.w = self.qw
        return pose

    @classmethod
    def from_pose_stamped(cls, pose: PoseStamped, label: str) -> 'Waypoint':
        """Create waypoint from ROS PoseStamped message"""
        return cls(
            label=label,
            x=pose.pose.position.x,
            y=pose.pose.position.y,
            z=pose.pose.position.z,
            qx=pose.pose.orientation.x,
            qy=pose.pose.orientation.y,
            qz=pose.pose.orientation.z,
            qw=pose.pose.orientation.w,
            frame_id=pose.header.frame_id
        )


class RobotAPI:
    """Core robot API providing all robot control functionality"""
    
    def __init__(self, node: Node):
        self.node = node
        self.navigator = TurtleBot4Navigator()
        
        # State management
        self.waypoints: Dict[str, Waypoint] = {}
        self.current_map: Optional[str] = None
        self.battery_state: Optional[BatteryState] = None
        self.last_image: Optional[Image] = None
        
        # Camera management
        self.cv_bridge = CvBridge()
        self.camera_topic = '/oakd/rgb/preview/image_raw'  # Default camera topic
        self.camera_monitoring_enabled = True  # Flag to control camera monitoring
        
        # Process management for localization/navigation
        self.localization_process = None
        self.navigation_process = None
        self.slam_process = None
        self.oakd_process = None
        
        # Navigation state
        self.navigation_active = False
        self.localization_active = False
        
        # TF2 for pose transforms with longer cache duration
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, node)
        
        # Publishers
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers with appropriate QoS
        # Battery uses best effort QoS (to match publisher)
        battery_qos = QoSProfile(depth=10)
        battery_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        node.create_subscription(BatteryState, '/battery_state', self._battery_callback, battery_qos)
        
        # Simple BEST_EFFORT QoS for camera subscription
        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.node.get_logger().info(f"🔗 Creating image subscription to topic: {self.camera_topic}")
        self.image_sub = node.create_subscription(Image, self.camera_topic, self._image_callback, image_qos)
        self.node.get_logger().info(f"📷 Image subscription created with BEST_EFFORT QoS")
        
        # Track image reception
        self.image_callback_count = 0
        self.last_image_time = None
        
        # Check if camera topic exists at startup
        self.node.create_timer(2.0, self._check_camera_topic_startup)
    

    def _check_camera_topic_startup(self):
        """Check camera topic status at startup"""
        if not self.camera_monitoring_enabled:
            return
            
        try:
            # Check if topic exists
            topic_names_and_types = self.node.get_topic_names_and_types()
            topic_exists = any(name == self.camera_topic for name, _ in topic_names_and_types)
            
            if not topic_exists:
                self.node.get_logger().error(f"❌ Camera topic {self.camera_topic} does not exist!")
                camera_topics = [name for name, _ in topic_names_and_types if 'image' in name or 'camera' in name or 'oakd' in name]
                if camera_topics:
                    self.node.get_logger().info(f"📋 Available camera topics: {camera_topics}")
                else:
                    self.node.get_logger().warn("⚠️  No camera topics found at all!")
            else:
                self.node.get_logger().info(f"✅ Camera topic {self.camera_topic} exists")
                
                # Only check for callback if monitoring is enabled and we have received images before
                if self.camera_monitoring_enabled and self.image_callback_count > 0:
                    # Check if we're still receiving callbacks
                    if self.last_image_time and (time.time() - self.last_image_time) > 2.0:
                        self.node.get_logger().error(f"🚨 Camera callback STOPPED! Count stuck at {self.image_callback_count}")
                        self.node.get_logger().info("🔄 Attempting to restart camera subscription...")
                        self._restart_camera_subscription()
        except Exception as e:
            self.node.get_logger().error(f"Error checking camera topic: {e}")
    
    def _check_topic_publishing(self):
        """Check if the camera topic is actually publishing data"""
        try:
            import subprocess
            
            # Use ros2 topic hz to check if topic is publishing
            self.node.get_logger().info(f"🔍 Checking if {self.camera_topic} is publishing...")
            
            # Run for 2 seconds to check publishing rate
            result = subprocess.run(
                ['timeout', '2', 'ros2', 'topic', 'hz', self.camera_topic],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0 and result.stdout.strip():
                self.node.get_logger().info(f"📊 Camera topic publishing rate: {result.stdout.strip()}")
                self.node.get_logger().warn("🤔 Topic is publishing but callback not triggered - possible QoS mismatch")
            else:
                self.node.get_logger().error(f"❌ Camera topic is NOT publishing data! Camera driver may be down.")
                self.node.get_logger().info("💡 Check if camera driver/node is running")
                
        except Exception as e:
            self.node.get_logger().error(f"Error checking topic publishing: {e}")
    
    def _restart_camera_subscription(self):
        """Restart the camera subscription with BEST_EFFORT QoS"""
        try:
            # Destroy old subscription
            if hasattr(self, 'image_sub') and self.image_sub is not None:
                self.node.destroy_subscription(self.image_sub)
                self.node.get_logger().info("🗑️  Destroyed old camera subscription")
            
            # Create new subscription with BEST_EFFORT QoS
            image_qos = QoSProfile(depth=1)
            image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            
            self.node.get_logger().info(f"🔄 Recreating camera subscription to {self.camera_topic}")
            self.image_sub = self.node.create_subscription(
                Image, 
                self.camera_topic, 
                self._image_callback, 
                image_qos
            )
            
            # Reset tracking
            self._last_image_count = self.image_callback_count
            self.node.get_logger().info("✅ Camera subscription restarted with BEST_EFFORT QoS")
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Failed to restart camera subscription: {e}")
    
    def _battery_callback(self, msg: BatteryState):
        self.battery_state = msg
        self.node.get_logger().debug(f"Battery callback: {msg.percentage:.2%} charge")
    
    def _image_callback(self, msg: Image):
        self.last_image = msg
        self.image_callback_count += 1
        self.last_image_time = time.time()
        
        # Convert ROS timestamp to readable format
        ros_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        timestamp_str = datetime.fromtimestamp(ros_time_sec).strftime("%H:%M:%S.%f")[:-3]
        
        # Log more frequently for the first few images, then less frequently
        if self.image_callback_count <= 5 or self.image_callback_count % 10 == 0:
            self.node.get_logger().info(f"📸 Received image #{self.image_callback_count}: {msg.width}x{msg.height}, encoding: {msg.encoding}, timestamp: {timestamp_str}")
        else:
            self.node.get_logger().debug(f"📸 Received image #{self.image_callback_count}: {msg.width}x{msg.height}, encoding: {msg.encoding}, timestamp: {timestamp_str}")

    # === Core API Functions ===
    
    def get_api_functions(self) -> List[Dict[str, Any]]:
        """Get list of all available API functions with auto-generated descriptions"""
        functions = []
        
        # Get all methods that don't start with underscore
        for attr_name in dir(self):
            if not attr_name.startswith('_') and attr_name != 'get_api_functions':
                attr = getattr(self, attr_name)
                if callable(attr) and hasattr(attr, '__doc__') and attr.__doc__:
                    func_info = self._generate_function_info(attr_name, attr)
                    if func_info:
                        functions.append(func_info)
        
        return functions
    
    def _generate_function_info(self, name: str, func: Callable) -> Optional[Dict[str, Any]]:
        """Auto-generate function information from docstring and signature"""
        import inspect
        
        if not func.__doc__:
            return None
            
        # Parse docstring for description and parameters
        lines = func.__doc__.strip().split('\n')
        description = lines[0].strip()
        
        # Get function signature
        sig = inspect.signature(func)
        args = {}
        
        for param_name, param in sig.parameters.items():
            if param_name == 'self':
                continue
                
            param_info = {
                "type": "string",  # Default type
                "description": f"Parameter {param_name}"
            }
            
            # Try to infer type from annotation
            if param.annotation != inspect.Parameter.empty:
                if param.annotation == str:
                    param_info["type"] = "string"
                elif param.annotation == float:
                    param_info["type"] = "number"
                elif param.annotation == int:
                    param_info["type"] = "integer"
                elif param.annotation == bool:
                    param_info["type"] = "boolean"
            
            # Set default value if available
            if param.default != inspect.Parameter.empty:
                param_info["default"] = param.default
                
            args[param_name] = param_info
        
        return {
            "function": name,
            "description": description,
            "args": args
        }
    
    def handle_api_call(self, function_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Handle API function call"""
        try:
            if not hasattr(self, function_name):
                return {"success": False, "error": f"Function '{function_name}' not found"}
            
            func = getattr(self, function_name)
            if not callable(func) or function_name.startswith('_'):
                return {"success": False, "error": f"Function '{function_name}' not callable"}
            
            result = func(**args)
            return {"success": True, "result": result}
            
        except Exception as e:
            return {"success": False, "error": str(e)}

    # === Robot Pose ===
    
    def get_current_robot_pose(self, frame_id: str = 'map') -> Optional[PoseStamped]:
        """Get current robot pose in specified frame"""
        try:
            # Wait for tf2 buffer to receive some transforms
            self._wait_for_transforms()
            
            # Log available frames for debugging
            frames = self.tf_buffer.all_frames_as_string()
            self.node.get_logger().debug(f"Available frames: {frames}")
            
            # Use latest available transform (Time(0) means latest)
            # This avoids timing issues with exact current time
            transform = self.tf_buffer.lookup_transform(
                frame_id, 'base_link', 
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=3.0))
            
            # Create pose at robot's current position
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Log available frames for debugging on error
            frames = self.tf_buffer.all_frames_as_string()
            self.node.get_logger().error(f"Failed to get robot pose in frame '{frame_id}': {e}")
            self.node.get_logger().error(f"Available frames: {frames}")
            return None
        except Exception as e:
            self.node.get_logger().error(f"Unexpected error getting robot pose: {e}")
            return None
    
    def _wait_for_transforms(self, max_wait_time: float = 2.0):
        """Wait for tf2 buffer to receive transforms"""
        start_time = time.time()
        checked_topics = False
        
        while time.time() - start_time < max_wait_time:
            # Check if we have any frames yet
            frames = self.tf_buffer.all_frames_as_string()
            if frames.strip():  # If we have at least some frames
                self.node.get_logger().debug(f"tf2 buffer available ({time.time() - start_time:.1f}s)")
                return
            
            # Check tf topics only once for debugging
            if not checked_topics:
                self._debug_tf_topics()
                checked_topics = True
            
            # Spin manually to help process transforms
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.node.get_logger().warn(f"tf2 buffer still empty after {max_wait_time}s wait")
    
    def _debug_tf_topics(self):
        """Debug tf topic availability and QoS"""
        try:
            import subprocess
            
            # Check if tf topics exist
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                tf_topics = [topic for topic in topics if 'tf' in topic]
                self.node.get_logger().info(f"Available tf topics: {tf_topics}")
                
                # Check tf topic info
                for topic in ['/tf', '/tf_static']:
                    if topic in topics:
                        info_result = subprocess.run(
                            ['ros2', 'topic', 'info', topic],
                            capture_output=True,
                            text=True,
                            timeout=5
                        )
                        if info_result.returncode == 0:
                            self.node.get_logger().debug(f"Topic {topic} info: {info_result.stdout}")
                        
            else:
                self.node.get_logger().warn("Could not list ROS topics for tf debugging")
                
        except Exception as e:
            self.node.get_logger().debug(f"tf topic debugging failed: {e}")

    # === Waypoint Management ===
    
    def add_waypoint(self, label: str, pose: Optional[PoseStamped] = None) -> str:
        """Add waypoint either from given pose or current robot position"""
        if pose is None:
            pose = self.get_current_robot_pose('map')
            if pose is None:
                return "Robot pose not available"
        
        # Ensure pose is in map frame for consistency
        if pose.header.frame_id != 'map':
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', pose.header.frame_id,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                pose = do_transform_pose(pose, transform)
            except Exception as e:
                return f"Failed to transform pose to map frame: {e}"
        
        waypoint = Waypoint.from_pose_stamped(pose, label)
        waypoint.frame_id = 'map'  # Ensure map frame
        self.waypoints[label] = waypoint
        return f"Added waypoint '{label}' at ({waypoint.x:.2f}, {waypoint.y:.2f})"
    
    def remove_waypoint(self, label: str) -> str:
        """Remove a waypoint by label"""
        if label in self.waypoints:
            del self.waypoints[label]
            return f"Removed waypoint '{label}'"
        return f"Waypoint '{label}' not found"
    
    def list_waypoints(self) -> List[str]:
        """List all available waypoints"""
        return list(self.waypoints.keys())
    
    def get_waypoint_info(self, label: str) -> Optional[Dict[str, Any]]:
        """Get detailed information about a waypoint"""
        if label in self.waypoints:
            return asdict(self.waypoints[label])
        return None
    
    def save_waypoints(self, filename: str) -> str:
        """Save waypoints to YAML file"""
        try:
            data = {label: asdict(wp) for label, wp in self.waypoints.items()}
            with open(filename, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            return f"Saved {len(self.waypoints)} waypoints to {filename}"
        except Exception as e:
            return f"Failed to save waypoints: {e}"
    
    def load_waypoints(self, filename: str) -> str:
        """Load waypoints from YAML file"""
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
            
            loaded_count = 0
            for label, wp_data in data.items():
                # Ensure frame_id is always set to 'map' for consistency
                wp_data['frame_id'] = 'map'
                self.waypoints[label] = Waypoint(**wp_data)
                loaded_count += 1
            
            return f"Loaded {loaded_count} waypoints from {filename} (all frames set to 'map')"
        except Exception as e:
            return f"Failed to load waypoints: {e}"



    # === Navigation ===
    
    def go_to_pose(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map') -> str:
        """Navigate to a specific pose with yaw orientation in degrees"""
        try:
            # Use TurtleBot4Navigator's getPoseStamped method
            target_pose = self.navigator.getPoseStamped([x, y], yaw)
            
            # Set the frame_id
            target_pose.header.frame_id = frame_id
            
            # Navigate to the pose
            self.navigator.waitUntilNav2Active()
            self.navigator.goToPose(target_pose)
            
            return f"Started navigation to pose ({x:.2f}, {y:.2f}) with yaw {yaw:.1f}° in {frame_id} frame"
            
        except Exception as e:
            return f"Navigation failed: {e}"
    
    def go_forward(self, distance: float = 1.0) -> str:
        """Move forward by specified distance in meters"""
        try:
            # Create a pose in base_link frame for relative movement
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = distance  # Forward movement
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            
            self.navigator.waitUntilNav2Active()
            self.navigator.goToPose(pose)
            
            return f"Started moving forward {distance} meters"
        except Exception as e:
            return f"Forward movement failed: {e}"
    
    def go_backward(self, distance: float = 1.0) -> str:
        """Move backward by specified distance in meters"""
        try:
            self.navigator.waitUntilNav2Active()
            self.navigator.backup(backup_dist=distance)
            return f"Started moving backward {distance} meters"
        except Exception as e:
            return f"Backward movement failed: {e}"
    
    def spin(self, angle_degrees: float) -> str:
        """Spin by specified angle in degrees (positive = counterclockwise/left, negative = clockwise/right)"""
        try:
            import math
            angle_radians = math.radians(angle_degrees)
            
            self.navigator.waitUntilNav2Active()
            self.navigator.spin(spin_dist=angle_radians)
            
            direction = "counterclockwise" if angle_degrees > 0 else "clockwise"
            return f"Started spinning {abs(angle_degrees):.1f} degrees {direction}"
        except Exception as e:
            return f"Spin failed: {e}"
    

    
    def move_to_waypoint(self, label: str) -> str:
        """Navigate to a specific waypoint"""
        if label not in self.waypoints:
            return f"Waypoint '{label}' not found"
        
        try:
            waypoint = self.waypoints[label]
            pose = waypoint.to_pose_stamped()
            
            self.navigator.waitUntilNav2Active()
            self.navigator.goToPose(pose)
            
            return f"Started navigation to waypoint '{label}'"
        except Exception as e:
            return f"Navigation failed: {e}"
    
    def cancel_navigation(self) -> str:
        """Cancel current navigation"""
        try:
            self.navigator.cancelTask()
            return "Navigation cancelled"
        except Exception as e:
            return f"Failed to cancel navigation: {e}"
    
    def get_navigation_status(self) -> Dict[str, Any]:
        """Get current navigation status"""
        try:
            is_complete = self.navigator.isTaskComplete()
            if is_complete:
                result = self.navigator.getResult()
                return {
                    "active": False,
                    "complete": True,
                    "result": str(result)
                }
            else:
                feedback = self.navigator.getFeedback()
                return {
                    "active": True,
                    "complete": False,
                    "feedback": str(feedback) if feedback else "No feedback"
                }
        except Exception as e:
            return {"active": False, "complete": False, "error": str(e)}

    # === Robot Control ===
    
    def dock(self) -> str:
        """Dock the robot at charging station"""
        try:
            # Docking doesn't require full Nav2 stack - uses IR sensors
            self.navigator.dock()
            return "Docking initiated"
        except Exception as e:
            return f"Docking failed: {e}"
    
    def undock(self) -> str:
        """Undock the robot from charging station"""
        try:
            # Undocking doesn't require full Nav2 stack
            self.navigator.undock()
            return "Undocking initiated"
        except Exception as e:
            return f"Undocking failed: {e}"
    
    def set_velocity(self, linear: float = 0.0, angular: float = 0.0, timeout: Optional[float] = None) -> str:
        """Set robot velocity (linear m/s, angular rad/s) with optional timeout in seconds"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        
        if timeout is not None:
            # Continuously publish velocity commands until timeout
            def publish_until_timeout():
                start_time = time.time()
                publish_rate = 0.1  # Publish every 100ms to maintain velocity
                
                while time.time() - start_time < timeout:
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(publish_rate)
                
                # Publish stop command when timeout reached
                stop_twist = Twist()  # All zeros
                self.cmd_vel_pub.publish(stop_twist)
            
            # Run publisher in background thread
            publisher_thread = threading.Thread(target=publish_until_timeout)
            publisher_thread.daemon = True
            publisher_thread.start()
            
            return f"Set velocity: linear={linear:.2f} m/s, angular={angular:.2f} rad/s for {timeout:.1f} seconds"
        else:
            return f"Set velocity: linear={linear:.2f} m/s, angular={angular:.2f} rad/s"

    # === Status and Sensors ===
    
    def get_battery_status(self) -> Dict[str, Any]:
        """Get current battery status"""
        # Wait a bit for battery data if not available yet
        if not self.battery_state:
            import time
            self.node.get_logger().info("Waiting for battery data...")
            
            # Wait up to 2 seconds for first battery message
            for i in range(20):  # 20 * 0.1s = 2s
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if self.battery_state:
                    break
                time.sleep(0.1)
        
        if not self.battery_state:
            return {"available": False, "message": "Battery data not available after waiting"}
        
        return {
            "available": True,
            "percentage": self.battery_state.percentage * 100 if self.battery_state.percentage else None,
            "voltage": self.battery_state.voltage,
            "current": self.battery_state.current,
            "charge": self.battery_state.charge,
            "capacity": self.battery_state.capacity,
            "power_supply_status": self.battery_state.power_supply_status
        }
    
    def get_robot_pose(self, frame_id: str = 'map') -> Optional[Dict[str, Any]]:
        """Get current robot pose in specified frame"""
        pose = self.get_current_robot_pose(frame_id)
        if not pose:
            return None
        
        return {
            "frame_id": pose.header.frame_id,
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
            "z": pose.pose.position.z,
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w
            }
        }
    
    def get_dock_status(self) -> str:
        """Get current docking status"""
        try:
            docked = self.navigator.getDockedStatus()
            return "DOCKED" if docked else "NOT_DOCKED"
        except Exception as e:
            return f"UNKNOWN: {e}"
    
    def get_topic_info(self) -> Dict[str, Any]:
        """Get information about available topics"""
        topic_names_and_types = self.node.get_topic_names_and_types()
        
        relevant_topics = {}
        for name, types in topic_names_and_types:
            if any(keyword in name for keyword in ['/battery_state', '/cmd_vel', '/odom', '/scan', '/oakd', '/image']):
                relevant_topics[name] = types
        
        return relevant_topics
    
    def get_camera_topic_status(self) -> Dict[str, Any]:
        """Get detailed status of the camera topic"""
        try:
            # Check if topic exists
            topic_names_and_types = self.node.get_topic_names_and_types()
            topic_exists = any(name == self.camera_topic for name, _ in topic_names_and_types)
            
            status = {
                "topic_exists": topic_exists,
                "subscription_active": hasattr(self, 'image_sub'),
                "images_received": self.image_callback_count,
                "last_image_available": self.last_image is not None,
                "last_image_time": self.last_image_time
            }
            
            if not topic_exists:
                status.update({
                    "message": f"Camera topic {self.camera_topic} does not exist",
                    "available_camera_topics": [name for name, _ in topic_names_and_types if 'image' in name or 'camera' in name or 'oakd' in name]
                })
                return status
            
            # Get topic info using ros2 topic info command
            import subprocess
            try:
                result = subprocess.run(
                    ['ros2', 'topic', 'info', self.camera_topic],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                
                if result.returncode == 0:
                    topic_info = result.stdout.strip()
                    status.update({
                        "topic_info": topic_info,
                        "message": f"Camera topic {self.camera_topic} is available"
                    })
                else:
                    status.update({
                        "message": f"Could not get detailed info for {self.camera_topic}",
                        "error": result.stderr
                    })
            except Exception as e:
                status.update({
                    "message": f"Error checking topic info: {e}"
                })
                
            return status
                
        except Exception as e:
            return {
                "error": f"Error checking camera topic status: {e}"
            }
    
    def diagnose_camera_issue(self) -> Dict[str, Any]:
        """Comprehensive camera diagnostics"""
        try:
            # Check if topic exists
            topic_names_and_types = self.node.get_topic_names_and_types()
            topic_exists = any(name == self.camera_topic for name, _ in topic_names_and_types)
            
            diagnostics = {
                "camera_topic": self.camera_topic,
                "topic_exists": topic_exists,
                "subscription_created": hasattr(self, 'image_sub'),
                "images_received": self.image_callback_count,
                "last_image_available": self.last_image is not None,
                "node_spinning": True  # If this function is called, the node is spinning
            }
            
            if not topic_exists:
                # Find alternative camera topics
                all_topics = [name for name, _ in topic_names_and_types]
                camera_topics = [name for name in all_topics if 'image' in name or 'camera' in name or 'oakd' in name]
                
                diagnostics.update({
                    "issue": "Camera topic does not exist",
                    "available_camera_topics": camera_topics,
                    "all_topics_count": len(all_topics),
                    "suggestion": "Check if camera driver is running or try a different camera topic"
                })
                
            elif self.image_callback_count == 0:
                diagnostics.update({
                    "issue": "Camera topic exists but no images received",
                    "possible_causes": [
                        "QoS mismatch between publisher and subscriber",
                        "Camera driver not publishing",
                        "Network connectivity issues (if using remote camera)",
                        "Camera hardware issues"
                    ],
                    "suggestion": "Check camera driver status and QoS settings"
                })
            else:
                diagnostics.update({
                    "status": "Camera working correctly",
                    "images_per_second": self.image_callback_count / max(1, time.time() - (self.last_image_time or time.time()))
                })
                
            return diagnostics
            
        except Exception as e:
                        return {
                "error": f"Error diagnosing camera issue: {e}"
            }
    
    def restart_camera_subscription(self) -> Dict[str, Any]:
        """Manually restart the camera subscription"""
        try:
            self._restart_camera_subscription()
            return {
                "success": True,
                "message": "Camera subscription restarted successfully"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Failed to restart camera subscription: {e}"
            }
    
    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive robot status"""
        return {
            "pose": self.get_robot_pose(),
            "battery": self.get_battery_status(),
            "dock_status": self.get_dock_status(),
            "navigation": self.get_navigation_status(),
            "waypoints_count": len(self.waypoints),
            "localization_active": self.localization_active,
            "navigation_active": self.navigation_active,
            "topics": self.get_topic_info(),
            "camera_topic": self.camera_topic,
            "last_image_available": self.last_image is not None,
            "camera_topic_status": self.get_camera_topic_status(),
            "current_camera_qos": "BEST_EFFORT",
            "images_received": self.image_callback_count
        }

    # === Camera Functions ===
    
    def get_current_image(self) -> Dict[str, Any]:
        """Get the current camera image as base64 encoded string"""
        try:
            if self.last_image is None:
                return {
                    "success": False,
                    "message": "No image available from camera - check if camera topic is publishing",
                    "error": "no_image"
                }
            
            # Use the ROS image timestamp instead of generating a fresh one
            ros_timestamp = self.last_image.header.stamp
            # Convert ROS timestamp to ISO format
            ros_time_sec = ros_timestamp.sec + ros_timestamp.nanosec / 1e9
            image_timestamp = datetime.fromtimestamp(ros_time_sec).isoformat()
            
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.last_image, "bgr8")
            
            # Encode image to JPEG
            success, encoded_image = cv2.imencode('.jpg', cv_image)
            
            if not success:
                return {
                    "success": False,
                    "message": "Failed to encode image",
                    "error": "encode_failed"
                }
            
            # Convert to base64
            image_base64 = base64.b64encode(encoded_image.tobytes()).decode('utf-8')
            
            return {
                "success": True,
                "message": "Current image retrieved",
                "image_base64": image_base64,
                "width": self.last_image.width,
                "height": self.last_image.height,
                "encoding": self.last_image.encoding,
                "timestamp": image_timestamp,
                "ros_timestamp_sec": ros_timestamp.sec,
                "ros_timestamp_nanosec": ros_timestamp.nanosec
            }
            
        except Exception as e:
            self.node.get_logger().error(f"Error getting current image: {e}")
            return {
                "success": False,
                "message": f"Failed to get current image: {e}",
                "error": str(e)
            }

    # === System Management ===
    
    def load_map(self, map_file: str) -> str:
        """Load a map file for navigation"""
        if not os.path.exists(map_file):
            return f"Map file not found: {map_file}"
        
        self.current_map = map_file
        return f"Map loaded: {map_file}"
    
    def start_localization(self, map_file: str = None) -> str:
        """Start localization with optional map file"""
        # Stop existing localization first to avoid duplicates
        if self.localization_process is not None:
            self.stop_localization()
        
        if map_file:
            self.load_map(map_file)
        
        if not self.current_map:
            return "No map loaded for localization"
        
        try:
            cmd = [
                'ros2', 'launch', 'turtlebot4_navigation', 'localization.launch.py',
                f'map:={self.current_map}'
            ]
            # Start new process group to properly manage child processes
            self.localization_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group
            )
            self.localization_active = True
            return f"Localization started with map: {self.current_map}"
        except Exception as e:
            return f"Failed to start localization: {e}"
    
    def stop_localization(self) -> str:
        """Stop localization"""
        if self.localization_process is not None:
            try:
                import signal
                # Kill entire process group to catch all child processes
                os.killpg(os.getpgid(self.localization_process.pid), signal.SIGTERM)
                try:
                    self.localization_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if graceful termination fails
                    os.killpg(os.getpgid(self.localization_process.pid), signal.SIGKILL)
                    self.localization_process.wait()
                self.localization_process = None
                self.localization_active = False
                return "Localization stopped"
            except Exception as e:
                self.localization_process = None
                self.localization_active = False
                return f"Failed to stop localization cleanly: {e}"
        else:
            return "Localization is not running"
    
    def start_navigation(self) -> str:
        """Start navigation stack"""
        # Stop existing navigation first to avoid duplicates
        if self.navigation_process is not None:
            self.stop_navigation()
        
        try:
            cmd = [
                'ros2', 'launch', 'turtlebot4_navigation', 'nav2.launch.py'
            ]
            # Start new process group to properly manage child processes
            self.navigation_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group
            )
            self.navigation_active = True
            return "Navigation started"
        except Exception as e:
            return f"Failed to start navigation: {e}"
    
    def stop_navigation(self) -> str:
        """Stop navigation stack"""
        if self.navigation_process is not None:
            try:
                import signal
                # Kill entire process group to catch all child processes
                os.killpg(os.getpgid(self.navigation_process.pid), signal.SIGTERM)
                try:
                    self.navigation_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if graceful termination fails
                    os.killpg(os.getpgid(self.navigation_process.pid), signal.SIGKILL)
                    self.navigation_process.wait()
                self.navigation_process = None
                self.navigation_active = False
                return "Navigation stopped"
            except Exception as e:
                self.navigation_process = None
                self.navigation_active = False
                return f"Failed to stop navigation cleanly: {e}"
        else:
            return "Navigation is not running"
    
    def start_slam(self) -> str:
        """Start SLAM (Simultaneous Localization and Mapping)"""
        # Stop existing SLAM first to avoid duplicates
        if self.slam_process is not None:
            self.stop_slam()
        
        try:
            cmd = [
                'ros2', 'launch', 'turtlebot4_navigation', 'slam.launch.py'
            ]
            # Start new process group to properly manage child processes
            self.slam_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group
            )
            return "SLAM started"
        except Exception as e:
            return f"Failed to start SLAM: {e}"
    
    def stop_slam(self) -> str:
        """Stop SLAM"""
        if self.slam_process is not None:
            try:
                import signal
                # Kill entire process group to catch all child processes
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGTERM)
                try:
                    self.slam_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Force kill if graceful termination fails
                    os.killpg(os.getpgid(self.slam_process.pid), signal.SIGKILL)
                    self.slam_process.wait()
                self.slam_process = None
                return "SLAM stopped"
            except Exception as e:
                self.slam_process = None
                return f"Failed to stop SLAM cleanly: {e}"
        else:
            return "SLAM is not running"
    
    def save_map(self, map_path: str) -> str:
        """Save current map to specified path"""
        try:
            # Remove .yaml extension for map_saver_cli (it adds .yaml/.pgm automatically)
            if map_path.endswith('.yaml'):
                map_path = map_path[:-5]
            
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path
            ]
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            stdout, stderr = proc.communicate()
            
            if proc.returncode == 0:
                return f"Map saved successfully to {map_path}.yaml"
            else:
                return f"Failed to save map: {stderr}"
        except Exception as e:
            return f"Failed to save map: {e}"


class LLMHandler:
    """Handle LLM interactions using the robot API"""
    
    def __init__(self, robot_api: RobotAPI, node: Node):
        self.robot_api = robot_api
        self.node = node
        self.gemini_api_key = os.getenv('GEMINI_API_KEY')
        
        if self.gemini_api_key:
            genai.configure(api_key=self.gemini_api_key)
            self.model = genai.GenerativeModel('gemini-1.5-flash')
        else:
            self.model = None
            node.get_logger().warn("GEMINI_API_KEY not set - LLM functionality disabled")
    
    def process_query(self, query: str, image_data: bytes = None) -> Dict[str, Any]:
        """Process natural language query and return API command"""
        if not self.model:
            return {"error": "LLM not available - GEMINI_API_KEY not set"}
        
        try:
            # Get available functions
            functions = self.robot_api.get_api_functions()
            
            # Create prompt
            prompt = f"""
You are a robot control assistant. Based on the user's request, respond with a JSON function call.

Available functions:
{json.dumps(functions, indent=2)}

User request: {query}

Respond ONLY with a JSON object in this format:
{{"function": "function_name", "args": {{"param": "value"}}}}

If no args are needed, use empty object: {{"function": "function_name", "args": {{}}}}
"""
            
            # Prepare content for API call
            content = [prompt]
            if image_data:
                content.append({
                    "mime_type": "image/jpeg",
                    "data": image_data
                })
            
            # Generate response
            response = self.model.generate_content(content)
            answer = response.text if hasattr(response, 'text') else str(response)
            
            # Parse JSON response
            api_command = self._extract_json(answer)
            
            return {
                "success": True,
                "raw_response": answer,
                "api_command": api_command
            }
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _extract_json(self, text: str) -> Optional[Dict[str, Any]]:
        """Extract JSON from LLM response"""
        import re
        
        text = text.strip()
        
        # Remove code block markers
        if text.startswith('```'):
            text = text.lstrip('`')
            if text.startswith('json'):
                text = text[4:]
            text = text.strip()
            if text.endswith('```'):
                text = text[:-3].strip()
        
        # Try to extract JSON object
        match = re.search(r'\{.*\}', text, re.DOTALL)
        if match:
            try:
                result = json.loads(match.group(0))
                # Ensure args field exists
                if 'function' in result and 'args' not in result:
                    result['args'] = {}
                return result
            except json.JSONDecodeError:
                pass
        
        return None
    
    def execute_command(self, api_command: Dict[str, Any]) -> Dict[str, Any]:
        """Execute API command and return result"""
        if not api_command or 'function' not in api_command:
            return {"success": False, "error": "Invalid API command format"}
        
        function_name = api_command['function']
        args = api_command.get('args', {})
        
        return self.robot_api.handle_api_call(function_name, args)


class RobotAPINode(Node):
    """Main ROS 2 node providing robot API and LLM interface"""
    
    def __init__(self):
        super().__init__('robot_api_node')
        
        # Clean up any leftover processes from previous sessions first
        self._cleanup_leftover_processes()
        
        # Declare parameters
        self.declare_parameter('auto_start_localization', False)
        self.declare_parameter('auto_start_navigation', False)
        self.declare_parameter('default_map', '')
        self.declare_parameter('waypoints_file', 'waypoints.yaml')
        
        # Initialize API and LLM handler
        self.robot_api = RobotAPI(self)
        self.llm_handler = LLMHandler(self.robot_api, self)
        
        # Create service server for generic API commands
        self.service_server = self.create_service(
            ExecuteCommand,
            '/spatibot/execute_command',
            self._execute_command_callback
        )
        
        # Auto-start services if configured
        self._auto_start_services()
        
        self.get_logger().info("Robot API Node started")
        self.get_logger().info(f"Available API functions: {len(self.robot_api.get_api_functions())}")
        self.get_logger().info("Service server '/spatibot/execute_command' ready")
    
    def _execute_command_callback(self, request, response):
        """Service callback for executing robot API commands"""
        try:
            # Extract function name and arguments
            function_name = request.function_name
            args = json.loads(request.args_json) if request.args_json else {}

            # Map function names to handler methods
            handlers = {
                'get_status': self.robot_api.get_status,
                'nav_to_pose': self.robot_api.go_to_pose,
                'nav_through_poses': self.robot_api.go_to_pose, # Placeholder for multiple poses
                'nav_waypoints': self.robot_api.move_to_waypoint,
                'nav_forward': self.robot_api.go_forward,
                'nav_backward': self.robot_api.go_backward,
                'nav_rotate': self.robot_api.spin, # Placeholder for angle
                'control_stop': self.robot_api.set_velocity, # Placeholder for linear, angular
                'set_velocity': self.robot_api.set_velocity, # Direct velocity control
                'dock': self.robot_api.dock,
                'undock': self.robot_api.undock,
                'start_mapping': self.robot_api.start_slam,
                'stop_mapping': self.robot_api.stop_slam,
                'save_map': self.robot_api.save_map,
                'start_localization': self.robot_api.start_localization,
                'start_navigation': self.robot_api.start_navigation,
                'start_oakd': self.start_oakd,  # Add new handler
                'stop_oakd': self.stop_oakd,    # Add new handler
                'get_current_image': self.robot_api.get_current_image,  # Add image handler
            }

            # Find the handler for the requested function
            handler = handlers.get(function_name)
            if handler:
                result = handler(**args)
                response.success = True
                response.message = "Operation completed successfully"
                response.data_json = json.dumps(result)
            else:
                response.success = False
                response.message = f"Function '{function_name}' not found or not callable"
                response.data_json = ""
            
            self.get_logger().debug(f"Service response: success={response.success}, message='{response.message}'")
            
        except Exception as e:
            self.get_logger().error(f"Service callback error: {e}")
            response.success = False
            response.message = f"Service error: {e}"
            response.data_json = ""
        
        return response
    
    def destroy_node(self):
        """Clean shutdown with process cleanup"""
        self.get_logger().info("Shutting down Robot API Node...")
        
        # Clean up any running processes
        if hasattr(self, 'oakd_process') and self.oakd_process:
            self.get_logger().info("Cleaning up camera process...")
            result = self.stop_oakd()
            self.get_logger().info(f"Camera cleanup: {result}")
        
        if self.robot_api.localization_process is not None:
            result = self.robot_api.stop_localization()
            self.get_logger().info(f"Localization cleanup: {result}")
        
        if self.robot_api.navigation_process is not None:
            result = self.robot_api.stop_navigation()
            self.get_logger().info(f"Navigation cleanup: {result}")
        
        if self.robot_api.slam_process is not None:
            result = self.robot_api.stop_slam()
            self.get_logger().info(f"SLAM cleanup: {result}")
            
        # Final camera process cleanup
        self._cleanup_camera_processes()
        
        super().destroy_node()
    
    def _cleanup_leftover_processes(self):
        """Clean up any leftover processes from previous sessions"""
        try:
            self.get_logger().info("Cleaning up leftover processes from previous sessions...")
            cleanup_count = 0
            
            # List of process patterns to clean up
            process_patterns = [
                'localization.launch.py',
                'nav2.launch.py', 
                'slam.launch.py',
                'lifecycle_manager_localization',
                'lifecycle_manager_navigation',
                'lifecycle_manager_slam',
                'bt_navigator',  # Behavior tree navigator
                'controller_server',  # Navigation controller
                'behavior_server',  # Navigation behavior server
                'smoother_server',  # Path smoother
                'waypoint_follower',  # Waypoint follower
                'local_costmap',  # Local costmap
                'global_costmap',  # Global costmap
                'map_server',  # Map server
                'velocity_smoother',  # Velocity smoother
                'turtlebot4_navigation',  # Catch any turtlebot4_navigation processes
                'oakd_with_turtlebot.launch.py',  # OAK-D camera launch
                'depthai_bridge',  # DepthAI ROS bridge
                'camera_node',  # Generic camera node
                'rgb_publisher',  # RGB image publisher
                'feature_tracker',  # Feature tracking node
                'stereo_node',  # Stereo processing node
                'spatibot_oakd'  # Our OAK-D package nodes
            ]
            
            for pattern in process_patterns:
                try:
                    # Use pkill to find and kill processes matching the pattern
                    result = subprocess.run(
                        ['pkill', '-f', pattern],
                        capture_output=True,
                        text=True,
                        timeout=5
                    )
                    if result.returncode == 0:
                        cleanup_count += 1
                        self.get_logger().info(f"Cleaned up processes matching: {pattern}")
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"Timeout cleaning up pattern: {pattern}")
                except subprocess.CalledProcessError:
                    # pkill returns 1 if no processes found - this is normal
                    pass
                except Exception as e:
                    self.get_logger().warn(f"Error cleaning pattern {pattern}: {e}")
            
            # Also clean up any zombie ROS2 nodes that might be causing name conflicts
            try:
                self.get_logger().info("Checking for conflicting ROS2 nodes...")
                # Check for any existing robot_api_node instances
                result = subprocess.run(
                    ['ros2', 'node', 'list'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                
                if result.returncode == 0:
                    nodes = result.stdout.strip().split('\n')
                    conflicting_nodes = [node for node in nodes if 'robot_api_node' in node or 'robot_cli_node' in node]
                    
                    if conflicting_nodes:
                        self.get_logger().warn(f"Found potentially conflicting nodes: {conflicting_nodes}")
                        self.get_logger().warn("If you see 'nodes share exact name' warnings, previous instances may still be running")
                        
            except Exception as e:
                self.get_logger().debug(f"Could not check ROS2 nodes: {e}")
            
            if cleanup_count > 0:
                self.get_logger().info(f"Cleaned up {cleanup_count} types of leftover processes")
                # Give processes time to fully terminate
                time.sleep(2)
            else:
                self.get_logger().info("No leftover processes found to clean up")
                
        except Exception as e:
            self.get_logger().error(f"Error during leftover process cleanup: {e}")
    
    def _auto_start_services(self):
        """Auto-start localization and navigation if configured"""
        auto_localization = self.get_parameter('auto_start_localization').value
        auto_navigation = self.get_parameter('auto_start_navigation').value
        default_map = self.get_parameter('default_map').value
        waypoints_file = self.get_parameter('waypoints_file').value
        
        if auto_localization and default_map:
            result = self.robot_api.start_localization(default_map)
            self.get_logger().info(f"Auto-start localization: {result}")
        
        if auto_navigation:
            result = self.robot_api.start_navigation()
            self.get_logger().info(f"Auto-start navigation: {result}")
        
        # Load waypoints if file exists
        if waypoints_file and os.path.exists(waypoints_file):
            result = self.robot_api.load_waypoints(waypoints_file)
            self.get_logger().info(f"Auto-load waypoints: {result}")

    def start_oakd(self, args=None):
        """Start OAK-D camera and processing node"""
        try:
            # Launch both the turtlebot4 oakd.launch.py and our processing node
            cmd = ['ros2', 'launch', 'spatibot_oakd', 'oakd_with_turtlebot.launch.py']
            self.oakd_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new process group
            )
            # Re-enable camera monitoring
            self.robot_api.camera_monitoring_enabled = True
            return {'success': True, 'message': 'OAK-D camera started'}
        except Exception as e:
            return {'success': False, 'message': f'Failed to start OAK-D camera: {str(e)}'}

    def stop_oakd(self, args=None):
        """Stop OAK-D camera and processing node"""
        try:
            # Disable camera monitoring before stopping
            self.robot_api.camera_monitoring_enabled = False
            
            if hasattr(self, 'oakd_process') and self.oakd_process:
                self.get_logger().info("Stopping OAK-D camera...")
                import signal
                import os
                import psutil
                
                # First try graceful shutdown
                os.killpg(os.getpgid(self.oakd_process.pid), signal.SIGTERM)
                try:
                    self.oakd_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("Camera process did not stop gracefully, force killing...")
                    
                    # Kill the entire process group
                    os.killpg(os.getpgid(self.oakd_process.pid), signal.SIGKILL)
                    self.oakd_process.wait()
                    
                    # Find and kill any remaining camera-related processes
                    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                        try:
                            # Check if process is related to camera
                            if any(pattern in ' '.join(proc.info['cmdline'] or []) 
                                  for pattern in ['oakd', 'depthai', 'camera_node', 'rgb_publisher']):
                                self.get_logger().warn(f"Killing leftover camera process: {proc.info['name']}")
                                os.kill(proc.info['pid'], signal.SIGKILL)
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            continue
                
                self.oakd_process = None
                self.get_logger().info("OAK-D camera stopped successfully")
            else:
                self.get_logger().warn("No camera process found to stop")
                
                # Even if no process is found, try to clean up any leftover camera processes
                self._cleanup_camera_processes()
            
            # Reset camera-related variables
            self.robot_api.last_image = None
            self.robot_api.image_callback_count = 0
            self.robot_api.last_image_time = None
            
            return {'success': True, 'message': 'OAK-D camera stopped'}
        except Exception as e:
            self.get_logger().error(f"Failed to stop OAK-D camera: {e}")
            return {'success': False, 'message': f'Failed to stop OAK-D camera: {str(e)}'}
            
    def _cleanup_camera_processes(self):
        """Clean up any leftover camera processes"""
        try:
            import psutil
            import signal
            
            camera_patterns = [
                'oakd_with_turtlebot.launch.py',
                'depthai_bridge',
                'camera_node',
                'rgb_publisher',
                'feature_tracker',
                'stereo_node',
                'spatibot_oakd'
            ]
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if any(pattern in ' '.join(proc.info['cmdline'] or []) for pattern in camera_patterns):
                        self.get_logger().info(f"Cleaning up camera process: {proc.info['name']}")
                        os.kill(proc.info['pid'], signal.SIGKILL)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                    
        except Exception as e:
            self.get_logger().error(f"Error during camera process cleanup: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotAPINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 