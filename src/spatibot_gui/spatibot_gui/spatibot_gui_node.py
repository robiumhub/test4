#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import json
import os
import yaml
from dataclasses import dataclass

# ROS2 message types for display (read-only)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan, BatteryState
from nav_msgs.msg import Path, Odometry

# For image processing
import cv2
from cv_bridge import CvBridge

# For plotting
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# Service interface
from spatibot_interfaces.srv import ExecuteCommand

@dataclass
class Waypoint:
    label: str
    x: float
    y: float
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = 'map'

class CollapsibleSection(ttk.Frame):
    def __init__(self, parent, title="Section", *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.show = tk.BooleanVar(value=False)
        self.title = title
        self.header = ttk.Button(self, text=f"▶ {title}", width=25, command=self.toggle)
        self.header.pack(fill=tk.X)
        self.body = ttk.Frame(self)
        self.body.pack(fill=tk.X, expand=True)
        self.body.pack_forget()
    
    def toggle(self):
        if self.show.get():
            self.body.pack_forget()
            self.show.set(False)
            self.header.config(text=f"▶ {self.title}")
        else:
            self.body.pack(fill=tk.X, expand=True)
            self.show.set(True)
            self.header.config(text=f"▼ {self.title}")

class ServiceBasedGUINode(Node):
    """Service-based GUI that calls robot_api_node for all robot operations"""
    
    def __init__(self):
        self.root = tk.Tk()
        super().__init__('spatibot_gui_service_client')
        
        # Declare and get parameters
        self.declare_parameter('camera_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('window_width', 1200)
        self.declare_parameter('window_height', 800)
        self.declare_parameter('image_max_size', 400)
        self.declare_parameter('max_trajectory_points', 1000)
        self.declare_parameter('enable_camera', False)  # Disabled by default
        
        # Get parameter values
        self.camera_topic = self.get_parameter('camera_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        self.image_max_size = self.get_parameter('image_max_size').value
        self.max_trajectory_points = self.get_parameter('max_trajectory_points').value
        self.enable_camera = self.get_parameter('enable_camera').value
        
        # Initialize CV bridge for image processing
        self.bridge = CvBridge()
        
        # Threading lock for thread safety
        self.lock = threading.Lock()
        
        # Initialize service client for robot API
        self.setup_service_client()
        
        # Initialize GUI
        self.root.title("SpatiBot Control GUI (Service-Based)")
        self.root.geometry(f"{self.window_width}x{self.window_height}")
        
        # Data storage for display
        self.current_image = None
        self.current_odom = None
        self.current_path = None
        self.current_scan = None
        self.current_battery = None
        self.trajectory_data = []
        
        # GUI state
        self.selected_map = None
        self.waypoint_library = {}  # label -> Waypoint
        self.current_route = []     # list of labels
        
        # Setup display subscriptions (read-only)
        self.setup_display_subscriptions()
        
        # Setup GUI
        self.setup_gui()
        
        # Setup timer for GUI updates
        self.create_timer(0.2, self.update_gui)  # 5 Hz GUI updates
        
        self.get_logger().info("Service-Based SpatiBot GUI started")
        
        # Gemini API key for LLM features
        self.gemini_api_key = os.environ.get("GEMINI_API_KEY", "")
        self.gemini_api_command = None

    def setup_service_client(self):
        """Setup service client for robot API communication"""
        self.service_client = self.create_client(ExecuteCommand, '/spatibot/execute_command')
        
        # Wait for service to be available
        self.get_logger().info("Waiting for robot API service...")
        if not self.service_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Robot API service not available! Make sure robot_api_node is running.")
            messagebox.showerror("Service Error", 
                "Robot API service not available!\nMake sure robot_api_node is running.")
            return False
        
        self.get_logger().info("Connected to robot API service")
        return True
    
    def call_robot_service(self, function_name: str, args: dict = None):
        """Call robot API service and return result"""
        try:
            if args is None:
                args = {}
            
            # Create service request
            request = ExecuteCommand.Request()
            request.function_name = function_name
            request.args_json = json.dumps(args)
            
            # Call service
            self.get_logger().debug(f"Service call: {function_name} with args: {request.args_json}")
            future = self.service_client.call_async(request)
            
            # Wait for response with timeout
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None:
                response = future.result()
                
                # Parse complex return data if present
                result_data = None
                if response.data_json:
                    try:
                        result_data = json.loads(response.data_json)
                    except json.JSONDecodeError:
                        self.get_logger().warn(f"Failed to parse data_json: {response.data_json}")
                
                return {
                    "success": response.success,
                    "result": response.message,
                    "data": result_data
                }
            else:
                return {
                    "success": False,
                    "error": "Service call failed or timed out"
                }
                
        except Exception as e:
            self.get_logger().error(f"Service call error: {e}")
            return {
                "success": False,
                "error": f"Service call failed: {e}"
            }
    
    def setup_display_subscriptions(self):
        """Setup ROS 2 subscribers for display data only (read-only)"""
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Conditionally create image subscription
        if self.enable_camera:
            self.image_sub = self.create_subscription(
                Image, self.camera_topic, self.image_callback, qos_profile
            )
            self.get_logger().info(f"Image subscription enabled on {self.camera_topic}")
        else:
            self.image_sub = None
            self.get_logger().info("Image subscription disabled")
        
        # Odometry for trajectory display
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odometry_topic, self.odom_callback, odom_qos
        )
        
        # Path for navigation display
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10
        )
        
        # Laser scan for visualization
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        
        # Battery status for display
        battery_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, battery_qos
        )
        
        self.get_logger().info("Display subscriptions established")

    def setup_gui(self):
        """Setup the GUI layout"""
        
        # Create main frames
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        display_frame = ttk.Frame(self.root)
        display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Control Panel
        self.setup_control_panel(control_frame)
        
        # Display Panel
        self.setup_display_panel(display_frame)

    def setup_control_panel(self, parent):
        """Setup the control panel with all robot control buttons"""
        control_panel = ttk.Frame(parent)
        
        # Basic Controls Section
        basic_controls = CollapsibleSection(control_panel, "Basic Controls")
        status_section = CollapsibleSection(basic_controls.body, "Service Status")
        status_section.pack(fill=tk.X, pady=2)
        self.status_label = ttk.Label(status_section.body, text="Service: Connected ✓")
        self.status_label.pack(fill=tk.X, pady=2)
        ttk.Button(status_section.body, text="Get Robot Status", command=self.get_robot_status).pack(fill=tk.X, pady=2)
        
        # Docking Controls
        dock_section = CollapsibleSection(basic_controls.body, "Docking Controls")
        dock_section.pack(fill=tk.X, pady=2)
        ttk.Button(dock_section.body, text="Dock", command=self.dock_robot).pack(fill=tk.X, pady=2)
        ttk.Button(dock_section.body, text="Undock", command=self.undock_robot).pack(fill=tk.X, pady=2)
        ttk.Button(dock_section.body, text="Get Dock Status", command=self.get_dock_status).pack(fill=tk.X, pady=2)
        
        # Movement Controls
        move_section = CollapsibleSection(basic_controls.body, "Movement Controls")
        move_section.pack(fill=tk.X, pady=2)
        
        # Velocity controls
        ttk.Label(move_section.body, text="Linear Velocity (m/s)").pack()
        self.linear_vel_var = tk.DoubleVar(value=0.0)
        linear_vel_scale = ttk.Scale(move_section.body, from_=-1.0, to=1.0, 
                                   variable=self.linear_vel_var, orient=tk.HORIZONTAL)
        linear_vel_scale.pack(fill=tk.X, pady=2)
        
        ttk.Label(move_section.body, text="Angular Velocity (rad/s)").pack()
        self.angular_vel_var = tk.DoubleVar(value=0.0)
        angular_vel_scale = ttk.Scale(move_section.body, from_=-2.0, to=2.0, 
                                    variable=self.angular_vel_var, orient=tk.HORIZONTAL)
        angular_vel_scale.pack(fill=tk.X, pady=2)
        
        # Direction buttons
        button_frame = ttk.Frame(move_section.body)
        button_frame.pack(pady=5)
        ttk.Button(button_frame, text="Forward", 
                  command=lambda: self.set_velocity(0.5, 0.0)).grid(row=0, column=1, padx=2)
        ttk.Button(button_frame, text="Backward", 
                  command=lambda: self.set_velocity(-0.5, 0.0)).grid(row=2, column=1, padx=2)
        ttk.Button(button_frame, text="Left", 
                  command=lambda: self.set_velocity(0.0, 1.0)).grid(row=1, column=0, padx=2)
        ttk.Button(button_frame, text="Right", 
                  command=lambda: self.set_velocity(0.0, -1.0)).grid(row=1, column=2, padx=2)
        ttk.Button(button_frame, text="Stop", 
                  command=self.stop_robot).grid(row=1, column=1, padx=2)
        
        # Custom velocity button
        vel_frame = ttk.Frame(move_section.body)
        vel_frame.pack(fill=tk.X, pady=2)
        ttk.Button(vel_frame, text="Set Custom Velocity", 
                  command=self.set_custom_velocity).pack(fill=tk.X, pady=2)
        
        # Spin controls
        spin_frame = ttk.Frame(move_section.body)
        spin_frame.pack(fill=tk.X, pady=2)
        ttk.Label(spin_frame, text="Spin Angle (degrees):").pack()
        self.spin_angle_var = tk.DoubleVar(value=90.0)
        spin_entry = ttk.Entry(spin_frame, textvariable=self.spin_angle_var)
        spin_entry.pack(fill=tk.X, pady=2)
        ttk.Button(spin_frame, text="Spin", command=self.spin_robot).pack(fill=tk.X, pady=2)
        
        # Navigation Controls
        nav_section = CollapsibleSection(basic_controls.body, "Navigation")
        nav_section.pack(fill=tk.X, pady=2)
        
        # Go to pose
        pose_frame = ttk.Frame(nav_section.body)
        pose_frame.pack(fill=tk.X, pady=2)
        ttk.Label(pose_frame, text="Go to Pose (x, y, yaw):").pack()
        
        coord_frame = ttk.Frame(pose_frame)
        coord_frame.pack(fill=tk.X, pady=2)
        
        self.pose_x_var = tk.DoubleVar(value=0.0)
        self.pose_y_var = tk.DoubleVar(value=0.0)
        self.pose_yaw_var = tk.DoubleVar(value=0.0)
        
        ttk.Entry(coord_frame, textvariable=self.pose_x_var, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Entry(coord_frame, textvariable=self.pose_y_var, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Entry(coord_frame, textvariable=self.pose_yaw_var, width=8).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(pose_frame, text="Go to Pose", command=self.go_to_pose).pack(fill=tk.X, pady=2)
        ttk.Button(nav_section.body, text="Cancel Navigation", command=self.cancel_navigation).pack(fill=tk.X, pady=2)
        ttk.Button(nav_section.body, text="Get Navigation Status", command=self.get_nav_status).pack(fill=tk.X, pady=2)
        
        # System Controls
        sys_section = CollapsibleSection(basic_controls.body, "System Management")
        sys_section.pack(fill=tk.X, pady=2)
        
        # Navigation stack
        nav2_frame = ttk.LabelFrame(sys_section.body, text="Navigation Stack", padding=5)
        nav2_frame.pack(fill=tk.X, pady=2)
        ttk.Button(nav2_frame, text="Start Navigation", command=self.start_navigation).pack(fill=tk.X, pady=1)
        ttk.Button(nav2_frame, text="Stop Navigation", command=self.stop_navigation).pack(fill=tk.X, pady=1)
        
        # Localization
        loc_frame = ttk.LabelFrame(sys_section.body, text="Localization", padding=5)
        loc_frame.pack(fill=tk.X, pady=2)
        
        self.map_label = ttk.Label(loc_frame, text="No map selected", wraplength=200)
        self.map_label.pack(fill=tk.X, pady=1)
        
        ttk.Button(loc_frame, text="Browse Map", command=self.browse_map).pack(fill=tk.X, pady=1)
        ttk.Button(loc_frame, text="Start Localization", command=self.start_localization).pack(fill=tk.X, pady=1)
        ttk.Button(loc_frame, text="Stop Localization", command=self.stop_localization).pack(fill=tk.X, pady=1)
        
        # SLAM
        slam_frame = ttk.LabelFrame(sys_section.body, text="SLAM (Mapping)", padding=5)
        slam_frame.pack(fill=tk.X, pady=2)
        ttk.Button(slam_frame, text="Start SLAM", command=self.start_slam).pack(fill=tk.X, pady=1)
        ttk.Button(slam_frame, text="Stop SLAM", command=self.stop_slam).pack(fill=tk.X, pady=1)
        ttk.Button(slam_frame, text="Save Map", command=self.save_map).pack(fill=tk.X, pady=1)
        
        # Waypoints & Routes
        way_section = CollapsibleSection(basic_controls.body, "Waypoints & Routes")
        way_section.pack(fill=tk.X, pady=2)
        
        # Waypoint List
        ttk.Label(way_section.body, text="Waypoint Library:").pack(anchor=tk.W)
        self.waypoint_listbox = tk.Listbox(way_section.body, height=5)
        self.waypoint_listbox.pack(fill=tk.X, pady=2)
        
        # Route List
        ttk.Label(way_section.body, text="Current Route:").pack(anchor=tk.W)
        self.route_listbox = tk.Listbox(way_section.body, height=4)
        self.route_listbox.pack(fill=tk.X, pady=2)
        
        # Waypoint controls
        wp_btn_frame = ttk.Frame(way_section.body)
        wp_btn_frame.pack(fill=tk.X, pady=2)
        
        wp_row1 = ttk.Frame(wp_btn_frame)
        wp_row1.pack(fill=tk.X, pady=1)
        ttk.Button(wp_row1, text="Add Current Pose", command=self.add_waypoint_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(wp_row1, text="Remove", command=self.remove_waypoint_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        
        wp_row2 = ttk.Frame(wp_btn_frame)
        wp_row2.pack(fill=tk.X, pady=1)
        ttk.Button(wp_row2, text="Go to Waypoint", command=self.go_to_waypoint_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(wp_row2, text="Refresh List", command=self.refresh_waypoint_lists).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        
        wp_row3 = ttk.Frame(wp_btn_frame)
        wp_row3.pack(fill=tk.X, pady=1)
        ttk.Button(wp_row3, text="Save WPs", command=self.save_waypoints_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(wp_row3, text="Load WPs", command=self.load_waypoints_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)

        # Camera Control Section
        camera_controls = CollapsibleSection(control_panel, "Camera Control")
        camera_buttons = ttk.Frame(camera_controls.body)
        camera_buttons.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(camera_buttons, text="Start Camera",
                  command=lambda: self.call_robot_service('start_oakd')).pack(side=tk.LEFT, padx=2)
        ttk.Button(camera_buttons, text="Stop Camera",
                  command=lambda: self.call_robot_service('stop_oakd')).pack(side=tk.LEFT, padx=2)
        
        # Pack all sections
        basic_controls.pack(fill=tk.X, padx=5, pady=2)
        camera_controls.pack(fill=tk.X, padx=5, pady=2)
        
        return control_panel

    def setup_display_panel(self, parent):
        """Setup the display panel for sensor data visualization"""
        
        # Create notebook for tabs
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # Camera Tab
        if self.enable_camera:
            camera_frame = ttk.Frame(notebook)
            notebook.add(camera_frame, text="Camera")
            
            self.image_label = ttk.Label(camera_frame, text="No image data")
            self.image_label.pack(expand=True)
        
        # Trajectory Tab
        trajectory_frame = ttk.Frame(notebook)
        notebook.add(trajectory_frame, text="Trajectory")
        
        # Create matplotlib figure for trajectory
        self.trajectory_fig = Figure(figsize=(6, 4), dpi=100)
        self.trajectory_ax = self.trajectory_fig.add_subplot(111)
        self.trajectory_canvas = FigureCanvasTkAgg(self.trajectory_fig, trajectory_frame)
        self.trajectory_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Scan Tab
        scan_frame = ttk.Frame(notebook)
        notebook.add(scan_frame, text="Laser Scan")
        
        # Create matplotlib figure for scan
        self.scan_fig = Figure(figsize=(6, 4), dpi=100)
        self.scan_ax = self.scan_fig.add_subplot(111, projection='polar')
        self.scan_canvas = FigureCanvasTkAgg(self.scan_fig, scan_frame)
        self.scan_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Status Tab
        status_frame = ttk.Frame(notebook)
        notebook.add(status_frame, text="Status")
        
        self.status_text = tk.Text(status_frame, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=scrollbar.set)
        
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # === Callback methods for display data ===
    
    def image_callback(self, msg):
        """Callback for camera image (display only)"""
        with self.lock:
            self.current_image = msg
    
    def odom_callback(self, msg):
        """Callback for odometry data (display only)"""
        with self.lock:
            self.current_odom = msg
            
            # Store trajectory point
            if len(self.trajectory_data) > self.max_trajectory_points:
                self.trajectory_data.pop(0)
            
            self.trajectory_data.append({
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'time': time.time()
            })
    
    def path_callback(self, msg):
        """Callback for navigation path (display only)"""
        with self.lock:
            self.current_path = msg
    
    def scan_callback(self, msg):
        """Callback for laser scan (display only)"""
        with self.lock:
            self.current_scan = msg
    
    def battery_callback(self, msg):
        """Callback for battery status (display only)"""
        with self.lock:
            self.current_battery = msg

    # === Service-based robot control methods ===
    
    def get_robot_status(self):
        """Get comprehensive robot status via service"""
        result = self.call_robot_service('get_status')
        self.display_service_result("Robot Status", result)
    
    def dock_robot(self):
        """Dock robot via service"""
        result = self.call_robot_service('dock')
        self.display_service_result("Dock", result)
    
    def undock_robot(self):
        """Undock robot via service"""
        result = self.call_robot_service('undock')
        self.display_service_result("Undock", result)
    
    def get_dock_status(self):
        """Get dock status via service"""
        result = self.call_robot_service('get_dock_status')
        self.display_service_result("Dock Status", result)
    
    def set_velocity(self, linear, angular):
        """Set robot velocity via service"""
        result = self.call_robot_service('set_velocity', {
            'linear': linear,
            'angular': angular
        })
        self.display_service_result("Set Velocity", result)
    
    def set_custom_velocity(self):
        """Set custom velocity from GUI sliders"""
        linear = self.linear_vel_var.get()
        angular = self.angular_vel_var.get()
        self.set_velocity(linear, angular)
    
    def stop_robot(self):
        """Stop robot via service"""
        result = self.call_robot_service('set_velocity', {
            'linear': 0.0,
            'angular': 0.0
        })
        self.display_service_result("Stop Robot", result)
    
    def spin_robot(self):
        """Spin robot by specified angle via service"""
        angle = self.spin_angle_var.get()
        result = self.call_robot_service('spin', {
            'angle_degrees': angle
        })
        self.display_service_result("Spin Robot", result)
    
    def go_to_pose(self):
        """Navigate to specified pose via service"""
        x = self.pose_x_var.get()
        y = self.pose_y_var.get()
        yaw = self.pose_yaw_var.get()
        
        result = self.call_robot_service('go_to_pose', {
            'x': x,
            'y': y,
            'yaw': yaw,
            'frame_id': 'map'
        })
        self.display_service_result("Go to Pose", result)
    
    def cancel_navigation(self):
        """Cancel current navigation via service"""
        result = self.call_robot_service('cancel_navigation')
        self.display_service_result("Cancel Navigation", result)
    
    def get_nav_status(self):
        """Get navigation status via service"""
        result = self.call_robot_service('get_navigation_status')
        self.display_service_result("Navigation Status", result)
    
    def start_navigation(self):
        """Start navigation stack via service"""
        result = self.call_robot_service('start_navigation')
        self.display_service_result("Start Navigation", result)
    
    def stop_navigation(self):
        """Stop navigation stack via service"""
        result = self.call_robot_service('stop_navigation')
        self.display_service_result("Stop Navigation", result)
    
    def browse_map(self):
        """Browse for map file"""
        filename = filedialog.askopenfilename(
            title="Select Map File",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        if filename:
            self.selected_map = filename
            self.map_label.config(text=f"Map: {os.path.basename(filename)}")
    
    def start_localization(self):
        """Start localization with selected map via service"""
        args = {}
        if self.selected_map:
            args['map_file'] = self.selected_map
        
        result = self.call_robot_service('start_localization', args)
        self.display_service_result("Start Localization", result)
    
    def stop_localization(self):
        """Stop localization via service"""
        result = self.call_robot_service('stop_localization')
        self.display_service_result("Stop Localization", result)
    
    def start_slam(self):
        """Start SLAM via service"""
        result = self.call_robot_service('start_slam')
        self.display_service_result("Start SLAM", result)
    
    def stop_slam(self):
        """Stop SLAM via service"""
        result = self.call_robot_service('stop_slam')
        self.display_service_result("Stop SLAM", result)
    
    def save_map(self):
        """Save current map via service"""
        filename = filedialog.asksaveasfilename(
            title="Save Map As",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        if filename:
            # Remove .yaml extension if present, as the service adds it
            if filename.endswith('.yaml'):
                filename = filename[:-5]
            
            result = self.call_robot_service('save_map', {
                'map_path': filename
            })
            self.display_service_result("Save Map", result)
    
    def add_waypoint_gui(self):
        """Add waypoint at current pose via service"""
        label = tk.simpledialog.askstring("Add Waypoint", "Enter waypoint label:")
        if label:
            result = self.call_robot_service('add_waypoint', {
                'label': label
            })
            self.display_service_result("Add Waypoint", result)
            if result.get('success'):
                self.refresh_waypoint_lists()
    
    def remove_waypoint_gui(self):
        """Remove selected waypoint via service"""
        selection = self.waypoint_listbox.curselection()
        if selection:
            label = self.waypoint_listbox.get(selection[0])
            result = self.call_robot_service('remove_waypoint', {
                'label': label
            })
            self.display_service_result("Remove Waypoint", result)
            if result.get('success'):
                self.refresh_waypoint_lists()
    
    def go_to_waypoint_gui(self):
        """Navigate to selected waypoint via service"""
        selection = self.waypoint_listbox.curselection()
        if selection:
            label = self.waypoint_listbox.get(selection[0])
            result = self.call_robot_service('move_to_waypoint', {
                'label': label
            })
            self.display_service_result("Go to Waypoint", result)
    
    def save_waypoints_gui(self):
        """Save waypoints to file via service"""
        filename = filedialog.asksaveasfilename(
            title="Save Waypoints",
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        if filename:
            result = self.call_robot_service('save_waypoints', {
                'filename': filename
            })
            self.display_service_result("Save Waypoints", result)
    
    def load_waypoints_gui(self):
        """Load waypoints from file via service"""
        filename = filedialog.askopenfilename(
            title="Load Waypoints",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        if filename:
            result = self.call_robot_service('load_waypoints', {
                'filename': filename
            })
            self.display_service_result("Load Waypoints", result)
            if result.get('success'):
                self.refresh_waypoint_lists()
    
    def refresh_waypoint_lists(self):
        """Refresh waypoint list from service"""
        result = self.call_robot_service('list_waypoints')
        if result.get('success') and result.get('data'):
            waypoints = result['data']
            
            # Update waypoint listbox
            self.waypoint_listbox.delete(0, tk.END)
            for wp in waypoints:
                self.waypoint_listbox.insert(tk.END, wp)
    
    def display_service_result(self, operation, result):
        """Display service call result in status tab"""
        timestamp = time.strftime("%H:%M:%S")
        
        if result.get('success'):
            message = f"[{timestamp}] {operation}: ✓ {result.get('result', 'Success')}\n"
            if result.get('data') is not None:
                message += f"  Data: {json.dumps(result['data'], indent=2)}\n"
        else:
            message = f"[{timestamp}] {operation}: ✗ {result.get('error', 'Failed')}\n"
        
        # Update status display
        self.status_text.insert(tk.END, message)
        self.status_text.see(tk.END)
        
        # Also log to ROS
        if result.get('success'):
            self.get_logger().info(f"{operation}: {result.get('result', 'Success')}")
        else:
            self.get_logger().error(f"{operation}: {result.get('error', 'Failed')}")

    def update_gui(self):
        """Update GUI displays with current sensor data"""
        
        # Update image display
        if self.enable_camera and self.current_image:
            self.update_image_display()
        
        # Update trajectory plot
        if self.current_odom:
            self.update_trajectory_plot()
        
        # Update scan plot
        if self.current_scan:
            self.update_scan_plot()
        
        # Update battery status
        if self.current_battery:
            self.update_battery_display()
    
    def update_image_display(self):
        """Update camera image display"""
        if not self.current_image:
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.current_image, "bgr8")
            
            # Resize image
            height, width = cv_image.shape[:2]
            if width > self.image_max_size or height > self.image_max_size:
                scale = min(self.image_max_size / width, self.image_max_size / height)
                new_width = int(width * scale)
                new_height = int(height * scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Convert to PhotoImage and display
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            image_pil = Image.fromarray(image_rgb)
            photo = ImageTk.PhotoImage(image_pil)
            
            self.image_label.config(image=photo)
            self.image_label.image = photo  # Keep a reference
            
        except Exception as e:
            self.get_logger().warn(f"Image display error: {e}")
    
    def update_trajectory_plot(self):
        """Update robot trajectory plot"""
        if not self.trajectory_data:
            return
        
        try:
            # Extract x, y coordinates
            x_coords = [point['x'] for point in self.trajectory_data]
            y_coords = [point['y'] for point in self.trajectory_data]
            
            # Clear and plot
            self.trajectory_ax.clear()
            self.trajectory_ax.plot(x_coords, y_coords, 'b-', linewidth=1)
            self.trajectory_ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=5)  # Current position
            
            self.trajectory_ax.set_xlabel('X (m)')
            self.trajectory_ax.set_ylabel('Y (m)')
            self.trajectory_ax.set_title('Robot Trajectory')
            self.trajectory_ax.grid(True)
            self.trajectory_ax.axis('equal')
            
            self.trajectory_canvas.draw()
            
        except Exception as e:
            self.get_logger().warn(f"Trajectory plot error: {e}")
    
    def update_scan_plot(self):
        """Update laser scan polar plot"""
        if not self.current_scan:
            return
        
        try:
            # Extract scan data
            ranges = np.array(self.current_scan.ranges)
            angles = np.linspace(self.current_scan.angle_min, 
                               self.current_scan.angle_max, 
                               len(ranges))
            
            # Filter invalid readings
            valid_idx = np.isfinite(ranges) & (ranges > self.current_scan.range_min) & (ranges < self.current_scan.range_max)
            ranges = ranges[valid_idx]
            angles = angles[valid_idx]
            
            # Clear and plot
            self.scan_ax.clear()
            self.scan_ax.scatter(angles, ranges, c='red', s=1)
            self.scan_ax.set_ylim(0, min(self.current_scan.range_max, 10))
            self.scan_ax.set_title('Laser Scan')
            
            self.scan_canvas.draw()
            
        except Exception as e:
            self.get_logger().warn(f"Scan plot error: {e}")
    
    def update_battery_display(self):
        """Update battery status in GUI"""
        if not self.current_battery:
            return
        
        try:
            voltage = self.current_battery.voltage
            percentage = self.current_battery.percentage * 100 if self.current_battery.percentage > 0 else -1
            
            if percentage >= 0:
                battery_text = f"Battery: {percentage:.1f}% ({voltage:.1f}V)"
            else:
                battery_text = f"Battery: {voltage:.1f}V"
            
            # Update status label
            current_text = self.status_label.cget("text")
            if "Battery:" in current_text:
                # Replace existing battery info
                lines = current_text.split('\n')
                for i, line in enumerate(lines):
                    if "Battery:" in line:
                        lines[i] = battery_text
                        break
                self.status_label.config(text='\n'.join(lines))
            else:
                # Add battery info
                self.status_label.config(text=current_text + '\n' + battery_text)
            
        except Exception as e:
            self.get_logger().warn(f"Battery display error: {e}")
    
    def quit_gui(self):
        """Clean shutdown of GUI"""
        self.get_logger().info("Shutting down GUI...")
        self.root.quit()
        self.root.destroy()


# Import these after the class definition to avoid circular imports
import tkinter.simpledialog
from PIL import Image, ImageTk


def main(args=None):
    """Main function to start the service-based GUI"""
    
    rclpy.init(args=args)
    
    try:
        gui_node = ServiceBasedGUINode()
        
        def process_ros_events():
            """Process ROS events in a separate thread"""
            while rclpy.ok():
                try:
                    rclpy.spin_once(gui_node, timeout_sec=0.1)
                except Exception as e:
                    gui_node.get_logger().error(f"ROS spin error: {e}")
                    break
        
        # Start ROS processing in background thread
        ros_thread = threading.Thread(target=process_ros_events, daemon=True)
        ros_thread.start()
        
        # Start GUI main loop
        gui_node.root.mainloop()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"GUI Error: {e}")
    finally:
        if rclpy.ok():
            try:
                gui_node.destroy_node()
            except:
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    main() 