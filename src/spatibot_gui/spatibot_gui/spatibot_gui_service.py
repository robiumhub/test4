#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import json
import os
import tkinter.simpledialog

# Service interface
from spatibot_interfaces.srv import ExecuteCommand


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
        
        # Initialize service client for robot API
        self.setup_service_client()
        
        # Initialize GUI
        self.root.title("SpatiBot Control GUI (Service-Based)")
        self.root.geometry("800x600")
        
        # GUI state
        self.selected_map = None
        
        # Setup GUI
        self.setup_gui()
        
        self.get_logger().info("Service-Based SpatiBot GUI started")

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

    def setup_gui(self):
        """Setup the GUI layout"""
        
        # Create main frames
        control_frame = ttk.Frame(self.root)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        status_frame = ttk.Frame(self.root)
        status_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Control Panel
        self.setup_control_panel(control_frame)
        
        # Status Panel
        self.setup_status_panel(status_frame)

    def setup_control_panel(self, parent):
        """Setup the control panel with collapsible service-based controls"""
        
        # Service Status
        status_section = CollapsibleSection(parent, "Service Status")
        status_section.pack(fill=tk.X, pady=2)
        self.service_status_label = ttk.Label(status_section.body, text="Service: Connected ✓")
        self.service_status_label.pack(pady=2)
        ttk.Button(status_section.body, text="Get Robot Status", command=self.get_robot_status).pack(fill=tk.X, pady=2)
        ttk.Button(status_section.body, text="Get Battery Status", command=self.get_battery_status).pack(fill=tk.X, pady=2)
        
        # Movement Controls
        move_section = CollapsibleSection(parent, "Movement Controls")
        move_section.pack(fill=tk.X, pady=2)
        
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
        
        # Velocity controls with sliders
        vel_frame = ttk.Frame(move_section.body)
        vel_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(vel_frame, text="Linear Velocity (m/s)").pack()
        self.linear_vel_var = tk.DoubleVar(value=0.0)
        linear_vel_scale = ttk.Scale(vel_frame, from_=-1.0, to=1.0, 
                                   variable=self.linear_vel_var, orient=tk.HORIZONTAL)
        linear_vel_scale.pack(fill=tk.X, pady=2)
        
        ttk.Label(vel_frame, text="Angular Velocity (rad/s)").pack()
        self.angular_vel_var = tk.DoubleVar(value=0.0)
        angular_vel_scale = ttk.Scale(vel_frame, from_=-2.0, to=2.0, 
                                    variable=self.angular_vel_var, orient=tk.HORIZONTAL)
        angular_vel_scale.pack(fill=tk.X, pady=2)
        
        ttk.Button(vel_frame, text="Set Custom Velocity", 
                  command=self.set_custom_velocity).pack(fill=tk.X, pady=2)
        
        # Spin controls
        spin_frame = ttk.Frame(move_section.body)
        spin_frame.pack(fill=tk.X, pady=5)
        ttk.Label(spin_frame, text="Spin Angle (degrees):").pack()
        self.spin_angle_var = tk.DoubleVar(value=90.0)
        spin_entry = ttk.Entry(spin_frame, textvariable=self.spin_angle_var)
        spin_entry.pack(fill=tk.X, pady=2)
        ttk.Button(spin_frame, text="Spin", command=self.spin_robot).pack(fill=tk.X, pady=2)
        
        # Navigation Controls
        nav_section = CollapsibleSection(parent, "Navigation")
        nav_section.pack(fill=tk.X, pady=2)
        
        # Go to pose
        pose_frame = ttk.Frame(nav_section.body)
        pose_frame.pack(fill=tk.X, pady=5)
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
        
        # System Management
        sys_section = CollapsibleSection(parent, "System Management")
        sys_section.pack(fill=tk.X, pady=2)
        
        # Navigation stack
        nav_frame = ttk.LabelFrame(sys_section.body, text="Navigation Stack", padding=5)
        nav_frame.pack(fill=tk.X, pady=2)
        ttk.Button(nav_frame, text="Start Navigation", command=self.start_navigation).pack(fill=tk.X, pady=1)
        ttk.Button(nav_frame, text="Stop Navigation", command=self.stop_navigation).pack(fill=tk.X, pady=1)
        
        # Localization
        loc_frame = ttk.LabelFrame(sys_section.body, text="Localization", padding=5)
        loc_frame.pack(fill=tk.X, pady=2)
        
        self.map_label = ttk.Label(loc_frame, text="No map selected")
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
        
        # Docking Controls
        dock_section = CollapsibleSection(parent, "Docking Controls")
        dock_section.pack(fill=tk.X, pady=2)
        ttk.Button(dock_section.body, text="Dock Robot", command=self.dock_robot).pack(fill=tk.X, pady=2)
        ttk.Button(dock_section.body, text="Undock Robot", command=self.undock_robot).pack(fill=tk.X, pady=2)
        ttk.Button(dock_section.body, text="Get Dock Status", command=self.get_dock_status).pack(fill=tk.X, pady=2)
        
        # Waypoints & Routes
        way_section = CollapsibleSection(parent, "Waypoints & Routes")
        way_section.pack(fill=tk.X, pady=2)
        
        # Waypoint List
        ttk.Label(way_section.body, text="Waypoints:").pack(anchor=tk.W)
        self.waypoint_listbox = tk.Listbox(way_section.body, height=5)
        self.waypoint_listbox.pack(fill=tk.X, pady=2)
        
        # Waypoint controls
        wp_btn_frame = ttk.Frame(way_section.body)
        wp_btn_frame.pack(fill=tk.X, pady=2)
        
        wp_row1 = ttk.Frame(wp_btn_frame)
        wp_row1.pack(fill=tk.X, pady=1)
        ttk.Button(wp_row1, text="Add Current", command=self.add_waypoint_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(wp_row1, text="Remove", command=self.remove_waypoint_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        
        wp_row2 = ttk.Frame(wp_btn_frame)
        wp_row2.pack(fill=tk.X, pady=1)
        ttk.Button(wp_row2, text="Go To", command=self.go_to_waypoint_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(wp_row2, text="Refresh", command=self.refresh_waypoint_lists).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        
        wp_row3 = ttk.Frame(wp_btn_frame)
        wp_row3.pack(fill=tk.X, pady=1)
        ttk.Button(wp_row3, text="Save WPs", command=self.save_waypoints_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)
        ttk.Button(wp_row3, text="Load WPs", command=self.load_waypoints_gui).pack(side=tk.LEFT, padx=1, expand=True, fill=tk.X)

    def setup_status_panel(self, parent):
        """Setup the status panel for displaying results"""
        
        status_label_frame = ttk.LabelFrame(parent, text="Service Call Results", padding=10)
        status_label_frame.pack(fill=tk.BOTH, expand=True)
        
        self.status_text = tk.Text(status_label_frame, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(status_label_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=scrollbar.set)
        
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Add initial message
        self.status_text.insert(tk.END, "Service-based GUI ready!\nAll robot operations go through the robot_api_node service.\n\n")

    # === Service-based robot control methods ===
    
    def get_robot_status(self):
        """Get comprehensive robot status via service"""
        result = self.call_robot_service('get_status')
        self.display_service_result("Robot Status", result)
    
    def set_velocity(self, linear, angular):
        """Set robot velocity via service"""
        result = self.call_robot_service('set_velocity', {
            'linear': linear,
            'angular': angular
        })
        self.display_service_result("Set Velocity", result)
    
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
        label = tkinter.simpledialog.askstring("Add Waypoint", "Enter waypoint label:")
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
    
    def refresh_waypoint_lists(self):
        """Refresh waypoint list from service"""
        result = self.call_robot_service('list_waypoints')
        if result.get('success') and result.get('data'):
            waypoints = result['data']
            
            # Update waypoint listbox
            self.waypoint_listbox.delete(0, tk.END)
            for wp in waypoints:
                # waypoints might be a list of dictionaries with 'label' keys
                if isinstance(wp, dict) and 'label' in wp:
                    self.waypoint_listbox.insert(tk.END, wp['label'])
                else:
                    # Otherwise treat as string
                    self.waypoint_listbox.insert(tk.END, str(wp))
    
    def save_waypoints_gui(self):
        """Save waypoints to file via service"""
        filename = filedialog.asksaveasfilename(
            title="Save Waypoints As",
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
    
    def get_battery_status(self):
        """Get battery status via service"""
        result = self.call_robot_service('get_battery_status')
        self.display_service_result("Battery Status", result)
    
    def set_custom_velocity(self):
        """Set custom velocity using sliders"""
        linear = self.linear_vel_var.get()
        angular = self.angular_vel_var.get()
        self.set_velocity(linear, angular)
    
    def get_nav_status(self):
        """Get navigation status via service"""
        result = self.call_robot_service('get_navigation_status')
        self.display_service_result("Navigation Status", result)
    
    def dock_robot(self):
        """Dock robot via service"""
        result = self.call_robot_service('dock')
        self.display_service_result("Dock Robot", result)
    
    def undock_robot(self):
        """Undock robot via service"""
        result = self.call_robot_service('undock')
        self.display_service_result("Undock Robot", result)
    
    def get_dock_status(self):
        """Get dock status via service"""
        result = self.call_robot_service('get_dock_status')
        self.display_service_result("Dock Status", result)
    
    def display_service_result(self, operation, result):
        """Display service call result in status area"""
        timestamp = time.strftime("%H:%M:%S")
        
        if result.get('success'):
            message = f"[{timestamp}] {operation}: ✓ {result.get('result', 'Success')}\n"
            if result.get('data') is not None:
                message += f"  Data: {json.dumps(result['data'], indent=2)}\n"
        else:
            message = f"[{timestamp}] {operation}: ✗ {result.get('error', 'Failed')}\n"
        
        message += "\n"
        
        # Update status display
        self.status_text.insert(tk.END, message)
        self.status_text.see(tk.END)
        
        # Also log to ROS
        if result.get('success'):
            self.get_logger().info(f"{operation}: {result.get('result', 'Success')}")
        else:
            self.get_logger().error(f"{operation}: {result.get('error', 'Failed')}")


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