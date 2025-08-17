#!/usr/bin/env python3

import argparse
import sys
import rclpy
from rclpy.node import Node
import json
import time
import signal
import atexit
from spatibot_interfaces.srv import ExecuteCommand

# Global CLI node reference for signal handlers
_global_cli_node = None

class RobotCLIClient(Node):
    """CLI service client for robot API functions"""
    
    def __init__(self):
        super().__init__('robot_cli_client')
        
        # Create service client
        self.service_client = self.create_client(ExecuteCommand, '/spatibot/execute_command')
        
        # Wait for service to be available
        self.get_logger().info("Waiting for robot API service...")
        if not self.service_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Robot API service not available! Make sure robot_api_node is running.")
            raise RuntimeError("Robot API service not available")
        
        self.get_logger().info("Connected to robot API service")
        
    def call_function(self, function_name: str, args: dict):
        """Call a robot API function via service and return result"""
        try:
            # Create service request
            request = ExecuteCommand.Request()
            request.function_name = function_name
            request.args_json = json.dumps(args) if args else ""
            
            # Call service
            self.get_logger().debug(f"Calling service: {function_name} with args: {request.args_json}")
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

def create_parser():
    """Create the argument parser with all available commands"""
    parser = argparse.ArgumentParser(description="Robot API CLI Testing Tool")
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # === Navigation Commands ===
    nav_group = subparsers.add_parser('nav', help='Navigation commands')
    nav_subparsers = nav_group.add_subparsers(dest='nav_command')
    
    # Go to pose
    goto_parser = nav_subparsers.add_parser('goto', help='Navigate to pose')
    goto_parser.add_argument('x', type=float, help='X coordinate')
    goto_parser.add_argument('y', type=float, help='Y coordinate')
    goto_parser.add_argument('--yaw', type=float, default=0.0, help='Yaw angle in degrees')
    goto_parser.add_argument('--frame', default='map', help='Frame ID')
    
    # Movement commands
    nav_subparsers.add_parser('forward', help='Move forward').add_argument('distance', type=float, help='Distance in meters')
    nav_subparsers.add_parser('backward', help='Move backward').add_argument('distance', type=float, help='Distance in meters')
    nav_subparsers.add_parser('spin', help='Spin by angle').add_argument('angle', type=float, help='Angle in degrees')
    
    # Navigation control
    nav_subparsers.add_parser('cancel', help='Cancel navigation')
    nav_subparsers.add_parser('status', help='Get navigation status')
    
    # === Waypoint Commands ===
    wp_group = subparsers.add_parser('waypoint', help='Waypoint commands')
    wp_subparsers = wp_group.add_subparsers(dest='wp_command')
    
    wp_subparsers.add_parser('add', help='Add waypoint at current position').add_argument('label', help='Waypoint label')
    wp_subparsers.add_parser('remove', help='Remove waypoint').add_argument('label', help='Waypoint label')
    wp_subparsers.add_parser('list', help='List all waypoints')
    wp_subparsers.add_parser('goto', help='Go to waypoint').add_argument('label', help='Waypoint label')
    wp_subparsers.add_parser('info', help='Get waypoint info').add_argument('label', help='Waypoint label')
    
    # Waypoint file operations
    wp_save = wp_subparsers.add_parser('save', help='Save waypoints to file')
    wp_save.add_argument('filename', help='File to save to')
    wp_load = wp_subparsers.add_parser('load', help='Load waypoints from file')
    wp_load.add_argument('filename', help='File to load from')
    
    # === Robot Control Commands ===
    ctrl_group = subparsers.add_parser('control', help='Robot control commands')
    ctrl_subparsers = ctrl_group.add_subparsers(dest='ctrl_command')
    
    # Velocity control
    vel_parser = ctrl_subparsers.add_parser('velocity', help='Set robot velocity')
    vel_parser.add_argument('linear', type=float, help='Linear velocity (m/s)')
    vel_parser.add_argument('angular', type=float, help='Angular velocity (rad/s)')
    vel_parser.add_argument('--timeout', type=float, help='Timeout in seconds')
    
    ctrl_subparsers.add_parser('stop', help='Stop robot')
    ctrl_subparsers.add_parser('dock', help='Dock robot')
    ctrl_subparsers.add_parser('undock', help='Undock robot')
    
    # === System Commands ===
    sys_group = subparsers.add_parser('system', help='System management commands')
    sys_subparsers = sys_group.add_subparsers(dest='sys_command')
    
    # Localization
    loc_start = sys_subparsers.add_parser('start-localization', help='Start localization')
    loc_start.add_argument('--map', help='Map file path')
    sys_subparsers.add_parser('stop-localization', help='Stop localization')
    
    # Navigation stack
    sys_subparsers.add_parser('start-navigation', help='Start navigation stack')
    sys_subparsers.add_parser('stop-navigation', help='Stop navigation stack')
    
    # SLAM
    sys_subparsers.add_parser('start-slam', help='Start SLAM')
    sys_subparsers.add_parser('stop-slam', help='Stop SLAM')
    
    # Map operations
    map_save = sys_subparsers.add_parser('save-map', help='Save current map')
    map_save.add_argument('path', help='Path to save map')
    map_load = sys_subparsers.add_parser('load-map', help='Load map')
    map_load.add_argument('path', help='Path to map file')
    
    # === Status Commands ===
    status_group = subparsers.add_parser('status', help='Status and sensor commands')
    status_subparsers = status_group.add_subparsers(dest='status_command')
    
    status_subparsers.add_parser('battery', help='Get battery status')
    status_subparsers.add_parser('pose', help='Get robot pose')
    status_subparsers.add_parser('dock', help='Get dock status')
    status_subparsers.add_parser('all', help='Get comprehensive status')
    
    # === Camera Commands ===
    camera_group = subparsers.add_parser('camera', help='Camera commands')
    camera_subparsers = camera_group.add_subparsers(dest='camera_command')
    
    # Camera control
    camera_subparsers.add_parser('start', help='Start OAK-D camera')
    camera_subparsers.add_parser('stop', help='Stop OAK-D camera')
    camera_subparsers.add_parser('current', help='Get current camera image')
    
    # === Utility Commands ===
    util_group = subparsers.add_parser('util', help='Utility commands')
    util_subparsers = util_group.add_subparsers(dest='util_command')
    
    util_subparsers.add_parser('functions', help='List all available API functions')
    
    return parser

def execute_command(cli_node, args):
    """Execute a single command and return result"""
    result = None
    
    # === Navigation Commands ===
    if args.command == 'nav':
        if args.nav_command == 'goto':
            result = cli_node.call_function('go_to_pose', {
                'x': args.x, 'y': args.y, 'yaw': args.yaw, 'frame_id': args.frame
            })
        elif args.nav_command == 'forward':
            result = cli_node.call_function('go_forward', {'distance': args.distance})
        elif args.nav_command == 'backward':
            result = cli_node.call_function('go_backward', {'distance': args.distance})
        elif args.nav_command == 'spin':
            result = cli_node.call_function('spin', {'angle_degrees': args.angle})
        elif args.nav_command == 'cancel':
            result = cli_node.call_function('cancel_navigation', {})
        elif args.nav_command == 'status':
            result = cli_node.call_function('get_navigation_status', {})
    
    # === Waypoint Commands ===
    elif args.command == 'waypoint':
        if args.wp_command == 'add':
            result = cli_node.call_function('add_waypoint', {'label': args.label})
        elif args.wp_command == 'remove':
            result = cli_node.call_function('remove_waypoint', {'label': args.label})
        elif args.wp_command == 'list':
            result = cli_node.call_function('list_waypoints', {})
        elif args.wp_command == 'goto':
            result = cli_node.call_function('move_to_waypoint', {'label': args.label})
        elif args.wp_command == 'info':
            result = cli_node.call_function('get_waypoint_info', {'label': args.label})
        elif args.wp_command == 'save':
            result = cli_node.call_function('save_waypoints', {'filename': args.filename})
        elif args.wp_command == 'load':
            result = cli_node.call_function('load_waypoints', {'filename': args.filename})
    
    # === Control Commands ===
    elif args.command == 'control':
        if args.ctrl_command == 'velocity':
            call_args = {'linear': args.linear, 'angular': args.angular}
            if args.timeout:
                call_args['timeout'] = args.timeout
            result = cli_node.call_function('set_velocity', call_args)
        elif args.ctrl_command == 'stop':
            result = cli_node.call_function('stop', {})
        elif args.ctrl_command == 'dock':
            result = cli_node.call_function('dock', {})
        elif args.ctrl_command == 'undock':
            result = cli_node.call_function('undock', {})
    
    # === System Commands ===
    elif args.command == 'system':
        if args.sys_command == 'start-localization':
            call_args = {}
            if args.map:
                call_args['map_file'] = args.map
            result = cli_node.call_function('start_localization', call_args)
        elif args.sys_command == 'stop-localization':
            result = cli_node.call_function('stop_localization', {})
        elif args.sys_command == 'start-navigation':
            result = cli_node.call_function('start_navigation', {})
        elif args.sys_command == 'stop-navigation':
            result = cli_node.call_function('stop_navigation', {})
        elif args.sys_command == 'start-slam':
            result = cli_node.call_function('start_slam', {})
        elif args.sys_command == 'stop-slam':
            result = cli_node.call_function('stop_slam', {})
        elif args.sys_command == 'save-map':
            result = cli_node.call_function('save_map', {'map_path': args.path})
        elif args.sys_command == 'load-map':
            result = cli_node.call_function('load_map', {'map_file': args.path})
    
    # === Status Commands ===
    elif args.command == 'status':
        if args.status_command == 'battery':
            result = cli_node.call_function('get_battery_status', {})
        elif args.status_command == 'pose':
            result = cli_node.call_function('get_robot_pose', {})
        elif args.status_command == 'dock':
            result = cli_node.call_function('get_dock_status', {})
        elif args.status_command == 'all':
            result = cli_node.call_function('get_status', {})
    
    # === Camera Commands ===
    elif args.command == 'camera':
        if args.camera_command == 'start':
            result = cli_node.call_function('start_oakd', {})
        elif args.camera_command == 'stop':
            result = cli_node.call_function('stop_oakd', {})
        elif args.camera_command == 'current':
            result = cli_node.call_function('get_current_image', {})
    
    # === Utility Commands ===
    elif args.command == 'util':
        if args.util_command == 'functions':
            # Call get_api_functions via service
            functions_result = cli_node.call_function('get_api_functions', {})
            if functions_result.get('success') and functions_result.get('data'):
                functions = functions_result['data']
                print("Available API Functions:")
                print("=" * 50)
                for func in functions:
                    print(f"• {func['function']}: {func['description']}")
                    if func.get('args'):
                        for arg_name, arg_info in func['args'].items():
                            print(f"  - {arg_name}: {arg_info.get('description', 'No description')}")
            else:
                print("✗ ERROR: Could not retrieve API functions")
            return
    
    # Print result
    if result:
        if result.get('success'):
            message = result.get('result', 'Command completed')
            print("✓ SUCCESS:", message)
            
            # Display detailed data if available (like pose, battery, status info)
            if result.get('data') is not None:
                import json
                data = result['data']
                if isinstance(data, dict):
                    print("\n📊 Details:")
                    for key, value in data.items():
                        if isinstance(value, (dict, list)):
                            print(f"  {key}:")
                            print(f"    {json.dumps(value, indent=4)}")
                        else:
                            print(f"  {key}: {value}")
                elif isinstance(data, list):
                    print(f"\n📋 Items ({len(data)}):")
                    for i, item in enumerate(data):
                        print(f"  {i+1}. {item}")
                else:
                    print(f"\n📄 Data: {data}")
        else:
            print("✗ ERROR:", result.get('error', 'Unknown error'))
    else:
        print("No result returned")

def cleanup_processes(cli_node):
    """Clean up CLI client resources (not server services)"""
    if cli_node is None:
        return
        
    try:
        # Only clean up client-side resources, not server services
        # The robot API node handles its own service lifecycle
        pass
        
    except Exception as e:
        print(f"⚠ Warning: Cleanup error: {e}")

def signal_handler(signum, frame):
    """Handle termination signals"""
    global _global_cli_node
    print(f"\n🛑 Received signal {signum}, cleaning up...")
    cleanup_processes(_global_cli_node)
    if _global_cli_node:
        _global_cli_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)

def register_cleanup_handlers(cli_node):
    """Register signal handlers and atexit for cleanup"""
    global _global_cli_node
    _global_cli_node = cli_node
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Termination
    
    # Register atexit handler
    atexit.register(lambda: cleanup_processes(_global_cli_node))

def interactive_mode():
    """Run interactive command shell with simplified single-thread architecture"""
    print("🤖 Robot CLI Interactive Mode")
    print("Type 'help' for commands, 'quit' or 'exit' to leave")
    print("=" * 50)
    
    # Initialize ROS
    rclpy.init()
    cli_node = RobotCLIClient()
    parser = create_parser()
    
    # Register cleanup handlers
    register_cleanup_handlers(cli_node)
    
    print("✓ Single-thread mode: No continuous spinning needed!")
    print("\nrobot> ", end='', flush=True)
    
    try:
        while rclpy.ok():
            try:
                user_input = input().strip()
                
                if not user_input:
                    print("robot> ", end='', flush=True)
                    continue
                
                # Handle special commands
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print("Goodbye! 👋")
                    break
                elif user_input.lower() in ['help', 'h']:
                    parser.print_help()
                    print("\nrobot> ", end='', flush=True)
                    continue
                elif user_input.lower() == 'clear':
                    import os
                    os.system('clear' if os.name == 'posix' else 'cls')
                    print("robot> ", end='', flush=True)
                    continue
                
                # Parse and execute command
                try:
                    cmd_args = user_input.split()
                    args = parser.parse_args(cmd_args)
                    
                    # Execute command (spin_until_future_complete handles all spinning)
                    execute_command(cli_node, args)
                    
                except SystemExit:
                    # Argparse calls sys.exit on error, catch it
                    print("Invalid command. Type 'help' for usage.")
                except Exception as e:
                    print(f"✗ ERROR: {e}")
                
                # Display prompt for next command
                print("\nrobot> ", end='', flush=True)
                    
            except EOFError:
                print("\nGoodbye! 👋")
                break
            except KeyboardInterrupt:
                print("\nUse 'quit' or 'exit' to leave")
                print("\nrobot> ", end='', flush=True)
                continue
                
    finally:
        cleanup_processes(cli_node)
        cli_node.destroy_node()
        rclpy.shutdown()

def main():
    """Main CLI function"""
    parser = create_parser()
    
    # Check if running with arguments (single command mode)
    if len(sys.argv) > 1:
        args = parser.parse_args()
        if not args.command:
            parser.print_help()
            return
        
        # Initialize ROS and run single command
        rclpy.init()
        cli_node = RobotCLIClient()
        
        # Register cleanup handlers
        register_cleanup_handlers(cli_node)
        
        try:
            execute_command(cli_node, args)
        except KeyboardInterrupt:
            print("\nCancelled by user")
        except Exception as e:
            print(f"✗ ERROR: {e}")
        finally:
            cleanup_processes(cli_node)
            cli_node.destroy_node()
            rclpy.shutdown()
    else:
        # No arguments - run interactive mode
        interactive_mode()

if __name__ == '__main__':
    main() 