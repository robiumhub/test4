import json
from typing import Any, Dict, List
import time

class RobotAPI:
    """
    LLM-callable API for robot/GUI control.
    Each method returns a dict with 'result' and additional fields as needed.
    """
    def __init__(self, gui_node):
        self.gui = gui_node
        self.controller = gui_node.controller

    def move_to_waypoint(self, label: str) -> Dict[str, Any]:
        try:
            self.gui.go_to_waypoint_gui(label)
            return {"result": "success", "message": f"Navigating to waypoint: {label}"}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def add_waypoint(self, label: str) -> Dict[str, Any]:
        try:
            self.gui.add_waypoint_gui(label)
            return {"result": "success", "message": f"Added waypoint: {label}"}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def remove_waypoint(self, label: str) -> Dict[str, Any]:
        try:
            self.controller.remove_waypoint(label)
            self.gui.refresh_waypoint_and_route_lists()
            return {"result": "success", "message": f"Removed waypoint: {label}"}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def list_waypoints(self) -> Dict[str, Any]:
        try:
            waypoints = [wp.label for wp in self.controller.get_waypoints()]
            return {"result": "success", "waypoints": waypoints}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def navigate_route(self) -> Dict[str, Any]:
        try:
            self.gui.navigate_route_gui()
            return {"result": "success", "message": "Navigating current route."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def add_to_route(self, label: str) -> Dict[str, Any]:
        try:
            self.controller.add_to_route(label)
            self.gui.refresh_waypoint_and_route_lists()
            return {"result": "success", "message": f"Added {label} to route."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def clear_route(self) -> Dict[str, Any]:
        try:
            self.controller.clear_route()
            self.gui.refresh_waypoint_and_route_lists()
            return {"result": "success", "message": "Route cleared."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def get_battery_status(self) -> Dict[str, Any]:
        try:
            state = self.gui.battery_state
            if state:
                return {"result": "success", "percentage": state.percentage, "voltage": state.voltage}
            else:
                return {"result": "error", "message": "No battery state available"}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def dock(self) -> Dict[str, Any]:
        try:
            self.gui.dock_robot()
            return {"result": "success", "message": "Docking robot."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def undock(self) -> Dict[str, Any]:
        try:
            self.gui.undock_robot()
            return {"result": "success", "message": "Undocking robot."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def relocalize_at_dock(self) -> Dict[str, Any]:
        try:
            self.gui.relocalize_at_dock()
            return {"result": "success", "message": "Relocalized at dock (0,0)."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def get_status(self) -> Dict[str, Any]:
        try:
            status = {
                "dock_status": getattr(self.gui, 'dock_status', None),
                "battery": getattr(self.gui, 'battery_state', None),
                "current_route": [wp.label for wp in self.controller.get_route()],
                "waypoints": [wp.label for wp in self.controller.get_waypoints()],
            }
            return {"result": "success", "status": status}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def move_forward(self, duration: float = 1.0, speed: float = 0.2) -> Dict[str, Any]:
        """Move robot forward for a set duration (seconds) at given speed (m/s)."""
        try:
            self.gui.set_velocity(speed, 0.0)
            time.sleep(duration)
            self.gui.stop_robot()
            return {"result": "success", "message": f"Moved forward for {duration} seconds at {speed} m/s."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def move_backward(self, duration: float = 1.0, speed: float = 0.2) -> Dict[str, Any]:
        """Move robot backward for a set duration (seconds) at given speed (m/s)."""
        try:
            self.gui.set_velocity(-speed, 0.0)
            time.sleep(duration)
            self.gui.stop_robot()
            return {"result": "success", "message": f"Moved backward for {duration} seconds at {speed} m/s."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def turn_left(self, duration: float = 1.0, angular_speed: float = 0.5) -> Dict[str, Any]:
        """Turn robot left for a set duration (seconds) at given angular speed (rad/s)."""
        try:
            self.gui.set_velocity(0.0, angular_speed)
            time.sleep(duration)
            self.gui.stop_robot()
            return {"result": "success", "message": f"Turned left for {duration} seconds at {angular_speed} rad/s."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def turn_right(self, duration: float = 1.0, angular_speed: float = 0.5) -> Dict[str, Any]:
        """Turn robot right for a set duration (seconds) at given angular speed (rad/s)."""
        try:
            self.gui.set_velocity(0.0, -angular_speed)
            time.sleep(duration)
            self.gui.stop_robot()
            return {"result": "success", "message": f"Turned right for {duration} seconds at {angular_speed} rad/s."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def set_velocity(self, linear: float = 0.0, angular: float = 0.0) -> Dict[str, Any]:
        """Set robot linear and angular velocity (m/s, rad/s)."""
        try:
            self.gui.set_velocity(linear, angular)
            return {"result": "success", "message": f"Set velocity: linear={linear}, angular={angular}"}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def stop(self) -> Dict[str, Any]:
        """Stop all robot movement."""
        try:
            self.gui.stop_robot()
            return {"result": "success", "message": "Robot stopped."}
        except Exception as e:
            return {"result": "error", "message": str(e)}

    def handle_call(self, call_json: Dict[str, Any]) -> Dict[str, Any]:
        """
        call_json: dict with 'function' and 'args'
        Example: {"function": "move_to_waypoint", "args": {"label": "dock"}}
        """
        func = call_json.get("function")
        args = call_json.get("args", {})
        if not hasattr(self, func):
            return {"result": "error", "message": f"Unknown function: {func}"}
        try:
            return getattr(self, func)(**args)
        except Exception as e:
            return {"result": "error", "message": str(e)}

# Example OpenAI-compatible function schema (for documentation or LLM use)
OPENAI_FUNCTIONS = [
    {
        "function": "move_to_waypoint",
        "description": "Navigate the robot to a named waypoint.",
        "args": {"label": "str (The name of the waypoint)"}
    },
    {
        "function": "add_waypoint",
        "description": "Add a new waypoint at the robot's current pose.",
        "args": {"label": "str (The name for the new waypoint)"}
    },
    {
        "function": "remove_waypoint",
        "description": "Remove a named waypoint.",
        "args": {"label": "str (The name of the waypoint to remove)"}
    },
    {
        "function": "list_waypoints",
        "description": "List all saved waypoints.",
        "args": {}
    },
    {
        "function": "navigate_route",
        "description": "Navigate through the current route (sequence of waypoints).",
        "args": {}
    },
    {
        "function": "add_to_route",
        "description": "Add a waypoint to the current route.",
        "args": {"label": "str (The name of the waypoint to add to the route)"}
    },
    {
        "function": "clear_route",
        "description": "Clear the current route.",
        "args": {}
    },
    {
        "function": "get_battery_status",
        "description": "Get the current battery percentage and voltage.",
        "args": {}
    },
    {
        "function": "dock",
        "description": "Dock the robot.",
        "args": {}
    },
    {
        "function": "undock",
        "description": "Undock the robot.",
        "args": {}
    },
    {
        "function": "relocalize_at_dock",
        "description": "Set initial pose at dock (0,0).",
        "args": {}
    },
    {
        "function": "get_status",
        "description": "Get current status (docking, battery, etc.).",
        "args": {}
    },
    {
        "function": "move_forward",
        "description": "Move robot forward for a set duration (seconds) at given speed (m/s).",
        "args": {"duration": "float (Duration in seconds, default 1.0)", "speed": "float (Speed in m/s, default 0.2)"}
    },
    {
        "function": "move_backward",
        "description": "Move robot backward for a set duration (seconds) at given speed (m/s).",
        "args": {"duration": "float (Duration in seconds, default 1.0)", "speed": "float (Speed in m/s, default 0.2)"}
    },
    {
        "function": "turn_left",
        "description": "Turn robot left for a set duration (seconds) at given angular speed (rad/s).",
        "args": {"duration": "float (Duration in seconds, default 1.0)", "angular_speed": "float (Angular speed in rad/s, default 0.5)"}
    },
    {
        "function": "turn_right",
        "description": "Turn robot right for a set duration (seconds) at given angular speed (rad/s).",
        "args": {"duration": "float (Duration in seconds, default 1.0)", "angular_speed": "float (Angular speed in rad/s, default 0.5)"}
    },
    {
        "function": "set_velocity",
        "description": "Set robot linear and angular velocity (m/s, rad/s).",
        "args": {"linear": "float (Linear velocity in m/s, default 0.0)", "angular": "float (Angular velocity in rad/s, default 0.0)"}
    },
    {
        "function": "stop",
        "description": "Stop all robot movement.",
        "args": {}
    }
] 