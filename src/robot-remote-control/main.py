#!/usr/bin/env python3

import asyncio
import json
from datetime import datetime
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import subprocess
import os
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Robot Remote Control", description="Web interface for robot control")

# Templates
templates = Jinja2Templates(directory="templates")

def run_ros_command(command):
    """Execute a ROS2 command and return the result"""
    try:
        result = subprocess.run(
            command, 
            shell=True, 
            capture_output=True, 
            text=True, 
            timeout=10
        )
        return {
            "success": result.returncode == 0,
            "output": result.stdout.strip(),
            "error": result.stderr.strip() if result.stderr else None
        }
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "Command timed out"}
    except Exception as e:
        return {"success": False, "error": str(e)}

def call_robot_service(function_name, args_json="{}"):
    """Call the robot service via ROS2"""
    # Escape quotes in the JSON for proper shell handling
    escaped_json = args_json.replace('"', '\\"')
    command = f"""ros2 service call /spatibot/execute_command spatibot_interfaces/srv/ExecuteCommand "{{function_name: '{function_name}', args_json: '{escaped_json}'}}" """
    logger.info(f"ROS command: {command}")
    return run_ros_command(command)

@app.get("/", response_class=HTMLResponse)
async def home(request: Request):
    """Main web interface"""
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api/status")
async def get_robot_status():
    """Get robot status"""
    result = call_robot_service("get_status", "{}")
    return result

@app.post("/api/navigation/{action}")
async def navigation_control(action: str, distance: float = 1.0):
    """Control robot navigation"""
    valid_actions = ["forward", "backward", "left", "right", "stop"]
    if action not in valid_actions:
        raise HTTPException(status_code=400, detail=f"Invalid action. Must be one of: {valid_actions}")
    
    if action == "stop":
        result = call_robot_service("control", '{"action": "stop"}')
    else:
        result = call_robot_service("nav", f'{{"direction": "{action}", "distance": {distance}}}')
    
    return result

@app.post("/api/control/{action}")
async def robot_control(action: str):
    """Basic robot control commands"""
    valid_actions = ["stop", "dock", "undock"]
    if action not in valid_actions:
        raise HTTPException(status_code=400, detail=f"Invalid action. Must be one of: {valid_actions}")
    
    if action == "dock":
        result = run_ros_command('ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"')
    elif action == "undock":
        result = run_ros_command('ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"')
    else:
        result = call_robot_service("control", f'{{"action": "{action}"}}')
    
    return result

@app.post("/api/velocity/{action}")
async def velocity_control(action: str, speed: float = 0.2):
    """Control robot using velocity commands"""
    valid_actions = ["forward", "backward", "left", "right", "stop"]
    if action not in valid_actions:
        raise HTTPException(status_code=400, detail=f"Invalid action. Must be one of: {valid_actions}")
    
    # Create proper JSON using json.dumps to ensure correct formatting
    if action == "stop":
        velocity_data = {"linear": 0.0, "angular": 0.0}
    elif action == "forward":
        velocity_data = {"linear": speed, "angular": 0.0}
    elif action == "backward":
        velocity_data = {"linear": -speed, "angular": 0.0}
    elif action == "left":
        velocity_data = {"linear": 0.0, "angular": speed * 2}
    elif action == "right":
        velocity_data = {"linear": 0.0, "angular": -speed * 2}
    
    # Convert to JSON string with proper formatting
    args_json = json.dumps(velocity_data)
    logger.info(f"Velocity control - Action: {action}, Speed: {speed}")
    logger.info(f"Generated JSON: {args_json}")
    
    result = call_robot_service("set_velocity", args_json)
    
    return result

# === Camera API Endpoints ===

@app.get("/api/camera/current")
async def get_current_image():
    """Get current camera image as base64"""
    try:
        result = call_robot_service("get_current_image", "{}")
        return result
    except Exception as e:
        logger.error(f"Error getting current image: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to get current image: {str(e)}")

@app.get("/api/nodes")
async def get_ros_nodes():
    """Get list of running ROS2 nodes"""
    result = run_ros_command("ros2 node list")
    if result["success"]:
        nodes = [node.strip() for node in result["output"].split('\n') if node.strip()]
        return {"success": True, "nodes": nodes}
    return result

@app.get("/api/topics")
async def get_ros_topics():
    """Get list of ROS2 topics"""
    result = run_ros_command("ros2 topic list")
    if result["success"]:
        topics = [topic.strip() for topic in result["output"].split('\n') if topic.strip()]
        return {"success": True, "topics": topics}
    return result

@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "message": "Robot Remote Control API is running"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
