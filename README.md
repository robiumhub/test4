# SpatiBot Project

A comprehensive ROS 2 Humble workspace for TurtleBot 4 control with GUI, camera integration, and AI capabilities.

## Features

- **SpatiBot GUI**: Tkinter-based GUI for robot control with docking, movement, navigation, and visualization
- **OAK-D Camera Integration**: DepthAI camera support with real-time image processing
- **Robot API Node**: Service-based API for programmatic robot control
- **LLM Integration**: Google Gemini AI integration for natural language robot commands
- **Navigation Stack**: Full TurtleBot 4 navigation with SLAM, localization, and path planning

## Quick Start

### Prerequisites

- Docker and Docker Compose
- X11 forwarding enabled (for GUI): `xhost +local:`

### Build and Run

1. **Build the Docker image:**
   ```bash
   docker compose build
   ```

2. **Start the container:**
   ```bash
   docker compose up -d
   ```

3. **Access the container:**
   ```bash
   docker exec -it test4-dev bash
   ```

4. **Build SpatiBot packages:**
   ```bash
   ./scripts/ros.sh build
   ```

## Usage

### Using the ROS Script

The `scripts/ros.sh` script provides convenient commands for building and running SpatiBot:

```bash
# Build SpatiBot packages
./scripts/ros.sh build

# Run GUI in demo mode (with simulated data)
./scripts/ros.sh gui-demo

# Run GUI in real mode (requires TurtleBot 4)
./scripts/ros.sh gui-real

# Run Robot API Node
./scripts/ros.sh api

# Run OAK-D camera
./scripts/ros.sh oakd

# Run Robot CLI
./scripts/ros.sh cli

# Clean build directories
./scripts/ros.sh clean
```

### Manual Commands

If you prefer manual commands:

```bash
# Source ROS environment
source /opt/ros/humble/setup.bash

# Build packages
colcon build --packages-select spatibot_interfaces spatibot_gui spatibot_oakd

# Source workspace
source install/setup.bash

# Launch GUI demo
ros2 launch spatibot_gui spatibot_gui_demo.launch.py

# Launch Robot API
ros2 run spatibot_gui robot_api_node

# Launch OAK-D camera
ros2 launch spatibot_oakd oakd_with_turtlebot.launch.py
```

### Accessing ROS2 Commands

To use ROS2 commands in the container shell:

**Option 1: Use the dev-shell script (Recommended)**
```bash
./scripts/dev-shell.sh
# ROS2 is automatically available in interactive shells
ros2 topic list
```

**Option 2: Source ROS2 environment manually**
```bash
docker exec -it test4-dev bash
source scripts/source-ros.sh
ros2 topic list
```

**Option 3: Direct sourcing**
```bash
docker exec -it test4-dev bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

## Project Structure

```
src/
├── spatibot_interfaces/     # Service and message definitions
├── spatibot_gui/           # GUI application and robot control
└── spatibot_oakd/          # OAK-D camera integration
```

## Configuration

### Environment Variables

- `ROS_DISTRO=humble`: ROS 2 distribution
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`: ROS middleware
- `DISPLAY=:0`: X11 display for GUI

### Docker Configuration

- **X11 Mounting**: `/tmp/.X11-unix:/tmp/.X11-unix:rw`
- **USB Access**: `/dev/bus/usb:/dev/bus/usb` (for OAK-D camera)
- **Workspace Mount**: `./:/workspace`

## Dependencies

### ROS Packages
- `ros-humble-joy`: Joystick support
- `ros-humble-teleop-twist-joy`: Teleop control
- `ros-humble-turtlebot4-*`: TurtleBot 4 packages
- `ros-humble-rviz2`: Visualization
- `ros-humble-cv-bridge`: Computer vision bridge
- `ros-humble-rmw-cyclonedds-cpp`: CycloneDDS middleware

### Python Packages
- `numpy`, `matplotlib`, `opencv-python`: Computer vision
- `pillow`: Image processing
- `pyyaml`: Configuration files
- `depthai`: DepthAI SDK
- `google-generativeai`: LLM integration
- `psutil`: System monitoring

## Troubleshooting

### GUI Not Displaying
- Ensure X11 forwarding is enabled: `xhost +local:`
- Check DISPLAY environment variable
- Verify X11 socket is mounted in Docker

### Camera Not Working
- Check USB device permissions
- Verify OAK-D camera is connected
- Check udev rules for DepthAI devices

### Build Errors
- Ensure ROS environment is sourced: `source /opt/ros/humble/setup.bash`
- Clean and rebuild: `./scripts/ros.sh clean && ./scripts/ros.sh build`

### Network Issues
- Check Docker network configuration
- Verify DNS settings in docker-compose.yml

## Development

### Adding New Packages
1. Create package in `src/`
2. Add to build script: `colcon build --packages-select your_package`
3. Update `scripts/ros.sh` if needed

### Debugging
- Use `ros2 topic list` to see available topics
- Use `ros2 node list` to see running nodes
- Check logs with `ros2 run --prefix 'ros2 log'`

## License

This project is licensed under the Apache License 2.0.
