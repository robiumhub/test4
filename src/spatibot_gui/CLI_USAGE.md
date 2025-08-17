# Robot CLI Testing Tool

A comprehensive command-line interface for testing all robot API functionalities.

## Installation

After making changes to setup.py, rebuild the package:

```bash
cd isaac_ros_ws
colcon build --packages-select spatibot_gui
source install/setup.bash
```

## Usage

```bash
robot_cli <command> <subcommand> [arguments]
```

## Available Commands

### 🧭 Navigation Commands (`nav`)

```bash
# Navigate to specific coordinates
robot_cli nav goto 2.0 1.5 --yaw 90 --frame map

# Move forward/backward by distance
robot_cli nav forward 1.0
robot_cli nav backward 0.5

# Spin by angle (degrees)
robot_cli nav spin 90    # Turn left 90°
robot_cli nav spin -90   # Turn right 90°
robot_cli nav spin 180   # Turn around

# Navigation control
robot_cli nav cancel     # Cancel current navigation
robot_cli nav status     # Get navigation status
```

### 📍 Waypoint Commands (`waypoint`)

```bash
# Add waypoint at current position
robot_cli waypoint add kitchen

# Remove waypoint
robot_cli waypoint remove kitchen

# List all waypoints
robot_cli waypoint list

# Go to waypoint
robot_cli waypoint goto kitchen

# Get waypoint information
robot_cli waypoint info kitchen

# Save/load waypoints to/from file
robot_cli waypoint save /workspaces/isaac_ros-dev/maps/garage_waypoints.yaml
robot_cli waypoint load /workspaces/isaac_ros-dev/maps/garage_waypoints.yaml
```

### 🎮 Robot Control Commands (`control`)

```bash
# Set velocity (linear m/s, angular rad/s)
robot_cli control velocity 0.5 0.0           # Move forward
robot_cli control velocity 0.0 1.0           # Spin in place
robot_cli control velocity 0.5 0.0 --timeout 3.0  # Move for 3 seconds

# Stop robot
robot_cli control stop

# Docking operations
robot_cli control dock    # Dock at charging station
robot_cli control undock  # Undock from charging station
```

### ⚙️ System Management Commands (`system`)

```bash
# Localization
robot_cli system start-localization --map /workspaces/isaac_ros-dev/maps/garage.yaml
robot_cli system stop-localization

# Navigation stack
robot_cli system start-navigation
robot_cli system stop-navigation

# SLAM (mapping)
robot_cli system start-slam
robot_cli system stop-slam

# Map operations
robot_cli system save-map /workspaces/isaac_ros-dev/maps/garage.yaml
robot_cli system load-map /workspaces/isaac_ros-dev/maps/garage.yaml
```

### 📊 Status Commands (`status`)

```bash
# Get battery status
robot_cli status battery

# Get robot pose
robot_cli status pose

# Get dock status
robot_cli status dock

# Get comprehensive status
robot_cli status all
```

### 📷 Camera Commands (`camera`)

```bash
# Control OAK-D camera
robot_cli camera start    # Start OAK-D camera
robot_cli camera stop     # Stop OAK-D camera
robot_cli camera current  # Get current camera image
```

### 🔧 Utility Commands (`util`)

```bash
# List all available API functions
robot_cli util functions
```

## Example Testing Workflows

### Basic Movement Test
```bash
# Check status
robot_cli status all

# Move forward 1 meter
robot_cli nav forward 1.0

# Turn right 90 degrees
robot_cli nav spin -90

# Move backward 0.5 meters
robot_cli nav backward 0.5

# Stop
robot_cli control stop
```

### Waypoint Navigation Test
```bash
# Add waypoint at current location
robot_cli waypoint add start_position

# Move to a new location manually or via navigation
robot_cli nav goto 2.0 1.0

# Add another waypoint
robot_cli waypoint add destination

# Go back to start
robot_cli waypoint goto start_position

# List all waypoints
robot_cli waypoint list

# Save waypoints for later
robot_cli waypoint save test_waypoints.yaml
```

### System Stack Test
```bash
# Start localization with map
robot_cli system start-localization --map /path/to/map.yaml

# Start navigation stack
robot_cli system start-navigation

# Test navigation
robot_cli nav goto 1.0 1.0

# Check status
robot_cli nav status

# Stop everything
robot_cli system stop-navigation
robot_cli system stop-localization
```

### SLAM Test
```bash
# Start SLAM for mapping
robot_cli system start-slam

# Drive around to build map
robot_cli control velocity 0.3 0.0 --timeout 5.0
robot_cli nav spin 90
robot_cli control velocity 0.3 0.0 --timeout 5.0

# Save the map
robot_cli system save-map /path/to/new_map

# Stop SLAM
robot_cli system stop-slam
```

### Camera Test
```bash
# Start OAK-D camera
robot_cli camera start

# Get current image
robot_cli camera current

# Stop camera
robot_cli camera stop
```

## Tips

- Always check `robot_cli status all` to see the current robot state
- Use `robot_cli util functions` to see all available functions and their parameters
- The `--timeout` parameter in velocity commands is useful for timed movements
- Waypoints are persistent within a session but can be saved/loaded from files
- Navigation commands work best when localization and navigation stacks are running

## Error Handling

The CLI provides clear success/error feedback:
- ✓ SUCCESS: Command completed successfully
- ✗ ERROR: Command failed with error message

If a command fails, check:
1. Robot API node is running: `ros2 node list | grep robot_api`
2. Required navigation stacks are running for navigation commands
3. Robot is properly localized for navigation
4. Battery level and emergency stops 