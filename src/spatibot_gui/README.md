# SpatiBot GUI

A comprehensive GUI application for TurtleBot 4 control with docking, movement, and visualization capabilities.

## Features

- **Docking Controls**: Dock and undock the robot
- **Movement Controls**: Direct velocity control with sliders and preset buttons
- **Navigation**: Set goal poses and cancel navigation
- **Camera Feed**: Real-time display of the robot's camera feed (`/oakd/rgb/preview/image_raw`)
- **Trajectory Visualization**: Plot robot path and planned navigation paths
- **Laser Scan Visualization**: Real-time display of laser scan data
- **Status Monitoring**: Real-time status updates

## Installation

### Prerequisites

- ROS 2 Humble
- Python 3.8+
- TurtleBot 4 with navigation stack running

### Dependencies

The package requires the following Python packages:
- opencv-python
- matplotlib
- numpy
- pillow

### Building

1. Navigate to your ROS 2 workspace:
```bash
cd /path/to/your/workspace
```

2. Build the package:
```bash
colcon build --packages-select spatibot_gui
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Running the GUI

#### Method 1: Direct execution
```bash
ros2 run spatibot_gui spatibot_gui
```

#### Method 2: Using launch file
```bash
ros2 launch spatibot_gui spatibot_gui_launch.py
```

#### Method 3: With custom display (for remote GUI)
```bash
ros2 launch spatibot_gui spatibot_gui_launch.py display:=:0
```

### GUI Components

#### Control Panel (Left Side)
- **Docking Controls**: Dock/Undock buttons
- **Movement Controls**: 
  - Linear and angular velocity sliders
  - Preset movement buttons (Forward, Backward, Left, Right, Stop)
- **Navigation Controls**: Set goal pose and cancel navigation
- **Status Display**: Current robot status
- **Quit Button**: Safely exit the application

#### Display Panel (Right Side)
- **Camera Feed Tab**: Real-time camera image
- **Trajectory Tab**: Robot path and planned navigation paths
- **Laser Scan Tab**: Real-time laser scan visualization

### ROS 2 Topics Used

#### Subscribed Topics
- `/oakd/rgb/preview/image_raw` - Camera feed
- `/odom` - Robot odometry
- `/plan` - Navigation plan
- `/scan` - Laser scan data

#### Published Topics
- `/cmd_vel` - Robot velocity commands
- `/goal_pose` - Navigation goal poses

#### Action Clients
- `/dock` - Docking action
- `/undock` - Undocking action

## Configuration

### Display Configuration

For remote GUI access, you may need to configure X11 forwarding:

1. Enable X11 forwarding when connecting via SSH:
```bash
ssh -X user@robot_ip
```

2. Set the DISPLAY environment variable:
```bash
export DISPLAY=:0
```

### Network Configuration

If running the GUI on a different machine than the robot:

1. Set ROS_DOMAIN_ID to match your robot:
```bash
export ROS_DOMAIN_ID=<your_domain_id>
```

2. Ensure proper network connectivity between the GUI machine and robot.

## Troubleshooting

### Common Issues

1. **GUI not displaying**: Check DISPLAY environment variable and X11 forwarding
2. **No camera feed**: Verify the camera topic is publishing and robot is powered on
3. **No movement response**: Check if `/cmd_vel` topic is being published and robot is not in emergency stop
4. **Docking not working**: Ensure the robot is near the dock and docking action server is running

### Debugging

Enable debug output:
```bash
ros2 run spatibot_gui spatibot_gui --ros-args --log-level debug
```

Check topic availability:
```bash
ros2 topic list
ros2 topic echo /oakd/rgb/preview/image_raw
```

## Safety Notes

- Always ensure the robot is in a safe environment before testing movement controls
- Keep emergency stop accessible when testing autonomous navigation
- Monitor the robot during docking operations
- The GUI provides direct velocity control - use with caution

## Contributing

Feel free to submit issues and enhancement requests!

## License

This package is licensed under the Apache License 2.0. 