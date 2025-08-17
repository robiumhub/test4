#!/usr/bin/env bash
# Simple script to source ROS2 environment for current shell

echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
    echo "✅ ROS2 and workspace environments sourced successfully!"
else
    echo "✅ ROS2 environment sourced successfully!"
    echo "⚠️  Workspace not built yet. Run './scripts/ros.sh build' to build packages."
fi
echo "You can now use: ros2 topic list"
