#!/bin/bash

# SpatiBot ROS Development Script
# This script provides convenient commands for building and running SpatiBot

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to source ROS environment
source_ros() {
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
        print_status "ROS Humble environment sourced"
    else
        print_error "ROS Humble setup.bash not found!"
        exit 1
    fi
}

# Function to source workspace
source_workspace() {
    if [ -f install/setup.bash ]; then
        source install/setup.bash
        print_status "Workspace environment sourced"
    else
        print_warning "Workspace setup.bash not found. Run build first."
    fi
}

# Function to build SpatiBot packages
build_spatibot() {
    print_status "Building SpatiBot packages..."
    source_ros
    colcon build --packages-select spatibot_interfaces spatibot_gui spatibot_oakd
    source_workspace
    print_success "SpatiBot packages built successfully!"
}

# Function to build all packages
build_all() {
    print_status "Building all packages..."
    source_ros
    colcon build
    source_workspace
    print_success "All packages built successfully!"
}

# Function to clean build
clean_build() {
    print_status "Cleaning build directories..."
    rm -rf build/ install/ log/
    print_success "Build directories cleaned!"
}

# Function to run GUI demo
run_gui_demo() {
    print_status "Starting SpatiBot GUI Demo..."
    source_ros
    source_workspace
    ros2 launch spatibot_gui spatibot_gui_demo.launch.py
}

# Function to run GUI real
run_gui_real() {
    print_status "Starting SpatiBot GUI Real..."
    source_ros
    source_workspace
    ros2 launch spatibot_gui spatibot_gui_real.launch.py
}

# Function to run robot API node
run_robot_api() {
    print_status "Starting Robot API Node..."
    source_ros
    source_workspace
    ros2 run spatibot_gui robot_api_node
}

# Function to run OAK-D camera
run_oakd() {
    print_status "Starting OAK-D Camera..."
    source_ros
    source_workspace
    ros2 launch spatibot_oakd oakd_with_turtlebot.launch.py
}

# Function to run CLI
run_cli() {
    print_status "Starting Robot CLI..."
    source_ros
    source_workspace
    ros2 run spatibot_gui robot_cli
}

# Function to run webserver
run_webserver() {
    print_status "Starting Robot Web Server..."
    source_ros
    source_workspace
    cd /workspace/src/robot-remote-control && python3 run_robot_server.py --port 8080
}

# Function to run OAK-D node only
run_oakd_node() {
    print_status "Starting OAK-D Node..."
    source_ros
    source_workspace
    ros2 run spatibot_oakd oakd_node
}

# Function to source ROS environment
source_ros_env() {
    print_status "Sourcing ROS environment..."
    source_ros
    print_success "ROS environment sourced"
}

# Function to source workspace environment
source_workspace_env() {
    print_status "Sourcing workspace environment..."
    source_ros
    source_workspace
    print_success "Workspace environment sourced"
}

# Function to source both environments
source_all() {
    print_status "Sourcing ROS and workspace environments..."
    source_ros
    source_workspace
    print_success "All environments sourced"
}

# Function to source environments for current shell (to be used with 'source' command)
source_for_shell() {
    echo "Sourcing ROS and workspace environments for current shell..."
    source /opt/ros/humble/setup.bash
    if [ -f /workspace/install/setup.bash ]; then
        source /workspace/install/setup.bash
    fi
    echo "Environments sourced successfully!"
    echo "You can now use: ros2 topic list"
}

# Function to source ROS2 environment (for direct sourcing)
source_ros2() {
    source /opt/ros/humble/setup.bash
    if [ -f /workspace/install/setup.bash ]; then
        source /workspace/install/setup.bash
    fi
}

# Function to show help
show_help() {
    echo "SpatiBot ROS Development Script"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build       - Build SpatiBot packages (spatibot_interfaces, spatibot_gui, spatibot_oakd)"
    echo "  build-all   - Build all packages in workspace"
    echo "  clean       - Clean build directories"
    echo "  source      - Source ROS and workspace environments"
    echo "  source-ros  - Source ROS environment only"
    echo "  source-ws   - Source workspace environment only"
    echo "  source-shell- Source environments for current shell (use: source scripts/ros.sh && source_for_shell)"
    echo "  gui-demo    - Run SpatiBot GUI in demo mode"
    echo "  gui-real    - Run SpatiBot GUI in real mode"
    echo "  api         - Run Robot API Node"
    echo "  oakd        - Run OAK-D camera with TurtleBot"
    echo "  oakd-node   - Run OAK-D node only"
    echo "  cli         - Run Robot CLI"
    echo "  webserver   - Run Robot Web Server"
    echo "  help        - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  $0 gui-demo"
    echo "  $0 api"
    echo "  $0 webserver"
}

# Main script logic
case "${1:-help}" in
    "build")
        build_spatibot
        ;;
    "build-all")
        build_all
        ;;
    "clean")
        clean_build
        ;;
    "gui-demo")
        run_gui_demo
        ;;
    "gui-real")
        run_gui_real
        ;;
    "api")
        run_robot_api
        ;;
    "oakd")
        run_oakd
        ;;
    "cli")
        run_cli
        ;;
    "webserver")
        run_webserver
        ;;
    "oakd-node")
        run_oakd_node
        ;;
    "source")
        source_all
        ;;
    "source-ros")
        source_ros_env
        ;;
    "source-ws")
        source_workspace_env
        ;;
    "source-shell")
        source_for_shell
        ;;
    "help"|*)
        show_help
        ;;
esac
