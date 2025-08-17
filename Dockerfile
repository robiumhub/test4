# Generated Dockerfile for Project Configuration
# Project: Project Configuration
# Type: custom
# Generated: 2025-08-17T17:21:03.285Z

FROM ros:humble

# Configure RMW implementation (CycloneDDS)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set working directory
WORKDIR /workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS and system packages required by SpatiBot
RUN apt-get update && apt-get install -y \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-turtlebot4-description \
    ros-humble-turtlebot4-msgs \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-viz \
    ros-humble-turtlebot4-bringup \
    ros-humble-rviz2 \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-default-generators \
    python3-colcon-common-extensions \
    python3-tk \
    udev \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# UDEV rules for DepthAI devices
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/depthai.rules

# Set environment variables
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/workspace/cyclonedds_pc.xml

# Python packages required by SpatiBot
RUN pip3 install --no-cache-dir --upgrade pip && pip3 install --no-cache-dir \
    numpy==1.24.3 \
    matplotlib==3.7.2 \
    opencv-python==4.8.1.78 \
    pillow==10.0.1 \
    pyyaml==6.0.1 \
    depthai \
    google-generativeai \
    psutil \
    fastapi \
    uvicorn[standard] \
    jinja2

# Set up ROS2 environment and bash aliases for convenience
RUN echo "# ROS2 Environment Setup" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then" >> /root/.bashrc && \
    echo "    source /workspace/install/setup.bash" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc && \
    echo "# SpatiBot Aliases" >> /root/.bashrc && \
    echo "alias spatigui='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch spatibot_gui spatibot_gui_real.launch.py'" >> /root/.bashrc && \
    echo "alias spatiguidemo='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch spatibot_gui spatibot_gui_demo.launch.py'" >> /root/.bashrc && \
    echo "alias build='source /opt/ros/humble/setup.bash && colcon build --packages-select spatibot_interfaces spatibot_gui spatibot_oakd && source install/setup.bash'" >> /root/.bashrc && \
    echo "alias buildclean='rm -rf build/ install/ && source /opt/ros/humble/setup.bash && colcon build --packages-select spatibot_interfaces spatibot_gui spatibot_oakd && source install/setup.bash'" >> /root/.bashrc && \
    echo "alias robotapi='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run spatibot_gui robot_api_node'" >> /root/.bashrc && \
    echo "alias robotcli='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run spatibot_gui robot_cli'" >> /root/.bashrc && \
    echo "alias oakd='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch spatibot_oakd oakd_with_turtlebot.launch.py'" >> /root/.bashrc && \
    echo "alias oakdnode='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run spatibot_oakd oakd_node'" >> /root/.bashrc && \
    echo "alias webserver='source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && cd /workspace/src/robot-remote-control && python3 run_robot_server.py --port 8080'" >> /root/.bashrc

# Expose ports (none required by default)
# Copy application code
COPY . .

# Set default command
CMD bash
