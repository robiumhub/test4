# Generated Dockerfile for Project Configuration
# Project: Project Configuration
# Type: custom
# Generated: 2025-08-17T17:21:03.285Z

FROM ros:humble-ros-base-jammy

# Configure RMW implementation
ENV RMW_IMPLEMENTATION=cyclonedds

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

# Set environment variables
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=cyclonedds

# Expose ports


# Copy application code
COPY . .

# Set default command
CMD bash
