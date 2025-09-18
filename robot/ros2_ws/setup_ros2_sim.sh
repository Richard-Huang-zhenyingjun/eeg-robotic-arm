#!/bin/bash
# Setup script for ROS2 simulation environment

echo "Setting up ROS2 simulation workspace..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace if src packages exist
if [ -d "src" ]; then
    colcon build --symlink-install
    source install/setup.bash
fi

echo "ROS2 workspace ready!"
