#!/bin/bash
# Setup script for EEG-Robotic-Arm development

echo "==================================="
echo "EEG-Robotic-Arm Workspace Setup"
echo "==================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}Docker is not running. Please start Docker Desktop.${NC}"
    exit 1
fi

# Check if container exists
if docker ps -a | grep -q ros2-sim; then
    echo -e "${GREEN}Container 'ros2-sim' found${NC}"
    
    # Start container if not running
    if ! docker ps | grep -q ros2-sim; then
        echo "Starting container..."
        docker start ros2-sim
    fi
    
    echo -e "${GREEN}Entering container...${NC}"
    docker exec -it ros2-sim bash -c "cd /workspace && source /opt/ros/humble/setup.bash && bash"
else
    echo -e "${RED}Container 'ros2-sim' not found${NC}"
    echo "Creating new container..."
    
    # Setup X11 (optional)
    if command -v xhost &> /dev/null; then
        IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
        xhost + $IP 2>/dev/null
        
        docker run -it \
          -e DISPLAY=$IP:0 \
          -v /tmp/.X11-unix:/tmp/.X11-unix \
          -v ~/eeg-robotic-arm:/workspace \
          --name ros2-sim \
          saching12/ros-mac-m1:humble-desktop \
          bash -c "cd /workspace && source /opt/ros/humble/setup.bash && bash"
    else
        docker run -it \
          -v ~/eeg-robotic-arm:/workspace \
          --name ros2-sim \
          saching12/ros-mac-m1:humble-desktop \
          bash -c "cd /workspace && source /opt/ros/humble/setup.bash && bash"
    fi
fi
