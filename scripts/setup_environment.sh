#!/bin/bash
# Environment setup script for EEG-Robotic-Arm project

echo "Setting up EEG-Robotic-Arm development environment..."

# Check Python version
python3 --version

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt

echo "Environment setup complete!"
echo "Remember to source ROS2 if using it: source /opt/ros/humble/setup.bash"
