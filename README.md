
cat > README.md << 'EOF'
# EEG-Controlled Robotic Arm

## Overview
This project builds an MVP system where EEG brain signals are decoded in real time to control a robotic arm.  
Goal: Demonstrate a closed-loop Brain-Computer Interface (BCI) using SSVEP signals for simple robotic manipulation.

## Hardware (Shopping List)
- OpenBCI Cyton Board (8-channel) + EEG cap, electrode gel, wipes
- 4x MG996R servo motors
- Arduino Uno/Nano + PCA9685 servo driver
- 5V 10A power supply
- Big red E-stop button
- Breadboard, wires, jumper cables, 3D-printed arm frame

## Software Stack
- **Python 3.11** (signal processing, decoding, UI)
- **ROS 2 Humble Hawksbill** (middleware, pub/sub messaging)
- **Arduino IDE** (firmware for servo control)
- **Git + GitHub** for version control

## Repo Structure
## Quick Start
1. Clone the repository
2. Install dependencies (see docs/setup.md)
3. Configure your hardware (see docs/hardware_setup.md)
4. Run the system (see scripts/run_system.sh)

## Development
- Main branch: Stable releases only
- Develop branch: Integration branch
- Feature branches: feature/your-feature-name

## License
MIT License (see LICENSE file)

## Contributors
- Your Name (@your-github-username)
