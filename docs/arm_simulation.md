
Arm Simulation System Documentation
Overview
The Arm Simulation System is a comprehensive software solution for developing and testing robotic arm control algorithms without physical hardware. Built during Week 2 of the EEG Robotic Arm project, it provides a complete pipeline from command input to visual feedback.
System Architecture
Component Architecture Diagram
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   User Input    │    │  MockSerial     │    │   ArmDriver     │
│                 │    │                 │    │                 │
│ Terminal        │───▶│ • Command       │───▶│ • Parse L/R/G   │
│ Commands        │    │   Processing    │    │ • Update Angles │
│ (L, R, G)       │    │ • Response      │    │ • Enforce       │
│                 │    │   Generation    │    │   Limits        │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ 3D Visualization│◀───│  Integration    │◀───│ Joint Angles    │
│                 │    │   Layer         │    │                 │
│ • Matplotlib    │    │                 │    │ [θ1,θ2,θ3,θ4]  │
│ • Real-time     │    │ run_sim_arm.py  │    │ + Gripper       │
│   Updates       │    │                 │    │   State         │
│ • 4-DOF Display │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘

Data Flow Architecture
User Command (e.g., "L")
        │
        ▼
┌─────────────────────────────────────────────────────────┐
│                MockSerial Layer                         │
│  • Receives: b"L\n"                                     │
│  • Logs: Command history                                │
│  • Returns: b"[MOCK] OK - Left joint moved\n"          │
└─────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────┐
│                ArmDriver Layer                          │
│  • Parses: "L" → base_rotation += 10°                  │
│  • Validates: Check joint limits (-90° to +90°)        │
│  • Updates: joint_angles[0] = new_angle                │
│  • Triggers: update_callback(joint_angles)             │
└─────────────────────────────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────┐
│              Visualization Layer                        │
│  • Receives: [θ1, θ2, θ3, θ4] angles                   │
│  • Calculates: Forward kinematics                       │
│  • Renders: 3D arm position                            │
│  • Updates: Real-time display                          │
└─────────────────────────────────────────────────────────┘

Component Details
1. MockSerial (sim/mock_serial.py)
Purpose: Simulates Arduino serial communication for hardware-independent development.
Key Features:
Drop-in replacement for pySerial Serial class
Command logging and history tracking
Realistic response generation
Error handling and timeout simulation
Command Processing:
# Command → Response mapping
{
    b'L\n': "[MOCK] OK - Left joint moved\n",
    b'R\n': "[MOCK] OK - Right joint moved\n", 
    b'G\n': "[MOCK] OK - Gripper [state]\n",
    b'STATUS\n': "[MOCK] STATUS - [full state]\n",
    b'RESET\n': "[MOCK] OK - Reset to home\n"
}

State Management:
Joint positions tracking
Gripper state (open/closed)
Command count statistics
Timestamp logging
2. ArmDriver (robot/arm_driver.py)
Purpose: Translates high-level commands into joint angle updates with safety constraints.
Command Parsing Logic:
def process_command(self, command: str) -> bool:
    normalized = command.upper().strip()
    
    if normalized == 'L':
        return self._handle_left_command()   # +10° base rotation
    elif normalized == 'R': 
        return self._handle_right_command()  # -10° base rotation
    elif normalized == 'G':
        return self._handle_grip_command()   # Toggle gripper
    else:
        return False  # Invalid command

Joint Limit Enforcement:
Base Joint (J1): -90° to +90° (configurable)
Shoulder Joint (J2): -90° to +90°
Elbow Joint (J3): -135° to +135°
Wrist Joint (J4): -90° to +90°
Safety Features:
Automatic limit clamping
Command validation
State consistency checking
Error logging
3. Arm4DOF Visualization (sim/arm_sim.py)
Purpose: Provides real-time 3D visualization of the robotic arm using matplotlib.
Forward Kinematics Implementation:
def forward_kinematics(self, joint_angles):
    """
    Calculate end-effector position from joint angles
    using Denavit-Hartenberg parameters.
    """
    θ1, θ2, θ3, θ4 = [deg2rad(angle) for angle in joint_angles]
    L0, L1, L2, L3 = self.link_lengths
    
    # Calculate joint positions
    x_coords = [x0, x1, x2, x3, x4]  # Base to end-effector
    y_coords = [y0, y1, y2, y3, y4]
    z_coords = [z0, z1, z2, z3, z4]
    
    return x_coords, y_coords, z_coords

Visualization Features:
Real-time joint position updates
Color-coded links and joints
Angle display in plot title
Interactive 3D viewing
Smooth animation support
4. Integration Layer (scripts/run_sim_arm.py)
Purpose: Orchestrates all components into a cohesive system with user interface.
System Initialization:
Create MockSerial instance
Initialize ArmDriver with custom limits
Setup 3D visualization
Register update callbacks
Start interactive command loop
Command Processing Pipeline:
def process_serial_command(self, command_bytes):
    # Step 1: MockSerial processing
    self.mock_serial.write(command_bytes)
    mock_response = self.mock_serial.readline()
    
    # Step 2: ArmDriver processing  
    command_str = command_bytes.decode().strip()
    success = self.arm_driver.process_command(command_str)
    
    # Step 3: Visualization update (automatic via callback)
    # self._update_visualization() called automatically
    
    return mock_response

Command Reference
Movement Commands
Command
Description
Angle Change
Limits
L
Rotate base left
+10°
-90° to +90°
R
Rotate base right
-10°
-90° to +90°
G
Toggle gripper
N/A
Open ↔ Closed

System Commands
Command
Description
Response
STATUS
Show current state
Joint angles, gripper state, command count
RESET
Return to home position
All joints to 0°, gripper open
LIMITS
Show joint limits
Display limit ranges for all joints
DEBUG
Toggle debug mode
Enable/disable verbose logging
HELP
Show command reference
Display all available commands
Q/QUIT
Exit program
Graceful shutdown

Implementation Details
Joint Angle Representation
Storage Format: List of 4 floating-point values [θ1, θ2, θ3, θ4]
θ1: Base rotation (around Z-axis)
θ2: Shoulder pitch (around Y-axis)
θ3: Elbow pitch (around Y-axis)
θ4: Wrist pitch (around Y-axis)
Units: Degrees (converted to radians for calculations)
Home Position: [0.0, 0.0, 0.0, 0.0]
Error Handling
Command Validation:
Case-insensitive command processing
Whitespace trimming
Unknown command rejection
Comprehensive error logging
Limit Enforcement:
Soft limits with warnings
Hard limits with clamping
User notification of limit violations
State consistency maintenance
System Recovery:
Graceful error handling
System reset capabilities
Component isolation
Diagnostic information
Performance Characteristics
Response Times
Command Processing: ~1-2 ms per command
Visualization Update: ~10-50 ms (depending on complexity)
Complete Pipeline: ~20-100 ms end-to-end
Throughput
Maximum Command Rate: ~100-500 commands/second
Practical Rate: ~10-20 commands/second (with visualization)
Batch Processing: 1000+ commands/second (headless mode)
Memory Usage
Base System: ~10-20 MB
With Visualization: ~50-100 MB
Command History: ~1 KB per 1000 commands
Limitations vs Real Hardware
Current Limitations
Physics Simulation:
❌ No collision detection
❌ No gravity effects
❌ No momentum/inertia modeling
❌ No joint friction
❌ No mechanical backlash
Hardware Realism:
❌ No communication delays
❌ No servo response time
❌ No position feedback errors
❌ No mechanical wear modeling
❌ No power consumption tracking
Control Complexity:
❌ Only discrete angle steps (10°)
❌ No smooth trajectory planning
❌ No velocity/acceleration control
❌ No force feedback
❌ No inverse kinematics
Advantages Over Real Hardware
Development Benefits:
✅ No hardware setup required
✅ Instant reset capability
✅ Perfect repeatability
✅ No wear and tear
✅ Safe limit testing
✅ Unlimited testing cycles
Debugging Features:
✅ Complete state visibility
✅ Command history tracking
✅ Real-time parameter adjustment
✅ Visual debugging aids
✅ Deterministic behavior
Integration Advantages:
✅ Parallel development support
✅ Automated testing capability
✅ Version control friendly
✅ CI/CD pipeline compatible
✅ Multi-developer access
Bridging to Real Hardware
Preparation for Hardware:
Command Interface: MockSerial → Real Serial transition
Timing Considerations: Add realistic delays
Error Handling: Implement hardware fault recovery
Calibration: Add joint offset corrections
Safety: Implement emergency stops
Code Portability:
# Easy hardware transition
if USE_REAL_HARDWARE:
    import serial
    ser = serial.Serial('/dev/ttyUSB0', 115200)
else:
    from sim.mock_serial import MockSerial
    ser = MockSerial()

# Same code works with both
ser.write(b"L\n")
response = ser.readline()

Testing Strategy
Unit Testing
MockSerial: Command processing, response generation
ArmDriver: Angle calculations, limit enforcement
Visualization: Forward kinematics, rendering
Integration: End-to-end command flow
Integration Testing
Command Pipeline: Input → Processing → Output
State Consistency: Multi-component synchronization
Error Propagation: Fault handling across layers
Performance: Throughput and latency measurements
User Acceptance Testing
Interactive Mode: Manual command verification
Demo Mode: Automated sequence validation
Limit Testing: Boundary condition verification
Recovery Testing: Error and reset scenarios
Future Enhancements
Short-term Improvements (Week 3)
Keyboard Control: Real-time key-press handling
ROS2 Integration: Publisher/subscriber architecture
Smoother Animation: Interpolated movement
Additional Commands: More complex movements
Medium-term Features (Week 4-6)
Inverse Kinematics: Target position control
Trajectory Planning: Smooth path generation
Collision Detection: Basic obstacle avoidance
Force Simulation: Gripper force modeling
Long-term Vision (Beyond Week 6)
Physics Engine: Full dynamics simulation
Machine Learning: Neural network control
VR/AR Interface: Immersive interaction
Digital Twin: Real-time hardware mirroring
Configuration Options
Joint Limits Customization
# Custom limit configuration
custom_limits = {
    0: (-90, 90),    # Base: ±90°
    1: (-45, 45),    # Shoulder: ±45°
    2: (-120, 120),  # Elbow: ±120°
    3: (-60, 60)     # Wrist: ±60°
}

arm_system = SimulatedArmSystem(joint_limits=custom_limits)

Visualization Settings
# Visualization customization
visualizer = Arm4DOF(
    link_lengths=[0.5, 1.2, 1.0, 0.8],  # Custom link sizes
    update_rate=30,                       # 30 FPS
    show_workspace=True,                  # Display reachable area
    show_trajectories=True                # Show movement paths
)

System Behavior
# System configuration
system_config = {
    'debug_mode': False,
    'auto_save_history': True,
    'command_timeout': 2.0,
    'visualization_fps': 20,
    'log_level': 'INFO'
}

Troubleshooting Guide
Common Issues
Import Errors:
# Solution: Check Python path
export PYTHONPATH="${PYTHONPATH}:./sim:./robot"
python3 scripts/run_sim_arm.py

Visualization Not Showing:
# Solution: Install GUI backend
pip3 install PyQt5
# or
pip3 install tkinter

Commands Not Responding:
Check command format (include newline)
Verify joint limits
Enable debug mode for diagnostics
Performance Issues:
Reduce visualization frame rate
Use headless mode for batch processing
Clear command history periodically
Debug Commands
# Enable debug mode
system.set_debug_mode(True)

# Check system status
status = system.get_system_status()
print(status)

# View command history
history = system.arm_driver.get_command_history()
for cmd in history[-5:]:  # Last 5 commands
    print(cmd)

Conclusion
The Arm Simulation System provides a robust foundation for robotic arm development and testing. While it has limitations compared to real hardware, it offers significant advantages for algorithm development, team collaboration, and educational purposes. The modular architecture ensures easy extension and hardware integration when ready.
The system successfully demonstrates the complete command-to-visualization pipeline, establishing the groundwork for more advanced features in subsequent development phases.


