
Documentation Images
This directory contains screenshots and visual documentation for the EEG Robotic Arm project.
Week 2 - Arm Simulation Screenshots
System Architecture
architecture_diagram.png - System component relationships
data_flow_diagram.png - Command processing pipeline
User Interface
interactive_mode_startup.png - Initial system startup screen
command_prompt.png - Interactive command interface
help_screen.png - Available commands display
3D Visualization
arm_home_position.png - Arm at home position (0°, 0°, 0°, 0°)
arm_left_rotation.png - Base rotated left (+30°)
arm_right_rotation.png - Base rotated right (-30°)
gripper_closed.png - Arm with gripper in closed state
joint_limits_demo.png - Arm at maximum joint limits
Command Demonstrations
demo_sequence.gif - Animated demo of L-L-R-G command sequence
limit_testing.gif - Joint limit enforcement demonstration
reset_functionality.gif - System reset to home position
Status and Debug Views
status_display.png - System status information
debug_mode.png - Debug mode output
error_handling.png - Error message examples
Integration Views
mockserial_output.png - MockSerial command logging
armdriver_state.png - ArmDriver joint angle updates
visualization_update.png - Real-time visualization updates
How to Capture Screenshots
For Terminal Output
# macOS
# Cmd + Shift + 4, then select terminal window

# Linux 
# gnome-screenshot -w

# Windows
# Alt + PrtScn

For 3D Visualization
# Run the system and capture matplotlib windows
python3 scripts/run_sim_arm.py

# Use system screenshot tools to capture the 3D plot window

For Creating GIFs
# Install screen recording software:
# macOS: QuickTime Player or LICEcap
# Linux: Byzanz or Peek  
# Windows: ScreenToGif

# Record 5-10 second clips of:
# - Command sequences
# - Joint limit testing
# - Reset functionality

Planned Screenshots
The following screenshots should be captured during system demonstration:
1. System Startup (startup_sequence.png)
Terminal showing initialization messages
3D visualization window opening
All components successfully loaded
2. Interactive Mode (interactive_demo.png)
Command prompt ready for input
3D arm visualization visible
Clean, professional interface
3. Command Processing (command_processing.png)
User typing 'L' command
Serial response displayed
Joint angle update shown
3D visualization updated
4. Status Information (status_display.png)
Output of STATUS command
Current joint angles
Gripper state
Command count
System health indicators
5. Joint Limits (joint_limits.png)
Arm at +90° limit
Warning message displayed
Visual indication of limit reached
6. System Architecture (architecture_overview.png)
Code structure visualization
Component relationships
Data flow arrows
Screenshot Standards
Technical Requirements
Resolution: Minimum 1920x1080
Format: PNG for static images, GIF for animations
Quality: High quality, clear text visibility
Annotations: Add callouts and labels where helpful
Content Guidelines
Show realistic usage scenarios
Include both success and error cases
Demonstrate key features clearly
Keep consistent visual style
File Naming Convention
category_description_version.extension

Examples:
- ui_startup_screen_v1.png
- demo_command_sequence_v2.gif
- debug_error_handling_v1.png
- architecture_system_overview_v1.png

Usage in Documentation
Markdown Integration
![System Architecture](images/architecture_system_overview_v1.png)
*Figure 1: Overall system architecture showing component relationships*

README Integration
## Quick Demo

![Demo GIF](docs/images/demo_command_sequence_v2.gif)

See the arm respond to L, R, and G commands in real-time!

Documentation Links
Reference images in arm_simulation.md
Include in week2_log.md
Use in project README for visual appeal
Future Additions
Week 3 Screenshots
ROS2 integration interface
Keyboard control demonstration
Multi-node communication
Week 4 Screenshots
BCI signal processing
Real-time EEG data display
Brain-computer interface demo
Week 5+ Screenshots
Complete system integration
User testing sessions
Performance metrics
Hardware transition (if applicable)
Maintenance
Regular Updates
Update screenshots when UI changes
Maintain consistency across versions
Archive old versions for reference
Document changes in git commits
Quality Assurance
Review images for clarity
Ensure text is readable
Verify technical accuracy
Test image links in documentation


