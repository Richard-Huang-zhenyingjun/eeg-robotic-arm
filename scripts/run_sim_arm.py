
#!/usr/bin/env python3
"""
Complete Arm Simulation Integration
Week 2, Day 4 - EEG Robotic Arm Project

Combines MockSerial, ArmDriver, and arm visualization for a complete
simulated robotic arm system with live command input and visual feedback.

Usage:
    python scripts/run_sim_arm.py

Commands:
    L - Rotate base left (+10°)
    R - Rotate base right (-10°)
    G - Toggle gripper
    STATUS - Show current state
    RESET - Return to home position
    LIMITS - Show/modify joint limits
    DEBUG - Toggle debug mode
    HELP - Show help
    Q/QUIT/EXIT - Quit program
"""

import sys
import os
import time
import threading
from typing import Dict, Any, Optional

# Add paths for imports
script_dir = os.path.dirname(__file__)
project_root = os.path.join(script_dir, '..')
sys.path.insert(0, os.path.join(project_root, 'sim'))
sys.path.insert(0, os.path.join(project_root, 'robot'))

try:
    from mock_serial import MockSerial
    from arm_driver import ArmDriver
    from arm_sim import Arm4DOF
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("Make sure all required files exist:")
    print("  - sim/mock_serial.py")
    print("  - robot/arm_driver.py") 
    print("  - sim/arm_sim.py")
    sys.exit(1)


class SimulatedArmSystem:
    """
    Complete simulated arm system integrating all components.
    
    Combines MockSerial communication, ArmDriver command processing,
    and real-time 3D visualization.
    """
    
    def __init__(self, joint_limits: Optional[Dict[int, tuple]] = None):
        """
        Initialize the complete arm simulation system.
        
        Args:
            joint_limits: Custom joint limits {joint_index: (min, max)}
        """
        print("🚀 Initializing Simulated Arm System...")
        
        # System components
        self.mock_serial = None
        self.arm_driver = None
        self.visualizer = None
        
        # System state
        self.debug_mode = False
        self.running = False
        self.command_count = 0
        
        # Initialize components
        self._initialize_components(joint_limits)
        
        print("✅ System initialization complete!")
    
    def _initialize_components(self, joint_limits: Optional[Dict[int, tuple]] = None):
        """Initialize all system components."""
        
        # 1. Initialize MockSerial
        print("  📡 Setting up mock serial communication...")
        self.mock_serial = MockSerial(port="ARM_SIM", baudrate=115200, timeout=2.0)
        print(f"     Mock serial ready on {self.mock_serial.port}")
        
        # 2. Initialize ArmDriver with custom limits
        print("  🤖 Setting up arm driver...")
        self.arm_driver = ArmDriver()
        
        # Apply custom joint limits if provided
        if joint_limits:
            self._apply_joint_limits(joint_limits)
        else:
            # Default: restrict base joint to -90° → +90° as per requirements
            self.arm_driver.joint_limits[0] = (-90, 90)
            print("     Applied default joint limits: Base ±90°")
        
        print(f"     Joint limits: {self.arm_driver.joint_limits}")
        
        # 3. Initialize Visualization
        print("  📊 Setting up 3D visualization...")
        try:
            self.visualizer = Arm4DOF(link_lengths=[0.5, 1.2, 1.0, 0.8])
            self.visualizer.set_joint_angles(self.arm_driver.joint_angles)
            
            # Set up callback to update visualization when arm moves
            self.arm_driver.set_update_callback(self._update_visualization)
            
            print("     3D visualization ready")
        except Exception as e:
            print(f"     ⚠️  Visualization error: {e}")
            self.visualizer = None
        
        # 4. Setup logging for integration
        self._setup_integration_logging()
    
    def _apply_joint_limits(self, joint_limits: Dict[int, tuple]):
        """Apply custom joint limits."""
        for joint_idx, (min_angle, max_angle) in joint_limits.items():
            if 0 <= joint_idx < len(self.arm_driver.joint_limits):
                old_limits = self.arm_driver.joint_limits[joint_idx]
                self.arm_driver.joint_limits[joint_idx] = (min_angle, max_angle)
                print(f"     Joint {joint_idx+1} limits: {old_limits} → ({min_angle}, {max_angle})")
    
    def _setup_integration_logging(self):
        """Setup logging for the integration system."""
        import logging
        
        self.logger = logging.getLogger('SimulatedArmSystem')
        self.logger.setLevel(logging.INFO)
        
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s: %(message)s',
                datefmt='%H:%M:%S'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
    
    def _update_visualization(self, angles):
        """Update the 3D visualization with new joint angles."""
        if self.visualizer and hasattr(self.visualizer, 'fig') and self.visualizer.fig:
            self.visualizer.set_joint_angles(angles)
            self.visualizer.update_plot()
            
            # Force matplotlib update
            import matplotlib.pyplot as plt
            plt.draw()
            plt.pause(0.01)
    
    def process_serial_command(self, command_bytes: bytes) -> bytes:
        """
        Process command through the complete pipeline:
        MockSerial → ArmDriver → Visualization
        
        Args:
            command_bytes: Command as bytes (e.g., b'L\n')
            
        Returns:
            Response bytes from mock serial
        """
        if self.debug_mode:
            self.logger.info(f"Processing serial command: {command_bytes}")
        
        # Step 1: Send to MockSerial
        self.mock_serial.write(command_bytes)
        mock_response = self.mock_serial.readline()
        
        # Step 2: Extract command and send to ArmDriver
        command_str = command_bytes.decode('utf-8', errors='ignore').strip()
        
        if command_str.upper() in ['L', 'R', 'G']:
            # Process through arm driver
            success = self.arm_driver.process_command(command_str)
            
            if success:
                self.command_count += 1
                if self.debug_mode:
                    state = self.arm_driver.get_current_state()
                    self.logger.info(f"Arm updated: {[f'{a:.1f}°' for a in state['joint_angles']]}")
            else:
                self.logger.warning(f"Arm driver failed to process: {command_str}")
        
        return mock_response
    
    def start_visualization(self):
        """Start the 3D visualization window."""
        if not self.visualizer:
            print("❌ Visualization not available")
            return False
        
        try:
            # Setup the plot
            self.visualizer.setup_plot()
            self.visualizer.update_plot()
            
            # Show the plot
            import matplotlib.pyplot as plt
            plt.ion()  # Interactive mode
            plt.show()
            
            print("✅ 3D visualization started")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start visualization: {e}")
            return False
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get complete system status."""
        arm_state = self.arm_driver.get_current_state()
        
        return {
            'serial_status': 'connected' if self.mock_serial.is_open else 'disconnected',
            'visualization_status': 'active' if self.visualizer else 'unavailable',
            'joint_angles': arm_state['joint_angles'],
            'gripper_state': 'open' if arm_state['gripper_open'] else 'closed',
            'total_commands': self.command_count,
            'joint_limits': self.arm_driver.joint_limits,
            'debug_mode': self.debug_mode
        }
    
    def reset_system(self):
        """Reset the entire system to initial state."""
        print("🔄 Resetting system...")
        
        # Reset arm driver
        self.arm_driver.reset_to_home()
        
        # Clear serial history
        self.mock_serial.clear_history()
        
        # Reset counters
        self.command_count = 0
        
        print("✅ System reset complete")
    
    def set_debug_mode(self, enabled: bool):
        """Enable or disable debug mode."""
        self.debug_mode = enabled
        level = 10 if enabled else 20  # DEBUG vs INFO
        self.logger.setLevel(level)
        self.arm_driver._logger.setLevel(level)
        
        print(f"🔧 Debug mode: {'enabled' if enabled else 'disabled'}")
    
    def shutdown(self):
        """Shutdown the system gracefully."""
        print("🛑 Shutting down system...")
        
        self.running = False
        
        if self.mock_serial:
            self.mock_serial.close()
        
        if self.visualizer:
            import matplotlib.pyplot as plt
            plt.ioff()
            plt.close('all')
        
        print("✅ System shutdown complete")


def print_help():
    """Print help information."""
    print("\n" + "="*60)
    print("SIMULATED ARM SYSTEM - COMMAND REFERENCE")
    print("="*60)
    print("Movement Commands:")
    print("  L        - Rotate base left (+10°)")
    print("  R        - Rotate base right (-10°)")
    print("  G        - Toggle gripper open/close")
    print("")
    print("System Commands:")
    print("  STATUS   - Show current system state")
    print("  RESET    - Reset arm to home position")
    print("  LIMITS   - Show current joint limits")
    print("  DEBUG    - Toggle debug mode on/off")
    print("  HELP     - Show this help message")
    print("  Q/QUIT   - Quit the program")
    print("")
    print("Notes:")
    print("  - Commands are case-insensitive")
    print("  - Base joint limited to ±90° by default")
    print("  - Real-time visualization updates automatically")
    print("="*60)


def run_interactive_mode(arm_system: SimulatedArmSystem):
    """
    Run the interactive command mode.
    
    Args:
        arm_system: The initialized arm system
    """
    print("\n" + "="*60)
    print("🚀 SIMULATED ARM SYSTEM - INTERACTIVE MODE")
    print("="*60)
    print("Type commands and watch the arm move in real-time!")
    print("Type 'HELP' for command reference, 'Q' to quit.")
    print("="*60)
    
    # Start visualization
    viz_started = arm_system.start_visualization()
    if not viz_started:
        print("⚠️  Continuing without visualization...")
    
    arm_system.running = True
    
    # Main command loop
    while arm_system.running:
        try:
            # Get user input
            user_input = input("\n🤖 Enter command: ").strip()
            
            if not user_input:
                continue
            
            command = user_input.upper()
            
            # Handle system commands
            if command in ['Q', 'QUIT', 'EXIT']:
                break
                
            elif command == 'HELP':
                print_help()
                continue
                
            elif command == 'STATUS':
                status = arm_system.get_system_status()
                print(f"\n📊 SYSTEM STATUS:")
                print(f"  Serial: {status['serial_status']}")
                print(f"  Visualization: {status['visualization_status']}")
                print(f"  Joint Angles: {[f'{a:+6.1f}°' for a in status['joint_angles']]}")
                print(f"  Gripper: {status['gripper_state']}")
                print(f"  Commands Processed: {status['total_commands']}")
                print(f"  Debug Mode: {status['debug_mode']}")
                continue
                
            elif command == 'RESET':
                arm_system.reset_system()
                continue
                
            elif command == 'LIMITS':
                limits = arm_system.arm_driver.joint_limits
                print(f"\n📏 JOINT LIMITS:")
                for i, (min_angle, max_angle) in enumerate(limits):
                    print(f"  Joint {i+1}: {min_angle:+6.1f}° to {max_angle:+6.1f}°")
                continue
                
            elif command == 'DEBUG':
                arm_system.set_debug_mode(not arm_system.debug_mode)
                continue
            
            # Handle movement commands
            elif command in ['L', 'R', 'G']:
                print(f"📤 Sending command: {command}")
                
                # Process through complete pipeline
                command_bytes = f"{command}\n".encode()
                response = arm_system.process_serial_command(command_bytes)
                
                # Show response
                response_str = response.decode('utf-8', errors='ignore').strip()
                print(f"📥 Response: {response_str}")
                
                # Show current state
                state = arm_system.get_system_status()
                print(f"📍 Current angle: {state['joint_angles'][0]:+6.1f}°, "
                      f"Gripper: {state['gripper_state']}")
                
            else:
                print(f"❌ Unknown command: '{user_input}'")
                print("   Type 'HELP' for available commands")
        
        except KeyboardInterrupt:
            print("\n🛑 Interrupt received, shutting down...")
            break
            
        except Exception as e:
            print(f"❌ Error: {e}")
            if arm_system.debug_mode:
                import traceback
                traceback.print_exc()
    
    # Cleanup
    arm_system.running = False
    arm_system.shutdown()
    print("\n👋 Goodbye!")


def run_automated_demo(arm_system: SimulatedArmSystem):
    """
    Run an automated demonstration of the system.
    
    Args:
        arm_system: The initialized arm system
    """
    print("\n" + "="*60)
    print("🎬 AUTOMATED DEMONSTRATION")
    print("="*60)
    
    # Start visualization
    if not arm_system.start_visualization():
        print("⚠️  Continuing demo without visualization...")
    
    # Demo sequence
    demo_commands = [
        ('L', 'Move base left (+10°)'),
        ('L', 'Move base left again (+10°)'),
        ('R', 'Move base right (-10°)'),
        ('G', 'Close gripper'),
        ('L', 'Move left with gripper closed'),
        ('L', 'Move left again'),
        ('G', 'Open gripper'),
        ('R', 'Move base right (-10°)'),
        ('R', 'Move base right again (-10°)'),
    ]
    
    print(f"Running {len(demo_commands)} commands with 2-second delays...")
    print("-" * 40)
    
    for i, (cmd, description) in enumerate(demo_commands, 1):
        print(f"\nStep {i}/{len(demo_commands)}: {description}")
        print(f"  Command: {cmd}")
        
        # Process command
        command_bytes = f"{cmd}\n".encode()
        response = arm_system.process_serial_command(command_bytes)
        
        # Show results
        response_str = response.decode('utf-8', errors='ignore').strip()
        state = arm_system.get_system_status()
        
        print(f"  Response: {response_str}")
        print(f"  New angle: {state['joint_angles'][0]:+6.1f}°")
        print(f"  Gripper: {state['gripper_state']}")
        
        # Delay for visualization
        if i < len(demo_commands):  # Don't delay after last command
            print("  ⏳ Waiting 2 seconds...")
            time.sleep(2)
    
    # Final status
    print(f"\n" + "="*40)
    print("🎉 DEMO COMPLETE!")
    final_status = arm_system.get_system_status()
    print(f"Final state: {[f'{a:+6.1f}°' for a in final_status['joint_angles']]}")
    print(f"Total commands: {final_status['total_commands']}")
    print("="*40)
    
    input("\nPress Enter to continue...")


def test_joint_limits(arm_system: SimulatedArmSystem):
    """
    Test joint limit enforcement.
    
    Args:
        arm_system: The initialized arm system
    """
    print("\n" + "="*60)
    print("🔧 TESTING JOINT LIMITS")
    print("="*60)
    
    # Get current limits
    limits = arm_system.arm_driver.joint_limits[0]  # Base joint
    min_angle, max_angle = limits
    
    print(f"Base joint limits: {min_angle}° to {max_angle}°")
    print("Testing limit enforcement...")
    
    # Reset to home
    arm_system.reset_system()
    
    # Test positive limit
    print(f"\n1. Testing positive limit ({max_angle}°):")
    moves_needed = int((max_angle + 5) / 10)  # Go slightly beyond limit
    
    for i in range(moves_needed):
        current_angle = arm_system.arm_driver.joint_angles[0]
        print(f"   Step {i+1}: Current {current_angle:+6.1f}° → ", end="")
        
        arm_system.process_serial_command(b"L\n")
        new_angle = arm_system.arm_driver.joint_angles[0]
        
        print(f"{new_angle:+6.1f}°")
        
        if new_angle == max_angle and current_angle == max_angle:
            print(f"   ✅ Limit reached and enforced at {max_angle}°")
            break
    
    # Reset and test negative limit
    arm_system.reset_system()
    
    print(f"\n2. Testing negative limit ({min_angle}°):")
    moves_needed = int(abs(min_angle - 5) / 10)  # Go slightly beyond limit
    
    for i in range(moves_needed):
        current_angle = arm_system.arm_driver.joint_angles[0]
        print(f"   Step {i+1}: Current {current_angle:+6.1f}° → ", end="")
        
        arm_system.process_serial_command(b"R\n")
        new_angle = arm_system.arm_driver.joint_angles[0]
        
        print(f"{new_angle:+6.1f}°")
        
        if new_angle == min_angle and current_angle == min_angle:
            print(f"   ✅ Limit reached and enforced at {min_angle}°")
            break
    
    print(f"\n✅ Joint limit testing complete!")
    arm_system.reset_system()


def main():
    """Main function."""
    print("="*70)
    print("🤖 SIMULATED ARM SYSTEM - COMPLETE INTEGRATION")
    print("Week 2, Day 4 - EEG Robotic Arm Project")
    print("="*70)
    
    # Parse command line arguments
    mode = 'interactive'  # default
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
    
    try:
        # Initialize the complete system
        # Apply custom joint limits: Base joint restricted to -90° → +90°
        custom_limits = {0: (-90, 90)}  # Base joint only
        arm_system = SimulatedArmSystem(joint_limits=custom_limits)
        
        # Run based on mode
        if mode == 'demo':
            run_automated_demo(arm_system)
        elif mode == 'test':
            test_joint_limits(arm_system)
        elif mode == 'interactive' or mode == '':
            run_interactive_mode(arm_system)
        else:
            print(f"❌ Unknown mode: {mode}")
            print("Available modes: interactive (default), demo, test")
            return 1
        
        return 0
        
    except KeyboardInterrupt:
        print("\n🛑 Program interrupted by user")
        return 1
        
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())


