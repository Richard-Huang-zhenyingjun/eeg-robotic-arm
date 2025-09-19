
#!/usr/bin/env python3
"""
Robot Arm Driver - Command to Angles
Week 2, Day 3 - EEG Robotic Arm Project

This module provides the ArmDriver class that translates high-level commands
(L, R, G) into joint angle updates for the 4-DOF robotic arm simulation.

Commands:
- L: Rotate base joint +10°
- R: Rotate base joint -10°
- G: Toggle gripper open/close

Integration with sim/arm_sim.py for real-time visualization.
"""

import sys
import os
from typing import List, Optional, Callable, Dict, Any
from datetime import datetime
import logging

# Add sim directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

try:
    from arm_sim import Arm4DOF
except ImportError:
    print("Warning: Could not import Arm4DOF. Make sure sim/arm_sim.py exists.")
    Arm4DOF = None


class ArmDriver:
    """
    Robot Arm Driver that converts commands to joint angle updates.
    
    Manages the state of a 4-DOF robotic arm and processes movement commands
    to update joint positions and gripper state.
    """
    
    def __init__(self, initial_angles: List[float] = None):
        """
        Initialize the arm driver.
        
        Args:
            initial_angles: Initial joint angles [J1, J2, J3, J4] in degrees
        """
        # Joint angles (degrees): [Base, Shoulder, Elbow, Wrist]
        self.joint_angles = initial_angles or [0.0, 0.0, 0.0, 0.0]
        
        # Gripper state
        self.gripper_open = True
        
        # Movement increments
        self.base_increment = 10.0  # degrees per L/R command
        
        # Joint limits (degrees)
        self.joint_limits = [
            (-180, 180),  # J1: Base rotation
            (-90, 90),    # J2: Shoulder pitch
            (-135, 135),  # J3: Elbow pitch
            (-90, 90)     # J4: Wrist pitch
        ]
        
        # Command history
        self.command_history = []
        
        # Setup logging
        self._setup_logging()
        
        # Command mapping
        self.command_map = {
            'L': self._handle_left_command,
            'R': self._handle_right_command,
            'G': self._handle_grip_command
        }
        
        # Callback for external updates (e.g., visualization)
        self.update_callback: Optional[Callable] = None
        
        self._logger.info("ArmDriver initialized with angles: {}°".format(
            [f"{a:.1f}" for a in self.joint_angles]))
    
    def _setup_logging(self):
        """Setup logging for the arm driver."""
        self._logger = logging.getLogger('ArmDriver')
        self._logger.setLevel(logging.INFO)
        
        # Create console handler if not already exists
        if not self._logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s - %(levelname)s: %(message)s',
                datefmt='%H:%M:%S'
            )
            handler.setFormatter(formatter)
            self._logger.addHandler(handler)
    
    def set_update_callback(self, callback: Callable):
        """
        Set callback function to be called when joint angles update.
        
        Args:
            callback: Function to call with updated angles
        """
        self.update_callback = callback
        self._logger.info("Update callback registered")
    
    def process_command(self, command: str) -> bool:
        """
        Process a single command and update joint angles.
        
        Args:
            command: Command string ('L', 'R', 'G', or variations)
            
        Returns:
            True if command was processed successfully, False otherwise
        """
        # Normalize command (uppercase, strip whitespace)
        normalized_cmd = command.upper().strip()
        
        # Log command
        self._logger.info(f"Processing command: '{command}' -> '{normalized_cmd}'")
        
        # Record command in history
        self.command_history.append({
            'timestamp': datetime.now(),
            'original_command': command,
            'normalized_command': normalized_cmd,
            'angles_before': self.joint_angles.copy(),
            'gripper_before': self.gripper_open
        })
        
        # Execute command
        if normalized_cmd in self.command_map:
            try:
                success = self.command_map[normalized_cmd]()
                
                # Update history with results
                self.command_history[-1].update({
                    'success': success,
                    'angles_after': self.joint_angles.copy(),
                    'gripper_after': self.gripper_open
                })
                
                # Trigger update callback
                if success and self.update_callback:
                    self.update_callback(self.joint_angles)
                
                return success
                
            except Exception as e:
                self._logger.error(f"Error executing command '{normalized_cmd}': {e}")
                self.command_history[-1]['success'] = False
                self.command_history[-1]['error'] = str(e)
                return False
        else:
            self._logger.warning(f"Unknown command: '{normalized_cmd}'")
            self.command_history[-1]['success'] = False
            self.command_history[-1]['error'] = f"Unknown command: {normalized_cmd}"
            return False
    
    def _handle_left_command(self) -> bool:
        """Handle left rotation command (L)."""
        old_angle = self.joint_angles[0]
        new_angle = old_angle + self.base_increment
        
        # Check limits
        min_angle, max_angle = self.joint_limits[0]
        if new_angle > max_angle:
            self._logger.warning(f"Left command would exceed limit ({new_angle}° > {max_angle}°)")
            new_angle = max_angle
        
        # Update angle
        self.joint_angles[0] = new_angle
        
        self._logger.info(f"🔄 Left rotation: {old_angle:.1f}° → {new_angle:.1f}°")
        return True
    
    def _handle_right_command(self) -> bool:
        """Handle right rotation command (R)."""
        old_angle = self.joint_angles[0]
        new_angle = old_angle - self.base_increment
        
        # Check limits
        min_angle, max_angle = self.joint_limits[0]
        if new_angle < min_angle:
            self._logger.warning(f"Right command would exceed limit ({new_angle}° < {min_angle}°)")
            new_angle = min_angle
        
        # Update angle
        self.joint_angles[0] = new_angle
        
        self._logger.info(f"🔄 Right rotation: {old_angle:.1f}° → {new_angle:.1f}°")
        return True
    
    def _handle_grip_command(self) -> bool:
        """Handle gripper toggle command (G)."""
        old_state = "open" if self.gripper_open else "closed"
        self.gripper_open = not self.gripper_open
        new_state = "open" if self.gripper_open else "closed"
        
        self._logger.info(f"🤖 Gripper: {old_state} → {new_state}")
        return True
    
    def process_command_sequence(self, commands: List[str]) -> Dict[str, Any]:
        """
        Process a sequence of commands.
        
        Args:
            commands: List of command strings
            
        Returns:
            Dictionary with processing results
        """
        results = {
            'total_commands': len(commands),
            'successful_commands': 0,
            'failed_commands': 0,
            'final_angles': None,
            'final_gripper_state': None,
            'processing_log': []
        }
        
        self._logger.info(f"Processing command sequence: {commands}")
        
        for i, cmd in enumerate(commands):
            success = self.process_command(cmd)
            
            results['processing_log'].append({
                'command_index': i,
                'command': cmd,
                'success': success,
                'angles_after': self.joint_angles.copy(),
                'gripper_after': self.gripper_open
            })
            
            if success:
                results['successful_commands'] += 1
            else:
                results['failed_commands'] += 1
        
        # Final state
        results['final_angles'] = self.joint_angles.copy()
        results['final_gripper_state'] = self.gripper_open
        
        self._logger.info(f"Sequence complete: {results['successful_commands']}/{results['total_commands']} successful")
        
        return results
    
    def get_current_state(self) -> Dict[str, Any]:
        """Get current arm state."""
        return {
            'joint_angles': self.joint_angles.copy(),
            'gripper_open': self.gripper_open,
            'total_commands': len(self.command_history),
            'last_command': self.command_history[-1] if self.command_history else None
        }
    
    def reset_to_home(self):
        """Reset arm to home position."""
        old_angles = self.joint_angles.copy()
        old_gripper = self.gripper_open
        
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.gripper_open = True
        
        self._logger.info(f"🏠 Reset to home: {old_angles} → {self.joint_angles}, gripper → open")
        
        # Trigger update callback
        if self.update_callback:
            self.update_callback(self.joint_angles)
    
    def set_joint_angles(self, angles: List[float], check_limits: bool = True) -> bool:
        """
        Directly set joint angles.
        
        Args:
            angles: List of 4 joint angles in degrees
            check_limits: Whether to enforce joint limits
            
        Returns:
            True if successful, False otherwise
        """
        if len(angles) != 4:
            self._logger.error(f"Expected 4 angles, got {len(angles)}")
            return False
        
        if check_limits:
            for i, angle in enumerate(angles):
                min_angle, max_angle = self.joint_limits[i]
                if angle < min_angle or angle > max_angle:
                    self._logger.error(f"Joint {i+1} angle {angle}° exceeds limits [{min_angle}, {max_angle}]")
                    return False
        
        old_angles = self.joint_angles.copy()
        self.joint_angles = angles.copy()
        
        self._logger.info(f"Angles updated: {old_angles} → {self.joint_angles}")
        
        # Trigger update callback
        if self.update_callback:
            self.update_callback(self.joint_angles)
        
        return True
    
    def get_command_history(self) -> List[Dict]:
        """Get command processing history."""
        return self.command_history.copy()
    
    def clear_history(self):
        """Clear command history."""
        self.command_history.clear()
        self._logger.info("Command history cleared")


class ArmDriverWithVisualization:
    """
    Integrated arm driver with real-time visualization.
    
    Combines ArmDriver with Arm4DOF simulation for live updates.
    """
    
    def __init__(self, initial_angles: List[float] = None):
        """
        Initialize integrated driver with visualization.
        
        Args:
            initial_angles: Initial joint angles in degrees
        """
        # Create driver
        self.driver = ArmDriver(initial_angles)
        
        # Create visualization (if available)
        self.visualizer = None
        if Arm4DOF is not None:
            self.visualizer = Arm4DOF()
            self.visualizer.set_joint_angles(self.driver.joint_angles)
            
            # Set up callback to update visualization
            self.driver.set_update_callback(self._update_visualization)
        else:
            print("Warning: Visualization not available (Arm4DOF not imported)")
    
    def _update_visualization(self, angles: List[float]):
        """Update the visualization with new angles."""
        if self.visualizer:
            self.visualizer.set_joint_angles(angles)
            if hasattr(self.visualizer, 'fig') and self.visualizer.fig:
                self.visualizer.update_plot()
    
    def process_command(self, command: str) -> bool:
        """Process command and update visualization."""
        return self.driver.process_command(command)
    
    def start_interactive_mode(self):
        """Start interactive command mode with visualization."""
        if not self.visualizer:
            print("Error: Visualization not available")
            return
        
        # Setup visualization
        self.visualizer.setup_plot()
        self.visualizer.update_plot()
        
        print("\n" + "="*60)
        print("INTERACTIVE ARM DRIVER WITH VISUALIZATION")
        print("="*60)
        print("Commands:")
        print("  L - Rotate base left (+10°)")
        print("  R - Rotate base right (-10°)")
        print("  G - Toggle gripper")
        print("  RESET - Return to home position")
        print("  STATUS - Show current state")
        print("  Q - Quit")
        print("="*60)
        
        # Show plot
        import matplotlib.pyplot as plt
        plt.ion()  # Interactive mode
        plt.show()
        
        while True:
            try:
                command = input("\nEnter command: ").strip().upper()
                
                if command == 'Q':
                    break
                elif command == 'RESET':
                    self.driver.reset_to_home()
                    self._update_visualization(self.driver.joint_angles)
                elif command == 'STATUS':
                    state = self.driver.get_current_state()
                    print(f"Joint angles: {[f'{a:.1f}°' for a in state['joint_angles']]}")
                    print(f"Gripper: {'open' if state['gripper_open'] else 'closed'}")
                    print(f"Total commands: {state['total_commands']}")
                elif command in ['L', 'R', 'G']:
                    success = self.driver.process_command(command)
                    if success:
                        # Force plot update
                        plt.draw()
                        plt.pause(0.1)
                    else:
                        print("Command failed!")
                else:
                    print(f"Unknown command: {command}")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
        
        plt.ioff()  # Turn off interactive mode
        print("Interactive mode ended.")
    
    def demo_command_sequence(self):
        """Demonstrate with a predefined command sequence."""
        if not self.visualizer:
            print("Error: Visualization not available")
            return
        
        # Setup visualization
        self.visualizer.setup_plot()
        
        # Demo sequence
        commands = ['L', 'L', 'R', 'G', 'L', 'G', 'R', 'R', 'R']
        
        print(f"\nDemo: Processing command sequence: {commands}")
        
        # Process with delays for visualization
        import matplotlib.pyplot as plt
        import time
        
        plt.ion()
        plt.show()
        
        for i, cmd in enumerate(commands):
            print(f"Step {i+1}: Command '{cmd}'")
            success = self.driver.process_command(cmd)
            
            if success:
                plt.draw()
                plt.pause(1.0)  # 1 second delay between commands
            else:
                print(f"  Failed to process command '{cmd}'")
        
        print("\nDemo complete!")
        print(f"Final state: {self.driver.get_current_state()}")
        
        plt.ioff()


def test_arm_driver():
    """Test the ArmDriver class functionality."""
    print("="*60)
    print("TESTING ARM DRIVER")
    print("="*60)
    
    # Create driver
    driver = ArmDriver()
    
    # Test individual commands
    test_commands = ['L', 'L', 'R', 'G', 'l', 'r', 'g', 'INVALID']
    
    print("\nTesting individual commands:")
    for cmd in test_commands:
        print(f"\nCommand: '{cmd}'")
        success = driver.process_command(cmd)
        state = driver.get_current_state()
        print(f"  Success: {success}")
        print(f"  Angles: {[f'{a:.1f}°' for a in state['joint_angles']]}")
        print(f"  Gripper: {'open' if state['gripper_open'] else 'closed'}")
    
    # Test command sequence
    print(f"\nTesting command sequence:")
    sequence = ['L', 'L', 'L', 'R', 'G']
    results = driver.process_command_sequence(sequence)
    
    print(f"Results: {results['successful_commands']}/{results['total_commands']} successful")
    print(f"Final angles: {[f'{a:.1f}°' for a in results['final_angles']]}")
    print(f"Final gripper: {'open' if results['final_gripper_state'] else 'closed'}")
    
    # Test limits
    print(f"\nTesting joint limits:")
    driver.reset_to_home()
    
    # Try to exceed limits
    for _ in range(20):  # Should hit limit before this
        driver.process_command('L')
    
    print(f"After 20 L commands: {driver.joint_angles[0]:.1f}°")
    
    print("\n✅ Arm driver testing complete!")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        # Run tests
        test_arm_driver()
    elif len(sys.argv) > 1 and sys.argv[1] == 'interactive':
        # Run interactive mode
        integrated = ArmDriverWithVisualization()
        integrated.start_interactive_mode()
    elif len(sys.argv) > 1 and sys.argv[1] == 'demo':
        # Run demo
        integrated = ArmDriverWithVisualization()
        integrated.demo_command_sequence()
    else:
        print("Usage:")
        print("  python arm_driver.py test        # Run tests")
        print("  python arm_driver.py interactive # Interactive mode")
        print("  python arm_driver.py demo        # Run demo sequence")


