#!/usr/bin/env python3
"""
Robot Arm Driver - Command to Angles
Week 2, Day 3 + Week 3, Day 1 - EEG Robotic Arm Project

Enhanced with U/D commands for keyboard control.
"""

import sys
import os
from typing import List, Optional, Callable, Dict, Any
from datetime import datetime
import logging

class ArmDriver:
    """Robot Arm Driver that converts commands to joint angle updates."""
    
    def __init__(self, initial_angles: List[float] = None):
        """Initialize the arm driver."""
        self.joint_angles = initial_angles or [0.0, 0.0, 0.0, 0.0]
        self.gripper_open = True
        self.base_increment = 10.0
        
        self.joint_limits = [
            (-180, 180),  # J1: Base rotation
            (-90, 90),    # J2: Shoulder pitch
            (-135, 135),  # J3: Elbow pitch
            (-90, 90)     # J4: Wrist pitch
        ]
        
        self.command_history = []
        self._setup_logging()
        
        # Command mapping with U/D commands
        self.command_map = {
            'L': self._handle_left_command,
            'R': self._handle_right_command,
            'G': self._handle_grip_command,
            'U': self._handle_up_command,
            'D': self._handle_down_command
        }
        
        self.update_callback: Optional[Callable] = None
        
        self._logger.info("ArmDriver initialized with angles: {}°".format(
            [f"{a:.1f}" for a in self.joint_angles]))
    
    def _setup_logging(self):
        """Setup logging for the arm driver."""
        self._logger = logging.getLogger('ArmDriver')
        self._logger.setLevel(logging.INFO)
        
        if not self._logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s - %(levelname)s: %(message)s',
                datefmt='%H:%M:%S'
            )
            handler.setFormatter(formatter)
            self._logger.addHandler(handler)
    
    def set_update_callback(self, callback: Callable):
        """Set callback function to be called when joint angles update."""
        self.update_callback = callback
        self._logger.info("Update callback registered")
    
    def process_command(self, command: str) -> bool:
        """Process a single command and update joint angles."""
        normalized_cmd = command.upper().strip()
        
        self._logger.info(f"Processing command: '{command}' -> '{normalized_cmd}'")
        
        self.command_history.append({
            'timestamp': datetime.now(),
            'original_command': command,
            'normalized_command': normalized_cmd,
            'angles_before': self.joint_angles.copy(),
            'gripper_before': self.gripper_open
        })
        
        if normalized_cmd in self.command_map:
            try:
                success = self.command_map[normalized_cmd]()
                
                self.command_history[-1].update({
                    'success': success,
                    'angles_after': self.joint_angles.copy(),
                    'gripper_after': self.gripper_open
                })
                
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
        
        min_angle, max_angle = self.joint_limits[0]
        if new_angle > max_angle:
            self._logger.warning(f"Left command would exceed limit ({new_angle}° > {max_angle}°)")
            new_angle = max_angle
        
        self.joint_angles[0] = new_angle
        self._logger.info(f"🔄 Left rotation: {old_angle:.1f}° → {new_angle:.1f}°")
        return True
    
    def _handle_right_command(self) -> bool:
        """Handle right rotation command (R)."""
        old_angle = self.joint_angles[0]
        new_angle = old_angle - self.base_increment
        
        min_angle, max_angle = self.joint_limits[0]
        if new_angle < min_angle:
            self._logger.warning(f"Right command would exceed limit ({new_angle}° < {min_angle}°)")
            new_angle = min_angle
        
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
    
    def _handle_up_command(self) -> bool:
        """Handle up movement command (U) - shoulder joint up."""
        old_angle = self.joint_angles[1]  # Shoulder joint
        new_angle = old_angle + self.base_increment
        
        min_angle, max_angle = self.joint_limits[1]
        if new_angle > max_angle:
            self._logger.warning(f"Up command would exceed limit ({new_angle}° > {max_angle}°)")
            new_angle = max_angle
        
        self.joint_angles[1] = new_angle
        self._logger.info(f"⬆️  Shoulder up: {old_angle:.1f}° → {new_angle:.1f}°")
        return True

    def _handle_down_command(self) -> bool:
        """Handle down movement command (D) - shoulder joint down."""
        old_angle = self.joint_angles[1]  # Shoulder joint  
        new_angle = old_angle - self.base_increment
        
        min_angle, max_angle = self.joint_limits[1]
        if new_angle < min_angle:
            self._logger.warning(f"Down command would exceed limit ({new_angle}° < {min_angle}°)")
            new_angle = min_angle
        
        self.joint_angles[1] = new_angle
        self._logger.info(f"⬇️  Shoulder down: {old_angle:.1f}° → {new_angle:.1f}°")
        return True
    
    def reset_to_home(self):
        """Reset arm to home position."""
        old_angles = self.joint_angles.copy()
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.gripper_open = True
        
        self._logger.info(f"🏠 Reset to home: {old_angles} → {self.joint_angles}")
        
        if self.update_callback:
            self.update_callback(self.joint_angles)
    
    def set_joint_angles(self, angles: List[float], check_limits: bool = True) -> bool:
        """Directly set joint angles."""
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
        
        if self.update_callback:
            self.update_callback(self.joint_angles)
        
        return True
    
    def get_current_state(self) -> Dict[str, Any]:
        """Get current arm state."""
        return {
            'joint_angles': self.joint_angles.copy(),
            'gripper_open': self.gripper_open,
            'total_commands': len(self.command_history),
            'last_command': self.command_history[-1] if self.command_history else None
        }
    
    def get_command_history(self) -> List[Dict]:
        """Get command processing history."""
        return self.command_history.copy()
    
    def clear_history(self):
        """Clear command history."""
        self.command_history.clear()
        self._logger.info("Command history cleared")


if __name__ == "__main__":
    # Test the driver
    driver = ArmDriver()
    print("Testing all commands:")
    for cmd in ['L', 'R', 'U', 'D', 'G']:
        success = driver.process_command(cmd)
        print(f"{cmd}: {'✅' if success else '❌'} - {driver.joint_angles}")
