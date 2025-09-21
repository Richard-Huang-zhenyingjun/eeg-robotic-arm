#!/usr/bin/env python3
"""Keyboard Control Driver - Week 3 Day 1"""

import sys
import os

# Add robot path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'robot'))

from arm_driver import ArmDriver

class KeyboardControlDriver:
    def __init__(self):
        self.arm_driver = ArmDriver()
        self.key_mappings = {
            'left': 'L',
            'right': 'R', 
            'up': 'U',
            'down': 'D',
            'g': 'G'
        }
        print("KeyboardControlDriver ready")
    
    def test_all_keys(self):
        print("Testing all key mappings:")
        for key, cmd in self.key_mappings.items():
            print(f"  {key} -> {cmd}: ", end="")
            success = self.arm_driver.process_command(cmd)
            print("✅" if success else "❌")
        
        print(f"Final angles: {self.arm_driver.joint_angles}")
        print(f"Gripper: {'open' if self.arm_driver.gripper_open else 'closed'}")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        print("Testing keyboard control...")
        kbd = KeyboardControlDriver()
        kbd.test_all_keys()
    else:
        print("Keyboard control module loaded")
