#!/usr/bin/env python3
"""
Enhanced Keyboard Control Driver with Command Logging
Week 3, Day 2 - EEG Robotic Arm Project
"""

import sys
import os
from datetime import datetime

# Add paths for imports
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'robot'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from arm_driver import ArmDriver

# Import command logger
try:
    from log_commands import CommandLoggerHook
    logging_available = True
    print("✅ Command logging available")
except ImportError:
    print("⚠️  Command logging not available")
    logging_available = False
    CommandLoggerHook = None

class KeyboardControlDriver:
    """Enhanced keyboard control driver with logging."""
    
    def __init__(self):
        self.arm_driver = ArmDriver()
        self.key_mappings = {
            'left': 'L',
            'right': 'R', 
            'up': 'U',
            'down': 'D',
            'g': 'G'
        }
        
        # Initialize command logging
        if logging_available:
            self.logger_hook = CommandLoggerHook()
            print("📝 Command logging enabled")
        else:
            self.logger_hook = None
            print("📝 Command logging disabled")
        
        print("KeyboardControlDriver ready with logging")
    
    def process_key_command(self, key_name: str) -> bool:
        """Process a key command and log it."""
        print(f"\n🎮 Processing key: {key_name}")
        
        if key_name not in self.key_mappings:
            print(f"❌ Unknown key: {key_name}")
            
            # Log failed command
            if self.logger_hook:
                self.logger_hook.log_command(f"UNKNOWN_KEY_{key_name}", success=False)
            
            return False
        
        command = self.key_mappings[key_name]
        
        # Process through arm driver
        success = self.arm_driver.process_command(command)
        
        # Log the command
        if self.logger_hook:
            metadata = {
                "key": key_name,
                "joint_angles_after": self.arm_driver.joint_angles.copy(),
                "gripper_open": self.arm_driver.gripper_open
            }
            self.logger_hook.log_command(command, success=success, metadata=metadata)
        
        if success:
            print(f"✅ Command {command} executed successfully")
            print(f"   Angles: {[f'{a:+6.1f}°' for a in self.arm_driver.joint_angles]}")
            print(f"   Gripper: {'open' if self.arm_driver.gripper_open else 'closed'}")
        else:
            print(f"❌ Command {command} failed")
        
        return success
    
    def test_all_keys(self):
        """Test all key mappings with logging."""
        print("🧪 Testing all key mappings with logging...")
        
        for key, cmd in self.key_mappings.items():
            print(f"\n--- Testing {key} -> {cmd} ---")
            success = self.process_key_command(key)
            print(f"Result: {'✅' if success else '❌'}")
        
        # Final state
        print(f"\n📊 Final State:")
        print(f"   Joint angles: {[f'{a:+6.1f}°' for a in self.arm_driver.joint_angles]}")
        print(f"   Gripper: {'open' if self.arm_driver.gripper_open else 'closed'}")
        
        # Show logging summary
        if self.logger_hook:
            summary = self.logger_hook.get_summary()
            print(f"\n📝 Logging Summary:")
            print(f"   Commands logged: {summary['commands_logged']}")
            print(f"   Log file: {summary['current_log_file']}")
    
    def end_session(self):
        """End the logging session."""
        if self.logger_hook:
            self.logger_hook.end_session()

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        print("Testing enhanced keyboard control with logging...")
        
        try:
            kbd = KeyboardControlDriver()
            kbd.test_all_keys()
            kbd.end_session()
            
            print("\n🎉 Test completed! Check data/logs/ for log files.")
            
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted")
        except Exception as e:
            print(f"❌ Test error: {e}")
    else:
        print("Enhanced keyboard control module with logging")
        print("Run with 'test' to test: python keyboard_control_enhanced.py test")
