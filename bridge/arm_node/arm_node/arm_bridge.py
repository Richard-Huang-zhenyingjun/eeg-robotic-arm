
#!/usr/bin/env python3
"""
ROS2 Arm Bridge Node
Week 3, Day 3 - EEG Robotic Arm Project

Complete bridge between keyboard control, arm simulation, and ROS2.
Integrates all components for a complete ROS2-enabled robotic arm system.
"""

import sys
import os
import threading
import time
from typing import Optional

# Add paths for project imports
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, '..', '..', '..')
sys.path.insert(0, os.path.join(project_root, 'sim'))
sys.path.insert(0, os.path.join(project_root, 'robot'))
sys.path.insert(0, os.path.join(project_root, 'scripts'))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

try:
    from keyboard_control import KeyboardControlDriver
    from joint_state_publisher import JointStatePublisher
    keyboard_available = True
except ImportError as e:
    print(f"⚠️  Keyboard control not available: {e}")
    keyboard_available = False


class ArmBridge(Node):
    """
    Complete bridge node that integrates keyboard control with ROS2.
    
    Provides both keyboard input and ROS2 topic interfaces for controlling
    the simulated robotic arm.
    """
    
    def __init__(self):
        super().__init__('arm_bridge')
        
        self.get_logger().info("🌉 Initializing Arm Bridge...")
        
        # Initialize keyboard control if available
        self.keyboard_control = None
        if keyboard_available:
            try:
                self.keyboard_control = KeyboardControlDriver()
                self.get_logger().info("✅ Keyboard control initialized")
            except Exception as e:
                self.get_logger().warning(f"⚠️  Keyboard control failed: {e}")
        
        # Create joint state publisher component
        try:
            self.joint_state_publisher = JointStatePublisher()
            self.get_logger().info("✅ Joint state publisher initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Joint state publisher failed: {e}")
            self.joint_state_publisher = None
        
        # Publishers
        self.command_pub = self.create_publisher(
            String,
            '/arm_commands',
            10
        )
        
        # Subscribers
        self.external_command_sub = self.create_subscription(
            String,
            '/external_arm_commands',
            self.external_command_callback,
            10
        )
        
        # Control state
        self.running = True
        self.keyboard_thread = None
        
        self.get_logger().info("🌉 Arm Bridge ready!")
        self.get_logger().info("   📥 Listening: /external_arm_commands")
        self.get_logger().info("   📤 Publishing: /arm_commands")
        self.get_logger().info("   📊 Publishing: /joint_states")
    
    def external_command_callback(self, msg: String):
        """Handle commands from external ROS2 topics."""
        command = msg.data.strip().upper()
        self.get_logger().info(f"📥 External command: {command}")
        
        # Forward to arm_commands topic
        self.publish_command(command)
    
    def publish_command(self, command: str):
        """Publish a command to the /arm_commands topic."""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"📤 Published command: {command}")
    
    def keyboard_control_loop(self):
        """Run keyboard control in a separate thread."""
        if not self.keyboard_control:
            return
        
        self.get_logger().info("🎮 Keyboard control thread started")
        
        # Simple keyboard simulation for testing
        # In real implementation, this would use the keyboard library
        test_commands = ['L', 'L', 'U', 'R', 'D', 'G']
        
        try:
            for i, cmd in enumerate(test_commands):
                if not self.running:
                    break
                
                self.get_logger().info(f"🎮 Simulating keyboard: {cmd}")
                
                # Process through keyboard control
                success = self.keyboard_control.process_key_command(cmd.lower())
                
                if success:
                    # Publish to ROS2 topic
                    self.publish_command(cmd)
                
                # Wait between commands
                time.sleep(2)
        
        except Exception as e:
            self.get_logger().error(f"❌ Keyboard control error: {e}")
        
        self.get_logger().info("🎮 Keyboard control thread ended")
    
    def start_keyboard_control(self):
        """Start keyboard control in a separate thread."""
        if self.keyboard_control and not self.keyboard_thread:
            self.keyboard_thread = threading.Thread(
                target=self.keyboard_control_loop,
                daemon=True
            )
            self.keyboard_thread.start()
    
    def stop_keyboard_control(self):
        """Stop keyboard control."""
        self.running = False
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=1.0)
    
    def shutdown(self):
        """Shutdown the bridge."""
        self.get_logger().info("🛑 Shutting down Arm Bridge...")
        
        self.stop_keyboard_control()
        
        if self.keyboard_control and hasattr(self.keyboard_control, 'end_session'):
            self.keyboard_control.end_session()
        
        self.get_logger().info("👋 Arm Bridge shutdown complete")


def main(args=None):
    """Main function to run the arm bridge node."""
    print("🌉 Starting ROS2 Arm Bridge...")
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create bridge node
        bridge = ArmBridge()
        
        # Start keyboard control
        bridge.start_keyboard_control()
        
        print("✅ Arm Bridge ready!")
        print("🎮 Keyboard control active (simulated)")
        print("📊 Joint states publishing to /joint_states")
        print("🔄 Use Ctrl+C to shutdown")
        
        # Spin the node
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if 'bridge' in locals():
            try:
                bridge.shutdown()
                bridge.destroy_node()
            except:
                pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("👋 Arm Bridge shutdown complete")


if __name__ == '__main__':
    main()


