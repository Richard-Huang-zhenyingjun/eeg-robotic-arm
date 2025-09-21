
#!/usr/bin/env python3
"""
ROS2 Joint State Publisher Node
Week 3, Day 3 - EEG Robotic Arm Project

Publishes joint states from the simulated robotic arm to ROS2.
Integrates with keyboard control system to provide real-time joint state updates.
"""

import sys
import os
import math
import threading
import time
from typing import List, Optional

# Add paths for project imports
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, '..', '..', '..')
sys.path.insert(0, os.path.join(project_root, 'sim'))
sys.path.insert(0, os.path.join(project_root, 'robot'))
sys.path.insert(0, os.path.join(project_root, 'scripts'))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose

try:
    from arm_driver import ArmDriver
    from log_commands import CommandLoggerHook
    arm_driver_available = True
except ImportError as e:
    print(f"⚠️  Arm driver not available: {e}")
    arm_driver_available = False
    ArmDriver = None
    CommandLoggerHook = None


class JointStatePublisher(Node):
    """
    ROS2 node that publishes joint states from the robotic arm simulation.
    
    Publishes sensor_msgs/JointState messages with current joint positions,
    velocities, and efforts for visualization in RViz and other ROS2 tools.
    """
    
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'gripper'])
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.joint_names = self.get_parameter('joint_names').value
        
        # Initialize arm driver
        if arm_driver_available:
            self.arm_driver = ArmDriver()
            self.logger_hook = CommandLoggerHook()
            self.get_logger().info("✅ Arm driver initialized")
        else:
            self.arm_driver = None
            self.logger_hook = None
            self.get_logger().warning("⚠️  Arm driver not available - using dummy data")
        
        # Joint state tracking
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_velocities = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_efforts = [0.0, 0.0, 0.0, 0.0]
        self.last_update_time = time.time()
        
        # Create QoS profile for joint states
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile
        )
        
        # Subscribers for commands
        self.command_sub = self.create_subscription(
            String,
            '/arm_commands',
            self.command_callback,
            10
        )
        
        # Timer for publishing joint states
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_joint_states
        )
        
        # Thread safety
        self.lock = threading.Lock()
        
        self.get_logger().info(f"🤖 Joint State Publisher started")
        self.get_logger().info(f"   Publishing at {self.publish_rate} Hz")
        self.get_logger().info(f"   Joint names: {self.joint_names}")
        self.get_logger().info(f"   Subscribed to: /arm_commands")
        self.get_logger().info(f"   Publishing to: /joint_states")
    
    def degrees_to_radians(self, degrees: float) -> float:
        """Convert degrees to radians."""
        return degrees * math.pi / 180.0
    
    def update_joint_positions(self, joint_angles: List[float], gripper_open: bool = True):
        """
        Update internal joint positions from arm driver.
        
        Args:
            joint_angles: List of joint angles in degrees [J1, J2, J3, J4]
            gripper_open: Whether gripper is open (True) or closed (False)
        """
        with self.lock:
            # Convert joint angles from degrees to radians
            self.current_joint_positions[0] = self.degrees_to_radians(joint_angles[0])  # Base
            self.current_joint_positions[1] = self.degrees_to_radians(joint_angles[1])  # Shoulder
            self.current_joint_positions[2] = self.degrees_to_radians(joint_angles[2])  # Elbow
            
            # Gripper position: 0.0 = closed, 0.04 = open (in meters)
            self.current_joint_positions[3] = 0.04 if gripper_open else 0.0
            
            # Calculate simple velocities (difference from last update)
            current_time = time.time()
            dt = current_time - self.last_update_time
            
            if dt > 0:
                # Simple velocity estimation (for visualization purposes)
                for i in range(len(self.current_joint_velocities)):
                    # Set small velocity for recently moved joints
                    self.current_joint_velocities[i] = 0.1 if abs(joint_angles[i]) > 0.1 else 0.0
            
            self.last_update_time = current_time
    
    def publish_joint_states(self):
        """Publish current joint states."""
        # Get current state from arm driver
        if self.arm_driver:
            with self.lock:
                joint_angles = self.arm_driver.joint_angles.copy()
                gripper_open = self.arm_driver.gripper_open
                self.update_joint_positions(joint_angles, gripper_open)
        else:
            # Dummy data for testing without arm driver
            with self.lock:
                # Create some moving dummy data
                t = time.time()
                self.current_joint_positions[0] = 0.3 * math.sin(t * 0.5)  # Base
                self.current_joint_positions[1] = 0.2 * math.cos(t * 0.7)  # Shoulder
                self.current_joint_positions[2] = 0.1 * math.sin(t * 1.1)  # Elbow
                self.current_joint_positions[3] = 0.04 if int(t) % 4 < 2 else 0.0  # Gripper
        
        # Create and populate JointState message
        joint_state = JointState()
        
        # Header
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = self.frame_id
        
        # Joint data
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions.copy()
        joint_state.velocity = self.current_joint_velocities.copy()
        joint_state.effort = self.current_joint_efforts.copy()
        
        # Publish
        self.joint_state_pub.publish(joint_state)
        
        # Log periodically (every 50 messages = 5 seconds at 10Hz)
        if hasattr(self, '_publish_count'):
            self._publish_count += 1
        else:
            self._publish_count = 1
        
        if self._publish_count % 50 == 0:
            positions_deg = [math.degrees(pos) for pos in self.current_joint_positions[:3]]
            gripper_pos = self.current_joint_positions[3]
            self.get_logger().info(
                f"📊 Joint States: "
                f"Base={positions_deg[0]:.1f}° "
                f"Shoulder={positions_deg[1]:.1f}° "
                f"Elbow={positions_deg[2]:.1f}° "
                f"Gripper={gripper_pos:.3f}m"
            )
    
    def command_callback(self, msg: String):
        """
        Handle incoming command messages.
        
        Args:
            msg: String message containing command (L, R, U, D, G)
        """
        command = msg.data.strip().upper()
        
        self.get_logger().info(f"Received command: {command}")
        
        if self.arm_driver:
            # Process command DIRECTLY through arm driver
            success = self.arm_driver.process_command(command)
            
            if success:
                self.get_logger().info(f"Command {command} executed")
                
                # Log command directly
                if self.logger_hook:
                    self.logger_hook.log_command(
                        command,
                        success=True,
                        metadata={
                            "source": "ros2_topic",
                            "joint_angles": self.arm_driver.joint_angles.copy(),
                            "gripper_open": self.arm_driver.gripper_open
                        }
                    )
            else:
                self.get_logger().warning(f"Command {command} failed")
        else:
            self.get_logger().warning(f"No arm driver - command {command} ignored")
    
    def get_current_state_summary(self) -> dict:
        """Get current state summary for debugging."""
        if self.arm_driver:
            return {
                "joint_angles_deg": self.arm_driver.joint_angles,
                "joint_positions_rad": self.current_joint_positions,
                "gripper_open": self.arm_driver.gripper_open,
                "publish_rate": self.publish_rate,
                "frame_id": self.frame_id
            }
        else:
            return {
                "joint_positions_rad": self.current_joint_positions,
                "publish_rate": self.publish_rate,
                "frame_id": self.frame_id,
                "mode": "dummy_data"
            }


def main(args=None):
    """Main function to run the joint state publisher node."""
    print("🚀 Starting ROS2 Joint State Publisher...")
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and spin the node
        node = JointStatePublisher()
        
        print("✅ Joint State Publisher ready!")
        print("📊 Publishing joint states to /joint_states")
        print("📥 Listening for commands on /arm_commands")
        print("🔄 Use Ctrl+C to shutdown")
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if 'node' in locals():
            try:
                # End logging session if available
                if hasattr(node, 'logger_hook') and node.logger_hook:
                    node.logger_hook.end_session()
                
                node.destroy_node()
            except:
                pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print("👋 Joint State Publisher shutdown complete")


if __name__ == '__main__':
    main()


