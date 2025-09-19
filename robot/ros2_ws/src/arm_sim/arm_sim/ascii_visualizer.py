#!/usr/bin/env python3
"""ASCII art visualization of robot arm"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import os
import math

class ASCIIVisualizer(Node):
    def __init__(self):
        super().__init__('ascii_visualizer')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.callback, 10)
        self.positions = [0, 0, 0, 0]
        
    def callback(self, msg):
        if len(msg.position) >= 4:
            self.positions = list(msg.position[:4])
            self.draw_robot()
    
    def draw_robot(self):
        os.system('clear')
        print("=" * 60)
        print("   4-DOF ROBOT ARM VISUALIZATION (ASCII)")
        print("=" * 60)
        
        # Simple side view representation
        j1 = self.positions[0]  # Base rotation (we'll show as angle)
        j2 = self.positions[1]  # Shoulder
        j3 = self.positions[2]  # Elbow
        j4 = self.positions[3]  # Wrist
        
        # Draw base
        print(f"\n   Base (J1): {j1:.2f} rad")
        print("      [===]")
        print("       |")
        
        # Draw link 1 (shoulder)
        angle2 = int(10 + j2 * 10)
        print(f"   Shoulder (J2): {j2:.2f} rad")
        print("       " + " " * max(0, angle2) + "/")
        print("      " + " " * max(0, angle2) + "/")
        
        # Draw link 2 (elbow)
        angle3 = int(10 + j3 * 10)
        print(f"   Elbow (J3): {j3:.2f} rad")
        print("     " + " " * max(0, angle3) + "/")
        print("    " + " " * max(0, angle3) + "/")
        
        # Draw end effector
        print(f"   Wrist (J4): {j4:.2f} rad")
        print("   " + " " * max(0, angle3) + "[]")
        
        print("\n" + "=" * 60)
        print("Use keyboard_control in another terminal to move joints")

def main(args=None):
    rclpy.init(args=args)
    node = ASCIIVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
