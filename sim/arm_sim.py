#!/usr/bin/env python3
"""
4-DOF Robotic Arm Simulation with Matplotlib Animation
Week 2, Day 1 - EEG Robotic Arm Project

This module simulates a 4-degree-of-freedom robotic arm using forward kinematics
and provides smooth animation using matplotlib's FuncAnimation.

Joints:
- J1: Base rotation (around Z-axis)
- J2: Shoulder pitch (around Y-axis) 
- J3: Elbow pitch (around Y-axis)
- J4: Wrist pitch (around Y-axis)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from typing import List, Tuple


class Arm4DOF:
    """4-DOF robotic arm simulator with forward kinematics."""
    
    def __init__(self, link_lengths: List[float] = None):
        """
        Initialize the 4-DOF arm.
        
        Args:
            link_lengths: List of 4 link lengths [base_height, shoulder, elbow, wrist]
        """
        # Default link lengths (in arbitrary units)
        self.link_lengths = link_lengths or [0.5, 1.0, 0.8, 0.6]
        
        # Current joint angles (in degrees)
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        
        # Joint limits (in degrees)
        self.joint_limits = [
            (-180, 180),  # J1: Base rotation
            (-90, 90),    # J2: Shoulder pitch
            (-135, 135),  # J3: Elbow pitch  
            (-90, 90)     # J4: Wrist pitch
        ]
        
        # Initialize plot elements
        self.fig = None
        self.ax = None
        self.line = None
        self.joint_dots = None
        self.animation = None
        
    def degrees_to_radians(self, angles: List[float]) -> List[float]:
        """Convert degrees to radians."""
        return [np.deg2rad(angle) for angle in angles]
    
    def forward_kinematics(self, joint_angles: List[float]) -> Tuple[List[float], List[float], List[float]]:
        """
        Calculate forward kinematics for the 4-DOF arm.
        
        Args:
            joint_angles: List of 4 joint angles in degrees [J1, J2, J3, J4]
            
        Returns:
            Tuple of (x_coords, y_coords, z_coords) for each joint position
        """
        # Convert to radians
        angles_rad = self.degrees_to_radians(joint_angles)
        theta1, theta2, theta3, theta4 = angles_rad
        
        # Link lengths
        L0, L1, L2, L3 = self.link_lengths
        
        # Calculate joint positions using forward kinematics
        # Base (origin)
        x0, y0, z0 = 0, 0, 0
        
        # Joint 1 (top of base, after base rotation)
        x1 = 0
        y1 = 0  
        z1 = L0
        
        # Joint 2 (end of shoulder link)
        x2 = L1 * np.cos(theta2) * np.cos(theta1)
        y2 = L1 * np.cos(theta2) * np.sin(theta1)
        z2 = L0 + L1 * np.sin(theta2)
        
        # Joint 3 (end of elbow link)
        x3 = (L1 * np.cos(theta2) + L2 * np.cos(theta2 + theta3)) * np.cos(theta1)
        y3 = (L1 * np.cos(theta2) + L2 * np.cos(theta2 + theta3)) * np.sin(theta1)
        z3 = L0 + L1 * np.sin(theta2) + L2 * np.sin(theta2 + theta3)
        
        # End effector (end of wrist link)
        x4 = (L1 * np.cos(theta2) + L2 * np.cos(theta2 + theta3) + L3 * np.cos(theta2 + theta3 + theta4)) * np.cos(theta1)
        y4 = (L1 * np.cos(theta2) + L2 * np.cos(theta2 + theta3) + L3 * np.cos(theta2 + theta3 + theta4)) * np.sin(theta1)
        z4 = L0 + L1 * np.sin(theta2) + L2 * np.sin(theta2 + theta3) + L3 * np.sin(theta2 + theta3 + theta4)
        
        # Return coordinates
        x_coords = [x0, x1, x2, x3, x4]
        y_coords = [y0, y1, y2, y3, y4]
        z_coords = [z0, z1, z2, z3, z4]
        
        return x_coords, y_coords, z_coords
    
    def set_joint_angles(self, angles: List[float]) -> bool:
        """
        Set joint angles with limit checking.
        
        Args:
            angles: List of 4 joint angles in degrees
            
        Returns:
            True if all angles are within limits, False otherwise
        """
        if len(angles) != 4:
            print(f"Error: Expected 4 angles, got {len(angles)}")
            return False
        
        # Check limits
        for i, angle in enumerate(angles):
            min_angle, max_angle = self.joint_limits[i]
            if angle < min_angle or angle > max_angle:
                print(f"Warning: Joint {i+1} angle {angle}° exceeds limits [{min_angle}, {max_angle}]")
                # Clamp to limits
                angles[i] = max(min_angle, min(max_angle, angle))
        
        self.joint_angles = angles.copy()
        return True
    
    def setup_plot(self):
        """Initialize the matplotlib figure and axes."""
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Set up the plot
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y') 
        self.ax.set_zlabel('Z')
        self.ax.set_title('4-DOF Robotic Arm Simulation')
        
        # Set axis limits
        total_reach = sum(self.link_lengths)
        self.ax.set_xlim([-total_reach, total_reach])
        self.ax.set_ylim([-total_reach, total_reach])
        self.ax.set_zlim([0, total_reach])
        
        # Initialize line and joint markers
        self.line, = self.ax.plot([], [], [], 'b-', linewidth=3, label='Arm Links')
        self.joint_dots, = self.ax.plot([], [], [], 'ro', markersize=8, label='Joints')
        
        # Add legend
        self.ax.legend()
        
        # Add grid
        self.ax.grid(True)
        
    def update_plot(self, frame=None):
        """Update the plot with current joint positions."""
        # Calculate forward kinematics
        x_coords, y_coords, z_coords = self.forward_kinematics(self.joint_angles)
        
        # Update line data
        self.line.set_data_3d(x_coords, y_coords, z_coords)
        self.joint_dots.set_data_3d(x_coords, y_coords, z_coords)
        
        # Update title with current angles
        title = f'4-DOF Arm - Angles: J1={self.joint_angles[0]:.1f}° J2={self.joint_angles[1]:.1f}° J3={self.joint_angles[2]:.1f}° J4={self.joint_angles[3]:.1f}°'
        self.ax.set_title(title)
        
        return self.line, self.joint_dots
    
    def animate_to_pose(self, target_angles: List[float], duration: float = 2.0, steps: int = 50):
        """
        Animate the arm from current pose to target pose.
        
        Args:
            target_angles: Target joint angles in degrees
            duration: Animation duration in seconds
            steps: Number of interpolation steps
        """
        if not self.set_joint_angles(target_angles):
            return False
            
        # Store original angles
        start_angles = self.joint_angles.copy()
        
        # Create interpolated path
        angle_steps = []
        for step in range(steps + 1):
            t = step / steps
            interpolated = [
                start_angles[i] + t * (target_angles[i] - start_angles[i])
                for i in range(4)
            ]
            angle_steps.append(interpolated)
        
        # Animation function
        def animate_frame(frame):
            if frame < len(angle_steps):
                self.joint_angles = angle_steps[frame]
            return self.update_plot()
        
        # Create and start animation
        interval = (duration * 1000) / steps  # milliseconds per frame
        self.animation = FuncAnimation(self.fig, animate_frame, frames=len(angle_steps),
                                     interval=interval, blit=True, repeat=False)
        
        return True
    
    def plot_static(self, angles: List[float] = None):
        """
        Create a static plot of the arm at specified angles.
        
        Args:
            angles: Joint angles in degrees (uses current if None)
        """
        if angles:
            self.set_joint_angles(angles)
            
        if self.fig is None:
            self.setup_plot()
            
        self.update_plot()
        plt.show()
    
    def print_arm_info(self):
        """Print current arm configuration."""
        print("\n" + "="*50)
        print("4-DOF ROBOTIC ARM CONFIGURATION")
        print("="*50)
        print(f"Link Lengths: {self.link_lengths}")
        print(f"Current Angles: {[f'{angle:.1f}°' for angle in self.joint_angles]}")
        print(f"Joint Limits: {self.joint_limits}")
        
        # Calculate and print end effector position
        x_coords, y_coords, z_coords = self.forward_kinematics(self.joint_angles)
        end_effector = (x_coords[-1], y_coords[-1], z_coords[-1])
        print(f"End Effector Position: ({end_effector[0]:.3f}, {end_effector[1]:.3f}, {end_effector[2]:.3f})")
        print("="*50)


def test_arm_simulation():
    """Test function to demonstrate the 4-DOF arm simulation."""
    print("Testing 4-DOF Robotic Arm Simulation...")
    
    # Create arm instance
    arm = Arm4DOF()
    arm.print_arm_info()
    
    # Test hardcoded pose: [30, -45, 60, 20]
    test_angles = [30, -45, 60, 20]
    print(f"\nSetting arm to test pose: {test_angles}")
    
    # Set up plot
    arm.setup_plot()
    
    # Animate to test pose
    print("Animating to test pose...")
    arm.animate_to_pose(test_angles, duration=3.0, steps=60)
    
    # Show the plot
    plt.show()
    
    # Print final configuration
    arm.print_arm_info()


def interactive_demo():
    """Interactive demo allowing user to input joint angles."""
    arm = Arm4DOF()
    arm.setup_plot()
    
    print("\n" + "="*50)
    print("INTERACTIVE 4-DOF ARM DEMO")
    print("="*50)
    print("Enter joint angles or 'q' to quit")
    print("Format: J1 J2 J3 J4 (e.g., '30 -45 60 20')")
    print("Limits: J1:±180°, J2:±90°, J3:±135°, J4:±90°")
    print("="*50)
    
    while True:
        try:
            user_input = input("\nEnter angles (or 'q' to quit): ").strip()
            
            if user_input.lower() == 'q':
                break
                
            angles = [float(x) for x in user_input.split()]
            
            if len(angles) != 4:
                print("Error: Please enter exactly 4 angles")
                continue
                
            print(f"Animating to: {angles}")
            arm.animate_to_pose(angles, duration=2.0)
            plt.draw()
            
        except ValueError:
            print("Error: Please enter valid numbers")
        except KeyboardInterrupt:
            break
    
    print("Demo ended.")


if __name__ == "__main__":
    # Run the test with hardcoded angles [30, -45, 60, 20]
    test_arm_simulation()
    
    # Uncomment the line below for interactive demo
    # interactive_demo()
