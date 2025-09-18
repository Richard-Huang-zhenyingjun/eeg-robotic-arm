#!/usr/bin/env python3
"""Simple PyBullet test for 4-DOF robotic arm"""
import pybullet as p
import pybullet_data
import time
import numpy as np

def test_pybullet_arm():
    """Test PyBullet installation with a simple arm simulation"""
    # Connect to PyBullet (use DIRECT for headless)
    physicsClient = p.connect(p.DIRECT)  # Change to p.GUI if X11 works
    
    # Set up simulation
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Load ground plane
    planeId = p.loadURDF("plane.urdf")
    print(f"Plane loaded with ID: {planeId}")
    
    # Simple test
    print("PyBullet is working! Simulation running for 3 seconds...")
    for i in range(240 * 3):  # 3 seconds at 240Hz
        p.stepSimulation()
        if i % 240 == 0:
            print(f"Simulated {i//240 + 1} second(s)")
    
    # Cleanup
    p.disconnect()
    print("Test complete!")

if __name__ == "__main__":
    print("Testing PyBullet installation...")
    test_pybullet_arm()
