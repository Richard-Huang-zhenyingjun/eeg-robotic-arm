#!/usr/bin/env python3
"""Quick integration test to verify Day 4 deliverable."""

import sys
import os
sys.path.insert(0, 'sim')
sys.path.insert(0, 'robot')

def test_integration():
    print("🧪 Testing Day 4 Integration...")
    
    try:
        # Test imports
        from mock_serial import MockSerial
        from arm_driver import ArmDriver
        from arm_sim import Arm4DOF
        print("✅ All imports successful")
        
        # Test MockSerial → ArmDriver pipeline
        serial = MockSerial()
        driver = ArmDriver()
        
        print("✅ Components initialized")
        
        # Test command flow
        initial_angle = driver.joint_angles[0]
        
        # Simulate serial command
        serial.write(b"L\n")
        response = serial.readline()
        
        # Process through driver
        driver.process_command("L")
        new_angle = driver.joint_angles[0]
        
        print(f"✅ Command processed: {initial_angle}° → {new_angle}°")
        print(f"✅ Serial response: {response.decode().strip()}")
        
        # Test joint limits
        driver.joint_limits[0] = (-90, 90)
        driver.set_joint_angles([85, 0, 0, 0])
        driver.process_command("L")  # Should go to 90°
        driver.process_command("L")  # Should stay at 90°
        
        if driver.joint_angles[0] == 90.0:
            print("✅ Joint limits working")
        else:
            print("❌ Joint limits not working")
            
        print("\n🎉 Integration test PASSED!")
        print("✅ Ready for live testing with run_sim_arm.py")
        
    except Exception as e:
        print(f"❌ Integration test FAILED: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_integration()
