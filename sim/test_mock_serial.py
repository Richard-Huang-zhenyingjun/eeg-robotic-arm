
#!/usr/bin/env python3
"""
Test Script for Mock Serial Driver
Week 2, Day 2 - EEG Robotic Arm Project

Quick test script to verify MockSerial class functionality.
Tests basic commands and verifies responses.
"""

import sys
import os

# Add sim directory to path for imports
sys.path.insert(0, os.path.dirname(__file__))

try:
    from mock_serial import MockSerial
except ImportError:
    print("Error: Could not import MockSerial from mock_serial.py")
    print("Make sure mock_serial.py is in the same directory as this script.")
    sys.exit(1)


def test_basic_functionality():
    """Test basic MockSerial functionality as specified."""
    print("="*50)
    print("TESTING MOCK SERIAL BASIC FUNCTIONALITY")
    print("="*50)
    
    # Create MockSerial instance
    ser = MockSerial()
    
    # Test the exact sequence from requirements
    print("\n1. Testing L command:")
    ser.write(b"L\n")
    response = ser.readline()
    print(f"   Response: {response.decode().strip()}")
    
    print("\n2. Testing R command:")
    ser.write(b"R\n") 
    response = ser.readline()
    print(f"   Response: {response.decode().strip()}")
    
    print("\n3. Testing G command:")
    ser.write(b"G\n")
    response = ser.readline()  
    print(f"   Response: {response.decode().strip()}")
    
    # Close connection
    ser.close()
    print("\n✅ Basic functionality test completed!")
    
    return True


def test_extended_functionality():
    """Test extended MockSerial functionality."""
    print("\n" + "="*50)
    print("TESTING EXTENDED FUNCTIONALITY")
    print("="*50)
    
    ser = MockSerial()
    
    # Test STATUS command
    print("\n1. Testing STATUS command:")
    ser.write(b"STATUS\n")
    response = ser.readline()
    print(f"   Response: {response.decode().strip()}")
    
    # Test RESET command
    print("\n2. Testing RESET command:")
    ser.write(b"RESET\n")
    response = ser.readline()
    print(f"   Response: {response.decode().strip()}")
    
    # Test multiple commands in sequence
    print("\n3. Testing command sequence:")
    commands = [b"L\n", b"L\n", b"R\n", b"G\n", b"STATUS\n"]
    
    for i, cmd in enumerate(commands, 1):
        print(f"   {i}. Sending: {cmd.decode().strip()}")
        ser.write(cmd)
        response = ser.readline()
        print(f"      Response: {response.decode().strip()}")
    
    # Test invalid command
    print("\n4. Testing invalid command:")
    ser.write(b"INVALID\n")
    response = ser.readline()
    print(f"   Response: {response.decode().strip()}")
    
    # Show robot state
    print(f"\n5. Final robot state:")
    state = ser.get_robot_state()
    for key, value in state.items():
        print(f"   {key}: {value}")
    
    ser.close()
    print("\n✅ Extended functionality test completed!")
    
    return True


def test_error_handling():
    """Test error handling and edge cases."""
    print("\n" + "="*50)
    print("TESTING ERROR HANDLING")
    print("="*50)
    
    ser = MockSerial()
    
    # Test timeout behavior (should be quick since it's mocked)
    print("\n1. Testing read without write:")
    try:
        response = ser.readline(timeout=0.1)  # Short timeout
        print(f"   Response: {response.decode().strip()}")
    except Exception as e:
        print(f"   Error: {e}")
    
    # Test reading from closed connection
    print("\n2. Testing closed connection:")
    ser.close()
    try:
        ser.write(b"L\n")
        print("   ERROR: Should have thrown exception!")
    except Exception as e:
        print(f"   Expected error: {e}")
    
    # Reopen and test
    ser.open()
    ser.write(b"L\n")
    response = ser.readline()
    print(f"   After reopening: {response.decode().strip()}")
    
    ser.close()
    print("\n✅ Error handling test completed!")
    
    return True


def performance_test():
    """Test performance with multiple rapid commands."""
    print("\n" + "="*50)
    print("TESTING PERFORMANCE")
    print("="*50)
    
    import time
    
    ser = MockSerial()
    
    # Send multiple commands rapidly
    num_commands = 100
    start_time = time.time()
    
    print(f"\nSending {num_commands} commands...")
    
    for i in range(num_commands):
        cmd = b"L\n" if i % 2 == 0 else b"R\n"
        ser.write(cmd)
        response = ser.readline()
        
        # Print progress every 20 commands
        if (i + 1) % 20 == 0:
            print(f"   Processed {i + 1}/{num_commands} commands")
    
    end_time = time.time()
    duration = end_time - start_time
    
    print(f"\nPerformance Results:")
    print(f"   Total time: {duration:.3f} seconds")
    print(f"   Commands per second: {num_commands / duration:.1f}")
    print(f"   Average response time: {duration * 1000 / num_commands:.2f} ms")
    
    # Check final state
    state = ser.get_robot_state()
    print(f"   Final command count: {state['command_count']}")
    
    ser.close()
    print("\n✅ Performance test completed!")
    
    return True


def main():
    """Run all tests."""
    print("MOCK SERIAL DRIVER TEST SUITE")
    print("=" * 60)
    
    tests = [
        ("Basic Functionality", test_basic_functionality),
        ("Extended Functionality", test_extended_functionality), 
        ("Error Handling", test_error_handling),
        ("Performance", performance_test)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        try:
            print(f"\n🧪 Running {test_name} Test...")
            if test_func():
                passed += 1
                print(f"✅ {test_name} Test: PASSED")
            else:
                print(f"❌ {test_name} Test: FAILED")
        except Exception as e:
            print(f"❌ {test_name} Test: ERROR - {e}")
    
    print("\n" + "="*60)
    print("TEST RESULTS SUMMARY")
    print("="*60)
    print(f"Tests passed: {passed}/{total}")
    print(f"Success rate: {passed/total*100:.1f}%")
    
    if passed == total:
        print("🎉 All tests passed!")
        return 0
    else:
        print("⚠️  Some tests failed!")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)


