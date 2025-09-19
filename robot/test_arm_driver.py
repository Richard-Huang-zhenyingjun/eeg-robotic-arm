
#!/usr/bin/env python3
"""
Unit Tests for Arm Driver
Week 2, Day 3 - EEG Robotic Arm Project

Comprehensive unit tests for the ArmDriver class to verify command parsing
and joint angle updates.
"""

import unittest
import sys
import os

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(__file__))

try:
    from arm_driver import ArmDriver
except ImportError:
    print("Error: Could not import ArmDriver from arm_driver.py")
    print("Make sure arm_driver.py is in the same directory.")
    sys.exit(1)


class TestArmDriver(unittest.TestCase):
    """Test cases for ArmDriver class."""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        self.driver = ArmDriver()
        # Suppress logging during tests
        self.driver._logger.setLevel(50)  # CRITICAL level to suppress INFO logs
    
    def tearDown(self):
        """Clean up after each test method."""
        pass
    
    def test_initialization(self):
        """Test driver initialization."""
        # Test default initialization
        driver = ArmDriver()
        self.assertEqual(driver.joint_angles, [0.0, 0.0, 0.0, 0.0])
        self.assertTrue(driver.gripper_open)
        
        # Test initialization with custom angles
        custom_angles = [10.0, 20.0, 30.0, 40.0]
        driver_custom = ArmDriver(custom_angles)
        self.assertEqual(driver_custom.joint_angles, custom_angles)
        self.assertTrue(driver_custom.gripper_open)
    
    def test_left_command(self):
        """Test left rotation command (L)."""
        initial_angle = self.driver.joint_angles[0]
        
        # Single L command
        success = self.driver.process_command('L')
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], initial_angle + 10.0)
        
        # Multiple L commands
        for i in range(1, 5):
            success = self.driver.process_command('L')
            self.assertTrue(success)
            expected_angle = initial_angle + (i + 1) * 10.0
            self.assertEqual(self.driver.joint_angles[0], expected_angle)
    
    def test_right_command(self):
        """Test right rotation command (R)."""
        initial_angle = self.driver.joint_angles[0]
        
        # Single R command
        success = self.driver.process_command('R')
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], initial_angle - 10.0)
        
        # Multiple R commands
        for i in range(1, 5):
            success = self.driver.process_command('R')
            self.assertTrue(success)
            expected_angle = initial_angle - (i + 1) * 10.0
            self.assertEqual(self.driver.joint_angles[0], expected_angle)
    
    def test_grip_command(self):
        """Test gripper toggle command (G)."""
        # Initial state should be open
        self.assertTrue(self.driver.gripper_open)
        
        # First G command - should close
        success = self.driver.process_command('G')
        self.assertTrue(success)
        self.assertFalse(self.driver.gripper_open)
        
        # Second G command - should open
        success = self.driver.process_command('G')
        self.assertTrue(success)
        self.assertTrue(self.driver.gripper_open)
        
        # Third G command - should close again
        success = self.driver.process_command('G')
        self.assertTrue(success)
        self.assertFalse(self.driver.gripper_open)
    
    def test_command_case_sensitivity(self):
        """Test that commands work with different cases."""
        initial_angle = self.driver.joint_angles[0]
        
        # Test lowercase
        success = self.driver.process_command('l')
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], initial_angle + 10.0)
        
        # Test uppercase
        success = self.driver.process_command('R')
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], initial_angle)
        
        # Test mixed case with whitespace
        success = self.driver.process_command(' g ')
        self.assertTrue(success)
        self.assertFalse(self.driver.gripper_open)
    
    def test_invalid_commands(self):
        """Test handling of invalid commands."""
        initial_state = self.driver.get_current_state()
        
        invalid_commands = ['X', 'INVALID', '123', '', ' ', 'LL', 'RR']
        
        for cmd in invalid_commands:
            success = self.driver.process_command(cmd)
            self.assertFalse(success)
            
            # State should be unchanged
            current_state = self.driver.get_current_state()
            self.assertEqual(current_state['joint_angles'], initial_state['joint_angles'])
            self.assertEqual(current_state['gripper_open'], initial_state['gripper_open'])
    
    def test_joint_limits(self):
        """Test that joint limits are enforced."""
        # Test positive limit (180°)
        # Start from 170° and try to go beyond 180°
        self.driver.set_joint_angles([170.0, 0.0, 0.0, 0.0])
        
        success = self.driver.process_command('L')  # Should go to 180°
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], 180.0)
        
        success = self.driver.process_command('L')  # Should stay at 180°
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], 180.0)
        
        # Test negative limit (-180°)
        self.driver.set_joint_angles([-170.0, 0.0, 0.0, 0.0])
        
        success = self.driver.process_command('R')  # Should go to -180°
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], -180.0)
        
        success = self.driver.process_command('R')  # Should stay at -180°
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles[0], -180.0)
    
    def test_command_sequence(self):
        """Test processing of command sequences."""
        commands = ['L', 'L', 'R', 'G', 'L']
        
        results = self.driver.process_command_sequence(commands)
        
        # Check results structure
        self.assertIn('total_commands', results)
        self.assertIn('successful_commands', results)
        self.assertIn('failed_commands', results)
        self.assertIn('final_angles', results)
        self.assertIn('final_gripper_state', results)
        
        # Check values
        self.assertEqual(results['total_commands'], len(commands))
        self.assertEqual(results['successful_commands'], len(commands))
        self.assertEqual(results['failed_commands'], 0)
        
        # Expected final state: L, L, R, G, L -> +10, +10, -10, toggle, +10 = +20°
        self.assertEqual(results['final_angles'][0], 20.0)
        self.assertFalse(results['final_gripper_state'])  # Gripper should be closed
    
    def test_command_sequence_with_failures(self):
        """Test command sequence with some invalid commands."""
        commands = ['L', 'INVALID', 'R', 'G', 'BAD']
        
        results = self.driver.process_command_sequence(commands)
        
        self.assertEqual(results['total_commands'], 5)
        self.assertEqual(results['successful_commands'], 3)  # L, R, G
        self.assertEqual(results['failed_commands'], 2)      # INVALID, BAD
        
        # Final angle should be: +10 (L) -10 (R) = 0°
        self.assertEqual(results['final_angles'][0], 0.0)
        self.assertFalse(results['final_gripper_state'])  # Gripper closed from G
    
    def test_reset_to_home(self):
        """Test reset to home functionality."""
        # Move arm to some position
        self.driver.process_command_sequence(['L', 'L', 'L', 'G'])
        
        # Verify it's not at home
        self.assertNotEqual(self.driver.joint_angles, [0.0, 0.0, 0.0, 0.0])
        self.assertFalse(self.driver.gripper_open)
        
        # Reset to home
        self.driver.reset_to_home()
        
        # Verify home position
        self.assertEqual(self.driver.joint_angles, [0.0, 0.0, 0.0, 0.0])
        self.assertTrue(self.driver.gripper_open)
    
    def test_set_joint_angles(self):
        """Test direct joint angle setting."""
        new_angles = [45.0, -30.0, 60.0, -15.0]
        
        success = self.driver.set_joint_angles(new_angles)
        self.assertTrue(success)
        self.assertEqual(self.driver.joint_angles, new_angles)
        
        # Test invalid number of angles
        success = self.driver.set_joint_angles([10.0, 20.0])  # Only 2 angles
        self.assertFalse(success)
        self.assertEqual(self.driver.joint_angles, new_angles)  # Should be unchanged
        
        # Test angles exceeding limits
        invalid_angles = [200.0, 0.0, 0.0, 0.0]  # 200° exceeds ±180° limit
        success = self.driver.set_joint_angles(invalid_angles)
        self.assertFalse(success)
        self.assertEqual(self.driver.joint_angles, new_angles)  # Should be unchanged
    
    def test_command_history(self):
        """Test command history tracking."""
        # Initially empty
        history = self.driver.get_command_history()
        self.assertEqual(len(history), 0)
        
        # Process some commands
        commands = ['L', 'R', 'G', 'INVALID']
        for cmd in commands:
            self.driver.process_command(cmd)
        
        # Check history
        history = self.driver.get_command_history()
        self.assertEqual(len(history), len(commands))
        
        for i, record in enumerate(history):
            self.assertIn('timestamp', record)
            self.assertIn('original_command', record)
            self.assertIn('normalized_command', record)
            self.assertIn('success', record)
            self.assertEqual(record['original_command'], commands[i])
            self.assertEqual(record['normalized_command'], commands[i].upper())
        
        # Clear history
        self.driver.clear_history()
        history = self.driver.get_command_history()
        self.assertEqual(len(history), 0)
    
    def test_get_current_state(self):
        """Test current state retrieval."""
        # Get initial state
        state = self.driver.get_current_state()
        
        # Check state structure
        self.assertIn('joint_angles', state)
        self.assertIn('gripper_open', state)
        self.assertIn('total_commands', state)
        self.assertIn('last_command', state)
        
        # Check initial values
        self.assertEqual(state['joint_angles'], [0.0, 0.0, 0.0, 0.0])
        self.assertTrue(state['gripper_open'])
        self.assertEqual(state['total_commands'], 0)
        self.assertIsNone(state['last_command'])
        
        # Process a command and check state update
        self.driver.process_command('L')
        state = self.driver.get_current_state()
        
        self.assertEqual(state['joint_angles'][0], 10.0)
        self.assertEqual(state['total_commands'], 1)
        self.assertIsNotNone(state['last_command'])
    
    def test_update_callback(self):
        """Test update callback functionality."""
        callback_calls = []
        
        def test_callback(angles):
            callback_calls.append(angles.copy())
        
        # Set callback
        self.driver.set_update_callback(test_callback)
        
        # Initially no calls
        self.assertEqual(len(callback_calls), 0)
        
        # Process commands
        self.driver.process_command('L')
        self.assertEqual(len(callback_calls), 1)
        self.assertEqual(callback_calls[0][0], 10.0)
        
        self.driver.process_command('R')
        self.assertEqual(len(callback_calls), 2)
        self.assertEqual(callback_calls[1][0], 0.0)
        
        # Reset should also trigger callback
        self.driver.reset_to_home()
        self.assertEqual(len(callback_calls), 3)
        self.assertEqual(callback_calls[2], [0.0, 0.0, 0.0, 0.0])
        
        # Invalid command should not trigger callback
        self.driver.process_command('INVALID')
        self.assertEqual(len(callback_calls), 3)  # Should still be 3


class TestArmDriverIntegration(unittest.TestCase):
    """Integration tests for ArmDriver with realistic scenarios."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.driver = ArmDriver()
        self.driver._logger.setLevel(50)  # Suppress logging
    
    def test_realistic_movement_sequence(self):
        """Test a realistic sequence of arm movements."""
        # Simulate picking up an object
        sequence = [
            'L', 'L', 'L',      # Move to position
            'G',                # Close gripper
            'R', 'R',           # Move back
            'G',                # Open gripper
            'L'                 # Small adjustment
        ]
        
        results = self.driver.process_command_sequence(sequence)
        
        self.assertEqual(results['successful_commands'], len(sequence))
        self.assertEqual(results['failed_commands'], 0)
        
        # Final position: +30 -20 +10 = +20°
        self.assertEqual(results['final_angles'][0], 20.0)
        
        # Gripper should be open
        self.assertTrue(results['final_gripper_state'])
    
    def test_boundary_conditions(self):
        """Test behavior at joint limits."""
        # Move to positive limit
        for _ in range(20):  # More than enough to hit +180° limit
            self.driver.process_command('L')
        
        self.assertEqual(self.driver.joint_angles[0], 180.0)
        
        # Try to go further - should stay at limit
        self.driver.process_command('L')
        self.assertEqual(self.driver.joint_angles[0], 180.0)
        
        # Move to negative limit
        for _ in range(40):  # More than enough to hit -180° limit
            self.driver.process_command('R')
        
        self.assertEqual(self.driver.joint_angles[0], -180.0)
        
        # Try to go further - should stay at limit
        self.driver.process_command('R')
        self.assertEqual(self.driver.joint_angles[0], -180.0)
    
    def test_mixed_command_formats(self):
        """Test various command formats and edge cases."""
        test_cases = [
            ('L', True, 10.0),
            ('l', True, 20.0),
            (' R ', True, 10.0),
            ('  g  ', True, None),  # Gripper state
            ('', False, 10.0),      # Empty string
            ('LR', False, 10.0),    # Multiple chars
            ('1', False, 10.0),     # Number
        ]
        
        for command, expected_success, expected_angle in test_cases:
            initial_angle = self.driver.joint_angles[0]
            success = self.driver.process_command(command)
            
            self.assertEqual(success, expected_success, 
                           f"Command '{command}' success mismatch")
            
            if expected_angle is not None:
                self.assertEqual(self.driver.joint_angles[0], expected_angle,
                               f"Command '{command}' angle mismatch")
    
    def test_state_consistency(self):
        """Test that driver state remains consistent."""
        # Process random sequence
        import random
        commands = ['L', 'R', 'G', 'L', 'R', 'L', 'G', 'R']
        random.shuffle(commands)
        
        for cmd in commands:
            old_state = self.driver.get_current_state()
            self.driver.process_command(cmd)
            new_state = self.driver.get_current_state()
            
            # Command count should always increase
            self.assertEqual(new_state['total_commands'], 
                           old_state['total_commands'] + 1)
            
            # Only joint 0 should change for L/R commands
            if cmd in ['L', 'R']:
                for i in [1, 2, 3]:
                    self.assertEqual(new_state['joint_angles'][i],
                                   old_state['joint_angles'][i])
                # Gripper should not change
                self.assertEqual(new_state['gripper_open'],
                               old_state['gripper_open'])
            
            # Only gripper should change for G command
            elif cmd == 'G':
                self.assertEqual(new_state['joint_angles'],
                               old_state['joint_angles'])


def run_performance_test():
    """Run performance test for many commands."""
    import time
    
    print("\n" + "="*50)
    print("PERFORMANCE TEST")
    print("="*50)
    
    driver = ArmDriver()
    driver._logger.setLevel(50)  # Suppress logging
    
    num_commands = 1000
    commands = ['L', 'R', 'G'] * (num_commands // 3)
    
    start_time = time.time()
    
    for cmd in commands:
        driver.process_command(cmd)
    
    end_time = time.time()
    duration = end_time - start_time
    
    print(f"Processed {num_commands} commands in {duration:.3f} seconds")
    print(f"Rate: {num_commands / duration:.1f} commands/second")
    print(f"Average time per command: {duration * 1000 / num_commands:.2f} ms")
    
    state = driver.get_current_state()
    print(f"Final state: {state}")


def main():
    """Run all tests."""
    print("ARM DRIVER UNIT TESTS")
    print("=" * 60)
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test classes
    suite.addTests(loader.loadTestsFromTestCase(TestArmDriver))
    suite.addTests(loader.loadTestsFromTestCase(TestArmDriverIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print("\nFAILURES:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print("\nERRORS:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    success_rate = (result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100
    print(f"\nSuccess rate: {success_rate:.1f}%")
    
    if result.wasSuccessful():
        print("🎉 All tests passed!")
        
        # Run performance test if all tests pass
        run_performance_test()
        
        return 0
    else:
        print("❌ Some tests failed!")
        return 1


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'perf':
        # Run only performance test
        run_performance_test()
    else:
        # Run full test suite
        exit_code = main()
        sys.exit(exit_code)


