
#!/usr/bin/env python3
"""
Arm Driver Integration Demo
Week 2, Day 3 - EEG Robotic Arm Project

Demonstrates the integration between ArmDriver and arm simulation,
showing how terminal commands update joint angles in real-time visualization.
"""

import sys
import os
import time

# Add paths for imports
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

try:
    from arm_driver import ArmDriver, ArmDriverWithVisualization
    from arm_sim import Arm4DOF
except ImportError as e:
    print(f"Import error: {e}")
    print("Make sure arm_driver.py and sim/arm_sim.py exist.")
    sys.exit(1)


def terminal_command_demo():
    """
    Demo showing how terminal commands update joint angles in simulation.
    This fulfills the Day 3 deliverable requirement.
    """
    print("\n" + "="*70)
    print("ARM DRIVER INTEGRATION DEMO")
    print("Terminal Commands → Joint Angles → Simulation Updates")
    print("="*70)
    
    # Create integrated driver with visualization
    try:
        integrated_driver = ArmDriverWithVisualization()
        
        if not integrated_driver.visualizer:
            print("⚠️  Visualization not available, running driver-only demo")
            return driver_only_demo()
        
        print("✅ Visualization system initialized")
        
    except Exception as e:
        print(f"❌ Error initializing visualization: {e}")
        print("Running driver-only demo instead...")
        return driver_only_demo()
    
    # Setup the visualization
    integrated_driver.visualizer.setup_plot()
    
    # Demo sequence with explanations
    demo_sequence = [
        ("L", "Rotate base LEFT (+10°)"),
        ("L", "Rotate base LEFT again (+10°)"),
        ("R", "Rotate base RIGHT (-10°)"),
        ("G", "Close gripper"),
        ("L", "Rotate base LEFT with gripper closed"),
        ("G", "Open gripper"),
        ("R", "Rotate base RIGHT"),
        ("R", "Rotate base RIGHT again")
    ]
    
    print("\nDemo Sequence:")
    print("-" * 50)
    
    import matplotlib.pyplot as plt
    plt.ion()  # Interactive mode for real-time updates
    plt.show()
    
    for i, (command, description) in enumerate(demo_sequence, 1):
        print(f"\nStep {i}: {description}")
        print(f"  Command: '{command}'")
        
        # Show state before
        state_before = integrated_driver.driver.get_current_state()
        print(f"  Before:  Angle={state_before['joint_angles'][0]:+6.1f}°, "
              f"Gripper={'open' if state_before['gripper_open'] else 'closed'}")
        
        # Process command
        success = integrated_driver.process_command(command)
        
        if success:
            # Show state after
            state_after = integrated_driver.driver.get_current_state()
            print(f"  After:   Angle={state_after['joint_angles'][0]:+6.1f}°, "
                  f"Gripper={'open' if state_after['gripper_open'] else 'closed'}")
            
            # Update visualization
            plt.draw()
            plt.pause(1.5)  # Pause to see the change
            
            print("  ✅ Command processed, simulation updated")
        else:
            print("  ❌ Command failed")
        
        # Small delay between commands
        time.sleep(0.5)
    
    plt.ioff()
    
    # Final state summary
    final_state = integrated_driver.driver.get_current_state()
    print(f"\n" + "="*50)
    print("FINAL STATE SUMMARY")
    print("="*50)
    print(f"Total commands processed: {final_state['total_commands']}")
    print(f"Final joint angles: {[f'{a:+6.1f}°' for a in final_state['joint_angles']]}")
    print(f"Final gripper state: {'open' if final_state['gripper_open'] else 'closed'}")
    
    # Show command history
    history = integrated_driver.driver.get_command_history()
    print(f"\nCommand History:")
    for i, record in enumerate(history, 1):
        status = "✅" if record['success'] else "❌"
        print(f"  {i}. '{record['original_command']}' {status}")
    
    print("\n🎉 Integration demo completed successfully!")
    print("✅ Deliverable achieved: Terminal commands update joint angles in simulation")


def driver_only_demo():
    """
    Fallback demo using driver only (no visualization).
    """
    print("\n" + "="*60)
    print("ARM DRIVER DEMO (NO VISUALIZATION)")
    print("="*60)
    
    driver = ArmDriver()
    
    # Demo sequence
    commands = ['L', 'L', 'R', 'G', 'L', 'G', 'R', 'R']
    
    print("Processing command sequence:")
    print(f"Commands: {commands}")
    print("-" * 40)
    
    for i, cmd in enumerate(commands, 1):
        print(f"\nStep {i}: Command '{cmd}'")
        
        # Show state before
        state_before = driver.get_current_state()
        print(f"  Before: {[f'{a:+6.1f}°' for a in state_before['joint_angles']]}, "
              f"Gripper: {'open' if state_before['gripper_open'] else 'closed'}")
        
        # Process command
        success = driver.process_command(cmd)
        
        # Show state after
        state_after = driver.get_current_state()
        print(f"  After:  {[f'{a:+6.1f}°' for a in state_after['joint_angles']]}, "
              f"Gripper: {'open' if state_after['gripper_open'] else 'closed'}")
        print(f"  Status: {'✅ Success' if success else '❌ Failed'}")
    
    # Final summary
    final_state = driver.get_current_state()
    print(f"\n" + "="*40)
    print("FINAL STATE")
    print("="*40)
    print(f"Commands processed: {final_state['total_commands']}")
    print(f"Joint angles: {[f'{a:+6.1f}°' for a in final_state['joint_angles']]}")
    print(f"Gripper: {'open' if final_state['gripper_open'] else 'closed'}")
    
    print("\n✅ Driver demo completed!")


def interactive_terminal_demo():
    """
    Interactive demo where user can enter commands manually.
    """
    print("\n" + "="*70)
    print("INTERACTIVE TERMINAL COMMAND DEMO")
    print("="*70)
    
    try:
        integrated_driver = ArmDriverWithVisualization()
        if integrated_driver.visualizer:
            print("✅ Starting with visualization")
            integrated_driver.start_interactive_mode()
        else:
            print("⚠️  No visualization available")
            interactive_driver_only()
    except Exception as e:
        print(f"❌ Error: {e}")
        interactive_driver_only()


def interactive_driver_only():
    """Interactive demo without visualization."""
    driver = ArmDriver()
    
    print("\nCommands: L (left), R (right), G (grip), STATUS, RESET, Q (quit)")
    print("-" * 50)
    
    while True:
        try:
            command = input("\nEnter command: ").strip()
            
            if command.upper() == 'Q':
                break
            elif command.upper() == 'STATUS':
                state = driver.get_current_state()
                print(f"  Angles: {[f'{a:+6.1f}°' for a in state['joint_angles']]}")
                print(f"  Gripper: {'open' if state['gripper_open'] else 'closed'}")
                print(f"  Commands: {state['total_commands']}")
            elif command.upper() == 'RESET':
                driver.reset_to_home()
                print("  🏠 Reset to home position")
            else:
                success = driver.process_command(command)
                if success:
                    state = driver.get_current_state()
                    print(f"  ✅ New angle: {state['joint_angles'][0]:+6.1f}°, "
                          f"Gripper: {'open' if state['gripper_open'] else 'closed'}")
                else:
                    print(f"  ❌ Invalid command: '{command}'")
                    
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"  Error: {e}")
    
    print("\nDemo ended.")


def test_integration():
    """
    Test the integration between driver and simulation.
    """
    print("\n" + "="*60)
    print("TESTING ARM DRIVER INTEGRATION")
    print("="*60)
    
    # Test 1: Driver basic functionality
    print("\n1. Testing ArmDriver basic functionality...")
    driver = ArmDriver()
    
    test_commands = ['L', 'R', 'G']
    all_passed = True
    
    for cmd in test_commands:
        success = driver.process_command(cmd)
        if success:
            print(f"   ✅ Command '{cmd}' processed successfully")
        else:
            print(f"   ❌ Command '{cmd}' failed")
            all_passed = False
    
    # Test 2: Visualization integration (if available)
    print("\n2. Testing visualization integration...")
    try:
        integrated = ArmDriverWithVisualization()
        if integrated.visualizer:
            print("   ✅ Visualization integration successful")
            
            # Test callback mechanism
            callback_called = False
            def test_callback(angles):
                nonlocal callback_called
                callback_called = True
            
            integrated.driver.set_update_callback(test_callback)
            integrated.driver.process_command('L')
            
            if callback_called:
                print("   ✅ Update callback mechanism working")
            else:
                print("   ❌ Update callback not triggered")
                all_passed = False
        else:
            print("   ⚠️  Visualization not available (expected in some environments)")
    except Exception as e:
        print(f"   ❌ Visualization integration failed: {e}")
        all_passed = False
    
    # Test 3: State consistency
    print("\n3. Testing state consistency...")
    driver.reset_to_home()
    initial_state = driver.get_current_state()
    
    # Process sequence and verify
    sequence = ['L', 'L', 'R', 'G']
    for cmd in sequence:
        driver.process_command(cmd)
    
    final_state = driver.get_current_state()
    expected_angle = 10.0  # L(+10) + L(+10) + R(-10) = +10
    
    if abs(final_state['joint_angles'][0] - expected_angle) < 0.001:
        print("   ✅ State consistency verified")
    else:
        print(f"   ❌ State inconsistency: expected {expected_angle}°, got {final_state['joint_angles'][0]}°")
        all_passed = False
    
    # Final result
    print(f"\n" + "="*40)
    if all_passed:
        print("🎉 ALL INTEGRATION TESTS PASSED!")
        print("✅ Ready for Day 4 development")
    else:
        print("❌ Some integration tests failed")
        print("🔧 Please check the issues above")
    
    return all_passed


def main():
    """Main function to run different demo modes."""
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python arm_integration_demo.py demo        # Run automated demo")
        print("  python arm_integration_demo.py interactive # Interactive mode")
        print("  python arm_integration_demo.py test        # Test integration")
        return
    
    mode = sys.argv[1].lower()
    
    if mode == 'demo':
        terminal_command_demo()
    elif mode == 'interactive':
        interactive_terminal_demo()
    elif mode == 'test':
        success = test_integration()
        sys.exit(0 if success else 1)
    else:
        print(f"Unknown mode: {mode}")
        print("Use 'demo', 'interactive', or 'test'")


if __name__ == "__main__":
    main()


