import time
# Import the robot control module (assume the core script is named `robot_control.py`)
import robot_control as rc

def test_robot_controls():
    """Test all core motion control functions of the differential drive robot"""
    print("=== Starting Robot Control Test Sequence ===")
    
    # Test 1: Emergency Stop (initial state)
    print("\n1. Testing Emergency Stop...")
    rc.stop()
    time.sleep(1)  # Pause to observe
    
    # Test 2: Straight Forward Movement
    print("\n2. Testing Forward Movement (5 rad/s for 3 seconds)...")
    rc.forward(2.0)
    time.sleep(4)
    rc.stop()
    time.sleep(1)
    
    # Test 3: Straight Backward Movement
    print("\n3. Testing Backward Movement (-5 rad/s for 3 seconds)...")
    rc.forward(-2.0)
    time.sleep(4)
    rc.stop()
    time.sleep(1)
    
    # Test 4: In-Place Clockwise Rotation
    print("\n4. Testing Clockwise Rotation (2 rad/s for 2 seconds)...")
    rc.rotate_in_place(2.0)
    time.sleep(2)
    rc.stop()
    time.sleep(1)
    
    # Test 5: In-Place Counterclockwise Rotation
    print("\n5. Testing Counterclockwise Rotation (-2 rad/s for 2 seconds)...")
    rc.rotate_in_place(-2.0)
    time.sleep(2)
    rc.stop()
    time.sleep(1)

    print("\n=== All Test Cases Completed ===")

if __name__ == "__main__":
    # Ensure CoppeliaSim simulation is RUNNING before executing!
    input("Press Enter to start test (make sure CoppeliaSim is running with the robot scene)...")
    test_robot_controls()