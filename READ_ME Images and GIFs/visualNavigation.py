import math
import time
import coppeliasim_zmqremoteapi_client as zmq

# Import corrected robot control module (aligned with Lua direction logic)
from robot_control import set_velocity, stop
from Shape_Recognition_HSV import getAppleLocations

# --- Initialize CoppeliaSim Client ---
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')


ROBOT_DUMMY_HANDLE = sim.getObject('/robot_base_plate/Robot')  
CAMERA_HANDLE = sim.getObject('/robot_base_plate/visionSensor')

# --- Helper Functions (1:1 with Lua, using Robot dummy) ---
def get_robot_pose():
    """
    Get robot pose from the Robot dummy (EXACT match to Lua's sysCall_actuation)
    Returns:
        tuple: (robot_x, robot_y, robot_yaw)
        - robot_x: X coordinate of Robot dummy (left/right, +left)
        - robot_y: Y coordinate of Robot dummy (forward/back, +forward)
        - robot_yaw: Yaw angle (Z-axis rotation) of Robot dummy (rad, [-pi, pi])
    """
    # Match Lua: sim.getObjectPosition(robot, -1)
    robot_pos = sim.getObjectPosition(ROBOT_DUMMY_HANDLE, -1)
    # Match Lua: sim.getObjectOrientation(robot, -1)
    robot_ori = sim.getObjectOrientation(ROBOT_DUMMY_HANDLE, -1)
    
    # Extract X/Y (matching Lua's robotPos[1], robotPos[2]) and yaw (robotOri[3])
    return robot_pos[0], robot_pos[1], robot_ori[2]

def compute_heading_error(robot_yaw, target_dx, target_dy):
    """
    Calculate angle error (EXACT copy of Lua's angleError logic)
    Args:
        robot_yaw: Yaw from Robot dummy (get_robot_pose)
        target_dx: X difference (target_x - robot_x, +left)
        target_dy: Y difference (robot_y - target_y, +forward)
    Returns:
        Normalized error in [-pi, pi]
    """
    # Match Lua: targetAngle = math.atan2(dx, dy)
    target_angle = math.atan2(target_dx, target_dy)
    angle_error = target_angle - robot_yaw

    # Match Lua's normalization
    if angle_error > math.pi:
        angle_error -= 2 * math.pi
    if angle_error < -math.pi:
        angle_error += 2 * math.pi
    return angle_error

# --- Core Navigation (Lua-matched logic with Robot dummy) ---
def go_to_target(target_x, target_y):
    """
    Navigate to target using Robot dummy coordinates (same as Lua's tree navigation)
    Args:
        target_x: World X of apple (matches Robot dummy's X frame)
        target_y: World Y of apple (matches Robot dummy's Y frame)
    """
    while True:
        # Get pose from Robot dummy (same as Lua's robotPos/robotOri)
        robot_x, robot_y, robot_yaw = get_robot_pose()

        # Match Lua's dx/dy calculation EXACTLY
        dx = target_x - robot_x  # + = target is left of robot
        dy = robot_y - target_y  # + = target is forward of robot
        distance = math.sqrt(dx**2 + dy**2)

        # Stop condition (matches Lua's safeDistance check)
        if distance < 0.3:  # Reduced for apples (Lua uses 1.8 for trees)
            stop()
            print(f"Reached apple at (X: {target_x:.2f}, Y: {target_y:.2f})")
            return

        # Calculate error (Lua-matched)
        ang_err = compute_heading_error(robot_yaw, dx, dy)

        # Match Lua's speed control (baseSpeed=1.0, k_turn=2.0)
        linear_vel = 1.0 * distance  # Lua's baseSpeed * distance
        angular_vel = 2.0 * ang_err  # Lua's k_turn * angleError

        # Speed clamping (prevent over-speed, same as Lua's implicit limits)
        linear_vel = max(min(linear_vel, 1.0), -1.0)
        angular_vel = max(min(angular_vel, 2.0), -2.0)

        # Send command to robot (corrected direction in robot_control.py)
        set_velocity(linear_vel, angular_vel)
        time.sleep(0.05)  # Match Lua's sysCall_actuation frequency (~20ms)

def visual_navigation():
    """Main loop: Detect apples → Navigate to nearest good apple (Robot dummy frame)"""
    print("Starting navigation (using Robot dummy for coordinates)...")
    while True:
        # Get apple positions (ensure they're in Robot dummy's world frame)
        good_apples, bad_apples = getAppleLocations(sim, CAMERA_HANDLE)

        if not good_apples:
            print("No good apples found → rotating to search...")
            set_velocity(0.0, 0.5)  # Rotate in place (Lua-matched angular speed)
            time.sleep(0.2)
            continue

        # Get current Robot dummy position for distance calculation
        robot_x, robot_y, _ = get_robot_pose()

        # Calculate distance to each apple (Robot dummy frame)
        apple_distances = []
        for apple in good_apples:
            # Apple coordinates MUST be in the same world frame as Robot dummy
            ax, ay, _ = apple
            dx = ax - robot_x
            dy = robot_y - ay
            dist = math.sqrt(dx**2 + dy**2)
            apple_distances.append(dist)

        # Select nearest apple (Robot dummy frame)
        nearest_idx = apple_distances.index(min(apple_distances))
        target_x, target_y, _ = good_apples[nearest_idx]
        print(f"Navigating to nearest apple: (X: {target_x:.2f}, Y: {target_y:.2f})")

        # Navigate using Robot dummy coordinates (Lua-matched logic)
        go_to_target(target_x, target_y)

if __name__ == "__main__":
    try:
        visual_navigation()
    except KeyboardInterrupt:
        stop()
        print("Navigation stopped (Robot dummy frame)")