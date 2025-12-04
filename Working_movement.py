# -*- coding: utf-8 -*-
"""
Created on Thu Dec  4 01:34:53 2025

@author: acsue
"""

import time
import math
import coppeliasim_zmqremoteapi_client as zmq
import Shape_Recognition_HSV as CV
from robot_control import set_velocity, stop
# ==========================================================
#  Connect to simulator
# ==========================================================
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

print("Connected to CoppeliaSim!")


# ==========================================================
#  Get object handles
# ==========================================================
left_motor = sim.getObject("/right_wheel_joint")
right_motor = sim.getObject("/left_wheel_joint")
robot = sim.getObject("/Robot")
camera_handle = sim.getObject("/visionSensor")
arm = sim.getObject("/IRB140")

# ==========================================================
#  Helper functions
# ==========================================================
def get_pose():
    """Returns (x, y, yaw) of robot in world frame."""
    pos = sim.getObjectPosition(robot, -1)       # [x, y, z]
    ori = sim.getObjectOrientation(robot, -1)    # [α, β, γ] yaw = γ
    return pos[0], pos[1], ori[2]

def match_coord_to_handle(sim, target_coords):
    '''
    The Robot needs a specific Object Handle (ID) to grab the apple.
    This function checks which simulation object is closest to the 
    coordinates detected by CV.
    '''
    closest_handle = -1
    min_dist = 0.1 # Tolerance of 10cm
    
    # Search objects named "apple_0" to "apple_20" (Adjust range if needed)
    # Or search all objects in a specific tree if you have a tree handle
    for i in range(50): 
        try:
            # Note: Ensure your apples in Sim are named "apple_0", "apple_1", etc.
            # or change this string to match your naming convention (e.g. "Apple", "Apple0", etc)
            obj_name = f"/apple_{i}" 
            h = sim.getObject(obj_name)
            
            pos = sim.getObjectPosition(h, -1)
            dist = math.sqrt(
                (pos[0] - target_coords[0])**2 + 
                (pos[1] - target_coords[1])**2 + 
                (pos[2] - target_coords[2])**2
            )
            
            if dist < min_dist:
                min_dist = dist
                closest_handle = h
        except:
            pass # Object name doesn't exist
            
    return closest_handle

def go_to_point(x_goal, y_goal, z_goal):
    """
    Move robot to (x_goal, y_goal) using proportional control.
    """
    print(f"\n→ Going to: ({x_goal:.2f}, {y_goal:.2f})")

    while True:
        x, y, yaw = get_pose()
        
        arm_pos = sim.getObjectPosition(arm, -1)
        arm_dx = x_goal - arm_pos[0]
        arm_dy = y_goal - arm_pos[1]
        arm_dz = z_goal - arm_pos[2]
        # Compute distance & angle to target
        dx = x_goal - x
        dy = y_goal - y
        distance = math.sqrt(dx*dx + dy*dy)
        distance_to_arm = math.sqrt(arm_dx*arm_dx + arm_dy*arm_dy + arm_dz*arm_dz)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - yaw
        
        # Normalize angle to [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Stop when close enough
        if distance_to_arm < 0.7:
            sim.setJointTargetVelocity(left_motor, 0)
            sim.setJointTargetVelocity(right_motor, 0)
            print("   ✔ Reached target.")
            
            ''' ARM CODE - ARM NOT WORKING
            # run Arm
            # 2. Run CV
            print("Analyzing image...")
            good_apples, bad_apples = CV.getAppleLocations(sim, camera_handle)

            print(f"Found {len(good_apples)} Good Apples.")

            # 3. Send Data to Lua
            count = 0
            for apple_pos in good_apples:
                # We must find the handle ID so the Lua script knows what to attach the suction to
                handle_id = match_coord_to_handle(sim, apple_pos)
                
                if handle_id != -1:
                    print(f"Sending Target: {apple_pos} (ID: {handle_id})")
                    
                    # Packet structure: [X, Y, Z, HANDLE_ID]
                    data_packet = [apple_pos[0], apple_pos[1], apple_pos[2], float(handle_id)]
                    
                    # Send signal
                    sim.setStringSignal("TargetApple", sim.packFloatTable(data_packet))
                    
                    # Small sleep to allow Lua to process the queue (CoppeliaSim signals are overwritten if sent too fast)
                    time.sleep(0.5) 
                    count += 1
                else:
                    print(f"Warning: CV saw apple at {apple_pos} but no Simulation Object matched nearby.")
            
            
            
            print(f"Sent {count} targets to Robot.")
            '''
            time.sleep(20)
            break
        
        # --------------------------------------------------
        #  Control law (proportional)
        # --------------------------------------------------
        K_linear = 1.0
        K_angular = 2.0

        v = K_linear * distance
        w = K_angular * angle_error

        # --------------------------------------------------
        # Convert (v, w) to wheel velocities
        # --------------------------------------------------
        wheel_separation = 1.27   # meters
        wheel_radius = 0.49975       # meters

        v_left = (v - w * wheel_separation/2) / wheel_radius
        v_right = (v + w * wheel_separation/2) / wheel_radius

        # Limit speeds
        v_left = max(min(v_left, 4.0), -4.0)
        v_right = max(min(v_right, 4.0), -4.0)

        # Send velocity commands
        sim.setJointTargetVelocity(left_motor, v_left)
        sim.setJointTargetVelocity(right_motor, v_right)
        print(distance)
        print()
        time.sleep(0.02)

def visual_navigation():
    """Main loop: Detect apples → Navigate to nearest good apple (Robot dummy frame)"""
    print("Starting navigation (using Robot dummy for coordinates)...")
    while True:
        # Get apple positions (ensure they're in Robot dummy's world frame)
        good_apples, bad_apples = CV.getAppleLocations(sim, camera_handle)

        if not good_apples:
            print("No good apples found → rotating to search...")
            sim.setJointTargetVelocity(left_motor, 0)
            sim.setJointTargetVelocity(right_motor, 0.5)
            time.sleep(0.2)
            continue

        # Get current Robot dummy position for distance calculation
        robot_x, robot_y, _ = get_pose()

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
        target_x, target_y, target_z = good_apples[nearest_idx]
        print(f"Navigating to nearest apple: (X: {target_x:.2f}, Y: {target_y:.2f})")

        # Navigate using Robot dummy coordinates (Lua-matched logic)
        go_to_point(target_x, target_y, target_z)
# ==========================================================
#  MAIN LOOP – list of waypoints
# ==========================================================
if __name__ == "__main__":
    try:
        visual_navigation()
    except KeyboardInterrupt:
        stop()
        print("Navigation stopped (Robot dummy frame)")
