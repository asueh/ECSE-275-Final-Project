# -*- coding: utf-8 -*-
"""
vision_control.py
Merged User CV code with Signal Sending logic.
"""

import coppeliasim_zmqremoteapi_client as zmq
import cv2 as cv
import numpy as np
import array
import math
import time
import sys

def getAppleLocations(sim, camera_handle):
    '''
    Adapted from user provided code.
    Returns: Good_apple_world (list of [x,y,z]), Bad_apple_world (list of [x,y,z])
    '''
    # Camera Constants
    pixels_per_inch = 560.0165995731867 
    meters_per_pixel = .0254 / pixels_per_inch
    
    # Get Vision Sensor Data
    image_buffer, resolution = sim.getVisionSensorImg(camera_handle)
    img = np.frombuffer(image_buffer, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
    
    # Camera pose data
    camera_matrix = np.array(sim.getObjectMatrix(camera_handle, -1)).reshape(3,4)
    Rot = camera_matrix[:,:3]
    position = camera_matrix[:,3]
    
    # Depth data
    depth_data = sim.getVisionSensorDepth(camera_handle, 1)
    float_array = array.array('f')
    float_array.frombytes(depth_data[0])
    depth_values = np.array(float_array).reshape(resolution[1], resolution[0])
    depth_values = cv.flip(depth_values, 0)
    
    cam_angle = sim.getObjectFloatParam(camera_handle, sim.visionfloatparam_perspective_angle)
    fx = resolution[0] / (2.0 * np.tan(cam_angle / 2.0))
    fy = resolution[1] / (2.0 * np.tan(cam_angle / 2.0) * resolution[1]/resolution[0])
    
    # Flip image
    img = cv.flip(img, 0)
    
    # HSV Thresholding
    hsv_img = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    lower_bound = np.array([0, 65, 65])
    upper_bound = np.array([10, 255, 255])
    mask = cv.inRange(hsv_img, lower_bound, upper_bound)
    
    # Contour Detection
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    approx_polys = []
    for cnt in contours:
        approx_polys.append(cv.approxPolyDP(cnt, 0.035 * cv.arcLength(cnt, True), True))
        
    Bad_apple_polys = []
    Good_apple_polys = []
    
    # Sort Good vs Bad
    for cnt in approx_polys:
        if len(cnt) <= 5:
            Bad_apple_polys.append(cnt)
        elif len(cnt) > 5:
            Good_apple_polys.append(cnt)
            
    # Helper to process centroids and convert to world coords
    def process_polys(polys):
        world_coords = []
        u0 = resolution[0] / 2.0
        v0 = resolution[1] / 2.0
        
        for cnt in polys:
            M = cv.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            
            # Clamp indices
            cX = min(max(cX, 0), resolution[0]-1)
            cY = min(max(cY, 0), resolution[1]-1)

            z = depth_values[cY, cX]
            
            # Camera Frame
            X_cam = (cX - u0) * z * meters_per_pixel / fx
            Y_cam = (cY - v0) * z * meters_per_pixel / fy
            Z_cam = z
            pnt = [X_cam, Y_cam, Z_cam]
            
            # World Frame Rotation
            world_pnt = Rot @ pnt + position
            # Correction for Z (as per your original code)
            world_pnt[2] = position[2] - (cY - v0) * z / fy
            
            world_coords.append(world_pnt)
        return world_coords

    good_world = process_polys(Good_apple_polys)
    bad_world = process_polys(Bad_apple_polys)
    
    return good_world, bad_world

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

# ================= MAIN EXECUTION =================
print("Connecting to CoppeliaSim ZMQ...")
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

# 1. Get Camera Handle
try:
    camera_handle = sim.getObject("/visionSensor")
    print("✓ Camera Found")
except:
    print("! Error: Could not find object named '/visionSensor'")
    sys.exit()

# 2. Run CV
print("Analyzing image...")
good_apples, bad_apples = getAppleLocations(sim, camera_handle)

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
