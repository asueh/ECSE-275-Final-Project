# -*- coding: utf-8 -*-
"""
CLEAN VISION CONTROLLER
Functionality:
1. Connects to CoppeliaSim
2. Uses Vision Sensor to find 'Good Apples'
3. Matches coordinates to physical object Handles
4. Sends the data to the Robot and disconnects.
"""

import coppeliasim_zmqremoteapi_client as zmq
import cv2 as cv
import numpy as np
import array
import math
import sys
import time

# === CONNECTION SETUP ===
print("Connecting to CoppeliaSim...")
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

try:
    camera_handle = sim.getObject("/visionSensor")
    print("✓ Vision Sensor Connected")
except:
    print("!!! Error: /visionSensor not found in scene.")
    sys.exit()

# =========================================================================
# PARTNER'S VISION CODE (Core Logic)
# =========================================================================
def getAppleLocations(sim, camera_handle):
    # Camera Constants
    pixels_per_inch = 560.0165995731867 
    meters_per_pixel = 0.0254 / pixels_per_inch
    
    # Obtain image
    image_buffer, resolution = sim.getVisionSensorImg(camera_handle)
    img = np.frombuffer(image_buffer, dtype = np.uint8).reshape(resolution[1], resolution[0], 3)
    
    # Camera Data
    camera_matrix = np.array(sim.getObjectMatrix(camera_handle, -1)).reshape(3,4)
    Rot = camera_matrix[:,:3]
    position = camera_matrix[:,3]
    
    orientation = sim.getObjectOrientation(camera_handle, -1)
    beta_deg = math.degrees(orientation[1])
    
    # Depth Data
    depth_data = sim.getVisionSensorDepth(camera_handle, 1)
    float_array = array.array('f')
    float_array.frombytes(depth_data[0])
    depth_values = np.array(float_array).reshape(resolution[1], resolution[0])
    depth_values = cv.flip(depth_values, 0)
    
    cam_angle = sim.getObjectFloatParam(camera_handle, sim.visionfloatparam_perspective_angle)
    fx = resolution[0] / (2.0 * np.tan(cam_angle / 2.0))
    fy = resolution[1] / (2.0 * np.tan(cam_angle / 2.0))
    img = cv.flip(img, 0)
    hsv_img = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    
    # Thresholding
    lower_bound = np.array([0, 65, 65])
    upper_bound = np.array([10, 255, 255])
    mask = cv.inRange(hsv_img, lower_bound, upper_bound)
    contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    approx = []
    for cnt in contours:
        approx.append(cv.approxPolyDP(cnt, 0.035 * cv.arcLength(cnt, True), True))

    Bad_apple_polys = []
    Good_apple_polys = []
    
    for cnt in approx:
        if len(cnt) <= 5:
            Bad_apple_polys.append(cnt)
        elif len(cnt) > 5:
            Good_apple_polys.append(cnt)
    
    Bad_apple_centroid = []
    Bad_apple_depth =[]
    for cnt in Bad_apple_polys:
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        Bad_apple_centroid.append([cX,cY])
        if cY < depth_values.shape[0] and cX < depth_values.shape[1]:
             centroid_depth = depth_values[cY,cX]
        else:
             centroid_depth = 0
        Bad_apple_depth.append(centroid_depth)
        
    Good_apple_centroid = []
    Good_apple_depth = []
    for cnt in Good_apple_polys:
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        Good_apple_centroid.append([cX, cY])
        if cY < depth_values.shape[0] and cX < depth_values.shape[1]:
            centroid_depth = depth_values[cY,cX]
        else:
            centroid_depth = 0
        Good_apple_depth.append(centroid_depth)
    
    # Coordinate Calculation
    u0 = resolution[0] / 2.0
    v0 = resolution[1] / 2.0
    
    Bad_apple_cam = []
    Bad_apple_world_z =[]
    for idx in range(len(Bad_apple_depth)):
        u = Bad_apple_centroid[idx][0]
        v = Bad_apple_centroid[idx][1]
        z = Bad_apple_depth[idx]
        if beta_deg < 20:
            delta_u = (u - u0) * z * meters_per_pixel / fx
            delta_v = (v - v0) * z * meters_per_pixel/ fx
            Z_cam = z / math.sqrt(1 + delta_u*delta_u + delta_v*delta_v)
            X_cam = delta_u * Z_cam - 0.2 - (0.02 * (20 - beta_deg))
            Y_cam = delta_v * Z_cam - 0.2 - (0.02 * (20 - beta_deg))
        else: 
            X_cam = (u - u0) * z * meters_per_pixel / fx
            Y_cam = (v - v0) * z * meters_per_pixel / fy
            Z_cam = z
        Bad_apple_cam.append([X_cam, Y_cam, Z_cam])
        Bad_apple_world_z.append(position[2] - (v - v0) * z / fy)
    
    Good_apple_cam = []
    Good_apple_world_z =[]
    for idx in range(len(Good_apple_depth)):
        u = Good_apple_centroid[idx][0]
        v = Good_apple_centroid[idx][1]
        z = Good_apple_depth[idx]
        if abs(beta_deg) < 20:
            delta_u = (u - u0) * z * meters_per_pixel / fx
            delta_v = (v - v0) * z * meters_per_pixel/ fx
            Z_cam = z / math.sqrt(1 + delta_u*delta_u + delta_v*delta_v)
            X_cam = delta_u * Z_cam - 0.2 - (0.02 * (20 - beta_deg))
            Y_cam = delta_v * Z_cam - 0.2 - (0.02 * (20 - beta_deg))
        else: 
            X_cam = (u - u0) * z * meters_per_pixel / fx
            Y_cam = (v - v0) * z * meters_per_pixel / fy
            Z_cam = z
        Good_apple_cam.append([X_cam, Y_cam, Z_cam])
        Good_apple_world_z.append(position[2] - (v - v0) * z / fy)
    
    Bad_apple_world = []
    for idx, pnt in enumerate(Bad_apple_cam):
        Bad_apple_world.append(Rot @ pnt + position)
        Bad_apple_world[idx][2] = Bad_apple_world_z[idx]
        
    Good_apple_world = []
    for idx, pnt in enumerate(Good_apple_cam):
        Good_apple_world.append(Rot @ pnt + position)
        Good_apple_world[idx][2] = Good_apple_world_z[idx]
        
    return Good_apple_world, Bad_apple_world

# =========================================================================
# HELPER: MATCH VISION COORDS TO SIMULATION HANDLES
# =========================================================================
def find_handle_by_proximity(target_pos, tolerance=0.5): 
    # Get all shapes in scene
    all_shapes = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    
    closest_handle = -1
    min_dist = 9999
    
    for handle in all_shapes:
        try:
            obj_pos = sim.getObjectPosition(handle, -1)
            dist = math.sqrt((obj_pos[0]-target_pos[0])**2 + 
                             (obj_pos[1]-target_pos[1])**2 + 
                             (obj_pos[2]-target_pos[2])**2)
            
            if dist < min_dist and dist < tolerance:
                min_dist = dist
                closest_handle = handle
        except:
            continue
            
    return closest_handle

# =========================================================================
# MAIN EXECUTION
# =========================================================================
print("\n--- PHASE 1: VISION SCAN ---")

good_apples, bad_apples = getAppleLocations(sim, camera_handle)

print(f"Vision Sensor found {len(good_apples)} good apples.")

if len(good_apples) == 0:
    print("No good apples found. Exiting.")
    sys.exit()

mission_payload = []
for pos in good_apples:
    # Match the coordinate to the physical object handle
    handle = find_handle_by_proximity(pos, tolerance=0.5) 
    
    if handle != -1:
        print(f"✓ Matched Coord {pos} -> Handle {handle}")
        mission_payload.extend([pos[0], pos[1], pos[2], float(handle)])
    else:
        print(f"⚠ Could not find physical object at {pos}")

if len(mission_payload) == 0:
    print("Error: Could not match any vision coordinates to physical objects.")
    sys.exit()

print(f"\nSending {int(len(mission_payload)/4)} targets to Robot...")
sim.setStringSignal("MissionData", sim.packFloatTable(mission_payload))

print("Data Sent. Python Disconnecting.")