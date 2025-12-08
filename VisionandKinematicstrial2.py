# -*- coding: utf-8 -*-
"""
UPDATED Vision Controller for Re-Positioned Robot
"""

import coppeliasim_zmqremoteapi_client as zmq
import cv2 as cv
import numpy as np
import array
import math
import time
import sys

# ==========================================
#      PARTNER'S CV FUNCTION (Robustified)
# ==========================================
def getAppleLocations(sim, camera_handle):
    '''
    Calculates World Coordinates based on current Camera Position.
    '''
    # 1. Get Image
    image_buffer, resolution = sim.getVisionSensorImg(camera_handle)
    img = np.frombuffer(image_buffer, dtype = np.uint8).reshape(resolution[1], resolution[0], 3)
    
    # 2. Get Camera REAL-TIME Absolute Position (Handles Robot Movement)
    # This automatically accounts for your new robot base coords [-0.099, etc]
    camera_matrix = np.array(sim.getObjectMatrix(camera_handle, -1)).reshape(3,4)
    Rot = camera_matrix[:,:3]
    cam_pos = camera_matrix[:,3]
    
    # Debug Print to confirm Camera sees the move
    # print(f"DEBUG: Camera is at X={cam_pos[0]:.2f}, Y={cam_pos[1]:.2f}, Z={cam_pos[2]:.2f}")

    orientation = sim.getObjectOrientation(camera_handle, -1)
    beta_deg = math.degrees(orientation[1])
    
    # 3. Depth Data
    depth_data = sim.getVisionSensorDepth(camera_handle, 1)
    float_array = array.array('f')
    float_array.frombytes(depth_data[0])
    depth_values = np.array(float_array).reshape(resolution[1], resolution[0])
    depth_values = cv.flip(depth_values, 0)
    
    # Camera Intrinsics
    cam_angle = sim.getObjectFloatParam(camera_handle, sim.visionfloatparam_perspective_angle)
    fx = resolution[0] / (2.0 * np.tan(cam_angle / 2.0))
    fy = resolution[1] / (2.0 * np.tan(cam_angle / 2.0))
    
    img = cv.flip(img, 0)
    hsv_img = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    
    # 4. HSV Thresholding (Red Apples)
    lower_bound = np.array([0, 65, 65])
    upper_bound = np.array([10, 255, 255])
    mask = cv.inRange(hsv_img, lower_bound, upper_bound)
    
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    approx_polys = []
    for cnt in contours:
        approx_polys.append(cv.approxPolyDP(cnt, 0.035 * cv.arcLength(cnt, True), True))
        
    good_apples = []
    
    # 5. Process Contours
    u0 = resolution[0] / 2.0
    v0 = resolution[1] / 2.0
    
    # Constants for angle compensation
    pixels_per_inch = 560.0165995731867 
    meters_per_pixel = 0.0254 / pixels_per_inch

    for cnt in approx_polys:
        # Filter noise
        if cv.contourArea(cnt) < 20: continue 

        # We assume >5 sides is good, but for now let's just grab ANY apple 
        # to ensure the arm moves. (Uncomment check if needed)
        # if len(cnt) <= 5: continue 
        
        M = cv.moments(cnt)
        if M["m00"] == 0: continue
        
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cX = min(max(cX, 0), resolution[0]-1)
        cY = min(max(cY, 0), resolution[1]-1)
        
        z = depth_values[cY, cX]
        
        # --- Partner's Angle Compensation Logic ---
        if abs(beta_deg) < 20:
            delta_u = (cX - u0) * z * meters_per_pixel / fx
            delta_v = (cY - v0) * z * meters_per_pixel / fx 
            denom = math.sqrt(1 + delta_u**2 + delta_v**2)
            Z_cam = z / denom if denom > 0 else z
            
            X_cam = delta_u * Z_cam - 0.2 - (0.02 * (20 - beta_deg))
            Y_cam = delta_v * Z_cam - 0.2 - (0.02 * (20 - beta_deg))
        else:
            X_cam = (cX - u0) * z * meters_per_pixel / fx
            Y_cam = (cY - v0) * z * meters_per_pixel / fy
            Z_cam = z
            
        # Transform to World Frame
        pnt = np.array([X_cam, Y_cam, Z_cam])
        world_pnt = Rot @ pnt + cam_pos # <--- Uses NEW Robot Position
        
        # Z-correction
        world_pnt[2] = cam_pos[2] - (cY - v0) * z / fy
        
        good_apples.append(world_pnt)
        
    return good_apples

# ==========================================
#      HELPER: MATCH COORD TO HANDLE
# ==========================================
def get_closest_handle(sim, target_pos):
    # INCREASED TOLERANCE to 0.40m because robot moved
    min_dist = 0.40 
    found_handle = -1
    
    # Check all potential apples
    for i in range(40): 
        try:
            obj_name = f"/apple_{i}"
            h = sim.getObject(obj_name)
            pos = sim.getObjectPosition(h, -1)
            
            dist = math.sqrt((pos[0]-target_pos[0])**2 + 
                             (pos[1]-target_pos[1])**2 + 
                             (pos[2]-target_pos[2])**2)
            
            if dist < min_dist:
                min_dist = dist
                found_handle = h
        except:
            pass
            
    return found_handle

# ==========================================
#           MAIN EXECUTION
# ==========================================
print("Connecting to CoppeliaSim...")
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

try:
    # 1. Get Camera
    # Ensure your hierarchy is correct: /visionSensor should be findable
    try:
        camera_handle = sim.getObject("/visionSensor")
    except:
        # Fallback if camera is attached to robot and not found globally
        print("Searching for local visionSensor...")
        camera_handle = sim.getObject("./visionSensor")
        
    print("✓ Camera Found")
    
    # 2. Run CV
    print("Capturing image...")
    found_apples = getAppleLocations(sim, camera_handle)
    
    print(f"✓ CV Detected {len(found_apples)} potential targets.")

    # 3. Send to Robot
    sent_count = 0
    for pos in found_apples:
        handle_id = get_closest_handle(sim, pos)
        
        if handle_id != -1:
            print(f"   -> MATCH! Sending Apple at {pos} (ID: {handle_id})")
            
            data_to_send = [pos[0], pos[1], pos[2], float(handle_id)]
            sim.setStringSignal("TargetApple", sim.packFloatTable(data_to_send))
            
            # Pause to let Lua react
            time.sleep(1.5) 
            sent_count += 1
        else:
            print(f"   -> No Sim Object found near {pos} (Check Tolerance)")

    if sent_count == 0:
        print("No targets sent. Try moving robot slightly or checking lighting.")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()