# -*- coding: utf-8 -*-
"""
Created on Thu Nov 20 13:13:26 2025

@author: acsue
"""

import coppeliasim_zmqremoteapi_client as zmq
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import array

def getAppleLocations():
    '''
        Get camera information to find and sort apples and compute locations
        
        Args: none
        
        Returns:
            Lists: 
                2 lists of floats containing the world coordinates of good and bad apples
                1st list is the coordinates of good apples, where each new row is a new apple, column wise goes [X,Y,Z]
                2nd list is the coordinates of bad apples, where each new row is a new apple, column wise goes [X,Y,Z]
            ex. Good_apple_world_coor = [[X1,Y1,Z1],[X2,Y2,Z2]]
                Bad_apple_world_coor = [[X3,Y3,Z3]]
        How to call:
            Good_apple_world_coor, Bad_apple_world_coor = getAppleLocations()
            
            
        NOTE: APPLE LOCATIONS ARE MOST ACCURATE WHEN ROBOT AND APPLE ARE WITHIN 3.5 m OF EACH OTHER
    '''
    ''' CAMERA CONSTANTS AND SET UP DO NOT MODIFY'''
    # Camera Constants
    pixels_per_inch = 560.0165995731867 
    meters_per_pixel = .0254 / pixels_per_inch
    
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    
    # Define the position of our drop "zone"
    #drop_target = sim.getObject('/drop_target')
    #droppt = sim.getObjectPose(drop_target,-1)
    
    # Get the camera handle
    camera_handle = sim.getObject("/visionSensor")
    
    ''' CODE FOR GETTING THE IMAGE SET UP CORRECTLY FOR CONTOUR DETECTION'''
    # Obtain the image from the image sensor
    image_buffer, resolution = sim.getVisionSensorImg(camera_handle)
    img = np.frombuffer(image_buffer, dtype = np.uint8).reshape(resolution[1], resolution[0], 3)
    
    # Camera pose data for later, put here for least amount of delay
    camera_matrix = np.array(sim.getObjectMatrix(camera_handle, -1)).reshape(3,4)
    Rot = camera_matrix[:,:3]
    position = camera_matrix[:,3]
    # Depth data for later, put here for least amount of delay
    depth_data = sim.getVisionSensorDepth(camera_handle, 1)
    float_array = array.array('f')
    float_array.frombytes(depth_data[0])
    depth_values = np.array(float_array).reshape(resolution[1], resolution[0])
    depth_values = cv.flip(depth_values, 0)
    
    cam_angle = sim.getObjectFloatParam(camera_handle, sim.visionfloatparam_perspective_angle)
    fx = resolution[0] / (2.0 * np.tan(cam_angle / 2.0))
    fy = resolution[1] / (2.0 * np.tan(cam_angle / 2.0) * resolution[1]/resolution[0])
    # We need to flip the image to align the horizontal axis with the camera frame
    img = cv.flip(img, 0)
    
    # Convert image to HSV
    hsv_img = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    
    ''' HSV Thresholding 
    Uses HSV thresholding rather than grayscale so that it can ignore the leaves of the tree and the manipulator when thresholding, rather than when doing polygon estimation.
    HSV thresholding allows for easier thresholding against different backgrounds, and makes it easier to single out one color.
    In this case, we are assuming all apples are bright red, as the majority of ripe apples are red, and only a few species are a different color.
    
    Basically, the HSV thresholding allows us to do find only the apples and ignore everything else.
    '''
    
    lower_bound = np.array([0, 65, 65])
    upper_bound = np.array([10, 255, 255])
    
    mask = cv.inRange(hsv_img, lower_bound, upper_bound)
    
    ''' Contour Detection'''
    # Finds contours of objects
    contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    # Display images in Python plots tab
    img_contours = cv.drawContours(img, contours, -1, (0,0,255), 2)
    fig, (ax, ax2) = plt.subplots(ncols=2)
    
    ''' Polygon Estimation
    
    Approximates a polygon curve by allowing a maximum value between the original contour and the approximated version.
    In this case, the approximation accuracy is 3.5% of the arcLength of the original contour.
    It returns a list of verticies.
    
    '''
    approx = []
    
    for cnt in contours:
        approx.append(cv.approxPolyDP(cnt, 0.035 * cv.arcLength(cnt, True), True))
    poly_img = cv.drawContours(img, approx, -1, (255, 255, 0), 2)
    ax.imshow(poly_img)
    
    ''' Sorting between good and bad apples 
    
    By using the number of verticies in each approximated polygon, we can decide if it is a good or bad apple.
    In this case, if it has less than 5 sides, it is considered a bad apple, and if it has more than 5, it is a good apple.
    '''
    Bad_apple_polys = []
    Good_apple_polys = []
    
    # sorts the polygon coordinates into good or bad apples
    for cnt in approx:
        # Code for bad apples 
        if len(cnt) <= 5:
            Bad_apple_polys.append(cnt)
        # Code for good apples
        elif len(cnt) > 5:
            Good_apple_polys.append(cnt)
    
    ''' Centroid and Depth Computing '''
    Bad_apple_centroid = []
    Bad_apple_depth =[]
    # Centroid for bad apples
    for cnt in Bad_apple_polys:
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        Bad_apple_centroid.append([cX,cY])
        centroid_depth = depth_values[cY,cX]
        Bad_apple_depth.append(centroid_depth)
        cv.circle(img_contours, (cX, cY), 2, (0, 255, 255), -1)
        
    Good_apple_centroid = []
    Good_apple_depth = []
    # Centroid for good apples
    for cnt in Good_apple_polys:
        M = cv.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        Good_apple_centroid.append([cX, cY])
        centroid_depth = depth_values[cY,cX]
        Good_apple_depth.append(centroid_depth)
        cv.circle(img_contours, (cX, cY), 1, (0, 255, 255), -1)
    
    ax2.imshow(img_contours)
    
    
    ''' Position from Centroid Computing '''
    # get image center
    
    u0 = resolution[0] / 2.0
    v0 = resolution[1] / 2.0
    
    # Get coordinates in camera frame
    Bad_apple_cam = []
    for idx in range(len(Bad_apple_depth)):
        u = Bad_apple_centroid[idx][0]
        v = Bad_apple_centroid[idx][1]
        z = Bad_apple_depth[idx]
        X_cam = (u - u0) * z * meters_per_pixel / fx
        Y_cam = (v - v0) * z * meters_per_pixel / fy
        Z_cam = z
        Bad_apple_cam.append([X_cam, Y_cam, Z_cam])
    
    
    Good_apple_cam = []
    for idx in range(len(Good_apple_depth)):
        z = Good_apple_depth[idx]
        u = Good_apple_centroid[idx][0]
        v = Good_apple_centroid[idx][1]
        X_cam = (u - u0) * z * meters_per_pixel / fx
        Y_cam = (v - v0) * z * meters_per_pixel/ fy
        Z_cam = z
        Good_apple_cam.append([X_cam, Y_cam, Z_cam])
    
    # Get world coordinates from camera coordinates
    Bad_apple_world = []
    for idx, pnt in enumerate(Bad_apple_cam):
        #Rotation to convert camera coordinates to world coordinates
        Bad_apple_world.append(Rot @ pnt + position)
        #Computes the actual z coordinate as the z coordinate is the only one that is fixed on the camera
        Bad_apple_world[idx][2] = position[2] - (v - v0) * z / fy
        
    Good_apple_world = []
    for idx, pnt in enumerate(Good_apple_cam):
        #Rotation to convert camera coordinates to world coordinates
        Good_apple_world.append(Rot @ pnt + position)
        #Computes the actual z coordinate as the z coordinate is the only one that is fixed on the camera
        Good_apple_world[idx][2] = position[2] - (v - v0) * z / fy
    
    return Good_apple_world, Bad_apple_world