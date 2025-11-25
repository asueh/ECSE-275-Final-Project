# -*- coding: utf-8 -*-
"""
Created on Mon Nov 24 09:31:27 2025

@author: 11eva
"""
import coppeliasim_zmqremoteapi_client as zmq
import numpy as np
import time
import sys  # <--- FIXED: Needed to use sys.exit()

print("Connecting to CoppeliaSim...")
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

# Wait for sim to start up
time.sleep(1)

# === 1. GET HANDLES ===
try:
    # FIXED: Based on your screenshot, 'visionSensor' is NOT inside IRB140.
    # It is a child of 'robot_base_plate'. 
    # We search for it globally using the slash.
    camera_handle = sim.getObject("/visionSensor") 
    print("✓ Camera Found")
    
    # FIXED: We assume apple_1 is in the world root.
    apple_handle = sim.getObject("/apple_1")
    print("✓ Apple_1 Found")
    
    # Optional: Get robot base
    # robot_handle = sim.getObject("/IRB140")

except Exception as e:
    print("\n!!! ERROR FINDING OBJECTS !!!")
    print("Double check your scene hierarchy names.")
    print(f"Details: {e}")
    sys.exit() # <--- FIXED: Uses sys.exit() instead of exit()

# === 2. GET COORDINATES ===
pos_world = sim.getObjectPosition(apple_handle, -1)
print(f"Apple_1 Position: {pos_world}")

# === 3. SEND COMMAND TO ROBOT ===
# Pack: [x, y, z, handle]
data_to_send = [pos_world[0], pos_world[1], pos_world[2], float(apple_handle)]

sim.setStringSignal("TargetApple", sim.packFloatTable(data_to_send))

print(">> Signal Sent to Robot! Check CoppeliaSim window.")