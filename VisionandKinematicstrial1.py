# -*- coding: utf-8 -*-
"""
Created on Thu Dec  4 12:33:55 2025

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
    camera_handle = sim.getObject("/visionSensor") 
    print("✓ Camera Found")
    
    # FIXED: We assume apple_1 is in the world root.
    apple_handle = sim.getObject("/apple_1")
    print("✓ Apple_1 Found")
    

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