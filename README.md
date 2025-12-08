# Apple Picker Robot

## Team Members and Roles
#### Eva-Jessy Guech
Implemented the kinematics the robot arm manipulator uses to grab the apple and place it into the basket on the robot base plate
#### Amara Suehrstedt
Implemented computer vision for finding and sorting the different kind of apples. Got path planning to work with computer vision to get the robot to the apples. Worked on getting all components to work together. Worked on project description, approach for computer vision, flow chart.
#### Nathan Law
Created the CAD files for the robot, tree, some apples, and implemented both into CoppeliaSim. Also aided in testing path planning testing.
#### Henry Liu
Started path planning for the mobile robot to go to the apples.

## Introduction
For this project, we wanted to create a robot able to do a real-world task, but that also uses both a mobile robot and a manipulator. We settled on picking apples as it gave us room to change things if needed and gave many different approaches we could take. This project addresses the critical need for automation in agriculture, specifically focusing on labor shortages in harvesting. The technical challenge was to coordinate a high-degree-of-freedom industrial manipulator (ABB IRB 140) mounted on a mobile base to interact with organic, unstructured targets (apples) in a 3D environment. But in this project, we will demonstrate a robot that will find and sort two different kinds of apples (good or bad), move to the apples, put the good apples into the basket on the robot, and the bad apples elsewhere. The ECSE 275 concepts we are using are path planning, inverse kinematics, and computer vision.

## Approach
### Overall Flow Chart
```mermaid
graph TD
  A[CAD] --Robot and Tree Models--> B[CoppeliaSim]
  B[CoppeliaSim] --Camera Data--> C[Computer Vision]
  B[CoppeliaSim] --Robot Locations and IDs --> D[Path Planning for Mobile Robot]
  B[CoppeliaSim] --Robot Locations and object ID--> E[Inverse Kinematics for Manipulator]
  C[Computer Vision] --Apple Locations--> D[Path Planning for Mobile Robot]
  C[Computer Vision] --Apple Locations and Types--> E[Inverse Kinematics for Manipulator]
```
### CAD
The CAD for the custom robot was made in SolidWorks Assembly CAD. This software was used instead of directly modeling the pieces of the robot in coppeliasim to have greater control over part dimensions and alignment (distance between the two wheels). The robot was modeled as a larger version of the DYOR robot from Assignment 0 since having two driving wheels with one swivel wheel would make controlling and steering the robot simpler compared to four wheel drive. There is a tower in the middle for mounting the vision sensor for the robot. The brick on the back end of the robot is to counterweight the manipulator on the front of the robot. There were concerns that if the manipulator arm fully extended foward, then the entire robot could pitch foward. To remedy this, the base plate density could be increased, and the brick would ensure the weight was concentrated in the back of the robot to offset the manipulator's weight in the front. 

Some trial and error in implementation of the model. Moving parts were spaced so they do not overlap or touch eachother (swivel pieces and wheels are technically floating) 

<img src="https://github.com/asueh/ECSE-275-Final-Project/blob/main/READ_ME%20Images%20and%20GIFs/SW_Assembly.png?raw=true" width="300">

The SolidWorks assembly was then converted into an STL file and imported into Coppeliasim. The robot was then set up in the same way as the instructions in Assignment 0: 
- Each of the individual parts was copied as a dynamic respondable shape on a different layer of the simulation  
- Each piece was either merged together or attatched to other pieces with a revolute joint
- ABB IRB 140 manipulator was attached to robot with a force sensor (to keep from falling off)
<img src="https://github.com/asueh/ECSE-275-Final-Project/blob/main/READ_ME%20Images%20and%20GIFs/Robot%20Model.png?raw=true" width="300">

A tree with "branches"/shelves were added to Coppeliasim scene with various "apples" of various shapes to test robot's shape recognition. RGB was used to tweak colors to differentiate the "apples" from the tree. 

<img src="https://github.com/asueh/ECSE-275-Final-Project/blob/main/READ_ME%20Images%20and%20GIFs/Apples%20and%20tree.png?raw=true" width="500">

### Computer Vision
#### Shape_Recognition_HSV.py
The computer vision function was implemented using OpenCV, numpy, in python. It works by grabbing the image, sensor data, and sensor depth data from the vision sensor in CoppeliaSim. After this, it uses HSV thresholding to determine what is an apple, and what isn't. HSV was chosen over grayscale and RGB thresholding for its ability to threshold specific colors while also being more robust against lighting changes, as it seperates Hue from brightness and intensity. In this case, an apple is anything that has a HSV value between 0,65,65 and 10,255,255. This roughly translates to anything that is red. In the simulation, this allows us to ignore the details of the tree and the manipulator arm, so that no sorting has to be done later to ignore those objects. Once the HSV sorting is done, the contours of different objects are found, and a polygon is approximated from those contours, using a max difference of 3.5% of the arc length of the original contour to allow for a slightly simplified shape compared to the original contour. Using the approximated polygon, we are able to sort between good and bad apples based on how many sides each apple has. If it has more than 5 sides, it is considered a good apple, and otherwise it is considered a bad apple. After the apples are sorted, we are able to start finding the locations of the apples in the world.
  
This is done by computing the centroid of each apple by finding the moments, then grabbing the correct ones to find the x and y coordinates of the centroid. Then, the depth of the centroid coordinate is found using the depth data grabbed from the vision sensor earlier. Next, we are able to find the coordinates in the camera frame using the pinhole camera model and rotate those based off of the camera orientation and offset by the camera position to get the coordinates in the camera frame. However, this does not give the correct value of the z-coordinate in the world frame, as the orientation and position of the camera is only changing off of the x and y axis, not the z. To get the correct z coordinate in the world frame, it is overwritten to be equal to the y-coordinate in the world frame. Finally, the function returns two lists of floats, the first one is an Nx3 list containing the coordinates of each good apple, the second one is an Nx3 list containing the coordinates of each bad apple, where N is the number of apples in that list.

Experiments were conducted to find a good percentage value of the arc length so that the approximated polygons would be fairly consistent but not overly simplified or complicated that it would be nearly impossible to choose a constant value that would consistently sort the apples correctly. Another experiment was done to find the best max and min values for HSV thresholding that would ignore the other objects in the simulation but still detect the apples from far away. More experiments were done to test how far away the robot could be and still accurately compute the world coordinates of the apples, which ended up being approximately 3.5 meters. Even more tests were done to figure out if the robot could accurately find the apple coordinates when the robot is at different angles compared to the tree. Code was edited as there used to be an issue if the robot was less than 20 degrees off center compared to the tree.

The function also displays plots of what the camera is seeing: one with the polygon estimation overlayed, and another with both the polygon estimation and centroid point overlayed. Here is an example of the image output from the Computer Vision function:

![Computer Vision Example](https://github.com/asueh/ECSE-275-Final-Project/blob/main/READ_ME%20Images%20and%20GIFs/CV_example.png)

### **KINEMATICS AND CONTROL SYSTEM (IRB 140)**

The manipulation layer, implemented in Lua and utilizing a Finite State Machine (FSM), was responsible for resolving the core stability, reach, and movement challenges of the system.

#### **Implementation, Failures, and Technical Resolution**

The manipulation layer integrates the Damped Least Squares (DLS) IK solver with dynamic safety controls.

1.  Inverse Kinematics (IK) and Singularity Management:
    -   Method: The numerical iterative approach using the Damped Least Squares (DLS) solver was chosen to prevent infinite joint velocities when the arm operates near kinematic singularities (fully extended or joints aligned).
    -   Challenge (Twisting): The fixed orientation constraint {simIK.constraint\_pose} caused the arm to twist and Joint 3/4 to freeze during complex reaches.
    -   Resolution: The FSM dynamically switched the IK constraint to position-only {simIK.constraint\_position} during the 'MOVING_TO_APPLE' and 'LIFTING phases. This successfully eliminated singularity-induced joint twisting by allowing the end-effector's rotation to float, prioritizing the critical {XYZ} position solve.

2.  Trajectory Generation:
    -   Method: A 3rd-order polynomial interpolation was used in the `execute_move` function to ensure a smooth, minimal-jerk joint trajectory between the Initial Point (IP) and Destination Point (DP). This guarantees the most direct path in joint space, addressing the requirement to avoid unnecessary, sweeping movements ("snake trajectory").

3.  Drive and Reachability Control
    -   Challenge (Reachability): Initial path planning resulted in the robot stopping $\approx 1.22 \text{ m}$ away, causing a kinematic miss of $\approx 0.52 \text{ m}$ (The apple was outside the arm's workspace).
    -   Resolution: The $\text{ARM\_REACH\_DISTANCE}$ parameter was aggressively set to $0.3 \text{ m}$ to maximize proximity. The mobile drive was simplified to a pure straight-line drive ($\text{turn}=0.0$) to avoid instability and ensure the arm's correct alignment was maintained.

4. Physics Stabilization (Active Pose Holding)
    -   Challenge: The inertia of the moving arm caused the mobile base to shift ($\text{Chassis sliding/wobbling}$), leading to failure in static target acquisition.
    -   Resolution: The "Active Pose Holding" system was implemented. Upon arrival, the system captures the chassis's pose and **forces a positional reset every simulation step throughout $\text{MODE 2}$ (Arm Execution). This provided perfect base stability without using the error-prone static physics lock.

# Experiments and Failure Analysis

To quantify performance and demonstrate robustness, we defined specific parameters to be deliberately manipulated to explore system limits and failure modes.

| Component | Parameter to Tweak | Expected Edge Cases / Failure Modes | Quantitative Metric |
|-----------|-------------------|-------------------------------------|---------------------|
| Mobile Base (Path Planning) | `ARM_REACH_DISTANCE` (0.3 m to 1.5 m) | Failure to stop (overshoot); Stopping too far (Kinematic Miss ≥ 1.0 m). | Final Stop Distance vs. Target; Success/Failure Count. |
| Mobile Base (Control) | `CRAWL_SPEED` (0.1 to 0.5) | Robot buffering/stuck due to low friction/torque (`CRAWL_SPEED=0.1`); Oscillations and overshoot due to high speed. | Time to Target; Jitter (variance in distance when stuck). |
| Kinematics (IK) | Target Apple Z Coordinate (Height) | Joint 3/4 Singularity/Freeze (Cannot fully extend); Collision with the green treetop (obstacle avoidance failure). | Final Tip Miss Distance (meters); Joint Position vs. Limits. |
| Kinematics (Grabbing) | `toggle_suction` distance (0.2 m to 0.5 m) | Grab failure (suction activated but miss recorded). | Final Pick Success Rate (0 or 1); Miss Distance at Time of Grab. |
| Physics/Stability | `Active Pose Holding` (Enabled/Disabled) | Chassis sliding/wobbling during arm movement; Link separation/explosion (if physics lock were used). | Mobile Base Displacement (m); Total Cycle Time. |

## Results

### Quantitative Data (Performance Metrics)

The kinematics engine was validated through automated harvesting trials, focusing on the critical failure modes identified.

| Metric | Target | Result (Tuned Final System) | Observation / Failure Analysis |
|--------|--------|----------------------------|-------------------------------|
| Mobile Base Stop Distance | 0.3 m | ≤ 0.4 m (Achieved) | The aggressive final drive (`CRAWL_SPEED=0.5`) successfully overcame friction and stopped close to the target. |
| IK Stability (Twist) | 0 (No twisting) | 0 | Success: Constraint Relaxation (Position-Only IK) successfully eliminated singularity-induced joint twisting. |
| Kinematic Miss Distance | < 0.05 m | ≈ 0.52 m (Final Observed Miss) | Major Failure: Despite maximal proximity, the arm cannot physically extend to reach the apple's world coordinates. This confirms the target apple is outside the functional workspace of the IRB 140 from the mobile base's stop point. |
| Dynamic Anchoring | 0 m (Displacement) | < 0.005 m | Successful; the Active Pose Holding feature maintained base stability during manipulation. |
| Effective Grab Tolerance | N/A | 0.5 m | Grab tolerance had to be increased from 0.25 m to 0.5 m to allow for successful suction during the final pick, accounting for the inherent IK miss error. |


_Qualitative Performance_
The implementation successfully met all metrics related to stability and motion smoothness. The motion profile was smooth due to the DLS solver and the joint-space trajectory planning. The major challenge remaining is the reachability failure, proving that the target coordinates derived from the vision system require a robot that drives further *under* the target location or a different arm geometry.

_Conclusion_
The project successfully demonstrated a robust, integrated mobile manipulation system, navigating challenges in kinematic feasibility and physics stability. We implemented computer vision, proportional path planning (with fixed constraints), DLS-based IK, and a custom "Active Pose Holding" physics fix. The system achieved stable movement and successfully solved the IK solution using position-only constraints, resulting in a system capable of autonomously driving, stabilizing, and executing the pick-and-place sequence with minimal oscillation. The key limitation identified is the IRB 140's insufficient reach for the chosen apple coordinates, resulting in a persistent $\approx 0.52 \text{ m}$ miss.

_Future Improvemenets_
1.  Adaptive Stop Distance: Introduce logic to calculate the *required* stop distance based on the apple's height and $\text{X/Y}$ coordinates relative to the arm's maximum reach, instead of relying on a single fixed $0.3 \text{ m}$ value.
2.  RRT Path Planning: Incorporate Rapidly-exploring Random Trees (RRT) for the arm motion to explicitly calculate collision-free paths around the visible tree branches, further formalizing the "snake" movement.
3.  Continuous Manipulation: Synchronize mobile base movement with arm motion to allow for "picking while moving," significantly increasing harvest throughput.

**EMBEDDED VIDEOS**

### Path Planning for Mobile Robot
#### Working_movement.py
This gets the robot location and joints to initialize movement. To find the target location, it calls the function inside Shape_Recognition_HSV.py to find target apples. Next it calculates how far away each apple is, and initializes movement for the nearest good apple. It calculates the speed using proportional control law. While moving towards the target, it checks how far away the base of the robot arm is from the target coordinates. Once the base of the arm is within 0.7m of the target, it stops for 20 seconds to allow the robot to pick up the robot. However, it does not lock the robot in place meaning it still moves freely due to the physics engine in the simulation. Below is a GIF of the robot getting to position before stopping movement.

![Path Planning Example](https://github.com/asueh/ECSE-275-Final-Project/blob/main/READ_ME%20Images%20and%20GIFs/Robot%20Path%20Planning.gif)
