import time
import coppeliasim_zmqremoteapi_client as zmq

# Global communication variables - Do not modify
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

# Robot physical constants - Do not modify
WHEEL_RADIUS = 0.25       # Wheel radius (meters)
WHEEL_BASE = 1.22         # Distance between wheel centers (meters)
MAX_WHEEL_SPEED = 20.0    # Maximum wheel angular velocity (rad/s)

# Global joint handle variables
LEFT_MOTOR_HANDLE = sim.getObject("/robot_base_plate/left_wheel")
RIGHT_MOTOR_HANDLE = sim.getObject("/robot_base_plate/right_wheel")

# Validate motor handles
if LEFT_MOTOR_HANDLE == -1 or RIGHT_MOTOR_HANDLE == -1:
    raise ValueError(
        "Failed to get wheel joint handles! Check:\n"
        f"Left wheel path: /robot_base_plate/left_wheel\n"
        f"Right wheel path: /robot_base_plate/right_wheel\n"
        "Ensure wheel objects are children of /robot_base_plate"
    )

# Core control functions
def _clamp(value, vmin, vmax):
    """Limit a value within a specified range"""
    return max(vmin, min(value, vmax))

def set_wheel_speed(vL, vR):
    """
    Set target angular velocity for left and right wheels
    
    Args:
        vL (float): Target speed for left wheel (rad/s)
        vR (float): Target speed for right wheel (rad/s)
    """
    vL_clamped = _clamp(vL, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    vR_clamped = _clamp(vR, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
    
    # Critical Fix: Invert ALL speed signs to correct reversed directions
    sim.setJointTargetVelocity(LEFT_MOTOR_HANDLE, -vL_clamped)
    sim.setJointTargetVelocity(RIGHT_MOTOR_HANDLE, -vR_clamped)

def stop():
    """Emergency stop: set both wheel speeds to 0"""
    set_wheel_speed(0.0, 0.0)

def forward(wheel_speed):
    """
    Move robot straight forward
    
    Args:
        wheel_speed (float): Target speed for both wheels (rad/s)
    """
    set_wheel_speed(-wheel_speed, -wheel_speed)

def rotate_in_place(wheel_speed):
    """
    Rotate robot in place
    
    Args:
        wheel_speed (float): Rotation speed (positive = clockwise, negative = counterclockwise) (rad/s)
    """
    set_wheel_speed(-wheel_speed, wheel_speed)  # No change here (relative speed preserved)

def set_velocity(linear_v, angular_w):
    """
    Calculate and set wheel speeds based on linear and angular velocity (differential drive model)
    
    Args:
        linear_v (float): Desired linear velocity (m/s)
        angular_w (float): Desired angular velocity (rad/s)
    """
    vL = (2.0 * linear_v - angular_w * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    vR = (2.0 * linear_v + angular_w * WHEEL_BASE) / (2.0 * WHEEL_RADIUS)
    set_wheel_speed(vL, vR)

def get_actual_wheel_speed():
    """
    Get real-time actual wheel speeds (not target speeds)
    
    Returns:
        tuple: (left_wheel_speed, right_wheel_speed) in rad/s
    """
    # Sync sign inversion to match set_wheel_speed logic (for accurate readings)
    vL_actual = -sim.getJointVelocity(LEFT_MOTOR_HANDLE)
    vR_actual = -sim.getJointVelocity(RIGHT_MOTOR_HANDLE)
    return vL_actual, vR_actual