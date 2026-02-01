"""
Quaternion utilities.
"""

import math
from typing import Tuple


def quat_normalize(w: float, x: float, y: float, z: float) -> Tuple[float, float, float, float]:
    """Normalize quaternion"""
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)


def quat_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Returns angles in radians.
    Convention: ZYX (yaw-pitch-roll)
    """
    # Normalize first
    w, x, y, z = quat_normalize(w, x, y, z)
    
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return (roll, pitch, yaw)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles to quaternion.
    
    Args:
        roll, pitch, yaw: Angles in radians
    
    Returns:
        (w, x, y, z) quaternion
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return quat_normalize(w, x, y, z)
