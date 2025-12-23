"""
Coordinate transformations for visualization

Quaternion to rotation matrix and related transforms
"""

import numpy as np
from ..data.sensor_data import Quaternion


def quaternion_to_rotation_matrix(q):
    """
    Convert quaternion to 3x3 rotation matrix

    Ports: quad_to_rotation from visualize.c

    Args:
        q: Quaternion (q0=w, q1=x, q2=y, q3=z)

    Returns:
        ndarray: 3x3 rotation matrix (row-major, for OpenGL column-major use transpose)
    """
    qw = np.float32(q.q0)
    qx = np.float32(q.q1)
    qy = np.float32(q.q2)
    qz = np.float32(q.q3)

    # Calculate rotation matrix elements
    # This is the standard quaternion to rotation matrix conversion
    rmatrix = np.zeros((3, 3), dtype=np.float32)

    rmatrix[0, 0] = np.float32(1.0) - np.float32(2.0) * qy * qy - np.float32(2.0) * qz * qz
    rmatrix[0, 1] = np.float32(2.0) * qx * qy - np.float32(2.0) * qz * qw
    rmatrix[0, 2] = np.float32(2.0) * qx * qz + np.float32(2.0) * qy * qw

    rmatrix[1, 0] = np.float32(2.0) * qx * qy + np.float32(2.0) * qz * qw
    rmatrix[1, 1] = np.float32(1.0) - np.float32(2.0) * qx * qx - np.float32(2.0) * qz * qz
    rmatrix[1, 2] = np.float32(2.0) * qy * qz - np.float32(2.0) * qx * qw

    rmatrix[2, 0] = np.float32(2.0) * qx * qz - np.float32(2.0) * qy * qw
    rmatrix[2, 1] = np.float32(2.0) * qy * qz + np.float32(2.0) * qx * qw
    rmatrix[2, 2] = np.float32(1.0) - np.float32(2.0) * qx * qx - np.float32(2.0) * qy * qy

    return rmatrix


def rotate_point(point, rotation_matrix):
    """
    Rotate a 3D point by rotation matrix

    Args:
        point: Point3D or (x, y, z) tuple
        rotation_matrix: 3x3 rotation matrix

    Returns:
        tuple: (x, y, z) rotated coordinates
    """
    if hasattr(point, 'x'):
        x, y, z = point.x, point.y, point.z
    else:
        x, y, z = point

    out_x = x * rotation_matrix[0, 0] + y * rotation_matrix[0, 1] + z * rotation_matrix[0, 2]
    out_y = x * rotation_matrix[1, 0] + y * rotation_matrix[1, 1] + z * rotation_matrix[1, 2]
    out_z = x * rotation_matrix[2, 0] + y * rotation_matrix[2, 1] + z * rotation_matrix[2, 2]

    return (out_x, out_y, out_z)


def quaternion_normalize(q):
    """
    Normalize quaternion to unit length

    Args:
        q: Quaternion

    Returns:
        Quaternion: Normalized quaternion
    """
    mag = np.sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3)

    if mag > np.float32(1e-6):
        inv_mag = np.float32(1.0) / mag
        return Quaternion(
            q0=q.q0 * inv_mag,
            q1=q.q1 * inv_mag,
            q2=q.q2 * inv_mag,
            q3=q.q3 * inv_mag
        )
    else:
        # Return identity quaternion if magnitude too small
        return Quaternion(q0=1.0, q1=0.0, q2=0.0, q3=0.0)


def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (yaw, pitch, roll)

    Args:
        q: Quaternion

    Returns:
        tuple: (yaw, pitch, roll) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.q0 * q.q1 + q.q2 * q.q3)
    cosr_cosp = 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.q0 * q.q2 - q.q3 * q.q1)
    if abs(sinp) >= 1.0:
        pitch = np.copysign(np.pi / 2.0, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.q0 * q.q3 + q.q1 * q.q2)
    cosy_cosp = 1.0 - 2.0 * (q.q2 * q.q2 + q.q3 * q.q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (yaw, pitch, roll)
