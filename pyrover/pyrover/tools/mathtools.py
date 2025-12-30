# #!/usr/bin/env python3

# import math
# from typing import Tuple


# def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
#     """
#     Convert Euler angles to quaternion (ZYX convention).
    
#     Parameters
#     ----------
#     roll : float
#         Roll angle in radians
#     pitch : float
#         Pitch angle in radians
#     yaw : float
#         Yaw angle in radians
        
#     Returns
#     -------
#     tuple
#         Quaternion as (x, y, z, w)
#     """
#     cr = math.cos(roll / 2)
#     sr = math.sin(roll / 2)
#     cp = math.cos(pitch / 2)
#     sp = math.sin(pitch / 2)
#     cy = math.cos(yaw / 2)
#     sy = math.sin(yaw / 2)
    
#     return (
#         sr * cp * cy - cr * sp * sy,  # x
#         cr * sp * cy + sr * cp * sy,  # y
#         cr * cp * sy - sr * sp * cy,  # z
#         cr * cp * cy + sr * sp * sy   # w
#     )


# def wrap_angle(x: float) -> float:
#     """ Wrap angle (radians) to [-pi, pi]

#     Args:
#         x (T.Union[float, np.ndarray]): angle to be wrapped

#     Returns:
#         T.Union[float, np.ndarray]: equivalent angle in [-pi, pi]
#     """
#     return (x + math.pi) % (2 * math.pi) - math.pi