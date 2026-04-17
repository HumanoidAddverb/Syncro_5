import numpy as np
import math

def rot_mat(x, y, z):
    """Return 3x3 rotation matrix from roll (x), pitch (y), yaw (z) in radians."""
    rot_x = np.array([
        [1, 0, 0],
        [0, math.cos(x), -math.sin(x)],
        [0, math.sin(x),  math.cos(x)]
    ])
    
    rot_y = np.array([
        [math.cos(y), 0, math.sin(y)],
        [0, 1, 0],
        [-math.sin(y), 0, math.cos(y)]
    ])
    
    rot_z = np.array([
        [math.cos(z), -math.sin(z), 0],
        [math.sin(z),  math.cos(z), 0],
        [0, 0, 1]
    ])

    # Combine rotations: R = Rz * Ry * Rx (standard intrinsic XYZ convention)
    return rot_z @ rot_y @ rot_x


def axis_angle_vector_to_rpy(ax, ay, az):
    angle = math.sqrt(ax*ax + ay*ay + az*az)
    if angle == 0:
        return 0.0, 0.0, 0.0

    ax /= angle
    ay /= angle
    az /= angle

    c = math.cos(angle)
    s = math.sin(angle)
    C = 1 - c

    R = np.array([
        [c + ax*ax*C,     ax*ay*C - az*s, ax*az*C + ay*s],
        [ay*ax*C + az*s,  c + ay*ay*C,    ay*az*C - ax*s],
        [az*ax*C - ay*s,  az*ay*C + ax*s, c + az*az*C]
    ])

    roll  = math.atan2(R[2,1], R[2,2])
    pitch = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
    yaw   = math.atan2(R[1,0], R[0,0])

    return roll, pitch, yaw

def get_grav(force, ori):
    mag=math.sqrt(force[0]**2+force[1]**2+force[2]**2)

    grav=force/mag
    grav=rot_mat(ori[0],ori[1],ori[2]) @ grav
    
    return grav , mag
