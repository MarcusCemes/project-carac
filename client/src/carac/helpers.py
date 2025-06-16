from math import acos, asin, atan2, cos, sin

from .instructions import *

Vec3 = tuple[float, float, float]
Vec4 = tuple[float, float, float, float]


def deg_to_rad(deg: float) -> float:
    return deg * (3.141592653589793 / 180.0)


def rad_to_deg(rad: float) -> float:
    return rad * (180.0 / 3.141592653589793)


def quat_to_euler(quat: Vec4) -> Vec3:
    """
    Convert quaternion to Euler angles using XYZ order (roll-pitch-yaw).
    This corresponds to rotations applied in order: X(roll), then Y(pitch), then Z(yaw).

    Args:
        quat: (qx, qy, qz, qw) quaternion

    Returns:
        (rx, ry, rz) Euler angles in radians - roll(X), pitch(Y), yaw(Z)
    """
    qx, qy, qz, qw = quat

    # Roll (X-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    # Clamp sinp to avoid numerical issues with asin
    sinp = max(-1.0, min(1.0, sinp))
    pitch = asin(sinp)

    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


def euler_xyz_intrinsic_to_quat(angles: Vec3) -> Vec4:
    """Converts 'xyz' intrinsic Euler angles (in radians) to a quaternion (qx, qy, qz, qw)."""
    rx, ry, rz = angles
    hrx, hry, hrz = rx / 2, ry / 2, rz / 2
    sx, cx = sin(hrx), cos(hrx)
    sy, cy = sin(hry), cos(hry)
    sz, cz = sin(hrz), cos(hrz)

    # Derived from standard 'xyz' intrinsic (X then new Y then newest Z)
    # q = qx i + qy j + qz k + qw
    qx = sx * cy * cz + cx * sy * sz
    qy = cx * sy * cz - sx * cy * sz
    qz = cx * cy * sz + sx * sy * cz
    qw = cx * cy * cz - sx * sy * sz
    return (qx, qy, qz, qw)


def quat_multiply(q1: Vec4, q2: Vec4) -> Vec4:
    """Multiplies two quaternions (q1 * q2). Result is (qx, qy, qz, qw)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (qx, qy, qz, qw)


def quat_conjugate(q: Vec4) -> Vec4:
    """Computes the conjugate of a quaternion."""
    qx, qy, qz, qw = q
    return (-qx, -qy, -qz, qw)


def quat_to_angle_rad(q: Vec4) -> float:
    """Extracts the total rotation angle (in radians) from a quaternion."""
    _, _, _, qw = q
    # Clamp qw to avoid domain errors with acos due to potential floating point inaccuracies
    qw_clamped = max(-1.0, min(1.0, qw))
    angle = 2 * acos(qw_clamped)
    return angle
