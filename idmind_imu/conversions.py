"""
Pure, hardware-free conversions from TinkerForge IMU Brick 2.0 raw counts to SI units.

This module turns the raw integer counts delivered by ``BrickIMUV2.CALLBACK_ALL_DATA`` into
SI-unit floats and builds ``sensor_msgs/Imu``-style covariance matrices. It has no dependency
on ROS or the ``tinkerforge`` bindings so it can be unit tested without either.
"""

import math
from typing import Optional, Sequence, Tuple

#: Raw quaternion counts -> dimensionless, divide by this.
QUAT_SCALE = 16383.0
#: Raw acceleration / linear_acceleration / gravity_vector counts (1 cm/s^2) -> m/s^2.
ACCEL_SCALE = 100.0
#: Raw magnetic_field counts (1/16 uT) -> tesla, combines the 1/16 and the uT->T factor.
MAG_SCALE = 16.0e6
#: Raw angular_velocity counts (1/16 deg/s) -> divide by this to get degrees/s.
GYRO_SCALE = 16.0
#: Raw euler_angle counts (1/16 deg) -> divide by this to get degrees.
EULER_SCALE = 16.0

#: Extra variance multiplier applied to yaw when fusion_mode is ON_WITHOUT_MAGNETOMETER,
#: since the yaw estimate is then relative and drifts over time.
YAW_DRIFT_FACTOR = 100.0

#: Variance scale factor keyed by system calibration level (0..3, worse -> larger).
_CALIBRATION_VARIANCE_FACTOR = {3: 1.0, 2: 4.0, 1: 25.0, 0: 100.0}

Vector3 = Sequence[float]
Vector4 = Sequence[float]


def decode_calibration(status: int) -> Tuple[int, int, int, int]:
    """
    Return (sys, gyro, acc, mag), each 0..3, from the raw calibration byte.

    Bits 0-1 are magnetometer, bits 2-3 are accelerometer, bits 4-5 are gyroscope, and
    bits 6-7 are system, as reported by the BNO-055.
    """
    mag = status & 0b11
    acc = (status >> 2) & 0b11
    gyro = (status >> 4) & 0b11
    sys = (status >> 6) & 0b11
    return (sys, gyro, acc, mag)


def quaternion_from_brick(quat: Vector4) -> Tuple[float, float, float, float]:
    """Convert brick (w, x, y, z) counts to ROS (x, y, z, w) floats, scaled by QUAT_SCALE."""
    w, x, y, z = quat
    return (x / QUAT_SCALE, y / QUAT_SCALE, z / QUAT_SCALE, w / QUAT_SCALE)


def euler_from_brick(euler: Vector3) -> Tuple[float, float, float]:
    """Convert brick (heading, roll, pitch) counts to (roll, pitch, yaw) in radians."""
    heading, roll, pitch = euler
    roll_deg = roll / EULER_SCALE
    pitch_deg = pitch / EULER_SCALE
    yaw_deg = heading / EULER_SCALE
    return (math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))


def acceleration_from_brick(vec: Vector3) -> Tuple[float, float, float]:
    """Convert brick acceleration/linear_acceleration/gravity_vector counts to m/s^2."""
    x, y, z = vec
    return (x / ACCEL_SCALE, y / ACCEL_SCALE, z / ACCEL_SCALE)


def magnetic_field_from_brick(vec: Vector3) -> Tuple[float, float, float]:
    """Convert brick magnetic_field counts to tesla."""
    x, y, z = vec
    return (x / MAG_SCALE, y / MAG_SCALE, z / MAG_SCALE)


def angular_velocity_from_brick(vec: Vector3) -> Tuple[float, float, float]:
    """Convert brick angular_velocity counts to rad/s."""
    x, y, z = vec
    return (
        math.radians(x / GYRO_SCALE),
        math.radians(y / GYRO_SCALE),
        math.radians(z / GYRO_SCALE),
    )


def diagonal_covariance(var_x: float, var_y: float, var_z: float) -> list:
    """Build a row-major 3x3 covariance matrix, as a flat list of 9, with zero off-diagonals."""
    return [
        float(var_x), 0.0, 0.0,
        0.0, float(var_y), 0.0,
        0.0, 0.0, float(var_z),
    ]


def orientation_covariance(
    fusion_mode: int,
    calibration: Optional[Tuple[int, int, int, int]],
    stddev: float,
) -> list:
    """
    Build the covariance for sensor_msgs/Imu.orientation_covariance.

    If fusion_mode is 0 (OFF), the orientation is meaningless and this returns the
    sensor_msgs/Imu "no estimate available" sentinel: element 0 is -1.0, the rest 0.0.

    Otherwise the base variance is stddev ** 2, scaled up as system calibration (element 0
    of calibration) worsens. When fusion_mode is 2 (ON_WITHOUT_MAGNETOMETER) the yaw
    estimate is relative and drifts, so the yaw variance (element 8) is additionally
    multiplied by YAW_DRIFT_FACTOR. calibration=None is treated as worst-case calibration.
    """
    if fusion_mode == 0:
        return [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    sys_calibration = calibration[0] if calibration is not None else 0
    factor = _CALIBRATION_VARIANCE_FACTOR.get(sys_calibration, 100.0)
    base_variance = (stddev ** 2) * factor

    yaw_variance = base_variance
    if fusion_mode == 2:
        yaw_variance *= YAW_DRIFT_FACTOR

    return diagonal_covariance(base_variance, base_variance, yaw_variance)
