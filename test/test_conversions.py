"""Unit tests for idmind_imu.conversions: pure math, no ROS, no hardware."""

import math

import pytest

from idmind_imu.conversions import (
    YAW_DRIFT_FACTOR,
    acceleration_from_brick,
    angular_velocity_from_brick,
    decode_calibration,
    diagonal_covariance,
    euler_from_brick,
    magnetic_field_from_brick,
    orientation_covariance,
    quaternion_from_brick,
)


def test_acceleration_from_brick_basic():
    """100 counts equals 1.0 m/s^2 on each axis; negatives and zero pass through linearly."""
    assert acceleration_from_brick((100, -100, 0)) == pytest.approx((1.0, -1.0, 0.0))


def test_acceleration_from_brick_arbitrary():
    """Arbitrary counts scale linearly by ACCEL_SCALE."""
    assert acceleration_from_brick((250, -50, 10)) == pytest.approx((2.5, -0.5, 0.1))


def test_magnetic_field_from_brick():
    """16 counts equals 1 uT = 1e-6 T; negatives and zero behave linearly."""
    assert magnetic_field_from_brick((16, -16, 0)) == pytest.approx((1e-6, -1e-6, 0.0))


def test_magnetic_field_from_brick_arbitrary():
    """32 counts equals 2 uT = 2e-6 T."""
    assert magnetic_field_from_brick((32, 8, -8)) == pytest.approx((2e-6, 0.5e-6, -0.5e-6))


def test_angular_velocity_from_brick():
    """16 counts equals 1.0 deg/s = 0.0174533 rad/s."""
    result = angular_velocity_from_brick((16, -16, 0))
    assert result == pytest.approx((0.0174533, -0.0174533, 0.0), abs=1e-6)


def test_angular_velocity_from_brick_matches_math_radians():
    """Cross-check against math.radians directly for an arbitrary count."""
    x, y, z = angular_velocity_from_brick((160, 0, -32))
    assert x == pytest.approx(math.radians(10.0))
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(math.radians(-2.0))


def test_quaternion_from_brick_identity():
    """16383 counts of w-only input maps to the ROS identity quaternion (0,0,0,1)."""
    assert quaternion_from_brick((16383, 0, 0, 0)) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_from_brick_reorders_components():
    """Four distinct values verify the (w,x,y,z)->(x,y,z,w) permutation, component by component."""
    x, y, z, w = quaternion_from_brick((1000, 2000, 3000, 4000))
    assert x == pytest.approx(2000 / 16383.0)
    assert y == pytest.approx(3000 / 16383.0)
    assert z == pytest.approx(4000 / 16383.0)
    assert w == pytest.approx(1000 / 16383.0)


def test_euler_from_brick_puts_heading_in_yaw_slot():
    """Heading (first brick field) must land in the yaw slot, not the roll slot."""
    heading_counts = 16 * 90  # 90 degrees of heading
    roll, pitch, yaw = euler_from_brick((heading_counts, 0, 0))
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(math.radians(90.0))


def test_euler_from_brick_all_distinct():
    """Distinct roll/pitch/heading counts each land in their correct output slot."""
    heading_counts = 16 * 30
    roll_counts = 16 * 10
    pitch_counts = 16 * 20
    roll, pitch, yaw = euler_from_brick((heading_counts, roll_counts, pitch_counts))
    assert roll == pytest.approx(math.radians(10.0))
    assert pitch == pytest.approx(math.radians(20.0))
    assert yaw == pytest.approx(math.radians(30.0))


def test_decode_calibration_zero():
    """0x00 decodes to all-uncalibrated."""
    assert decode_calibration(0x00) == (0, 0, 0, 0)


def test_decode_calibration_full():
    """0xFF decodes to all-fully-calibrated."""
    assert decode_calibration(0xFF) == (3, 3, 3, 3)


def test_decode_calibration_distinct_levels():
    """0b11_10_01_00 verifies field ORDER: sys=3, gyro=2, acc=1, mag=0."""
    assert decode_calibration(0b11_10_01_00) == (3, 2, 1, 0)


def test_diagonal_covariance_shape_and_offdiagonals():
    """Off-diagonal elements are exactly 0.0 and the diagonal sits at indices 0, 4, 8."""
    cov = diagonal_covariance(1.0, 2.0, 3.0)
    assert len(cov) == 9
    for i in (1, 2, 3, 5, 6, 7):
        assert cov[i] == 0.0
    assert cov[0] == pytest.approx(1.0)
    assert cov[4] == pytest.approx(2.0)
    assert cov[8] == pytest.approx(3.0)


def test_orientation_covariance_fusion_off_is_invalid_marker():
    """fusion_mode 0 must return the -1.0 sentinel meaning no orientation estimate."""
    cov = orientation_covariance(0, (3, 3, 3, 3), 0.01)
    assert cov[0] == -1.0
    assert cov[1:] == [0.0] * 8


def test_orientation_covariance_offdiagonals_are_zero():
    """Off-diagonal elements are exactly 0.0 in the normal (non-sentinel) case too."""
    cov = orientation_covariance(1, (3, 3, 3, 3), 0.01)
    for i in (1, 2, 3, 5, 6, 7):
        assert cov[i] == 0.0


def test_orientation_covariance_better_calibration_is_smaller_variance():
    """Higher system calibration must give strictly smaller variance than lower."""
    cov_best = orientation_covariance(1, (3, 0, 0, 0), 0.01)
    cov_worst = orientation_covariance(1, (0, 0, 0, 0), 0.01)
    assert cov_best[0] < cov_worst[0]
    assert cov_best[8] < cov_worst[8]


def test_orientation_covariance_fusion_mode_2_inflates_yaw_only():
    """fusion_mode 2 must inflate only element 8 (yaw) relative to fusion_mode 1."""
    calibration = (3, 3, 3, 3)
    cov_mode1 = orientation_covariance(1, calibration, 0.01)
    cov_mode2 = orientation_covariance(2, calibration, 0.01)
    assert cov_mode1[0] == pytest.approx(cov_mode2[0])
    assert cov_mode1[4] == pytest.approx(cov_mode2[4])
    assert cov_mode2[8] == pytest.approx(cov_mode1[8] * YAW_DRIFT_FACTOR)
    assert cov_mode2[8] > cov_mode1[8]


def test_orientation_covariance_none_calibration_is_worst_case():
    """calibration=None must behave the same as an explicit sys=0 (worst-case) calibration."""
    cov_none = orientation_covariance(1, None, 0.01)
    cov_worst = orientation_covariance(1, (0, 0, 0, 0), 0.01)
    assert cov_none[0] == pytest.approx(cov_worst[0])
    assert cov_none[8] == pytest.approx(cov_worst[8])
