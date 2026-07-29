// Copyright 2024 IDMind
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// Unit tests for idmind_imu/conversions.hpp: pure math, no ROS, no hardware.

#include <gtest/gtest.h>

#include <cmath>

#include "idmind_imu/conversions.hpp"

using idmind_imu::conversions::Calibration;
using idmind_imu::conversions::acceleration_from_brick;
using idmind_imu::conversions::angular_velocity_from_brick;
using idmind_imu::conversions::decode_calibration;
using idmind_imu::conversions::diagonal_covariance;
using idmind_imu::conversions::euler_from_brick;
using idmind_imu::conversions::kYawDriftFactor;
using idmind_imu::conversions::magnetic_field_from_brick;
using idmind_imu::conversions::orientation_covariance;
using idmind_imu::conversions::quaternion_from_brick;

namespace
{
/// Degrees to radians, so expectations read in the units the datasheet uses.
double radians(double degrees) {return degrees * M_PI / 180.0;}
}  // namespace

TEST(Conversions, AccelerationFromBrickBasic)
{
  // 100 counts equals 1.0 m/s^2 on each axis; negatives and zero pass through linearly.
  const auto result = acceleration_from_brick({100, -100, 0});
  EXPECT_DOUBLE_EQ(result[0], 1.0);
  EXPECT_DOUBLE_EQ(result[1], -1.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

TEST(Conversions, AccelerationFromBrickArbitrary)
{
  const auto result = acceleration_from_brick({250, -50, 10});
  EXPECT_NEAR(result[0], 2.5, 1e-12);
  EXPECT_NEAR(result[1], -0.5, 1e-12);
  EXPECT_NEAR(result[2], 0.1, 1e-12);
}

TEST(Conversions, MagneticFieldFromBrick)
{
  // 16 counts equals 1 uT = 1e-6 T.
  const auto result = magnetic_field_from_brick({16, -16, 0});
  EXPECT_NEAR(result[0], 1e-6, 1e-18);
  EXPECT_NEAR(result[1], -1e-6, 1e-18);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

TEST(Conversions, MagneticFieldFromBrickArbitrary)
{
  const auto result = magnetic_field_from_brick({32, 8, -8});
  EXPECT_NEAR(result[0], 2e-6, 1e-18);
  EXPECT_NEAR(result[1], 0.5e-6, 1e-18);
  EXPECT_NEAR(result[2], -0.5e-6, 1e-18);
}

TEST(Conversions, AngularVelocityFromBrick)
{
  // 16 counts equals 1.0 deg/s = 0.0174533 rad/s.
  const auto result = angular_velocity_from_brick({16, -16, 0});
  EXPECT_NEAR(result[0], 0.0174533, 1e-6);
  EXPECT_NEAR(result[1], -0.0174533, 1e-6);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
}

TEST(Conversions, AngularVelocityFromBrickMatchesRadians)
{
  const auto result = angular_velocity_from_brick({160, 0, -32});
  EXPECT_NEAR(result[0], radians(10.0), 1e-12);
  EXPECT_NEAR(result[1], 0.0, 1e-12);
  EXPECT_NEAR(result[2], radians(-2.0), 1e-12);
}

TEST(Conversions, QuaternionFromBrickIdentity)
{
  // 16383 counts of w-only input maps to the ROS identity quaternion (0, 0, 0, 1).
  const auto result = quaternion_from_brick({16383, 0, 0, 0});
  EXPECT_DOUBLE_EQ(result[0], 0.0);
  EXPECT_DOUBLE_EQ(result[1], 0.0);
  EXPECT_DOUBLE_EQ(result[2], 0.0);
  EXPECT_DOUBLE_EQ(result[3], 1.0);
}

TEST(Conversions, QuaternionFromBrickReordersComponents)
{
  // Four distinct values verify the (w,x,y,z)->(x,y,z,w) permutation component by component.
  const auto result = quaternion_from_brick({1000, 2000, 3000, 4000});
  EXPECT_NEAR(result[0], 2000 / 16383.0, 1e-12);
  EXPECT_NEAR(result[1], 3000 / 16383.0, 1e-12);
  EXPECT_NEAR(result[2], 4000 / 16383.0, 1e-12);
  EXPECT_NEAR(result[3], 1000 / 16383.0, 1e-12);
}

TEST(Conversions, EulerFromBrickPutsHeadingInYawSlot)
{
  const auto result = euler_from_brick({16 * 90, 0, 0});
  EXPECT_NEAR(result[0], 0.0, 1e-12);
  EXPECT_NEAR(result[1], 0.0, 1e-12);
  EXPECT_NEAR(result[2], radians(90.0), 1e-12);
}

TEST(Conversions, EulerFromBrickAllDistinct)
{
  // Distinct roll/pitch/heading counts each land in their correct output slot.
  const auto result = euler_from_brick({16 * 30, 16 * 10, 16 * 20});
  EXPECT_NEAR(result[0], radians(10.0), 1e-12);
  EXPECT_NEAR(result[1], radians(20.0), 1e-12);
  EXPECT_NEAR(result[2], radians(30.0), 1e-12);
}

TEST(Conversions, DecodeCalibrationZero)
{
  EXPECT_EQ(decode_calibration(0x00), (Calibration{0, 0, 0, 0}));
}

TEST(Conversions, DecodeCalibrationFull)
{
  EXPECT_EQ(decode_calibration(0xFF), (Calibration{3, 3, 3, 3}));
}

TEST(Conversions, DecodeCalibrationDistinctLevels)
{
  // 0b11'10'01'00 verifies field ORDER: sys=3, gyro=2, acc=1, mag=0.
  EXPECT_EQ(decode_calibration(0b11100100), (Calibration{3, 2, 1, 0}));
}

TEST(Conversions, DiagonalCovarianceShapeAndOffDiagonals)
{
  const auto cov = diagonal_covariance(1.0, 2.0, 3.0);
  for (int i : {1, 2, 3, 5, 6, 7}) {
    EXPECT_DOUBLE_EQ(cov[i], 0.0) << "off-diagonal element " << i << " must be exactly zero";
  }
  EXPECT_DOUBLE_EQ(cov[0], 1.0);
  EXPECT_DOUBLE_EQ(cov[4], 2.0);
  EXPECT_DOUBLE_EQ(cov[8], 3.0);
}

TEST(Conversions, OrientationCovarianceFusionOffIsInvalidMarker)
{
  // fusion_mode 0 must return the -1.0 sentinel meaning "no orientation estimate".
  const auto cov = orientation_covariance(0, Calibration{3, 3, 3, 3}, 0.01);
  EXPECT_DOUBLE_EQ(cov[0], -1.0);
  for (size_t i = 1; i < cov.size(); ++i) {
    EXPECT_DOUBLE_EQ(cov[i], 0.0);
  }
}

TEST(Conversions, OrientationCovarianceOffDiagonalsAreZero)
{
  const auto cov = orientation_covariance(1, Calibration{3, 3, 3, 3}, 0.01);
  for (int i : {1, 2, 3, 5, 6, 7}) {
    EXPECT_DOUBLE_EQ(cov[i], 0.0);
  }
}

TEST(Conversions, OrientationCovarianceBetterCalibrationIsSmallerVariance)
{
  const auto best = orientation_covariance(1, Calibration{3, 0, 0, 0}, 0.01);
  const auto worst = orientation_covariance(1, Calibration{0, 0, 0, 0}, 0.01);
  EXPECT_LT(best[0], worst[0]);
  EXPECT_LT(best[8], worst[8]);
}

TEST(Conversions, OrientationCovarianceFusionMode2InflatesYawOnly)
{
  const Calibration calibration{3, 3, 3, 3};
  const auto mode1 = orientation_covariance(1, calibration, 0.01);
  const auto mode2 = orientation_covariance(2, calibration, 0.01);
  EXPECT_DOUBLE_EQ(mode1[0], mode2[0]);
  EXPECT_DOUBLE_EQ(mode1[4], mode2[4]);
  EXPECT_NEAR(mode2[8], mode1[8] * kYawDriftFactor, 1e-18);
  EXPECT_GT(mode2[8], mode1[8]);
}

TEST(Conversions, OrientationCovarianceNoCalibrationIsWorstCase)
{
  const auto none = orientation_covariance(1, std::nullopt, 0.01);
  const auto worst = orientation_covariance(1, Calibration{0, 0, 0, 0}, 0.01);
  EXPECT_DOUBLE_EQ(none[0], worst[0]);
  EXPECT_DOUBLE_EQ(none[8], worst[8]);
}
