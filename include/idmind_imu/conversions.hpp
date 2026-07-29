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
/// Pure, hardware-free conversions from IMU Brick 2.0 raw counts to SI units.
///
/// This header turns the raw integer counts delivered by ``IMU_V2_CALLBACK_ALL_DATA`` into
/// SI-unit doubles and builds ``sensor_msgs/Imu``-style covariance matrices. It depends on
/// neither ROS nor the TinkerForge bindings, so it can be unit tested without either.

#ifndef IDMIND_IMU__CONVERSIONS_HPP_
#define IDMIND_IMU__CONVERSIONS_HPP_

#include <array>
#include <cstdint>
#include <optional>

namespace idmind_imu
{
namespace conversions
{

/// Raw quaternion counts -> dimensionless, divide by this.
constexpr double kQuatScale = 16383.0;
/// Raw acceleration / linear_acceleration / gravity_vector counts (1 cm/s^2) -> m/s^2.
constexpr double kAccelScale = 100.0;
/// Raw magnetic_field counts (1/16 uT) -> tesla; combines the 1/16 and the uT->T factor.
constexpr double kMagScale = 16.0e6;
/// Raw angular_velocity counts (1/16 deg/s) -> divide by this to get degrees/s.
constexpr double kGyroScale = 16.0;
/// Raw euler_angle counts (1/16 deg) -> divide by this to get degrees.
constexpr double kEulerScale = 16.0;

/// Extra variance multiplier applied to yaw when fusion_mode is ON_WITHOUT_MAGNETOMETER,
/// since the yaw estimate is then relative and drifts over time.
constexpr double kYawDriftFactor = 100.0;

/// Four calibration levels, each 0..3, ordered (sys, gyro, acc, mag).
using Calibration = std::array<uint8_t, 4>;
/// A row-major 3x3 covariance matrix, flattened, as carried by sensor_msgs messages.
using Covariance = std::array<double, 9>;

/// Return (sys, gyro, acc, mag), each 0..3, from the raw BNO-055 calibration byte.
///
/// Bits 0-1 are magnetometer, bits 2-3 accelerometer, bits 4-5 gyroscope, bits 6-7 system.
Calibration decode_calibration(uint8_t status);

/// Convert brick (w, x, y, z) counts to ROS (x, y, z, w) doubles, scaled by kQuatScale.
std::array<double, 4> quaternion_from_brick(const std::array<int16_t, 4> & quat);

/// Convert brick (heading, roll, pitch) counts to (roll, pitch, yaw) in radians.
std::array<double, 3> euler_from_brick(const std::array<int16_t, 3> & euler);

/// Convert brick acceleration/linear_acceleration/gravity_vector counts to m/s^2.
std::array<double, 3> acceleration_from_brick(const std::array<int16_t, 3> & vec);

/// Convert brick magnetic_field counts to tesla.
std::array<double, 3> magnetic_field_from_brick(const std::array<int16_t, 3> & vec);

/// Convert brick angular_velocity counts to rad/s.
std::array<double, 3> angular_velocity_from_brick(const std::array<int16_t, 3> & vec);

/// Build a row-major 3x3 covariance matrix with exactly-zero off-diagonal terms.
Covariance diagonal_covariance(double var_x, double var_y, double var_z);

/// Build the covariance for sensor_msgs/Imu.orientation_covariance.
///
/// If \p fusion_mode is 0 (OFF) the orientation is meaningless and this returns the
/// sensor_msgs/Imu "no estimate available" sentinel: element 0 is -1.0, the rest 0.0.
///
/// Otherwise the base variance is ``stddev ** 2``, scaled up as system calibration
/// (element 0 of \p calibration) worsens. When \p fusion_mode is 2
/// (ON_WITHOUT_MAGNETOMETER) the yaw estimate is relative and drifts, so the yaw variance
/// (element 8) is additionally multiplied by kYawDriftFactor. A disengaged \p calibration
/// is treated as worst-case calibration.
Covariance orientation_covariance(
  int fusion_mode,
  const std::optional<Calibration> & calibration,
  double stddev);

}  // namespace conversions
}  // namespace idmind_imu

#endif  // IDMIND_IMU__CONVERSIONS_HPP_
