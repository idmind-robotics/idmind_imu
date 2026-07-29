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

#include "idmind_imu/conversions.hpp"

#include <cmath>
#include <map>

namespace idmind_imu
{
namespace conversions
{

namespace
{

/// Degrees -> radians, so this file never depends on a particular M_PI spelling.
double radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

/// Variance scale factor keyed by system calibration level (0..3, worse -> larger).
double calibration_variance_factor(uint8_t sys_calibration)
{
  switch (sys_calibration) {
    case 3:
      return 1.0;
    case 2:
      return 4.0;
    case 1:
      return 25.0;
    default:
      return 100.0;
  }
}

}  // namespace

Calibration decode_calibration(uint8_t status)
{
  const uint8_t mag = status & 0b11;
  const uint8_t acc = (status >> 2) & 0b11;
  const uint8_t gyro = (status >> 4) & 0b11;
  const uint8_t sys = (status >> 6) & 0b11;
  return Calibration{sys, gyro, acc, mag};
}

std::array<double, 4> quaternion_from_brick(const std::array<int16_t, 4> & quat)
{
  // The brick reports (w, x, y, z); ROS wants (x, y, z, w).
  return std::array<double, 4>{
    quat[1] / kQuatScale,
    quat[2] / kQuatScale,
    quat[3] / kQuatScale,
    quat[0] / kQuatScale,
  };
}

std::array<double, 3> euler_from_brick(const std::array<int16_t, 3> & euler)
{
  // The brick reports (heading, roll, pitch); this returns (roll, pitch, yaw).
  const double heading_deg = euler[0] / kEulerScale;
  const double roll_deg = euler[1] / kEulerScale;
  const double pitch_deg = euler[2] / kEulerScale;
  return std::array<double, 3>{radians(roll_deg), radians(pitch_deg), radians(heading_deg)};
}

std::array<double, 3> acceleration_from_brick(const std::array<int16_t, 3> & vec)
{
  return std::array<double, 3>{
    vec[0] / kAccelScale, vec[1] / kAccelScale, vec[2] / kAccelScale};
}

std::array<double, 3> magnetic_field_from_brick(const std::array<int16_t, 3> & vec)
{
  return std::array<double, 3>{vec[0] / kMagScale, vec[1] / kMagScale, vec[2] / kMagScale};
}

std::array<double, 3> angular_velocity_from_brick(const std::array<int16_t, 3> & vec)
{
  return std::array<double, 3>{
    radians(vec[0] / kGyroScale),
    radians(vec[1] / kGyroScale),
    radians(vec[2] / kGyroScale),
  };
}

Covariance diagonal_covariance(double var_x, double var_y, double var_z)
{
  return Covariance{
    var_x, 0.0, 0.0,
    0.0, var_y, 0.0,
    0.0, 0.0, var_z,
  };
}

Covariance orientation_covariance(
  int fusion_mode,
  const std::optional<Calibration> & calibration,
  double stddev)
{
  if (fusion_mode == 0) {
    return Covariance{-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  const uint8_t sys_calibration = calibration.has_value() ? (*calibration)[0] : 0;
  const double base_variance = stddev * stddev * calibration_variance_factor(sys_calibration);

  double yaw_variance = base_variance;
  if (fusion_mode == 2) {
    yaw_variance *= kYawDriftFactor;
  }

  return diagonal_covariance(base_variance, base_variance, yaw_variance);
}

}  // namespace conversions
}  // namespace idmind_imu
