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
/// Driver contract shared by all IMU hardware backends.
///
/// This header defines the data the node receives (``ImuSample``), the hardware-relevant
/// configuration a driver accepts (``DriverConfig``), the status a driver reports about
/// itself (``DriverState``), and the abstract interface (``ImuDriver``) every concrete driver
/// must implement. It depends on ``rclcpp`` only for ``rclcpp::Logger``, which is a standalone
/// value type obtainable via ``rclcpp::get_logger`` without any node, so this layer can be
/// unit tested without constructing a node.

#ifndef IDMIND_IMU__DRIVERS__DRIVER_HPP_
#define IDMIND_IMU__DRIVERS__DRIVER_HPP_

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "idmind_imu/conversions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace idmind_imu
{

/// One reading from an IMU, already converted to SI units in ROS conventions.
///
/// Every field is optional so a driver that cannot supply a given quantity simply leaves it
/// disengaged rather than fabricating a value.
struct ImuSample
{
  /// Orientation as ROS (x, y, z, w).
  std::optional<std::array<double, 4>> orientation;
  /// Angular velocity (x, y, z) in rad/s.
  std::optional<std::array<double, 3>> angular_velocity;
  /// Linear acceleration (x, y, z) in m/s^2.
  std::optional<std::array<double, 3>> linear_acceleration;
  /// Magnetic field (x, y, z) in tesla.
  std::optional<std::array<double, 3>> magnetic_field;
  /// Gravity vector (x, y, z) in m/s^2.
  std::optional<std::array<double, 3>> gravity;
  /// Euler angles as (roll, pitch, yaw) in radians.
  std::optional<std::array<double, 3>> euler;
  /// Sensor temperature in degrees Celsius.
  std::optional<double> temperature;
  /// Calibration levels as (sys, gyro, acc, mag), each 0..3.
  std::optional<conversions::Calibration> calibration;
  /// The fusion mode the sample was produced under.
  std::optional<int> fusion_mode;
};

/// The hardware-relevant subset of the node's parameters, handed to a driver whole.
///
/// The node always knows every field, so a driver receives a complete desired configuration
/// rather than a partial patch; a driver decides for itself which fields actually changed.
struct DriverConfig
{
  /// Hostname/IP of the IMU transport (e.g. BrickDaemon).
  std::string host{"localhost"};
  /// TCP port of the IMU transport.
  int port{4223};
  /// Desired sample rate of the sensor stream, in Hz.
  double imu_freq{20.0};
  /// Whether the device's LEDs should be lit.
  bool imu_leds{false};
  /// Sensor fusion mode; 0 means off, in which case orientation is not meaningful.
  int imu_fusion_mode{2};
  /// Whether the transport should reconnect by itself after a drop.
  bool auto_reconnect{true};
  /// Which device field to report as linear_acceleration: "linear" or "raw".
  std::string acceleration_source{"linear"};
};

/// A snapshot of a driver's connection and device status, for diagnostics.
struct DriverState
{
  /// Whether the transport is connected.
  bool connected{false};
  /// Whether an IMU device has been found on that transport.
  bool device_present{false};
  /// Human-readable hardware identifier, for diagnostic_updater.
  std::string hardware_id{"unknown"};
  /// Human-readable detail about the current state.
  std::string detail;
};

/// Called with each new sample, from whatever thread the hardware library delivers it on.
using SampleCallback = std::function<void (const ImuSample &)>;

/// Abstract base for a hardware-specific IMU driver.
///
/// A driver owns the transport connection to one IMU and reports readings by invoking its
/// sample callback with an ``ImuSample``, from whatever thread the underlying hardware
/// library delivers them on. The node is responsible for thread-safety on its own side; the
/// driver is responsible for never blocking the caller of ``start()``.
class ImuDriver
{
public:
  /// Store the initial config, sample callback, and logger.
  ImuDriver(const DriverConfig & config, SampleCallback on_sample, rclcpp::Logger logger)
  : config_(config), on_sample_(std::move(on_sample)), logger_(std::move(logger)) {}

  virtual ~ImuDriver() = default;

  ImuDriver(const ImuDriver &) = delete;
  ImuDriver & operator=(const ImuDriver &) = delete;

  /// Begin connecting and streaming. Must not block; spawn threads if needed.
  virtual void start() = 0;

  /// Tear down the driver. Idempotent: safe to call twice or before ``start()``.
  virtual void stop() = 0;

  /// Apply a changed configuration, e.g. in response to updated ROS parameters.
  virtual void apply_config(const DriverConfig & config) = 0;

  /// Return a snapshot of the driver's current connection and device status.
  virtual DriverState state() const = 0;

protected:
  /// The most recently requested configuration.
  DriverConfig config_;
  /// The node's sample sink.
  SampleCallback on_sample_;
  /// Where the driver reports its own problems.
  rclcpp::Logger logger_;
};

}  // namespace idmind_imu

#endif  // IDMIND_IMU__DRIVERS__DRIVER_HPP_
