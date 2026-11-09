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
/// Name-based lookup for concrete IMU driver implementations.
///
/// Adding an IMU model means adding one driver and one line in ``registry.cpp``; no node
/// code changes. Unlike the Python original there is no lazy import to arrange: every
/// registered driver is linked into the same library, so a missing hardware dependency is a
/// link-time rather than a run-time concern.

#ifndef IDMIND_IMU__DRIVERS__REGISTRY_HPP_
#define IDMIND_IMU__DRIVERS__REGISTRY_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "idmind_imu/drivers/driver.hpp"

namespace idmind_imu
{

/// Thrown by ``make_driver`` when the requested driver name is not registered.
class UnknownDriverError : public std::runtime_error
{
public:
  /// Build the error with a message naming the failed lookup and the available drivers.
  explicit UnknownDriverError(const std::string & message)
  : std::runtime_error(message) {}
};

/// Return the registered driver names, in registration order.
std::vector<std::string> available_drivers();

/// Construct the driver registered under \p name.
///
/// \throws UnknownDriverError if \p name is not registered; the message lists what is.
std::unique_ptr<ImuDriver> make_driver(
  const std::string & name,
  const DriverConfig & config,
  SampleCallback on_sample,
  rclcpp::Logger logger);

}  // namespace idmind_imu

#endif  // IDMIND_IMU__DRIVERS__REGISTRY_HPP_
