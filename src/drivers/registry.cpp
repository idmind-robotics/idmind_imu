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

#include "idmind_imu/drivers/registry.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "idmind_imu/drivers/brick_v2.hpp"

namespace idmind_imu
{

namespace
{

/// Builds one driver instance; the value half of the registry table.
using DriverFactory = std::function<std::unique_ptr<ImuDriver>(
      const DriverConfig &, SampleCallback, rclcpp::Logger)>;

/// Wrap a concrete driver type into a factory, so the table below stays one line per driver.
template<typename DriverT>
DriverFactory factory_for()
{
  return [](const DriverConfig & config, SampleCallback on_sample, rclcpp::Logger logger) {
           return std::unique_ptr<ImuDriver>(
             new DriverT(config, std::move(on_sample), std::move(logger)));
         };
}

/// Maps a driver name to its factory. A vector, not a map, so ordering is stable in messages.
const std::vector<std::pair<std::string, DriverFactory>> & registry()
{
  static const std::vector<std::pair<std::string, DriverFactory>> table{
    {"brick_v2", factory_for<ImuBrickV2Driver>()},
  };
  return table;
}

}  // namespace

std::vector<std::string> available_drivers()
{
  std::vector<std::string> names;
  names.reserve(registry().size());
  for (const auto & entry : registry()) {
    names.push_back(entry.first);
  }
  return names;
}

std::unique_ptr<ImuDriver> make_driver(
  const std::string & name,
  const DriverConfig & config,
  SampleCallback on_sample,
  rclcpp::Logger logger)
{
  for (const auto & entry : registry()) {
    if (entry.first == name) {
      return entry.second(config, std::move(on_sample), std::move(logger));
    }
  }

  std::string known;
  for (const auto & entry : registry()) {
    if (!known.empty()) {
      known += ", ";
    }
    known += entry.first;
  }
  throw UnknownDriverError(
          "Unknown IMU driver '" + name + "'; available drivers: " + known);
}

}  // namespace idmind_imu
