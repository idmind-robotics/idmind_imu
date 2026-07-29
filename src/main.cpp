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
/// Entry point: spins an ``ImuNode`` under a multi-threaded executor until interrupted.

#include <memory>

#include "idmind_imu/imu_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  {
    std::shared_ptr<idmind_imu::ImuNode> node;
    try {
      node = std::make_shared<idmind_imu::ImuNode>();
    } catch (const std::exception & exc) {
      RCLCPP_FATAL(
        rclcpp::get_logger("idmind_imu"), "Failed to start the IMU node: %s", exc.what());
      rclcpp::shutdown();
      return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    try {
      executor.spin();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(node->get_logger(), "Executor stopped: %s", exc.what());
      exit_code = 1;
    }

    // Stop the driver before the node is destroyed, so no driver thread can publish into a
    // half-destroyed node.
    executor.remove_node(node);
    node->stop_driver();
  }

  rclcpp::shutdown();
  return exit_code;
}
