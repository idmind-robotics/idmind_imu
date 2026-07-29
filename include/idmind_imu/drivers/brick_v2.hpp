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
/// Driver for the TinkerForge IMU Brick 2.0 (BNO-055), reached over BrickDaemon TCP.
///
/// All unit conversion is delegated to ``idmind_imu/conversions.hpp``; this driver only
/// speaks raw TinkerForge counts to the hardware and hands SI-unit ``ImuSample`` instances
/// upward.
///
/// Threading model: the TinkerForge bindings dispatch every registered callback serially on
/// one internal callback thread, so the enumerate and all-data handlers here do the minimum
/// possible work and never perform a blocking hardware round-trip. All configuration I/O
/// happens on a dedicated worker thread fed by a job queue, so a slow config round-trip can
/// never stall delivery of the sensor stream. A separate connect-retry thread handles the
/// initial (blocking) ``ipcon_connect``, since the library's auto-reconnect only covers
/// reconnection after a first successful connect.

#ifndef IDMIND_IMU__DRIVERS__BRICK_V2_HPP_
#define IDMIND_IMU__DRIVERS__BRICK_V2_HPP_

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "idmind_imu/drivers/driver.hpp"

extern "C" {
#include "tinkerforge/brick_imu_v2.h"
#include "tinkerforge/ip_connection.h"
}

namespace idmind_imu
{

/// Driver for a single TinkerForge IMU Brick 2.0, connected through BrickDaemon.
class ImuBrickV2Driver : public ImuDriver
{
public:
  /// Build the driver; does not touch the network until ``start()`` is called.
  ImuBrickV2Driver(
    const DriverConfig & config, SampleCallback on_sample, rclcpp::Logger logger);

  /// Stop the driver if still running, then release the IP connection.
  ~ImuBrickV2Driver() override;

  /// Register callbacks, start the config worker, and start connect-retrying.
  void start() override;

  /// Tear the driver down. Idempotent: safe before ``start()`` or called twice.
  void stop() override;

  /// Store the desired configuration and enqueue a convergence job on the worker thread.
  void apply_config(const DriverConfig & config) override;

  /// Return an immutable snapshot of the current connection and device status.
  DriverState state() const override;

private:
  /// Owns an ``IMUV2`` and calls ``imu_v2_destroy`` exactly once when the last user drops it.
  ///
  /// Held by ``shared_ptr`` so the worker thread can copy the handle out under the lock and
  /// then do blocking hardware I/O with the lock released, without the enumerate callback
  /// being able to destroy the device from under it.
  class Device
  {
public:
    /// Create and register the device object against \p ipcon.
    Device(const std::string & uid, IPConnection * ipcon);
    /// Destroy the device object, deregistering it from its IP connection.
    ~Device();

    Device(const Device &) = delete;
    Device & operator=(const Device &) = delete;

    /// The raw handle, for the ``imu_v2_*`` calls.
    IMUV2 * handle() {return &imu_;}
    /// The device's base58 UID.
    const std::string & uid() const {return uid_;}

private:
    IMUV2 imu_{};
    std::string uid_;
  };

  /// A unit of blocking hardware work, run on the worker thread.
  using Job = std::function<void ()>;

  // -- C callback trampolines; ``user_data`` is always the driver instance -----------------

  static void connected_trampoline(uint8_t connect_reason, void * user_data);
  static void disconnected_trampoline(uint8_t disconnect_reason, void * user_data);
  static void enumerate_trampoline(
    const char * uid, const char * connected_uid, char position,
    uint8_t hardware_version[3], uint8_t firmware_version[3],
    uint16_t device_identifier, uint8_t enumeration_type, void * user_data);
  static void all_data_trampoline(
    int16_t acceleration[3], int16_t magnetic_field[3], int16_t angular_velocity[3],
    int16_t euler_angle[3], int16_t quaternion[4], int16_t linear_acceleration[3],
    int16_t gravity_vector[3], int8_t temperature, uint8_t calibration_status,
    void * user_data);

  // -- Connection lifecycle -----------------------------------------------------------------

  /// Retry ``ipcon_connect`` roughly once a second until it succeeds or stop is requested.
  void connect_loop();
  /// Handle a transport connect: mark connected, then trigger enumeration.
  void on_connected(uint8_t connect_reason);
  /// Handle a transport disconnect: clear device state.
  void on_disconnected(uint8_t disconnect_reason);
  /// Handle an enumeration event: identify and wire up the brick, or drop it.
  void on_enumerate(const char * uid, uint16_t device_identifier, uint8_t enumeration_type);
  /// Handle one all-data frame: convert counts to SI units and hand the sample upward.
  void on_all_data(
    const int16_t acceleration[3], const int16_t magnetic_field[3],
    const int16_t angular_velocity[3], const int16_t euler_angle[3],
    const int16_t quaternion[4], const int16_t linear_acceleration[3],
    const int16_t gravity_vector[3], int8_t temperature, uint8_t calibration_status);

  // -- Configuration (worker thread only) ---------------------------------------------------

  /// Consume config jobs from the queue until stop is requested.
  void worker_loop();
  /// Push one job onto the worker queue, unless the driver is stopping.
  void enqueue(Job job);
  /// Read the device's current config and issue only the setters actually needed.
  void do_apply_config();
  /// Converge the two independent LED booleans onto the single ``imu_leds`` setting.
  void apply_leds(IMUV2 * imu, const DriverConfig & config);
  /// Set the sensor fusion mode if it differs from the desired one.
  void apply_fusion_mode(IMUV2 * imu, const DriverConfig & config);
  /// Set the all-data callback period from ``imu_freq`` if it differs from the current.
  void apply_data_period(IMUV2 * imu, const DriverConfig & config);

  IPConnection ipcon_{};
  /// Whether ``ipcon_create`` has run, so the destructor knows to call ``ipcon_destroy``.
  bool ipcon_created_{false};

  /// Guards config_, state_, device_, and started_.
  mutable std::mutex mutex_;
  /// The currently wired-up device, or null when none is present.
  std::shared_ptr<Device> device_;
  /// Connection/device status reported through ``state()``.
  DriverState state_;
  /// Whether ``start()`` has run and ``stop()`` has not.
  bool started_{false};

  /// Guards jobs_ and stopping_, and is waited on by the worker thread.
  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::deque<Job> jobs_;
  /// Set by ``stop()``; makes the worker and connect loops unwind.
  bool stopping_{true};

  std::thread worker_thread_;
  std::thread connect_thread_;
};

}  // namespace idmind_imu

#endif  // IDMIND_IMU__DRIVERS__BRICK_V2_HPP_
