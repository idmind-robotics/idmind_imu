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

#include "idmind_imu/drivers/brick_v2.hpp"

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "idmind_imu/conversions.hpp"

namespace idmind_imu
{

namespace
{

/// How long the connect loop waits between retries of the blocking ``ipcon_connect``.
constexpr auto kConnectRetryInterval = std::chrono::milliseconds(1000);

/// Copy a fixed-size C array of counts into the std::array the conversions take.
template<size_t N>
std::array<int16_t, N> to_array(const int16_t * raw)
{
  std::array<int16_t, N> out{};
  for (size_t i = 0; i < N; ++i) {
    out[i] = raw[i];
  }
  return out;
}

}  // namespace

// -- Device -----------------------------------------------------------------------------------

ImuBrickV2Driver::Device::Device(const std::string & uid, IPConnection * ipcon)
: uid_(uid)
{
  imu_v2_create(&imu_, uid_.c_str(), ipcon);
}

ImuBrickV2Driver::Device::~Device()
{
  imu_v2_destroy(&imu_);
}

// -- Construction / destruction ---------------------------------------------------------------

ImuBrickV2Driver::ImuBrickV2Driver(
  const DriverConfig & config, SampleCallback on_sample, rclcpp::Logger logger)
: ImuDriver(config, std::move(on_sample), std::move(logger))
{
  ipcon_create(&ipcon_);
  ipcon_created_ = true;
}

ImuBrickV2Driver::~ImuBrickV2Driver()
{
  stop();
  if (ipcon_created_) {
    ipcon_destroy(&ipcon_);
    ipcon_created_ = false;
  }
}

// -- ImuDriver interface ----------------------------------------------------------------------

void ImuBrickV2Driver::start()
{
  bool auto_reconnect = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_) {
      return;
    }
    started_ = true;
    auto_reconnect = config_.auto_reconnect;
  }

  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    stopping_ = false;
    jobs_.clear();
  }

  ipcon_set_auto_reconnect(&ipcon_, auto_reconnect);
  ipcon_register_callback(
    &ipcon_, IPCON_CALLBACK_CONNECTED,
    reinterpret_cast<void (*)(void)>(&ImuBrickV2Driver::connected_trampoline), this);
  ipcon_register_callback(
    &ipcon_, IPCON_CALLBACK_DISCONNECTED,
    reinterpret_cast<void (*)(void)>(&ImuBrickV2Driver::disconnected_trampoline), this);
  ipcon_register_callback(
    &ipcon_, IPCON_CALLBACK_ENUMERATE,
    reinterpret_cast<void (*)(void)>(&ImuBrickV2Driver::enumerate_trampoline), this);

  worker_thread_ = std::thread(&ImuBrickV2Driver::worker_loop, this);
  connect_thread_ = std::thread(&ImuBrickV2Driver::connect_loop, this);
}

void ImuBrickV2Driver::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_) {
      return;
    }
    started_ = false;
  }

  // Wake both background loops before touching the connection, so neither can start new work
  // against a connection that is about to go away.
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    stopping_ = true;
    jobs_.clear();
  }
  job_cv_.notify_all();

  ipcon_register_callback(&ipcon_, IPCON_CALLBACK_CONNECTED, nullptr, nullptr);
  ipcon_register_callback(&ipcon_, IPCON_CALLBACK_DISCONNECTED, nullptr, nullptr);
  ipcon_register_callback(&ipcon_, IPCON_CALLBACK_ENUMERATE, nullptr, nullptr);

  // Returns once the library's callback thread has exited, so no callback can be in flight
  // after this point. Safe to call when never connected: it returns E_NOT_CONNECTED.
  ipcon_disconnect(&ipcon_);

  // Joining is unbounded by design: detaching would leave a thread holding `this` after the
  // driver is destroyed. A connect() to an unreachable host can therefore delay shutdown by
  // as long as the OS TCP connect timeout.
  if (connect_thread_.joinable()) {
    connect_thread_.join();
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  // The connect loop may have completed a connect between the disconnect above and its own
  // stop check; tear that one down too now that no thread can open another.
  ipcon_disconnect(&ipcon_);

  // Only now, with no thread left that could be mid-call on it, release the device. `device`
  // looks unused on purpose: it holds the last reference until the lock is released, so
  // imu_v2_destroy runs outside the critical section.
  std::shared_ptr<Device> device;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    device = std::move(device_);
    device_.reset();
    state_.connected = false;
    state_.device_present = false;
    state_.detail = "stopped";
  }
}

void ImuBrickV2Driver::apply_config(const DriverConfig & config)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
  }
  enqueue([this]() {do_apply_config();});
}

DriverState ImuBrickV2Driver::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

// -- C callback trampolines -------------------------------------------------------------------

void ImuBrickV2Driver::connected_trampoline(uint8_t connect_reason, void * user_data)
{
  static_cast<ImuBrickV2Driver *>(user_data)->on_connected(connect_reason);
}

void ImuBrickV2Driver::disconnected_trampoline(uint8_t disconnect_reason, void * user_data)
{
  static_cast<ImuBrickV2Driver *>(user_data)->on_disconnected(disconnect_reason);
}

void ImuBrickV2Driver::enumerate_trampoline(
  const char * uid, const char * connected_uid, char position,
  uint8_t hardware_version[3], uint8_t firmware_version[3],
  uint16_t device_identifier, uint8_t enumeration_type, void * user_data)
{
  (void)connected_uid;
  (void)position;
  (void)hardware_version;
  (void)firmware_version;
  static_cast<ImuBrickV2Driver *>(user_data)->on_enumerate(
    uid, device_identifier, enumeration_type);
}

void ImuBrickV2Driver::all_data_trampoline(
  int16_t acceleration[3], int16_t magnetic_field[3], int16_t angular_velocity[3],
  int16_t euler_angle[3], int16_t quaternion[4], int16_t linear_acceleration[3],
  int16_t gravity_vector[3], int8_t temperature, uint8_t calibration_status,
  void * user_data)
{
  static_cast<ImuBrickV2Driver *>(user_data)->on_all_data(
    acceleration, magnetic_field, angular_velocity, euler_angle, quaternion,
    linear_acceleration, gravity_vector, temperature, calibration_status);
}

// -- Connection lifecycle ---------------------------------------------------------------------

void ImuBrickV2Driver::connect_loop()
{
  std::string host;
  int port = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    host = config_.host;
    port = config_.port;
  }

  while (true) {
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      if (stopping_) {
        return;
      }
    }

    const int result = ipcon_connect(&ipcon_, host.c_str(), static_cast<uint16_t>(port));
    if (result == E_OK || result == E_ALREADY_CONNECTED) {
      return;
    }

    RCLCPP_WARN(
      logger_, "IMU brick: connect to %s:%d failed (error %d)", host.c_str(), port, result);

    // Wait out the retry interval, but wake immediately if stop() fires meanwhile.
    std::unique_lock<std::mutex> lock(job_mutex_);
    job_cv_.wait_for(lock, kConnectRetryInterval, [this] {return stopping_;});
    if (stopping_) {
      return;
    }
  }
}

void ImuBrickV2Driver::on_connected(uint8_t connect_reason)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = true;
    state_.detail = "connected, enumerating";
  }
  RCLCPP_INFO(
    logger_, "IMU brick: connected to BrickDaemon (reason=%u)",
    static_cast<unsigned>(connect_reason));

  const int result = ipcon_enumerate(&ipcon_);
  if (result != E_OK) {
    RCLCPP_ERROR(logger_, "IMU brick: enumerate failed (error %d)", result);
  }
}

void ImuBrickV2Driver::on_disconnected(uint8_t disconnect_reason)
{
  std::shared_ptr<Device> dropped;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
    state_.device_present = false;
    state_.detail =
      "disconnected (reason=" + std::to_string(static_cast<unsigned>(disconnect_reason)) + ")";
    // Held until the lock is released, so imu_v2_destroy runs outside the critical section.
    dropped = std::move(device_);
    device_.reset();
  }
  RCLCPP_WARN(
    logger_, "IMU brick: disconnected from BrickDaemon (reason=%u)",
    static_cast<unsigned>(disconnect_reason));
}

void ImuBrickV2Driver::on_enumerate(
  const char * uid, uint16_t device_identifier, uint8_t enumeration_type)
{
  if (enumeration_type == IPCON_ENUMERATION_TYPE_DISCONNECTED) {
    std::shared_ptr<Device> dropped;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (device_ && device_->uid() == uid) {
        dropped = std::move(device_);
        device_.reset();
        state_.device_present = false;
        state_.detail = "device disconnected";
      }
    }
    return;
  }

  if (device_identifier != IMU_V2_DEVICE_IDENTIFIER) {
    return;
  }

  // Cheap, non-blocking work only: creating the device object and registering a callback are
  // both local operations. Every hardware round-trip is deferred to the worker thread.
  auto device = std::make_shared<Device>(uid, &ipcon_);
  imu_v2_register_callback(
    device->handle(), IMU_V2_CALLBACK_ALL_DATA,
    reinterpret_cast<void (*)(void)>(&ImuBrickV2Driver::all_data_trampoline), this);

  std::shared_ptr<Device> replaced;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    replaced = std::move(device_);
    device_ = device;
    state_.device_present = true;
    state_.hardware_id = std::string("IMU Brick 2.0 (") + uid + ")";
    state_.detail = "device enumerated";
  }

  RCLCPP_INFO(logger_, "IMU brick: found device %s", uid);
  enqueue([this]() {do_apply_config();});
}

// -- Sensor data ------------------------------------------------------------------------------

void ImuBrickV2Driver::on_all_data(
  const int16_t acceleration[3], const int16_t magnetic_field[3],
  const int16_t angular_velocity[3], const int16_t euler_angle[3],
  const int16_t quaternion[4], const int16_t linear_acceleration[3],
  const int16_t gravity_vector[3], int8_t temperature, uint8_t calibration_status)
{
  // Runs on the TinkerForge callback thread. Never let an exception escape: it would cross a
  // C frame, and a broken consumer must not be able to kill delivery of subsequent samples.
  try {
    int fusion_mode = 0;
    std::string accel_source;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fusion_mode = config_.imu_fusion_mode;
      accel_source = config_.acceleration_source;
    }

    const auto linear = (accel_source == "raw") ?
      conversions::acceleration_from_brick(to_array<3>(acceleration)) :
      conversions::acceleration_from_brick(to_array<3>(linear_acceleration));

    ImuSample sample;
    sample.orientation = conversions::quaternion_from_brick(to_array<4>(quaternion));
    sample.angular_velocity =
      conversions::angular_velocity_from_brick(to_array<3>(angular_velocity));
    sample.linear_acceleration = linear;
    sample.magnetic_field = conversions::magnetic_field_from_brick(to_array<3>(magnetic_field));
    sample.gravity = conversions::acceleration_from_brick(to_array<3>(gravity_vector));
    sample.euler = conversions::euler_from_brick(to_array<3>(euler_angle));
    sample.temperature = static_cast<double>(temperature);
    sample.calibration = conversions::decode_calibration(calibration_status);
    sample.fusion_mode = fusion_mode;

    if (on_sample_) {
      on_sample_(sample);
    }
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(logger_, "IMU brick: all-data callback failed: %s", exc.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "IMU brick: all-data callback failed with a non-standard exception");
  }
}

// -- Configuration (worker thread only) -------------------------------------------------------

void ImuBrickV2Driver::enqueue(Job job)
{
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (stopping_) {
      return;
    }
    jobs_.push_back(std::move(job));
  }
  job_cv_.notify_one();
}

void ImuBrickV2Driver::worker_loop()
{
  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_cv_.wait(lock, [this] {return stopping_ || !jobs_.empty();});
      if (stopping_) {
        return;
      }
      job = std::move(jobs_.front());
      jobs_.pop_front();
    }

    try {
      job();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(logger_, "IMU brick: config job failed: %s", exc.what());
    }
  }
}

void ImuBrickV2Driver::do_apply_config()
{
  // Copy the device handle and the desired config out under the lock, then do every blocking
  // hardware round-trip with the lock released. The shared_ptr keeps the device alive even if
  // the enumerate callback swaps in a replacement meanwhile.
  std::shared_ptr<Device> device;
  DriverConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    device = device_;
    config = config_;
  }

  if (!device) {
    return;
  }

  apply_leds(device->handle(), config);
  apply_fusion_mode(device->handle(), config);
  apply_data_period(device->handle(), config);
}

void ImuBrickV2Driver::apply_leds(IMUV2 * imu, const DriverConfig & config)
{
  // are_leds_on (the orientation/direction LEDs) and is_status_led_enabled are independent on
  // the hardware, so they are read and driven separately. Combining them cannot converge.
  const bool desired = config.imu_leds;

  bool leds_on = false;
  int result = imu_v2_are_leds_on(imu, &leds_on);
  if (result != E_OK) {
    RCLCPP_ERROR(logger_, "IMU brick: are_leds_on failed (error %d)", result);
  } else if (leds_on != desired) {
    result = desired ? imu_v2_leds_on(imu) : imu_v2_leds_off(imu);
    if (result != E_OK) {
      RCLCPP_ERROR(logger_, "IMU brick: setting LEDs failed (error %d)", result);
    }
  }

  bool status_led = false;
  result = imu_v2_is_status_led_enabled(imu, &status_led);
  if (result != E_OK) {
    RCLCPP_ERROR(logger_, "IMU brick: is_status_led_enabled failed (error %d)", result);
  } else if (status_led != desired) {
    result = desired ? imu_v2_enable_status_led(imu) : imu_v2_disable_status_led(imu);
    if (result != E_OK) {
      RCLCPP_ERROR(logger_, "IMU brick: setting the status LED failed (error %d)", result);
    }
  }
}

void ImuBrickV2Driver::apply_fusion_mode(IMUV2 * imu, const DriverConfig & config)
{
  const uint8_t desired = static_cast<uint8_t>(config.imu_fusion_mode);

  uint8_t current = 0;
  int result = imu_v2_get_sensor_fusion_mode(imu, &current);
  if (result != E_OK) {
    RCLCPP_ERROR(logger_, "IMU brick: get_sensor_fusion_mode failed (error %d)", result);
    return;
  }
  if (current != desired) {
    result = imu_v2_set_sensor_fusion_mode(imu, desired);
    if (result != E_OK) {
      RCLCPP_ERROR(logger_, "IMU brick: set_sensor_fusion_mode failed (error %d)", result);
    }
  }
}

void ImuBrickV2Driver::apply_data_period(IMUV2 * imu, const DriverConfig & config)
{
  if (config.imu_freq <= 0.0) {
    RCLCPP_WARN(
      logger_, "IMU brick: invalid imu_freq %f, skipping data period update", config.imu_freq);
    return;
  }
  // A period of 0 tells the device to disable the callback entirely, so any imu_freq above
  // 1000 Hz - which truncates to 0 here - must be clamped rather than silently killing the
  // stream.
  const double raw_period = 1000.0 / config.imu_freq;
  uint32_t desired = static_cast<uint32_t>(raw_period);
  if (desired < 1) {
    RCLCPP_WARN(
      logger_, "IMU brick: imu_freq %f would disable the data callback, clamping period to 1ms",
      config.imu_freq);
    desired = 1;
  }

  uint32_t current = 0;
  int result = imu_v2_get_all_data_period(imu, &current);
  if (result != E_OK) {
    RCLCPP_ERROR(logger_, "IMU brick: get_all_data_period failed (error %d)", result);
    return;
  }
  if (current != desired) {
    result = imu_v2_set_all_data_period(imu, desired);
    if (result != E_OK) {
      RCLCPP_ERROR(logger_, "IMU brick: set_all_data_period failed (error %d)", result);
    }
  }
}

}  // namespace idmind_imu
