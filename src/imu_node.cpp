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

#include "idmind_imu/imu_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "idmind_imu/drivers/registry.hpp"
#include "idmind_imu/noise_estimator.hpp"
#include "rclcpp/version.h"

namespace idmind_imu
{

namespace
{

/// Build a ParameterDescriptor carrying just a description, as every parameter here does.
rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return descriptor;
}

/// Format a double with two decimals, for diagnostic key/value pairs.
std::string two_decimals(double value)
{
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%.2f", value);
  return std::string(buffer);
}

/// Convert a 3-element parameter vector to a fixed array, or throw if the length is wrong.
std::array<double, 3> to_stddev_array(const std::vector<double> & values, const char * name)
{
  if (values.size() != 3) {
    throw std::invalid_argument(
            std::string(name) + " must have exactly 3 elements (x, y, z), got " +
            std::to_string(values.size()));
  }
  return std::array<double, 3>{values[0], values[1], values[2]};
}

/// Combine a measured variance with a fallback covariance, axis by axis.
///
/// A perfectly quiet axis - a stuck sensor, or one whose quantised output never moves while
/// the robot is still - measures exactly zero variance. Publishing that would claim the
/// reading is perfectly certain, so such an axis takes the fallback instead. The choice is
/// per axis on purpose: one silent axis must not discard a good measurement on the other two.
conversions::Covariance blended_covariance(
  const std::optional<std::array<double, 3>> & measured,
  const conversions::Covariance & fallback)
{
  if (!measured.has_value()) {
    return fallback;
  }

  // A fallback of -1 in slot 0 is the "no estimate available" sentinel, which has no per-axis
  // variances to borrow from.
  const bool fallback_is_sentinel = fallback[0] < 0.0;
  std::array<double, 3> variance{fallback[0], fallback[4], fallback[8]};

  for (size_t axis = 0; axis < 3; ++axis) {
    const double value = (*measured)[axis];
    if (value > 0.0) {
      variance[axis] = value;
    } else if (fallback_is_sentinel) {
      return fallback;
    }
  }
  return conversions::covariance_from_variance(variance);
}

/// Parse a 3-element stddev parameter into \p target.
///
/// Fills \p result and returns false when the length is wrong, so the caller can bail out.
bool assign_stddev(
  const rclcpp::Parameter & parameter, std::array<double, 3> & target,
  rcl_interfaces::msg::SetParametersResult & result)
{
  const auto & values = parameter.as_double_array();
  if (values.size() != 3) {
    result.successful = false;
    result.reason = parameter.get_name() + " must have exactly 3 elements (x, y, z)";
    return false;
  }
  target = std::array<double, 3>{values[0], values[1], values[2]};
  return true;
}

/// Seconds between two steady-clock points.
double seconds_between(
  std::chrono::steady_clock::time_point from, std::chrono::steady_clock::time_point to)
{
  return std::chrono::duration<double>(to - from).count();
}

}  // namespace

namespace
{
/// 0.3 deg/s in rad/s - the stddev the pre-parameter hardcoded variance was built from.
const double kGyroStddev = 0.3 * M_PI / 180.0;
}  // namespace

const std::vector<double> kDefaultAngularVelocityStddev{kGyroStddev, kGyroStddev, kGyroStddev};
const std::vector<double> kDefaultLinearAccelerationStddev{0.1, 0.1, std::sqrt(0.05)};
const std::vector<double> kDefaultMagneticFieldStddev{0.6e-6, 0.6e-6, 0.6e-6};

ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("idmind_imu", options)
{
  const std::string node_prefix = std::string(this->get_name()) + "/";

  pub_callbacks_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  srv_callbacks_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  main_callbacks_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // -- Parameters ---------------------------------------------------------------------------
  driver_name_ = this->declare_parameter<std::string>(
    "driver", "brick_v2", describe("Name of the IMU driver backend to use"));
  host_ = this->declare_parameter<std::string>(
    "host", "localhost", describe("Hostname/IP of the IMU transport (e.g. BrickDaemon)"));
  port_ = this->declare_parameter<int>("port", 4223, describe("Port of the IMU transport"));
  control_freq_ = this->declare_parameter<double>(
    "control_freq", 20.0, describe("Frequency of the watchdog loop"));
  imu_freq_ = this->declare_parameter<double>(
    "imu_freq", 20.0, describe("Frequency of IMU stream"));
  imu_frame_ = this->declare_parameter<std::string>(
    "imu_frame", "imu", describe("Frame name for the IMU"));
  imu_leds_ = this->declare_parameter<bool>(
    "imu_leds", false, describe("Enable/Disable IMU Leds"));
  imu_fusion_mode_ = this->declare_parameter<int>(
    "imu_fusion_mode", 2, describe("Fusion Mode of the IMU"));
  timeout_ = this->declare_parameter<double>(
    "timeout", 1.0, describe("Timeout for IMU Error"));
  auto_reconnect_ = this->declare_parameter<bool>(
    "auto_reconnect", true, describe("Enable driver-level auto-reconnect"));
  acceleration_source_ = this->declare_parameter<std::string>(
    "acceleration_source", "linear",
    describe("Which device field to report as linear_acceleration"));
  orientation_stddev_ = this->declare_parameter<double>(
    "orientation_stddev", 0.01, describe("Base orientation stddev at full calibration"));
  // Defaults to 0, which sensor_msgs/Temperature reads as "variance unknown" - the same thing
  // this node published before the parameter existed. Set it to opt into a real variance.
  temperature_stddev_ = this->declare_parameter<double>(
    "temperature_stddev", 0.0,
    describe("Temperature stddev in degrees Celsius; 0 publishes variance 0 (unknown)"));
  angular_velocity_stddev_ = to_stddev_array(
    this->declare_parameter<std::vector<double>>(
      "angular_velocity_stddev", kDefaultAngularVelocityStddev,
      describe("Per-axis angular velocity stddev (rad/s), scaled by gyro calibration")),
    "angular_velocity_stddev");
  linear_acceleration_stddev_ = to_stddev_array(
    this->declare_parameter<std::vector<double>>(
      "linear_acceleration_stddev", kDefaultLinearAccelerationStddev,
      describe("Per-axis linear acceleration stddev (m/s^2), scaled by accel calibration")),
    "linear_acceleration_stddev");
  magnetic_field_stddev_ = to_stddev_array(
    this->declare_parameter<std::vector<double>>(
      "magnetic_field_stddev", kDefaultMagneticFieldStddev,
      describe("Per-axis magnetic field stddev (tesla), scaled by mag calibration")),
    "magnetic_field_stddev");

  noise_window_ = this->declare_parameter<int>(
    "noise_window", 100,
    describe("Samples of live signal used to measure covariance; <2 disables measurement"));
  noise_estimate_when_still_ = this->declare_parameter<bool>(
    "noise_estimate_when_still", true,
    describe("Only measure noise while the robot looks stationary"));
  stationary_gyro_threshold_ = this->declare_parameter<double>(
    "stationary_gyro_threshold", 0.02,
    describe("Angular velocity magnitude (rad/s) below which the robot counts as still"));
  stationary_accel_threshold_ = this->declare_parameter<double>(
    "stationary_accel_threshold", 0.2,
    describe("Acceleration deviation (m/s^2) from the window mean allowed while still"));

  if (control_freq_ <= 0.0) {
    throw std::invalid_argument("control_freq must be > 0");
  }
  reset_noise_estimators_locked();

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->update_parameters(parameters);
    });

  // -- Services ------------------------------------------------------------------------------
  ready_service_ = this->create_service<std_srvs::srv::Trigger>(
    node_prefix + "ready",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      this->report_ready(request, response);
    },
    // Humble's create_service only takes an rmw_qos_profile_t here; Iron onwards deprecates
    // that overload in favour of rclcpp::QoS. Pick by rclcpp version so both build warning-free.
#if RCLCPP_VERSION_MAJOR >= 21
    rclcpp::ServicesQoS(), srv_callbacks_);
#else
    rmw_qos_profile_services_default, srv_callbacks_);
#endif

  // -- Publishers ----------------------------------------------------------------------------
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = pub_callbacks_;

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(node_prefix + "imu", 10, pub_options);
  temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
    node_prefix + "temperature", 10, pub_options);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
    node_prefix + "magnetic_field", 10, pub_options);
  euler_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    node_prefix + "euler", 10, pub_options);
  gravity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    node_prefix + "gravity", 10, pub_options);
  calib_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
    node_prefix + "calibration", 10, pub_options);
  timer_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    node_prefix + "timer", 10, pub_options);

  // -- Driver ---------------------------------------------------------------------------------
  driver_ = make_driver(
    driver_name_, driver_config(),
    [this](const ImuSample & sample) {this->on_sample(sample);},
    this->get_logger());
  driver_->start();

  // -- Diagnostics ------------------------------------------------------------------------------
  updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  updater_->setHardwareID(driver_->state().hardware_id);
  updater_->add("Connection", this, &ImuNode::diagnose_connection);
  updater_->add("Data flow", this, &ImuNode::diagnose_data_flow);
  updater_->add("Calibration", this, &ImuNode::diagnose_calibration);

  // -- Watchdog ----------------------------------------------------------------------------------
  restart_watchdog_timer(control_freq_);

  ready_ = true;
  log("Node is initialized.");
}

ImuNode::~ImuNode()
{
  stop_driver();
}

void ImuNode::stop_driver()
{
  if (driver_) {
    driver_->stop();
  }
}

DriverState ImuNode::driver_state() const
{
  return driver_->state();
}

ImuNode::PublishParams ImuNode::publish_params_locked() const
{
  PublishParams params;
  params.imu_frame = imu_frame_;
  params.orientation_stddev = orientation_stddev_;
  params.temperature_stddev = temperature_stddev_;
  params.angular_velocity_stddev = angular_velocity_stddev_;
  params.linear_acceleration_stddev = linear_acceleration_stddev_;
  params.magnetic_field_stddev = magnetic_field_stddev_;
  params.measured_angular_velocity = angular_velocity_noise_.variance();
  params.measured_linear_acceleration = linear_acceleration_noise_.variance();
  params.measured_magnetic_field = magnetic_field_noise_.variance();
  params.measured_orientation = orientation_noise_.variance();
  return params;
}

void ImuNode::reset_noise_estimators_locked()
{
  const size_t window = noise_window_ > 1 ? static_cast<size_t>(noise_window_) : 2;
  angular_velocity_noise_ = RollingVariance(window, AngleWrap::None);
  linear_acceleration_noise_ = RollingVariance(window, AngleWrap::None);
  magnetic_field_noise_ = RollingVariance(window, AngleWrap::None);
  // Euler angles wrap: a yaw sitting near +/- pi would otherwise show enormous fake variance.
  orientation_noise_ = RollingVariance(window, AngleWrap::Radians);
}

bool ImuNode::is_stationary_locked(const ImuSample & sample) const
{
  if (sample.angular_velocity.has_value()) {
    const auto & w = *sample.angular_velocity;
    const double rate = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (rate > stationary_gyro_threshold_) {
      return false;
    }
  }

  // Rotation alone misses straight-line acceleration, so also require the acceleration to sit
  // close to what the window has been seeing. Only meaningful once the window has filled.
  if (sample.linear_acceleration.has_value() && linear_acceleration_noise_.ready()) {
    const auto & a = *sample.linear_acceleration;
    const auto centre = linear_acceleration_noise_.mean();
    double squared = 0.0;
    for (size_t axis = 0; axis < 3; ++axis) {
      const double d = a[axis] - centre[axis];
      squared += d * d;
    }
    if (std::sqrt(squared) > stationary_accel_threshold_) {
      return false;
    }
  }

  return true;
}

void ImuNode::update_noise_estimators_locked(const ImuSample & sample)
{
  if (noise_window_ < 2) {
    return;
  }
  if (noise_estimate_when_still_ && !is_stationary_locked(sample)) {
    // Hold the last good estimate rather than poisoning it with motion.
    return;
  }

  if (sample.angular_velocity.has_value()) {
    angular_velocity_noise_.push(*sample.angular_velocity);
  }
  if (sample.linear_acceleration.has_value()) {
    linear_acceleration_noise_.push(*sample.linear_acceleration);
  }
  if (sample.magnetic_field.has_value()) {
    magnetic_field_noise_.push(*sample.magnetic_field);
  }
  if (sample.euler.has_value()) {
    orientation_noise_.push(*sample.euler);
  }
}

DriverConfig ImuNode::driver_config() const
{
  DriverConfig config;
  config.host = host_;
  config.port = port_;
  config.imu_freq = imu_freq_;
  config.imu_leds = imu_leds_;
  config.imu_fusion_mode = imu_fusion_mode_;
  config.auto_reconnect = auto_reconnect_;
  config.acceleration_source = acceleration_source_;
  config.orientation_stddev = orientation_stddev_;
  return config;
}

// -- Services / parameters --------------------------------------------------------------------

void ImuNode::report_ready(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  response->success = ready_;
  response->message =
    std::string(this->get_name()) + " is " + (ready_ ? "ready" : "not ready");
}

rcl_interfaces::msg::SetParametersResult ImuNode::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool driver_config_changed = false;
  bool control_freq_changed = false;

  // Held across the whole scan: the publish parameters below are read by on_sample on a
  // driver thread. The lock is released before apply_config so the node lock is never held
  // while acquiring the driver's, which would invert the driver callback's lock order.
  std::unique_lock<std::mutex> lock(mutex_);

  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();

    if (name == "driver") {
      driver_name_ = parameter.as_string();
    } else if (name == "host") {
      host_ = parameter.as_string();
      driver_config_changed = true;
    } else if (name == "port") {
      port_ = static_cast<int>(parameter.as_int());
      driver_config_changed = true;
    } else if (name == "control_freq") {
      if (parameter.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "control_freq must be > 0";
        return result;
      }
      control_freq_ = parameter.as_double();
      control_freq_changed = true;
    } else if (name == "imu_freq") {
      imu_freq_ = parameter.as_double();
      driver_config_changed = true;
    } else if (name == "imu_frame") {
      imu_frame_ = parameter.as_string();
    } else if (name == "imu_leds") {
      imu_leds_ = parameter.as_bool();
      driver_config_changed = true;
    } else if (name == "imu_fusion_mode") {
      imu_fusion_mode_ = static_cast<int>(parameter.as_int());
      driver_config_changed = true;
    } else if (name == "timeout") {
      timeout_ = parameter.as_double();
    } else if (name == "auto_reconnect") {
      auto_reconnect_ = parameter.as_bool();
      driver_config_changed = true;
    } else if (name == "acceleration_source") {
      acceleration_source_ = parameter.as_string();
      driver_config_changed = true;
    } else if (name == "orientation_stddev") {
      // 0 is allowed and means "unknown": orientation_covariance then reports the -1
      // sentinel rather than an all-zero matrix. Negative is simply nonsense.
      if (parameter.as_double() < 0.0) {
        result.successful = false;
        result.reason = "orientation_stddev must be >= 0";
        return result;
      }
      orientation_stddev_ = parameter.as_double();
      driver_config_changed = true;
    } else if (name == "temperature_stddev") {
      if (parameter.as_double() < 0.0) {
        result.successful = false;
        result.reason = "temperature_stddev must be >= 0";
        return result;
      }
      temperature_stddev_ = parameter.as_double();
    } else if (name == "angular_velocity_stddev") {
      if (!assign_stddev(parameter, angular_velocity_stddev_, result)) {
        return result;
      }
    } else if (name == "linear_acceleration_stddev") {
      if (!assign_stddev(parameter, linear_acceleration_stddev_, result)) {
        return result;
      }
    } else if (name == "magnetic_field_stddev") {
      if (!assign_stddev(parameter, magnetic_field_stddev_, result)) {
        return result;
      }
    } else if (name == "noise_window") {
      noise_window_ = static_cast<int>(parameter.as_int());
      // Resizing invalidates the accumulated samples, so start the estimate over.
      reset_noise_estimators_locked();
    } else if (name == "noise_estimate_when_still") {
      noise_estimate_when_still_ = parameter.as_bool();
    } else if (name == "stationary_gyro_threshold") {
      stationary_gyro_threshold_ = parameter.as_double();
    } else if (name == "stationary_accel_threshold") {
      stationary_accel_threshold_ = parameter.as_double();
    }
  }

  const DriverConfig config = driver_config();
  const double control_freq = control_freq_;
  lock.unlock();

  if (control_freq_changed) {
    restart_watchdog_timer(control_freq);
  }

  // Config is forwarded on change only; the driver must never be polled on a schedule.
  if (driver_config_changed && driver_) {
    driver_->apply_config(config);
  }

  return result;
}

// -- Sample handling (runs on a driver thread) ------------------------------------------------

void ImuNode::on_sample(const ImuSample & sample)
{
  const rclcpp::Time stamp = this->get_clock()->now();
  const int fusion_mode = sample.fusion_mode.value_or(0);

  // Snapshot the parameters this publish pass needs while holding the lock. They are written
  // by update_parameters on an executor thread, so reading them straight off the members from
  // this driver thread would be a data race - and a torn std::string read is not theoretical.
  PublishParams params;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    last_sample_ = now;
    sample_times_.push_back(now);
    if (sample_times_.size() > kRateWindow) {
      sample_times_.pop_front();
    }
    if (sample.calibration.has_value()) {
      last_calibration_ = sample.calibration;
    }
    update_noise_estimators_locked(sample);
    params = publish_params_locked();
  }

  if (sample.orientation.has_value() || sample.angular_velocity.has_value() ||
    sample.linear_acceleration.has_value())
  {
    publish_imu(stamp, sample, fusion_mode, params);
  }

  if (sample.temperature.has_value()) {
    publish_temperature(stamp, *sample.temperature, params);
  }

  if (sample.magnetic_field.has_value()) {
    publish_magnetic_field(stamp, *sample.magnetic_field, sample.calibration, params);
  }

  if (sample.euler.has_value()) {
    std_msgs::msg::Float32 euler_msg;
    euler_msg.data = static_cast<float>((*sample.euler)[2]);
    euler_pub_->publish(euler_msg);
  }

  if (sample.gravity.has_value()) {
    publish_gravity(stamp, *sample.gravity, params);
  }

  if (sample.calibration.has_value()) {
    std_msgs::msg::UInt8MultiArray calib_msg;
    calib_msg.data.assign(sample.calibration->begin(), sample.calibration->end());
    calib_pub_->publish(calib_msg);
  }
}

void ImuNode::publish_imu(
  const rclcpp::Time & stamp, const ImuSample & sample, int fusion_mode,
  const PublishParams & params)
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = params.imu_frame;

  if (sample.orientation.has_value()) {
    msg.orientation.x = (*sample.orientation)[0];
    msg.orientation.y = (*sample.orientation)[1];
    msg.orientation.z = (*sample.orientation)[2];
    msg.orientation.w = (*sample.orientation)[3];
  }
  // Prefer variance measured from the live signal; fall back to the configured stddev scaled
  // by calibration when no window has filled yet. Fusion off still wins over both.
  const auto orientation_fallback = conversions::orientation_covariance(
    fusion_mode, sample.calibration, params.orientation_stddev);
  if (fusion_mode == 0) {
    // No orientation estimate exists at all; a measured variance cannot rescue that.
    msg.orientation_covariance = orientation_fallback;
  } else {
    msg.orientation_covariance =
      blended_covariance(params.measured_orientation, orientation_fallback);
  }

  if (sample.angular_velocity.has_value()) {
    msg.angular_velocity.x = (*sample.angular_velocity)[0];
    msg.angular_velocity.y = (*sample.angular_velocity)[1];
    msg.angular_velocity.z = (*sample.angular_velocity)[2];
  }
  msg.angular_velocity_covariance = blended_covariance(
    params.measured_angular_velocity,
    conversions::scaled_covariance(
      params.angular_velocity_stddev, sample.calibration,
      conversions::CalibrationAxis::Gyroscope));

  if (sample.linear_acceleration.has_value()) {
    msg.linear_acceleration.x = (*sample.linear_acceleration)[0];
    msg.linear_acceleration.y = (*sample.linear_acceleration)[1];
    msg.linear_acceleration.z = (*sample.linear_acceleration)[2];
  }
  msg.linear_acceleration_covariance = blended_covariance(
    params.measured_linear_acceleration,
    conversions::scaled_covariance(
      params.linear_acceleration_stddev, sample.calibration,
      conversions::CalibrationAxis::Accelerometer));

  imu_pub_->publish(msg);
}

void ImuNode::publish_temperature(
  const rclcpp::Time & stamp, double temperature, const PublishParams & params)
{
  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = params.imu_frame;
  msg.temperature = temperature;
  // sensor_msgs/Temperature treats variance 0.0 as "unknown", which is what a
  // temperature_stddev of 0 deliberately reproduces.
  msg.variance = params.temperature_stddev * params.temperature_stddev;
  temp_pub_->publish(msg);
}

void ImuNode::publish_magnetic_field(
  const rclcpp::Time & stamp, const std::array<double, 3> & magnetic_field,
  const std::optional<conversions::Calibration> & calibration,
  const PublishParams & params)
{
  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = params.imu_frame;
  msg.magnetic_field.x = magnetic_field[0];
  msg.magnetic_field.y = magnetic_field[1];
  msg.magnetic_field.z = magnetic_field[2];
  msg.magnetic_field_covariance = blended_covariance(
    params.measured_magnetic_field,
    conversions::scaled_covariance(
      params.magnetic_field_stddev, calibration, conversions::CalibrationAxis::Magnetometer));
  mag_pub_->publish(msg);
}

void ImuNode::publish_gravity(
  const rclcpp::Time & stamp, const std::array<double, 3> & gravity,
  const PublishParams & params)
{
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = params.imu_frame;
  msg.vector.x = gravity[0];
  msg.vector.y = gravity[1];
  msg.vector.z = gravity[2];
  gravity_pub_->publish(msg);
}

// -- Watchdog (runs on an executor thread, no hardware I/O) -----------------------------------

void ImuNode::restart_watchdog_timer(double control_freq)
{
  const auto period = std::chrono::duration<double>(1.0 / control_freq);
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {this->watchdog();},
    main_callbacks_);
}

void ImuNode::watchdog()
{
  try {
    const auto now = std::chrono::steady_clock::now();
    const double period = seconds_between(last_watchdog_, now);
    last_watchdog_ = now;

    std_msgs::msg::Float32 heartbeat;
    heartbeat.data = static_cast<float>(period);
    timer_pub_->publish(heartbeat);

    std::optional<std::chrono::steady_clock::time_point> last_sample;
    double timeout = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_sample = last_sample_;
      timeout = timeout_;
    }
    if (!last_sample.has_value() || seconds_between(*last_sample, now) > timeout) {
      log("No data from IMU driver", "warn");
    }

    // Diagnostics are NOT forced from here: the Updater runs its own 1 Hz timer. Forcing an
    // update every watchdog tick would republish /diagnostics at control_freq.
  } catch (const std::exception & exc) {
    // CRITICAL: log and continue. Never shut down or cancel the timer from here - a single
    // transient exception must not leave the node permanently inert.
    log(std::string("Exception in watchdog: ") + exc.what(), "error");
  } catch (...) {
    log("Unknown exception in watchdog", "error");
  }
}

// -- Diagnostics tasks ------------------------------------------------------------------------

void ImuNode::diagnose_connection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const DriverState state = driver_->state();
  if (!state.connected) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not connected: " + state.detail);
  } else if (!state.device_present) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No device found: " + state.detail);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, state.detail);
  }
}

void ImuNode::diagnose_data_flow(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto now = std::chrono::steady_clock::now();

  std::optional<std::chrono::steady_clock::time_point> last_sample;
  std::deque<std::chrono::steady_clock::time_point> sample_times;
  double timeout = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_sample = last_sample_;
    sample_times = sample_times_;
    timeout = timeout_;
  }

  const bool have_age = last_sample.has_value();
  const double age = have_age ? seconds_between(*last_sample, now) : 0.0;

  double rate = 0.0;
  if (sample_times.size() >= 2) {
    const double span = seconds_between(sample_times.front(), sample_times.back());
    if (span > 0.0) {
      rate = static_cast<double>(sample_times.size() - 1) / span;
    }
  }

  if (!have_age || age > timeout) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No data within timeout");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Receiving data");
  }

  stat.add("rate_hz", two_decimals(rate));
  stat.add("age_s", have_age ? two_decimals(age) : std::string("n/a"));
}

void ImuNode::diagnose_calibration(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<conversions::Calibration> calibration;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calibration = last_calibration_;
  }

  conversions::Calibration values{0, 0, 0, 0};
  if (!calibration.has_value()) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "No calibration data yet");
  } else {
    values = *calibration;
    const uint8_t worst = *std::min_element(values.begin(), values.end());
    if (worst == 0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Partially uncalibrated");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Fully calibrated");
    }
  }

  stat.add("sys", std::to_string(values[0]));
  stat.add("gyro", std::to_string(values[1]));
  stat.add("acc", std::to_string(values[2]));
  stat.add("mag", std::to_string(values[3]));
}

// -- Logging ----------------------------------------------------------------------------------

void ImuNode::log(const std::string & message, const std::string & alert)
{
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (message == last_message_ && seconds_between(last_message_time_, now) < 1.0) {
      return;
    }
    last_message_ = message;
    last_message_time_ = now;
  }

  const std::string full = std::string(this->get_name()) + ": " + message;
  if (alert == "warn") {
    RCLCPP_WARN(this->get_logger(), "%s", full.c_str());
  } else if (alert == "error") {
    RCLCPP_ERROR(this->get_logger(), "%s", full.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "%s", full.c_str());
  }
}

}  // namespace idmind_imu
