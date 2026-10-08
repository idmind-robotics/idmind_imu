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
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <optional>
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

/// printf into a std::string, for log and diagnostic messages.
std::string format(const char * fmt, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  return std::string(buffer);
}

/// A 3-element parameter vector as a fixed array. The length is validated beforehand.
std::array<double, 3> to_array3(const std::vector<double> & values)
{
  return std::array<double, 3>{values[0], values[1], values[2]};
}

// Diagnostic task names, which are also the ConditionTracker scopes. Monitors key on these.
const char kConnection[] = "Connection";
const char kDataFlow[] = "Data flow";
const char kCalibration[] = "Calibration";

/// Why \p parameter's value is invalid, or nothing if it is fine.
///
/// The one rule set for both startup (invalid -> default, IMU_PARAM_FALLBACK) and runtime
/// (invalid -> the set is rejected with this reason).
std::optional<std::string> check_parameter(const rclcpp::Parameter & parameter)
{
  const std::string & name = parameter.get_name();
  if (name == "control_freq" || name == "imu_freq" || name == "timeout" ||
    name == "frozen_timeout" || name == "quaternion_norm_tolerance" ||
    name == "max_acceleration" || name == "recovery_backoff_initial" ||
    name == "recovery_window" || name == "reconnect_probe_period")
  {
    if (!(parameter.as_double() > 0.0)) {
      return name + " must be > 0";
    }
  } else if (name == "orientation_stddev" || name == "temperature_stddev" ||  // NOLINT
    name == "calibration_grace_period" || name == "log_reminder_period" ||
    name == "stationary_gyro_threshold" || name == "stationary_accel_threshold")
  {
    // 0 is meaningful for each: "unknown" for a stddev, "off" for the others.
    if (!(parameter.as_double() >= 0.0)) {
      return name + " must be >= 0";
    }
  } else if (name == "rate_warn_ratio") {
    const double ratio = parameter.as_double();
    if (!(ratio > 0.0 && ratio <= 1.0)) {
      return name + " must be in (0, 1]";
    }
  } else if (name == "port") {
    const auto port = parameter.as_int();
    if (port < 1 || port > 65535) {
      return name + " must be in [1, 65535]";
    }
  } else if (name == "imu_fusion_mode") {
    const auto mode = parameter.as_int();
    if (mode < 0 || mode > 3) {
      return name + " must be 0, 1, 2 or 3";
    }
  } else if (name == "recovery_max_attempts") {
    if (parameter.as_int() < 0) {
      return name + " must be >= 0";
    }
  } else if (name == "acceleration_source") {
    const std::string & source = parameter.as_string();
    if (source != "linear" && source != "raw") {
      return name + " must be \"linear\" or \"raw\"";
    }
  } else if (name == "angular_velocity_stddev" || name == "linear_acceleration_stddev" ||  // NOLINT
    name == "magnetic_field_stddev")
  {
    if (parameter.as_double_array().size() != 3) {
      return name + " must have exactly 3 elements (x, y, z)";
    }
  }
  return std::nullopt;
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

}  // namespace

namespace
{
/// 0.3 deg/s in rad/s - the stddev the pre-parameter hardcoded variance was built from.
const double kGyroStddev = 0.3 * M_PI / 180.0;
}  // namespace

const std::vector<double> kDefaultAngularVelocityStddev{kGyroStddev, kGyroStddev, kGyroStddev};
const std::vector<double> kDefaultLinearAccelerationStddev{0.1, 0.1, std::sqrt(0.05)};
const std::vector<double> kDefaultMagneticFieldStddev{0.6e-6, 0.6e-6, 0.6e-6};

const char * node_state_name(NodeState state)
{
  switch (state) {
    case NodeState::DISCONNECTED: return "DISCONNECTED";
    case NodeState::NO_DEVICE: return "NO_DEVICE";
    case NodeState::RUNNING: return "RUNNING";
    default: return "RECOVERING";
  }
}

bool ImuNode::Reading::operator==(const Reading & other) const
{
  return orientation == other.orientation && angular_velocity == other.angular_velocity &&
         linear_acceleration == other.linear_acceleration &&
         raw_acceleration == other.raw_acceleration && magnetic_field == other.magnetic_field;
}

double ImuNode::now_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

template<typename T>
T ImuNode::declare(
  const std::string & name, const T & default_value, const std::string & description)
{
  const T value = this->declare_parameter<T>(name, default_value, describe(description));
  const auto reason = check_parameter(rclcpp::Parameter(name, value));
  if (!reason.has_value()) {
    return value;
  }
  param_fallbacks_[name] = *reason;
  this->set_parameter(rclcpp::Parameter(name, default_value));
  return default_value;
}

ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("idmind_imu", options),
  tracker_(
    [this](health::Level level, const std::string & message) {log_condition(level, message);},
    &ImuNode::now_seconds)
{
  const std::string node_prefix = std::string(this->get_name()) + "/";
  start_time_ = now_seconds();
  last_watchdog_ = start_time_;
  last_change_ = start_time_;

  // -- Parameters ---------------------------------------------------------------------------
  driver_name_ = declare<std::string>(
    "driver", "brick_v2", "Name of the IMU driver backend to use");
  host_ = declare<std::string>(
    "host", "localhost", "Hostname/IP of the IMU transport (e.g. BrickDaemon)");
  port_ = declare<int>("port", 4223, "Port of the IMU transport");
  control_freq_ = declare<double>(
    "control_freq", 20.0, "Frequency of the watchdog loop");
  imu_freq_ = declare<double>(
    "imu_freq", 20.0, "Frequency of IMU stream");
  imu_frame_ = declare<std::string>(
    "imu_frame", "imu", "Frame name for the IMU");
  imu_leds_ = declare<bool>(
    "imu_leds", false, "Enable/Disable IMU Leds");
  imu_fusion_mode_ = declare<int>(
    "imu_fusion_mode", 2, "Fusion Mode of the IMU");
  timeout_ = declare<double>(
    "timeout", 1.0, "Timeout for IMU Error");
  auto_reconnect_ = declare<bool>(
    "auto_reconnect", true, "Enable driver-level auto-reconnect");
  acceleration_source_ = declare<std::string>(
    "acceleration_source", "linear",
    "Which device field to report as linear_acceleration");
  orientation_stddev_ = declare<double>(
    "orientation_stddev", 0.01, "Base orientation stddev at full calibration");
  // Defaults to 0, which sensor_msgs/Temperature reads as "variance unknown" - the same thing
  // this node published before the parameter existed. Set it to opt into a real variance.
  temperature_stddev_ = declare<double>(
    "temperature_stddev", 0.0,
    "Temperature stddev in degrees Celsius; 0 publishes variance 0 (unknown)");
  angular_velocity_stddev_ = to_array3(
    declare<std::vector<double>>(
      "angular_velocity_stddev", kDefaultAngularVelocityStddev,
      "Per-axis angular velocity stddev (rad/s), scaled by gyro calibration"));
  linear_acceleration_stddev_ = to_array3(
    declare<std::vector<double>>(
      "linear_acceleration_stddev", kDefaultLinearAccelerationStddev,
      "Per-axis linear acceleration stddev (m/s^2), scaled by accel calibration"));
  magnetic_field_stddev_ = to_array3(
    declare<std::vector<double>>(
      "magnetic_field_stddev", kDefaultMagneticFieldStddev,
      "Per-axis magnetic field stddev (tesla), scaled by mag calibration"));

  noise_window_ = declare<int>(
    "noise_window", 100,
    "Samples of live signal used to measure covariance; <2 disables measurement");
  noise_estimate_when_still_ = declare<bool>(
    "noise_estimate_when_still", true,
    "Only measure noise while the robot looks stationary");
  stationary_gyro_threshold_ = declare<double>(
    "stationary_gyro_threshold", 0.02,
    "Angular velocity magnitude (rad/s) below which the robot counts as still");
  stationary_accel_threshold_ = declare<double>(
    "stationary_accel_threshold", 0.2,
    "Acceleration deviation (m/s^2) from the window mean allowed while still");

  rate_warn_ratio_ = declare<double>(
    "rate_warn_ratio", 0.8,
    "IMU_RATE_LOW when the data rate stays below this fraction of imu_freq");
  frozen_timeout_ = declare<double>(
    "frozen_timeout", 5.0,
    "IMU_DATA_FROZEN when every field stays bit-identical this long (s)");
  quaternion_norm_tolerance_ = declare<double>(
    "quaternion_norm_tolerance", 0.05,
    "Samples whose quaternion norm is outside 1 +/- this are dropped");
  max_acceleration_ = declare<double>(
    "max_acceleration", 156.9,
    "Samples whose raw acceleration magnitude exceeds this (m/s^2) are dropped");
  calibration_grace_period_ = declare<double>(
    "calibration_grace_period", 60.0,
    "Seconds after the first sample before IMU_UNCALIBRATED can raise");
  const int recovery_max_attempts = declare<int>(
    "recovery_max_attempts", 3,
    "Recovery attempts before IMU_RECOVERY_EXHAUSTED and slow probing");
  const double recovery_backoff_initial = declare<double>(
    "recovery_backoff_initial", 1.0,
    "Delay (s) before the first recovery attempt; doubles after each");
  const double recovery_window = declare<double>(
    "recovery_window", 10.0,
    "Seconds data must stay healthy for a recovery to count as successful");
  const double reconnect_probe_period = declare<double>(
    "reconnect_probe_period", 30.0,
    "Seconds between recovery probes once the attempts are exhausted");
  const double log_reminder_period = declare<double>(
    "log_reminder_period", 30.0,
    "Seconds between log reminders of a condition that stays active; 0 disables them");
  reset_noise_estimators_locked();

  tracker_.reminder_period = log_reminder_period;
  policy_ = health::RecoveryPolicy(
    recovery_max_attempts, recovery_backoff_initial, recovery_window, reconnect_probe_period);
  connect_debounce_.raise_after = timeout_;
  device_debounce_.raise_after = timeout_;
  refresh_param_fallback_locked();

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
    rclcpp::ServicesQoS());
#else
    rmw_qos_profile_services_default);
#endif

  standby_service_ = this->create_service<std_srvs::srv::SetBool>(
    node_prefix + "standby",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      this->set_standby(request, response);
    },
#if RCLCPP_VERSION_MAJOR >= 21
    rclcpp::ServicesQoS());
#else
    rmw_qos_profile_services_default);
#endif

  // -- Publishers ----------------------------------------------------------------------------
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(node_prefix + "imu", 10);
  temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
    node_prefix + "temperature", 10);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
    node_prefix + "magnetic_field", 10);
  gravity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    node_prefix + "gravity", 10);

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

  RCLCPP_INFO(
    this->get_logger(), "Node is initialized (driver %s, %s:%d).", driver_name_.c_str(),
    host_.c_str(), port_);
}

ImuNode::~ImuNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down.");
  stop_driver();
}

void ImuNode::refresh_param_fallback_locked()
{
  std::string message;
  for (const auto & entry : param_fallbacks_) {
    message += (message.empty() ? "" : "; ") + entry.second;
  }
  tracker_.set(
    kConnection, health::PARAM_FALLBACK, !param_fallbacks_.empty(), health::WARN,
    "invalid parameters, defaults used: " + message);
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
  return config;
}

// -- Services / parameters --------------------------------------------------------------------

void ImuNode::report_ready(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  NodeState state;
  bool fresh = false;
  std::vector<health::Condition> active;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state = node_state_;
    fresh = data_fresh_;
    active = tracker_.active();
  }
  response->success = state == NodeState::RUNNING && fresh;
  response->message = std::string(this->get_name()) +
    (response->success ? " is ready" : " is not ready") + " (" + node_state_name(state) + ")";
  if (!response->success && !active.empty()) {
    response->message += ": " + active.front().code;
  }
}

void ImuNode::set_standby(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    standby_ = request->data;
  }
  response->success = true;
  response->message = request->data ?
    "standby enabled: data-topic publishing suspended" :
    "standby disabled: data-topic publishing resumed";
  RCLCPP_DEBUG(this->get_logger(), "%s", response->message.c_str());
}

bool ImuNode::standby() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return standby_;
}

rcl_interfaces::msg::SetParametersResult ImuNode::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Validate the whole batch first, so a rejected set changes nothing.
  for (const auto & parameter : parameters) {
    const auto reason = check_parameter(parameter);
    if (reason.has_value()) {
      result.successful = false;
      result.reason = *reason;
      return result;
    }
  }

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
      connect_debounce_.raise_after = timeout_;
      device_debounce_.raise_after = timeout_;
    } else if (name == "auto_reconnect") {
      auto_reconnect_ = parameter.as_bool();
      driver_config_changed = true;
    } else if (name == "acceleration_source") {
      acceleration_source_ = parameter.as_string();
      driver_config_changed = true;
    } else if (name == "orientation_stddev") {
      // 0 means "unknown": orientation_covariance then reports the -1 sentinel rather than
      // an all-zero matrix.
      orientation_stddev_ = parameter.as_double();
    } else if (name == "temperature_stddev") {
      temperature_stddev_ = parameter.as_double();
    } else if (name == "angular_velocity_stddev") {
      angular_velocity_stddev_ = to_array3(parameter.as_double_array());
    } else if (name == "linear_acceleration_stddev") {
      linear_acceleration_stddev_ = to_array3(parameter.as_double_array());
    } else if (name == "magnetic_field_stddev") {
      magnetic_field_stddev_ = to_array3(parameter.as_double_array());
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
    } else if (name == "rate_warn_ratio") {
      rate_warn_ratio_ = parameter.as_double();
    } else if (name == "frozen_timeout") {
      frozen_timeout_ = parameter.as_double();
    } else if (name == "quaternion_norm_tolerance") {
      quaternion_norm_tolerance_ = parameter.as_double();
    } else if (name == "max_acceleration") {
      max_acceleration_ = parameter.as_double();
    } else if (name == "calibration_grace_period") {
      calibration_grace_period_ = parameter.as_double();
    } else if (name == "recovery_max_attempts") {
      policy_.max_attempts = static_cast<int>(parameter.as_int());
    } else if (name == "recovery_backoff_initial") {
      policy_.backoff_initial = parameter.as_double();
    } else if (name == "recovery_window") {
      policy_.window = parameter.as_double();
    } else if (name == "reconnect_probe_period") {
      policy_.probe_period = parameter.as_double();
    } else if (name == "log_reminder_period") {
      tracker_.reminder_period = parameter.as_double();
    }
    // A valid value now replaces whatever default it fell back to at startup.
    param_fallbacks_.erase(name);
  }
  refresh_param_fallback_locked();

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
  const double now = now_seconds();
  const int fusion_mode = sample.fusion_mode.value_or(0);

  // Snapshot the parameters this publish pass needs while holding the lock. They are written
  // by update_parameters on an executor thread, so reading them straight off the members from
  // this driver thread would be a data race - and a torn std::string read is not theoretical.
  PublishParams params;
  bool standby = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!first_sample_.has_value()) {
      first_sample_ = now;
    }
    last_sample_ = now;
    sample_times_.push_back(now);
    if (sample_times_.size() > kRateWindow) {
      sample_times_.pop_front();
    }
    if (sample.calibration.has_value()) {
      last_calibration_ = sample.calibration;
    }
    last_fusion_mode_ = fusion_mode;

    // An invalid sample still proves data is flowing, but it must reach neither the noise
    // estimate nor a filter downstream.
    const auto reason = invalid_reason_locked(sample);
    if (reason.has_value()) {
      last_invalid_ = now;
      invalid_reason_ = *reason;
      return;
    }

    const Reading reading{
      sample.orientation, sample.angular_velocity, sample.linear_acceleration,
      sample.raw_acceleration, sample.magnetic_field};
    if (!last_reading_.has_value() || !(*last_reading_ == reading)) {
      last_reading_ = reading;
      last_change_ = now;
    }

    update_noise_estimators_locked(sample);
    params = publish_params_locked();
    standby = standby_;
  }

  // Standby suppresses only publishing: the bookkeeping above still runs, so the watchdog,
  // diagnostics and the noise estimate stay current and resuming is seamless.
  if (standby) {
    return;
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

  if (sample.gravity.has_value()) {
    publish_gravity(stamp, *sample.gravity, params);
  }
}

std::optional<std::string> ImuNode::invalid_reason_locked(const ImuSample & sample) const
{
  for (const auto * field : {&sample.angular_velocity, &sample.linear_acceleration,
      &sample.raw_acceleration, &sample.magnetic_field, &sample.gravity, &sample.euler})
  {
    if (field->has_value() && !health::all_finite(**field)) {
      return std::string("non-finite value");
    }
  }
  if (sample.temperature.has_value() && !std::isfinite(*sample.temperature)) {
    return std::string("non-finite temperature");
  }
  if (sample.orientation.has_value()) {
    if (!health::all_finite(*sample.orientation)) {
      return std::string("non-finite orientation");
    }
    // With fusion off there is no orientation to check: it is published as "unknown".
    if (sample.fusion_mode.value_or(0) != 0 &&
      !health::quaternion_norm_ok(*sample.orientation, quaternion_norm_tolerance_))
    {
      const auto & q = *sample.orientation;
      return format(
        "quaternion norm %.3f outside 1 +/- %.3f",
        std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]),
        quaternion_norm_tolerance_);
    }
  }
  const auto & acceleration =
    sample.raw_acceleration.has_value() ? sample.raw_acceleration : sample.linear_acceleration;
  if (acceleration.has_value()) {
    const double magnitude = health::magnitude(*acceleration);
    if (magnitude > max_acceleration_) {
      return format(
        "acceleration %.1f m/s^2 above max_acceleration %.1f", magnitude, max_acceleration_);
    }
  }
  return std::nullopt;
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
    [this]() {this->watchdog();});
}

size_t ImuNode::watchdog_tick_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return watchdog_tick_count_;
}

void ImuNode::watchdog()
{
  try {
    // Read before taking mutex_: the node never holds its own lock while taking the driver's.
    const DriverState state = driver_->state();
    const double now = now_seconds();

    std::optional<RecoveryAction> action;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_watchdog_period_ = now - last_watchdog_;
      last_watchdog_ = now;
      ++watchdog_tick_count_;
      action = evaluate_health_locked(state, now);
    }
    // Non-blocking: the driver runs the mechanics on its own worker thread.
    if (action.has_value()) {
      driver_->recover(*action);
    }

    // Diagnostics are NOT forced from here: the Updater runs its own 1 Hz timer. Forcing an
    // update every watchdog tick would republish /diagnostics at control_freq.
  } catch (const std::exception & exc) {
    // CRITICAL: log and continue. Never shut down or cancel the timer from here - a single
    // transient exception must not leave the node permanently inert.
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Exception in watchdog: %s", exc.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Unknown exception in watchdog");
  }
}

double ImuNode::rate_locked(double now) const
{
  // Only recent samples count, so the rate falls to zero when data stops and a past gap does
  // not drag it down once data is back. At low imu_freq the span stretches to hold a few.
  const double span = std::max(kRateSpan, 3.0 / imu_freq_);
  size_t count = 0;
  std::optional<double> first;
  for (const double t : sample_times_) {
    if (now - t <= span) {
      ++count;
      if (!first.has_value()) {
        first = t;
      }
    }
  }
  if (count < 2 || sample_times_.back() <= *first) {
    return 0.0;
  }
  return static_cast<double>(count - 1) / (sample_times_.back() - *first);
}

std::optional<RecoveryAction> ImuNode::evaluate_health_locked(
  const DriverState & state, double now)
{
  using health::ERROR;
  using health::WARN;

  // -- Connection ---------------------------------------------------------------------------
  // Debounced by `timeout`, so the first moments of a normal bringup raise nothing.
  const std::string driver_error = state.last_error.empty() ? state.detail : state.last_error;
  tracker_.set(
    kConnection, health::CONNECT_FAILED, connect_debounce_.update(!state.connected, now), ERROR,
    "not connected to " + host_ + ":" + std::to_string(port_) + ": " + driver_error);
  tracker_.set(
    kConnection, health::DEVICE_MISSING,
    device_debounce_.update(state.connected && !state.device_present, now), ERROR,
    "connected, but no IMU found");
  tracker_.set(
    kConnection, health::CONFIG_FAILED, state.config_failed, WARN,
    "applying the configuration failed: " + state.last_error);

  // -- Data flow ----------------------------------------------------------------------------
  // Before the first sample, age counts from startup.
  const double age = now - last_sample_.value_or(start_time_);
  const bool no_data = age > timeout_;
  data_fresh_ = last_sample_.has_value() && !no_data;
  tracker_.set(kDataFlow, health::NO_DATA, no_data, ERROR, format("no sample for %.1f s", age));

  const double rate = rate_locked(now);
  const double rate_floor = rate_warn_ratio_ * imu_freq_;
  tracker_.set(
    kDataFlow, health::RATE_LOW, rate_debounce_.update(!no_data && rate < rate_floor, now), WARN,
    format("data rate %.1f Hz below %.1f Hz", rate, rate_floor));

  if (no_data) {
    // Data resuming after a gap must not count the gap as time spent frozen.
    last_reading_.reset();
  }
  const bool frozen =
    last_reading_.has_value() && now - last_change_ >= frozen_timeout_;
  tracker_.set(
    kDataFlow, health::DATA_FROZEN, frozen, ERROR,
    format("every field unchanged for %.1f s", now - last_change_));

  tracker_.set(
    kDataFlow, health::DATA_INVALID,
    last_invalid_.has_value() && now - *last_invalid_ < kInvalidHold, WARN,
    "dropping invalid samples: " + invalid_reason_);

  // -- Calibration --------------------------------------------------------------------------
  // The BNO-055 starts at 0 and calibrates as it moves, so nothing raises during the grace
  // period. Which levels count depends on the fusion mode: see health::calibration_level.
  const bool settled =
    first_sample_.has_value() && now - *first_sample_ >= calibration_grace_period_;
  const bool uncalibrated = settled && last_calibration_.has_value() &&
    health::calibration_level(*last_calibration_, last_fusion_mode_) == 0;
  tracker_.set(
    kCalibration, health::UNCALIBRATED, uncalibrated, WARN,
    health::fusion_uses_magnetometer(last_fusion_mode_) ?
    "a calibration level the fusion mode uses is still 0" :
    "gyro or acc calibration still 0");

  // -- Recovery -----------------------------------------------------------------------------
  // Only while connected: before that, the driver's connect loop and the library's
  // auto-reconnect own the transport.
  const bool problem = state.connected && (no_data || frozen);
  const bool attempt = policy_.update(problem, now);
  tracker_.set(
    kConnection, health::RECOVERING, policy_.recovering && !policy_.exhausted, WARN,
    format("recovering, %d of %d attempts made", policy_.attempts, policy_.max_attempts));
  tracker_.set(
    kConnection, health::RECOVERY_EXHAUSTED, problem && policy_.exhausted, ERROR,
    format(
      "recovery failed after %d attempts; probing every %.0f s", policy_.attempts,
      policy_.probe_period));

  if (!state.connected) {
    node_state_ = NodeState::DISCONNECTED;
  } else if (!state.device_present) {
    node_state_ = NodeState::NO_DEVICE;
  } else if (policy_.recovering) {
    node_state_ = NodeState::RECOVERING;
  } else {
    node_state_ = NodeState::RUNNING;
  }

  if (!attempt) {
    return std::nullopt;
  }
  // Step 1 once, then step 2 for every later attempt and every slow probe.
  return (policy_.attempts > 1 || policy_.exhausted) ?
         RecoveryAction::Reconnect : RecoveryAction::Rearm;
}

// -- Diagnostics tasks ------------------------------------------------------------------------

void ImuNode::summarize_locked(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & scope,
  const std::string & ok_message) const
{
  const auto active = tracker_.active(scope);
  if (active.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, ok_message);
  } else {
    stat.summary(active.front().level, "[" + active.front().code + "] " + active.front().msg);
  }
  stat.add("active_codes", tracker_.codes(scope));
}

void ImuNode::diagnose_connection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const DriverState state = driver_->state();
  // The device is usually found after construction, so keep the hardware id current.
  updater_->setHardwareID(state.hardware_id);

  std::lock_guard<std::mutex> lock(mutex_);
  summarize_locked(stat, kConnection, state.detail);
  stat.add("state", node_state_name(node_state_));
  stat.add("recovery_attempts", std::to_string(policy_.attempts));
  stat.add("last_error", state.last_error);
  stat.add("detail", state.detail);
}

void ImuNode::diagnose_data_flow(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const double now = now_seconds();

  std::lock_guard<std::mutex> lock(mutex_);
  summarize_locked(stat, kDataFlow, "Receiving data");
  stat.add("rate_hz", two_decimals(rate_locked(now)));
  stat.add(
    "age_s", last_sample_.has_value() ? two_decimals(now - *last_sample_) : std::string("n/a"));
  // The watchdog's own measured tick period - a leading indicator of executor stalls, since
  // it moves at control_freq while the rest of this task only resolves at the 1 Hz Updater rate.
  stat.add("loop_period_s", two_decimals(last_watchdog_period_));
  // Samples still flow from the driver in standby; the data topics are just held silent.
  stat.add("standby", standby_ ? "true" : "false");
}

void ImuNode::diagnose_calibration(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(mutex_);
  conversions::Calibration values{0, 0, 0, 0};
  if (!last_calibration_.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No calibration data yet");
    stat.add("active_codes", "");
  } else {
    values = *last_calibration_;
    summarize_locked(stat, kCalibration, "Calibrated");
  }

  // All four are always reported, even those the current fusion mode ignores.
  stat.add("sys", std::to_string(values[0]));
  stat.add("gyro", std::to_string(values[1]));
  stat.add("acc", std::to_string(values[2]));
  stat.add("mag", std::to_string(values[3]));
  stat.add("fusion_mode", std::to_string(last_fusion_mode_));
}

// -- Logging ----------------------------------------------------------------------------------

void ImuNode::log_condition(health::Level level, const std::string & message)
{
  if (level == health::ERROR) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  } else if (level == health::WARN) {
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  }
}

}  // namespace idmind_imu
