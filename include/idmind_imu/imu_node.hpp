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
/// Hardware-agnostic ROS 2 node for streaming IMU data.
///
/// This node contains no TinkerForge-specific code: it talks only to the ``ImuDriver``
/// interface obtained through ``idmind_imu::make_driver``, so changing the ``driver``
/// parameter is enough to point it at a different IMU backend. Unit conversion and covariance
/// construction are delegated to ``idmind_imu/conversions.hpp``.

#ifndef IDMIND_IMU__IMU_NODE_HPP_
#define IDMIND_IMU__IMU_NODE_HPP_

#include <array>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "idmind_imu/drivers/driver.hpp"
#include "idmind_imu/health.hpp"
#include "idmind_imu/noise_estimator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace idmind_imu
{

/// Default angular-velocity stddev per axis, rad/s. 0.3 deg/s, matching the pre-parameter
/// hardcoded value, so a config that does not set the parameter behaves as before.
extern const std::vector<double> kDefaultAngularVelocityStddev;
/// Default linear-acceleration stddev per axis, m/s^2: sqrt of the old 0.01/0.01/0.05.
extern const std::vector<double> kDefaultLinearAccelerationStddev;
/// Default magnetic-field stddev per axis, tesla: the old 0.6 uT.
extern const std::vector<double> kDefaultMagneticFieldStddev;
/// Most recent sample timestamps kept to estimate the observed data rate.
constexpr size_t kRateWindow = 50;
/// Only samples this recent (seconds) count towards the rate, so a past gap ages out.
constexpr double kRateSpan = 2.0;
/// How long (seconds) the rate must stay low before IMU_RATE_LOW raises, and stay back at
/// rate before it clears.
constexpr double kRateDebounce = 3.0;
/// How long (seconds) IMU_DATA_INVALID stays raised after the last invalid sample.
constexpr double kInvalidHold = 1.0;

/// The node's coarse state, reported as the ``state`` key of the Connection diagnostic.
enum class NodeState
{
  /// Transport (e.g. BrickDaemon) not connected.
  DISCONNECTED,
  /// Connected, but no IMU found on it.
  NO_DEVICE,
  /// Device present and no recovery in progress.
  RUNNING,
  /// Connected but silent or frozen; the recovery ladder is running.
  RECOVERING,
};

/// "DISCONNECTED", "NO_DEVICE", "RUNNING" or "RECOVERING".
const char * node_state_name(NodeState state);

/// ROS 2 node that streams samples from an ``ImuDriver`` onto a fixed set of topics.
///
/// All hardware access goes through the driver named by the ``driver`` parameter; this class
/// only converts ``ImuSample`` instances into ROS messages and manages parameters, services,
/// and diagnostics.
class ImuNode : public rclcpp::Node
{
public:
  /// Declare parameters, create publishers/service/diagnostics, and start the driver.
  explicit ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Stop the driver, so no callback can fire into a half-destroyed node.
  ~ImuNode() override;

  /// Stop the underlying driver. Idempotent; safe to call during shutdown.
  void stop_driver();

  /// Return a snapshot of the driver's connection and device status, for tests.
  DriverState driver_state() const;

  /// Number of watchdog ticks observed so far, for tests: proves the watchdog is still
  /// running without depending on a topic or the independently-timed diagnostics Updater.
  size_t watchdog_tick_count() const;

  /// Whether the node is currently in standby (data-topic publishing suppressed), for tests.
  bool standby() const;

private:
  /// The node-side parameters one publish pass needs, snapshotted together under the lock.
  ///
  /// These are written by ``update_parameters`` on an executor thread and read by
  /// ``on_sample`` on a driver thread, so they are copied out once per sample rather than
  /// read off the members while publishing.
  struct PublishParams
  {
    std::string imu_frame;
    double orientation_stddev{0.01};
    double temperature_stddev{0.0};
    std::array<double, 3> angular_velocity_stddev{};
    std::array<double, 3> linear_acceleration_stddev{};
    std::array<double, 3> magnetic_field_stddev{};

    /// Variance measured from the live signal. Disengaged until a window has filled, in which
    /// case the configured stddev scaled by calibration is published instead.
    std::optional<std::array<double, 3>> measured_angular_velocity;
    std::optional<std::array<double, 3>> measured_linear_acceleration;
    std::optional<std::array<double, 3>> measured_magnetic_field;
    std::optional<std::array<double, 3>> measured_orientation;
  };

  /// The value of every field the frozen-data check compares. A frame counts as frozen only
  /// when all of them repeat: a still robot can legitimately repeat any single one.
  struct Reading
  {
    std::optional<std::array<double, 4>> orientation;
    std::optional<std::array<double, 3>> angular_velocity;
    std::optional<std::array<double, 3>> linear_acceleration;
    std::optional<std::array<double, 3>> raw_acceleration;
    std::optional<std::array<double, 3>> magnetic_field;

    bool operator==(const Reading & other) const;
  };

  /// Declare one parameter. An invalid startup value falls back to \p default_value, is
  /// written back so ``ros2 param get`` tells the truth, and raises IMU_PARAM_FALLBACK.
  template<typename T>
  T declare(const std::string & name, const T & default_value, const std::string & description);

  /// Why \p sample must be dropped, or nothing if it is valid. Caller must hold ``mutex_``.
  std::optional<std::string> invalid_reason_locked(const ImuSample & sample) const;

  /// Raise and clear every condition from the current state, and run the recovery policy.
  /// Returns the recovery action to take, if any. Caller must hold ``mutex_``.
  std::optional<RecoveryAction> evaluate_health_locked(const DriverState & state, double now);

  /// The first active condition of \p scope as (level, message), or (OK, \p ok_message).
  void summarize_locked(
    diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & scope,
    const std::string & ok_message) const;

  /// Raise or clear IMU_PARAM_FALLBACK from ``param_fallbacks_``. Caller must hold ``mutex_``.
  void refresh_param_fallback_locked();

  /// Observed sample rate over the recent samples. Caller must hold ``mutex_``.
  double rate_locked(double now) const;

  /// Seconds on the steady clock, the time base of all health bookkeeping.
  static double now_seconds();

  /// Copy the current publish parameters and noise estimates. Caller must hold ``mutex_``.
  PublishParams publish_params_locked() const;

  /// Feed \p sample into the noise estimators. Caller must hold ``mutex_``.
  ///
  /// When ``noise_estimate_when_still`` is set, samples taken while the robot is moving are
  /// skipped: the variance of a moving IMU is dominated by real motion, not by sensor noise,
  /// and publishing that would tell a filter to distrust the IMU exactly when it matters.
  void update_noise_estimators_locked(const ImuSample & sample);

  /// Whether \p sample looks like it was taken at rest. Caller must hold ``mutex_``.
  bool is_stationary_locked(const ImuSample & sample) const;

  /// Rebuild the estimators after a window-size change. Caller must hold ``mutex_``.
  void reset_noise_estimators_locked();

  /// Build the hardware-relevant config subset from the current parameter values.
  DriverConfig driver_config() const;

  /// Reply with whether the IMU is RUNNING with fresh data; the message carries the state
  /// and whatever is blocking readiness.
  void report_ready(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Enter or leave standby. In standby the node keeps receiving samples from the driver - so
  /// diagnostics, the watchdog and the noise estimate stay live - but publishes nothing on its
  /// data topics. ``request->data`` true enters standby, false resumes publishing.
  void set_standby(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /// Validate and apply changed parameters, forwarding the hardware-relevant subset.
  rcl_interfaces::msg::SetParametersResult update_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  /// Convert one sample into ROS messages and publish. Runs on a driver thread.
  void on_sample(const ImuSample & sample);

  void publish_imu(
    const rclcpp::Time & stamp, const ImuSample & sample, int fusion_mode,
    const PublishParams & params);
  void publish_temperature(
    const rclcpp::Time & stamp, double temperature, const PublishParams & params);
  void publish_magnetic_field(
    const rclcpp::Time & stamp, const std::array<double, 3> & magnetic_field,
    const std::optional<conversions::Calibration> & calibration,
    const PublishParams & params);
  void publish_gravity(
    const rclcpp::Time & stamp, const std::array<double, 3> & gravity,
    const PublishParams & params);

  /// Record a tick and check data staleness. Never performs hardware I/O.
  void watchdog();

  /// Recreate the watchdog timer at \p control_freq (rclcpp has no timer period setter).
  void restart_watchdog_timer(double control_freq);

  void diagnose_connection(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_data_flow(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_calibration(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /// Sink for ConditionTracker: WARN and ERROR at their level, clears (OK) at INFO.
  void log_condition(health::Level level, const std::string & message);

  // -- Parameters ---------------------------------------------------------------------------

  std::string driver_name_;
  std::string host_;
  int port_{4223};
  double control_freq_{20.0};
  double imu_freq_{20.0};
  std::string imu_frame_;
  bool imu_leds_{false};
  int imu_fusion_mode_{2};
  double timeout_{1.0};
  bool auto_reconnect_{true};
  std::string acceleration_source_;
  double orientation_stddev_{0.01};
  double temperature_stddev_{0.0};
  std::array<double, 3> angular_velocity_stddev_{};
  std::array<double, 3> linear_acceleration_stddev_{};
  std::array<double, 3> magnetic_field_stddev_{};
  int noise_window_{100};
  bool noise_estimate_when_still_{true};
  double stationary_gyro_threshold_{0.02};
  double stationary_accel_threshold_{0.2};
  double rate_warn_ratio_{0.8};
  double frozen_timeout_{5.0};
  double quaternion_norm_tolerance_{0.05};
  double max_acceleration_{156.9};
  double calibration_grace_period_{60.0};

  // -- Shared state, guarded by mutex_ (touched by a driver thread and an executor thread) --

  mutable std::mutex mutex_;
  /// Steady-clock seconds at construction: data age is measured from here until a sample.
  double start_time_{0.0};
  std::optional<double> first_sample_;
  std::optional<double> last_sample_;
  std::optional<conversions::Calibration> last_calibration_;
  int last_fusion_mode_{0};
  std::deque<double> sample_times_;
  /// Frozen-data check: the last valid reading and when it last changed.
  std::optional<Reading> last_reading_;
  double last_change_{0.0};
  /// Invalid-data check: when the last invalid sample arrived, and why it was dropped.
  std::optional<double> last_invalid_;
  std::string invalid_reason_;

  /// Health. All of it is touched only under mutex_.
  health::ConditionTracker tracker_;
  health::RecoveryPolicy policy_;
  health::Debounce connect_debounce_{1.0, 0.0};
  health::Debounce device_debounce_{1.0, 0.0};
  health::Debounce rate_debounce_{kRateDebounce, kRateDebounce};
  NodeState node_state_{NodeState::DISCONNECTED};
  /// Parameters whose startup value was invalid, with the reason; cleared by a valid set.
  std::map<std::string, std::string> param_fallbacks_;

  /// Sliding-window noise estimators, all touched only under mutex_.
  RollingVariance angular_velocity_noise_{100};
  RollingVariance linear_acceleration_noise_{100};
  RollingVariance magnetic_field_noise_{100};
  RollingVariance orientation_noise_{100, AngleWrap::Radians};
  double last_watchdog_{0.0};
  /// The watchdog's last measured tick period, surfaced as the "loop_period_s" diagnostic key.
  double last_watchdog_period_{0.0};
  /// Ticks observed so far; read by watchdog_tick_count() for tests.
  size_t watchdog_tick_count_{0};

  /// Whether the last sample is younger than ``timeout``, as of the last watchdog tick.
  bool data_fresh_{false};

  /// When true, ``on_sample`` still runs all its bookkeeping but publishes nothing. Written by
  /// the ``standby`` service on an executor thread, read by ``on_sample`` on a driver thread.
  bool standby_{false};

  // -- ROS entities --------------------------------------------------------------------------

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr standby_service_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::unique_ptr<diagnostic_updater::Updater> updater_;
  std::unique_ptr<ImuDriver> driver_;
};

}  // namespace idmind_imu

#endif  // IDMIND_IMU__IMU_NODE_HPP_
