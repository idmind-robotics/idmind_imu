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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "idmind_imu/drivers/driver.hpp"
#include "idmind_imu/noise_estimator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
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
/// Number of recent sample timestamps kept to estimate the observed data rate.
constexpr size_t kRateWindow = 50;

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

  /// Reply with whether the node has finished initializing.
  void report_ready(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

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

  /// Log \p message, suppressing an identical message repeated within one second.
  void log(const std::string & message, const std::string & alert = "info");

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

  // -- Shared state, guarded by mutex_ (touched by a driver thread and an executor thread) --

  mutable std::mutex mutex_;
  std::optional<std::chrono::steady_clock::time_point> last_sample_;
  std::optional<conversions::Calibration> last_calibration_;
  std::deque<std::chrono::steady_clock::time_point> sample_times_;

  /// Sliding-window noise estimators, all touched only under mutex_.
  RollingVariance angular_velocity_noise_{100};
  RollingVariance linear_acceleration_noise_{100};
  RollingVariance magnetic_field_noise_{100};
  RollingVariance orientation_noise_{100, AngleWrap::Radians};
  std::chrono::steady_clock::time_point last_watchdog_{std::chrono::steady_clock::now()};
  /// The watchdog's last measured tick period, surfaced as the "loop_period_s" diagnostic key.
  double last_watchdog_period_{0.0};
  /// Ticks observed so far; read by watchdog_tick_count() for tests.
  size_t watchdog_tick_count_{0};

  bool ready_{false};

  /// Guards the duplicate-suppression state of ``log``, which is called from several threads
  /// and must never depend on mutex_ (a caller may already hold it).
  mutable std::mutex log_mutex_;
  std::string last_message_;
  std::chrono::steady_clock::time_point last_message_time_{std::chrono::steady_clock::now()};

  // -- ROS entities --------------------------------------------------------------------------

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
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
