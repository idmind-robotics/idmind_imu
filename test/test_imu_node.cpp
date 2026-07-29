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
/// Integration tests of idmind_imu::ImuNode against FakeBrickd.
///
/// These exercise the real node (brick_v2 driver, real ROS publishers and service) with no
/// hardware and no brickd: the fake stands in for BrickDaemon on an ephemeral port. The node
/// runs under a MultiThreadedExecutor on a background thread; a separate subscriber node
/// receives its topics. Every wait has a hard deadline, so a regression shows up as a fast,
/// clear failure instead of a hang.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "fake_brickd.hpp"
#include "idmind_imu/imu_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

using idmind_imu::ImuNode;
using idmind_imu::testing::AllData;
using idmind_imu::testing::FakeBrickd;

namespace
{

/// Deadline for the node to connect and discover the fake device.
constexpr auto kStartDeadline = std::chrono::seconds(10);
/// Deadline for a pushed sample to reach a subscriber.
constexpr auto kMessageDeadline = std::chrono::seconds(10);

/// Poll \p predicate until it holds or \p timeout elapses; returns whether it held.
template<typename Predicate, typename Duration>
bool wait_until(Predicate predicate, Duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return predicate();
}

/// Collects every message received on one topic, thread-safely.
template<typename MsgT>
class Collector
{
public:
  /// Store one received message.
  void add(const typename MsgT::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    messages_.push_back(*msg);
  }

  /// How many messages have arrived so far.
  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_.size();
  }

  /// A copy of every message received so far.
  std::vector<MsgT> all() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }

  /// The first message received; only valid once ``size() > 0``.
  MsgT front() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_.front();
  }

private:
  mutable std::mutex mutex_;
  std::vector<MsgT> messages_;
};

/// Owns an ImuNode, a subscriber node, and the executor spinning both.
class Harness
{
public:
  /// Build the node against \p fake, applying \p overrides on top of the transport settings.
  explicit Harness(
    const FakeBrickd & fake, const std::vector<rclcpp::Parameter> & overrides = {})
  {
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter("host", "127.0.0.1"),
      rclcpp::Parameter("port", static_cast<int>(fake.port())),
    };
    parameters.insert(parameters.end(), overrides.begin(), overrides.end());

    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);

    node = std::make_shared<ImuNode>(options);
    subscriber = std::make_shared<rclcpp::Node>("test_imu_node_subscriber");

    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->add_node(subscriber);
    spin_thread = std::thread([this] {executor->spin();});
  }

  /// Tear down the driver, executor, and both nodes, in that order.
  ~Harness()
  {
    node->stop_driver();
    executor->cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
    executor->remove_node(subscriber);
    executor->remove_node(node);
    subscriber.reset();
    node.reset();
  }

  /// Subscribe to \p topic under the node's namespace, collecting every message.
  template<typename MsgT>
  std::shared_ptr<Collector<MsgT>> subscribe(const std::string & topic)
  {
    auto collector = std::make_shared<Collector<MsgT>>();
    auto subscription = subscriber->create_subscription<MsgT>(
      "/idmind_imu/" + topic, 10,
      [collector](const typename MsgT::SharedPtr msg) {collector->add(msg);});
    subscriptions.push_back(subscription);
    return collector;
  }

  /// Block until the driver reports the fake device present.
  bool wait_for_device()
  {
    return wait_until([this] {return node->driver_state().device_present;}, kStartDeadline);
  }

  std::shared_ptr<ImuNode> node;
  std::shared_ptr<rclcpp::Node> subscriber;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::thread spin_thread;
};

/// Starts a fake brick daemon for each test and initialises rclpy-equivalent context once.
class ImuNodeTest : public ::testing::Test
{
protected:
  void SetUp() override {fake_.start();}
  void TearDown() override {fake_.stop();}

  FakeBrickd fake_;
};

}  // namespace

TEST_F(ImuNodeTest, ImuMessageHasCorrectUnitsFrameAndZeroOffDiagonals)
{
  Harness harness(fake_);
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.angular_velocity = {16, 0, 0};
  data.quaternion = {16383, 0, 0, 0};
  data.linear_acceleration = {100, 0, 0};
  data.calibration_status = 0b11100100;  // sys=3, gyro=2, acc=1, mag=0

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        return imu_messages->size() > 0;
      },
      kMessageDeadline)) << "no Imu message received";

  const auto msg = imu_messages->front();
  EXPECT_EQ(msg.header.frame_id, "imu");
  EXPECT_NEAR(msg.angular_velocity.x, 0.0174533, 1e-6);
  EXPECT_NEAR(msg.linear_acceleration.x, 1.0, 1e-9);
  EXPECT_NEAR(msg.orientation.w, 1.0, 1e-9);

  for (int i : {1, 2, 3, 5, 6, 7}) {
    EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[i], 0.0) << "off-diagonal " << i;
    EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[i], 0.0) << "off-diagonal " << i;
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[i], 0.0) << "off-diagonal " << i;
  }
}

TEST_F(ImuNodeTest, CovarianceIsMeasuredFromTheLiveSignal)
{
  // The whole point of the noise estimator: with a varying signal the published covariance
  // must differ between messages. A calibration-scaled matrix is a 4-value step function and
  // would be byte-identical in every message, which is what "fixed matrix" meant.
  Harness harness(fake_, {rclcpp::Parameter("noise_window", 8),
      rclcpp::Parameter("noise_estimate_when_still", false)});
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  // Feed a noisy gyro signal, one distinct value per push.
  int tick = 0;
  const bool got_enough = wait_until(
    [&] {
      AllData data;
      const int16_t jitter = static_cast<int16_t>((tick % 7) - 3);
      data.angular_velocity = {static_cast<int16_t>(16 + jitter), jitter, 0};
      data.linear_acceleration = {static_cast<int16_t>(100 + jitter), 0, 0};
      data.calibration_status = 0xFF;  // pinned, so calibration cannot explain any change
      ++tick;
      fake_.push_all_data(data);
      return imu_messages->size() >= 40;
    },
    kMessageDeadline);
  ASSERT_TRUE(got_enough) << "not enough Imu messages";

  const auto all = imu_messages->all();
  bool covariance_changed = false;
  double first_seen = -1.0;
  for (const auto & msg : all) {
    const double value = msg.angular_velocity_covariance[0];
    if (value <= 0.0) {
      continue;  // still on the fallback before the first window filled
    }
    if (first_seen < 0.0) {
      first_seen = value;
    } else if (std::fabs(value - first_seen) > 1e-12) {
      covariance_changed = true;
      break;
    }
  }
  EXPECT_TRUE(covariance_changed)
    << "angular_velocity_covariance never changed across " << all.size()
    << " messages - it is still a fixed matrix";
}

TEST_F(ImuNodeTest, MeasuredCovarianceNeverCollapsesToZero)
{
  // A perfectly constant signal has zero measured variance, which must NOT be published as an
  // all-zero matrix - that reads as "perfectly certain". It has to fall back to the sentinel.
  Harness harness(fake_, {rclcpp::Parameter("noise_window", 4),
      rclcpp::Parameter("noise_estimate_when_still", false)});
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData constant;
  constant.angular_velocity = {16, 0, 0};
  constant.calibration_status = 0xFF;
  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(constant);
        return imu_messages->size() >= 20;
      },
      kMessageDeadline)) << "not enough Imu messages";

  for (const auto & msg : imu_messages->all()) {
    const auto & cov = msg.angular_velocity_covariance;
    const bool all_zero = std::all_of(cov.begin(), cov.end(), [](double v) {return v == 0.0;});
    EXPECT_FALSE(all_zero) << "published an all-zero covariance meaning 'perfectly certain'";
    // A quiet signal falls back to the configured stddev rather than the unknown sentinel.
    EXPECT_GT(cov[0], 0.0);
  }
}

TEST_F(ImuNodeTest, NoiseWindowBelowTwoFallsBackToConfiguredStddev)
{
  Harness harness(fake_, {rclcpp::Parameter("noise_window", 0)});
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.calibration_status = 0xFF;  // fully calibrated -> factor 1, so the bare stddev squared
  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        return imu_messages->size() >= 5;
      },
      kMessageDeadline)) << "no Imu messages";

  EXPECT_NEAR(
    imu_messages->all().back().angular_velocity_covariance[0], 0.005236 * 0.005236, 1e-9);
}

TEST_F(ImuNodeTest, CovarianceTracksCalibrationRatherThanBeingFixed)
{
  // The published covariance must actually respond to the device's calibration levels. Push
  // an uncalibrated frame then a fully calibrated one and require the matrix to shrink; a
  // hardcoded matrix would report identical values for both. noise_window 0 pins this to the
  // calibration path so the live estimator cannot mask what is being tested.
  Harness harness(fake_, {rclcpp::Parameter("noise_window", 0)});
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData uncalibrated;
  uncalibrated.calibration_status = 0x00;  // every level 0
  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(uncalibrated);
        return imu_messages->size() > 0;
      },
      kMessageDeadline)) << "no Imu message received";
  const auto cold = imu_messages->front();

  auto warm_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  AllData calibrated;
  calibrated.calibration_status = 0xFF;  // every level 3
  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(calibrated);
        const auto all = warm_messages->all();
        if (all.empty()) {
          return false;
        }
        const double warm_variance = all.back().angular_velocity_covariance[0];
        return warm_variance < cold.angular_velocity_covariance[0];
      },
      kMessageDeadline)) << "covariance did not change with calibration - is it hardcoded?";

  const auto warm = warm_messages->all().back();
  EXPECT_LT(warm.angular_velocity_covariance[0], cold.angular_velocity_covariance[0]);
  EXPECT_LT(warm.linear_acceleration_covariance[0], cold.linear_acceleration_covariance[0]);
  EXPECT_LT(warm.orientation_covariance[0], cold.orientation_covariance[0]);
  // Fully calibrated means factor 1, so the base stddev comes through unscaled.
  EXPECT_NEAR(warm.angular_velocity_covariance[0], 0.005236 * 0.005236, 1e-9);
  EXPECT_NEAR(cold.angular_velocity_covariance[0], 0.005236 * 0.005236 * 100.0, 1e-7);
}

TEST_F(ImuNodeTest, MagneticFieldCovarianceTracksMagCalibration)
{
  Harness harness(fake_, {rclcpp::Parameter("noise_window", 0)});
  auto fields = harness.subscribe<sensor_msgs::msg::MagneticField>("magnetic_field");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.calibration_status = 0b11111100;  // sys/gyro/acc = 3, mag = 0
  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        return fields->size() > 0;
      },
      kMessageDeadline)) << "no MagneticField message received";

  // mag level 0 -> factor 100 on the 0.6 uT default.
  EXPECT_NEAR(fields->front().magnetic_field_covariance[0], 0.6e-6 * 0.6e-6 * 100.0, 1e-19);
}

TEST_F(ImuNodeTest, StddevParameterMustHaveThreeElements)
{
  Harness harness(fake_);

  const auto results = harness.node->set_parameters(
    {rclcpp::Parameter("angular_velocity_stddev", std::vector<double>{1.0, 2.0})});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].successful);
  EXPECT_NE(results[0].reason.find("3 elements"), std::string::npos);
}

TEST_F(ImuNodeTest, CalibrationTopicPreservesFieldOrder)
{
  Harness harness(fake_);
  auto calibrations = harness.subscribe<std_msgs::msg::UInt8MultiArray>("calibration");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.calibration_status = 0b11100100;  // sys=3, gyro=2, acc=1, mag=0

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        return calibrations->size() > 0;
      },
      kMessageDeadline)) << "no calibration message received";

  // Four distinct levels, so a wrong field order cannot pass this assertion.
  const auto msg = calibrations->front();
  ASSERT_EQ(msg.data.size(), 4u);
  EXPECT_EQ(msg.data[0], 3);
  EXPECT_EQ(msg.data[1], 2);
  EXPECT_EQ(msg.data[2], 1);
  EXPECT_EQ(msg.data[3], 0);
}

TEST_F(ImuNodeTest, PublishesTemperatureMagneticFieldGravityAndEuler)
{
  Harness harness(fake_);
  auto temperatures = harness.subscribe<sensor_msgs::msg::Temperature>("temperature");
  auto fields = harness.subscribe<sensor_msgs::msg::MagneticField>("magnetic_field");
  auto gravities = harness.subscribe<geometry_msgs::msg::Vector3Stamped>("gravity");
  auto eulers = harness.subscribe<std_msgs::msg::Float32>("euler");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.temperature = 25;
  data.magnetic_field = {16, 0, 0};
  data.gravity_vector = {0, 0, 981};
  data.euler_angle = {16 * 90, 0, 0};  // heading = 90 degrees

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        const bool scalars_ok = temperatures->size() > 0 && eulers->size() > 0;
        const bool vectors_ok = fields->size() > 0 && gravities->size() > 0;
        return scalars_ok && vectors_ok;
      },
      kMessageDeadline)) << "not every derived topic was published";

  EXPECT_NEAR(temperatures->front().temperature, 25.0, 1e-6);
  EXPECT_EQ(temperatures->front().header.frame_id, "imu");
  EXPECT_NEAR(fields->front().magnetic_field.x, 1e-6, 1e-15);
  EXPECT_NEAR(gravities->front().vector.z, 9.81, 1e-6);
  // The euler topic carries yaw in radians, not degrees.
  EXPECT_NEAR(eulers->front().data, M_PI / 2.0, 1e-4);
}

TEST_F(ImuNodeTest, TemperatureVarianceIsStddevSquared)
{
  Harness harness(fake_, {rclcpp::Parameter("temperature_stddev", 2.0)});
  auto temperatures = harness.subscribe<sensor_msgs::msg::Temperature>("temperature");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.temperature = 25;

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        return temperatures->size() > 0;
      },
      kMessageDeadline)) << "no Temperature message received";

  EXPECT_DOUBLE_EQ(temperatures->front().variance, 4.0);
}

TEST_F(ImuNodeTest, TemperatureVarianceIsZeroWhenStddevIsZero)
{
  // 0.0 is the sensor_msgs/Temperature convention for "variance unknown", so a stddev of 0
  // must pass straight through rather than being replaced by a default.
  Harness harness(fake_, {rclcpp::Parameter("temperature_stddev", 0.0)});
  auto temperatures = harness.subscribe<sensor_msgs::msg::Temperature>("temperature");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.temperature = 25;

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        return temperatures->size() > 0;
      },
      kMessageDeadline)) << "no Temperature message received";

  EXPECT_DOUBLE_EQ(temperatures->front().variance, 0.0);
}

TEST_F(ImuNodeTest, NegativeTemperatureStddevIsRejected)
{
  Harness harness(fake_);

  const auto results =
    harness.node->set_parameters({rclcpp::Parameter("temperature_stddev", -1.0)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].successful);
  EXPECT_NE(results[0].reason.find("temperature_stddev"), std::string::npos);
}

TEST_F(ImuNodeTest, DiagnosticsReportsCalibrationWarning)
{
  Harness harness(fake_);
  auto diagnostics = std::make_shared<Collector<diagnostic_msgs::msg::DiagnosticArray>>();
  auto subscription = harness.subscriber->create_subscription<
    diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [diagnostics](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      diagnostics->add(msg);
    });
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.calibration_status = 0b11100100;  // mag=0 -> partially uncalibrated

  const bool seen = wait_until(
    [&] {
      fake_.push_all_data(data);
      for (const auto & msg : diagnostics->all()) {
        for (const auto & status : msg.status) {
          if (status.name.find("Calibration") != std::string::npos &&
          status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
          {
            return true;
          }
        }
      }
      return false;
    },
    kMessageDeadline);

  EXPECT_TRUE(seen) << "no calibration WARN diagnostic seen";
}

TEST_F(ImuNodeTest, DiagnosticsReportsConnectionOkOnceTheDeviceIsPresent)
{
  Harness harness(fake_);
  auto diagnostics = std::make_shared<Collector<diagnostic_msgs::msg::DiagnosticArray>>();
  auto subscription = harness.subscriber->create_subscription<
    diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10,
    [diagnostics](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      diagnostics->add(msg);
    });
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  const bool seen = wait_until(
    [&] {
      for (const auto & msg : diagnostics->all()) {
        for (const auto & status : msg.status) {
          if (status.name.find("Connection") != std::string::npos &&
          status.level == diagnostic_msgs::msg::DiagnosticStatus::OK)
          {
            return true;
          }
        }
      }
      return false;
    },
    kMessageDeadline);

  EXPECT_TRUE(seen) << "connection never reported OK";
}

TEST_F(ImuNodeTest, OrientationCovarianceSentinelWhenFusionModeOff)
{
  Harness harness(fake_, {rclcpp::Parameter("imu_fusion_mode", 0)});
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(AllData{});
        return imu_messages->size() > 0;
      },
      kMessageDeadline)) << "no Imu message received";

  EXPECT_DOUBLE_EQ(imu_messages->front().orientation_covariance[0], -1.0);
}

TEST_F(ImuNodeTest, ImuFrameParameterIsHonoured)
{
  Harness harness(fake_, {rclcpp::Parameter("imu_frame", std::string("imu2"))});
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(AllData{});
        return imu_messages->size() > 0;
      },
      kMessageDeadline)) << "no Imu message received";

  EXPECT_EQ(imu_messages->front().header.frame_id, "imu2");
}

TEST_F(ImuNodeTest, HeartbeatIsPublishedOnTheTimerTopic)
{
  Harness harness(fake_, {rclcpp::Parameter("control_freq", 50.0)});
  auto heartbeats = harness.subscribe<std_msgs::msg::Float32>("timer");

  ASSERT_TRUE(wait_until([&] {return heartbeats->size() >= 3;}, kMessageDeadline))
    << "watchdog heartbeat was not published";
  EXPECT_GT(heartbeats->front().data, 0.0f);
}

TEST_F(ImuNodeTest, ReadyServiceReportsReady)
{
  Harness harness(fake_);

  auto client = harness.subscriber->create_client<std_srvs::srv::Trigger>("/idmind_imu/ready");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10))) << "ready service never came up";

  auto future = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    future.wait_for(std::chrono::seconds(10)), std::future_status::ready)
    << "ready service did not answer";

  const auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_NE(response->message.find("is ready"), std::string::npos);
}

TEST_F(ImuNodeTest, ControlFreqParameterIsRejectedWhenNotPositive)
{
  Harness harness(fake_);

  const auto results = harness.node->set_parameters({rclcpp::Parameter("control_freq", 0.0)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].successful);
  EXPECT_NE(results[0].reason.find("control_freq"), std::string::npos);
}

TEST_F(ImuNodeTest, ChangingImuFreqReachesTheDevice)
{
  // The node must forward hardware-relevant parameter changes to the driver, which must in
  // turn converge the device - without ever polling it on a schedule.
  Harness harness(fake_);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";
  ASSERT_TRUE(wait_until([&] {return fake_.all_data_period() == 50;}, kStartDeadline))
    << "initial imu_freq of 20Hz never reached the device";

  const auto results = harness.node->set_parameters({rclcpp::Parameter("imu_freq", 10.0)});
  ASSERT_EQ(results.size(), 1u);
  ASSERT_TRUE(results[0].successful);

  EXPECT_TRUE(wait_until([&] {return fake_.all_data_period() == 100;}, kStartDeadline))
    << "changed imu_freq never reached the device";
}

TEST_F(ImuNodeTest, WatchdogSurvivesAndKeepsRunningWithoutData)
{
  // The watchdog must never shut itself down: the pre-refactor node cancelled its own timer
  // on the first exception, leaving the process alive but permanently inert. With no samples
  // arriving it logs a warning every tick and must keep ticking regardless.
  Harness harness(fake_, {rclcpp::Parameter("control_freq", 50.0),
      rclcpp::Parameter("timeout", 0.05)});
  auto heartbeats = harness.subscribe<std_msgs::msg::Float32>("timer");

  ASSERT_TRUE(wait_until([&] {return heartbeats->size() >= 5;}, kMessageDeadline))
    << "watchdog stopped ticking";
  const size_t after_first = heartbeats->size();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_GT(heartbeats->size(), after_first) << "watchdog stopped ticking after the timeout";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
