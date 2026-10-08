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
/// runs under a SingleThreadedExecutor on a background thread; a separate subscriber node
/// receives its topics. Every wait has a hard deadline, so a regression shows up as a fast,
/// clear failure instead of a hang.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "fake_brickd.hpp"
#include "idmind_imu/imu_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using idmind_imu::ImuNode;
using idmind_imu::testing::AllData;
using idmind_imu::testing::FakeBrickd;
using idmind_imu::testing::Silence;

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
      // Publish /diagnostics at 5 Hz so raise/clear checks do not wait a second each.
      rclcpp::Parameter("diagnostic_updater.period", 0.2),
    };
    // An override replaces a default above rather than duplicating it.
    for (const auto & override_parameter : overrides) {
      parameters.erase(
        std::remove_if(
          parameters.begin(), parameters.end(),
          [&](const rclcpp::Parameter & p) {return p.get_name() == override_parameter.get_name();}),
        parameters.end());
      parameters.push_back(override_parameter);
    }

    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);

    node = std::make_shared<ImuNode>(options);
    subscriber = std::make_shared<rclcpp::Node>("test_imu_node_subscriber");

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
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
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  std::thread spin_thread;
};

/// Keeps the latest status of every diagnostic task published on /diagnostics.
///
/// The received state lives in a shared_ptr the callback owns a copy of, so a callback the
/// executor is still running when the watcher goes out of scope cannot touch freed memory.
class DiagnosticsWatcher
{
public:
  explicit DiagnosticsWatcher(const std::shared_ptr<rclcpp::Node> & node)
  : state_(std::make_shared<State>())
  {
    subscription_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      [state = state_](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state->mutex);
        for (const auto & status : msg->status) {
          state->latest[status.name] = status;
        }
      });
  }

  /// The latest status of \p task ("Connection", "Data flow" or "Calibration").
  std::optional<DiagnosticStatus> status(const std::string & task) const
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    for (const auto & entry : state_->latest) {
      if (entry.first.size() >= task.size() &&
        entry.first.compare(entry.first.size() - task.size(), task.size(), task) == 0)
      {
        return entry.second;
      }
    }
    return std::nullopt;
  }

  /// The value of \p key in the latest status of \p task, or "" if absent.
  std::string value(const std::string & task, const std::string & key) const
  {
    const auto latest = status(task);
    if (latest.has_value()) {
      for (const auto & kv : latest->values) {
        if (kv.key == key) {
          return kv.value;
        }
      }
    }
    return "";
  }

  /// Whether \p code is listed in the latest ``active_codes`` of \p task.
  bool has_code(const std::string & task, const std::string & code) const
  {
    const std::string codes = "," + value(task, "active_codes") + ",";
    return codes.find("," + code + ",") != std::string::npos;
  }

  /// The latest level of \p task, or 255 if it was never seen.
  int level(const std::string & task) const
  {
    const auto latest = status(task);
    return latest.has_value() ? latest->level : 255;
  }

private:
  struct State
  {
    std::mutex mutex;
    std::map<std::string, DiagnosticStatus> latest;
  };

  std::shared_ptr<State> state_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
};

/// Streams all-data frames from the fake on a background thread, like a real brick does.
///
/// By default every frame differs slightly (accelerometer noise), so the frozen-data check
/// stays quiet; ``set_frame(frame, false)`` repeats one frame exactly.
class Streamer
{
public:
  explicit Streamer(FakeBrickd & fake, AllData frame = AllData{})
  : fake_(fake), frame_(frame)
  {
    thread_ = std::thread(
      [this] {
        int tick = 0;
        while (!stop_.load()) {
          AllData data;
          std::chrono::milliseconds period;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            data = frame_;
            period = period_;
            if (vary_) {
              data.acceleration[0] = static_cast<int16_t>(data.acceleration[0] + tick % 3);
            }
          }
          ++tick;
          fake_.push_all_data(data);
          std::this_thread::sleep_for(period);
        }
      });
  }

  ~Streamer()
  {
    stop_.store(true);
    thread_.join();
  }

  void set_frame(const AllData & frame, bool vary = true)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_ = frame;
    vary_ = vary;
  }

  void set_period(std::chrono::milliseconds period)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    period_ = period;
  }

private:
  FakeBrickd & fake_;
  std::mutex mutex_;
  AllData frame_;
  bool vary_{true};
  std::chrono::milliseconds period_{10};
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

/// A healthy frame: identity orientation, gravity on z, every calibration level at 3.
AllData healthy_frame()
{
  AllData data;
  data.acceleration = {0, 0, 981};
  data.calibration_status = 0xFF;
  return data;
}

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

TEST_F(ImuNodeTest, DiagnosticsCalibrationPreservesFieldOrder)
{
  // Calibration has no dedicated topic - /diagnostics is the semantic replacement, reporting
  // sys/gyro/acc/mag as named key-values rather than an unlabeled array.
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
  data.calibration_status = 0b11100100;  // sys=3, gyro=2, acc=1, mag=0

  std::map<std::string, std::string> values;
  const bool seen = wait_until(
    [&] {
      fake_.push_all_data(data);
      for (const auto & msg : diagnostics->all()) {
        for (const auto & status : msg.status) {
          if (status.name.find("Calibration") != std::string::npos) {
            values.clear();
            for (const auto & kv : status.values) {
              values[kv.key] = kv.value;
            }
            return true;
          }
        }
      }
      return false;
    },
    kMessageDeadline);

  ASSERT_TRUE(seen) << "no calibration diagnostic seen";
  // Four distinct levels, so a wrong field order cannot pass this assertion.
  EXPECT_EQ(values["sys"], "3");
  EXPECT_EQ(values["gyro"], "2");
  EXPECT_EQ(values["acc"], "1");
  EXPECT_EQ(values["mag"], "0");
}

TEST_F(ImuNodeTest, PublishesTemperatureMagneticFieldAndGravity)
{
  Harness harness(fake_);
  auto temperatures = harness.subscribe<sensor_msgs::msg::Temperature>("temperature");
  auto fields = harness.subscribe<sensor_msgs::msg::MagneticField>("magnetic_field");
  auto gravities = harness.subscribe<geometry_msgs::msg::Vector3Stamped>("gravity");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData data;
  data.temperature = 25;
  data.magnetic_field = {16, 0, 0};
  data.gravity_vector = {0, 0, 981};

  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(data);
        return temperatures->size() > 0 && fields->size() > 0 && gravities->size() > 0;
      },
      kMessageDeadline)) << "not every derived topic was published";

  EXPECT_NEAR(temperatures->front().temperature, 25.0, 1e-6);
  EXPECT_EQ(temperatures->front().header.frame_id, "imu");
  EXPECT_NEAR(fields->front().magnetic_field.x, 1e-6, 1e-15);
  EXPECT_NEAR(gravities->front().vector.z, 9.81, 1e-6);
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

TEST_F(ImuNodeTest, WatchdogTicksRegularly)
{
  // There is no ~/timer topic; the watchdog's own tick count (exposed for tests, like
  // driver_state()) is the direct way to observe it running, independent of the diagnostics
  // Updater's separate 1 Hz timer.
  Harness harness(fake_, {rclcpp::Parameter("control_freq", 50.0)});

  ASSERT_TRUE(wait_until([&] {return harness.node->watchdog_tick_count() >= 3;}, kMessageDeadline))
    << "watchdog did not tick";
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

  ASSERT_TRUE(wait_until([&] {return harness.node->watchdog_tick_count() >= 5;}, kMessageDeadline))
    << "watchdog stopped ticking";
  const size_t after_first = harness.node->watchdog_tick_count();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_GT(harness.node->watchdog_tick_count(), after_first)
    << "watchdog stopped ticking after the timeout";
}

TEST_F(ImuNodeTest, StandbyServiceSuspendsAndResumesPublishing)
{
  Harness harness(fake_);
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  auto client =
    harness.subscriber->create_client<std_srvs::srv::SetBool>("/idmind_imu/standby");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10)))
    << "standby service never came up";

  auto call_standby = [&](bool enable) {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = enable;
      auto future = client->async_send_request(request);
      EXPECT_EQ(future.wait_for(std::chrono::seconds(10)), std::future_status::ready)
        << "standby service did not answer";
      EXPECT_TRUE(future.get()->success);
    };

  // Publishing works before standby.
  ASSERT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(AllData{});
        return imu_messages->size() > 0;
      },
      kMessageDeadline)) << "no Imu message before standby";

  call_standby(true);
  ASSERT_TRUE(wait_until([&] {return harness.node->standby();}, kMessageDeadline));

  // Let any sample already in flight drain, then require the count to stay put while more
  // samples keep arriving from the fake device.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  const size_t count_at_standby = imu_messages->size();
  for (int i = 0; i < 20; ++i) {
    fake_.push_all_data(AllData{});
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_EQ(imu_messages->size(), count_at_standby) << "node published while in standby";

  call_standby(false);
  EXPECT_TRUE(
    wait_until(
      [&] {
        fake_.push_all_data(AllData{});
        return imu_messages->size() > count_at_standby;
      },
      kMessageDeadline)) << "publishing did not resume after standby was disabled";
}

// -- Health: every condition code must raise and clear ----------------------------------------

namespace
{

constexpr auto kHealthDeadline = std::chrono::seconds(10);

/// Call the ready service and return the response, or nullptr if it did not answer.
std::shared_ptr<std_srvs::srv::Trigger::Response> call_ready(Harness & harness)
{
  auto client = harness.subscriber->create_client<std_srvs::srv::Trigger>("/idmind_imu/ready");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    return nullptr;
  }
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    return nullptr;
  }
  return future.get();
}

}  // namespace

TEST_F(ImuNodeTest, ReadyOnlyOnceDataIsFlowing)
{
  Harness harness(fake_, {rclcpp::Parameter("control_freq", 50.0)});
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  auto before = call_ready(harness);
  ASSERT_TRUE(before) << "ready service did not answer";
  EXPECT_FALSE(before->success) << "ready with no data: " << before->message;
  EXPECT_NE(before->message.find("not ready"), std::string::npos);

  Streamer streamer(fake_, healthy_frame());
  std::string message;
  EXPECT_TRUE(
    wait_until(
      [&] {
        auto response = call_ready(harness);
        message = response ? response->message : "";
        return response && response->success;
      },
      kHealthDeadline)) << "never ready while streaming: " << message;
  EXPECT_NE(message.find("is ready (RUNNING)"), std::string::npos) << message;
}

TEST_F(ImuNodeTest, NoDataRaisesAndClears)
{
  Harness harness(fake_, {rclcpp::Parameter("timeout", 0.3),
      rclcpp::Parameter("recovery_backoff_initial", 60.0)});
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  ASSERT_TRUE(
    wait_until([&] {return diagnostics.has_code("Data flow", "IMU_NO_DATA");}, kHealthDeadline))
    << "IMU_NO_DATA never raised";
  EXPECT_EQ(diagnostics.level("Data flow"), DiagnosticStatus::ERROR);

  Streamer streamer(fake_, healthy_frame());
  EXPECT_TRUE(
    wait_until(
      [&] {
        return !diagnostics.has_code("Data flow", "IMU_NO_DATA") &&
        diagnostics.level("Data flow") == DiagnosticStatus::OK;
      },
      kHealthDeadline)) << "IMU_NO_DATA never cleared";
}

TEST_F(ImuNodeTest, ConnectFailedRaisesAndClearsWhenThePortIsCorrected)
{
  // Nothing listens on port 1. The connect loop re-reads host and port on every try, so
  // correcting the parameter is enough to recover.
  Harness harness(fake_, {rclcpp::Parameter("port", 1), rclcpp::Parameter("timeout", 0.3)});
  DiagnosticsWatcher diagnostics(harness.subscriber);

  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Connection", "IMU_CONNECT_FAILED");}, kHealthDeadline))
    << "IMU_CONNECT_FAILED never raised";
  EXPECT_EQ(diagnostics.level("Connection"), DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostics.value("Connection", "state"), "DISCONNECTED");
  EXPECT_NE(diagnostics.value("Connection", "last_error").find(":1 "), std::string::npos)
    << diagnostics.value("Connection", "last_error");

  const auto results =
    harness.node->set_parameters({rclcpp::Parameter("port", static_cast<int>(fake_.port()))});
  ASSERT_TRUE(results[0].successful);

  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Connection", "IMU_CONNECT_FAILED");}, kHealthDeadline))
    << "IMU_CONNECT_FAILED never cleared";
  EXPECT_TRUE(harness.wait_for_device());
}

TEST_F(ImuNodeTest, DeviceMissingRaisesAndClearsThroughReEnumeration)
{
  fake_.set_answer_enumerate(false);
  Harness harness(fake_, {rclcpp::Parameter("timeout", 0.3),
      rclcpp::Parameter("recovery_backoff_initial", 0.2),
      rclcpp::Parameter("recovery_max_attempts", 100)});
  DiagnosticsWatcher diagnostics(harness.subscriber);

  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Connection", "IMU_DEVICE_MISSING");}, kHealthDeadline))
    << "IMU_DEVICE_MISSING never raised";
  EXPECT_EQ(diagnostics.value("Connection", "state"), "NO_DEVICE");

  // Recovery step 1 re-enumerates, which is what finds a late device.
  fake_.set_answer_enumerate(true);
  EXPECT_TRUE(harness.wait_for_device()) << "device never found after it started answering";
  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Connection", "IMU_DEVICE_MISSING");}, kHealthDeadline))
    << "IMU_DEVICE_MISSING never cleared";
}

TEST_F(ImuNodeTest, RateLowRaisesAndClears)
{
  // 5 Hz against imu_freq 20 Hz: well under the 0.8 ratio. Debounced 3 s each way.
  Harness harness(fake_, {rclcpp::Parameter("timeout", 1.0)});
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  Streamer streamer(fake_, healthy_frame());
  streamer.set_period(std::chrono::milliseconds(200));
  ASSERT_TRUE(
    wait_until([&] {return diagnostics.has_code("Data flow", "IMU_RATE_LOW");}, kHealthDeadline))
    << "IMU_RATE_LOW never raised";
  EXPECT_EQ(diagnostics.level("Data flow"), DiagnosticStatus::WARN);

  streamer.set_period(std::chrono::milliseconds(10));
  EXPECT_TRUE(
    wait_until([&] {return !diagnostics.has_code("Data flow", "IMU_RATE_LOW");}, kHealthDeadline))
    << "IMU_RATE_LOW never cleared";
}

TEST_F(ImuNodeTest, InvalidQuaternionIsDroppedFlaggedAndClears)
{
  Harness harness(fake_);
  DiagnosticsWatcher diagnostics(harness.subscriber);
  auto imu_messages = harness.subscribe<sensor_msgs::msg::Imu>("imu");
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  // An all-zero quaternion is what the BNO-055 sends before fusion has a solution.
  AllData invalid = healthy_frame();
  invalid.quaternion = {0, 0, 0, 0};
  Streamer streamer(fake_, invalid);

  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Data flow", "IMU_DATA_INVALID");}, kHealthDeadline))
    << "IMU_DATA_INVALID never raised";
  EXPECT_EQ(diagnostics.level("Data flow"), DiagnosticStatus::WARN);
  EXPECT_NE(diagnostics.status("Data flow")->message.find("quaternion"), std::string::npos);
  EXPECT_EQ(imu_messages->size(), 0u) << "an invalid sample was published";
  // Invalid data still proves the stream is alive.
  EXPECT_FALSE(diagnostics.has_code("Data flow", "IMU_NO_DATA"));

  streamer.set_frame(healthy_frame());
  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Data flow", "IMU_DATA_INVALID");}, kHealthDeadline))
    << "IMU_DATA_INVALID never cleared";
  EXPECT_GT(imu_messages->size(), 0u);
}

TEST_F(ImuNodeTest, SaturatedAccelerationIsInvalidAboveTheConfiguredLimit)
{
  Harness harness(fake_, {rclcpp::Parameter("max_acceleration", 50.0)});
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData saturated = healthy_frame();
  saturated.acceleration = {6000, 0, 0};  // 60 m/s^2
  Streamer streamer(fake_, saturated);

  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Data flow", "IMU_DATA_INVALID");}, kHealthDeadline))
    << "IMU_DATA_INVALID never raised";
  EXPECT_NE(
    diagnostics.status("Data flow")->message.find("max_acceleration"), std::string::npos);

  streamer.set_frame(healthy_frame());
  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Data flow", "IMU_DATA_INVALID");}, kHealthDeadline))
    << "IMU_DATA_INVALID never cleared";
}

TEST_F(ImuNodeTest, FrozenDataRaisesAndClears)
{
  Harness harness(fake_, {rclcpp::Parameter("frozen_timeout", 0.5),
      rclcpp::Parameter("recovery_backoff_initial", 60.0)});
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  Streamer streamer(fake_, healthy_frame());
  streamer.set_frame(healthy_frame(), false);  // bit-identical, frame after frame

  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Data flow", "IMU_DATA_FROZEN");}, kHealthDeadline))
    << "IMU_DATA_FROZEN never raised";
  EXPECT_EQ(diagnostics.level("Data flow"), DiagnosticStatus::ERROR);

  streamer.set_frame(healthy_frame(), true);
  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Data flow", "IMU_DATA_FROZEN");}, kHealthDeadline))
    << "IMU_DATA_FROZEN never cleared";
}

TEST_F(ImuNodeTest, ConfigFailedRaisesAndClears)
{
  // The fake ignores get_all_data_period, so the first convergence times out in the bindings.
  fake_.set_answer_get_all_data_period(false);
  Harness harness(fake_, {rclcpp::Parameter("recovery_backoff_initial", 60.0)});
  DiagnosticsWatcher diagnostics(harness.subscriber);

  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Connection", "IMU_CONFIG_FAILED");}, kHealthDeadline))
    << "IMU_CONFIG_FAILED never raised";
  EXPECT_NE(
    diagnostics.value("Connection", "last_error").find("get_all_data_period"), std::string::npos);

  fake_.set_answer_get_all_data_period(true);
  const auto results = harness.node->set_parameters({rclcpp::Parameter("imu_freq", 10.0)});
  ASSERT_TRUE(results[0].successful);
  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Connection", "IMU_CONFIG_FAILED");}, kHealthDeadline))
    << "IMU_CONFIG_FAILED never cleared";
}

TEST_F(ImuNodeTest, UncalibratedIgnoresSysAndMagInFusionModeTwo)
{
  // The shipped mode 2 never raises sys above 0. The old rule reported WARN forever.
  Harness harness(fake_, {rclcpp::Parameter("calibration_grace_period", 0.3)});
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  // Bits 0-1 mag, 2-3 acc, 4-5 gyro, 6-7 sys.
  AllData frame = healthy_frame();
  frame.calibration_status = 0b00111100;  // sys=0, gyro=3, acc=3, mag=0
  Streamer streamer(fake_, frame);

  ASSERT_TRUE(
    wait_until(
      [&] {
        return diagnostics.level("Calibration") == DiagnosticStatus::OK &&
        diagnostics.value("Calibration", "sys") == "0";
      },
      kHealthDeadline)) << "sys=0 in mode 2 must not degrade calibration";

  frame.calibration_status = 0b00110000;  // acc=0
  streamer.set_frame(frame);
  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Calibration", "IMU_UNCALIBRATED");}, kHealthDeadline))
    << "IMU_UNCALIBRATED never raised";
  EXPECT_EQ(diagnostics.level("Calibration"), DiagnosticStatus::WARN);

  frame.calibration_status = 0b00111100;
  streamer.set_frame(frame);
  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Calibration", "IMU_UNCALIBRATED");}, kHealthDeadline))
    << "IMU_UNCALIBRATED never cleared";
}

TEST_F(ImuNodeTest, NoUncalibratedWarningDuringTheGracePeriod)
{
  Harness harness(fake_);  // default 60 s grace
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";

  AllData frame = healthy_frame();
  frame.calibration_status = 0;  // everything still at 0, as at power-up
  Streamer streamer(fake_, frame);

  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.value("Calibration", "gyro") == "0";}, kHealthDeadline));
  EXPECT_EQ(diagnostics.level("Calibration"), DiagnosticStatus::OK);
  EXPECT_FALSE(diagnostics.has_code("Calibration", "IMU_UNCALIBRATED"));
}

TEST_F(ImuNodeTest, InvalidStartupParameterFallsBackToItsDefault)
{
  // The node used to throw and die on a bad control_freq. Now it runs on the default and
  // says so, until a valid value is set.
  Harness harness(fake_, {rclcpp::Parameter("control_freq", 0.0),
      rclcpp::Parameter("frozen_timeout", -1.0)});
  DiagnosticsWatcher diagnostics(harness.subscriber);

  EXPECT_DOUBLE_EQ(harness.node->get_parameter("control_freq").as_double(), 20.0);
  EXPECT_DOUBLE_EQ(harness.node->get_parameter("frozen_timeout").as_double(), 5.0);
  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Connection", "IMU_PARAM_FALLBACK");}, kHealthDeadline))
    << "IMU_PARAM_FALLBACK never raised";
  ASSERT_TRUE(
    wait_until([&] {return harness.node->watchdog_tick_count() >= 3;}, kHealthDeadline))
    << "watchdog not running on the fallback control_freq";

  const auto results = harness.node->set_parameters(
    {rclcpp::Parameter("control_freq", 10.0), rclcpp::Parameter("frozen_timeout", 4.0)});
  ASSERT_TRUE(results[0].successful);
  ASSERT_TRUE(results[1].successful);
  EXPECT_TRUE(
    wait_until(
      [&] {return !diagnostics.has_code("Connection", "IMU_PARAM_FALLBACK");}, kHealthDeadline))
    << "IMU_PARAM_FALLBACK never cleared";
}

TEST_F(ImuNodeTest, InvalidRuntimeParametersAreRejectedAsAWhole)
{
  Harness harness(fake_);
  const auto result = harness.node->set_parameters_atomically(
    {rclcpp::Parameter("frozen_timeout", 3.0), rclcpp::Parameter("rate_warn_ratio", 1.5)});
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("rate_warn_ratio"), std::string::npos) << result.reason;
  EXPECT_DOUBLE_EQ(harness.node->get_parameter("frozen_timeout").as_double(), 5.0);
}

// -- Recovery ladder ---------------------------------------------------------------------------

namespace
{

/// Fast recovery timings, so a whole ladder fits in a few seconds.
std::vector<rclcpp::Parameter> fast_recovery(int max_attempts)
{
  return {
    rclcpp::Parameter("control_freq", 50.0),
    rclcpp::Parameter("timeout", 0.3),
    rclcpp::Parameter("recovery_backoff_initial", 0.2),
    rclcpp::Parameter("recovery_max_attempts", max_attempts),
    rclcpp::Parameter("recovery_window", 1.0),
    rclcpp::Parameter("reconnect_probe_period", 0.5),
  };
}

}  // namespace

TEST_F(ImuNodeTest, RecoveryStopsAtRearmWhenThatHeals)
{
  Harness harness(fake_, fast_recovery(3));
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";
  Streamer streamer(fake_, healthy_frame());
  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.level("Data flow") == DiagnosticStatus::OK;}, kHealthDeadline));

  fake_.set_silence(Silence::UntilRearm);
  ASSERT_TRUE(
    wait_until([&] {return diagnostics.has_code("Connection", "IMU_RECOVERING");}, kHealthDeadline))
    << "IMU_RECOVERING never raised";

  EXPECT_TRUE(
    wait_until(
      [&] {
        return !diagnostics.has_code("Data flow", "IMU_NO_DATA") &&
        !diagnostics.has_code("Connection", "IMU_RECOVERING");
      },
      kHealthDeadline)) << "rearm did not heal the stream";
  EXPECT_EQ(fake_.connection_count(), 1u) << "a rearm was enough; reconnecting was not";
  EXPECT_EQ(diagnostics.value("Connection", "state"), "RUNNING");
}

TEST_F(ImuNodeTest, RecoveryEscalatesToReconnectWhenRearmDoesNotHeal)
{
  Harness harness(fake_, fast_recovery(3));
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";
  Streamer streamer(fake_, healthy_frame());
  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.level("Data flow") == DiagnosticStatus::OK;}, kHealthDeadline));

  fake_.set_silence(Silence::UntilReconnect);
  ASSERT_TRUE(wait_until([&] {return fake_.connection_count() >= 2;}, kHealthDeadline))
    << "never escalated to a reconnect";
  EXPECT_TRUE(
    wait_until(
      [&] {
        return diagnostics.level("Data flow") == DiagnosticStatus::OK &&
        diagnostics.level("Connection") == DiagnosticStatus::OK;
      },
      kHealthDeadline)) << "reconnect did not heal the stream";
}

TEST_F(ImuNodeTest, RecoveryExhaustsThenKeepsProbingAndRecovers)
{
  Harness harness(fake_, fast_recovery(2));
  DiagnosticsWatcher diagnostics(harness.subscriber);
  ASSERT_TRUE(harness.wait_for_device()) << "device never found";
  Streamer streamer(fake_, healthy_frame());
  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.level("Data flow") == DiagnosticStatus::OK;}, kHealthDeadline));
  const size_t period_sets =
    fake_.request_count(idmind_imu::testing::kFunctionSetAllDataPeriod);

  fake_.set_silence(Silence::Forever);

  // Step 1, re-arm: the period is re-sent on the same connection.
  ASSERT_TRUE(
    wait_until(
      [&] {
        return fake_.request_count(idmind_imu::testing::kFunctionSetAllDataPeriod) > period_sets;
      },
      kHealthDeadline)) << "step 1 (rearm) never ran";
  // Step 2, reconnect.
  ASSERT_TRUE(wait_until([&] {return fake_.connection_count() >= 2;}, kHealthDeadline))
    << "step 2 (reconnect) never ran";
  // Budget spent: ERROR, but no latch.
  ASSERT_TRUE(
    wait_until(
      [&] {return diagnostics.has_code("Connection", "IMU_RECOVERY_EXHAUSTED");}, kHealthDeadline))
    << "IMU_RECOVERY_EXHAUSTED never raised";
  EXPECT_EQ(diagnostics.level("Connection"), DiagnosticStatus::ERROR);
  EXPECT_FALSE(diagnostics.has_code("Connection", "IMU_RECOVERING"));
  // Slow probes keep reconnecting.
  const size_t connections = fake_.connection_count();
  ASSERT_TRUE(
    wait_until([&] {return fake_.connection_count() > connections;}, kHealthDeadline))
    << "no probe after exhaustion";

  fake_.set_silence(Silence::None);
  EXPECT_TRUE(
    wait_until(
      [&] {
        return diagnostics.level("Data flow") == DiagnosticStatus::OK &&
        diagnostics.level("Connection") == DiagnosticStatus::OK;
      },
      kHealthDeadline)) << "never recovered once the fake healed";
  auto ready = call_ready(harness);
  ASSERT_TRUE(ready);
  EXPECT_TRUE(ready->success) << ready->message;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
