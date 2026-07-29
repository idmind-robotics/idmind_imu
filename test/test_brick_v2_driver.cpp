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
/// Tests of the real ImuBrickV2Driver against FakeBrickd.
///
/// Unlike test_fake_brickd.cpp (which proves the fake is convincing to the raw bindings),
/// these exercise the driver itself: connection lifecycle, unit conversion on the all-data
/// path, the acceleration_source switch, that configuration converges once rather than being
/// polled, and clean shutdown. Every wait has a hard deadline, so a regression shows up as a
/// fast, clear failure instead of a hang.

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "fake_brickd.hpp"
#include "idmind_imu/drivers/brick_v2.hpp"
#include "rclcpp/rclcpp.hpp"

using idmind_imu::DriverConfig;
using idmind_imu::ImuBrickV2Driver;
using idmind_imu::ImuSample;
using idmind_imu::testing::AllData;
using idmind_imu::testing::FakeBrickd;

namespace
{

/// Deadline for the driver to connect and discover the fake device.
constexpr auto kStartDeadline = std::chrono::seconds(5);
/// Deadline for a pushed sample to arrive at the sample callback.
constexpr auto kSampleDeadline = std::chrono::seconds(5);
/// Deadline for config convergence and thread teardown to complete.
constexpr auto kSettleDeadline = std::chrono::seconds(5);
/// Window over which config-related requests must NOT keep growing (the polling regression).
constexpr auto kSteadyStateWindow = std::chrono::milliseconds(1500);

/// Poll \p predicate until it holds or \p timeout elapses; returns whether it held.
template<typename Predicate, typename Duration>
bool wait_until(Predicate predicate, Duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

/// A thread-safe queue the driver's sample callback pushes into.
class SampleQueue
{
public:
  /// Store one sample and wake any waiter.
  void push(const ImuSample & sample)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      samples_.push_back(sample);
    }
    cv_.notify_all();
  }

  /// Pop the oldest sample, waiting up to \p timeout; returns nullopt on timeout.
  std::optional<ImuSample> pop(std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this] {return !samples_.empty();})) {
      return std::nullopt;
    }
    ImuSample sample = samples_.front();
    samples_.pop_front();
    return sample;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<ImuSample> samples_;
};

/// Owns a fake brick daemon for one test, started and stopped around it.
class BrickV2DriverTest : public ::testing::Test
{
protected:
  void SetUp() override {fake_.start();}
  void TearDown() override {fake_.stop();}

  /// Build a driver pointed at the fake, with \p config's non-transport fields applied.
  std::unique_ptr<ImuBrickV2Driver> make_driver(DriverConfig config = DriverConfig{})
  {
    config.host = "127.0.0.1";
    config.port = fake_.port();
    return std::make_unique<ImuBrickV2Driver>(
      config, [this](const ImuSample & sample) {samples_.push(sample);},
      rclcpp::get_logger("test_brick_v2_driver"));
  }

  FakeBrickd fake_;
  SampleQueue samples_;
};

}  // namespace

TEST_F(BrickV2DriverTest, StartReturnsPromptlyAndReachesConnected)
{
  auto driver = make_driver();

  const auto started_at = std::chrono::steady_clock::now();
  driver->start();
  const auto elapsed = std::chrono::steady_clock::now() - started_at;
  EXPECT_LT(elapsed, std::chrono::seconds(1)) << "start() must not block on the connection";

  ASSERT_TRUE(
    wait_until(
      [&] {return driver->state().connected && driver->state().device_present;},
      kStartDeadline)) << "driver did not reach connected+device_present in time";

  const auto state = driver->state();
  EXPECT_TRUE(state.connected);
  EXPECT_TRUE(state.device_present);
  EXPECT_NE(state.hardware_id.find(fake_.uid()), std::string::npos);

  driver->stop();
}

TEST_F(BrickV2DriverTest, AllDataCallbackConvertsToSiUnits)
{
  auto driver = make_driver();
  driver->start();
  ASSERT_TRUE(wait_until([&] {return driver->state().device_present;}, kStartDeadline))
    << "device never found";

  AllData data;
  data.magnetic_field = {16, 0, 0};
  data.angular_velocity = {16, 0, 0};
  data.quaternion = {16383, 0, 0, 0};
  data.linear_acceleration = {100, 0, 0};
  data.temperature = 25;
  data.calibration_status = 0b11100100;  // sys=3, gyro=2, acc=1, mag=0
  fake_.push_all_data(data);

  const auto sample = samples_.pop(kSampleDeadline);
  ASSERT_TRUE(sample.has_value()) << "no sample delivered";

  ASSERT_TRUE(sample->angular_velocity.has_value());
  EXPECT_NEAR((*sample->angular_velocity)[0], 0.0174533, 1e-6);
  ASSERT_TRUE(sample->linear_acceleration.has_value());
  EXPECT_NEAR((*sample->linear_acceleration)[0], 1.0, 1e-9);
  ASSERT_TRUE(sample->magnetic_field.has_value());
  EXPECT_NEAR((*sample->magnetic_field)[0], 1e-6, 1e-15);
  ASSERT_TRUE(sample->orientation.has_value());
  EXPECT_NEAR((*sample->orientation)[3], 1.0, 1e-9);  // ROS xyzw -> w
  ASSERT_TRUE(sample->temperature.has_value());
  EXPECT_NEAR(*sample->temperature, 25.0, 1e-9);
  // (sys, gyro, acc, mag): four distinct levels, so a wrong field order cannot pass.
  ASSERT_TRUE(sample->calibration.has_value());
  EXPECT_EQ(*sample->calibration, (idmind_imu::conversions::Calibration{3, 2, 1, 0}));

  driver->stop();
}

TEST_F(BrickV2DriverTest, AccelerationSourceRawSelectsTheRawField)
{
  DriverConfig config;
  config.acceleration_source = "raw";
  auto driver = make_driver(config);
  driver->start();
  ASSERT_TRUE(wait_until([&] {return driver->state().device_present;}, kStartDeadline))
    << "device never found";

  AllData data;
  data.acceleration = {500, 0, 0};
  data.linear_acceleration = {100, 0, 0};
  fake_.push_all_data(data);

  const auto sample = samples_.pop(kSampleDeadline);
  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->linear_acceleration.has_value());
  EXPECT_NEAR((*sample->linear_acceleration)[0], 5.0, 1e-9);

  driver->stop();
}

TEST_F(BrickV2DriverTest, AccelerationSourceDefaultSelectsTheLinearField)
{
  auto driver = make_driver();
  driver->start();
  ASSERT_TRUE(wait_until([&] {return driver->state().device_present;}, kStartDeadline))
    << "device never found";

  AllData data;
  data.acceleration = {500, 0, 0};
  data.linear_acceleration = {100, 0, 0};
  fake_.push_all_data(data);

  const auto sample = samples_.pop(kSampleDeadline);
  ASSERT_TRUE(sample.has_value());
  ASSERT_TRUE(sample->linear_acceleration.has_value());
  EXPECT_NEAR((*sample->linear_acceleration)[0], 1.0, 1e-9);

  driver->stop();
}

TEST_F(BrickV2DriverTest, ConfigConvergesOnTheDeviceState)
{
  DriverConfig config;
  config.imu_freq = 10.0;
  config.imu_leds = true;
  config.imu_fusion_mode = 1;
  auto driver = make_driver(config);
  driver->start();

  ASSERT_TRUE(
    wait_until(
      [&] {
        const bool fusion_ok = fake_.sensor_fusion_mode() == 1;
        const bool period_ok = fake_.all_data_period() == 100;
        const bool leds_ok = fake_.leds_on() && fake_.status_led_enabled();
        return fusion_ok && period_ok && leds_ok;
      },
      kSettleDeadline))
    << "config never converged: fusion=" << static_cast<int>(fake_.sensor_fusion_mode())
    << " period=" << fake_.all_data_period() << " leds=" << fake_.leds_on()
    << " status_led=" << fake_.status_led_enabled();

  driver->stop();
}

TEST_F(BrickV2DriverTest, ConfigIsAppliedOnceAndNotPolled)
{
  // Regression guard for the old implementation's 1 Hz config polling: once the driver has
  // settled, the count of requests recorded by the fake must stop growing. The disconnect
  // probe is the only thing the bindings send on an idle connection, and it fires every 5s,
  // well outside the window checked here.
  DriverConfig config;
  config.imu_freq = 10.0;
  config.imu_leds = true;
  config.imu_fusion_mode = 1;
  auto driver = make_driver(config);
  driver->start();

  ASSERT_TRUE(
    wait_until(
      [&] {return fake_.all_data_period() == 100 && fake_.sensor_fusion_mode() == 1;},
      kSettleDeadline)) << "config was never applied";
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // let trailing setters land

  const size_t first_count = fake_.request_count();
  std::this_thread::sleep_for(kSteadyStateWindow);
  const size_t second_count = fake_.request_count();

  EXPECT_EQ(second_count, first_count)
    << "requests kept growing after settling: " << first_count << " -> " << second_count;

  driver->stop();
}

TEST_F(BrickV2DriverTest, ApplyConfigReconvergesTheDevice)
{
  auto driver = make_driver();
  driver->start();
  ASSERT_TRUE(wait_until([&] {return driver->state().device_present;}, kStartDeadline))
    << "device never found";
  ASSERT_TRUE(wait_until([&] {return fake_.all_data_period() == 50;}, kSettleDeadline))
    << "initial config never applied";

  DriverConfig changed;
  changed.host = "127.0.0.1";
  changed.port = fake_.port();
  changed.imu_freq = 10.0;
  changed.imu_fusion_mode = 1;
  changed.imu_leds = true;
  driver->apply_config(changed);

  EXPECT_TRUE(
    wait_until(
      [&] {
        const bool period_ok = fake_.all_data_period() == 100;
        const bool fusion_ok = fake_.sensor_fusion_mode() == 1;
        return period_ok && fusion_ok && fake_.leds_on();
      },
      kSettleDeadline)) << "changed config never reached the device";

  driver->stop();
}

TEST_F(BrickV2DriverTest, StopIsCleanAndIdempotent)
{
  auto driver = make_driver();
  driver->start();
  ASSERT_TRUE(wait_until([&] {return driver->state().connected;}, kStartDeadline))
    << "driver never connected";

  driver->stop();
  driver->stop();  // idempotent: must not raise or hang

  const auto state = driver->state();
  EXPECT_FALSE(state.connected);
  EXPECT_FALSE(state.device_present);
  EXPECT_EQ(state.detail, "stopped");
}

TEST_F(BrickV2DriverTest, StopBeforeStartIsSafe)
{
  auto driver = make_driver();
  driver->stop();  // must not hang or crash before start()
  EXPECT_FALSE(driver->state().connected);
}

TEST(BrickV2DriverNoServer, StartWithoutAServerDoesNotBlockOrCrash)
{
  // Port 1 is reserved and nothing listens on it, so connect() fails immediately and the
  // driver must keep retrying in the background rather than blocking start() or dying.
  DriverConfig config;
  config.host = "127.0.0.1";
  config.port = 1;

  ImuBrickV2Driver driver(
    config, [](const ImuSample &) {}, rclcpp::get_logger("test_brick_v2_no_server"));

  const auto started_at = std::chrono::steady_clock::now();
  driver.start();
  EXPECT_LT(std::chrono::steady_clock::now() - started_at, std::chrono::seconds(1));

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(driver.state().connected);

  driver.stop();
}
