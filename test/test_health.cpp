// Copyright 2026 IDMind
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
/// Unit tests for the health primitives: pure logic, no ROS, no hardware.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "idmind_imu/health.hpp"

namespace health = idmind_imu::health;
using health::ConditionTracker;
using health::Debounce;
using health::RecoveryPolicy;

namespace
{

/// A tracker whose clock and log are plain test variables.
struct TrackerHarness
{
  double now{0.0};
  std::vector<std::pair<health::Level, std::string>> lines;
  ConditionTracker tracker{
    [this](health::Level level, const std::string & msg) {lines.emplace_back(level, msg);},
    [this]() {return now;},
    30.0};
};

}  // namespace

TEST(Health, LevelNames)
{
  EXPECT_STREQ(health::level_name(health::OK), "OK");
  EXPECT_STREQ(health::level_name(health::WARN), "WARN");
  EXPECT_STREQ(health::level_name(health::ERROR), "ERROR");
}

TEST(Tracker, LogsOnceOnRaiseAndOnceOnClear)
{
  TrackerHarness h;
  for (int i = 0; i < 100; ++i) {
    h.tracker.set("Data flow", health::NO_DATA, true, health::ERROR, "no data");
  }
  ASSERT_EQ(h.lines.size(), 1u);
  EXPECT_EQ(h.lines[0].first, health::ERROR);
  EXPECT_EQ(h.lines[0].second, "[IMU_NO_DATA] no data");

  h.now = 2.5;
  h.tracker.clear("Data flow", health::NO_DATA);
  h.tracker.clear("Data flow", health::NO_DATA);
  ASSERT_EQ(h.lines.size(), 2u);
  EXPECT_EQ(h.lines[1].first, health::OK);
  EXPECT_EQ(h.lines[1].second, "[IMU_NO_DATA] cleared after 2.5 s");
}

TEST(Tracker, RemindsWhileActive)
{
  TrackerHarness h;
  h.tracker.set("Data flow", health::NO_DATA, true, health::ERROR, "no data");
  h.now = 29.9;
  h.tracker.set("Data flow", health::NO_DATA, true, health::ERROR, "no data");
  EXPECT_EQ(h.lines.size(), 1u);
  h.now = 30.0;
  h.tracker.set("Data flow", health::NO_DATA, true, health::ERROR, "no data");
  ASSERT_EQ(h.lines.size(), 2u);
  EXPECT_EQ(h.lines[1].second, "[IMU_NO_DATA] no data (still active, 30 s)");
}

TEST(Tracker, LevelChangeIsATransition)
{
  TrackerHarness h;
  h.tracker.set("Data flow", health::RATE_LOW, true, health::WARN, "slow");
  h.tracker.set("Data flow", health::RATE_LOW, true, health::ERROR, "very slow");
  ASSERT_EQ(h.lines.size(), 2u);
  EXPECT_EQ(h.lines[1].first, health::ERROR);
}

TEST(Tracker, ScopesAreIndependentAndSortedBySeverity)
{
  TrackerHarness h;
  h.tracker.set("Data flow", health::RATE_LOW, true, health::WARN, "slow");
  h.now = 1.0;
  h.tracker.set("Data flow", health::NO_DATA, true, health::ERROR, "none");
  h.tracker.set("Calibration", health::UNCALIBRATED, true, health::WARN, "cal");

  const auto data = h.tracker.active("Data flow");
  ASSERT_EQ(data.size(), 2u);
  EXPECT_EQ(data[0].code, health::NO_DATA);
  EXPECT_EQ(data[1].code, health::RATE_LOW);
  EXPECT_EQ(h.tracker.level("Data flow"), health::ERROR);
  EXPECT_EQ(h.tracker.level("Calibration"), health::WARN);
  EXPECT_EQ(h.tracker.level("Connection"), health::OK);
  EXPECT_TRUE(h.tracker.is_active("Calibration", health::UNCALIBRATED));
  EXPECT_FALSE(h.tracker.is_active("Data flow", health::UNCALIBRATED));
  EXPECT_EQ(h.tracker.active().size(), 3u);
}

TEST(Tracker, CodesJoinsActiveCodes)
{
  TrackerHarness h;
  EXPECT_EQ(h.tracker.codes("Data flow"), "");
  h.tracker.set("Data flow", health::RATE_LOW, true, health::WARN, "slow");
  h.tracker.set("Data flow", health::NO_DATA, true, health::ERROR, "none");
  EXPECT_EQ(h.tracker.codes("Data flow"), "IMU_NO_DATA,IMU_RATE_LOW");
}

TEST(Recovery, WaitsOneBackoffThenAttemptsWithDoublingDelay)
{
  RecoveryPolicy policy(3, 1.0, 10.0, 30.0);
  EXPECT_FALSE(policy.update(true, 0.0));
  EXPECT_TRUE(policy.recovering);
  EXPECT_FALSE(policy.update(true, 0.9));
  EXPECT_TRUE(policy.update(true, 1.0));
  EXPECT_EQ(policy.attempts, 1);
  EXPECT_FALSE(policy.update(true, 2.9));
  EXPECT_TRUE(policy.update(true, 3.0));
  EXPECT_EQ(policy.attempts, 2);
  EXPECT_FALSE(policy.update(true, 6.9));
  EXPECT_TRUE(policy.update(true, 7.0));
  EXPECT_EQ(policy.attempts, 3);
  EXPECT_FALSE(policy.exhausted);
}

TEST(Recovery, ExhaustedKeepsProbingAndNeverLatches)
{
  // The IMU drives nothing, so there is nothing for a human to clear: once the budget is
  // spent the policy reports it and keeps probing at the slow period, forever.
  RecoveryPolicy policy(1, 1.0, 10.0, 30.0);
  policy.update(true, 0.0);
  EXPECT_TRUE(policy.update(true, 1.0));
  EXPECT_FALSE(policy.update(true, 2.9));
  EXPECT_FALSE(policy.update(true, 3.0));
  EXPECT_TRUE(policy.exhausted);
  EXPECT_FALSE(policy.update(true, 32.9));
  EXPECT_TRUE(policy.update(true, 33.0));
  EXPECT_FALSE(policy.update(true, 62.9));
  EXPECT_TRUE(policy.update(true, 63.0));
  EXPECT_TRUE(policy.exhausted);
  EXPECT_EQ(policy.attempts, 1);
}

TEST(Recovery, SuccessResetsOnlyAfterAHealthyWindow)
{
  RecoveryPolicy policy(3, 1.0, 10.0, 30.0);
  policy.update(true, 0.0);
  policy.update(true, 1.0);
  EXPECT_FALSE(policy.update(false, 2.0));
  EXPECT_FALSE(policy.recovering);
  EXPECT_EQ(policy.attempts, 1);
  policy.update(false, 10.9);
  EXPECT_EQ(policy.attempts, 1);
  policy.update(false, 11.0);
  EXPECT_EQ(policy.attempts, 0);
}

TEST(Recovery, ProblemReturningInsideTheWindowCountsAsAnotherFailure)
{
  RecoveryPolicy policy(3, 1.0, 10.0, 30.0);
  policy.update(true, 0.0);
  policy.update(true, 1.0);
  policy.update(false, 2.0);
  // Back within the window: the next attempt waits the second backoff step and is #2.
  EXPECT_FALSE(policy.update(true, 5.0));
  EXPECT_FALSE(policy.update(true, 6.9));
  EXPECT_TRUE(policy.update(true, 7.0));
  EXPECT_EQ(policy.attempts, 2);
}

TEST(Recovery, ExhaustionClearsAfterAHealthyWindow)
{
  RecoveryPolicy policy(1, 1.0, 10.0, 30.0);
  policy.update(true, 0.0);
  policy.update(true, 1.0);
  policy.update(true, 3.0);
  ASSERT_TRUE(policy.exhausted);
  policy.update(false, 5.0);
  EXPECT_TRUE(policy.exhausted);
  policy.update(false, 11.0);
  EXPECT_FALSE(policy.exhausted);
  EXPECT_EQ(policy.attempts, 0);
}

TEST(Debounce, RaisesAndClearsOnlyAfterHolding)
{
  Debounce debounce(3.0, 2.0);
  EXPECT_FALSE(debounce.update(true, 0.0));
  EXPECT_FALSE(debounce.update(true, 2.9));
  EXPECT_TRUE(debounce.update(true, 3.0));
  EXPECT_TRUE(debounce.update(false, 4.0));
  EXPECT_TRUE(debounce.update(false, 5.9));
  EXPECT_TRUE(debounce.active());
  EXPECT_FALSE(debounce.update(false, 6.0));
}

TEST(Debounce, ABlipRestartsTheTimer)
{
  Debounce debounce(3.0, 0.0);
  debounce.update(true, 0.0);
  debounce.update(false, 2.0);
  EXPECT_FALSE(debounce.update(true, 4.0));
  EXPECT_FALSE(debounce.update(true, 6.9));
  EXPECT_TRUE(debounce.update(true, 7.0));
  // Zero clear time: the first good reading clears it.
  EXPECT_FALSE(debounce.update(false, 7.1));
}

TEST(Validation, QuaternionNorm)
{
  EXPECT_TRUE(health::quaternion_norm_ok({0.0, 0.0, 0.0, 1.0}, 0.05));
  EXPECT_TRUE(health::quaternion_norm_ok({0.0, 0.0, 0.0, 1.04}, 0.05));
  EXPECT_FALSE(health::quaternion_norm_ok({0.0, 0.0, 0.0, 1.06}, 0.05));
  // The brick's failure mode: an all-zero quaternion before fusion has a solution.
  EXPECT_FALSE(health::quaternion_norm_ok({0.0, 0.0, 0.0, 0.0}, 0.05));
  EXPECT_FALSE(
    health::quaternion_norm_ok({0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}, 0.05));
}

TEST(Validation, AllFiniteAndMagnitude)
{
  EXPECT_TRUE(health::all_finite(std::array<double, 3>{1.0, 2.0, 3.0}));
  EXPECT_FALSE(
    health::all_finite(std::array<double, 3>{1.0, std::numeric_limits<double>::infinity(), 3.0}));
  EXPECT_DOUBLE_EQ(health::magnitude({3.0, 4.0, 0.0}), 5.0);
}

TEST(Calibration, IgnoresSysAndMagWhenFusionDoesNotUseTheMagnetometer)
{
  // (sys, gyro, acc, mag). In mode 2 sys never rises: it must not pin the level at 0.
  const idmind_imu::conversions::Calibration cal{0, 3, 2, 0};
  EXPECT_EQ(health::calibration_level(cal, 2), 2);
  EXPECT_EQ(health::calibration_level(cal, 0), 2);
  EXPECT_EQ(health::calibration_level(cal, 1), 0);
  EXPECT_EQ(health::calibration_level(cal, 3), 0);
  EXPECT_TRUE(health::fusion_uses_magnetometer(1));
  EXPECT_TRUE(health::fusion_uses_magnetometer(3));
  EXPECT_FALSE(health::fusion_uses_magnetometer(0));
  EXPECT_FALSE(health::fusion_uses_magnetometer(2));
}
