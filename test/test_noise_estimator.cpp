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
/// Unit tests for the sliding-window noise estimator: pure math, no ROS, no hardware.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "idmind_imu/noise_estimator.hpp"

using idmind_imu::AngleWrap;
using idmind_imu::RollingVariance;
using idmind_imu::wrap_to_pi;

TEST(NoiseEstimator, NotReadyUntilTheWindowIsFull)
{
  // A partially filled window would report an over-confident variance exactly when the
  // estimate matters most, so nothing is reported until it fills.
  RollingVariance estimator(5);
  for (int i = 0; i < 4; ++i) {
    EXPECT_FALSE(estimator.ready());
    EXPECT_FALSE(estimator.variance().has_value());
    estimator.push({1.0, 1.0, 1.0});
  }
  estimator.push({1.0, 1.0, 1.0});
  EXPECT_TRUE(estimator.ready());
  EXPECT_TRUE(estimator.variance().has_value());
}

TEST(NoiseEstimator, ConstantSignalHasZeroVariance)
{
  RollingVariance estimator(10);
  for (int i = 0; i < 10; ++i) {
    estimator.push({2.5, -1.0, 0.0});
  }
  const auto variance = estimator.variance();
  ASSERT_TRUE(variance.has_value());
  for (double v : *variance) {
    EXPECT_NEAR(v, 0.0, 1e-18);
  }
}

TEST(NoiseEstimator, MatchesHandComputedSampleVariance)
{
  // Values 1..5: mean 3, sum of squared deviations 10, sample variance 10 / 4 = 2.5.
  RollingVariance estimator(5);
  for (int i = 1; i <= 5; ++i) {
    const double v = static_cast<double>(i);
    estimator.push({v, 2.0 * v, 0.0});
  }
  const auto variance = estimator.variance();
  ASSERT_TRUE(variance.has_value());
  EXPECT_NEAR((*variance)[0], 2.5, 1e-12);
  EXPECT_NEAR((*variance)[1], 10.0, 1e-12);  // doubled values -> 4x the variance
  EXPECT_NEAR((*variance)[2], 0.0, 1e-18);
}

TEST(NoiseEstimator, WindowSlidesAndForgetsOldSamples)
{
  RollingVariance estimator(3);
  for (int i = 0; i < 3; ++i) {
    estimator.push({100.0, 0.0, 0.0});
  }
  ASSERT_TRUE(estimator.variance().has_value());
  EXPECT_GT((*estimator.variance())[0], -1.0);

  // Push a full window of a different constant; the old samples must be gone entirely.
  for (int i = 0; i < 3; ++i) {
    estimator.push({7.0, 0.0, 0.0});
  }
  EXPECT_NEAR(estimator.mean()[0], 7.0, 1e-12);
  EXPECT_NEAR((*estimator.variance())[0], 0.0, 1e-18);
  EXPECT_EQ(estimator.size(), 3u);
}

TEST(NoiseEstimator, SurvivesLargeMeanWithTinyVariance)
{
  // The catastrophic-cancellation case: linear acceleration sits near 9.81 while its noise is
  // ~1e-3. A naive E[x^2] - E[x]^2 loses this completely.
  RollingVariance estimator(200);
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(9.81, 0.001);
  std::vector<double> pushed;
  for (int i = 0; i < 200; ++i) {
    const double v = noise(rng);
    pushed.push_back(v);
    estimator.push({v, 0.0, 0.0});
  }

  double mean = 0.0;
  for (double v : pushed) {
    mean += v;
  }
  mean /= static_cast<double>(pushed.size());
  double expected = 0.0;
  for (double v : pushed) {
    expected += (v - mean) * (v - mean);
  }
  expected /= static_cast<double>(pushed.size() - 1);

  const auto variance = estimator.variance();
  ASSERT_TRUE(variance.has_value());
  EXPECT_NEAR((*variance)[0], expected, 1e-15);
  EXPECT_GT((*variance)[0], 0.0);
  EXPECT_LT((*variance)[0], 1e-4);
}

TEST(NoiseEstimator, WrapToPiNormalises)
{
  EXPECT_NEAR(wrap_to_pi(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrap_to_pi(M_PI - 0.1), M_PI - 0.1, 1e-12);
  EXPECT_NEAR(wrap_to_pi(-M_PI + 0.1), -M_PI + 0.1, 1e-12);
  EXPECT_NEAR(wrap_to_pi(2.0 * M_PI + 0.5), 0.5, 1e-12);
  EXPECT_NEAR(wrap_to_pi(-2.0 * M_PI - 0.5), -0.5, 1e-12);

  // The half-open [-pi, pi) convention: an angle equivalent to pi comes back as -pi. Both
  // represent the same direction, so assert equivalence rather than one representative.
  EXPECT_NEAR(std::fabs(wrap_to_pi(3.0 * M_PI)), M_PI, 1e-12);
  EXPECT_NEAR(std::fabs(wrap_to_pi(M_PI)), M_PI, 1e-12);
}

TEST(NoiseEstimator, WrapToPiOutputIsAlwaysInRange)
{
  for (int i = -20; i <= 20; ++i) {
    const double angle = i * 0.7 * M_PI;
    const double wrapped = wrap_to_pi(angle);
    EXPECT_GE(wrapped, -M_PI - 1e-12);
    EXPECT_LE(wrapped, M_PI + 1e-12);
    // Wrapping must preserve the direction it represents.
    EXPECT_NEAR(std::sin(wrapped), std::sin(angle), 1e-9);
    EXPECT_NEAR(std::cos(wrapped), std::cos(angle), 1e-9);
  }
}

TEST(NoiseEstimator, AngleWrappingKeepsVarianceSmallAcrossTheSeam)
{
  // Yaw jittering either side of +/- pi is a quiet signal. Treated linearly it would look like
  // it swings the full 2*pi, reporting a variance near 9.87 instead of ~1e-6.
  RollingVariance linear(10, AngleWrap::None);
  RollingVariance circular(10, AngleWrap::Radians);
  for (int i = 0; i < 10; ++i) {
    const double yaw = (i % 2 == 0) ? (M_PI - 0.001) : (-M_PI + 0.001);
    linear.push({0.0, 0.0, yaw});
    circular.push({0.0, 0.0, yaw});
  }

  const auto linear_variance = linear.variance();
  const auto circular_variance = circular.variance();
  ASSERT_TRUE(linear_variance.has_value());
  ASSERT_TRUE(circular_variance.has_value());

  EXPECT_GT((*linear_variance)[2], 1.0) << "linear treatment should blow up across the seam";
  EXPECT_LT((*circular_variance)[2], 1e-4) << "wrapped treatment must stay small";
}

TEST(NoiseEstimator, CircularMeanSitsBetweenWrappedValues)
{
  RollingVariance circular(4, AngleWrap::Radians);
  for (int i = 0; i < 2; ++i) {
    circular.push({0.0, 0.0, M_PI - 0.01});
    circular.push({0.0, 0.0, -M_PI + 0.01});
  }
  // The mean direction is +/- pi, not 0 as an arithmetic mean would give.
  EXPECT_NEAR(std::fabs(circular.mean()[2]), M_PI, 1e-6);
}

TEST(NoiseEstimator, SetWindowShrinksAndInvalidates)
{
  RollingVariance estimator(10);
  for (int i = 0; i < 10; ++i) {
    estimator.push({1.0, 2.0, 3.0});
  }
  ASSERT_TRUE(estimator.ready());

  estimator.set_window(4);
  EXPECT_EQ(estimator.window(), 4u);
  EXPECT_EQ(estimator.size(), 4u);
  EXPECT_TRUE(estimator.ready());

  estimator.set_window(50);
  EXPECT_FALSE(estimator.ready()) << "a grown window must refill before reporting again";
}

TEST(NoiseEstimator, WindowIsClampedToAUsableMinimum)
{
  // Sample variance needs at least two points; a window of 0 or 1 must not divide by zero.
  RollingVariance estimator(0);
  EXPECT_GE(estimator.window(), 2u);
  estimator.push({1.0, 1.0, 1.0});
  estimator.push({3.0, 1.0, 1.0});
  const auto variance = estimator.variance();
  ASSERT_TRUE(variance.has_value());
  EXPECT_NEAR((*variance)[0], 2.0, 1e-12);  // values 1 and 3 -> sample variance 2
  EXPECT_TRUE(std::isfinite((*variance)[0]));
}

TEST(NoiseEstimator, ResetClearsEverything)
{
  RollingVariance estimator(3);
  for (int i = 0; i < 3; ++i) {
    estimator.push({1.0, 1.0, 1.0});
  }
  ASSERT_TRUE(estimator.ready());
  estimator.reset();
  EXPECT_FALSE(estimator.ready());
  EXPECT_EQ(estimator.size(), 0u);
  EXPECT_FALSE(estimator.variance().has_value());
}
