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
/// Per-axis rolling variance over a sliding window of samples.
///
/// This is what makes the published covariances genuinely live: instead of a base stddev
/// scaled by one of four discrete calibration factors - which is constant in practice once
/// the device settles - the covariance is measured from the signal actually arriving.
///
/// Pure maths: no ROS, no hardware, unit tested on its own.

#ifndef IDMIND_IMU__NOISE_ESTIMATOR_HPP_
#define IDMIND_IMU__NOISE_ESTIMATOR_HPP_

#include <array>
#include <cstddef>
#include <deque>
#include <optional>

namespace idmind_imu
{

/// Whether a channel's values are plain scalars or angles that wrap at +/- pi.
enum class AngleWrap
{
  /// Ordinary quantities: angular velocity, acceleration, magnetic field.
  None,
  /// Angles in radians. Yaw sitting near +/- pi would otherwise show enormous fake variance.
  Radians,
};

/// Sliding-window, per-axis variance estimator for a 3-axis signal.
///
/// Keeps the last ``window`` samples and recomputes the variance in two passes (mean, then
/// summed squared deviations) rather than from running sums of squares. That costs O(window)
/// per sample - negligible at IMU rates - and avoids the catastrophic cancellation that
/// ``E[x^2] - E[x]^2`` suffers for a signal like linear acceleration, where the mean (~9.81)
/// dwarfs the variance (~1e-3).
class RollingVariance
{
public:
  /// Build an estimator over \p window samples, treating values per \p wrap.
  explicit RollingVariance(size_t window = 100, AngleWrap wrap = AngleWrap::None);

  /// Append one sample, evicting the oldest once the window is full.
  void push(const std::array<double, 3> & sample);

  /// Drop every stored sample, e.g. when the window size changes.
  void reset();

  /// How many samples are currently stored.
  size_t size() const {return samples_.size();}

  /// The configured window length.
  size_t window() const {return window_;}

  /// Whether enough samples have accumulated for a meaningful estimate.
  ///
  /// Requires a full window: a partially filled one would report an over-confident variance
  /// early on, which is precisely when the estimate matters most.
  bool ready() const {return samples_.size() >= window_ && window_ >= 2;}

  /// Per-axis mean over the window. Circular mean when wrapping is enabled.
  std::array<double, 3> mean() const;

  /// Per-axis sample variance (N-1 denominator), or nullopt while not ``ready()``.
  std::optional<std::array<double, 3>> variance() const;

private:
  size_t window_;
  AngleWrap wrap_;
  std::deque<std::array<double, 3>> samples_;
};

/// Wrap \p angle into [-pi, pi].
double wrap_to_pi(double angle);

}  // namespace idmind_imu

#endif  // IDMIND_IMU__NOISE_ESTIMATOR_HPP_
