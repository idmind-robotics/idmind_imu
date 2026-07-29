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

#include "idmind_imu/noise_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace idmind_imu
{

double wrap_to_pi(double angle)
{
  const double two_pi = 2.0 * M_PI;
  double wrapped = std::fmod(angle + M_PI, two_pi);
  if (wrapped < 0.0) {
    wrapped += two_pi;
  }
  return wrapped - M_PI;
}

RollingVariance::RollingVariance(size_t window, AngleWrap wrap)
: window_(std::max<size_t>(window, 2)), wrap_(wrap)
{
}

void RollingVariance::push(const std::array<double, 3> & sample)
{
  samples_.push_back(sample);
  while (samples_.size() > window_) {
    samples_.pop_front();
  }
}

void RollingVariance::reset()
{
  samples_.clear();
}

void RollingVariance::set_window(size_t window)
{
  const size_t clamped = std::max<size_t>(window, 2);
  if (clamped == window_) {
    return;
  }
  window_ = clamped;
  while (samples_.size() > window_) {
    samples_.pop_front();
  }
}

std::array<double, 3> RollingVariance::mean() const
{
  std::array<double, 3> result{0.0, 0.0, 0.0};
  if (samples_.empty()) {
    return result;
  }
  const double count = static_cast<double>(samples_.size());

  for (size_t axis = 0; axis < 3; ++axis) {
    if (wrap_ == AngleWrap::Radians) {
      // Averaging angles arithmetically is wrong across the +/- pi seam, so average the unit
      // vectors and take the resulting direction.
      double sum_sin = 0.0;
      double sum_cos = 0.0;
      for (const auto & sample : samples_) {
        sum_sin += std::sin(sample[axis]);
        sum_cos += std::cos(sample[axis]);
      }
      result[axis] = std::atan2(sum_sin / count, sum_cos / count);
    } else {
      double sum = 0.0;
      for (const auto & sample : samples_) {
        sum += sample[axis];
      }
      result[axis] = sum / count;
    }
  }
  return result;
}

std::optional<std::array<double, 3>> RollingVariance::variance() const
{
  if (!ready()) {
    return std::nullopt;
  }

  const std::array<double, 3> centre = mean();
  const double count = static_cast<double>(samples_.size());
  std::array<double, 3> result{0.0, 0.0, 0.0};

  for (size_t axis = 0; axis < 3; ++axis) {
    double sum_squared = 0.0;
    for (const auto & sample : samples_) {
      double deviation = sample[axis] - centre[axis];
      if (wrap_ == AngleWrap::Radians) {
        deviation = wrap_to_pi(deviation);
      }
      sum_squared += deviation * deviation;
    }
    // Sample variance: the window is a sample of the noise process, not the population.
    result[axis] = sum_squared / (count - 1.0);
  }
  return result;
}

}  // namespace idmind_imu
