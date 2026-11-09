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

#include "idmind_imu/health.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace idmind_imu
{
namespace health
{

namespace
{

std::string format_seconds(const char * fmt, double seconds)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), fmt, seconds);
  return buffer;
}

void sort_by_severity(std::vector<Condition> & conds)
{
  std::stable_sort(
    conds.begin(), conds.end(), [](const Condition & a, const Condition & b) {
      if (a.level != b.level) {
        return a.level > b.level;
      }
      return a.since < b.since;
    });
}

}  // namespace

const char * level_name(Level level)
{
  switch (level) {
    case OK: return "OK";
    case WARN: return "WARN";
    default: return "ERROR";
  }
}

ConditionTracker::ConditionTracker(LogFn log, ClockFn clock, double reminder_period)
: reminder_period(reminder_period), log_(std::move(log)), clock_(std::move(clock))
{
}

bool ConditionTracker::set(
  const std::string & scope, const std::string & code, bool active, Level level,
  const std::string & msg)
{
  const auto key = std::make_pair(scope, code);
  const double now = clock_();
  auto it = active_.find(key);
  if (active) {
    if (it == active_.end()) {
      active_.emplace(key, Condition{code, level, msg, now, now});
      log_(level, "[" + code + "] " + msg);
    } else {
      Condition & cond = it->second;
      if (level != cond.level) {
        // Escalation or de-escalation is a transition too
        log_(level, "[" + code + "] " + msg);
        cond.last_log = now;
      }
      cond.level = level;
      cond.msg = msg;
      if (reminder_period > 0 && now - cond.last_log >= reminder_period) {
        cond.last_log = now;
        log_(
          level, "[" + code + "] " + msg +
          format_seconds(" (still active, %.0f s)", now - cond.since));
      }
    }
  } else if (it != active_.end()) {
    const double since = it->second.since;
    active_.erase(it);
    log_(OK, "[" + code + "] " + format_seconds("cleared after %.1f s", now - since));
  }
  return active;
}

bool ConditionTracker::clear(const std::string & scope, const std::string & code)
{
  return set(scope, code, false);
}

bool ConditionTracker::is_active(const std::string & scope, const std::string & code) const
{
  return active_.count(std::make_pair(scope, code)) > 0;
}

std::vector<Condition> ConditionTracker::active(const std::string & scope) const
{
  std::vector<Condition> conds;
  for (const auto & entry : active_) {
    if (entry.first.first == scope) {
      conds.push_back(entry.second);
    }
  }
  sort_by_severity(conds);
  return conds;
}

std::vector<Condition> ConditionTracker::active() const
{
  std::vector<Condition> conds;
  for (const auto & entry : active_) {
    conds.push_back(entry.second);
  }
  sort_by_severity(conds);
  return conds;
}

Level ConditionTracker::level(const std::string & scope) const
{
  Level worst = OK;
  for (const auto & entry : active_) {
    if (entry.first.first == scope) {
      worst = std::max(worst, entry.second.level);
    }
  }
  return worst;
}

std::string ConditionTracker::codes(const std::string & scope) const
{
  std::string joined;
  for (const auto & cond : active(scope)) {
    if (!joined.empty()) {
      joined += ",";
    }
    joined += cond.code;
  }
  return joined;
}

RecoveryPolicy::RecoveryPolicy(
  int max_attempts, double backoff_initial, double window, double probe_period)
: max_attempts(max_attempts), backoff_initial(backoff_initial), window(window),
  probe_period(probe_period)
{
}

double RecoveryPolicy::backoff(int n) const
{
  return backoff_initial * std::pow(2.0, n);
}

bool RecoveryPolicy::update(bool problem, double now)
{
  if (!problem) {
    recovering = false;
    next_attempt_.reset();
    if (attempts > 0 && last_attempt_ && now - *last_attempt_ >= window) {
      // Stayed healthy for a whole window after the last attempt: it worked
      attempts = 0;
      exhausted = false;
      last_attempt_.reset();
    }
    return false;
  }

  recovering = true;
  if (!next_attempt_) {
    // Problem just appeared: hold for one backoff step before the first attempt
    next_attempt_ = now + (exhausted ? probe_period : backoff(attempts));
  }
  if (now < *next_attempt_) {
    return false;
  }
  if (attempts >= max_attempts) {
    if (!exhausted) {
      // The budget ran out on this tick: report it, and wait a full probe period
      exhausted = true;
      next_attempt_ = now + probe_period;
      return false;
    }
    last_attempt_ = now;
    next_attempt_ = now + probe_period;
    return true;
  }

  attempts += 1;
  last_attempt_ = now;
  next_attempt_ = now + backoff(attempts);
  return true;
}

Debounce::Debounce(double raise_after, double clear_after)
: raise_after(raise_after), clear_after(clear_after)
{
}

bool Debounce::update(bool bad, double now)
{
  if (bad == active_) {
    since_.reset();
    return active_;
  }
  if (!since_) {
    since_ = now;
  }
  if (now - *since_ >= (bad ? raise_after : clear_after)) {
    active_ = bad;
    since_.reset();
  }
  return active_;
}

void Debounce::reset()
{
  active_ = false;
  since_.reset();
}

double magnitude(const std::array<double, 3> & v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool quaternion_norm_ok(const std::array<double, 4> & q, double tolerance)
{
  if (!all_finite(q)) {
    return false;
  }
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return std::abs(norm - 1.0) <= tolerance;
}

bool fusion_uses_magnetometer(int fusion_mode)
{
  return fusion_mode == 1 || fusion_mode == 3;
}

int calibration_level(const conversions::Calibration & calibration, int fusion_mode)
{
  const int gyro_acc = std::min(calibration[1], calibration[2]);
  if (!fusion_uses_magnetometer(fusion_mode)) {
    return gyro_acc;
  }
  return std::min({gyro_acc, static_cast<int>(calibration[0]), static_cast<int>(calibration[3])});
}

}  // namespace health
}  // namespace idmind_imu
