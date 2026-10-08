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
/// Health bookkeeping for the IMU node, kept free of ROS so it can be unit tested.
///
/// - Condition codes: stable identifiers a monitor can parse, attached to every log line and
///   diagnostic status the node emits for a non-nominal condition.
/// - ConditionTracker: edge-triggered logging of those conditions, with a periodic reminder.
/// - RecoveryPolicy: retry a bounded number of times with backoff, then keep probing slowly.
/// - Debounce: a condition must hold for a while before it raises, and be gone before it clears.
/// - Sample validation and the calibration rule.
///
/// An IMU-specific, trimmed copy of the primitives in ``idmind_zlac8015d``. The IMU drives
/// nothing, so unlike the motors there is no latch and no human-required state.

#ifndef IDMIND_IMU__HEALTH_HPP_
#define IDMIND_IMU__HEALTH_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "idmind_imu/conversions.hpp"

namespace idmind_imu
{
namespace health
{

/// Severity, matching ``diagnostic_msgs/DiagnosticStatus`` OK/WARN/ERROR.
enum Level : uint8_t { OK = 0, WARN = 1, ERROR = 2 };

/// "OK", "WARN" or "ERROR".
const char * level_name(Level level);

// Stable condition codes. Monitors parse these, never the human-readable message.
constexpr char CONNECT_FAILED[] = "IMU_CONNECT_FAILED";
constexpr char DEVICE_MISSING[] = "IMU_DEVICE_MISSING";
constexpr char NO_DATA[] = "IMU_NO_DATA";
constexpr char RATE_LOW[] = "IMU_RATE_LOW";
constexpr char DATA_INVALID[] = "IMU_DATA_INVALID";
constexpr char DATA_FROZEN[] = "IMU_DATA_FROZEN";
constexpr char CONFIG_FAILED[] = "IMU_CONFIG_FAILED";
constexpr char UNCALIBRATED[] = "IMU_UNCALIBRATED";
constexpr char RECOVERING[] = "IMU_RECOVERING";
constexpr char RECOVERY_EXHAUSTED[] = "IMU_RECOVERY_EXHAUSTED";
constexpr char PARAM_FALLBACK[] = "IMU_PARAM_FALLBACK";
constexpr char STANDBY[] = "IMU_STANDBY";

/// One active condition.
struct Condition
{
  std::string code;
  Level level;
  std::string msg;
  /// Clock time it was raised.
  double since;
  /// Clock time it was last logged.
  double last_log;
};

/// Tracks active conditions per scope (a diagnostic task name) and logs only on transitions.
///
/// Logging once on raise, once on clear and every ``reminder_period`` while active keeps a
/// persistent condition visible without flooding the log at the watchdog rate. A clear is
/// logged at ``OK``, which the node maps to INFO. Not thread-safe: the caller locks.
class ConditionTracker
{
public:
  using LogFn = std::function<void (Level, const std::string &)>;
  using ClockFn = std::function<double ()>;

  ConditionTracker(LogFn log, ClockFn clock, double reminder_period = 30.0);

  /// Raise (active=true) or clear (active=false) a condition. Returns active.
  bool set(
    const std::string & scope, const std::string & code, bool active, Level level = ERROR,
    const std::string & msg = "");
  bool clear(const std::string & scope, const std::string & code);
  bool is_active(const std::string & scope, const std::string & code) const;
  /// Active conditions of scope, most severe first, then oldest first.
  std::vector<Condition> active(const std::string & scope) const;
  /// Every active condition, most severe first, then oldest first.
  std::vector<Condition> active() const;
  /// Worst level in scope, OK if none.
  Level level(const std::string & scope) const;
  /// Comma-separated codes of scope, in ``active(scope)`` order, for a diagnostic KeyValue.
  std::string codes(const std::string & scope) const;

  /// Seconds between reminders of a condition that stays active; <= 0 disables them.
  double reminder_period;

private:
  LogFn log_;
  ClockFn clock_;
  std::map<std::pair<std::string, std::string>, Condition> active_;
};

/// Decides when to attempt a recovery for a persisting problem.
///
/// Call ``update(problem, now)`` every watchdog tick. It returns true when the caller must
/// attempt a recovery now. Attempts are spaced by an exponential backoff starting at
/// ``backoff_initial``, and the first one waits one backoff step so a single late sample does
/// not trigger it. An attempt only counts as successful once the problem stays away for
/// ``window`` seconds; a problem that comes back inside the window counts as another failed
/// attempt. After ``max_attempts`` failed attempts the policy is ``exhausted`` and attempts
/// once every ``probe_period`` instead, forever: there is no latch.
class RecoveryPolicy
{
public:
  RecoveryPolicy(
    int max_attempts = 3, double backoff_initial = 1.0, double window = 10.0,
    double probe_period = 30.0);

  bool update(bool problem, double now);

  int max_attempts;
  double backoff_initial;
  double window;
  double probe_period;
  /// Failed attempts since the last success. Stops counting once exhausted.
  int attempts = 0;
  /// The budget is spent; only slow probes remain.
  bool exhausted = false;
  /// A problem is present and recovery is pending or in progress.
  bool recovering = false;

private:
  double backoff(int attempts) const;

  std::optional<double> next_attempt_;
  std::optional<double> last_attempt_;
};

/// A condition that raises only after it held for ``raise_after`` seconds and clears only
/// after it was absent for ``clear_after`` seconds, so it does not flap.
class Debounce
{
public:
  Debounce(double raise_after, double clear_after);

  /// Feed the instantaneous condition; returns the debounced state.
  bool update(bool bad, double now);
  bool active() const {return active_;}
  void reset();

  double raise_after;
  double clear_after;

private:
  bool active_{false};
  std::optional<double> since_;
};

/// True if every element is finite.
template<std::size_t N>
bool all_finite(const std::array<double, N> & values)
{
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

/// Euclidean norm of a 3-vector.
double magnitude(const std::array<double, 3> & v);

/// True if the quaternion is finite and its norm is within ``1 +/- tolerance``.
bool quaternion_norm_ok(const std::array<double, 4> & q, double tolerance);

/// Whether a BNO-055 fusion mode uses the magnetometer (1 = on, 3 = on without fast mag
/// calibration). Modes 0 (off) and 2 (no magnetometer) do not.
bool fusion_uses_magnetometer(int fusion_mode);

/// The calibration level that matters for \p fusion_mode, from (sys, gyro, acc, mag).
///
/// ``sys`` only rises when the magnetometer is in use, so in modes 0 and 2 it stays 0
/// forever; it and ``mag`` are ignored there and only gyro and acc count.
int calibration_level(const conversions::Calibration & calibration, int fusion_mode);

}  // namespace health
}  // namespace idmind_imu

#endif  // IDMIND_IMU__HEALTH_HPP_
