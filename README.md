# idmind_imu

ROS 2 wrappers for the IMUs used on IDMind robots.

The package is split into a **hardware-agnostic node** and a **driver layer**. The node owns
all ROS concerns (parameters, topics, diagnostics, watchdog); each driver owns one IMU model
and reports readings as SI-unit samples. Supporting a new IMU means adding one driver file,
not another node.

Currently shipped drivers:

| `driver` | Hardware |
|---|---|
| `brick_v2` | [TinkerForge IMU Brick 2.0](https://www.tinkerforge.com/en/doc/Hardware/Bricks/IMU_V2_Brick.html) (BNO-055) |

Written in **C++** (`ament_cmake`). Tested on **Ubuntu 22.04** with **ROS 2 Humble**, and
also builds and passes its tests on **ROS 2 Jazzy**.

---

## Prerequisites

For the `brick_v2` driver:

Install [BrickDaemon](https://www.tinkerforge.com/en/doc/Software/Brickd.html#brickd) — it
proxies between USB and the TCP port the driver connects to.

That is the only runtime prerequisite.

The TinkerForge **C bindings are fetched at configure time** from the official release
(pinned to a version and SHA256 in `CMakeLists.txt`) and compiled into the package. This is
the upstream-recommended way to use them: TinkerForge ships
[no precompiled library](https://www.tinkerforge.com/en/doc/Software/API_Bindings_C.html)
and no distro package, so there is nothing to `apt install` or link against. Nothing extra to
install, and `rosdep install` works unmodified.

> **A clean build needs network access.** To build offline, download
> `tinkerforge_c_bindings_<version>.zip` yourself and point the build at its `source/`
> directory:
> ```bash
> colcon build --packages-select idmind_imu \
>   --cmake-args -DTINKERFORGE_C_BINDINGS_DIR=/path/to/source
> ```
>
> To upgrade the bindings, bump `TINKERFORGE_C_BINDINGS_VERSION` and
> `TINKERFORGE_C_BINDINGS_SHA256` together.

The device UID is discovered automatically by enumeration; you do not need to configure it.

## Build and run

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select idmind_imu
source install/setup.bash

ros2 launch idmind_imu idmind_imu_brick.launch.py
```

Launch arguments: `robot_name` (namespace), `imu_cfg` (parameter file), `log_level`
(default `warn`), and `diagnostics_topic` (default `/diagnostics`; set it to the robot's
region, e.g. `/diagnostics/base`).

Or directly, selecting a driver:

```bash
ros2 run idmind_imu imu_node --ros-args -p driver:=brick_v2 -p imu_frame:=imu
```

---

## Node: `imu_node`

The node is named **`idmind_imu`**, and topic names are built from the node name — so with the
shipped launch file the topics are `/idmind_imu/...`.

The console scripts `imu_brick_node` and `imu_brick_node_v2` also exist and run this same
node; they are retained so existing launch files keep working.

### Parameters

All are dynamically reconfigurable (`ros2 param set`); hardware-relevant changes are pushed
to the driver immediately rather than polled.

Values are validated. An invalid value **at startup** falls back to its default, is written
back (so `ros2 param get` shows what is really used), and raises `IMU_PARAM_FALLBACK` until a
valid value is set. An invalid value **at runtime** is rejected with a reason, and the whole
batch is left unapplied.

| Name | Type | Default | Description |
|---|---|---|---|
| `driver` | string | `brick_v2` | Which IMU backend to load |
| `host` | string | `localhost` | Address of the transport (BrickDaemon) |
| `port` | int | `4223` | Port of the transport |
| `control_freq` | double | `20.0` | Watchdog loop frequency (Hz) |
| `imu_freq` | double | `20.0` | Requested sensor stream frequency (Hz) |
| `imu_frame` | string | `imu` | `frame_id` stamped on all messages |
| `imu_leds` | bool | `false` | Drive the brick's LEDs and status LED |
| `imu_fusion_mode` | int | `2` | BNO-055 fusion mode — see below |
| `timeout` | double | `1.0` | Seconds without data before `IMU_NO_DATA`; also the grace for `IMU_CONNECT_FAILED` / `IMU_DEVICE_MISSING` |
| `auto_reconnect` | bool | `true` | Let the driver reconnect automatically |
| `acceleration_source` | string | `linear` | `linear` (gravity removed) or `raw` (gravity included) |
| `orientation_stddev` | double | `0.01` | Base orientation stddev at full calibration |
| `temperature_stddev` | double | `0.0` | Temperature stddev (°C); `0` (the default) publishes variance `0` = unknown |
| `angular_velocity_stddev` | double[3] | `[0.005236]*3` | Per-axis gyro stddev (rad/s), scaled by **gyro** calibration |
| `linear_acceleration_stddev` | double[3] | `[0.1, 0.1, 0.2236]` | Per-axis accel stddev (m/s²), scaled by **acc** calibration |
| `magnetic_field_stddev` | double[3] | `[6e-7]*3` | Per-axis mag stddev (T), scaled by **mag** calibration |
| `noise_window` | int | `100` | Samples used to measure covariance from the live signal; `<2` disables |
| `noise_estimate_when_still` | bool | `true` | Only update the estimate while the robot looks stationary |
| `stationary_gyro_threshold` | double | `0.02` | Gyro magnitude (rad/s) below which the robot counts as still |
| `stationary_accel_threshold` | double | `0.2` | Allowed accel deviation (m/s²) from the window mean while still |
| `rate_warn_ratio` | double | `0.8` | `IMU_RATE_LOW` when the rate stays below this × `imu_freq` for 3 s |
| `frozen_timeout` | double | `5.0` | `IMU_DATA_FROZEN` when every field is bit-identical this long (s) |
| `quaternion_norm_tolerance` | double | `0.05` | Samples with quaternion norm outside `1 ± this` are dropped |
| `max_acceleration` | double | `156.9` | Samples with raw acceleration above this (m/s², 16 g) are dropped |
| `calibration_grace_period` | double | `60.0` | Seconds after the first sample before `IMU_UNCALIBRATED` can raise |
| `recovery_max_attempts` | int | `3` | Recovery attempts before `IMU_RECOVERY_EXHAUSTED` |
| `recovery_backoff_initial` | double | `1.0` | Delay (s) before the first attempt; doubles after each |
| `recovery_window` | double | `10.0` | Seconds of healthy data for a recovery to count as successful |
| `reconnect_probe_period` | double | `30.0` | Seconds between probes once the attempts are exhausted |
| `log_reminder_period` | double | `30.0` | Seconds between log reminders of a condition still active; `0` disables |

**Fusion modes:** `0` off (raw data — orientation is meaningless), `1` on with magnetometer
(absolute heading), `2` on without magnetometer (**relative yaw that drifts** — the default),
`3` on without fast magnetometer calibration.

**`acceleration_source`** decides what fills `Imu.linear_acceleration`. The default `linear`
preserves historical behaviour and reports gravity-compensated acceleration. Consumers such
as `robot_localization` generally expect gravity to be *included*; use `raw` for those.

### Published topics

| Topic | Type | Notes |
|---|---|---|
| `~/imu` | `sensor_msgs/Imu` | Orientation (quaternion), angular velocity (rad/s), linear acceleration (m/s²) |
| `~/temperature` | `sensor_msgs/Temperature` | Sensor temperature (°C); `variance` from `temperature_stddev` |
| `~/magnetic_field` | `sensor_msgs/MagneticField` | Tesla |
| `~/gravity` | `geometry_msgs/Vector3Stamped` | Gravity vector (m/s²) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Published at 1 Hz by `diagnostic_updater`; remapped by the `diagnostics_topic` launch argument |

### Services

| Name | Type | Description |
|---|---|---|
| `~/ready` | `std_srvs/Trigger` | `success` only when the state is `RUNNING` and data is fresh. `message` carries the state and, if not ready, the most severe active code, e.g. `idmind_imu is not ready (NO_DEVICE): IMU_DEVICE_MISSING` |
| `~/standby` | `std_srvs/SetBool` | `data: true` suspends all data-topic publishing; `data: false` resumes it. Samples keep arriving from the driver while suspended, so the watchdog, diagnostics and the noise estimate stay live (the `standby` key in the "Data flow" diagnostic reflects the current state). |

### Covariances

- Off-diagonal terms are always exactly zero — the matrices are strictly diagonal.
- **Covariance is measured from the live signal.** Each axis reports the variance of the last
  `noise_window` samples, so the matrix changes from message to message rather than being a
  constant. This is the primary source; the fallbacks below only apply when it is unavailable.

- **The estimate is only updated while the robot is still.** A moving IMU's variance is
  dominated by real motion, not sensor noise, and publishing that would tell a filter to
  distrust the IMU exactly when it is manoeuvring. While moving, the last good estimate is
  held. Set `noise_estimate_when_still: false` to track the raw signal unconditionally.

- **Fallback chain**, applied per axis, so a single silent axis never discards a good
  measurement on the other two:
  1. measured variance, once the window has filled and that axis is non-zero;
  2. otherwise the configured `*_stddev`, scaled by the BNO-055 calibration level for *that*
     sensor — orientation with `sys`, angular velocity with `gyro`, linear acceleration with
     `acc`, magnetic field with `mag`:

     | Calibration level | Variance multiplier |
     |---|---|
     | 3 (fully calibrated) | ×1 |
     | 2 | ×4 |
     | 1 | ×25 |
     | 0 (uncalibrated) | ×100 |

  3. `imu_fusion_mode: 0` always overrides both for orientation, with the `-1` sentinel.

  > Note the BNO-055 only raises its **`sys`** calibration when the magnetometer is in use. In
  > `imu_fusion_mode: 2` (the default, no magnetometer) `sys` stays at 0 permanently, so that
  > fallback is pinned at ×100 — which is why measuring from the signal matters.

- **Yaw wrapping is handled.** Orientation variance uses a circular mean and wrapped
  deviations, so a yaw sitting near ±π does not report a fake variance of ~9.87 rad².

- A stddev of `0`, negative, or `NaN` yields the `-1` sentinel rather than an all-zero matrix.
  This matters: `sensor_msgs` has no "all zeros means unknown" convention, so a zero matrix
  tells a consumer the reading is *perfectly certain*.
- `orientation_covariance[0]` is set to **`-1`** when `imu_fusion_mode` is `0`. That is the
  `sensor_msgs/Imu` convention for "no orientation estimate available", and downstream
  filters check it.
- Otherwise orientation variance scales with the reported **system calibration** level, and
  in fusion mode `2` the yaw term is inflated further because yaw is relative and drifts
  without the magnetometer.

Expect the orientation covariance to start large and shrink as the BNO-055 calibrates. That
is intentional and honest; consumers that previously saw a fixed `1e-4` will now see
orientation de-weighted during warm-up.

### Diagnostics

Status is reported on `/diagnostics` only, as severity levels plus KeyValues, by three tasks:
**Connection**, **Data flow** and **Calibration**. Every task has an `active_codes` key, a
comma-separated list of the condition codes below, most severe first. The task's level is the
worst of them, and its message is the worst condition's message, prefixed with its code.
Monitors should parse the codes, never the messages.

| Code | Level | Task | Raised when | Cleared when |
|---|---|---|---|---|
| `IMU_CONNECT_FAILED` | ERROR | Connection | Transport unreachable for `timeout` | Connected |
| `IMU_DEVICE_MISSING` | ERROR | Connection | Connected, but no IMU found for `timeout` | Device enumerated |
| `IMU_CONFIG_FAILED` | WARN | Connection | A hardware getter/setter failed while applying the config | The next config apply succeeds |
| `IMU_RECOVERING` | WARN | Connection | Silent or frozen while connected; recovery pending or running | Data healthy again |
| `IMU_RECOVERY_EXHAUSTED` | ERROR | Connection | `recovery_max_attempts` spent; probing every `reconnect_probe_period` | Data healthy again |
| `IMU_PARAM_FALLBACK` | WARN | Connection | A startup parameter was invalid; its default is in use | A valid value is set |
| `IMU_NO_DATA` | ERROR | Data flow | No sample for `timeout` | A sample arrives |
| `IMU_RATE_LOW` | WARN | Data flow | Rate below `rate_warn_ratio` × `imu_freq` for 3 s | Back at rate for 3 s |
| `IMU_DATA_INVALID` | WARN | Data flow | A sample was dropped: non-finite value, quaternion norm outside `1 ± quaternion_norm_tolerance`, or raw acceleration above `max_acceleration` | 1 s without an invalid sample |
| `IMU_DATA_FROZEN` | ERROR | Data flow | **Every** field (quaternion, gyro, linear and raw acceleration, mag) bit-identical for `frozen_timeout` | Any field changes |
| `IMU_UNCALIBRATED` | WARN | Calibration | A calibration level the fusion mode uses is 0, after `calibration_grace_period` | That level rises |

Other KeyValues:

- **Connection:** `state`, `recovery_attempts`, `last_error` (the driver's most recent error),
  `detail`.
- **Data flow:** `rate_hz`, `age_s`, `loop_period_s` (the watchdog's own measured tick period, a
  leading indicator of an executor stall, since it moves at `control_freq` while the rest of
  the task only resolves at the Updater's 1 Hz rate), and `standby`.
- **Calibration:** `sys`, `gyro`, `acc`, `mag` and `fusion_mode`. All four levels are always
  reported. The task is `STALE` until the first sample.

**Node states**, reported as `state`: `DISCONNECTED` (transport down), `NO_DEVICE` (connected,
no IMU), `RUNNING`, and `RECOVERING` (connected, but silent or frozen). Standby is a flag, not
a state.

**Recovery.** While connected but silent or frozen, the node runs a ladder: first it
re-enumerates and re-sends the all-data period (a brick that stopped streaming still reports
the period it was given), then it drops and reopens the BrickDaemon connection. Attempts back
off by doubling from `recovery_backoff_initial`, and one only counts as successful after
`recovery_window` seconds of healthy data. After `recovery_max_attempts`, the node raises
`IMU_RECOVERY_EXHAUSTED` and keeps reconnecting once every `reconnect_probe_period`. There is
no latch and nothing to clear by hand: the IMU drives nothing, so retrying is always safe.

**Calibration** ignores `sys` and `mag` in fusion modes `0` and `2`, which do not use the
magnetometer: there `sys` never rises above 0, and counting it reported `WARN` forever.

**Frozen data on a still robot.** On a perfectly still robot, the gyro can repeat one value
(1/16 °/s steps), the magnetometer is suspended in mode `2`, and the fused quaternion can sit
on one value. Only raw accelerometer noise (about 1 LSB = 1 cm/s² RMS, from the datasheet)
keeps the frame changing, which is why `IMU_DATA_FROZEN` requires *every* field to repeat,
and for 5 s (about 100 samples at 20 Hz). **This has not yet been measured on hardware.**
Before relying on it, park a robot in mode `2` for 10 minutes, record `~/imu`, and check that
the longest run of identical frames is well below `frozen_timeout`.

**Logging.** Each condition is logged once when raised (WARN or ERROR, prefixed with its
code), once when cleared (INFO), and every `log_reminder_period` while it stays active.
Otherwise INFO is used only at startup and shutdown. Drivers log only at DEBUG and report
problems through their state.

---

## Adding a new IMU driver

1. Add `include/idmind_imu/drivers/<your_imu>.hpp` and `src/drivers/<your_imu>.cpp` with a
   class deriving from `ImuDriver` (`include/idmind_imu/drivers/driver.hpp`) implementing
   `start()`, `stop()`, `apply_config()`, `state()` and `recover()`.
2. Convert readings to SI units and report them by calling the sample callback with an
   `ImuSample`. Put reusable unit maths in `src/conversions.cpp` and unit test it there.
3. Add one line to the table in `src/drivers/registry.cpp`, and the new `.cpp` to the library
   in `CMakeLists.txt`.
4. Select it at runtime with `-p driver:=<your_imu>`.

`start()` and `recover()` must not block — spawn threads if the transport needs them — and
`stop()` must be idempotent. Report problems through `DriverState` (`config_failed`,
`last_error`) rather than logging them: the node turns them into condition codes. Fill
`raw_acceleration` if the hardware has it, since the saturation and frozen-data checks use it.

## Tests

```bash
colcon test --packages-select idmind_imu && colcon test-result --verbose
```

The suite runs with **no hardware and no BrickDaemon**: `test/fake_brickd.cpp` implements the
TinkerForge wire protocol as a fake daemon on an ephemeral port, so enumeration, getters,
setters and the sensor callback path are all exercised against the real, unmodified bindings.

| Target | What it covers |
|---|---|
| `test_conversions` | Pure unit maths: scaling, field order, covariance construction |
| `test_health` | Condition tracker, recovery policy, debounce, sample validation, calibration rule |
| `test_fake_brickd` | That the fake is convincing to the raw TinkerForge bindings |
| `test_brick_v2_driver` | Driver lifecycle, SI conversion, config convergence, recovery actions, error reporting, clean shutdown |
| `test_imu_node` | The real node end to end: topics, services, live reconfigure, and every condition code raising **and** clearing, including the full recovery ladder |

The fake can inject faults: a silent stream that heals on re-arm, on reconnect or never, an
unanswered enumerate, and an unanswered getter.

Every wait in the suite polls against a hard deadline rather than sleeping, so a regression
surfaces as a fast, clear failure instead of a hang.
