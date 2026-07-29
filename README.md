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
| `timeout` | double | `1.0` | Seconds without data before diagnostics report an error |
| `auto_reconnect` | bool | `true` | Let the driver reconnect automatically |
| `acceleration_source` | string | `linear` | `linear` (gravity removed) or `raw` (gravity included) |
| `orientation_stddev` | double | `0.01` | Base orientation stddev at full calibration |
| `temperature_stddev` | double | `0.0` | Temperature stddev (°C); `0` (the default) publishes variance `0` = unknown |
| `angular_velocity_stddev` | double[3] | `[0.005236]*3` | Per-axis gyro stddev (rad/s), scaled by **gyro** calibration |
| `linear_acceleration_stddev` | double[3] | `[0.1, 0.1, 0.2236]` | Per-axis accel stddev (m/s²), scaled by **acc** calibration |
| `magnetic_field_stddev` | double[3] | `[6e-7]*3` | Per-axis mag stddev (T), scaled by **mag** calibration |

**Fusion modes:** `0` off (raw data — orientation is meaningless), `1` on with magnetometer
(absolute heading), `2` on without magnetometer (**relative yaw that drifts** — the default),
`3` on without fast magnetometer calibration.

**`acceleration_source`** decides what fills `Imu.linear_acceleration`. The default `linear`
preserves historical behaviour and reports gravity-compensated acceleration. Consumers such
as `robot_localization` generally expect gravity to be *included*; use `raw` for those.

### Published topics

| Topic | Type | Notes |
|---|---|---|
| `~/imu` | `sensor_msgs/Imu` | Orientation, angular velocity (rad/s), linear acceleration (m/s²) |
| `~/temperature` | `sensor_msgs/Temperature` | Sensor temperature (°C); `variance` from `temperature_stddev` |
| `~/magnetic_field` | `sensor_msgs/MagneticField` | Tesla |
| `~/euler` | `std_msgs/Float32` | **Yaw only, in radians** |
| `~/gravity` | `geometry_msgs/Vector3Stamped` | Gravity vector (m/s²) |
| `~/calibration` | `std_msgs/UInt8MultiArray` | `[sys, gyro, acc, mag]`, each 0–3 |
| `~/timer` | `std_msgs/Float32` | Watchdog heartbeat: measured loop period (s) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Published at 1 Hz by `diagnostic_updater` |

### Services

| Name | Type | Description |
|---|---|---|
| `~/ready` | `std_srvs/Trigger` | Reports whether the node finished initialising |

### Covariances

- Off-diagonal terms are always exactly zero — the matrices are strictly diagonal.
- **No covariance is a fixed matrix.** Each of the four scales with the BNO-055 calibration
  level for *that* sensor — orientation with `sys`, angular velocity with `gyro`, linear
  acceleration with `acc`, magnetic field with `mag` — using the factor table below. Expect
  every covariance to start large and shrink as the device calibrates.

  | Calibration level | Variance multiplier |
  |---|---|
  | 3 (fully calibrated) | ×1 |
  | 2 | ×4 |
  | 1 | ×25 |
  | 0 (uncalibrated) | ×100 |

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

Three tasks are published: **Connection** (transport reachable, device enumerated),
**Data flow** (observed rate and age of the last sample; `ERROR` past `timeout`), and
**Calibration** (`WARN` while any of sys/gyro/acc/mag is 0).

---

## Adding a new IMU driver

1. Add `include/idmind_imu/drivers/<your_imu>.hpp` and `src/drivers/<your_imu>.cpp` with a
   class deriving from `ImuDriver` (`include/idmind_imu/drivers/driver.hpp`) implementing
   `start()`, `stop()`, `apply_config()` and `state()`.
2. Convert readings to SI units and report them by calling the sample callback with an
   `ImuSample`. Put reusable unit maths in `src/conversions.cpp` and unit test it there.
3. Add one line to the table in `src/drivers/registry.cpp`, and the new `.cpp` to the library
   in `CMakeLists.txt`.
4. Select it at runtime with `-p driver:=<your_imu>`.

`start()` must not block — spawn threads if the transport needs them — and `stop()` must be
idempotent.

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
| `test_fake_brickd` | That the fake is convincing to the raw TinkerForge bindings |
| `test_brick_v2_driver` | Driver lifecycle, SI conversion, config convergence, clean shutdown |
| `test_imu_node` | The real node end to end: topics, diagnostics, service, live reconfigure |

Every wait in the suite polls against a hard deadline rather than sleeping, so a regression
surfaces as a fast, clear failure instead of a hang.
