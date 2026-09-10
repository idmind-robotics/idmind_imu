# ros2_ws

IDMind robotics ROS 2 workspace. **ROS 2 Humble on Ubuntu 22.04.**

The workspace root is *not* a git repository — each package under `src/` is its own repo and
is consumed elsewhere as a git submodule. Never `git init` at the root, and keep changes
scoped to a single package per commit.

## Packages

| Package | Build type | State |
|---|---|---|
| `idmind_imu` | `ament_python` | Working driver for the TinkerForge IMU Brick 2.0 |
| `idmind_docking` | `ament_python` | Skeleton — `dock_detection_node` is a stub `print()` |
| `idmind_gazebo` | `ament_cmake` | Simulation assets: worlds, meshes, models, xacros |

## Commands

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select idmind_imu
source install/setup.bash

ros2 launch idmind_imu idmind_imu_brick.launch.py

# Lint (ament defaults: flake8 max-line-length 99)
colcon test --packages-select idmind_imu && colcon test-result --verbose
```

`--symlink-install` matters here: without it, edits to Python nodes require a rebuild.

## idmind_imu

Driver for the TinkerForge IMU Brick 2.0 (BNO-055), reached over BrickDaemon TCP at
`localhost:4223`. Requires `brickd` running plus the `tinkerforge` Python bindings. `brickd`
is external; `tinkerforge` is not in rosdep, so it is declared in `requirements.txt`
(`pip install -r requirements.txt`) rather than `package.xml`.

Two nodes ship as console scripts. **`imu_brick_node_v2` is the live one** — it is what the
launch file starts. `imu_brick_node` is the legacy version, unused by any launch file, kept
deliberately for now; do not "helpfully" delete it or sync it with v2.

### Runtime interface

The launch file names the node `idmind_imu` and namespaces it under the `robot_name` launch
argument (default empty). Topic names are built from `self.get_name() + "/"`, so the real
topics are `/idmind_imu/*`, or `/<robot_name>/idmind_imu/*` when `robot_name` is set.

`README.MD` was rewritten to match the source (v2, namespaced, full topic/service/parameter
tables). If in doubt, still trust the source.

- Publishers: `imu` (`sensor_msgs/Imu`), `temperature`, `magnetic_field`, `euler`
  (`std_msgs/Float32`, yaw in **radians**), `gravity` (`geometry_msgs/Vector3Stamped`),
  `calibration` (`std_msgs/UInt8MultiArray`, `[sys, gyro, acc, mag]` each 0–3), `timer`,
  and `/diagnostics`.
- Services: `ready` (`std_srvs/Trigger`); `standby` (`std_srvs/SetBool`) — `data: true`
  suspends publishing on the data topics (samples are still received, `publish_imu` just
  returns early after refreshing `last_imu_msg`), `data: false` resumes.
- Parameters: `control_freq`, `imu_freq`, `imu_frame`, `imu_leds`, `imu_fusion_mode`,
  `timeout`, `auto_reconnect` — all dynamically reconfigurable via `update_parameters`.
  `config/idmind_imu.yaml` overrides `imu_frame` to `"imu2"` (code default is `"imu"`).

The package installs meshes and xacros under `models/`, but nothing in the package loads them
and no TF is broadcast for `imu_frame` — the transform must come from an external robot
description.

### Threading model — read before touching `imu_brick_node_v2.py`

`MultiThreadedExecutor(num_threads=6)` with three `ReentrantCallbackGroup`s. Three kinds of
thread touch node state:

1. **`main_loop`** — a 20 Hz ROS timer supervising the link: connect, enumerate, watchdog the
   data timeout, poll config.
2. **A daemon thread** for the initial `ipcon.connect()`, which blocks.
3. **TinkerForge-owned threads** delivering `CALLBACK_CONNECTED` / `CALLBACK_DISCONNECTED` /
   `CALLBACK_ENUMERATE` and, crucially, `BrickIMUV2.CALLBACK_ALL_DATA` — `publish_imu` runs
   here and publishes directly from a non-ROS thread.

`self.imu` and `self.imu_uid` must only be read or written under `self._imu_lock`. The
established pattern is to copy the reference out under the lock and do hardware I/O outside
it (see `update_config`). Device discovery matches `device_identifier == 18`.

### Known defects (recorded, not yet fixed)

The user is aware of these and will schedule the work — don't fix them opportunistically.
Full write-up with line references: `~/.claude/plans/quiet-launching-nygaard.md`.

- `main_loop`'s exception handler calls `self.shutdown()`, which cancels the main timer and
  sets `_shutdown_in_progress` with no path back. A single transient exception leaves the
  process alive but permanently inert. **This is the one that bites in the field.**
- `angular_velocity_covariance` and `magnetic_field_covariance` are built with `[x] * 9`, so
  the off-diagonal terms carry nonzero garbage that downstream filters will consume.
- `orientation_covariance` is a fixed `1e-4` diagonal regardless of fusion mode or reported
  calibration; `orientation_covariance[0] = -1` is never used to signal an invalid quaternion.
- `enumerate_callback` builds a new `BrickIMUV2` and a new timer on every enumeration and only
  `cancel()`s the old timer rather than `destroy_timer()`ing it.
- `update_config` does four blocking USB round-trips every second regardless of whether
  anything changed, and its LED read-back ORs two independent LEDs so it cannot converge.

## Conventions

- Node classes follow the IDMind pattern: a `ready` `Trigger` service, an
  `add_on_set_parameters_callback` handler, a `/diagnostics` publisher, a `timer` heartbeat
  topic, and a `self.log(msg, level, alert=...)` wrapper with duplicate suppression rather
  than direct `get_logger()` calls. Match this in new nodes.
- Parameters are declared with `ParameterDescriptor(description=...)` and read immediately via
  `.get_parameter_value().<type>_value`.
- Tests are ament lint boilerplate only (`test_flake8`, `test_pep257`, `test_copyright`) —
  there is no functional test suite to lean on. Verify changes against real hardware or by
  running the node and inspecting topics. `test_flake8` / `test_pep257` exclude the frozen
  `imu_brick_node.py`; keep `imu_brick_node_v2.py` and everything else lint-clean.
- Do **not** run `graphify` on this workspace.
