# idmind_imu

ROS 2 wrappers for IMUs on IDMind robots. **ROS 2 Humble / Ubuntu 22.04**, `ament_python`.

This repo is its own git repository, consumed elsewhere as a **git submodule**. It normally
sits at `~/ros2_ws/src/idmind_imu`; the workspace root is not a git repo. Keep commits scoped
to this package. Do **not** run `graphify` here.

## Commands

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select idmind_imu
source install/setup.bash
ros2 launch idmind_imu idmind_imu_brick.launch.py

colcon test --packages-select idmind_imu && colcon test-result --verbose
```

`--symlink-install` matters: without it, edits to Python files require a rebuild.

To run pytest directly you must **append** to `PYTHONPATH`, not replace it — `PYTHONPATH=.`
clobbers the ROS paths and every `rclpy` import fails:

```bash
source /opt/ros/humble/setup.bash
cd src/idmind_imu && PYTHONPATH=".:$PYTHONPATH" python3 -m pytest test/ -q
```

## Architecture

Hardware-agnostic node + pluggable driver layer. Adding an IMU model means adding one driver
file, never another node.

```
idmind_imu/
  imu_node.py       # ROS node: params, publishers, diagnostics, watchdog. No hardware code.
  conversions.py    # pure unit + covariance maths. No ROS, no tinkerforge. Unit tested.
  drivers/
    base.py         # ImuDriver ABC, ImuSample, DriverState
    registry.py     # name -> class; drivers imported lazily
    brick_v2.py     # TinkerForge IMU Brick 2.0
```

The node picks a driver via the `driver` parameter, hands it a config dict and an `on_sample`
callback, and converts the resulting `ImuSample` into messages. Drivers emit **SI units
only** — raw counts never reach the node.

Node name is `idmind_imu`; topics derive from the node name, so they are `/idmind_imu/*`.
Three console scripts (`imu_node`, `imu_brick_node_v2`, `imu_brick_node`) all run this same
node — the legacy names are kept so deployed launch files keep working.

## Threading — read before touching `drivers/brick_v2.py`

The `tinkerforge` bindings dispatch **every** callback serially on one internal
`Callback-Processor` thread. Blocking in a callback stalls the entire sensor stream. So:

- `_enumerate_callback` does only cheap work and **enqueues** config to a worker thread.
- All hardware getters/setters run on that dedicated worker thread, never in a callback.
- The initial `ipcon.connect()` blocks, so it runs on its own retry thread; the library's
  auto-reconnect only covers reconnection *after* a first successful connect.
- `_all_data_callback` converts and calls `on_sample`, swallowing exceptions so a bad
  consumer cannot kill the callback thread.

Guard `self._imu` / `self._state` with `self._lock`, and never hold the lock across a
blocking hardware call — copy the reference out, then do I/O.

In the node, `on_sample` runs on a driver thread while the watchdog runs on an executor
thread; shared state is under `self._lock`, and publishing happens outside it.

## Invariants that exist for a reason — do not "simplify" these

- **The watchdog must never shut down.** Its exception handler logs and continues. The
  pre-refactor node called `shutdown()` there, which cancelled its own timer and set a
  latching flag, leaving the process alive but permanently inert after one transient error.
- **Config is applied on change only**, from `apply_config()` or once after enumeration. The
  old node polled four blocking USB getters every second forever. Do not reintroduce a poll.
- **Covariances come from `conversions.diagonal_covariance`** so off-diagonals are exactly
  zero. The old code used `[x] * 9`, which left garbage cross-terms for `robot_localization`.
- **`orientation_covariance[0] = -1`** when fusion mode is 0 — the `sensor_msgs/Imu` "no
  orientation" signal. Otherwise variance scales with system calibration, and fusion mode 2
  inflates yaw (relative heading, drifts without the magnetometer).
- **LEDs are two independent booleans** (`are_leds_on`, `is_status_led_enabled`), read and
  compared separately. The old code OR-ed them, which could never converge upward.
- **Do not add `tinkerforge` to `package.xml`.** `python3-tinkerforge` has no rosdep rule, so
  declaring it breaks `rosdep install`. It is a documented manual prerequisite.

## Units (IMU Brick 2.0 — v2, not v1)

The v1 device has **different** units; ignore the v1 docs. Verified v2 raw units:
acceleration / linear_acceleration / gravity_vector `1 cm/s²` (÷100); magnetic_field
`1/16 µT` (÷16e6 → T); angular_velocity `1/16 °/s`; euler_angle `1/16 °`, ordered
**(heading, roll, pitch)**; quaternion ÷16383, ordered **(w, x, y, z)** — ROS wants
(x, y, z, w); temperature already °C. Calibration byte: bits 0-1 mag, 2-3 acc, 4-5 gyro,
6-7 sys, each 0–3.

## Testing

`test/fake_brickd.py` is a fake BrickDaemon speaking the real TinkerForge wire protocol on an
ephemeral port, so the whole suite runs with **no hardware and no brickd**. Key traps it
handles: `GET_IDENTITY` (255) must be answered or `check_validity()` poisons every later
getter with `WRONG_DEVICE_TYPE`; unsolicited packets need sequence number 0; the enumerate
reply must be exactly 34 bytes and `CALLBACK_ALL_DATA` exactly 54; and the idle socket must
never be closed (the disconnect probe needs no reply).

Tests must poll with a deadline, never a bare sleep. If you start a node with
`subprocess.Popen(["ros2", "run", ...])`, use `start_new_session=True` and kill the process
**group** — `terminate()` kills only the `ros2` wrapper and leaves an orphan node publishing
to `/diagnostics`, which silently corrupts any rate measurement.

## Conventions

- Parameters declared with `ParameterDescriptor(description=...)`, read immediately via
  `.get_parameter_value().<type>_value`.
- Node exposes a `ready` `Trigger` service, an `add_on_set_parameters_callback` handler, and
  a `timer` heartbeat topic. Use `self.log(msg, alert=...)` (duplicate-suppressing) rather
  than `get_logger()` directly.
- Diagnostics go through `diagnostic_updater.Updater`, which publishes `/diagnostics` on its
  own 1 Hz timer. Do **not** call `force_update()` from the watchdog — that would republish
  at `control_freq`.
- Lint: `ament_flake8` + `ament_pep257`, max line length **99**, both run by `colcon test`.
  Note ament ignores `D212` but **enforces `D213`**: a multi-line docstring puts its summary
  on the *second* line.
