# idmind_imu

ROS 2 wrappers for IMUs on IDMind robots. **ROS 2 Humble / Ubuntu 22.04**, `ament_cmake`,
**C++17**. Also builds and passes its tests on Jazzy.

This repo is its own git repository, consumed elsewhere as a **git submodule**. It normally
sits at `~/ros2_ws/src/idmind_imu`; the workspace root is not a git repo. Keep commits scoped
to this package. Do **not** run `graphify` here.

## Commands

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select idmind_imu
source install/setup.bash
ros2 launch idmind_imu idmind_imu_brick.launch.py

colcon test --packages-select idmind_imu && colcon test-result --all --verbose
```

`--symlink-install` only affects the launch/config/model files now — C++ changes always need a
rebuild.

To run one gtest binary directly, with its full output:

```bash
source install/setup.bash
./build/idmind_imu/test_brick_v2_driver --gtest_filter='*ConfigIsAppliedOnce*'
```

## Architecture

Hardware-agnostic node + pluggable driver layer. Adding an IMU model means adding one driver
file, never another node.

```
include/idmind_imu/
  conversions.hpp        # pure unit + covariance maths. No ROS, no tinkerforge. Unit tested.
  noise_estimator.hpp    # sliding-window per-axis variance. Pure, unit tested.
  imu_node.hpp           # ROS node: params, publishers, diagnostics, watchdog. No hardware code.
  drivers/driver.hpp     # ImuDriver ABC, ImuSample, DriverConfig, DriverState
  drivers/registry.hpp   # name -> factory
  drivers/brick_v2.hpp   # TinkerForge IMU Brick 2.0
src/                     # matching .cpp files, plus main.cpp
test/                    # gtest suites + fake_brickd.{hpp,cpp}
```

The node picks a driver via the `driver` parameter, hands it a `DriverConfig` and an
`on_sample` callback, and converts the resulting `ImuSample` into messages. Drivers emit
**SI units only** — raw counts never reach the node.

Node name is `idmind_imu`; topics derive from the node name, so they are `/idmind_imu/*`.
Three executables (`imu_node`, `imu_brick_node_v2`, `imu_brick_node`) are all built from the
same `main.cpp` — the legacy names are kept so deployed launch files keep working.

Unlike the Python original, `apply_config` takes the **whole** `DriverConfig`, not a partial
patch: the node always knows every field, and the driver decides what actually changed. The
node still only calls it when a hardware-relevant parameter changed.

## TinkerForge bindings (fetched, not vendored)

`CMakeLists.txt` pulls the official C bindings zip at configure time via `FetchContent`,
pinned to `TINKERFORGE_C_BINDINGS_VERSION` + `TINKERFORGE_C_BINDINGS_SHA256`, and compiles
`ip_connection.c` and `brick_imu_v2.c` into a static `tinkerforge_bindings` target.

There is deliberately nothing to `find_package()`: TinkerForge ships **no precompiled library
and no distro package** ("we do not offer a precompiled library"), and their documented
install method is to compile their source into your project. Verified: `apt-cache search
tinkerforge` and the rosdep database are both empty.

- **Never use their `_latest` URL.** It would change the driver under us between builds.
  Upgrading means bumping the version *and* the SHA256 together.
- A clean build **needs network access**. `-DTINKERFORGE_C_BINDINGS_DIR=/path/to/source`
  points at a pre-downloaded `source/` directory for offline builds; the build hard-fails with
  a clear message if any of the four expected files is missing.
- The headers are staged into `${CMAKE_CURRENT_BINARY_DIR}/tinkerforge_include/tinkerforge/`
  so includes read `"tinkerforge/ip_connection.h"` (cpplint requires the directory prefix);
  upstream has them flat in `source/`. They are installed too, because the public
  `brick_v2.hpp` includes them.
- The sources live in the build tree, so no linter sees them and no exclusion list is needed.
  They are compiled with `-w`.

## Threading — read before touching `src/drivers/brick_v2.cpp`

The bindings dispatch **every** callback serially on one internal callback thread. Blocking in
a callback stalls the entire sensor stream. So:

- `on_enumerate` does only cheap work (create device, register callback) and **enqueues** the
  config job to a worker thread.
- All hardware getters/setters run on that dedicated worker thread, never in a callback.
- The initial `ipcon_connect` blocks, so it runs on its own retry thread; the library's
  auto-reconnect only covers reconnection *after* a first successful connect.
- `on_all_data` converts and calls `on_sample`, swallowing exceptions — an exception escaping
  into the C frame above it would be undefined behaviour.

`self._imu`'s C++ equivalent is `device_`, a **`shared_ptr<Device>`** guarded by `mutex_`.
Copy the `shared_ptr` out under the lock and do hardware I/O with the lock released
(`do_apply_config` is the reference example) — that is what keeps the device alive if the
enumerate callback swaps in a replacement mid-call. `Device`'s destructor calls
`imu_v2_destroy`, which is non-blocking, so dropping the last reference from a callback thread
is safe.

`stop()` ordering is load-bearing: set the stop flag, unregister the ipcon callbacks,
`ipcon_disconnect` (which **joins the library's callback thread**, so nothing can be in flight
afterwards), join our own threads, then release the device. The joins are deliberately
unbounded — detaching would leave a thread holding `this` after destruction.

In the node, `on_sample` runs on a driver thread while the watchdog runs on an executor
thread; shared state is under `mutex_`, and publishing happens outside it. `log()` has its own
`log_mutex_` so it can be called from a path that already holds `mutex_`.

## Invariants that exist for a reason — do not "simplify" these

- **The watchdog must never shut down.** Its exception handler logs and continues. The
  pre-refactor node called `shutdown()` there, which cancelled its own timer and set a
  latching flag, leaving the process alive but permanently inert after one transient error.
- **Config is applied on change only**, from `apply_config()` or once after enumeration. The
  old node polled four blocking USB getters every second forever. Do not reintroduce a poll.
  `test_brick_v2_driver.ConfigIsAppliedOnceAndNotPolled` is the regression guard.
- **Covariances come from `conversions::diagonal_covariance`** so off-diagonals are exactly
  zero. The old code used `[x] * 9`, which left garbage cross-terms for `robot_localization`.
- **Covariance is measured from the live signal**, not looked up. Calibration scaling is only
  a *fallback*: it is a 4-value step function, so once the device settles the matrix is
  byte-identical in every message — which is indistinguishable from the hardcoded matrix it
  replaced. `RollingVariance` is what makes it actually move. Do not "simplify" back to the
  lookup.
- **The noise estimate is gated on the robot being still.** A moving IMU's variance measures
  motion, not noise; publishing it tells a filter to distrust the IMU exactly when it is
  manoeuvring. `is_stationary_locked` checks gyro magnitude *and* deviation from the window
  mean acceleration — gyro alone misses straight-line acceleration.
- **Fallback is per axis, never all-or-nothing.** Angular velocity is quantised to 1/16 °/s,
  so a still robot really can report an identical value on one axis for a whole window,
  measuring exactly zero variance there. `blended_covariance` takes the fallback for that axis
  only. An earlier all-or-nothing version silently reverted the whole matrix to the fixed one.
- **Variance is computed in two passes, not from running sums of squares.** Linear
  acceleration has a mean near 9.81 and noise near 1e-3; `E[x^2] - E[x]^2` loses it entirely
  to cancellation. `test_noise_estimator` pins this case.
- **Orientation variance wraps.** Euler angles use a circular mean and wrapped deviations, or
  a yaw near +/- pi reports ~9.87 rad^2 instead of ~1e-6.
- **`orientation_covariance[0] = -1`** when fusion mode is 0 — the `sensor_msgs/Imu` "no
  orientation" signal. Otherwise variance scales with system calibration, and fusion mode 2
  inflates yaw (relative heading, drifts without the magnetometer).
- **LEDs are two independent booleans** (`are_leds_on`, `is_status_led_enabled`), read and
  compared separately. The old code OR-ed them, which could never converge upward.
- **Do not add a `tinkerforge` dependency to `package.xml`.** There is no rosdep rule for it;
  the bindings are fetched by CMake precisely so no such dependency is needed.

## The BNO-055 `sys` calibration trap

The `sys` calibration level only rises when the **magnetometer** is in use. In
`imu_fusion_mode: 2` (the shipped default — fusion without magnetometer) it stays at 0
forever, so anything keyed on `sys` is pinned at the worst-case ×100 factor permanently. That
is why the calibration-scaled covariance looked like a fixed matrix in the field, and why the
live measurement is the primary source rather than the scaling. Diagnosing this is one
command: `ros2 topic echo /idmind_imu/calibration` and watch the first element.

## Units (IMU Brick 2.0 — v2, not v1)

The v1 device has **different** units; ignore the v1 docs. Verified v2 raw units:
acceleration / linear_acceleration / gravity_vector `1 cm/s²` (÷100); magnetic_field
`1/16 µT` (÷16e6 → T); angular_velocity `1/16 °/s`; euler_angle `1/16 °`, ordered
**(heading, roll, pitch)**; quaternion ÷16383, ordered **(w, x, y, z)** — ROS wants
(x, y, z, w); temperature already °C. Calibration byte: bits 0-1 mag, 2-3 acc, 4-5 gyro,
6-7 sys, each 0–3.

## Testing

`test/fake_brickd.cpp` is a fake BrickDaemon speaking the real TinkerForge wire protocol on an
ephemeral port, so the whole suite runs with **no hardware and no brickd**. Key traps it
handles: `GET_IDENTITY` (255) must be answered or `device_check_validity()` poisons every
later getter with `WRONG_DEVICE_TYPE`; unsolicited packets need sequence number 0; the
enumerate reply must be exactly 34 bytes and the all-data callback exactly 54; the idle socket
must never be closed (the disconnect probe needs no reply); and a >32-bit base58 UID must be
folded to 32 bits with the **same bit shuffle the bindings use**, or reply headers will not
match the client's device table.

**Callback lifetime.** Anything a TinkerForge callback writes into must outlive the callback
thread, which only stops when `ipcon_destroy`/`ipcon_disconnect` runs. A capture object
declared as a local in a gtest body is destroyed when the body returns, i.e. *before*
`TearDown` tears the connection down — a late packet then writes through a dangling pointer.
`FakeBrickdTest::make_slot()` exists for exactly this; ThreadSanitizer caught the original
version of this bug.

**Under ThreadSanitizer** the suite is not silent, and the noise has two known, benign
sources. Classify a report by the `#0` frame of each stack — deeper frames are only callers.

1. *The TinkerForge bindings.* Most reports have `#0` inside the fetched `ip_connection.c`
   (`ipcon_handle_response` vs `ipcon_send_request`, `ipcon_disconnect_unlocked` vs
   `ipcon_receive_loop`). Upstream, in code we keep byte-identical. Not ours to fix, but note
   it does ship compiled into our library.
2. *Condition-variable predicates.* Four reports have `#0` in our own code — `Slot::get`,
   `SampleQueue::pop`, and `stopping_` in `connect_loop`'s `wait_for` predicate vs `stop()`.
   In every one, TSan's own output shows **both** threads holding the same mutex
   (`mutexes: write M…` on both stacks), and the source confirms it. A race needs at least one
   access without the common lock, so these are not defects; TSan does not fully model the
   atomic unlock-and-wait of `pthread_cond_wait`, and every one of these accesses is inside or
   immediately after a `condition_variable` predicate.

TSan did catch one real bug: `FakeBrickd::stop()` used to write `server_fd_` while
`accept_loop()` read it. The listening socket is now closed only after the accept thread has
been joined. If you touch that teardown, re-run TSan.

TSan needs ASLR turned down in a container: `sysctl -w vm.mmap_rnd_bits=28` (needs
`--privileged`) and `setarch "$(uname -m)" -R ./test_binary`.

Tests must poll with a deadline, never a bare sleep. If you start a node with
`ros2 run` from a script, use `setsid` and kill the process **group** — killing the `ros2`
wrapper alone leaves an orphan node publishing to `/diagnostics`, which silently corrupts any
rate measurement.

## Conventions

- Parameters declared with `ParameterDescriptor` (see the `describe()` helper), read
  immediately via `declare_parameter<T>()`'s return value.
- Node exposes a `ready` `Trigger` service, a `standby` `SetBool` service (suspends data-topic
  publishing without stopping the driver — `on_sample` still runs its bookkeeping), an
  `add_on_set_parameters_callback` handler, and a `timer` heartbeat topic. Use
  `log(msg, "warn")` (duplicate-suppressing) rather than `RCLCPP_*` directly in node code.
  Drivers use `RCLCPP_*` on their own logger.
- Diagnostics go through `diagnostic_updater::Updater`, which publishes `/diagnostics` on its
  own 1 Hz timer. Do **not** call `force_update()` from the watchdog — that would republish
  at `control_freq`.
- `control_freq` changes **recreate** the watchdog timer (`restart_watchdog_timer`); rclcpp
  has no period setter.
- Lint: `ament_lint_auto` with the common set (cpplint, uncrustify, lint_cmake, xmllint), max
  line length **100**. `ament_cmake_cppcheck` is excluded — it chokes on the fetched third-party C.
  Uncrustify is picky about ternaries (operator at end of line), brace-init spacing
  (`struct pollfd pfd {…}`), and continuation indentation; run it before assuming a failure is
  real.
