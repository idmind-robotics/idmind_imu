# idmind_imu

## Description

ROS 2 package providing the driver for IMU devices used on IDMind robots. It currently
supports the **TinkerForge IMU Brick 2.0** (Bosch BNO-055 sensor), exposing orientation,
angular velocity, linear acceleration, magnetic field, gravity, temperature and calibration
state.

Tested on **Ubuntu 22.04 / ROS 2 Humble**. Build type: `ament_python`.

### Prerequisites

1. Install [BrickDaemon](https://www.tinkerforge.com/en/doc/Software/Brickd.html#brickd) and
   make sure it is running (default TCP endpoint `localhost:4223`).
2. Install the Python dependencies that rosdep does not cover:
   ```bash
   pip install -r requirements.txt   # tinkerforge bindings
   ```
   The remaining dependencies are declared in `package.xml` and install with
   `rosdep install --from-paths src -y`.
3. Identify the device UID if needed (via
   [BrickViewer](https://www.tinkerforge.com/en/doc/Software/Brickv.html#brickv) or
   `sudo lsusb -v -d 16d0:063d | grep iSerial`). The node auto-discovers a single IMU Brick,
   so this is only for multi-device setups.

---

## Usage

```bash
ros2 launch idmind_imu idmind_imu_brick.launch.py
```

### Launch arguments

| Name | Default | Description |
|---|---|---|
| `robot_name` | `""` | Namespace for the node; topics become `/<robot_name>/idmind_imu/*` |
| `simulation` | `False` | Reserved for a uniform bringup API; not consumed by the node yet |
| `imu_cfg` | installed `config/idmind_imu.yaml` | Parameter file passed to the node |

Parameter defaults live in `config/idmind_imu.yaml`. Treat that file as read-only defaults —
override it with your own copy rather than editing it in place.

---

## Nodes

Two console scripts are installed. **`imu_brick_node_v2` is the maintained node** and the one
the launch file starts. `imu_brick_node` is a frozen legacy version, used by no launch file
and kept only for reference.

### imu_brick_node_v2

Node name: `idmind_imu`. All topic and service names below are relative to the node, i.e.
`/idmind_imu/<name>` (or `/<robot_name>/idmind_imu/<name>` when `robot_name` is set).

Responsibilities:

- Connect to BrickDaemon (retrying in the background) and discover the IMU Brick 2.0.
- Stream sensor data as standard ROS messages at `imu_freq`.
- Apply parameter changes to the device and watchdog the data stream, reporting status on
  `/diagnostics`.
- Suspend/resume publishing on request (`~/standby`).

#### Published topics

| Topic | Type | Description |
|---|---|---|
| `~/imu` | `sensor_msgs/msg/Imu` | Orientation (quaternion), angular velocity (rad/s), linear acceleration (m/s²) |
| `~/temperature` | `sensor_msgs/msg/Temperature` | IMU chip temperature (°C) |
| `~/magnetic_field` | `sensor_msgs/msg/MagneticField` | Magnetometer reading (T) |
| `~/euler` | `std_msgs/msg/Float32` | Yaw angle in **radians** (Euler Z) |
| `~/gravity` | `geometry_msgs/msg/Vector3Stamped` | Gravity vector (m/s²) |
| `~/calibration` | `std_msgs/msg/UInt8MultiArray` | `[sys, gyro, acc, mag]`, each `0`–`3` |
| `~/timer` | `std_msgs/msg/Float32` | Main-loop heartbeat |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Connection / data / calibration status |

> The `euler`, `timer` and `calibration` topics use `std_msgs` primitives for historical
> reasons. New consumers should read orientation from `~/imu` and calibration from
> `/diagnostics`.

#### Services

| Name | Type | Description |
|---|---|---|
| `~/ready` | `std_srvs/srv/Trigger` | Reports whether the node finished initialising |
| `~/standby` | `std_srvs/srv/SetBool` | `data: true` suspends publishing on the data topics; `data: false` resumes. Samples keep arriving from the device while suspended, so the connection watchdog stays quiet. |

#### Parameters

| Name | Type | Default | Description |
|---|---|---|---|
| `control_freq` | `double` | `20.0` | Main-loop / watchdog frequency (Hz) |
| `imu_freq` | `double` | `20.0` | IMU data-stream frequency (Hz) |
| `imu_frame` | `string` | `"imu"` | `frame_id` on published messages (`config/idmind_imu.yaml` sets `"imu2"`) |
| `imu_leds` | `bool` | `false` | Enable the IMU Brick status LEDs |
| `imu_fusion_mode` | `int` | `2` | BNO-055 sensor-fusion mode (see TinkerForge docs) |
| `timeout` | `double` | `1.0` | Seconds without data before an `ERROR` diagnostic is raised |
| `auto_reconnect` | `bool` | `false` | Enable the TinkerForge library's auto-reconnect |

All parameters are dynamically reconfigurable (`ros2 param set /idmind_imu <name> <value>`).

---

## Build & test

```bash
colcon build --symlink-install --packages-select idmind_imu
colcon test --packages-select idmind_imu && colcon test-result --all --verbose
```

Tests are `ament_flake8` / `ament_pep257` / `ament_copyright` lint checks. The legacy
`imu_brick_node.py` is excluded from the style linters (see `test/`).
