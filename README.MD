# idmind_imu

## Description
ROS2 package providing drivers for IMU devices used on IDMind robots. Currently supports the **TinkerForge IMU Brick 2.0** (BNO-055 sensor), which provides orientation, angular velocity, linear acceleration, magnetic field, and temperature data.

Tested on **Ubuntu 22.04** with **ROS2 Humble**.

### Prerequisites (TinkerForge IMU Brick)
1. Install [BrickDaemon](https://www.tinkerforge.com/en/doc/Software/Brickd.html#brickd)
2. Install [Python3 TinkerForge bindings](https://www.tinkerforge.com/en/doc/Software/API_Bindings_Python.html#api-bindings-python-install)
3. Identify the device UID (use [BrickViewer](https://www.tinkerforge.com/en/doc/Software/Brickv.html#brickv) or `sudo lsusb -v -d 16d0:063d | grep iSerial`)

---

## Launch Files

- **idmind_imu_brick.launch.py**  
  Starts the TinkerForge IMU Brick 2.0 driver node.

```bash
ros2 launch idmind_imu idmind_imu_brick.launch.py
```

---

## Nodes

### imu_brick_node
Driver node for the TinkerForge IMU Brick 2.0 (BNO-055).

Responsibilities:
- Connects to the BrickDaemon and opens the IMU Brick device
- Reads orientation (quaternion), angular velocity, linear acceleration, magnetic field, and temperature
- Publishes sensor data as standard ROS messages
- Handles reconnection on timeout

#### Parameters
| Name | Description | Default |
|------|-------------|---------|
| `control_freq` | Main loop frequency (Hz) | `20.0` |
| `imu_freq` | IMU data stream frequency (Hz) | `20.0` |
| `imu_frame` | TF frame name for IMU data | `"imu"` |
| `imu_leds` | Enable the status LEDs on the IMU Brick | `False` |
| `imu_fusion_mode` | BNO-055 fusion mode (see TinkerForge docs) | `2` |
| `timeout` | Timeout before reporting a connection error (s) | `1.0` |

#### Publishers
| Topic | Type | Description |
|-------|------|-------------|
| `imu_brick_node/imu` | `sensor_msgs/msg/Imu` | Orientation, angular velocity, and linear acceleration |
| `imu_brick_node/temperature` | `sensor_msgs/msg/Temperature` | IMU chip temperature |
| `imu_brick_node/magnetic_field` | `sensor_msgs/msg/MagneticField` | Magnetometer reading |
| `imu_brick_node/euler` | `std_msgs/msg/Float32` | Yaw angle in degrees (Euler Z) |
| `imu_brick_node/timer` | `std_msgs/msg/Float32` | Loop timing diagnostic |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Node diagnostic status |

#### Services
| Name | Type | Description |
|------|------|-------------|
| `imu_brick_node/ready` | `std_srvs/srv/Trigger` | Returns whether the node is ready |
