# ROS2 package idmind_imu

## Description
This package contains the [ROS2](https://docs.ros.org/en/humble/index.html) nodes for communication with IMU devices.

## Usage
1. Clone the repository into `<ros2_ws>/src`
3. Run `colcon build` under `<ros2_ws>`
4. Test the device running `ros2 run idmind_imu <imu_code>_node.py` or `ros2 launch idmind_imu idmind_<imu_code>.launch.py`
    
### Supported devices:
##### TinkerForge IMU Brick 2.0 (with BNO-055 IMU)
Website: https://www.tinkerforge.com/en/doc/Hardware/Bricks/IMU_V2_Brick.html

Usage:
1. Install [BrickDaemon](https://www.tinkerforge.com/en/doc/Software/Brickd.html#brickd)
2. Install [Python3 bindings](https://www.tinkerforge.com/en/doc/Software/API_Bindings_Python.html#api-bindings-python-install)
3. Check UID (use [BrickViewer](https://www.tinkerforge.com/en/doc/Software/Brickv.html#brickv) or `sudo lsusb -v -d 16d0:063d | grep iSerial`)

### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt