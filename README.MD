#ROS package idmind_imu

## Description
This package contains the nodes responsible for communication with IMU devices and publishing the information under ROS topics.

## Usage
1. Clone the repository into `<catkin_ws>/src`
2. Clone the `idmind_serial2` package
3. Run `catkin_make`
4. Test the device running `rosrun idmind_imu idmind_<imu_code>.py` or `roslaunch idmind_imu idmind_imu.launch imu:=<imu_code>`
5. Use it in your launch files adding


    <include file="$(find idmind_imu)/launch/idmind_imu.launch">
        <arg name="imu" value="<imu_code>" />
    </include>
    
### Supported devices:
##### TinkerForge IMU Brick 2.0 (with BNO-055 IMU)
Website: https://www.tinkerforge.com/en/doc/Hardware/Bricks/IMU_V2_Brick.html

Usage:
1. Install [BrickDaemon](https://www.tinkerforge.com/en/doc/Software/Brickd.html#brickd)
2. Install [Python bindings](https://www.tinkerforge.com/en/doc/Software/API_Bindings_Python.html#api-bindings-python-install)
3. Check UID (use [BrickViewer](https://www.tinkerforge.com/en/doc/Software/Brickv.html#brickv) or `sudo lsusb -v -d 16d0:063d | grep iSerial`)


##### SparkFun OpenLog_Artemis (with ICM-20948 IMU)
Website: https://www.sparkfun.com/products/16832

NOTE: Due to some incompatibility, the firmware developed by IDMind does not work for new flashes. Please update.

Usage:
1. Run Firmware Uploader (python3 firmwares/OpenLogArtemis/Artemis-Firmware-Upload-GUI/tools/artemis_firmware_uploader_gui.py) and choose firmwares/OpenLogArtemis/OpenLog_Artemis-V10-v21.bin
3. Use Serial Monitor from ArduinoIDE (or another serial communication program) to configure device:
3.1. Send any character to enter menu
3.2. Send '3' to enter "Configure IMU Logging", '12' to Enable DMP. '15' to enable Accelerometer, '16' to enable Gyroscope and '17' for Compass.
3.3. Wait or choose 'x' to leave menu.
3.4 Send '1' to enter "Configure Terminal Output". Send '1' to disable SD logging, '3' to choose Baudrate and '4' to choose frequency. You should provide those values to the IMU node, although node and IMU frequency can be adjusted by Dynamic Parameters.
3.5. Wait or choose 'x' to leave menu.
4. When running the node, choose <imu_code> = "artemisv2". If an error appears, rebooting once seems to solve the issue.

##### SparkFun 9DoF Razor IMU M0 - razor
Website: https://www.sparkfun.com/products/14001

Usage: Upload firmwares/razor/idmind_imu_stream.ino to device

##### SparkFun MPU6050 on IDMindBoard - mpu6050
Website: https://www.sparkfun.com/products/11028

### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt