#ROS package idmind_imu

## Description
This package contains the nodes responsible for communication with IMU devices and publishing the information under ROS topics.

## Usage
1. Clone the repository into `<catkin_ws>/src`
2. Clone the `idmind_serial2` package
3. Run `catkin_make`
4. Test the device running `rosrun idmind_imu idmind_<imu_code>.py` or `roslaunch idmind_imu idmind_imu.launch imu:=<imu_code`
5. Use it in your launch files adding


    <include file="$(find idmind_imu)/launch/idmind_imu.launch">
        <arg name="imu" value="<imu_code>" />
    </include>
    
### Supported devices:
##### SparkFun OpenLog_Artemis (with ICM-20948 IMU)
Website: https://www.sparkfun.com/products/16832

Usage:
1. Download and install Arduino IDE. 
2. Install ICM-20948 Library (https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) - Uncomment line 29 of ICM_20948_C.h to enable DMP - and SparkFun Apollo3 Boards (version 1.2.1, because 2.0 did not work as of Feb2021). Select the "Sparkfun Artemis RedBoard ATP".
3. Compile and upload one of the sketch alternatives:    
    1. Compile and Upload Razor_AHRS sketch. The gyro drift is high in presence of magnetic fields. Use imu_filter_madgwick to compensate.
    2. Compile and Upload OpenLog_Artemis sketch. This does not provide orientation, an external library would be required for integration.
    3. Compile and Upload idmind_artemis sketch. This provides accurate orientation but no linear accelerations or angular velocities.

##### SparkFun 9DoF Razor IMU M0 - razor
Website: https://www.sparkfun.com/products/14001

Usage: Upload idmind_imu_stream.ino to device

##### SparkFun MPU6050 on IDMindBoard - mpu6050
Website: https://www.sparkfun.com/products/11028

### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt