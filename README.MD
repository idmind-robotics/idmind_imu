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
2. Download ICM-20948 Library (https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary) and copy to Arduino/libraries. Uncomment line 29 of ICM_20948_C.h to enable DMP.
3. Run ArduinoIDE and install SparkFun Apollo3 Boards. Under "Tools > Board" select the "Sparkfun Artemis RedBoard ATP".
4. Open, compile and upload OpenLogArtemis.ino.
5. Test the software by opening the Tools > Serial Monitor and writing "f" to get "IDMind OpenLog Artemis" and "r" to receive one reading.

##### SparkFun 9DoF Razor IMU M0 - razor
Website: https://www.sparkfun.com/products/14001

Usage: Upload idmind_imu_stream.ino to device

##### SparkFun MPU6050 on IDMindBoard - mpu6050
Website: https://www.sparkfun.com/products/11028

### Contacts
- Carlos Neves - cneves@idmind.pt
- Henrique Póvoa - hpovoa@idmind.pt