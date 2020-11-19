/************************************************************
  IDMind IMU for use with Sparkfun 9DoF Razor M0
  Enable only the Accelerometer and Gyroscope
  Use the DMP
  Publish Orientation Quaternion regularly

  Development environment specifics:
  Arduino IDE 1.8.5setSensors
  SparkFun 9DoF Razor IMU M0

  Created: 20 Nov 2018
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB
#define STATUS_LED_PIN 13  // Pin number of status LED

MPU9250_DMP imu;
int inByte = 0;         // incoming serial byte
int counter = 0;
bool not_connected = true;
float rad = 3.14159265359 / 180.0;
float g = 9.80057;

void setup()
{
  digitalWrite(STATUS_LED_PIN, LOW);
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_SEND_RAW_ACCEL | //Send Raw accelerations values
               DMP_FEATURE_SEND_CAL_GYRO | //Send calibrated gyro values
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               100); // Set DMP FIFO update rate to 50 Hz, between 4 and 200 Hz
   
   digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop()
{  
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      //imu.computeEulerAngles();
      //printIMUData();
      printAll();
    }
  }
  /*if (SerialPort.available() > 0) {    
    inByte = SerialPort.read();
    switch(inByte){
      case 113: // 'q'
        printQuat();
        break;
      case 103: // 'g'
        printGyro();
        break;
      case 97:  // 'a'
        printAcc();
        break;        
      case 42:  //'*'
        printAll();
        break;
      case 108: // 'l'
        digitalWrite(STATUS_LED_PIN, LOW);
        break;
    }
  }*/

  delay(20);    

}

void printQuat(void)
{
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);
  SerialPort.println("Q: " + String(q0, 4) + " " +
                     String(q1, 4) + " " + String(q2, 4) +
                     " " + String(q3, 4));  
}

void printGyro(void){
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  SerialPort.println("G: " + String(gyroX, 4) + " " +
                     String(gyroY, 4) + " " + String(gyroZ, 4));  
}

void printAcc(void){
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  SerialPort.println("A: " + String(accelX, 4) + " " +
                     String(accelY, 4) + " " + String(accelZ, 4));    
}

void printAll(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.


  // Output unit: g, ROS unit: m/s^2
  float accelX = imu.calcAccel(imu.ax) * g;
  float accelY = imu.calcAccel(imu.ay) * g;
  float accelZ = imu.calcAccel(imu.az) * g;
  
  // Output unit: dps (degree per second), ROS unit: rad/s
  float gyroX = imu.calcGyro(imu.gx) * rad;
  float gyroY = imu.calcGyro(imu.gy) * rad;
  float gyroZ = imu.calcGyro(imu.gz) * rad;

  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);

//
//  float magX = imu.calcMag(imu.mx); // magX is x-axis magnetic field in uT
//  float magY = imu.calcMag(imu.my); // magY is y-axis magnetic field in uT
//  float magZ = imu.calcMag(imu.mz); // magZ is z-axis magnetic field in uT
//  SerialPort.println("M: " + String(magX) + " " + String(magY) + " " + String(magZ));
  
  SerialPort.println("Q: " + String(q0, 4) + " " + String(q1, 4) + " " + String(q2, 4) +" " + String(q3, 4) + " | " +
                    "A: " + String(accelX, 4) + " " + String(accelY, 4) + " " + String(accelZ, 4)  + " | " +
                    "G: " + String(gyroX, 4) + " " + String(gyroY, 4) + " " + String(gyroZ, 4));
                
}
