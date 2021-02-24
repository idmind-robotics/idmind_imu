/****************************************************************
 * IDMinds Driver for OpenLog Artemis, focusing in ICM 20948 IMU
 *  Board: SparkFun RedBoard Artemis ATP
 *  
 ***************************************************************/

// OLA Specifics:
const byte PIN_STAT_LED = 19;
const byte PIN_IMU_POWER = 27;
const byte PIN_PWR_LED = 29;
const byte PIN_IMU_INT = 37;
const byte PIN_IMU_CHIP_SELECT = 44;

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU (Install manually from github)

#define CS_PIN PIN_IMU_CHIP_SELECT // Which pin you connect CS to. Used only when "USE_SPI" is defined. OLA uses pin 44.


ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object

int incomingByte = 0;

void setup() {
  setPowerLED(HIGH);
  SPI.begin();  // SPI Port is used to communicate with the IMU
  enableCIPOpullUp(); // Enable CIPO pull-up on the OLA
  Serial.begin(115200);
  Serial.println("IDMinds OpenLog Artemis Driver Initializing");
  setStatusLED(HIGH);
  while(!beginIMU())
    Serial.println("IMU did not initiate correctly");
    delay(100);
  Serial.print("IMU initialized");
  setStatusLED(LOW);  
}

void loop() {
  if(Serial.available() > 0){
    // read the incoming byte:
    incomingByte = Serial.read();
    if(incomingByte == 0x20){
      Serial.print(0x20);
      Serial.println("IDMind OpenLog_Artemis");
      delay(500);
    }
  }
  /*
   * This Block probably does not work with DMP active, so it is better to comment it out 
   */
  /*if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
//    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT( &myICM );   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }else{
    Serial.println("Waiting for data");
    delay(500);
  }*/
  
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data or incomplete data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  
  if(( myICM.status == ICM_20948_Stat_Ok ) || ( myICM.status == ICM_20948_Stat_FIFOMoreDataAvail )) // Was valid data available?
  {
    //Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) Serial.print( "0" );
    //if ( data.header < 0x10) Serial.print( "0" );
    //Serial.println( data.header, HEX );
    
    if ( (data.header & DMP_header_bitmap_Quat9) > 0 ) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //Serial.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      float q1 = ((float)data.Quat9.Data.Q1) / 1073741824.0; // Convert to float. Divide by 2^30
      float q2 = ((float)data.Quat9.Data.Q2) / 1073741824.0; // Convert to float. Divide by 2^30
      float q3 = ((float)data.Quat9.Data.Q3) / 1073741824.0; // Convert to float. Divide by 2^30
      float q4 = sqrt(1-q1*q1-q2*q2-q3*q3);
      
      //Serial.printf("Q1:%.3f Q2:%.3f Q3:%.3f Q4:%.3f\r\n", q1, q2, q3, q4);
    }
  }

  if ( myICM.status != ICM_20948_Stat_FIFOMoreDataAvail ) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }  
}
