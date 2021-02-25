/*
 * Library of functions, to keep the main file clean
 */
void setIMUPower(bool enable)
{
  pinMode(PIN_IMU_POWER, OUTPUT);
  digitalWrite(PIN_IMU_POWER, enable);
}


void setPowerLED(bool enable)
{
  pinMode(PIN_PWR_LED, OUTPUT);
  digitalWrite(PIN_PWR_LED, enable); // Turn the Power LED on/off
}

void setStatusLED(bool enable)
{
  pinMode(PIN_STAT_LED, OUTPUT);
  digitalWrite(PIN_STAT_LED, enable); // Turn the Stat LED on/off
}

bool enableCIPOpullUp()
{
  //Add CIPO pull-up
  ap3_err_t retval = AP3_OK;
  am_hal_gpio_pincfg_t cipoPinCfg = AP3_GPIO_DEFAULT_PINCFG;
  cipoPinCfg.uFuncSel = AM_HAL_PIN_6_M0MISO;
  cipoPinCfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA;
  cipoPinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL;
  cipoPinCfg.uIOMnum = AP3_SPI_IOM;
  cipoPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  padMode(MISO, cipoPinCfg, &retval);
  return (retval == AP3_OK);
}

bool beginIMU(){
  Serial.println("IMU initializing...");
  // OLA Specific
  pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
  digitalWrite(PIN_IMU_CHIP_SELECT, HIGH);
  
  //Reset ICM by power cycling it
  setIMUPower(LOW);
  delay(10);
  setIMUPower(HIGH);
  delay(100); // Wait for the IMU to power up

  // Open SPI Port
  myICM.begin( CS_PIN, SPI ); 
  if( myICM.status != ICM_20948_Stat_Ok ){
    Serial.print("Error initialising IMU: ");
    Serial.println( myICM.statusString() );
    return false;
  }

  // ICM 20948 Configuration for DMP readings
  Serial.println("IMU configuration.");
  bool success = true; // Use success to show if the configuration was successful
  
  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  success &= (myICM.setClockSource(ICM_20948_Clock_Auto) == ICM_20948_Stat_Ok); // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  uint8_t zero = 0;
  success &= (myICM.write(AGB0_REG_PWR_MGMT_2, &zero, 1) == ICM_20948_Stat_Ok); // Write one byte to the PWR_MGMT_2 register
  
  // Configure Gyro/Accel in Low Power Mode (cycled) with LP_CONFIG
  // Sensors: ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mag, ICM_20948_Internal_Tmp, ICM_20948_Internal_Mst
  success &= (myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled ) == ICM_20948_Stat_Ok);

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
  myFSS.g = dps2000;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
  success &= (myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS ) == ICM_20948_Stat_Ok);

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //success &= (myICM.intEnableOverflowFIFO( 0x1F ) == ICM_20948_Stat_Ok); // Enable the interrupt on all FIFOs
  
  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  success &= (myICM.write(AGB0_REG_FIFO_EN_1, &zero, 1) == ICM_20948_Stat_Ok);
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  success &= (myICM.write(AGB0_REG_FIFO_EN_2, &zero, 1) == ICM_20948_Stat_Ok);

  // Turn off data ready interrupt through INT_ENABLE_1
  success &= (myICM.intEnableRawDataReady(false) == ICM_20948_Stat_Ok);

  // Reset FIFO through FIFO_RST
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 43; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 43 = 25Hz
  mySmplrt.a = 44; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 44 = 25Hz
  //myICM.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt ); // ** Note: comment this line to leave the sample rates at the maximum **
  
  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  success &= (myICM.loadDMPFirmware() == ICM_20948_Stat_Ok);

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  success &= (myICM.setDMPstartAddress() == ICM_20948_Stat_Ok); // Defaults to DMP_START_ADDRESS

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x4000000 when FSR is 4g
  const unsigned char accScale[4] = {0x40, 0x00, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE, 4, &accScale[0]) == ICM_20948_Stat_Ok); // Write 0x4000000 to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x40000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  success &= (myICM.writeDMPmems(ACC_SCALE2, 4, &accScale2[0]) == ICM_20948_Stat_Ok); // Write 0x40000 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // Magnetometer full scale is +/- 4900uT so _I think_ we need to multiply by 2^30 / 4900 = 0x000357FA
  // The magnetometer Y and Z axes are reversed compared to the accelerometer so we'll invert those
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x00, 0x03, 0x57, 0xFA};
  const unsigned char mountMultiplierMinus[4] = {0xFF, 0xFC, 0xA8, 0x05};
  success &= (myICM.writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  success &= (myICM.writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //success &= (myICM.intEnableDMP(true) == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER
  //    INV_ICM20948_SENSOR_GYROSCOPE
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED
  //    INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON
  //    INV_ICM20948_SENSOR_STEP_DETECTOR
  //    INV_ICM20948_SENSOR_STEP_COUNTER
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD
  //    INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION
  //    INV_ICM20948_SENSOR_FLIP_PICKUP
  //    INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR
  //    INV_ICM20948_SENSOR_GRAVITY
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION
  //    INV_ICM20948_SENSOR_ORIENTATION

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Set the DMP Output Data Rate for Quat9 to 20Hz.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 20) == ICM_20948_Stat_Ok);
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 20) == ICM_20948_Stat_Ok);

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  
  // Check success
  if( success )
    Serial.println("DMP enabled!");
  else
  {
    Serial.println("Enable DMP failed!");
  }
  
  return true;
}

void printPaddedInt16b( int16_t val ){
  if(val > 0){
    Serial.print(" ");
    if(val < 10000){ Serial.print("0"); }
    if(val < 1000 ){ Serial.print("0"); }
    if(val < 100  ){ Serial.print("0"); }
    if(val < 10   ){ Serial.print("0"); }
  }else{
    Serial.print("-");
    if(abs(val) < 10000){ Serial.print("0"); }
    if(abs(val) < 1000 ){ Serial.print("0"); }
    if(abs(val) < 100  ){ Serial.print("0"); }
    if(abs(val) < 10   ){ Serial.print("0"); }
  }
  Serial.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  Serial.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  Serial.print(", ");
  printPaddedInt16b( agmt.acc.axes.y );
  Serial.print(", ");
  printPaddedInt16b( agmt.acc.axes.z );
  Serial.print(" ], Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  Serial.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  Serial.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  Serial.print(" ], Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  Serial.print(", ");
  printPaddedInt16b( agmt.mag.axes.y );
  Serial.print(", ");
  printPaddedInt16b( agmt.mag.axes.z );
  Serial.print(" ], Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  Serial.print(" ]");
  Serial.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    Serial.print("-");
  }else{
    Serial.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      Serial.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    Serial.print(-val, decimals);
  }else{
    Serial.print(val, decimals);
  }
}

void printScaledAGMT( ICM_20948_SPI *sensor ){
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( sensor->accX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->accY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->accZ(), 5, 2 );
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( sensor->gyrX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->gyrY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->gyrZ(), 5, 2 );
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat( sensor->magX(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->magY(), 5, 2 );
  Serial.print(", ");
  printFormattedFloat( sensor->magZ(), 5, 2 );
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat( sensor->temp(), 5, 2 );
  Serial.print(" ]");
  Serial.println();
}
