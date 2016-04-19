#include "sensors.h"
#include "globals.h"
#include "eeprom_emulation.h"

// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = -Z;}
#endif



uint8_t rawADC[6];  

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis;

  if (calibratingG>0) {
    good_calib = 0;
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 400) g[axis]=0;
      // Sum up 400 readings
      g[axis] +=gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=g[axis]/400;
      }
    }
    calibratingG--;
  }
  if(calibratingG == 0 && good_calib == 0 && (gyroADC[0]-gyroZero[0] > 30 || gyroADC[0]-gyroZero[0] < -30  || gyroADC[1]-gyroZero[1] > 30 || gyroADC[1]-gyroZero[1] < -30 || gyroADC[2]-gyroZero[2] > 30 || gyroADC[2]-gyroZero[2] < -30)){
    calibratingG = 400;
    gyroZero[0] = 0;
    gyroZero[1] = 0;
    gyroZero[2] = 0;
  }else{
    good_calib = 1; 
  }

  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    previousGyroADC[axis] = gyroADC[axis];
  }
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 400) a[axis]=0;
      // Sum up 400 readings
      a[axis] +=accADC[axis];
      // Clear global variables for next reading
      accADC[axis]=0;
      conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      conf.accZero[ROLL]  = a[ROLL]/400;
      conf.accZero[PITCH] = a[PITCH]/400;
      conf.accZero[YAW]   = a[YAW]/400-acc_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeParams(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  accADC[ROLL]  -=  conf.accZero[ROLL] ;
  accADC[PITCH] -=  conf.accZero[PITCH];
  accADC[YAW]   -=  conf.accZero[YAW] ;
}






// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************

void Gyro_init() {
  bmg160_set_range_reg(0x00);
  bmg160_set_bw(0x03);			//47Hz
  bmg160_set_data_enable(1);	//new Data interrupt
}

void Gyro_getADC () {
  //i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  static struct bmg160_data_t bmg160_gyro_data;
  bmg160_get_data_XYZ(&bmg160_gyro_data);
  GYRO_ORIENTATION(bmg160_gyro_data.datax/4 , // range: +/- 8192; +/- 2000 deg/sec
	            bmg160_gyro_data.datay/4 ,
	            bmg160_gyro_data.dataz/4 );
  GYRO_Common();
}

void ACC_init () {
  bma2x2_set_range(BMA2x2_RANGE_8G);
  bma2x2_set_bw(BMA2x2_BW_500HZ);
  //bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
  acc_1G = 512;

}

void ACC_getADC () {
	static struct bma2x2_accel_data bma2x2_accel_data;
  	bma2x2_read_accel_xyz(&bma2x2_accel_data);

  ACC_ORIENTATION( bma2x2_accel_data.x/8 ,
                   bma2x2_accel_data.y/8 ,
                   bma2x2_accel_data.z/8 );
  ACC_Common();
}

  
void initSensors() {
  if (GYRO) Gyro_init();
  if (ACC) {ACC_init();acc_25deg = acc_1G * 0.423;}
  f.I2C_INIT_DONE = 1;
}