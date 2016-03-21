/**\mainpage
*
*
**************************************************************************
* Copyright (C) 2015 Bosch Sensortec GmbH. All Rights Reserved.
*
* File:		main.c
*
* Date:		2015/02/02
*
* Revision:	1.0
*
* Usage:	Part of BMF055 Data Stream Project
*
**************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*
*
*************************************************************************/
/*!
*
* @file		main.c
* @author	Bosch Sensortec
*
* @brief	Main Source File
*
*/


/************************************************************************/
/* Includes                                                             */
/************************************************************************/

#include "bmf055.h"
#include "eeprom_emulator_support.h"
#include "globals.h"
#include "serial.h"
#include "eeprom_emulation.h"
#include "sensors.h"
#include "output.h"
#include "rx.h"
#include "imu.h"


struct config conf;

uint16_t time=0;
uint16_t pTime=0;


uint8_t s3D            = 0; // 3D an = 1 aus ist 0
uint8_t NUMBER_MOTOR   = 0;
uint8_t throttleTest   = 0;
uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingG;
uint16_t acc_1G;             // this is the 1G measured acceleration
int16_t  acc_25deg;
int16_t  headFreeModeHold;
int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
int16_t  heading,magHold;
uint8_t  vbat;               // battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];
int32_t  EstAlt;             // in cm
int16_t  BaroPID = 0;
int32_t  AltHold;
uint8_t  MULTITYPE = 0;
int16_t  errorAltitudeI = 0;
int16_t  accZCal = 0;
int16_t  Zadd    = 0;
int8_t   SetupMode    = 0;
int8_t   good_calib = 0;
uint8_t  failsave = 0;


int16_t  debug[4];

struct flags_struct f;


int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;


int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[8];          // interval [1000;2000]
int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
volatile uint8_t rcFrameComplete; // for serial rc receiver Spektrum


// **************
// gyro+acc IMU
// **************
int16_t gyroData[3] = {0,0,0};
int16_t gyroZero[3] = {0,0,0};
int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[6] = {1000,1000,1000,1000,1000,1000};
int16_t servo[6] = {1500,1500,1500,1500,1500,1500};
	
static uint8_t dynP8[3], dynD8[3];
	
	
void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
	static uint32_t calibratedAccTime;
	uint16_t tmp,tmp2;
	uint8_t axis,prop1,prop2;

	#define BREAKPOINT 1500
	// PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
	if   (rcData[THROTTLE]<BREAKPOINT) {
		prop2 = 100;
		} else {
		if (rcData[THROTTLE]<2000) {
			prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
			} else {
			prop2 = 100 - conf.dynThrPID;
		}
	}

	for(axis=0;axis<3;axis++) {
		tmp = min(abs(rcData[axis]-conf.MIDRC),500);
		if(axis!=2) { //ROLL & PITCH
			tmp2 = tmp/100;
			rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
			prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
			prop1 = (uint16_t)prop1*prop2/100;
			} else {      // YAW
			rcCommand[axis] = tmp;
			prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
		}
		dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
		dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
		if (rcData[axis]<conf.MIDRC) rcCommand[axis] = -rcCommand[axis];
	}
	tmp = constrain(rcData[THROTTLE],conf.MINCHECK,2000);
	tmp = (uint32_t)(tmp-conf.MINCHECK)*1000/(2000-conf.MINCHECK); // [MINCHECK;2000] -> [0;1000]
	tmp2 = tmp/100;
	rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
	
	if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
		//LEDPIN_TOGGLE; TBD
		} else {
		//if (f.ACC_CALIBRATED) {LEDPIN_OFF;} TBD
		//if (f.ARMED) {LEDPIN_ON;} TBD
	}


	if ( currentTime > calibratedAccTime ) {
		if (! f.SMALL_ANGLES_25) {
			// the multi uses ACC and is not calibrated or is too much inclinated
			f.ACC_CALIBRATED = 0;
			//LEDPIN_TOGGLE; TBD
			calibratedAccTime = currentTime + 500000;
			} else {
			f.ACC_CALIBRATED = 1;
		}
	}
	
	serialCom();

}

/************************************************************************/
/* Main Function Definition                                             */
/************************************************************************/

/*!
* @brief	Initializes the whole system and runs the desired application
*
* This is the main function of the project. It calls initialization functions
* of the MCU and the sensors. In the infinite loop it repeatedly checks
* the USART module read buffer and Streams sensor data periodically (100 ms) via USART.
*
*/
int main(void)
{
	/********************* Initialize global variables **********************/

	bmf055_input_state = USART_INPUT_STATE_PRINT_DATA;
	
	/************************* Initializations ******************************/
	
	/*Initialize SAMD20 MCU*/
	system_init();
	
	/*Initialize clock module of SAMD20 MCU - Internal RC clock*/
	//clock_initialize();
	
	/*SPI master for communicating with sensors*/
	spi_initialize();
	
	/*eeprom emulator for configuration storage */
	eeprom_emulator_initialize();
	
	/*Initialize timers */
	tc_initialize();
	
	/*Initialize UART for communication with PC*/
	usart_initialize();
	
	/*Enable the system interrupts*/
	system_interrupt_enable_global();/* All interrupts have a priority of level 0 which is the highest. */
	
	/* Initialize the sensors */
	bmf055_sensors_initialize();
	
	readEEPROM();
	checkFirstTime(0);
	//readEEPROM();
	
	configureReceiver();
	initSensors();
	previousTime = micros();
	calibratingG = 400;
	f.SMALL_ANGLES_25=1; // important for gyro only conf
  if(conf.copterType == 0){//0=Bi,1=Tri,2=QUADP,3=QUADX,4=Y4,5=Y6,6=H6P,7=H6X,8=Vtail4
    MULTITYPE      = 4;
    NUMBER_MOTOR   = 2;
  }
  if(conf.copterType == 1){
    MULTITYPE      = 1;
    NUMBER_MOTOR   = 3;
  }
  if(conf.copterType == 2){
    MULTITYPE      = 2;
    NUMBER_MOTOR   = 4;
  }
  if(conf.copterType == 3){
    MULTITYPE      = 3;
    NUMBER_MOTOR   = 4;
  }
  if(conf.copterType == 4){
    MULTITYPE      = 9;
    NUMBER_MOTOR   = 4;
  }
  if(conf.copterType == 5){
    MULTITYPE      = 6;
    NUMBER_MOTOR   = 6;
  }
  if(conf.copterType == 6){
    MULTITYPE      = 7;
    NUMBER_MOTOR   = 6;
  }
  if(conf.copterType == 7){
    MULTITYPE      = 10;
    NUMBER_MOTOR   = 6;
  }     
  if(conf.copterType == 8){
    MULTITYPE      = 17;
    NUMBER_MOTOR   = 4;
  }
  
  initOutput();
	
	/************************** Infinite Loop *******************************/
	while (true)
	{
		/*bmf055_sensors_data_print();
		for(int i=0;i<1000000;i++){
			asm("nop");
		}*/
		
		static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t beepon = 0;
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm=0,ITerm=0,PTermACC=0,ITermACC=0,PTermGYRO=0,ITermGYRO=0,DTerm=0;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static uint32_t BeepTime  = 0;
  static uint8_t stickarmed = 0;
  //static int16_t initialThrottleHold;
  
  if(!rcOptions[BOXARM] && stickarmed == 0 && f.ARMED == 0){
    if(rcData[YAW]<conf.MINCHECK && rcData[ROLL]>conf.MAXCHECK){
      conf.calibState=1;
      writeParams(1);
      while(true){
        //blinkLED(10,30,1);
      }      
    }
  } 
 
  while(SetupMode == 1){
    checkSetup();
  }
 
 
  if(conf.RxType == 1 || conf.RxType == 2){
    if (rcFrameComplete) computeRC();
  }
  
  if(!rcOptions[BOXARM] && stickarmed == 0) {
    f.ARMED = 0;
  }

  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
    if(failsave < 250)failsave++;
    debug[0] = failsave;
    if(conf.RxType != 1 && conf.RxType != 2){
      computeRC();
    }
  
    if ((rcData[THROTTLE] < conf.MINCHECK && s3D == 0) || (rcData[THROTTLE] > (1500-conf.MIDDLEDEADBAND) && rcData[THROTTLE] < (1500+conf.MIDDLEDEADBAND) && s3D == 1 && f.ARMED == 0)) {
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;

      rcDelayCommand++;
      if (rcData[YAW] < conf.MINCHECK && rcData[PITCH] < conf.MINCHECK && !f.ARMED) {
        if (rcDelayCommand == 20 && failsave < 20) {
          calibratingG=400;
        }
      }else if (rcData[YAW] > conf.MAXCHECK && rcData[PITCH] > conf.MAXCHECK && !f.ARMED) {
        if (rcDelayCommand == 20) {
          previousTime = micros();
        }
      }else if (conf.activate[BOXARM] > 0) {
        if ( rcOptions[BOXARM] && f.OK_TO_ARM && good_calib) {
	  f.ARMED = 1;
          stickarmed = 0;
        } else if (f.ARMED) f.ARMED = 0;
        rcDelayCommand = 0;
        
      
      } else if ( (rcData[YAW] < conf.MINCHECK ) && f.ARMED && !rcOptions[BOXARM] && s3D == 0 && conf.ArmRoll == 0) {
        if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[YAW] > conf.MAXCHECK ) && rcData[PITCH] < conf.MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED && !rcOptions[BOXARM] && s3D == 0 && conf.ArmRoll == 0) {
        if (rcDelayCommand == 20 && good_calib) {
	  f.ARMED = 1;
          stickarmed = 1;
        }
  
       } else if ( (rcData[ROLL] < conf.MINCHECK ) && f.ARMED && !rcOptions[BOXARM] && s3D == 0 && conf.ArmRoll == 1) {
        if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[ROLL] > conf.MAXCHECK ) && rcData[PITCH] < conf.MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED && !rcOptions[BOXARM] && s3D == 0 && conf.ArmRoll == 1) {
        if (rcDelayCommand == 20 && good_calib) {
	  f.ARMED = 1;
          stickarmed = 1;
        }       
        
        
      } else
        rcDelayCommand = 0;
    } else if (rcData[THROTTLE] > conf.MAXCHECK && !f.ARMED) {
      if (rcData[YAW] < conf.MINCHECK && rcData[PITCH] < conf.MINCHECK) {        // throttle=max, yaw=left, pitch=min
        if (rcDelayCommand == 20) calibratingA=400;
        rcDelayCommand++;
      } else if (rcData[PITCH] > conf.MAXCHECK) {
         conf.angleTrim[PITCH]+=2;writeParams(1);
      } else if (rcData[PITCH] < conf.MINCHECK) {
         conf.angleTrim[PITCH]-=2;writeParams(1);
      } else if (rcData[ROLL] > conf.MAXCHECK) {
         conf.angleTrim[ROLL]+=2;writeParams(1);
      } else if (rcData[ROLL] < conf.MINCHECK) {
         conf.angleTrim[ROLL]-=2;writeParams(1);
      } else {
        rcDelayCommand = 0;
      }
    }
    
    

    uint16_t auxState = 0;
    for(i=0;i<4;i++)
      auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
    for(i=0;i<CHECKBOXITEMS;i++)
      rcOptions[i] = (auxState & conf.activate[i])>0;
      
      
     if(failsave > 200 && f.ARMED){
      rcOptions[BOXACC] = 1;
      s3D = 0;
      rcData[THROTTLE] = 1190;
      rcCommand[THROTTLE] = 1190;
    }

    if (rcOptions[BOXACC] && s3D == 0) { 
      // bumpless transfer to Level mode
      if (!f.ACC_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.ACC_MODE = 1;
      }  
    } else {
      // failsafe support
      f.ACC_MODE = 0;
    }
    if (rcOptions[BOXBEEP]) { 
      f.FSBEEP = 1;
      if(currentTime > BeepTime){
        BeepTime = currentTime + 50000;
        if(beepon == 0){
          if(conf.RxType == 0){
            //digitalWrite(A2,HIGH); 
          }else{
            //digitalWrite(8,HIGH); 
          }          
          beepon = 1;
        }else{
          if(conf.RxType == 0){
            //digitalWrite(A2,LOW); 
          }else{
            //digitalWrite(8,LOW); 
          }
          beepon = 0;
        }
      }
    } else {
      f.FSBEEP = 0;
      if(conf.RxType == 0){
        //digitalWrite(A2,LOW); 
      }else{
        //digitalWrite(8,LOW); 
      }
    }    

    
    if (rcOptions[BOXHORIZON] && s3D == 0) { 
      // bumpless transfer to Horizon mode
      if (!f.HORIZON_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.HORIZON_MODE = 1;
      }  
    } else {
      f.HORIZON_MODE = 0;
    }
    
    if (rcOptions[BOX3D] && conf.F3D == 1) {  
      if(f.ARMED == 0 && s3D == 0){
        s3D = 1;
        f.ACC_MODE = 0;
        f.HORIZON_MODE = 0;
      }
    } else if(f.ARMED == 0){
      s3D = 0;
    }   
    

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
  }
 
  computeIMU();
  int16_t prop;
  if (f.HORIZON_MODE)
    prop = max(abs(rcCommand[PITCH]),abs(rcCommand[ROLL])); // range [0;500]  
  
  
  if (f.ACC_MODE){
    if(Zadd > 0)Zadd--;
    if(Zadd < 0)Zadd++;
  }else{
    Zadd = 0;
  }

  

  //**** PITCH & ROLL & YAW PID ****    
  for(axis=0;axis<3;axis++) {
    if ((f.ACC_MODE || f.HORIZON_MODE) && axis<2 ) { //LEVEL MODE
      // 50 degrees max inclination
      errorAngle = constrain(2*rcCommand[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
      #ifdef LEVEL_PDF
        PTermACC      = -(int32_t)angle[axis]*conf.P8[PIDLEVEL]/100 ;
      #else  
        PTermACC      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      #endif
      PTermACC = constrain(PTermACC,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

      errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
      ITermACC           = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    }
    if ( !f.ACC_MODE || f.HORIZON_MODE || axis == 2 ) { // MODE relying on GYRO or YAW axis
      if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                               else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
      error -= gyroData[axis];

      PTermGYRO = rcCommand[axis];
      
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
      ITermGYRO         = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if ( f.HORIZON_MODE && axis<2) {
      PTerm = ((int32_t)PTermACC*(500-prop) + (int32_t)PTermGYRO*prop)/500;
      ITerm = ((int32_t)ITermACC*(500-prop) + (int32_t)ITermGYRO*prop)/500;
    } else {
      if ( f.ACC_MODE && axis<2) {
        PTerm = PTermACC;
        ITerm = ITermACC;
      } else {
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }
    }
    if (abs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
                            else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation   

    delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    if (abs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
                      else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation
                      
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }
  
  mixTable();

  currentTime = micros();
  cycleTime = currentTime - previousTime;
  
  //while(cycleTime < 2400){
    //currentTime = micros();
    //cycleTime = currentTime - previousTime;  
  //}
  previousTime = currentTime;
  if(!throttleTest || calibratingG == 0){
    writeMotors();
  }

  
		
		//serialCom();
		//writeMotors();
		//computeRC();
		//for(int i=0; i<1000;i++){
		//	asm("nop");
		//}
		/* Process USART inputs */
		/*if (USART_COMMAND_PROCESS_FLAG)
		{
			bmf055_usart_read_process(usart_rx_string[0]);
			
			// Reset USART Flag 
			USART_COMMAND_PROCESS_FLAG = false;
		}*/
		
		/* Print sensor data periodically regarding TC6 interrupt flag (Default Period 100 ms)*/
		/*if (READ_SENSORS_FLAG)
		{
			bmf055_sensors_data_print();
			
			
			READ_SENSORS_FLAG = false;
		}*/
		
	} /* !while (true) */
		
}

