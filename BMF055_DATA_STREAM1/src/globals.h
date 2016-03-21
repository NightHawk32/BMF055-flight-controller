/*
 * globals.h
 *
 * Created: 05.03.2016 18:35:52
 *  Author: Lukas
 */ 




#ifndef GLOBALS_H_
#define GLOBALS_H_

#define  VERSION  210

#define COUNT_MAX_16BIT			UINT16_C(0xFFFF)
#define PWM_MIN_PULSE			UINT16_C(24000)
#define ONESHOT_MIN_PULSE		UINT16_C(2400)

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//#define abs(x) ((x)>0?(x):-(x))
#define PI 3.14159265358979323846

#define ACC 0
#define MAG 0
#define GYRO 1
#define BARO 0
#define SONAR 0
#define GPS 0

#define SPEK_MAX_CHANNEL 7
#define SPEK_FRAME_SIZE 16

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 100
#endif


/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 400.0f
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#ifndef GYR_CMPFM_FACTOR
  #define GYR_CMPFM_FACTOR 200.0f
#endif

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
  #define GYRO_SCALE ((1998 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //(MPU6050)
  //#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
  // +-2000/sec deg scale
  //#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
  #define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif 
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

/*********** RC alias *****************/
enum rc {
	ROLL,
	PITCH,
	YAW,
	THROTTLE,
	AUX1,
	AUX2,
	AUX3,
	AUX4
};

enum pid {
	PIDROLL,
	PIDPITCH,
	PIDYAW,
	PIDALT,
	PIDPOS,
	PIDPOSR,
	PIDNAVR,
	PIDLEVEL,
	PIDMAG,
	PIDVEL,     // not used currently
	PIDITEMS
};

enum box {
	BOXACC,
	BOXHORIZON,
	BOXARM,
	BOXBEEP,
	BOX3D,
	CHECKBOXITEMS
};

struct flags_struct {
	uint8_t OK_TO_ARM :1 ;
	uint8_t ARMED :1 ;
	uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
	uint8_t ACC_CALIBRATED :1 ;
	uint8_t ACC_MODE :1 ;
	uint8_t HORIZON_MODE :1 ;
	uint8_t SMALL_ANGLES_25 :1 ;
	uint8_t FSBEEP :1 ;
};

extern struct flags_struct f;

// ************************
// EEPROM Layout definition
// ************************
struct config{
	uint8_t checkNewConf;
	uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
	uint8_t rcRate8;
	uint8_t rcExpo8;
	uint8_t rollPitchRate;
	uint8_t yawRate;
	uint8_t dynThrPID;
	uint8_t thrMid8;
	uint8_t thrExpo8;
	int16_t accZero[3];
	int16_t angleTrim[2];
	uint16_t activate[CHECKBOXITEMS];

	uint8_t F3D;
	uint8_t MIDDLEDEADBAND;
	
	uint8_t sOneShot;
	
	uint8_t copterType;
	
	uint8_t RxType;
	
	int16_t MINTHROTTLE;
	int16_t MAXTHROTTLE;
	int16_t MINCOMMAND;
	int16_t MIDRC;
	int16_t MINCHECK;
	int16_t MAXCHECK;
	
	uint16_t BILeftMiddle;
	uint16_t BIRightMiddle;
	uint16_t TriYawMiddle;
	uint8_t  YAW_DIRECTION;
	uint8_t  BiLeftDir;
	uint8_t  BiRightDir;
	uint8_t  DigiServo;
	
	uint8_t  ArmRoll;
	
	uint8_t  MPU6050_DLPF_CFG;
	
	uint16_t  s3DMIDDLE;
	
	uint8_t calibState;
};

extern volatile unsigned long tc6_overflows;

extern struct config conf;

extern int16_t axisPID[3];
extern int16_t motor[6];
extern int16_t servo[6];

extern uint8_t s3D;
extern uint8_t NUMBER_MOTOR;
extern uint8_t  MULTITYPE;
extern int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
extern int16_t  i2c_errors_count;
extern int16_t  heading,magHold;
extern int16_t  headFreeModeHold;
extern int16_t gyroData[3];
extern int16_t gyroZero[3];
extern int16_t angle[2];
extern int32_t  EstAlt;
extern uint16_t calibratingA;
extern int16_t  debug[4];
extern uint16_t calibratingG;
extern int8_t good_calib;
extern uint16_t acc_1G;
extern int16_t  acc_25deg;
extern uint8_t throttleTest;
extern uint16_t time;
extern uint16_t pTime;
extern uint8_t rcOptions[CHECKBOXITEMS];
extern int16_t  Zadd;
extern volatile uint8_t rcFrameComplete; // for serial rc receiver Spektrum
extern uint8_t  failsave;
extern uint16_t cycleTime; 
/*uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     
uint16_t calibratingA = 0; 
uint16_t calibratingG;
uint16_t acc_1G;            
int16_t  acc_25deg;
int16_t  headFreeModeHold;
int16_t  heading,magHold;
uint8_t  vbat;               
uint8_t  rcOptions[CHECKBOXITEMS];
           
int16_t  BaroPID = 0;
int32_t  AltHold;

int16_t  errorAltitudeI = 0;
int16_t  accZCal = 0;
int16_t  Zadd    = 0;
int8_t   SetupMode    = 0;
int8_t   good_calib = 0;
uint8_t  failsave = 0;

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;*/

extern int16_t rcData[8];        
extern int16_t rcCommand[4];     
extern int16_t lookupPitchRollRC[6];
extern int16_t lookupThrottleRC[11];
extern int8_t   SetupMode;

unsigned long micros(void);
unsigned long millis(void);
void annexCode(void);

#endif /* GLOBALS_H_ */