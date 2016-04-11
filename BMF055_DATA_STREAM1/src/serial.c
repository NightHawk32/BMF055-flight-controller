/*
 * serial.c
 *
 * Created: 05.03.2016 20:34:54
 *  Author: Lukas
 */ 

#include "serial.h"
#include "globals.h"
#include "eeprom_emulation.h"
#include <string.h>


/************************************************************************/
/* Global Variables                                                     */
/************************************************************************/

volatile uint8_t serialHeadRX,serialTailRX;
uint8_t serialBufferRX[RX_BUFFER_SIZE];
volatile uint8_t headTX,tailTX;
uint8_t bufTX[TX_BUFFER_SIZE];
uint8_t inBuf[INBUF_SIZE];
uint8_t PWM_PIN[8] = {9,10,5,6,11,13};   //for a quad+: rear,right,left,front

static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP = 0;



const char boxnames[] = // names for dynamic generation of config GUI
"LEVEL;"
"HORIZON;"
"ARM;"
"FS_BEEP;"
"3D_MODE;"
;

const uint8_t boxids[]= {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
	0,1,2,3,4
};

const char pidnames[] =
"ROLL;"
"PITCH;"
"YAW;"
"ALT;"
"Pos;"
"PosR;"
"NavR;"
"LEVEL;"
"MAG;"
"VEL;"
;


uint32_t read32() {
	uint32_t t = read16();
	t+= (uint32_t)read16()<<16;
	return t;
}

uint16_t read16() {
	uint16_t t = read8();
	t+= (uint16_t)read8()<<8;
	return t;
}

uint8_t read8()  {
	return inBuf[indRX++]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
	serialize8('$');
	serialize8('M');
	serialize8(err ? '!' : '>');
	checksum = 0; // start calculating a new checksum
	serialize8(s);
	serialize8(cmdMSP);
}

void headSerialReply(uint8_t s) {
	headSerialResponse(0, s);
}

void headSerialError(uint8_t s) {
	headSerialResponse(1, s);
}

void tailSerialReply() {
	serialize8(checksum);UartSendData();
}

void serializeNames(const char * s) {
	uint8_t i=0;
	for (const char * c = s; i < strlen(s); c++,i++) {
		serialize8(*c);
	}
}

void checkSetup(){
	static uint8_t reciving = 0;
	static uint8_t setupRecBuf[32];
	static uint8_t setupRecCount = 0;
	while(SerialAvailable()){
		
		if(reciving == 1){
			setupRecBuf[setupRecCount++] = SerialRead();
			if(setupRecCount >= 31){
				conf.F3D            = setupRecBuf[0]; // ESC's are set to 3D Mode
				conf.MIDDLEDEADBAND = setupRecBuf[1]; //nur für 3D
				
				conf.sOneShot       = setupRecBuf[2]; //0=normaler betrieb (4xxhz) 1 ist oneshot 125
				
				conf.copterType     = setupRecBuf[3]; //0=Bi,1=Tri,2=QUADP,3=QUADX,4=Y4,5=Y6,6=H6P,7=H6X,8=Vtail4
				
				conf.RxType         = setupRecBuf[4]; //0StandardRX,1sat1024,2sat2048,3PPMGrSp,4PPMRobHiFu,5PPMHiSanOthers
				
				conf.MINTHROTTLE   = (setupRecBuf[5]<<8) | setupRecBuf[6];
				conf.MAXTHROTTLE   = (setupRecBuf[7]<<8) | setupRecBuf[8];
				conf.MINCOMMAND    = (setupRecBuf[9]<<8) | setupRecBuf[10];
				conf.MIDRC         = (setupRecBuf[11]<<8) | setupRecBuf[12];
				conf.MINCHECK      = (setupRecBuf[13]<<8) | setupRecBuf[14];
				conf.MAXCHECK      = (setupRecBuf[15]<<8) | setupRecBuf[16];
				
				conf.BILeftMiddle  = (setupRecBuf[17]<<8) | setupRecBuf[18];
				conf.BIRightMiddle = (setupRecBuf[19]<<8) | setupRecBuf[20];
				conf.TriYawMiddle  = (setupRecBuf[21]<<8) | setupRecBuf[22];
				if(setupRecBuf[23] == 0)conf.YAW_DIRECTION = 1; else conf.YAW_DIRECTION =-1;
				if(setupRecBuf[24] == 0)conf.BiLeftDir = 1; else conf.BiLeftDir =-1;
				if(setupRecBuf[25] == 0)conf.BiRightDir = 1; else conf.BiRightDir =-1;
				
				conf.DigiServo = setupRecBuf[26];
				conf.ArmRoll       = setupRecBuf[27];  // arm und disarm über roll stat yaw
				
				conf.MPU6050_DLPF_CFG = setupRecBuf[28]; //0=256(aus),1=188,2=98,3=42,4=20,5=10,6=5
				conf.s3DMIDDLE = (setupRecBuf[29]<<8) | setupRecBuf[30];
				
				writeParams(1); //TBD
				//delay(30);
				NVIC_SystemReset();
				// setup WDT
				//WDTCSR = 0;
				//wdt_reset();
				//set up WDT reset
				//WDTCSR = (1<<WDCE)|(1<<WDE);
				//Start watchdog timer with 0,125s prescaler
				//WDTCSR = (1<<WDE)|(1<<WDP0)|(1<<WDP1);
				while(1);
			}
			}else if(SerialRead() == 'A'){
			reciving = 1;
			}else if(SerialRead() == 'B'){
			serialize8('B');UartSendData();
			serialize8(conf.F3D);UartSendData();
			serialize8(conf.MIDDLEDEADBAND);UartSendData();
			
			serialize8(conf.sOneShot);UartSendData();
			
			serialize8(conf.copterType);UartSendData();
			
			serialize8(conf.RxType); UartSendData();
			
			serialize8(conf.MINTHROTTLE&0xff);UartSendData(); serialize8(conf.MINTHROTTLE>>8);UartSendData();
			serialize8(conf.MAXTHROTTLE&0xff);UartSendData(); serialize8(conf.MAXTHROTTLE>>8);UartSendData();
			serialize8(conf.MINCOMMAND&0xff);UartSendData(); serialize8(conf.MINCOMMAND>>8);UartSendData();
			serialize8(conf.MIDRC&0xff);UartSendData(); serialize8(conf.MIDRC>>8);UartSendData();
			serialize8(conf.MINCHECK&0xff);UartSendData(); serialize8(conf.MINCHECK>>8);UartSendData();
			serialize8(conf.MAXCHECK&0xff);UartSendData(); serialize8(conf.MAXCHECK>>8);UartSendData();
			
			serialize8(conf.BILeftMiddle&0xff);UartSendData(); serialize8(conf.BILeftMiddle>>8);UartSendData();
			serialize8(conf.BIRightMiddle&0xff);UartSendData(); serialize8(conf.BIRightMiddle>>8);UartSendData();
			serialize8(conf.TriYawMiddle&0xff);UartSendData(); serialize8(conf.TriYawMiddle>>8);UartSendData();
			if(conf.YAW_DIRECTION == 1){ serialize8(0);UartSendData(); }else{  serialize8(1);UartSendData();}
			if(conf.BiLeftDir == 1){ serialize8(0);UartSendData(); }else{  serialize8(1);UartSendData();}
			if(conf.BiRightDir == 1){ serialize8(0);UartSendData(); }else{  serialize8(1);UartSendData();}
			serialize8(conf.DigiServo);UartSendData();
			
			serialize8(conf.ArmRoll);UartSendData();
			
			serialize8(conf.MPU6050_DLPF_CFG);UartSendData();
			serialize8(conf.s3DMIDDLE&0xff);UartSendData(); serialize8(conf.s3DMIDDLE>>8);UartSendData();
			serialize8(';');UartSendData();
			}else if(SerialRead() == 'X'){
			SetupMode = 0;
		}
	}
}


void serialCom() {
	uint8_t c;
	static uint8_t offset;
	static uint8_t dataSize;
	static enum _serial_state {
		IDLE,
		HEADER_START,
		HEADER_M,
		HEADER_ARROW,
		HEADER_SIZE,
		HEADER_CMD,
	} c_state = IDLE;
	
	while (SerialAvailable()) {
		
		uint8_t bytesTXBuff = ((uint8_t)(headTX-tailTX))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
		if (bytesTXBuff > TX_BUFFER_SIZE - 40 ) return; // ensure there is enough free TX buffer to go further (40 bytes margin)
		c = SerialRead();
		
		if(c == '#' && SetupMode == 0 && offset == 0 && indRX == 0 && checksum == 0 && cmdMSP != '#'){
			SetupMode = 1;
			return;
		}
		
		if (c_state == IDLE) {
			c_state = (c=='$') ? HEADER_START : IDLE;
			if (c_state == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
			} else if (c_state == HEADER_START) {
			c_state = (c=='M') ? HEADER_M : IDLE;
			} else if (c_state == HEADER_M) {
			c_state = (c=='<') ? HEADER_ARROW : IDLE;
			} else if (c_state == HEADER_ARROW) {
			if (c > INBUF_SIZE) {  // now we are expecting the payload size
				c_state = IDLE;
				continue;
			}
			dataSize = c;
			offset = 0;
			checksum = 0;
			indRX = 0;
			checksum ^= c;
			c_state = HEADER_SIZE;  // the command is to follow
			} else if (c_state == HEADER_SIZE) {
			cmdMSP = c;
			checksum ^= c;
			c_state = HEADER_CMD;
			} else if (c_state == HEADER_CMD && offset < dataSize) {
			checksum ^= c;
			inBuf[offset++] = c;
			} else if (c_state == HEADER_CMD && offset >= dataSize) {
			if (checksum == c) {  // compare calculated and transferred checksum
				evaluateCommand();  // we got a valid packet, evaluate it
			}
			c_state = IDLE;
		}
	}
}

void evaluateCommand() {
	switch(cmdMSP) {
		case MSP_SET_RAW_RC:
		for(uint8_t i=0;i<8;i++) {
			rcData[i] = read16();
		}
		headSerialReply(0);
		break;
		case MSP_SET_PID:
		for(uint8_t i=0;i<PIDITEMS;i++) {
			conf.P8[i]=read8();
			conf.I8[i]=read8();
			conf.D8[i]=read8();
		}
		headSerialReply(0);
		break;
		case MSP_SET_BOX:
		for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
			conf.activate[i]=read16();
		}
		headSerialReply(0);
		break;
		case MSP_SET_RC_TUNING:
		conf.rcRate8 = read8();
		conf.rcExpo8 = read8();
		conf.rollPitchRate = read8();
		conf.yawRate = read8();
		conf.dynThrPID = read8();
		conf.thrMid8 = read8();
		conf.thrExpo8 = read8();
		headSerialReply(0);
		break;
		case MSP_SET_MISC:
		headSerialReply(0);
		break;
		
		case MSP_IDENT:
		headSerialReply(7);
		serialize8(VERSION);   // multiwii version
		serialize8(MULTITYPE); // type of multicopter
		serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
		serialize32(0);        // "capability"
		break;
		case MSP_STATUS:
		headSerialReply(11);
		serialize16(cycleTime);
		serialize16(i2c_errors_count);
		serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
		serialize32(f.ACC_MODE<<BOXACC|f.HORIZON_MODE<<BOXHORIZON|f.ARMED<<BOXARM|f.FSBEEP<<BOXBEEP|s3D<<BOX3D);
		serialize8(0);   // current setting
		break;
		case MSP_RAW_IMU:
		headSerialReply(18);
		for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]);
		for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]);
		for(uint8_t i=0;i<3;i++) serialize16(magADC[i]);
		break;
		case MSP_SERVO:
		headSerialReply(16);
		for(uint8_t i=0;i<8;i++)
		serialize16(servo[i]);
		break;
		case MSP_MOTOR:
		headSerialReply(16);
		for(uint8_t i=0;i<8;i++) {
			serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
		}
		break;
		case MSP_RC:
		headSerialReply(8 * 2);
		for(uint8_t i=0;i<8;i++) serialize16(rcData[i]);
		break;
		case MSP_ATTITUDE:
		headSerialReply(8);
		for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
		serialize16(heading);
		serialize16(headFreeModeHold);
		break;
		case MSP_ALTITUDE:
		headSerialReply(6);
		serialize32(EstAlt);
		serialize16(0);                  // added since r1172
		break;
		case MSP_RC_TUNING:
		headSerialReply(7);
		serialize8(conf.rcRate8);
		serialize8(conf.rcExpo8);
		serialize8(conf.rollPitchRate);
		serialize8(conf.yawRate);
		serialize8(conf.dynThrPID);
		serialize8(conf.thrMid8);
		serialize8(conf.thrExpo8);
		break;
		case MSP_PID:
		headSerialReply(3*PIDITEMS);
		for(uint8_t i=0;i<PIDITEMS;i++) {
			serialize8(conf.P8[i]);
			serialize8(conf.I8[i]);
			serialize8(conf.D8[i]);
		}
		break;
		case MSP_PIDNAMES:
		headSerialReply(strlen(pidnames));
		serializeNames(pidnames);
		break;
		case MSP_BOX:
		headSerialReply(2*CHECKBOXITEMS);
		for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
			serialize16(conf.activate[i]);
		}
		break;
		case MSP_BOXNAMES:
		headSerialReply(strlen(boxnames));
		serializeNames(boxnames);
		break;
		case MSP_BOXIDS:
		headSerialReply(CHECKBOXITEMS);
		for(uint8_t i=0;i<CHECKBOXITEMS;i++) {
			serialize8(boxids[i]);
		}
		break;
		case MSP_MISC:
		headSerialReply(2);
		serialize16(0);
		break;
		case MSP_MOTOR_PINS:
		headSerialReply(8);
		for(uint8_t i=0;i<8;i++) {
			serialize8(PWM_PIN[i]);
		}
		break;
		case MSP_RESET_CONF:
		conf.checkNewConf++;
		checkFirstTime(1);
		headSerialReply(0);
		break;
		case MSP_ACC_CALIBRATION:
		if(!f.ARMED) calibratingA=512;
		headSerialReply(0);
		break;
		case MSP_MAG_CALIBRATION:
		headSerialReply(0);
		break;
		case MSP_EEPROM_WRITE:
		writeParams(0);
		headSerialReply(0);
		break;
		case MSP_DEBUG:
		headSerialReply(8);
		for(uint8_t i=0;i<4;i++) {
			serialize16(debug[i]); // 4 variables are here for general monitoring purpose
		}
		break;
		default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
		headSerialError(0);
		break;
	}
	tailSerialReply();
}

void evaluateOtherData(uint8_t sr) {
	switch (sr) {
		// Note: we may receive weird characters here which could trigger unwanted features during flight.
		//       this could lead to a crash easily.
		//       Please use if (!f.ARMED) where neccessary
		
	}
}

void serialize32(uint32_t a) {
	serialize8((a    ) & 0xFF);
	serialize8((a>> 8) & 0xFF);
	serialize8((a>>16) & 0xFF);
	serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
	serialize8((a   ) & 0xFF);
	serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
	uint8_t t = headTX;
	if (++t >= TX_BUFFER_SIZE) t = 0;
	bufTX[t] = a;
	checksum ^= a;
	headTX = t;
}

void UartSendData() {
	while(headTX != tailTX) {
		if (++tailTX >= TX_BUFFER_SIZE) tailTX = 0;
		uint8_t* p = bufTX+tailTX;
		usart_write_buffer_wait(&usart_instance, p,1);
	}
}

void store_uart_in_buf(uint8_t data) {
	uint8_t h = serialHeadRX;
	if (++h >= RX_BUFFER_SIZE) h = 0;
	if (h == serialTailRX) return; // we did not bite our own tail?
	serialBufferRX[serialHeadRX] = data;
	serialHeadRX = h;
}

uint8_t SerialRead() {
	uint8_t t = serialTailRX;
	uint8_t c = serialBufferRX[t];
	if (serialHeadRX != t) {
		if (++t >= RX_BUFFER_SIZE) t = 0;
		serialTailRX = t;
	}
	return c;
}


uint8_t SerialAvailable() {
	return (serialHeadRX - serialTailRX)%RX_BUFFER_SIZE;
}

void SerialWrite(uint8_t c){
	serialize8(c);UartSendData();                // Serial0 TX is driven via a buffer and a background interrupt
}