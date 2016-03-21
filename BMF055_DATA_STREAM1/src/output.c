/*
 * output.c
 *
 * Created: 06.03.2016 19:23:02
 *  Author: Lukas
 */ 

#include <tc.h>
#include <tc_interrupt.h>
#include "eeprom_emulation.h"
#include "output.h"
#include "globals.h"
#include "serial.h"

static uint8_t  cycleCounter = 0;
struct tc_module tc_instance1,tc_instance2;

volatile bool tc_instance1_callback_flag;
volatile bool tc_instance2_callback_flag;

/**************************************************************************************/
/*****   Writes the Motors and Servos values to the Timer PWM compare registers   ******/
/**************************************************************************************/
void writeMotors() {
	
	// map motor values to conf.MINTHROTTLE, conf.MAXTHROTTLE
	// shit 3 or 0 bits left
	
	
      //uint16_t PWM_t; 
     // ################ first Update Motors (125us-250us or 1000us-2000us) #####################
          
    // ##########################  at last start AT32u4 ONESHOTS  ######################################
     // Nun bei allen 8 PWMs gleichzeitig Update erzwingen, ONESHOTS starten und Reset vorbereiten.
     //TCNT1 = 0xFFFF;  OCR1A=0; OCR1B=0; OCR1C=0;  // die drei Timer1-ONESHOTS starten
     //PORTF |= 1<<6;  PORTF |= 1<<7;               // set Ports PF6(ADC6), PF7(ADC7) for SW_Oneshots with Timer3
     //TCNT3 = 0xFFFF;  OCR3A=0; OCR3B=0; OCR3C=0;  // die drei Timer3-ONESHOTS starten
     //if(conf.sOneShot == 1){                   // die beiden Timer4-ONESHOTS starten
        //TCCR4B = B00000011;  // Start Timer4 mit Prescaler= 4 / Imp= 125.. 250us @9Bit, Bit.7=PWM4X clear !!! (PWM Inversion Mode)
     //}else{
        //TCCR4B = B00000110;  // Start Timer4 mit Prescaler=32 / Imp=1000..2000us @9Bit, Bit.7=PWM4X clear !!! (PWM Inversion Mode)
     //}
	 //tc_disable(&tc_instance1);
	 //tc_disable(&tc_instance2);
	 //tc_set_top_value(&tc_instance1,24000);
	 //tc_set_top_value(&tc_instance2,24000);
	 //tc_start_counter(&tc_instance1);
	 if(tc_instance2_callback_flag &&
		tc_instance2_callback_flag){
		 
		uint16_t timer_val = COUNT_MAX_16BIT-ONESHOT_MIN_PULSE;
		if(conf.sOneShot != 1) timer_val=COUNT_MAX_16BIT-PWM_MIN_PULSE; 
		
		tc_set_count_value(&tc_instance1,timer_val);
		tc_set_count_value(&tc_instance2,timer_val);
		
		tc_instance1_callback_flag = false;
		tc_instance2_callback_flag = false;
		
		if(conf.sOneShot == 0){
			tc_set_compare_value(&tc_instance1,0,COUNT_MAX_16BIT-PWM_MIN_PULSE+(motor[0]<<3));
			tc_set_compare_value(&tc_instance1,1,COUNT_MAX_16BIT-PWM_MIN_PULSE+(motor[1]<<3));
			tc_set_compare_value(&tc_instance2,0,COUNT_MAX_16BIT-PWM_MIN_PULSE+(motor[2]<<3));
			tc_set_compare_value(&tc_instance2,1,COUNT_MAX_16BIT-PWM_MIN_PULSE+(motor[3]<<3));
              //OCR1A = motor[0]<<4;   // Imp=1000.. 2000us @11Bit, Port B5 9
              //OCR1B = motor[1]<<4;   // Imp=1000.. 2000us @11Bit, Port B6 10
              //if(conf.copterType != 0) OCR1C = motor[4]<<4;   // Imp=1000.. 2000us @11Bit, Port B7 11
          }else{
			  tc_set_compare_value(&tc_instance1,0,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[0]);
			  tc_set_compare_value(&tc_instance1,1,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[1]);
			  tc_set_compare_value(&tc_instance2,0,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[2]);
			  tc_set_compare_value(&tc_instance2,1,COUNT_MAX_16BIT-ONESHOT_MIN_PULSE+motor[3]);
              //OCR1A = motor[0]<<1;   // Imp= 125.. 0250us @11Bit, Port B5 
              //OCR1B = motor[1]<<1;   // Imp= 125.. 0250us @11Bit, Port B6 
              //if(conf.copterType != 0) OCR1C = motor[4]<<1;   // Imp= 125.. 0250us @11Bit, Port B7
          }
          //if(conf.sOneShot == 0){      // Timer3 A 
              //if(conf.copterType > 1) OCR3A = motor[2]<<4;   // Imp=1000.. 2000us @11Bit, Port C6 5
          //}else{
              //if(conf.copterType > 1) OCR3A = motor[2]<<1;   // Imp= 125.. 0250us @11Bit, Port C6
          //}
        // ###### Update-Reihenfolge bei Timer4:  beide Compare-Register besetzen ######
          //PWM_t = 2047-motor[5];  TC4H = (PWM_t >> 8);  OCR4A = (unsigned char)(PWM_t);  // Port C7 13
          //if(conf.copterType != 1){ ;
            //PWM_t = 2047-motor[3];  TC4H = (PWM_t >> 8);  OCR4D = (unsigned char)(PWM_t);  // Port D7 6
          //}else{                    
            //PWM_t = 2047-motor[2];  TC4H = (PWM_t >> 8);  OCR4D = (unsigned char)(PWM_t); // Port D7 6
          //} 
          //if(conf.sOneShot == 0){    
              //OCR3B = motor[6]<<4;   // Imp=1000.. 2000us @11Bit, Port variable (z.Zt. PF5)
              //OCR3C = motor[7]<<4;   // Imp=1000.. 2000us @11Bit, Port variable (z.Zt. PF6)
          //}else{
              //OCR3B = motor[6]<<1;   // Imp= 125.. 250us @11Bit, Port variable 
              //OCR3C = motor[7]<<1;   // Imp= 125.. 250us @11Bit, Port variable
          //}
     // #####################  then Update Servos (fix 1000us-2000us) #############################
        //if ((cycleCounter == 1 && conf.DigiServo == 1) || (cycleCounter == 5 && conf.DigiServo == 0)){ 
          //if(conf.copterType == 0){
            //OCR1C = servo[4]<<4;   // Imp=900.. 2100us @11Bit
            //OCR3A = servo[5]<<4;   // Imp=900.. 2100us @11Bit
          //}
          //if(conf.copterType == 1){
            //OCR3A = servo[5]<<4;   // Imp=900.. 2100us @11Bit
          //}
          //cycleCounter = 0;
        //}else{
          //cycleCounter++;
        //}
		 
		tc_enable(&tc_instance1); 
		tc_enable(&tc_instance2);
		
	 }	 
	 //tc_enable(&tc_instance2);
}


/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void tc_instance1_callback (struct tc_module *const module_inst_ptr)
{
	tc_instance1_callback_flag = true;
}
void tc_instance2_callback (struct tc_module *const module_inst_ptr)
{
	tc_instance2_callback_flag = true;
}


void initOutput() {
	struct	tc_config config_tc1;
	tc_get_config_defaults(&config_tc1);
	config_tc1.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc1.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc1.clock_source = GCLK_GENERATOR_1;
	config_tc1.oneshot = true;
	config_tc1.pwm_channel[0].enabled = true;
	config_tc1.pwm_channel[0].pin_out = PIN_PA22F_TC4_WO0;
	config_tc1.pwm_channel[0].pin_mux = MUX_PA22F_TC4_WO0;
	config_tc1.pwm_channel[1].enabled = true;
	config_tc1.pwm_channel[1].pin_out = PIN_PA23F_TC4_WO1;
	config_tc1.pwm_channel[1].pin_mux = MUX_PA23F_TC4_WO1;
	tc_init(&tc_instance1, TC4, &config_tc1); //PA22,23
	tc_register_callback(&tc_instance1, tc_instance1_callback,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance1, TC_CALLBACK_OVERFLOW);
	tc_instance1_callback_flag = true;
	//tc_set_compare_value(&tc_instance1,0,COUNT_MAX_16BIT-24000+8000);
	//tc_set_compare_value(&tc_instance1,1,COUNT_MAX_16BIT-24000+8000);
	//tc_set_top_value(&tc_instance1,24000);
  
    struct	tc_config config_tc2;
    tc_get_config_defaults(&config_tc2);
    config_tc2.counter_size    = TC_COUNTER_SIZE_16BIT;
    config_tc2.wave_generation = TC_WAVE_GENERATION_NORMAL_PWM;
	config_tc2.clock_source = GCLK_GENERATOR_1;
	config_tc2.oneshot = true;
	config_tc2.pwm_channel[0].enabled = true;
    config_tc2.pwm_channel[0].pin_out = PIN_PB00F_TC7_WO0;
    config_tc2.pwm_channel[0].pin_mux = MUX_PB00F_TC7_WO0;
    config_tc2.pwm_channel[1].enabled = true;
    config_tc2.pwm_channel[1].pin_out = PIN_PB01F_TC7_WO1;
    config_tc2.pwm_channel[1].pin_mux = MUX_PB01F_TC7_WO1;
    tc_init(&tc_instance2, TC7, &config_tc2); //PB00,01
	tc_register_callback(&tc_instance2, tc_instance2_callback,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance2, TC_CALLBACK_OVERFLOW);
	tc_instance2_callback_flag = true;
	//tc_set_compare_value(&tc_instance2,0,COUNT_MAX_16BIT-8000);
	//tc_set_compare_value(&tc_instance2,1,COUNT_MAX_16BIT-8000);
	//tc_set_top_value(&tc_instance2,24000);
	
	/*struct	tc_config config_tc3;
    tc_get_config_defaults(&config_tc3);
    config_tc3.counter_size    = TC_COUNTER_SIZE_16BIT;
    config_tc3.wave_generation = TC_WAVE_GENERATION_MATCH_PWM;
	config_tc3.clock_source = GCLK_GENERATOR_1;
	config_tc3.oneshot = true;
    config_tc3.pwm_channel[1].enabled = true;
    config_tc3.pwm_channel[1].pin_out = PIN_PB02F_TC6_WO0;
    config_tc3.pwm_channel[1].pin_mux = MUX_PB02F_TC6_WO0;
    tc_init(&tc_instance3, TC6, &config_tc3); //PB03
	tc_register_callback(&tc_instance3, tc_instance3_callback,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance3, TC_CALLBACK_OVERFLOW);
	tc_instance3_callback_flag=true;
	tc_set_compare_value(&tc_instance3,1,8000);
	tc_set_top_value(&tc_instance3,24000);*/
	
	//pA24: TC5
	//PB02: TC6
  
	//tc_enable(&tc_instance1);
	//tc_enable(&tc_instance2);
  

}

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  int16_t minMotor;
  int16_t useThrottle;
  uint8_t i;
  
  if(s3D == 1){
    if ((rcData[THROTTLE]) > conf.s3DMIDDLE){
      useThrottle = constrain(rcData[THROTTLE], conf.s3DMIDDLE+conf.MIDDLEDEADBAND, conf.MAXTHROTTLE); 
    }else{
      useThrottle = constrain(rcData[THROTTLE], conf.MINCOMMAND, conf.s3DMIDDLE-conf.MIDDLEDEADBAND); 
    }
    axisPID[ROLL] = axisPID[ROLL]/2;
    axisPID[PITCH] = axisPID[PITCH]/2;
    axisPID[YAW] = axisPID[YAW]/2;
  }else if(conf.F3D == 1){
    useThrottle = constrain(((rcCommand[THROTTLE]-1000)>>1)+conf.s3DMIDDLE, conf.s3DMIDDLE+((conf.MINCOMMAND-1000)>>1), conf.MAXTHROTTLE); 
    if(f.ACC_MODE){
      useThrottle = useThrottle + (Zadd/20);
    }
    axisPID[ROLL] = axisPID[ROLL]/2;
    axisPID[PITCH] = axisPID[PITCH]/2;
    axisPID[YAW] = axisPID[YAW]/2;    
  }else{
    if(f.ACC_MODE){
      useThrottle = rcCommand[THROTTLE] + (Zadd/10);
    }else{
      useThrottle = rcCommand[THROTTLE];
    }
  }
  
  if (NUMBER_MOTOR > 3){
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  } 
  
  #define PIDMIX(X,Y,Z) useThrottle + axisPID[ROLL]*X + axisPID[PITCH]*Y + conf.YAW_DIRECTION * axisPID[YAW]*Z


  /****************                   main Mix Table                ******************/
  if(conf.copterType == 0){ // BI
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT        
    servo[4]  = constrain(conf.BILeftMiddle + ((conf.YAW_DIRECTION * axisPID[YAW]) + axisPID[PITCH])*conf.BiLeftDir, 900, 2100); //LEFT
    servo[5]  = constrain(conf.BIRightMiddle + ((conf.YAW_DIRECTION * axisPID[YAW]) - axisPID[PITCH])*conf.BiRightDir, 900, 2100); //RIGHT
  }else if(conf.copterType == 1){ // TRI
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = constrain(conf.TriYawMiddle + conf.YAW_DIRECTION * axisPID[YAW], 900, 2100); //REAR
  }else if(conf.copterType == 2){ // QUADP
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  }else if(conf.copterType == 3){ // QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  }else if(conf.copterType == 4){ // Y4
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  }else if(conf.copterType == 5){ // Y6
    motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT  
  }else if(conf.copterType == 6){ // HEXFP
    motor[0] = PIDMIX(-7/8,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-7/8,-1/2,-1); //FRONT_R
    motor[2] = PIDMIX(+7/8,+1/2,+1); //REAR_L
    motor[3] = PIDMIX(+7/8,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
    motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
  }else if(conf.copterType == 7){ // HEXFX
    motor[0] = PIDMIX(-1/2,+7/8,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-7/8,+1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+7/8,-1); //REAR_L
    motor[3] = PIDMIX(+1/2,-7/8,-1); //FRONT_L
    motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
    motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
  }else if(conf.copterType == 8){ // V Tail
    motor[0] = PIDMIX(+0,+1, +1); //REAR_R
    motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
    motor[2] = PIDMIX(+0,+1, -1); //REAR_L
    motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
  }
  
  /****************                Filter the Motors values                ******************/
  maxMotor=motor[0];
  minMotor=motor[0];
 
    
  if(s3D == 1){  
     for(i=1;i< NUMBER_MOTOR;i++){
      if (motor[i]>maxMotor) maxMotor=motor[i];
      if (motor[i]<minMotor) minMotor=motor[i];
    }
    for (i = 0; i < NUMBER_MOTOR; i++) {     
      if (maxMotor > conf.MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - conf.MAXTHROTTLE;    
        
      if (minMotor < conf.MINCOMMAND) // this is a way to still have good gyro corrections if at least one motor reaches its min.
        motor[i] += conf.MINCOMMAND - minMotor;   
  
      if ((rcData[THROTTLE]) > conf.s3DMIDDLE){
        motor[i] = constrain(motor[i], conf.s3DMIDDLE+conf.MIDDLEDEADBAND, conf.MAXTHROTTLE); 
      }else{
        motor[i] = constrain(motor[i], conf.MINCOMMAND, conf.s3DMIDDLE-conf.MIDDLEDEADBAND); 
      }
      if (!f.ARMED)
        motor[i] = conf.s3DMIDDLE;
    }
  }else if(conf.F3D == 1){
    for(i=1;i< NUMBER_MOTOR;i++){
      if (motor[i]>maxMotor) maxMotor=motor[i];
      if (motor[i]<minMotor) minMotor=motor[i];
    }
    for (i = 0; i < NUMBER_MOTOR; i++) {
      if (maxMotor > conf.MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - conf.MAXTHROTTLE;

      if (minMotor < conf.s3DMIDDLE+((conf.MINTHROTTLE-1000)>>1)) // this is a way to still have good gyro corrections if at least one motor reaches its min.
        motor[i] += conf.s3DMIDDLE+((conf.MINTHROTTLE-1000)>>1) - minMotor;  
        
        motor[i] = constrain(motor[i], conf.s3DMIDDLE+((conf.MINTHROTTLE-1000)>>1), conf.MAXTHROTTLE);    
/*      if ((rcData[THROTTLE]) < conf.MINCHECK)
        motor[i] = conf.s3DMIDDLE+((conf.MINTHROTTLE-1000)>>1);*/
      if (!f.ARMED){
        motor[i] = conf.s3DMIDDLE;
		if(!(usart_instance.hw->USART.CTRLA.reg & SERCOM_USART_CTRLA_ENABLE)){
			usart_initialize();	
		}
	  }else{
		usart_disable(&usart_instance);
	  }
    }     
  }else{
    for(i=1;i< NUMBER_MOTOR;i++){
      if (motor[i]>maxMotor) maxMotor=motor[i];
      if (motor[i]<minMotor) minMotor=motor[i];
    }
    for (i = 0; i < NUMBER_MOTOR; i++) {
      if (maxMotor > conf.MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - conf.MAXTHROTTLE;
        motor[i] = constrain(motor[i], conf.MINTHROTTLE, conf.MAXTHROTTLE);    

      if (minMotor < conf.MINTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its min.
        motor[i] += conf.MINTHROTTLE - minMotor;  
        
/*      if ((rcData[THROTTLE]) < conf.MINCHECK)
        motor[i] = conf.MINTHROTTLE;*/
      if (!f.ARMED){
        motor[i] = conf.MINCOMMAND;
		if(!(usart_instance.hw->USART.CTRLA.reg & SERCOM_USART_CTRLA_ENABLE)){
		  usart_initialize();
		}
	  }else{
	  usart_disable(&usart_instance);
	  }
    }   
  }
  if(throttleTest){
    for(i=0;i< NUMBER_MOTOR;i++){
      if(rcData[THROTTLE] > 1500){
        motor[i] = conf.MAXTHROTTLE;
      }else if(rcData[THROTTLE] < 1500){
        motor[i] = conf.MINCOMMAND;
      }else
        motor[i] = rcData[THROTTLE];
    }
  }
  
  if(conf.calibState){
    f.ARMED=0;
    f.OK_TO_ARM=0;
    rcOptions[BOXARM] = 0;
    if(!pTime){
      pTime=millis();
    }
    time=millis();
    if(conf.calibState==1){
      if(time-pTime<1000){
        for(int i=0;i< NUMBER_MOTOR;i++){
        motor[i] = conf.MAXTHROTTLE;
        }  
      }else{
        for(int i=0;i< NUMBER_MOTOR;i++){
        motor[i] = conf.MINCOMMAND;
        }
      }
      if(time-pTime>3000){
        conf.calibState=2;
        writeParams(1);
        while(true){
          //blinkLED(10,30,1); TBD
        }
      }
    }else if(conf.calibState==2){
      if(time-pTime<1000){
        for(int i=0;i< NUMBER_MOTOR;i++){
        motor[i] = conf.MAXTHROTTLE;
        }  
      }else{
        for(int i=0;i< NUMBER_MOTOR;i++){
        motor[i] = conf.s3DMIDDLE;
        }
      }
      if(time-pTime>3000){
        conf.calibState=0;
        writeParams(1);
        while(true){
          //blinkLED(10,30,1); TBD
        }
      }
    }  
  }

}