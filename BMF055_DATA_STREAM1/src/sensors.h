#ifndef SENSORS_H_
#define SENSORS_H_

#include "bma2x2.h"
#include "bmg160.h"
#include "bmg160.h"
  
void GYRO_Common(void);
void ACC_Common(void);

void Gyro_init(void);
void Gyro_getADC (void);
void ACC_init (void);
void ACC_getADC (void);
void initSensors(void);

#endif