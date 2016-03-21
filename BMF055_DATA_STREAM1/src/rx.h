/*
 * rx.h
 *
 * Created: 06.03.2016 19:24:07
 *  Author: Lukas
 */ 

#ifndef RX_H_
#define RX_H_


void configureReceiver(void);
void rxInt(void);
void SpektrumISR(void);
uint16_t readRawRC(uint8_t chan);
void computeRC(void);


#endif