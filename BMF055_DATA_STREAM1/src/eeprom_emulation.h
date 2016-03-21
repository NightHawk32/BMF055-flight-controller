/*
 * eeprom.h
 *
 * Created: 06.03.2016 10:36:12
 *  Author: Lukas
 */ 

/*
 * globals.h
 *
 * Created: 05.03.2016 18:35:52
 *  Author: Lukas
 */ 




#ifndef EEPROM_EMULATION_H_
#define EEPROM_EMULATION_H_

#include "eeprom_emulator_support.h"

#define EEPROM_CONF_VERSION 91

void readEEPROM(void);
void writeParams(uint8_t b);
void checkFirstTime(uint8_t guiReset);

#endif /* EEPROM_EMULATION_H_ */