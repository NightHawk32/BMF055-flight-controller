#ifndef EEPROM_EMULATION_H_
#define EEPROM_EMULATION_H_

#include "eeprom_emulator_support.h"

#define EEPROM_CONF_VERSION 91

void readEEPROM(void);
void writeParams(uint8_t b);
void checkFirstTime(uint8_t guiReset);

#endif /* EEPROM_EMULATION_H_ */