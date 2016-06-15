/************************************************************************/
/* Include Own Header                                                   */
/************************************************************************/

#include "eeprom_emulator_support.h"

/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/

void eeprom_emulator_initialize(void)
{
	eeprom_emulator_configure();
}


void eeprom_emulator_configure(void)
{
	/* Setup EEPROM emulator service */
    enum status_code error_code = eeprom_emulator_init();
    if (error_code == STATUS_ERR_NO_MEMORY) {
        while (true) {
            /* No EEPROM section has been set in the device's fuses */
        }
    }
    else if (error_code != STATUS_OK) {
        /* Erase the emulated EEPROM memory (assume it is unformatted or
         * irrecoverably corrupt) */
        eeprom_emulator_erase_memory();
        eeprom_emulator_init();
    }
}