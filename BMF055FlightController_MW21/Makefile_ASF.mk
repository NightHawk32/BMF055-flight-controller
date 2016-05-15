##################### ASF COMPILER FLAGS AND DEFINES #########################
CFLAGS    += 	-DCONF_SPI_MASTER_ENABLE=true \
				-DSPI_CALLBACK_MODE=true \
				-DARM_MATH_CM0=true \
				-DEXTINT_CALLBACK_MODE=true \
				-DI2C_SLAVE_CALLBACK_MODE=true \
				-DADC_CALLBACK_MODE=true \
				-DTC_ASYNC=true \
				-DUSART_CALLBACK_MODE=true \
				-DWDT_CALLBACK_MODE=true



##################### ASF COMPILER FLAGS AND DEFINES #########################
ASF_INCLUDEPATHS += \
src \
src/ASF \
src/ASF_Support \
src/ASF/common \
src/ASF/common2 \
src/ASF/sam0 \
src/ASF/sam0/utils/ \
src/ASF/thirdparty \
src/ASF/common/boards \
src/ASF/common/services \
src/ASF/common/utils \
src/ASF/common/utils/interrupt \
src/ASF/common2/components \
src/ASF/common2/services \
src/ASF/common2/components/display \
src/ASF/common2/components/display/ssd1306 \
src/ASF/common2/services/delay \
src/ASF/common2/services/gfx_mono \
src/ASF/common2/services/delay/sam0 \
src/ASF/common2/services/gfx_mono/docsrc \
src/ASF/common2/services/gfx_mono/tools \
src/ASF/sam0/services/eeprom/emulator/main_array \
src/ASF/sam0/boards \
src/ASF/sam0/drivers \
src/ASF/sam0/boards/samd20_xplained_pro \
src/ASF/sam0/drivers/extint \
src/ASF/sam0/drivers/nvm \
src/ASF/sam0/drivers/port \
src/ASF/sam0/drivers/sercom \
src/ASF/sam0/drivers/system \
src/ASF/sam0/drivers/tc \
src/ASF/sam0/drivers/wdt \
src/ASF/sam0/drivers/extint/quick_start_callback \
src/ASF/sam0/drivers/extint/quick_start_polled \
src/ASF/sam0/drivers/port/quick_start \
src/ASF/sam0/drivers/sercom/i2c \
src/ASF/sam0/drivers/sercom/spi \
src/ASF/sam0/drivers/sercom/usart \
src/ASF/sam0/drivers/sercom/i2c/quick_start_slave \
src/ASF/sam0/drivers/sercom/i2c/quick_start_slave_callback \
src/ASF/sam0/drivers/sercom/spi/quick_start_master \
src/ASF/sam0/drivers/sercom/spi/quick_start_master_callback \
src/ASF/sam0/drivers/sercom/spi/quick_start_slave \
src/ASF/sam0/drivers/sercom/spi/quick_start_slave_callback \
src/ASF/sam0/drivers/sercom/usart/quick_start \
src/ASF/sam0/drivers/sercom/usart/quick_start_callback \
src/ASF/sam0/drivers/system/clock/clock_samd20 \
src/ASF/sam0/drivers/system/clock \
src/ASF/sam0/drivers/system/power/power_sam_d_r \
src/ASF/sam0/drivers/system/reset/reset_sam_d_r \
src/ASF/sam0/drivers/system/interrupt \
src/ASF/sam0/drivers/system/interrupt/system_interrupt_samd20 \
src/ASF/sam0/drivers/system/pinmux \
src/ASF/sam0/drivers/system/clock/quick_start_clock \
src/ASF/sam0/drivers/system/clock/quick_start_gclk \
src/ASF/sam0/drivers/system/interrupt/quick_start \
src/ASF/sam0/drivers/system/pinmux/quick_start \
src/ASF/sam0/drivers/tc/quick_start \
src/ASF/sam0/drivers/tc/quick_start_callback \
src/ASF/sam0/drivers/wdt/quick_start \
src/ASF/sam0/drivers/wdt/quick_start_callback \
src/ASF/sam0/utils/cmsis \
src/ASF/sam0/utils/header_files \
src/ASF/sam0/utils/make \
src/ASF/sam0/utils/preprocessor \
src/ASF/sam0/utils/syscalls \
src/ASF/sam0/utils/cmsis/samd20 \
src/ASF/sam0/utils/cmsis/samd20/include \
src/ASF/sam0/utils/cmsis/samd20/source \
src/ASF/sam0/utils/cmsis/samd20/include/component \
src/ASF/sam0/utils/cmsis/samd20/include/instance \
src/ASF/sam0/utils/cmsis/samd20/include/pio \
src/ASF/sam0/utils/cmsis/samd20/source/gcc \
src/ASF/sam0/utils/syscalls/gcc \
src/ASF/thirdparty/CMSIS \
src/ASF/thirdparty/demo \
src/ASF/thirdparty/CMSIS/Include \
src/ASF/common2/boards/user_board \
src/ASF_Support \
src/config


##################### ASF SOURCE FOR COMPLIE #########################
ASF_SRC += \
src/ASF/common/utils/interrupt/interrupt_sam_nvic.c \
src/ASF/common2/boards/user_board/init.c \
src/ASF/sam0/drivers/nvm/nvm.c \
src/ASF/sam0/drivers/port/port.c \
src/ASF/sam0/drivers/sercom/sercom_interrupt.c \
src/ASF/sam0/drivers/sercom/sercom.c \
src/ASF/sam0/drivers/sercom/spi/spi.c \
src/ASF/sam0/drivers/sercom/spi/spi_interrupt.c \
src/ASF/sam0/drivers/sercom/usart/usart_interrupt.c \
src/ASF/sam0/drivers/sercom/usart/usart.c \
src/ASF/sam0/drivers/system/clock/clock_samd20/clock.c \
src/ASF/sam0/drivers/system/clock/clock_samd20/gclk.c \
src/ASF/sam0/drivers/system/interrupt/system_interrupt.c \
src/ASF/sam0/drivers/system/pinmux/pinmux.c \
src/ASF/sam0/drivers/system/system.c \
src/ASF/sam0/drivers/tc/tc_sam_d_r/tc.c \
src/ASF/sam0/drivers/tc/tc_interrupt.c \
src/ASF/sam0/services/eeprom/emulator/main_array/eeprom.c \
src/ASF/sam0/utils/cmsis/samd20/source/system_samd20.c \
src/ASF_Support/clock_support.c \
src/ASF_Support/eeprom_emulator_support.c \
src/ASF_Support/spi_support.c \
src/ASF_Support/tc_support.c \
src/ASF_Support/usart_support.c \
