## INCLUDES FOR APPLICATION
APP_INCLUDEPATHS += \
src \
src/config \
src/Sensors \

## APPLICATION SOURCES: Sensors
APP_APP_SRC += \
src/Sensors/bma2x2_support.c \
src/Sensors/bma2x2.c \
src/Sensors/bmg160_support.c \
src/Sensors/bmg160.c \
src/Sensors/bmm050_support.c \
src/Sensors/bmm050.c \

## APPLICATION SOURCES: MultiWii 
APP_APP_SRC += \
src/bmf055.c \
src/eeprom_emulation.c \
src/imu.c \
src/main.c \
src/output.c \
src/rx.c \
src/sensors.c \
src/serial.c \
