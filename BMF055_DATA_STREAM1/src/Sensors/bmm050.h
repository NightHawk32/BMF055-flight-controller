/*
****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* bmm050.h
* Date: 2014/10/17
* Revision: 2.0.2 $
*
* Usage: Sensor Driver for  BMM050 and BMM150 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/****************************************************************************/
/*! \file bmm050.h
    \brief BMM050 Sensor Driver Support Header File */

#ifndef __BMM050_H__
#define __BMM050_H__

/* For Enabling and Disabling the floating point API's */
#define ENABLE_FLOAT

/*******************************************************
*	These definition uses for define the data types
********************************************************
*	While porting the API please consider the following
*	Please check the version of C standard
*	Are you using Linux platform
*******************************************************/

/*********************************************************
*	This definition uses for the Linux platform support
*	Please use the types.h for your data types definitions
*********************************************************/
#ifdef	__KERNEL__

#include <linux/types.h>

#else /* ! __KERNEL__ */
/**********************************************************
*	These definition uses for define the C
*	standard version data types
***********************************************************/
# if defined(__STDC_VERSION__)

/************************************************
*	compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define	s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t
/************************************************
 *	compiler is C99 C standard
************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
which is used to fixed the integer size*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	u8	uint8_t
#define	u16	uint16_t
#define	u32	uint32_t
#define	u64	uint64_t

/*signed integer types*/
#define s8	int8_t
#define	s16	int16_t
#define	s32	int32_t
#define	s64	int64_t
/************************************************
 *	compiler is C89 or other C standard
************************************************/
#else /*  !defined(__STDC_VERSION__) */
/*	By default it is defined as 32 bit machine configuration*/
/*	define the definition based on your machine configuration*/
/*	define the data types based on your
	machine/compiler/controller configuration*/
#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define s64 long int
#define u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long int

#else
#warning The data types defined above which not supported \
define the data types manualy
#endif
#endif

/*** This else will execute for the compilers
 *	which are not supported the C standards
 *	Like C89/C99/C11***/
#else
/*	By default it is defined as 32 bit machine configuration*/
/*	define the definition based on your machine configuration*/
/*	define the data types based on your
	machine/compiler/controller configuration*/
#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define s64 long int
#define u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define s64 long long int
#define u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set s64 manually.
#endif

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined  MACHINE_64_BIT
/*signed integer types*/
#define	s8	signed char
#define	s16	signed short int
#define	s32	signed int
#define	s64	signed long int

/*unsigned integer types*/
#define	u8	unsigned char
#define	u16	unsigned short int
#define	u32	unsigned int
#define	u64	unsigned long int

#else
/* #warning The data types defined above which not supported \
define the data types manualy */

#include "compiler.h"

#define s8	S8
#define s16	S16
#define s32	S32
#define s64	S64
#define u8	U8
#define u16	U16
#define u32	U32
#define u64	U64

#endif
#endif
#endif

#define BMM050_BUS_WR_RETURN_TYPE s8
#define BMM050_BUS_WR_PARAM_TYPES \
u8, u8, u8 *, u8
#define BMM050_BUS_WR_PARAM_ORDER \
(device_addr, register_addr, register_data, wr_len)
#define BMM050_BUS_WRITE_FUNC( \
device_addr, register_addr, register_data, wr_len) \
bus_write(device_addr, register_addr, register_data, wr_len)

#define BMM050_BUS_RD_RETURN_TYPE s8

#define BMM050_BUS_RD_PARAM_TYPES \
u8, u8, u8 *, u8

#define BMM050_BUS_RD_PARAM_ORDER (device_addr, register_addr, register_data)

#define BMM050_BUS_READ_FUNC(device_addr, register_addr, register_data, rd_len)\
bus_read(device_addr, register_addr, register_data, rd_len)


#define BMM050_DELAY_RETURN_TYPE void

#define BMM050_DELAY_FUNC(delay_in_msec) \
delay_func(delay_in_msec)


#define BMM050_RETURN_FUNCTION_TYPE        s8
#define BMM050_I2C_ADDRESS                 0x10

/* Fixed Data Registers */
#define BMM050_CHIP_ID                     0x40
/* Data Registers*/
#define BMM050_DATAX_LSB                   0x42
#define BMM050_DATAX_MSB                   0x43
#define BMM050_DATAY_LSB                   0x44
#define BMM050_DATAY_MSB                   0x45
#define BMM050_DATAZ_LSB                   0x46
#define BMM050_DATAZ_MSB                   0x47
#define BMM050_R_LSB                       0x48
#define BMM050_R_MSB                       0x49

/* Data Registers for remapped axis(XandY)
 * this only applicable for BMX055 */
#define BMM050_BMX055_REMAPPED_DATAY_LSB      0x42
#define BMM050_BMX055_REMAPPED_DATAY_MSB      0x43
#define BMM050_BMX055_REMAPPED_DATAX_LSB      0x44
#define BMM050_BMX055_REMAPPED_DATAX_MSB      0x45

/* Status Registers */
#define BMM050_INT_STAT                    0x4A

/* Control Registers */
#define BMM050_POWER_CONTROL               0x4B
#define BMM050_CONTROL                     0x4C
#define BMM050_INT_CONTROL                 0x4D
#define BMM050_SENS_CONTROL                0x4E
#define BMM050_LOW_THRES                   0x4F
#define BMM050_HIGH_THRES                  0x50
#define BMM050_REP_XY                      0x51
#define BMM050_REP_Z                       0x52

/* Trim Extended Registers */
#define BMM050_DIG_X1                      0x5D
#define BMM050_DIG_Y1                      0x5E
#define BMM050_DIG_Z4_LSB                  0x62
#define BMM050_DIG_Z4_MSB                  0x63
#define BMM050_DIG_X2                      0x64
#define BMM050_DIG_Y2                      0x65
#define BMM050_DIG_Z2_LSB                  0x68
#define BMM050_DIG_Z2_MSB                  0x69
#define BMM050_DIG_Z1_LSB                  0x6A
#define BMM050_DIG_Z1_MSB                  0x6B
#define BMM050_DIG_XYZ1_LSB                0x6C
#define BMM050_DIG_XYZ1_MSB                0x6D
#define BMM050_DIG_Z3_LSB                  0x6E
#define BMM050_DIG_Z3_MSB                  0x6F
#define BMM050_DIG_XY2                     0x70
#define BMM050_DIG_XY1                     0x71


/* Data X LSB Register */
#define BMM050_DATAX_LSB_VALUEX__POS        3
#define BMM050_DATAX_LSB_VALUEX__LEN        5
#define BMM050_DATAX_LSB_VALUEX__MSK        0xF8
#define BMM050_DATAX_LSB_VALUEX__REG        BMM050_DATAX_LSB

/* Data X SelfTest Register */
#define BMM050_DATAX_LSB_TESTX__POS         0
#define BMM050_DATAX_LSB_TESTX__LEN         1
#define BMM050_DATAX_LSB_TESTX__MSK         0x01
#define BMM050_DATAX_LSB_TESTX__REG         BMM050_DATAX_LSB

/* Data Y LSB Register */
#define BMM050_DATAY_LSB_VALUEY__POS        3
#define BMM050_DATAY_LSB_VALUEY__LEN        5
#define BMM050_DATAY_LSB_VALUEY__MSK        0xF8
#define BMM050_DATAY_LSB_VALUEY__REG        BMM050_DATAY_LSB

/* Data Y SelfTest Register */
#define BMM050_DATAY_LSB_TESTY__POS         0
#define BMM050_DATAY_LSB_TESTY__LEN         1
#define BMM050_DATAY_LSB_TESTY__MSK         0x01
#define BMM050_DATAY_LSB_TESTY__REG         BMM050_DATAY_LSB

/* Data Z LSB Register */
#define BMM050_DATAZ_LSB_VALUEZ__POS        1
#define BMM050_DATAZ_LSB_VALUEZ__LEN        7
#define BMM050_DATAZ_LSB_VALUEZ__MSK        0xFE
#define BMM050_DATAZ_LSB_VALUEZ__REG        BMM050_DATAZ_LSB

/* Data Z SelfTest Register */
#define BMM050_DATAZ_LSB_TESTZ__POS         0
#define BMM050_DATAZ_LSB_TESTZ__LEN         1
#define BMM050_DATAZ_LSB_TESTZ__MSK         0x01
#define BMM050_DATAZ_LSB_TESTZ__REG         BMM050_DATAZ_LSB

/* Hall Resistance LSB Register */
#define BMM050_R_LSB_VALUE__POS             2
#define BMM050_R_LSB_VALUE__LEN             6
#define BMM050_R_LSB_VALUE__MSK             0xFC
#define BMM050_R_LSB_VALUE__REG             BMM050_R_LSB

#define BMM050_DATA_RDYSTAT__POS            0
#define BMM050_DATA_RDYSTAT__LEN            1
#define BMM050_DATA_RDYSTAT__MSK            0x01
#define BMM050_DATA_RDYSTAT__REG            BMM050_R_LSB

/* Data X LSB Remapped Register only applicable for BMX055 */
#define BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX__POS        3
#define BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX__LEN        5
#define BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX__MSK        0xF8
#define BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX__REG\
BMM050_BMX055_REMAPPED_DATAX_LSB

/* Data Y LSB Remapped Register only applicable for BMX055  */
#define BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY__POS        3
#define BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY__LEN        5
#define BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY__MSK        0xF8
#define BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY__REG\
BMM050_BMX055_REMAPPED_DATAY_LSB

/* Interrupt Status Register */
#define BMM050_INT_STAT_DOR__POS            7
#define BMM050_INT_STAT_DOR__LEN            1
#define BMM050_INT_STAT_DOR__MSK            0x80
#define BMM050_INT_STAT_DOR__REG            BMM050_INT_STAT

#define BMM050_INT_STAT_OVRFLOW__POS        6
#define BMM050_INT_STAT_OVRFLOW__LEN        1
#define BMM050_INT_STAT_OVRFLOW__MSK        0x40
#define BMM050_INT_STAT_OVRFLOW__REG        BMM050_INT_STAT

#define BMM050_INT_STAT_HIGH_THZ__POS       5
#define BMM050_INT_STAT_HIGH_THZ__LEN       1
#define BMM050_INT_STAT_HIGH_THZ__MSK       0x20
#define BMM050_INT_STAT_HIGH_THZ__REG       BMM050_INT_STAT

#define BMM050_INT_STAT_HIGH_THY__POS       4
#define BMM050_INT_STAT_HIGH_THY__LEN       1
#define BMM050_INT_STAT_HIGH_THY__MSK       0x10
#define BMM050_INT_STAT_HIGH_THY__REG       BMM050_INT_STAT

#define BMM050_INT_STAT_HIGH_THX__POS       3
#define BMM050_INT_STAT_HIGH_THX__LEN       1
#define BMM050_INT_STAT_HIGH_THX__MSK       0x08
#define BMM050_INT_STAT_HIGH_THX__REG       BMM050_INT_STAT

#define BMM050_INT_STAT_LOW_THZ__POS        2
#define BMM050_INT_STAT_LOW_THZ__LEN        1
#define BMM050_INT_STAT_LOW_THZ__MSK        0x04
#define BMM050_INT_STAT_LOW_THZ__REG        BMM050_INT_STAT

#define BMM050_INT_STAT_LOW_THY__POS        1
#define BMM050_INT_STAT_LOW_THY__LEN        1
#define BMM050_INT_STAT_LOW_THY__MSK        0x02
#define BMM050_INT_STAT_LOW_THY__REG        BMM050_INT_STAT

#define BMM050_INT_STAT_LOW_THX__POS        0
#define BMM050_INT_STAT_LOW_THX__LEN        1
#define BMM050_INT_STAT_LOW_THX__MSK        0x01
#define BMM050_INT_STAT_LOW_THX__REG        BMM050_INT_STAT

/* Power Control Register */
#define BMM050_POWER_CONTROL_SOFT_RST_7__POS       7
#define BMM050_POWER_CONTROL_SOFT_RST_7__LEN       1
#define BMM050_POWER_CONTROL_SOFT_RST_7__MSK       0x80
#define BMM050_POWER_CONTROL_SOFT_RST_7__REG       BMM050_POWER_CONTROL

#define BMM050_POWER_CONTROL_SPI3_ENABLE__POS     2
#define BMM050_POWER_CONTROL_SPI3_ENABLE__LEN     1
#define BMM050_POWER_CONTROL_SPI3_ENABLE__MSK     0x04
#define BMM050_POWER_CONTROL_SPI3_ENABLE__REG     BMM050_POWER_CONTROL

#define BMM050_POWER_CONTROL_SOFT_RST_1__POS       1
#define BMM050_POWER_CONTROL_SOFT_RST_1__LEN       1
#define BMM050_POWER_CONTROL_SOFT_RST_1__MSK       0x02
#define BMM050_POWER_CONTROL_SOFT_RST_1__REG       BMM050_POWER_CONTROL

#define BMM050_POWER_CONTROL_POWER_CONTROL_BIT__POS         0
#define BMM050_POWER_CONTROL_POWER_CONTROL_BIT__LEN         1
#define BMM050_POWER_CONTROL_POWER_CONTROL_BIT__MSK         0x01
#define BMM050_POWER_CONTROL_POWER_CONTROL_BIT__REG         BMM050_POWER_CONTROL

/* Control Register */
#define BMM050_CONTROL_ADVANCED_SELFTEST__POS            6
#define BMM050_CONTROL_ADVANCED_SELFTEST__LEN            2
#define BMM050_CONTROL_ADVANCED_SELFTEST__MSK            0xC0
#define BMM050_CONTROL_ADVANCED_SELFTEST__REG            BMM050_CONTROL

#define BMM050_CONTROL_DATA_RATE__POS                3
#define BMM050_CONTROL_DATA_RATE__LEN                3
#define BMM050_CONTROL_DATA_RATE__MSK                0x38
#define BMM050_CONTROL_DATA_RATE__REG                BMM050_CONTROL

#define BMM050_CONTROL_OPERATION_MODE__POS            1
#define BMM050_CONTROL_OPERATION_MODE__LEN            2
#define BMM050_CONTROL_OPERATION_MODE__MSK            0x06
#define BMM050_CONTROL_OPERATION_MODE__REG            BMM050_CONTROL

#define BMM050_CONTROL_SELFTEST__POS            0
#define BMM050_CONTROL_SELFTEST__LEN            1
#define BMM050_CONTROL_SELFTEST__MSK            0x01
#define BMM050_CONTROL_SELFTEST__REG            BMM050_CONTROL

/* Interrupt Control Register */
#define BMM050_INT_CONTROL_DOR_EN__POS            7
#define BMM050_INT_CONTROL_DOR_EN__LEN            1
#define BMM050_INT_CONTROL_DOR_EN__MSK            0x80
#define BMM050_INT_CONTROL_DOR_EN__REG            BMM050_INT_CONTROL

#define BMM050_INT_CONTROL_OVRFLOW_EN__POS        6
#define BMM050_INT_CONTROL_OVRFLOW_EN__LEN        1
#define BMM050_INT_CONTROL_OVRFLOW_EN__MSK        0x40
#define BMM050_INT_CONTROL_OVRFLOW_EN__REG        BMM050_INT_CONTROL

#define BMM050_INT_CONTROL_HIGH_THZ_EN__POS       5
#define BMM050_INT_CONTROL_HIGH_THZ_EN__LEN       1
#define BMM050_INT_CONTROL_HIGH_THZ_EN__MSK       0x20
#define BMM050_INT_CONTROL_HIGH_THZ_EN__REG       BMM050_INT_CONTROL

#define BMM050_INT_CONTROL_HIGH_THY_EN__POS       4
#define BMM050_INT_CONTROL_HIGH_THY_EN__LEN       1
#define BMM050_INT_CONTROL_HIGH_THY_EN__MSK       0x10
#define BMM050_INT_CONTROL_HIGH_THY_EN__REG       BMM050_INT_CONTROL

#define BMM050_INT_CONTROL_HIGH_THX_EN__POS       3
#define BMM050_INT_CONTROL_HIGH_THX_EN__LEN       1
#define BMM050_INT_CONTROL_HIGH_THX_EN__MSK       0x08
#define BMM050_INT_CONTROL_HIGH_THX_EN__REG       BMM050_INT_CONTROL

#define BMM050_INT_CONTROL_LOW_THZ_EN__POS        2
#define BMM050_INT_CONTROL_LOW_THZ_EN__LEN        1
#define BMM050_INT_CONTROL_LOW_THZ_EN__MSK        0x04
#define BMM050_INT_CONTROL_LOW_THZ_EN__REG        BMM050_INT_CONTROL

#define BMM050_INT_CONTROL_LOW_THY_EN__POS        1
#define BMM050_INT_CONTROL_LOW_THY_EN__LEN        1
#define BMM050_INT_CONTROL_LOW_THY_EN__MSK        0x02
#define BMM050_INT_CONTROL_LOW_THY_EN__REG        BMM050_INT_CONTROL

#define BMM050_INT_CONTROL_LOW_THX_EN__POS        0
#define BMM050_INT_CONTROL_LOW_THX_EN__LEN        1
#define BMM050_INT_CONTROL_LOW_THX_EN__MSK        0x01
#define BMM050_INT_CONTROL_LOW_THX_EN__REG        BMM050_INT_CONTROL

/* Sensor Control Register */
#define BMM050_SENS_CONTROL_DRDY_EN__POS          7
#define BMM050_SENS_CONTROL_DRDY_EN__LEN          1
#define BMM050_SENS_CONTROL_DRDY_EN__MSK          0x80
#define BMM050_SENS_CONTROL_DRDY_EN__REG          BMM050_SENS_CONTROL

#define BMM050_SENS_CONTROL_IE__POS               6
#define BMM050_SENS_CONTROL_IE__LEN               1
#define BMM050_SENS_CONTROL_IE__MSK               0x40
#define BMM050_SENS_CONTROL_IE__REG               BMM050_SENS_CONTROL

#define BMM050_SENS_CONTROL_CHANNELZ__POS         5
#define BMM050_SENS_CONTROL_CHANNELZ__LEN         1
#define BMM050_SENS_CONTROL_CHANNELZ__MSK         0x20
#define BMM050_SENS_CONTROL_CHANNELZ__REG         BMM050_SENS_CONTROL

#define BMM050_SENS_CONTROL_CHANNELY__POS         4
#define BMM050_SENS_CONTROL_CHANNELY__LEN         1
#define BMM050_SENS_CONTROL_CHANNELY__MSK         0x10
#define BMM050_SENS_CONTROL_CHANNELY__REG         BMM050_SENS_CONTROL

#define BMM050_SENS_CONTROL_CHANNELX__POS         3
#define BMM050_SENS_CONTROL_CHANNELX__LEN         1
#define BMM050_SENS_CONTROL_CHANNELX__MSK         0x08
#define BMM050_SENS_CONTROL_CHANNELX__REG         BMM050_SENS_CONTROL

#define BMM050_SENS_CONTROL_DR_POLARITY__POS      2
#define BMM050_SENS_CONTROL_DR_POLARITY__LEN      1
#define BMM050_SENS_CONTROL_DR_POLARITY__MSK      0x04
#define BMM050_SENS_CONTROL_DR_POLARITY__REG      BMM050_SENS_CONTROL

#define BMM050_SENS_CONTROL_INTERRUPT_LATCH__POS            1
#define BMM050_SENS_CONTROL_INTERRUPT_LATCH__LEN            1
#define BMM050_SENS_CONTROL_INTERRUPT_LATCH__MSK            0x02
#define BMM050_SENS_CONTROL_INTERRUPT_LATCH__REG            BMM050_SENS_CONTROL

#define BMM050_SENS_CONTROL_INTERRUPT_POLARITY__POS         0
#define BMM050_SENS_CONTROL_INTERRUPT_POLARITY__LEN         1
#define BMM050_SENS_CONTROL_INTERRUPT_POLARITY__MSK         0x01
#define BMM050_SENS_CONTROL_INTERRUPT_POLARITY__REG         BMM050_SENS_CONTROL

/* Register 6D */
#define BMM050_DIG_XYZ1_MSB__POS         0
#define BMM050_DIG_XYZ1_MSB__LEN         7
#define BMM050_DIG_XYZ1_MSB__MSK         0x7F
#define BMM050_DIG_XYZ1_MSB__REG         BMM050_DIG_XYZ1_MSB
/*****************************************************************/

/*General Info data's*/
#define BMM050_SOFT_RESET7_ON              1
#define BMM050_SOFT_RESET1_ON              1
#define BMM050_SOFT_RESET7_OFF             0
#define BMM050_SOFT_RESET1_OFF             0
#define BMM050_DELAY_SOFTRESET             1

/** Error code definitions**/
#define E_BMM050_NULL_PTR           ((s8)-127)
#define BMM050_ERROR				((s8)-1)
#define E_BMM050_OUT_OF_RANGE       ((s8)-2)
#define BMM050_NULL                 ((u8)0)
#define E_BMM050_UNDEFINED_MODE        0

/* Constants */
#define BMM050_DELAY_POWEROFF_SUSPEND      1
#define BMM050_DELAY_SUSPEND_SLEEP         2
#define BMM050_DELAY_SLEEP_ACTIVE          1
#define BMM050_DELAY_ACTIVE_SLEEP          1
#define BMM050_DELAY_SLEEP_SUSPEND         1
#define BMM050_DELAY_ACTIVE_SUSPEND        1
#define BMM050_DELAY_SLEEP_POWEROFF        1
#define BMM050_DELAY_ACTIVE_POWEROFF       1
#define BMM050_DELAY_SETTLING_TIME         2

#define BMM050_X_AXIS               0
#define BMM050_Y_AXIS               1
#define BMM050_Z_AXIS               2
#define BMM050_RESISTANCE           3
#define BMM050_X                    1
#define BMM050_Y                    2
#define BMM050_Z                    4
#define BMM050_XYZ                  7

#define BMM050_ZERO_U8X                         0
#define BMM050_DISABLE                          0
#define BMM050_ENABLE                           1
#define BMM050_CHANNEL_DISABLE                  1
#define BMM050_CHANNEL_ENABLE                   0
#define BMM050_INTPIN_LATCH_ENABLE              1
#define BMM050_INTPIN_LATCH_DISABLE             0
#define BMM050_OFF                              0
#define BMM050_ON                               1

#define BMM050_NORMAL_MODE                      0x00
#define BMM050_FORCED_MODE                      0x01
#define BMM050_SUSPEND_MODE                     0x02
#define BMM050_SLEEP_MODE                       0x03

#define BMM050_ADVANCED_SELFTEST_OFF            0
#define BMM050_ADVANCED_SELFTEST_NEGATIVE       2
#define BMM050_ADVANCED_SELFTEST_POSITIVE       3

#define BMM050_NEGATIVE_SATURATION_Z            -32767
#define BMM050_POSITIVE_SATURATION_Z            32767

#define BMM050_SPI_RD_MASK                      0x80
#define BMM050_READ_SET                         0x01

/* Bus read and bus write */
#define BMM050_WR_FUNC_PTR \
	s8 (*bus_write)(u8, u8, \
	u8 *, u8)

#define BMM050_RD_FUNC_PTR \
	s8 (*bus_read)(u8, u8, \
	u8 *, u8)

#define BMM050_MDELAY_DATA_TYPE		u32

/*Shifting Constants*/
#define SHIFT_RIGHT_1_POSITION                  1
#define SHIFT_RIGHT_2_POSITION                  2
#define SHIFT_RIGHT_3_POSITION                  3
#define SHIFT_RIGHT_4_POSITION                  4
#define SHIFT_RIGHT_5_POSITION                  5
#define SHIFT_RIGHT_6_POSITION                  6
#define SHIFT_RIGHT_7_POSITION                  7
#define SHIFT_RIGHT_8_POSITION                  8

#define SHIFT_LEFT_1_POSITION                   1
#define SHIFT_LEFT_2_POSITION                   2
#define SHIFT_LEFT_3_POSITION                   3
#define SHIFT_LEFT_4_POSITION                   4
#define SHIFT_LEFT_5_POSITION                   5
#define SHIFT_LEFT_6_POSITION                   6
#define SHIFT_LEFT_7_POSITION                   7
#define SHIFT_LEFT_8_POSITION                   8

/* Conversion factors*/
#define BMM050_CONVFACTOR_LSB_UT                6

/* get bit slice  */
#define BMM050_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

/* Set bit slice */
#define BMM050_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/* compensated output value returned if sensor had overflow */
#define BMM050_OVERFLOW_OUTPUT			-32768
#define BMM050_OVERFLOW_OUTPUT_S32		((s32)(-2147483647-1))
#define BMM050_OVERFLOW_OUTPUT_FLOAT	0.0f
#define BMM050_FLIP_OVERFLOW_ADCVAL		-4096
#define BMM050_HALL_OVERFLOW_ADCVAL		-16384


#define BMM050_PRESETMODE_LOWPOWER                  1
#define BMM050_PRESETMODE_REGULAR                   2
#define BMM050_PRESETMODE_HIGHACCURACY              3
#define BMM050_PRESETMODE_ENHANCED                  4

/* PRESET MODES - DATA RATES */
#define BMM050_LOWPOWER_DR                       BMM050_DR_10HZ
#define BMM050_REGULAR_DR                        BMM050_DR_10HZ
#define BMM050_HIGHACCURACY_DR                   BMM050_DR_20HZ
#define BMM050_ENHANCED_DR                       BMM050_DR_10HZ

/* PRESET MODES - REPETITIONS-XY RATES */
#define BMM050_LOWPOWER_REPXY                     1
#define BMM050_REGULAR_REPXY                      4
#define BMM050_HIGHACCURACY_REPXY                23
#define BMM050_ENHANCED_REPXY                     7

/* PRESET MODES - REPETITIONS-Z RATES */
#define BMM050_LOWPOWER_REPZ                      2
#define BMM050_REGULAR_REPZ                      14
#define BMM050_HIGHACCURACY_REPZ                 82
#define BMM050_ENHANCED_REPZ                     26

/* Data Rates */

#define BMM050_DR_10HZ                     0
#define BMM050_DR_02HZ                     1
#define BMM050_DR_06HZ                     2
#define BMM050_DR_08HZ                     3
#define BMM050_DR_15HZ                     4
#define BMM050_DR_20HZ                     5
#define BMM050_DR_25HZ                     6
#define BMM050_DR_30HZ                     7

/*user defined Structures*/
struct bmm050_mag_data_s16_t {
	s16 datax;
	s16 datay;
	s16 dataz;
	u16 resistance;
};
struct bmm050_mag_s32_data_t {
	s32 datax;
	s32 datay;
	s32 dataz;
	u16 resistance;
};
struct bmm050_mag_data_float_t {
	float datax;
	float datay;
	float  dataz;
	u16 resistance;
};

/*user defined Structures for remapped functions
 * this only applicable for BMX055*/
struct bmm050_remapped_mag_s16_data_t {
	s16 datax;
	s16 datay;
	s16 dataz;
	u16 resistance;
};
struct bmm050_remapped_mag_s32_data_t {
	s32 datax;
	s32 datay;
	s32 dataz;
	u16 resistance;
};
struct bmm050_remapped_mag_data_float_t {
	float datax;
	float datay;
	float  dataz;
	u16 resistance;
};
struct bmm050 {
	u8 company_id;
	u8 dev_addr;

	BMM050_WR_FUNC_PTR;
	BMM050_RD_FUNC_PTR;
	void(*delay_msec)(BMM050_MDELAY_DATA_TYPE);

	s8 dig_x1;
	s8 dig_y1;

	s8 dig_x2;
	s8 dig_y2;

	u16 dig_z1;
	s16 dig_z2;
	s16 dig_z3;
	s16 dig_z4;

	u8 dig_xy1;
	s8 dig_xy2;

	u16 dig_xyz1;
};
/*****************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x40 bit from 0 to 7
 *
 *	 \param  bmm050 *p_bmm050 structure pointer
 *
 *	While changing the parameter of the bmm050
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_init(struct bmm050 *bmm050);
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                          data of X,Y,Z values and RHALL values
 *                          from location 42h to 49h
 *
 *
 *
 *
 *  \param bmm050_mag_raw_data_t * mag_data :
 *	pointer holding the data of mag raw data
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_mag_data_XYZ(
struct bmm050_mag_data_s16_t *mag_data);
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                          data of X,Y,Z values and RHALL values
 *                          from location 42h to 49h
 *	In this function X and Y axis is remapped
 *	X is read from the address 0x44 & 0x45
 *	Y is read from the address 0x42 & 0x43
 *	this API is only applicable for BMX055 sensor
 *
 *
 *
 *  \param bmm050_remapped_mag_s16_data_t * mag_data :
 *	pointer holding the data of mag data
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_bmx055_remapped_mag_data_XYZ(
struct bmm050_remapped_mag_s16_data_t *mag_data);
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                   data of X,Y,Z values and RHALL values
 *                   from location 42h to 49h
 *	The output value of compensated X, Y, Z and R as s32
 *
 *
 *  \param bmm050_mag_s32_data_t * mag_data :
 *	pointer holding the data of bmm050_mdata
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_mag_data_XYZ_s32(
struct bmm050_mag_s32_data_t *mag_data);
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                   data of X,Y,Z values and RHALL values
 *                   from location 42h to 49h
 *	The output value of compensated X, Y, Z and R as s32
 *	In this function X and Y axis is remapped
 *	X is read from the address 0x44 & 0x45
 *	Y is read from the address 0x42 & 0x43
 *	this API is only applicable for BMX055 sensor
 *
 *
 *  \param bmm050_mdata * mag_data : pointer holding the data of bmm050_mdata
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_bmx055_remapped_mag_data_XYZ_s32(
struct bmm050_remapped_mag_s32_data_t *mag_data);
#ifdef ENABLE_FLOAT
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                   data of X,Y,Z values and RHALL values
 *                   from location 42h to 49h
 *	The output value of compensated X, Y, Z and R as float
 *
 *
 *
 *  \param bmm050_mdata * mag_data : pointer holding the data of bmm050_mdata
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_mag_data_XYZ_float(
struct bmm050_mag_data_float_t *mag_data);
#endif
#ifdef ENABLE_FLOAT
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                   data of X,Y,Z values and RHALL values
 *                   from location 42h to 49h
 *	The output value of compensated X, Y, Z and R as float
 *
 *	In this function X and Y axis is remapped
 *	X is read from the address 0x44 & 0x45
 *	Y is read from the address 0x42 & 0x43
 *	this API is only applicable for BMX055 sensor
 *
 *  \param bmm050_remapped_mag_data_float_t * mag_data :
 *		pointer holding the data of mag
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_bmx055_remapped_mag_data_XYZ_float(
struct bmm050_remapped_mag_data_float_t *mag_data);
#endif
/*****************************************************************************
 * Description: *//**\brief This API reads the data from
 *           the given register address
 *
 *
 *
 *
 *  \param u8 v_addr_u8, u8 *v_data_u8, u8 v_len_u8
 *         v_addr_u8 -> Address of the register
 *         data -> address of the variable, read value will be kept
 *         v_len_u8  -> Length of the data
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8);
/*****************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                the data is written in the corresponding register address
 *
 *
 *
 *
 *  \param u8 v_addr_u8, u8 v_data_u8, u8 v_len_u8
 *          v_addr_u8 -> Address of the register
 *          v_data_u8 -> Data to be written to the register
 *          v_len_u8  -> Length of the Data
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the self test of the sensor
 *	in the register 0x4C bit 0
 *
 *
 *
 *  \param u8 v_selftest_u8 : The value of selftest
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_selftest(u8 v_selftest_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to read the selftest of the sensor
 *
 *
 *
 *
 *  \param u8 *v_selftest_xyz:
 *	Pointer holding the self test value of X,Y and Z
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_selftest_XYZ(
u8 *v_selftest_xyz);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the spi3
 *	in the register 0x4B bit 2
 *
 *
 *
 *  \param u8 v_value_u8 : the value of spi3
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_spi3(u8 v_value_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the data rate of the sensor
 *	in the register 0x4C bit 3 to 5
 *
 *
 *
 *  \param u8 v_data_rate_u8 : The value of v_data_rate_u8
 *	v_data_rate_u8 ->
 *		000		-	10
 *		001		-	2
 *		010		-	6
 *		011		-	8
 *		100		-	15
 *		101		-	20
 *		110		-	25
 *		111		-	30
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_data_rate(u8 v_data_rate_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the data rate of the sensor
 *	in the register 0x4C bit 3 to 5
 *
 *
 *
 *  \param u8 *v_data_rate_u8 : pointer holding the value of data rate
 *		v_data_rate_u8 ->
 *		000		-	10
 *		001		-	2
 *		010		-	6
 *		011		-	8
 *		100		-	15
 *		101		-	20
 *		110		-	25
 *		111		-	30
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_data_rate(u8 *v_data_rate_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the functional state
 *	in the register 0x4C bit 1 and 2
 *
 *
 *
 *
 *  \param u8 v_functional_state_u8: The value of functional mode
 *	BMM050_NORMAL_MODE		-> 0x00
 *	BMM050_SUSPEND_MODE		-> 0x01
 *	BMM050_FORCED_MODE		-> 0x02
 *	BMM050_SLEEP_MODE		-> 0x03
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_functional_state(
u8 v_functional_state_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the functional state
 *	in the register 0x4C bit 1 and 2
 *
 *
 *
 *
 *  \param u8 *v_functional_state_u8:
 *	pointer holding the value of functional mode
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_functional_state(
u8 *v_functional_state_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to perform the
 *	advanced self test
 *
 *
 *
 *  \param s16 *v_diff_z_s16 : pointer holding the output of advance self test
 *
 *	\return the output of advance self test
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_perform_advanced_selftest(
s16 *v_diff_z_s16);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the trim values
 *
 *
 *
 *
 *  \param u8 : None
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_init_trim_registers(void);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the advanced self test
 *	in the register 0x4C bit 6 and 7
 *
 *
 *
 *  \param u8 v_advanced_selftest_u8 : the value of advanced self test
 *	v_advanced_selftest_u8 ->
 *		BMM050_ADVANCED_SELFTEST_OFF			0
 *		BMM050_ADVANCED_SELFTEST_NEGATIVE       2
 *		BMM050_ADVANCED_SELFTEST_POSITIVE       3
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_advanced_selftest(
u8 v_advanced_selftest_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the advanced self test
 *	in the register 0x4C bit 6 and 7
 *
 *
 *
 *  \param u8 *v_advanced_selftest_u8 : pointer holding
 *				the value of advanced self test
 *	v_advanced_selftest_u8 ->
 *		BMM050_ADVANCED_SELFTEST_OFF			0
 *		BMM050_ADVANCED_SELFTEST_NEGATIVE       2
 *		BMM050_ADVANCED_SELFTEST_POSITIVE       3
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_advanced_selftest(
u8 *v_advanced_selftest_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the power control bit
 *	in the register 0x4B bit 0
 *
 *
 *
 *  \param u8 *v_power_mode_u8 : pointer holding the value of power control bit
 *	v_power_mode_u8 - > 0 disable the power control bit
 *		 - > 1 enable the power control bit
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_power_mode(u8 *v_power_mode_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the power control bit
 *	in the register 0x4B bit 0
 *
 *
 *
 *  \param u8 v_power_mode_u8 : the value of power control bit
 *	v_power_mode_u8 - > 0 disable the power control bit
 *		 - > 1 enable the power control bit
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_power_mode(u8 v_power_mode_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the x and y
 *	repetition in the register 0x51 bit 0 to 7
 *
 *
 *
 *  \param u8 *v_rep_xy_u8 : pointer holding the
 *	value of x and y repetitions
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_rep_XY(
u8 *v_rep_xy_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the x and y
 *	repetition in the register 0x51 bit 0 to 7
 *
 *
 *
 *  \param u8 v_rep_xy_u8 : The value of x and y repetitions
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_rep_XY(
u8 v_rep_xy_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the z repetition in the
 *	register 0x52 bit 0 to 7
 *
 *
 *
 *  \param u8 *v_rep_z_u8 : pointer holding the value of z repetitions
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_rep_Z(
u8 *v_rep_z_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the z repetition in the
 *	register 0x52 bit 0 to 7
 *
 *
 *
 *  \param u8 v_rep_z_u8 : The value of z repetitions
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_rep_Z(
u8 v_rep_z_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the preset modes
 *	The preset mode setting is depend on data rate, xy and z repetitions
 *
 *
 *
 *  \param u8 *v_presetmode_u8: Pointer holding the value of presetmode
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_presetmode(
u8 *v_presetmode_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the pre-set modes
 *	The pre-set mode setting is depend on data rate, xy and z repetitions
 *
 *
 *
 *  \param u8 v_presetmode_u8: The value of preset mode selection value
 *	BMM050_PRESETMODE_LOWPOWER		-> 1
 *	BMM050_PRESETMODE_REGULAR		-> 2
 *	BMM050_PRESETMODE_HIGHACCURACY	-> 3
 *	BMM050_PRESETMODE_ENHANCED		-> 4
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_presetmode(u8 v_presetmode_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated X data
 *	the out put of X as s16
 *
 *
 *
 *  \param s16 mag_data_x : The value of X data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated X data value output as s16
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s16 bmm050_compensate_X(s16 mag_data_x, u16 data_r);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated X data
 *	the out put of X as s32
 *
 *
 *
 *  \param s16 mag_data_x : The value of X data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated X data value output as s32
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmm050_compensate_X_s32(s16 mag_data_x, u16 data_r);
#ifdef ENABLE_FLOAT
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated X data
 *	the out put of X as float
 *
 *
 *
 *  \param s16 mag_data_x : The value of X data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated X data value output as float
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
float bmm050_compensate_X_float(s16 mag_data_x, u16 data_r);
#endif
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Y data
 *	the out put of Y as s16
 *
 *
 *
 *  \param s16 mag_data_y : The value of Y data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated Y data value output as s16
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s16 bmm050_compensate_Y(s16 mag_data_y, u16 data_r);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Y data
 *	the out put of Y as s32
 *
 *
 *
 *  \param s16 mag_data_y : The value of Y data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated Y data value output as s32
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmm050_compensate_Y_s32(s16 mag_data_y, u16 data_r);
#ifdef ENABLE_FLOAT
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Y data
 *	the out put of Y as float
 *
 *
 *
 *  \param s16 mag_data_y : The value of Y data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated Y data value output as float
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
float bmm050_compensate_Y_float(s16 mag_data_y, u16 data_r);
#endif
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z data
 *	the out put of Z as s16
 *
 *
 *
 *  \param s16 mag_data_z : The value of Z data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated Z data value output as s16
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s16 bmm050_compensate_Z(s16 mag_data_z, u16 data_r);
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z data
 *	the out put of Z as s32
 *
 *
 *
 *  \param s16 mag_data_z : The value of Z data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated Z data value output as s32
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
s32 bmm050_compensate_Z_s32(s16 mag_data_z, u16 data_r);
#ifdef ENABLE_FLOAT
/*****************************************************************************
 *	Description: *//**\brief This API used to get the compensated Z data
 *	the out put of Z as float
 *
 *
 *
 *  \param s16 mag_data_z : The value of Z data
 *			u16 data_r : The value of R data
 *
 *	\return results of compensated Z data value output as float
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
float bmm050_compensate_Z_float (s16 mag_data_z, u16 data_r);
#endif
/*****************************************************************************
 *	Description: *//**\brief This API used to set the control measurement
 *	X data in the register 0x4E bit 3
 *
 *
 *  \param u8 v_enable_disable_u8: The value of control measurement-x
 *	v_enable_disable_u8 -> 0 -> disable
 *	v_enable_disable_u8 -> 1 -> enable
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_control_measurement_x(
u8 v_enable_disable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used to set the control measurement
 *	Y data in the register 0x4E bit 4
 *
 *
 *  \param u8 v_enable_disable_u8: The value of control measurement-y
 *	v_enable_disable_u8 -> 0 -> disable
 *	v_enable_disable_u8 -> 1 -> enable
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_set_control_measurement_y(
u8 v_enable_disable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API used reset the register values to default
 *	in the register 0x4B
 *
 *
 *  \param : None
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_soft_rst(void);
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                   raw data of X,Y,Z values and RHALL values
 *                   from location 42h to 49h
 *
 *
 *
 *
 *  \param bmm050_mag_data_s16_t * mag_data :
 *	pointer holding the data of raw data
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_raw_xyz(
struct bmm050_mag_data_s16_t *mag_data);
/*****************************************************************************
 * Description: *//**\brief This API reads Magnetometer
 *                   raw data of X,Y,Z values and RHALL values
 *                   from location 42h to 49h
 *	In this function X and Y axis is remapped
 *	X is read from the address 0x44 & 0x45
 *	Y is read from the address 0x42 & 0x43
 *	this API is only applicable for BMX055 sensor
 *
 *
 *
 *  \param bmm050_remapped_mag_s16_data_t * mag_data :
 *	pointer holding the data of mag
 *
 *
 *	\return results of bus communication function
 *
 ***************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_bmx055_remapped_raw_xyz(
struct bmm050_remapped_mag_s16_data_t *mag_data);

#endif
