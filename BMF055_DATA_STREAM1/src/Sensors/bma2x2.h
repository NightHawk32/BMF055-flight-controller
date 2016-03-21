/*
****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* bma2x2.h
* Date: 2014/10/17
* Revision: 2.0.2 $
*
* Usage: Sensor Driver for BMA2x2 sensor
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
/*! \file bma2x2.h
    \brief BMA2x2 Sensor Driver Support Header File */
/* user defined code to be added here ... */
#ifndef __BMA2x2_H__
#define __BMA2x2_H__

/*******************************************************
* These definition uses for define the data types
********************************************************
*While porting the API please consider the following
*Please check the version of C standard
*Are you using Linux platform
*******************************************************/

/*********************************************************
* This definition uses for the Linux platform support
* Please use the types.h for your data types definitions
*********************************************************/
#ifdef	__KERNEL__

#include <linux/types.h>

#else /* ! __KERNEL__ */
/**********************************************************
* These definition uses for define the C
* standard version data types
***********************************************************/
# if defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
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
 * compiler is C99 C standard
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
 * compiler is C89 or other C standard
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

/*Example....
* #define YOUR_H_DEFINE  < <Doxy Comment for YOUR_H_DEFINE>
* Define the calling convention of YOUR bus communication routine.
* \note This includes types of parameters.
* This example shows the configuration for an SPI bus link.

* If your communication function looks like this:

* write_my_bus_xy(u8 device_addr,
* u8 register_addr, u8 * data, u8 length);

* The BMA2x2_WR_FUNC_PTR would equal:

* #define     BMA2x2_WR_FUNC_PTR char
* (* bus_write)(u8, u8, u8 *, u8)

* Parameters can be mixed as needed refer to
* the \ref BMA2x2_BUS_WRITE_FUNC  macro.
*/
#define BMA2x2_WR_FUNC_PTR s8(*bus_write)\
(u8, u8, u8 *, u8)



/** link macro between API function calls and bus write function
*  \note The bus write function can change since
* this is a system dependant issue.

* If the bus_write parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

* If the parameters are differently ordered or your communication function
* like I2C need to know the device address,
* you can change this macro accordingly.


* define BMA2x2_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_write(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR communication routine in
* a way that equals your definition in the
* \ref BMA2x2_WR_FUNC_PTR definition.

*/




#define BMA2x2_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
bus_write(dev_addr, reg_addr, reg_data, wr_len)


/** Define the calling convention of YOUR bus communication routine.
*\note This includes types of parameters. This example
*shows the configuration for an SPI bus link.

*If your communication function looks like this:

*read_my_bus_xy(u8 device_addr,
*u8 register_addr, u8* data, u8 length);

*The BMA2x2_RD_FUNC_PTR would equal:

*#define     BMA2x2_RD_FUNC_PTR s8
*(* bus_read)(u8, u8, u8*, u8)

*Parameters can be mixed as needed refer to the
\ref BMA2x2_BUS_READ_FUNC  macro.
*/

#define BMA2x2_SPI_RD_MASK 0x80
/* for spi read transactions on SPI the MSB has to be set */
#define BMA2x2_RD_FUNC_PTR s8(*bus_read)\
(u8, u8, u8 *, u8)
#define BMA2x2_BRD_FUNC_PTR s8(*burst_read)\
(u8, u8, u8 *, u32)


/** link macro between API function calls and bus read function
* \note The bus write function can change since
* this is a system dependant issue.

* If the bus_read parameter calling order is like:
* reg_addr, reg_data, wr_len it would be as it is here.

*  If the parameters are differently ordered or your
*  communication function like I2C need to know the device address,
*  you can change this macro accordingly.


*  define BMA2x2_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
* bus_read(dev_addr, reg_addr, reg_data, wr_len)

* This macro lets all API functions call YOUR
* communication routine in a way that equals your definition in the
* \ref BMA2x2_WR_FUNC_PTR definition.

* \note: this macro also includes the "MSB='1'" for reading BMA2x2 addresses.
*/


#define BMA2x2_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
bus_read(dev_addr, reg_addr, reg_data, r_len)
#define BMA2x2_BURST_READ_FUNC(device_addr,\
register_addr, register_data, rd_len)\
burst_read(device_addr, register_addr, register_data, rd_len)

/* The following definition of I2C address is used for the following sensors
* BMA255
* BMA355
* BMA280
* BMA282
* BMA223
* BMA254
* BMA284
* BMA250E
* BMA222E
*/
#define BMA2x2_I2C_ADDR1                0x18
#define BMA2x2_I2C_ADDR2                0x19

/* The following definition of I2C address is used for the following sensors
* BMC150
* BMC056
* BMC156
*/
#define BMA2x2_I2C_ADDR3                0x10
#define BMA2x2_I2C_ADDR4                0x11

#define         C_BMA2x2_ZERO_U8X                       ((u8)0)
#define         C_BMA2x2_ONE_U8X                        ((u8)1)
#define         C_BMA2x2_TWO_U8X                        ((u8)2)
#define         C_BMA2x2_THREE_U8X                      ((u8)3)
#define         C_BMA2x2_FOUR_U8X                       ((u8)4)
#define         C_BMA2x2_FIVE_U8X                       ((u8)5)
#define         C_BMA2x2_SIX_U8X                        ((u8)6)
#define         C_BMA2x2_SEVEN_U8X                      ((u8)7)
#define         C_BMA2x2_EIGHT_U8X                      ((u8)8)
#define         C_BMA2x2_NINE_U8X                       ((u8)9)
#define         C_BMA2x2_TWELVE_U8X                     ((u8)12)
#define         C_BMA2x2_FIFETEEN_U8X                   ((u8)15)
#define         C_BMA2x2_SIXTEEN_U8X                    ((u8)16)
#define         C_BMA2x2_THIRTYTWO_U8X                  ((u8)32)



/*
API error codes
*/
#define E_OUT_OF_RANGE          ((s8)-2)
#define E_BMA2x2_NULL_PTR       ((s8)-127)
#define BMA2x2_NULL             ((u8)0)
#define BMA2x2_ERROR			((s8)-1)
#define	BMA2x2_SUCCESS			((u8)0)

#define	BMA2x2_RETURN_FUNCTION_TYPE        s8
/**< This refers BMA2x2 return type as char */



 /*  register definitions */
#define BMA2x2_EEP_OFFSET                       0x16
#define BMA2x2_IMAGE_BASE                       0x38
#define BMA2x2_IMAGE_LEN                        22


#define BMA2x2_CHIP_ID_REG                      0x00
#define BMA2x2_X_AXIS_LSB_REG                   0x02
#define BMA2x2_X_AXIS_MSB_REG                   0x03
#define BMA2x2_Y_AXIS_LSB_REG                   0x04
#define BMA2x2_Y_AXIS_MSB_REG                   0x05
#define BMA2x2_Z_AXIS_LSB_REG                   0x06
#define BMA2x2_Z_AXIS_MSB_REG                   0x07
#define BMA2x2_TEMP_REG							0x08
#define BMA2x2_STAT1_REG						0x09
#define BMA2x2_STAT2_REG						0x0A
#define BMA2x2_STAT_TAP_SLOPE_REG				0x0B
#define BMA2x2_STAT_ORIENT_HIGH_REG				0x0C
#define BMA2x2_STAT_FIFO_REG					0x0E
#define BMA2x2_RANGE_SELECT_REG                 0x0F
#define BMA2x2_BW_SELECT_REG                    0x10
#define BMA2x2_MODE_CTRL_REG                    0x11
#define BMA2x2_LOW_NOISE_CTRL_REG               0x12
#define BMA2x2_DATA_CTRL_REG                    0x13
#define BMA2x2_RST_REG                          0x14
#define BMA2x2_INTR_ENABLE1_REG                 0x16
#define BMA2x2_INTR_ENABLE2_REG                 0x17
#define BMA2x2_INTR_SLOW_NO_MOTION_REG          0x18
#define BMA2x2_INTR1_PAD_SELECT_REG             0x19
#define BMA2x2_INTR_DATA_SELECT_REG             0x1A
#define BMA2x2_INTR2_PAD_SELECT_REG             0x1B
#define BMA2x2_INTR_SOURCE_REG                  0x1E
#define BMA2x2_INTR_SET_REG                     0x20
#define BMA2x2_INTR_CTRL_REG                    0x21
#define BMA2x2_LOW_DURN_REG                     0x22
#define BMA2x2_LOW_THRES_REG                    0x23
#define BMA2x2_LOW_HIGH_HYST_REG                0x24
#define BMA2x2_HIGH_DURN_REG                    0x25
#define BMA2x2_HIGH_THRES_REG                   0x26
#define BMA2x2_SLOPE_DURN_REG                   0x27
#define BMA2x2_SLOPE_THRES_REG                  0x28
#define BMA2x2_SLOW_NO_MOTION_THRES_REG         0x29
#define BMA2x2_TAP_PARAM_REG                    0x2A
#define BMA2x2_TAP_THRES_REG                    0x2B
#define BMA2x2_ORIENT_PARAM_REG                 0x2C
#define BMA2x2_THETA_BLOCK_REG                  0x2D
#define BMA2x2_THETA_FLAT_REG                   0x2E
#define BMA2x2_FLAT_HOLD_TIME_REG               0x2F
#define BMA2x2_FIFO_WML_TRIG                    0x30
#define BMA2x2_SELFTEST_REG                     0x32
#define BMA2x2_EEPROM_CTRL_REG                  0x33
#define BMA2x2_SERIAL_CTRL_REG                  0x34
#define BMA2x2_OFFSET_CTRL_REG                  0x36
#define BMA2x2_OFFSET_PARAMS_REG                0x37
#define BMA2x2_OFFSET_X_AXIS_REG                0x38
#define BMA2x2_OFFSET_Y_AXIS_REG                0x39
#define BMA2x2_OFFSET_Z_AXIS_REG                0x3A
#define BMA2x2_GP0_REG                          0x3B
#define BMA2x2_GP1_REG                          0x3C
#define BMA2x2_FIFO_MODE_REG                    0x3E
#define BMA2x2_FIFO_DATA_OUTPUT_REG             0x3F

#define BMA2x2_12_RESOLUTION                    0
#define BMA2x2_10_RESOLUTION                    1
#define BMA2x2_14_RESOLUTION                    2

/* register write and read delays */

#define BMA2x2_MDELAY_DATA_TYPE                 u32
#define BMA2x2_EE_W_DELAY                       28

/* delay after EEP write is 28 msec
* bma2x2 acceleration data
* brief Structure containing acceleration values
* for x,y and z-axis in S16
*/
struct bma2x2_accel_data {
s16 x,
y,
z;
};

struct bma2x2_accel_data_temp {
s16 x,
y,
z;
s8 temp;
};


struct  bma2x2_accel_eight_resolution {
s8 x,
y,
z;
};

struct bma2x2_accel_eight_resolution_temp {
s8 x,
y,
z;
s8 temp;
};

/****************************************************************************
 *	struct bma2x2_t is used for assigning the following parameters.
 *
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Burst read function pointer: BMA2x2_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
****************************************************************************/

struct bma2x2_t {
u8 v_power_mode_u8;
/**< save current bma2x2 operation mode */
u8 chip_id;
/**< save bma2x2's chip id which has to be*/
u8 ctrl_mode_reg;
/* the value of power mode register 0x11*/
u8 low_mode_reg;
/* the value of power mode register 0x12*/
u8 dev_addr;
/**< initializes bma2x2's I2C device address*/
u8 fifo_config;
BMA2x2_WR_FUNC_PTR;
/**< function pointer to the SPI/I2C write function */
BMA2x2_RD_FUNC_PTR;
/**< function pointer to the SPI/I2C read function */
BMA2x2_BRD_FUNC_PTR;

void(*delay_msec)(BMA2x2_MDELAY_DATA_TYPE);
/**< function pointer to a pause in mili seconds function */
};

#define BMA2x2_CHIP_ID__POS             0
#define BMA2x2_CHIP_ID__MSK             0xFF
#define BMA2x2_CHIP_ID__LEN             8
#define BMA2x2_CHIP_ID__REG             BMA2x2_CHIP_ID_REG

/* DATA REGISTERS */
#define BMA2x2_NEW_DATA_X__POS          0
#define BMA2x2_NEW_DATA_X__LEN          1
#define BMA2x2_NEW_DATA_X__MSK          0x01
#define BMA2x2_NEW_DATA_X__REG          BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X14_LSB__POS           2
#define BMA2x2_ACCEL_X14_LSB__LEN           6
#define BMA2x2_ACCEL_X14_LSB__MSK           0xFC
#define BMA2x2_ACCEL_X14_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X12_LSB__POS           4
#define BMA2x2_ACCEL_X12_LSB__LEN           4
#define BMA2x2_ACCEL_X12_LSB__MSK           0xF0
#define BMA2x2_ACCEL_X12_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X10_LSB__POS           6
#define BMA2x2_ACCEL_X10_LSB__LEN           2
#define BMA2x2_ACCEL_X10_LSB__MSK           0xC0
#define BMA2x2_ACCEL_X10_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X8_LSB__POS           0
#define BMA2x2_ACCEL_X8_LSB__LEN           0
#define BMA2x2_ACCEL_X8_LSB__MSK           0x00
#define BMA2x2_ACCEL_X8_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X_MSB__POS           0
#define BMA2x2_ACCEL_X_MSB__LEN           8
#define BMA2x2_ACCEL_X_MSB__MSK           0xFF
#define BMA2x2_ACCEL_X_MSB__REG           BMA2x2_X_AXIS_MSB_REG

#define BMA2x2_NEW_DATA_Y__POS          0
#define BMA2x2_NEW_DATA_Y__LEN          1
#define BMA2x2_NEW_DATA_Y__MSK          0x01
#define BMA2x2_NEW_DATA_Y__REG          BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y14_LSB__POS           2
#define BMA2x2_ACCEL_Y14_LSB__LEN           6
#define BMA2x2_ACCEL_Y14_LSB__MSK           0xFC
#define BMA2x2_ACCEL_Y14_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y12_LSB__POS           4
#define BMA2x2_ACCEL_Y12_LSB__LEN           4
#define BMA2x2_ACCEL_Y12_LSB__MSK           0xF0
#define BMA2x2_ACCEL_Y12_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y10_LSB__POS           6
#define BMA2x2_ACCEL_Y10_LSB__LEN           2
#define BMA2x2_ACCEL_Y10_LSB__MSK           0xC0
#define BMA2x2_ACCEL_Y10_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y8_LSB__POS           0
#define BMA2x2_ACCEL_Y8_LSB__LEN           0
#define BMA2x2_ACCEL_Y8_LSB__MSK           0x00
#define BMA2x2_ACCEL_Y8_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y_MSB__POS           0
#define BMA2x2_ACCEL_Y_MSB__LEN           8
#define BMA2x2_ACCEL_Y_MSB__MSK           0xFF
#define BMA2x2_ACCEL_Y_MSB__REG           BMA2x2_Y_AXIS_MSB_REG

#define BMA2x2_NEW_DATA_Z__POS          0
#define BMA2x2_NEW_DATA_Z__LEN          1
#define BMA2x2_NEW_DATA_Z__MSK          0x01
#define BMA2x2_NEW_DATA_Z__REG          BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z14_LSB__POS           2
#define BMA2x2_ACCEL_Z14_LSB__LEN           6
#define BMA2x2_ACCEL_Z14_LSB__MSK           0xFC
#define BMA2x2_ACCEL_Z14_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z12_LSB__POS           4
#define BMA2x2_ACCEL_Z12_LSB__LEN           4
#define BMA2x2_ACCEL_Z12_LSB__MSK           0xF0
#define BMA2x2_ACCEL_Z12_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z10_LSB__POS           6
#define BMA2x2_ACCEL_Z10_LSB__LEN           2
#define BMA2x2_ACCEL_Z10_LSB__MSK           0xC0
#define BMA2x2_ACCEL_Z10_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z8_LSB__POS           0
#define BMA2x2_ACCEL_Z8_LSB__LEN           0
#define BMA2x2_ACCEL_Z8_LSB__MSK           0x00
#define BMA2x2_ACCEL_Z8_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z_MSB__POS           0
#define BMA2x2_ACCEL_Z_MSB__LEN           8
#define BMA2x2_ACCEL_Z_MSB__MSK           0xFF
#define BMA2x2_ACCEL_Z_MSB__REG           BMA2x2_Z_AXIS_MSB_REG

/* Temperature */
#define BMA2x2_ACCEL_TEMP_MSB__POS           0
#define BMA2x2_ACCEL_TEMP_MSB__LEN           8
#define BMA2x2_ACCEL_TEMP_MSB__MSK           0xFF
#define BMA2x2_ACCEL_TEMP_MSB__REG           BMA2x2_TEMPERATURE_REG

/*  INTERRUPT STATUS BITS  */
#define BMA2x2_LOW_G_INTR_STAT__POS          0
#define BMA2x2_LOW_G_INTR_STAT__LEN          1
#define BMA2x2_LOW_G_INTR_STAT__MSK          0x01
#define BMA2x2_LOW_G_INTR_STAT__REG          BMA2x2_STAT1_REG

#define BMA2x2_HIGH_G_INTR_STAT__POS          1
#define BMA2x2_HIGH_G_INTR_STAT__LEN          1
#define BMA2x2_HIGH_G_INTR_STAT__MSK          0x02
#define BMA2x2_HIGH_G_INTR_STAT__REG          BMA2x2_STAT1_REG

#define BMA2x2_SLOPE_INTR_STAT__POS          2
#define BMA2x2_SLOPE_INTR_STAT__LEN          1
#define BMA2x2_SLOPE_INTR_STAT__MSK          0x04
#define BMA2x2_SLOPE_INTR_STAT__REG          BMA2x2_STAT1_REG


#define BMA2x2_SLOW_NO_MOTION_INTR_STAT__POS          3
#define BMA2x2_SLOW_NO_MOTION_INTR_STAT__LEN          1
#define BMA2x2_SLOW_NO_MOTION_INTR_STAT__MSK          0x08
#define BMA2x2_SLOW_NO_MOTION_INTR_STAT__REG          BMA2x2_STAT1_REG

#define BMA2x2_DOUBLE_TAP_INTR_STAT__POS     4
#define BMA2x2_DOUBLE_TAP_INTR_STAT__LEN     1
#define BMA2x2_DOUBLE_TAP_INTR_STAT__MSK     0x10
#define BMA2x2_DOUBLE_TAP_INTR_STAT__REG     BMA2x2_STAT1_REG

#define BMA2x2_SINGLE_TAP_INTR_STAT__POS     5
#define BMA2x2_SINGLE_TAP_INTR_STAT__LEN     1
#define BMA2x2_SINGLE_TAP_INTR_STAT__MSK     0x20
#define BMA2x2_SINGLE_TAP_INTR_STAT__REG     BMA2x2_STAT1_REG

#define BMA2x2_ORIENT_INTR_STAT__POS         6
#define BMA2x2_ORIENT_INTR_STAT__LEN         1
#define BMA2x2_ORIENT_INTR_STAT__MSK         0x40
#define BMA2x2_ORIENT_INTR_STAT__REG         BMA2x2_STAT1_REG

#define BMA2x2_FLAT_INTR_STAT__POS           7
#define BMA2x2_FLAT_INTR_STAT__LEN           1
#define BMA2x2_FLAT_INTR_STAT__MSK           0x80
#define BMA2x2_FLAT_INTR_STAT__REG           BMA2x2_STAT1_REG

#define BMA2x2_FIFO_FULL_INTR_STAT__POS           5
#define BMA2x2_FIFO_FULL_INTR_STAT__LEN           1
#define BMA2x2_FIFO_FULL_INTR_STAT__MSK           0x20
#define BMA2x2_FIFO_FULL_INTR_STAT__REG           BMA2x2_STAT2_REG

#define BMA2x2_FIFO_WM_INTR_STAT__POS           6
#define BMA2x2_FIFO_WM_INTR_STAT__LEN           1
#define BMA2x2_FIFO_WM_INTR_STAT__MSK           0x40
#define BMA2x2_FIFO_WM_INTR_STAT__REG           BMA2x2_STAT2_REG

#define BMA2x2_DATA_INTR_STAT__POS           7
#define BMA2x2_DATA_INTR_STAT__LEN           1
#define BMA2x2_DATA_INTR_STAT__MSK           0x80
#define BMA2x2_DATA_INTR_STAT__REG           BMA2x2_STAT2_REG

#define BMA2x2_SLOPE_FIRST_X__POS        0
#define BMA2x2_SLOPE_FIRST_X__LEN        1
#define BMA2x2_SLOPE_FIRST_X__MSK        0x01
#define BMA2x2_SLOPE_FIRST_X__REG        BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_SLOPE_FIRST_Y__POS        1
#define BMA2x2_SLOPE_FIRST_Y__LEN        1
#define BMA2x2_SLOPE_FIRST_Y__MSK        0x02
#define BMA2x2_SLOPE_FIRST_Y__REG        BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_SLOPE_FIRST_Z__POS        2
#define BMA2x2_SLOPE_FIRST_Z__LEN        1
#define BMA2x2_SLOPE_FIRST_Z__MSK        0x04
#define BMA2x2_SLOPE_FIRST_Z__REG        BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_SLOPE_SIGN_STAT__POS         3
#define BMA2x2_SLOPE_SIGN_STAT__LEN         1
#define BMA2x2_SLOPE_SIGN_STAT__MSK         0x08
#define BMA2x2_SLOPE_SIGN_STAT__REG         BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_TAP_FIRST_X__POS        4
#define BMA2x2_TAP_FIRST_X__LEN        1
#define BMA2x2_TAP_FIRST_X__MSK        0x10
#define BMA2x2_TAP_FIRST_X__REG        BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_TAP_FIRST_Y__POS        5
#define BMA2x2_TAP_FIRST_Y__LEN        1
#define BMA2x2_TAP_FIRST_Y__MSK        0x20
#define BMA2x2_TAP_FIRST_Y__REG        BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_TAP_FIRST_Z__POS        6
#define BMA2x2_TAP_FIRST_Z__LEN        1
#define BMA2x2_TAP_FIRST_Z__MSK        0x40
#define BMA2x2_TAP_FIRST_Z__REG        BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_TAP_SIGN_STAT__POS         7
#define BMA2x2_TAP_SIGN_STAT__LEN         1
#define BMA2x2_TAP_SIGN_STAT__MSK         0x80
#define BMA2x2_TAP_SIGN_STAT__REG         BMA2x2_STAT_TAP_SLOPE_REG

#define BMA2x2_HIGH_G_FIRST_X__POS        0
#define BMA2x2_HIGH_G_FIRST_X__LEN        1
#define BMA2x2_HIGH_G_FIRST_X__MSK        0x01
#define BMA2x2_HIGH_G_FIRST_X__REG        BMA2x2_STAT_ORIENT_HIGH_REG

#define BMA2x2_HIGH_G_FIRST_Y__POS        1
#define BMA2x2_HIGH_G_FIRST_Y__LEN        1
#define BMA2x2_HIGH_G_FIRST_Y__MSK        0x02
#define BMA2x2_HIGH_G_FIRST_Y__REG        BMA2x2_STAT_ORIENT_HIGH_REG

#define BMA2x2_HIGH_G_FIRST_Z__POS        2
#define BMA2x2_HIGH_G_FIRST_Z__LEN        1
#define BMA2x2_HIGH_G_FIRST_Z__MSK        0x04
#define BMA2x2_HIGH_G_FIRST_Z__REG        BMA2x2_STAT_ORIENT_HIGH_REG

#define BMA2x2_HIGH_G_SIGN_STAT__POS         3
#define BMA2x2_HIGH_G_SIGN_STAT__LEN         1
#define BMA2x2_HIGH_G_SIGN_STAT__MSK         0x08
#define BMA2x2_HIGH_G_SIGN_STAT__REG         BMA2x2_STAT_ORIENT_HIGH_REG

#define BMA2x2_ORIENT_STAT__POS             4
#define BMA2x2_ORIENT_STAT__LEN             3
#define BMA2x2_ORIENT_STAT__MSK             0x70
#define BMA2x2_ORIENT_STAT__REG             BMA2x2_STAT_ORIENT_HIGH_REG

#define BMA2x2_FLAT_STAT__POS               7
#define BMA2x2_FLAT_STAT__LEN               1
#define BMA2x2_FLAT_STAT__MSK               0x80
#define BMA2x2_FLAT_STAT__REG               BMA2x2_STAT_ORIENT_HIGH_REG

/*FIFO_STATUS*/
#define BMA2x2_FIFO_FRAME_COUNT_STAT__POS             0
#define BMA2x2_FIFO_FRAME_COUNT_STAT__LEN             7
#define BMA2x2_FIFO_FRAME_COUNT_STAT__MSK             0x7F
#define BMA2x2_FIFO_FRAME_COUNT_STAT__REG             BMA2x2_STAT_FIFO_REG

#define BMA2x2_FIFO_OVERRUN_STAT__POS             7
#define BMA2x2_FIFO_OVERRUN_STAT__LEN             1
#define BMA2x2_FIFO_OVERRUN_STAT__MSK             0x80
#define BMA2x2_FIFO_OVERRUN_STAT__REG             BMA2x2_STAT_FIFO_REG

#define BMA2x2_RANGE_SELECT__POS             0
#define BMA2x2_RANGE_SELECT__LEN             4
#define BMA2x2_RANGE_SELECT__MSK             0x0F
#define BMA2x2_RANGE_SELECT__REG             BMA2x2_RANGE_SELECT_REG

#define BMA2x2_BW__POS             0
#define BMA2x2_BW__LEN             5
#define BMA2x2_BW__MSK             0x1F
#define BMA2x2_BW__REG             BMA2x2_BW_SELECT_REG

#define BMA2x2_SLEEP_DURN__POS             1
#define BMA2x2_SLEEP_DURN__LEN             4
#define BMA2x2_SLEEP_DURN__MSK             0x1E
#define BMA2x2_SLEEP_DURN__REG             BMA2x2_MODE_CTRL_REG

#define BMA2x2_MODE_CTRL__POS             5
#define BMA2x2_MODE_CTRL__LEN             3
#define BMA2x2_MODE_CTRL__MSK             0xE0
#define BMA2x2_MODE_CTRL__REG             BMA2x2_MODE_CTRL_REG


#define BMA2x2_ENABLE_LOW_POWER__POS          6
#define BMA2x2_ENABLE_LOW_POWER__LEN          1
#define BMA2x2_ENABLE_LOW_POWER__MSK          0x40
#define BMA2x2_ENABLE_LOW_POWER__REG          BMA2x2_MODE_CTRL_REG



#define BMA2x2_SLEEP_TIMER__POS          5
#define BMA2x2_SLEEP_TIMER__LEN          1
#define BMA2x2_SLEEP_TIMER__MSK          0x20
#define BMA2x2_SLEEP_TIMER__REG          BMA2x2_LOW_NOISE_CTRL_REG


#define BMA2x2_LOW_POWER_MODE__POS          6
#define BMA2x2_LOW_POWER_MODE__LEN          1
#define BMA2x2_LOW_POWER_MODE__MSK          0x40
#define BMA2x2_LOW_POWER_MODE__REG          BMA2x2_LOW_NOISE_CTRL_REG
/**     DISABLE MSB SHADOWING PROCEDURE          **/
#define BMA2x2_DIS_SHADOW_PROC__POS       6
#define BMA2x2_DIS_SHADOW_PROC__LEN       1
#define BMA2x2_DIS_SHADOW_PROC__MSK       0x40
#define BMA2x2_DIS_SHADOW_PROC__REG       BMA2x2_DATA_CTRL_REG

/**     FILTERED OR UNFILTERED ACCELERATION DATA  **/
#define BMA2x2_ENABLE_DATA_HIGH_BW__POS         7
#define BMA2x2_ENABLE_DATA_HIGH_BW__LEN         1
#define BMA2x2_ENABLE_DATA_HIGH_BW__MSK         0x80
#define BMA2x2_ENABLE_DATA_HIGH_BW__REG         BMA2x2_DATA_CTRL_REG

#define BMA2x2_ENABLE_SOFT_RESET_VALUE        0xB6

/**     INTERRUPT ENABLE REGISTER              **/
#define BMA2x2_ENABLE_SLOPE_X_INTR__POS         0
#define BMA2x2_ENABLE_SLOPE_X_INTR__LEN         1
#define BMA2x2_ENABLE_SLOPE_X_INTR__MSK         0x01
#define BMA2x2_ENABLE_SLOPE_X_INTR__REG         BMA2x2_INTR_ENABLE1_REG

#define BMA2x2_ENABLE_SLOPE_Y_INTR__POS         1
#define BMA2x2_ENABLE_SLOPE_Y_INTR__LEN         1
#define BMA2x2_ENABLE_SLOPE_Y_INTR__MSK         0x02
#define BMA2x2_ENABLE_SLOPE_Y_INTR__REG         BMA2x2_INTR_ENABLE1_REG

#define BMA2x2_ENABLE_SLOPE_Z_INTR__POS         2
#define BMA2x2_ENABLE_SLOPE_Z_INTR__LEN         1
#define BMA2x2_ENABLE_SLOPE_Z_INTR__MSK         0x04
#define BMA2x2_ENABLE_SLOPE_Z_INTR__REG         BMA2x2_INTR_ENABLE1_REG

#define BMA2x2_ENABLE_DOUBLE_TAP_INTR__POS      4
#define BMA2x2_ENABLE_DOUBLE_TAP_INTR__LEN      1
#define BMA2x2_ENABLE_DOUBLE_TAP_INTR__MSK      0x10
#define BMA2x2_ENABLE_DOUBLE_TAP_INTR__REG      BMA2x2_INTR_ENABLE1_REG

#define BMA2x2_ENABLE_SINGLE_TAP_INTR__POS      5
#define BMA2x2_ENABLE_SINGLE_TAP_INTR__LEN      1
#define BMA2x2_ENABLE_SINGLE_TAP_INTR__MSK      0x20
#define BMA2x2_ENABLE_SINGLE_TAP_INTR__REG      BMA2x2_INTR_ENABLE1_REG

#define BMA2x2_ENABLE_ORIENT_INTR__POS          6
#define BMA2x2_ENABLE_ORIENT_INTR__LEN          1
#define BMA2x2_ENABLE_ORIENT_INTR__MSK          0x40
#define BMA2x2_ENABLE_ORIENT_INTR__REG          BMA2x2_INTR_ENABLE1_REG

#define BMA2x2_ENABLE_FLAT_INTR__POS            7
#define BMA2x2_ENABLE_FLAT_INTR__LEN            1
#define BMA2x2_ENABLE_FLAT_INTR__MSK            0x80
#define BMA2x2_ENABLE_FLAT_INTR__REG            BMA2x2_INTR_ENABLE1_REG

/**     INTERRUPT ENABLE REGISTER              **/
#define BMA2x2_ENABLE_HIGH_G_X_INTR__POS         0
#define BMA2x2_ENABLE_HIGH_G_X_INTR__LEN         1
#define BMA2x2_ENABLE_HIGH_G_X_INTR__MSK         0x01
#define BMA2x2_ENABLE_HIGH_G_X_INTR__REG         BMA2x2_INTR_ENABLE2_REG

#define BMA2x2_ENABLE_HIGH_G_Y_INTR__POS         1
#define BMA2x2_ENABLE_HIGH_G_Y_INTR__LEN         1
#define BMA2x2_ENABLE_HIGH_G_Y_INTR__MSK         0x02
#define BMA2x2_ENABLE_HIGH_G_Y_INTR__REG         BMA2x2_INTR_ENABLE2_REG

#define BMA2x2_ENABLE_HIGH_G_Z_INTR__POS         2
#define BMA2x2_ENABLE_HIGH_G_Z_INTR__LEN         1
#define BMA2x2_ENABLE_HIGH_G_Z_INTR__MSK         0x04
#define BMA2x2_ENABLE_HIGH_G_Z_INTR__REG         BMA2x2_INTR_ENABLE2_REG

#define BMA2x2_ENABLE_LOW_G_INTR__POS            3
#define BMA2x2_ENABLE_LOW_G_INTR__LEN            1
#define BMA2x2_ENABLE_LOW_G_INTR__MSK            0x08
#define BMA2x2_ENABLE_LOW_G_INTR__REG            BMA2x2_INTR_ENABLE2_REG

#define BMA2x2_ENABLE_NEW_DATA_INTR__POS        4
#define BMA2x2_ENABLE_NEW_DATA_INTR__LEN        1
#define BMA2x2_ENABLE_NEW_DATA_INTR__MSK        0x10
#define BMA2x2_ENABLE_NEW_DATA_INTR__REG        BMA2x2_INTR_ENABLE2_REG


#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR__POS        5
#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR__LEN        1
#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR__MSK        0x20
#define BMA2x2_INTR_FIFO_FULL_ENABLE_INTR__REG        BMA2x2_INTR_ENABLE2_REG

#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR__POS        6
#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR__LEN        1
#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR__MSK        0x40
#define BMA2x2_INTR_FIFO_WM_ENABLE_INTR__REG        BMA2x2_INTR_ENABLE2_REG

/*INT SLO NO MOT*/
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR__POS        0
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR__LEN        1
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR__MSK        0x01
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR__REG        \
BMA2x2_INTR_SLOW_NO_MOTION_REG

#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR__POS        1
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR__LEN        1
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR__MSK        0x02
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR__REG        \
BMA2x2_INTR_SLOW_NO_MOTION_REG

#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR__POS        2
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR__LEN        1
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR__MSK        0x04
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR__REG        \
BMA2x2_INTR_SLOW_NO_MOTION_REG

#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR__POS        3
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR__LEN        1
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR__MSK        0x08
#define BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR__REG        \
BMA2x2_INTR_SLOW_NO_MOTION_REG

#define BMA2x2_ENABLE_INTR1_PAD_LOW_G__POS        0
#define BMA2x2_ENABLE_INTR1_PAD_LOW_G__LEN        1
#define BMA2x2_ENABLE_INTR1_PAD_LOW_G__MSK        0x01
#define BMA2x2_ENABLE_INTR1_PAD_LOW_G__REG        BMA2x2_INTR1_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G__POS       1
#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G__LEN       1
#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G__MSK       0x02
#define BMA2x2_ENABLE_INTR1_PAD_HIGH_G__REG       BMA2x2_INTR1_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_SLOPE__POS       2
#define BMA2x2_ENABLE_INTR1_PAD_SLOPE__LEN       1
#define BMA2x2_ENABLE_INTR1_PAD_SLOPE__MSK       0x04
#define BMA2x2_ENABLE_INTR1_PAD_SLOPE__REG       BMA2x2_INTR1_PAD_SELECT_REG


#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION__POS        3
#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION__LEN        1
#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION__MSK        0x08
#define BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION__REG        \
BMA2x2_INTR1_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP__POS      4
#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP__LEN      1
#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP__MSK      0x10
#define BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP__REG      BMA2x2_INTR1_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__POS     5
#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__LEN     1
#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__MSK     0x20
#define BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__REG     BMA2x2_INTR1_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_ORIENT__POS      6
#define BMA2x2_ENABLE_INTR1_PAD_ORIENT__LEN      1
#define BMA2x2_ENABLE_INTR1_PAD_ORIENT__MSK      0x40
#define BMA2x2_ENABLE_INTR1_PAD_ORIENT__REG      BMA2x2_INTR1_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_FLAT__POS        7
#define BMA2x2_ENABLE_INTR1_PAD_FLAT__LEN        1
#define BMA2x2_ENABLE_INTR1_PAD_FLAT__MSK        0x80
#define BMA2x2_ENABLE_INTR1_PAD_FLAT__REG        BMA2x2_INTR1_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_LOW_G__POS        0
#define BMA2x2_ENABLE_INTR2_PAD_LOW_G__LEN        1
#define BMA2x2_ENABLE_INTR2_PAD_LOW_G__MSK        0x01
#define BMA2x2_ENABLE_INTR2_PAD_LOW_G__REG        BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G__POS       1
#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G__LEN       1
#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G__MSK       0x02
#define BMA2x2_ENABLE_INTR2_PAD_HIGH_G__REG       BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_SLOPE__POS       2
#define BMA2x2_ENABLE_INTR2_PAD_SLOPE__LEN       1
#define BMA2x2_ENABLE_INTR2_PAD_SLOPE__MSK       0x04
#define BMA2x2_ENABLE_INTR2_PAD_SLOPE__REG       BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION__POS        3
#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION__LEN        1
#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION__MSK        0x08
#define BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION__REG        \
BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP__POS      4
#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP__LEN      1
#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP__MSK      0x10
#define BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP__REG      BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__POS     5
#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__LEN     1
#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__MSK     0x20
#define BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__REG     BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_ORIENT__POS      6
#define BMA2x2_ENABLE_INTR2_PAD_ORIENT__LEN      1
#define BMA2x2_ENABLE_INTR2_PAD_ORIENT__MSK      0x40
#define BMA2x2_ENABLE_INTR2_PAD_ORIENT__REG      BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_FLAT__POS        7
#define BMA2x2_ENABLE_INTR2_PAD_FLAT__LEN        1
#define BMA2x2_ENABLE_INTR2_PAD_FLAT__MSK        0x80
#define BMA2x2_ENABLE_INTR2_PAD_FLAT__REG        BMA2x2_INTR2_PAD_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA__POS     0
#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA__LEN     1
#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA__MSK     0x01
#define BMA2x2_ENABLE_INTR1_PAD_NEWDATA__REG     BMA2x2_INTR_DATA_SELECT_REG


#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM__POS     1
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM__LEN     1
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM__MSK     0x02
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_WM__REG     BMA2x2_INTR_DATA_SELECT_REG

#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL__POS     2
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL__LEN     1
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL__MSK     0x04
#define BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL__REG     BMA2x2_INTR_DATA_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL__POS     5
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL__LEN     1
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL__MSK     0x20
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL__REG     BMA2x2_INTR_DATA_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM__POS     6
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM__LEN     1
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM__MSK     0x40
#define BMA2x2_ENABLE_INTR2_PAD_FIFO_WM__REG     BMA2x2_INTR_DATA_SELECT_REG

#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA__POS     7
#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA__LEN     1
#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA__MSK     0x80
#define BMA2x2_ENABLE_INTR2_PAD_NEWDATA__REG     BMA2x2_INTR_DATA_SELECT_REG

/*****          INTERRUPT SOURCE SELECTION                      *****/
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G__POS        0
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G__LEN        1
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G__MSK        0x01
#define BMA2x2_UNFILT_INTR_SOURCE_LOW_G__REG        BMA2x2_INTR_SOURCE_REG

#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G__POS       1
#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G__LEN       1
#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G__MSK       0x02
#define BMA2x2_UNFILT_INTR_SOURCE_HIGH_G__REG       BMA2x2_INTR_SOURCE_REG

#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE__POS       2
#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE__LEN       1
#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE__MSK       0x04
#define BMA2x2_UNFILT_INTR_SOURCE_SLOPE__REG       BMA2x2_INTR_SOURCE_REG


#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION__POS        3
#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION__LEN        1
#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION__MSK        0x08
#define BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION__REG        \
BMA2x2_INTR_SOURCE_REG

#define BMA2x2_UNFILT_INTR_SOURCE_TAP__POS         4
#define BMA2x2_UNFILT_INTR_SOURCE_TAP__LEN         1
#define BMA2x2_UNFILT_INTR_SOURCE_TAP__MSK         0x10
#define BMA2x2_UNFILT_INTR_SOURCE_TAP__REG         BMA2x2_INTR_SOURCE_REG

#define BMA2x2_UNFILT_INTR_SOURCE_DATA__POS        5
#define BMA2x2_UNFILT_INTR_SOURCE_DATA__LEN        1
#define BMA2x2_UNFILT_INTR_SOURCE_DATA__MSK        0x20
#define BMA2x2_UNFILT_INTR_SOURCE_DATA__REG        BMA2x2_INTR_SOURCE_REG

/*****  INTERRUPT PAD ACTIVE LEVEL AND OUTPUT TYPE       *****/
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL__POS       0
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL__MSK       0x01
#define BMA2x2_INTR1_PAD_ACTIVE_LEVEL__REG       BMA2x2_INTR_SET_REG

#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL__POS       2
#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL__MSK       0x04
#define BMA2x2_INTR2_PAD_ACTIVE_LEVEL__REG       BMA2x2_INTR_SET_REG


/*****  OUTPUT TYPE IF SET TO 1 IS : OPEN DRIVE , IF NOT SET
	IT IS PUSH-PULL                                  *****/
#define BMA2x2_INTR1_PAD_OUTPUT_TYPE__POS        1
#define BMA2x2_INTR1_PAD_OUTPUT_TYPE__LEN        1
#define BMA2x2_INTR1_PAD_OUTPUT_TYPE__MSK        0x02
#define BMA2x2_INTR1_PAD_OUTPUT_TYPE__REG        BMA2x2_INTR_SET_REG

#define BMA2x2_INTR2_PAD_OUTPUT_TYPE__POS        3
#define BMA2x2_INTR2_PAD_OUTPUT_TYPE__LEN        1
#define BMA2x2_INTR2_PAD_OUTPUT_TYPE__MSK        0x08
#define BMA2x2_INTR2_PAD_OUTPUT_TYPE__REG        BMA2x2_INTR_SET_REG
/*****               INTERRUPT MODE SELECTION              ******/
#define BMA2x2_LATCH_INTR__POS                0
#define BMA2x2_LATCH_INTR__LEN                4
#define BMA2x2_LATCH_INTR__MSK                0x0F
#define BMA2x2_LATCH_INTR__REG                BMA2x2_INTR_CTRL_REG

/*****               LATCHED INTERRUPT RESET               ******/
#define BMA2x2_RESET_INTR__POS           7
#define BMA2x2_RESET_INTR__LEN           1
#define BMA2x2_RESET_INTR__MSK           0x80
#define BMA2x2_RESET_INTR__REG           BMA2x2_INTR_CTRL_REG

/*****               LOW-G HYSTERESIS                       ******/
#define BMA2x2_LOW_G_HYST__POS                   0
#define BMA2x2_LOW_G_HYST__LEN                   2
#define BMA2x2_LOW_G_HYST__MSK                   0x03
#define BMA2x2_LOW_G_HYST__REG                   BMA2x2_LOW_HIGH_HYST_REG

/*****               LOW-G INTERRUPT MODE                   ******/
/*****       IF 1 -- SUM MODE , 0 -- SINGLE MODE            ******/
#define BMA2x2_LOW_G_INTR_MODE__POS               2
#define BMA2x2_LOW_G_INTR_MODE__LEN               1
#define BMA2x2_LOW_G_INTR_MODE__MSK               0x04
#define BMA2x2_LOW_G_INTR_MODE__REG               BMA2x2_LOW_HIGH_HYST_REG

/*****               HIGH-G HYSTERESIS                       ******/
#define BMA2x2_HIGH_G_HYST__POS                  6
#define BMA2x2_HIGH_G_HYST__LEN                  2
#define BMA2x2_HIGH_G_HYST__MSK                  0xC0
#define BMA2x2_HIGH_G_HYST__REG                  BMA2x2_LOW_HIGH_HYST_REG

/*****               SLOPE DURATION                        ******/
#define BMA2x2_SLOPE_DURN__POS                    0
#define BMA2x2_SLOPE_DURN__LEN                    2
#define BMA2x2_SLOPE_DURN__MSK                    0x03
#define BMA2x2_SLOPE_DURN__REG                    BMA2x2_SLOPE_DURN_REG

/*SLO_NO_MOT_DURN ADDED*/
#define BMA2x2_SLOW_NO_MOTION_DURN__POS                    2
#define BMA2x2_SLOW_NO_MOTION_DURN__LEN                    6
#define BMA2x2_SLOW_NO_MOTION_DURN__MSK                    0xFC
#define BMA2x2_SLOW_NO_MOTION_DURN__REG                    BMA2x2_SLOPE_DURN_REG

/*****               TAP DURATION                        ******/
#define BMA2x2_TAP_DURN__POS                    0
#define BMA2x2_TAP_DURN__LEN                    3
#define BMA2x2_TAP_DURN__MSK                    0x07
#define BMA2x2_TAP_DURN__REG                    BMA2x2_TAP_PARAM_REG


/*****               TAP SHOCK DURATION                 ******/
#define BMA2x2_TAP_SHOCK_DURN__POS             6
#define BMA2x2_TAP_SHOCK_DURN__LEN             1
#define BMA2x2_TAP_SHOCK_DURN__MSK             0x40
#define BMA2x2_TAP_SHOCK_DURN__REG             BMA2x2_TAP_PARAM_REG

/* This advance tap interrupt only uses for the chip id 0xFB */
/*****               ADV TAP INT                        ******/
#define BMA2x2_ADV_TAP_INTR__POS                5
#define BMA2x2_ADV_TAP_INTR__LEN                1
#define BMA2x2_ADV_TAP_INTR__MSK                0x20
#define BMA2x2_ADV_TAP_INTR__REG                BMA2x2_TAP_PARAM_REG

/*****               TAP QUIET DURATION                 ******/
#define BMA2x2_TAP_QUIET_DURN__POS             7
#define BMA2x2_TAP_QUIET_DURN__LEN             1
#define BMA2x2_TAP_QUIET_DURN__MSK             0x80
#define BMA2x2_TAP_QUIET_DURN__REG             BMA2x2_TAP_PARAM_REG

/*****               TAP THRESHOLD                       ******/
#define BMA2x2_TAP_THRES__POS                  0
#define BMA2x2_TAP_THRES__LEN                  5
#define BMA2x2_TAP_THRES__MSK                  0x1F
#define BMA2x2_TAP_THRES__REG                  BMA2x2_TAP_THRES_REG

/*****               TAP SAMPLES                         ******/
#define BMA2x2_TAP_SAMPLES__POS                6
#define BMA2x2_TAP_SAMPLES__LEN                2
#define BMA2x2_TAP_SAMPLES__MSK                0xC0
#define BMA2x2_TAP_SAMPLES__REG                BMA2x2_TAP_THRES_REG

/*****       ORIENTATION MODE                        ******/
#define BMA2x2_ORIENT_MODE__POS                  0
#define BMA2x2_ORIENT_MODE__LEN                  2
#define BMA2x2_ORIENT_MODE__MSK                  0x03
#define BMA2x2_ORIENT_MODE__REG                  BMA2x2_ORIENT_PARAM_REG

/*****       ORIENTATION BLOCKING                    ******/
#define BMA2x2_ORIENT_BLOCK__POS                 2
#define BMA2x2_ORIENT_BLOCK__LEN                 2
#define BMA2x2_ORIENT_BLOCK__MSK                 0x0C
#define BMA2x2_ORIENT_BLOCK__REG                 BMA2x2_ORIENT_PARAM_REG

/*****       ORIENTATION HYSTERESIS                  ******/
#define BMA2x2_ORIENT_HYST__POS                  4
#define BMA2x2_ORIENT_HYST__LEN                  3
#define BMA2x2_ORIENT_HYST__MSK                  0x70
#define BMA2x2_ORIENT_HYST__REG                  BMA2x2_ORIENT_PARAM_REG

/*****       ORIENTATION AXIS SELECTION              ******/
/***** IF SET TO 1 -- X AND Z ARE SWAPPED , Y IS INVERTED */
#define BMA2x2_ORIENT_UD_ENABLE__POS                  6
#define BMA2x2_ORIENT_UD_ENABLE__LEN                  1
#define BMA2x2_ORIENT_UD_ENABLE__MSK                  0x40
#define BMA2x2_ORIENT_UD_ENABLE__REG                  BMA2x2_THETA_BLOCK_REG


/*****       THETA BLOCKING                    ******/
#define BMA2x2_THETA_BLOCK__POS                  0
#define BMA2x2_THETA_BLOCK__LEN                  6
#define BMA2x2_THETA_BLOCK__MSK                  0x3F
#define BMA2x2_THETA_BLOCK__REG                  BMA2x2_THETA_BLOCK_REG

/*****       THETA FLAT                        ******/
#define BMA2x2_THETA_FLAT__POS                  0
#define BMA2x2_THETA_FLAT__LEN                  6
#define BMA2x2_THETA_FLAT__MSK                  0x3F
#define BMA2x2_THETA_FLAT__REG                  BMA2x2_THETA_FLAT_REG

/*****      FLAT HOLD TIME                     ******/
#define BMA2x2_FLAT_HOLD_TIME__POS              4
#define BMA2x2_FLAT_HOLD_TIME__LEN              2
#define BMA2x2_FLAT_HOLD_TIME__MSK              0x30
#define BMA2x2_FLAT_HOLD_TIME__REG              BMA2x2_FLAT_HOLD_TIME_REG

/*****      FLAT HYS                           ******/
#define BMA2x2_FLAT_HYST__POS                   0
#define BMA2x2_FLAT_HYST__LEN                   3
#define BMA2x2_FLAT_HYST__MSK                   0x07
#define BMA2x2_FLAT_HYST__REG                   BMA2x2_FLAT_HOLD_TIME_REG

/*****      FIFO WATER MARK LEVEL TRIGGER RETAIN                        ******/
#define BMA2x2_FIFO_WML_TRIG_RETAIN__POS                   0
#define BMA2x2_FIFO_WML_TRIG_RETAIN__LEN                   6
#define BMA2x2_FIFO_WML_TRIG_RETAIN__MSK                   0x3F
#define BMA2x2_FIFO_WML_TRIG_RETAIN__REG                   BMA2x2_FIFO_WML_TRIG

/*****      ACTIVATE SELF TEST                 ******/
#define BMA2x2_ENABLE_SELFTEST__POS                0
#define BMA2x2_ENABLE_SELFTEST__LEN                2
#define BMA2x2_ENABLE_SELFTEST__MSK                0x03
#define BMA2x2_ENABLE_SELFTEST__REG                BMA2x2_SELFTEST_REG

/*****     SELF TEST -- NEGATIVE               ******/
#define BMA2x2_NEG_SELFTEST__POS               2
#define BMA2x2_NEG_SELFTEST__LEN               1
#define BMA2x2_NEG_SELFTEST__MSK               0x04
#define BMA2x2_NEG_SELFTEST__REG               BMA2x2_SELFTEST_REG

/*****     EEPROM CONTROL                      ******/
/* SETTING THIS BIT  UNLOCK'S WRITING SETTING REGISTERS TO EEPROM */
#define BMA2x2_UNLOCK_EE_PROG_MODE__POS     0
#define BMA2x2_UNLOCK_EE_PROG_MODE__LEN     1
#define BMA2x2_UNLOCK_EE_PROG_MODE__MSK     0x01
#define BMA2x2_UNLOCK_EE_PROG_MODE__REG     BMA2x2_EEPROM_CTRL_REG


/* SETTING THIS BIT STARTS WRITING SETTING REGISTERS TO EEPROM */
#define BMA2x2_START_EE_PROG_TRIG__POS      1
#define BMA2x2_START_EE_PROG_TRIG__LEN      1
#define BMA2x2_START_EE_PROG_TRIG__MSK      0x02
#define BMA2x2_START_EE_PROG_TRIG__REG      BMA2x2_EEPROM_CTRL_REG


/* STATUS OF WRITING TO EEPROM */
#define BMA2x2_EE_PROG_READY__POS          2
#define BMA2x2_EE_PROG_READY__LEN          1
#define BMA2x2_EE_PROG_READY__MSK          0x04
#define BMA2x2_EE_PROG_READY__REG          BMA2x2_EEPROM_CTRL_REG


/* UPDATE IMAGE REGISTERS WRITING TO EEPROM */
#define BMA2x2_UPDATE_IMAGE__POS                3
#define BMA2x2_UPDATE_IMAGE__LEN                1
#define BMA2x2_UPDATE_IMAGE__MSK                0x08
#define BMA2x2_UPDATE_IMAGE__REG                BMA2x2_EEPROM_CTRL_REG

#define BMA2x2_EE_REMAIN__POS                4
#define BMA2x2_EE_REMAIN__LEN                4
#define BMA2x2_EE_REMAIN__MSK                0xF0
#define BMA2x2_EE_REMAIN__REG                BMA2x2_EEPROM_CTRL_REG
/* SPI INTERFACE MODE SELECTION */
#define BMA2x2_ENABLE_SPI_MODE_3__POS              0
#define BMA2x2_ENABLE_SPI_MODE_3__LEN              1
#define BMA2x2_ENABLE_SPI_MODE_3__MSK              0x01
#define BMA2x2_ENABLE_SPI_MODE_3__REG              BMA2x2_SERIAL_CTRL_REG

/* I2C WATCHDOG PERIOD SELECTION */
#define BMA2x2_I2C_WDT_PERIOD__POS        1
#define BMA2x2_I2C_WDT_PERIOD__LEN        1
#define BMA2x2_I2C_WDT_PERIOD__MSK        0x02
#define BMA2x2_I2C_WDT_PERIOD__REG        BMA2x2_SERIAL_CTRL_REG

/* I2C WATCHDOG ENABLE */
#define BMA2x2_ENABLE_I2C_WDT__POS            2
#define BMA2x2_ENABLE_I2C_WDT__LEN            1
#define BMA2x2_ENABLE_I2C_WDT__MSK            0x04
#define BMA2x2_ENABLE_I2C_WDT__REG            BMA2x2_SERIAL_CTRL_REG

/* SPI INTERFACE MODE SELECTION */
/* SETTING THIS BIT  UNLOCK'S WRITING TRIMMING REGISTERS TO EEPROM */
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__POS        4
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__LEN        4
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__MSK        0xF0
#define BMA2x2_UNLOCK_EE_WRITE_TRIM__REG        BMA2x2_CTRL_UNLOCK_REG

/**    OFFSET  COMPENSATION     **/
/**    SLOW COMPENSATION FOR X,Y,Z AXIS      **/
#define BMA2x2_ENABLE_SLOW_COMP_X__POS              0
#define BMA2x2_ENABLE_SLOW_COMP_X__LEN              1
#define BMA2x2_ENABLE_SLOW_COMP_X__MSK              0x01
#define BMA2x2_ENABLE_SLOW_COMP_X__REG              BMA2x2_OFFSET_CTRL_REG

#define BMA2x2_ENABLE_SLOW_COMP_Y__POS              1
#define BMA2x2_ENABLE_SLOW_COMP_Y__LEN              1
#define BMA2x2_ENABLE_SLOW_COMP_Y__MSK              0x02
#define BMA2x2_ENABLE_SLOW_COMP_Y__REG              BMA2x2_OFFSET_CTRL_REG

#define BMA2x2_ENABLE_SLOW_COMP_Z__POS              2
#define BMA2x2_ENABLE_SLOW_COMP_Z__LEN              1
#define BMA2x2_ENABLE_SLOW_COMP_Z__MSK              0x04
#define BMA2x2_ENABLE_SLOW_COMP_Z__REG              BMA2x2_OFFSET_CTRL_REG

/**    FAST COMPENSATION READY FLAG          **/
#define BMA2x2_FAST_CAL_RDY_STAT__POS             4
#define BMA2x2_FAST_CAL_RDY_STAT__LEN             1
#define BMA2x2_FAST_CAL_RDY_STAT__MSK             0x10
#define BMA2x2_FAST_CAL_RDY_STAT__REG             BMA2x2_OFFSET_CTRL_REG

/**    FAST COMPENSATION FOR X,Y,Z AXIS      **/
#define BMA2x2_CAL_TRIGGER__POS                5
#define BMA2x2_CAL_TRIGGER__LEN                2
#define BMA2x2_CAL_TRIGGER__MSK                0x60
#define BMA2x2_CAL_TRIGGER__REG                BMA2x2_OFFSET_CTRL_REG

/**    RESET OFFSET REGISTERS                **/
#define BMA2x2_RST_OFFSET__POS           7
#define BMA2x2_RST_OFFSET__LEN           1
#define BMA2x2_RST_OFFSET__MSK           0x80
#define BMA2x2_RST_OFFSET__REG           BMA2x2_OFFSET_CTRL_REG

/**     SLOW COMPENSATION  CUTOFF               **/
#define BMA2x2_COMP_CUTOFF__POS                 0
#define BMA2x2_COMP_CUTOFF__LEN                 1
#define BMA2x2_COMP_CUTOFF__MSK                 0x01
#define BMA2x2_COMP_CUTOFF__REG                 BMA2x2_OFFSET_PARAMS_REG

/**     COMPENSATION TARGET                  **/
#define BMA2x2_COMP_TARGET_OFFSET_X__POS        1
#define BMA2x2_COMP_TARGET_OFFSET_X__LEN        2
#define BMA2x2_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA2x2_COMP_TARGET_OFFSET_X__REG        BMA2x2_OFFSET_PARAMS_REG

#define BMA2x2_COMP_TARGET_OFFSET_Y__POS        3
#define BMA2x2_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA2x2_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA2x2_COMP_TARGET_OFFSET_Y__REG        BMA2x2_OFFSET_PARAMS_REG

#define BMA2x2_COMP_TARGET_OFFSET_Z__POS        5
#define BMA2x2_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA2x2_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA2x2_COMP_TARGET_OFFSET_Z__REG        BMA2x2_OFFSET_PARAMS_REG

/**     FIFO DATA SELECT              **/
#define BMA2x2_FIFO_DATA_SELECT__POS                 0
#define BMA2x2_FIFO_DATA_SELECT__LEN                 2
#define BMA2x2_FIFO_DATA_SELECT__MSK                 0x03
#define BMA2x2_FIFO_DATA_SELECT__REG                 BMA2x2_FIFO_MODE_REG



/*FIFO MODE*/

#define BMA2x2_FIFO_MODE__POS                 6
#define BMA2x2_FIFO_MODE__LEN                 2
#define BMA2x2_FIFO_MODE__MSK                 0xC0
#define BMA2x2_FIFO_MODE__REG                 BMA2x2_FIFO_MODE_REG


#define BMA2x2_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA2x2_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/** \endcond */


/* CONSTANTS */

#define BMA2x2_STAT1                             0
/**< It refers BMA2x2 STATUS_INT1 */
#define BMA2x2_STAT2                             1
/**< It refers BMA2x2 STATUS__INTR2_ */
#define BMA2x2_STAT3                             2
/**< It refers BMA2x2 STATUS_INTR_TAP */
#define BMA2x2_STAT4                             3
/**< It refers BMA2x2 STATUS_INTR_ORIENT */
#define BMA2x2_STAT5                             4
/**< It refers BMA2x2 STATUS_INTR_FIFO */

/* range and bandwidth */

#define BMA2x2_RANGE_2G                 3
/**< sets range to +/- 2G mode \see BMA2x2_set_range() */
#define BMA2x2_RANGE_4G                 5
/**< sets range to +/- 4G mode \see BMA2x2_set_range() */
#define BMA2x2_RANGE_8G                 8
/**< sets range to +/- 8G mode \see BMA2x2_set_range() */
#define BMA2x2_RANGE_16G                12
/**< sets range to +/- 16G mode \see BMA2x2_set_range() */


#define BMA2x2_BW_7_81HZ        0x08
 /**< sets bandwidth to LowPass 7.81  HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_15_63HZ       0x09
/**< sets bandwidth to LowPass 15.63 HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_31_25HZ       0x0A
/**< sets bandwidth to LowPass 31.25 HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_62_50HZ       0x0B
 /**< sets bandwidth to LowPass 62.50 HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_125HZ         0x0C
 /**< sets bandwidth to LowPass 125HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_250HZ         0x0D
/**< sets bandwidth to LowPass 250HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_500HZ         0x0E
/**< sets bandwidth to LowPass 500HZ \see BMA2x2_set_bandwidth() */
#define BMA2x2_BW_1000HZ        0x0F
 /**< sets bandwidth to LowPass 1000HZ \see BMA2x2_set_bandwidth() */

/*        SLEEP DURATION              */
#define BMA2x2_SLEEP_DURN_0_5MS        0x05
/* sets sleep duration to 0.5 ms  */
#define BMA2x2_SLEEP_DURN_1MS          0x06
 /* sets sleep duration to 1 ms */
#define BMA2x2_SLEEP_DURN_2MS          0x07
/* sets sleep duration to 2 ms */
#define BMA2x2_SLEEP_DURN_4MS          0x08
/* sets sleep duration to 4 ms */
#define BMA2x2_SLEEP_DURN_6MS          0x09
/* sets sleep duration to 6 ms*/
#define BMA2x2_SLEEP_DURN_10MS         0x0A
/* sets sleep duration to 10 ms */
#define BMA2x2_SLEEP_DURN_25MS         0x0B
/* sets sleep duration to 25 ms */
#define BMA2x2_SLEEP_DURN_50MS         0x0C
 /* sets sleep duration to 50 ms */
#define BMA2x2_SLEEP_DURN_100MS        0x0D
 /* sets sleep duration to 100 ms */
#define BMA2x2_SLEEP_DURN_500MS        0x0E
 /* sets sleep duration to 500 ms */
#define BMA2x2_SLEEP_DURN_1S           0x0F
  /* sets sleep duration to 1 s */

/*        LATCH DURATION              */
#define BMA2x2_LATCH_DURN_NON_LATCH    0x00
/* sets LATCH duration to NON LATCH  */
#define BMA2x2_LATCH_DURN_250MS        0x01
/* sets LATCH duration to 250 ms */
#define BMA2x2_LATCH_DURN_500MS        0x02
/* sets LATCH duration to 500 ms */
#define BMA2x2_LATCH_DURN_1S           0x03
 /* sets LATCH duration to 1 s */
#define BMA2x2_LATCH_DURN_2S           0x04
 /* sets LATCH duration to 2 s*/
#define BMA2x2_LATCH_DURN_4S           0x05
 /* sets LATCH duration to 4 s */
#define BMA2x2_LATCH_DURN_8S           0x06
 /* sets LATCH duration to 8 s */
#define BMA2x2_LATCH_DURN_LATCH        0x07
 /* sets LATCH duration to LATCH */
#define BMA2x2_LATCH_DURN_NON_LATCH1   0x08
 /* sets LATCH duration to NON LATCH1 */
#define BMA2x2_LATCH_DURN_250US        0x09
 /* sets LATCH duration to 250 Us */
#define BMA2x2_LATCH_DURN_500US        0x0A
 /* sets LATCH duration to 500 Us */
#define BMA2x2_LATCH_DURN_1MS          0x0B
 /* sets LATCH duration to 1 Ms */
#define BMA2x2_LATCH_DURN_12_5MS       0x0C
/* sets LATCH duration to 12.5 Ms */
#define BMA2x2_LATCH_DURN_25MS         0x0D
/* sets LATCH duration to 25 Ms */
#define BMA2x2_LATCH_DURN_50MS         0x0E
 /* sets LATCH duration to 50 Ms */
#define BMA2x2_LATCH_DURN_LATCH1       0x0F
/* sets LATCH duration to LATCH*/

/* mode settings */

#define BMA2x2_MODE_NORMAL             0
#define BMA2x2_MODE_LOWPOWER1          1
#define BMA2x2_MODE_SUSPEND            2
#define BMA2x2_MODE_DEEP_SUSPEND       3
#define BMA2x2_MODE_LOWPOWER2          4
#define BMA2x2_MODE_STANDBY            5

/* BMA2x2 AXIS      */

#define BMA2x2_X_AXIS           0
/**< It refers BMA2x2 X-axis */
#define BMA2x2_Y_AXIS           1
/**< It refers BMA2x2 Y-axis */
#define BMA2x2_Z_AXIS           2
/**< It refers BMA2x2 Z-axis */

/*  INTERRUPT TYPES    */

#define BMA2x2_LOW_G_INTR       0
#define BMA2x2_HIGH_G_X_INTR    1
#define BMA2x2_HIGH_G_Y_INTR    2
#define BMA2x2_HIGH_G_Z_INTR    3
#define BMA2x2_DATA_ENABLE      4
#define BMA2x2_SLOPE_X_INTR     5
#define BMA2x2_SLOPE_Y_INTR     6
#define BMA2x2_SLOPE_Z_INTR     7
#define BMA2x2_SINGLE_TAP_INTR  8
#define BMA2x2_DOUBLE_TAP_INTR  9
#define BMA2x2_ORIENT_INTR      10
#define BMA2x2_FLAT_INTR        11
#define BMA2x2_FIFO_FULL_INTR   12
#define BMA2x2_FIFO_WM_INTR     13

/*  INTERRUPTS PADS  */

#define BMA2x2_INTR1_LOW_G             0
#define BMA2x2_INTR2_LOW_G             1
#define BMA2x2_INTR1_HIGH_G            0
#define BMA2x2_INTR2_HIGH_G            1
#define BMA2x2_INTR1_SLOPE             0
#define BMA2x2_INTR2_SLOPE             1
#define BMA2x2_INTR1_SLOW_NO_MOTION    0
#define BMA2x2_INTR2_SLOW_NO_MOTION    1
#define BMA2x2_INTR1_DOUBLE_TAP        0
#define BMA2x2_INTR2_DOUBLE_TAP        1
#define BMA2x2_INTR1_SINGLE_TAP        0
#define BMA2x2_INTR2_SINGLE_TAP        1
#define BMA2x2_INTR1_ORIENT            0
#define BMA2x2_INTR2_ORIENT            1
#define BMA2x2_INTR1_FLAT              0
#define BMA2x2_INTR2_FLAT              1
#define BMA2x2_INTR1_NEWDATA           0
#define BMA2x2_INTR2_NEWDATA           1
#define BMA2x2_INTR1_FIFO_WM           0
#define BMA2x2_INTR2_FIFO_WM           1
#define BMA2x2_INTR1_FIFO_FULL         0
#define BMA2x2_INTR2_FIFO_FULL         1

/*       SOURCE REGISTER        */

#define BMA2x2_SOURCE_LOW_G            0
#define BMA2x2_SOURCE_HIGH_G           1
#define BMA2x2_SOURCE_SLOPE            2
#define BMA2x2_SOURCE_SLOW_NO_MOTION   3
#define BMA2x2_SOURCE_TAP              4
#define BMA2x2_SOURCE_DATA             5

#define BMA2x2_INTR1_OUTPUT      0
#define BMA2x2_INTR2_OUTPUT      1
#define BMA2x2_INTR1_LEVEL       0
#define BMA2x2_INTR2_LEVEL       1

/*    DURATION         */

#define BMA2x2_LOW_DURN                0
#define BMA2x2_HIGH_DURN               1
#define BMA2x2_SLOPE_DURN              2
#define BMA2x2_SLOW_NO_MOTION_DURN     3

/*      THRESHOLD        */

#define BMA2x2_LOW_THRES                0
#define BMA2x2_HIGH_THRES               1
#define BMA2x2_SLOPE_THRES              2
#define BMA2x2_SLOW_NO_MOTION_THRES     3


#define BMA2x2_LOW_G_HYST                0
#define BMA2x2_HIGH_G_HYST               1

#define BMA2x2_ORIENT_THETA             0
#define BMA2x2_FLAT_THETA               1

#define BMA2x2_I2C_SELECT               0
#define BMA2x2_I2C_EN                   1

/*    COMPENSATION           */

#define BMA2x2_SLOW_COMP_X              0
#define BMA2x2_SLOW_COMP_Y              1
#define BMA2x2_SLOW_COMP_Z              2

/*       OFFSET TRIGGER          */

#define BMA2x2_CUT_OFF                  0
#define BMA2x2_OFFSET_TRIGGER_X         1
#define BMA2x2_OFFSET_TRIGGER_Y         2
#define BMA2x2_OFFSET_TRIGGER_Z         3

/*       GP REGISTERS           */

#define BMA2x2_GP0                      0
#define BMA2x2_GP1                      1

/*    SLO NO MOT REGISTER          */

#define BMA2x2_SLOW_NO_MOTION_ENABLE_X          0
#define BMA2x2_SLOW_NO_MOTION_ENABLE_Y          1
#define BMA2x2_SLOW_NO_MOTION_ENABLE_Z          2
#define BMA2x2_SLOW_NO_MOTION_ENABLE_SELECT     3


/* wake up */

#define BMA2x2_WAKE_UP_DURN_20MS         0
#define BMA2x2_WAKE_UP_DURN_80MS         1
#define BMA2x2_WAKE_UP_DURN_320MS        2
#define BMA2x2_WAKE_UP_DURN_2560MS       3


/* LG/HG thresholds are in LSB and depend on RANGE setting */
/* no range check on threshold calculation */

#define BMA2x2_SELFTEST0_ON            1
#define BMA2x2_SELFTEST1_ON            2

#define BMA2x2_EE_W_OFF                 0
#define BMA2x2_EE_W_ON                  1

/* Resolution Settings */
#define BMA2x2_RESOLUTION_12_BIT        0
#define BMA2x2_RESOLUTION_10_BIT        1
#define BMA2x2_RESOLUTION_14_BIT        3

#define BMA2x2           0x16
#define BMA280           0x17
#define BMA222E          0x18
#define BMA250E          0x19
/** Macro to convert floating point
low-g-thresholds in G to 8-bit register values.<br>
  * Example: BMA2x2_LOW_TH_IN_G( 0.3, 2.0) generates
  * the register value for 0.3G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_LOW_THRES_IN_G(gthres, range)  ((256 * gthres) / range)

/** Macro to convert floating point high-g-thresholds
    in G to 8-bit register values.<br>
  * Example: BMA2x2_HIGH_TH_IN_G( 1.4, 2.0)
  * generates the register value for 1.4G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_HIGH_THRES_IN_G(gthres, range)   ((256 * gthres) / range)

/** Macro to convert floating point low-g-hysteresis
in G to 8-bit register values.<br>
  * Example: BMA2x2_LOW_HY_IN_G( 0.2, 2.0)
  *generates the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_LOW_HYST_IN_G(ghyst, range)   ((32 * ghyst) / range)

/** Macro to convert floating point high-g-hysteresis
   in G to 8-bit register values.<br>
  * Example: BMA2x2_HIGH_HY_IN_G( 0.2, 2.0) generates
  *the register value for 0.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */
#define BMA2x2_HIGH_HYST_IN_G(ghyst, range)    ((32 * ghyst) / range)


/** Macro to convert floating point G-thresholds
    to 8-bit register values<br>
  * Example: BMA2x2_SLOPE_TH_IN_G( 1.2, 2.0)
  * generates the register value for 1.2G threshold in 2G mode.
  * \brief convert g-values to 8-bit value
 */

#define BMA2x2_SLOPE_THRES_IN_G(gthres, range)    ((128 * gthres) / range)
/** user defined Enums
* Example..
* enum {
* E_YOURDATA1, < <DOXY Comment for E_YOURDATA1>
* E_YOURDATA2  < <DOXY Comment for E_YOURDATA2>
* };
*/
/** Example...
* struct DUMMY_STRUCT {
* data1, < <DOXY Comment for data1>
* data2  < <DOXY Comment for data1>
* };*/
/****************************************************************************
 * Description: *//**\brief This API reads the data from
 * the given register continuously
 *
 *
 *
 *
 *  \param u8 v_addr_u8, u8 *v_data_u8
 *                       v_addr_u8 -> Address of the register
 *                       v_data_u8 -> address of the variable,
 *                               read v_value_u8 will be kept
 * \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_burst_read(u8 v_addr_u8,
u8 *v_data_u8, u32 v_len_u32);
/****************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	 \param  bma2x2_t *bma2x2 structure pointer
 *
 *	While changing the parameter of the bma2x2_t
 *	consider the following point:
 *	Changing the reference v_value_u8 of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference v_value_u8 of the parameter
 *	(Better case don't change the reference v_value_u8 of the parameter)
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_init(struct bma2x2_t *bma2x2);
/****************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                the data is written in the corresponding register address
 *
 *
 *
 *
 *  \param u8 v_adr_u8, u8 *v_data_u8
 *                       v_adr_u8 -> Address of the register
 *                       v_data_u8 -> address of the variable,
 *                               read v_value_u8 will be kept
 *          v_len_u8  -> Length of the Data
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_write_reg(u8 v_adr_u8,
u8 *v_data_u8, u8 v_len_u8);
/****************************************************************************
 * Description: *//**\brief This API reads the data from
 *           the given register address
 *
 *
 *
 *
 *  \param u8 v_adr_u8, u8 *v_data_u8, u8 v_len_u8
 *         v_adr_u8 -> Address of the register
 *         v_data_u8 -> address of the variable, read v_value_u8 will be kept
 *         v_len_u8  -> Length of the data
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_reg(u8 v_adr_u8,
u8 *v_data_u8, u8 v_len_u8);
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data X values
 *                          from location 02h and 03h
 *
 *
 *
 *
 *  \param short  *v_accel_x_s16 : pointer holding the data of accel
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_x(s16 *v_accel_x_s16);
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data X values of
 *                 8bit  resolution  from location 03h
 *
 *
 *
 *
 *  \param short  *v_accel_x_s8   :  pointer holding the data of x
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_x(
s8 *v_accel_x_s8);
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values
 *                          from location 04h and 05h
 *
 *
 *
 *
 *  \param short  *v_accel_y_s16   :  pointer holding the data of y
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_y(s16 *v_accel_y_s16);
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values of
 *                 8bit  resolution  from location 05h
 *
 *
 *
 *
 *  \param short  *v_accel_y_s8   :  pointer holding the data of y
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_y(
s8 *v_accel_y_s8);
/****************************************************************************
 * Description: *//**\brief This API reads acceleration data Z values
 *                          from location 06h and 07h
 *
 *
 *
 *
 *  \param short  *v_accel_z_s16 : pointer holding the data of accel
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_z(s16 *v_accel_z_s16);
/***************************************************************************
 * Description: *//**\brief This API reads acceleration data Y values of
 *                 8bit  resolution  from location 07h
 *
 *
 *
 *
 *  \param short  *v_accel_z_s8 : pointer holding the data of z
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_elight_resolution_z(
s8 *v_accel_z_s8);
/****************************************************************************
 * Description: *//**\brief This API reads acceleration data X,Y,Z values
 *                          from location 02h to 07h
 *
 *
 *
 *
 *  \param bma2x2accel_data * accel : pointer holding the data of accel
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_xyz(
struct bma2x2_accel_data *accel);
/****************************************************************************
 * Description: *//**\brief This API reads acceleration of 8 bit resolution
 *                          v_data_u8 of X,Y,Z values
 *                          from location 03h , 05h and 07h
 *
 *
 *
 *
 *  \param bma2x2_accel_eight_resolution * accel :
 *	pointer holding the data of accel
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_eight_resolution_xyz(
struct bma2x2_accel_eight_resolution *accel);
/****************************************************************************
 * Description: *//**\brief This API Reads tap slope status register byte
 *                          from location 0Bh
 *
 *
 *
 *
 *   \param u8 * v_stat_tap_u8 : Pointer holding the status of tap
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_tap_stat(
u8 *v_stat_tap_u8);
/***************************************************************************
 * Description: *//**\brief This API Reads orient status register byte
 *                          from location 0Ch
 *
 *
 *
 *
 *  \param u8 *v_stat_orient_u8 : Pointer holding the status_orient
 *
 *
 *
 * \return Result of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_orient_stat(
u8 *v_stat_orient_u8);
/*****************************************************************************
 * Description: *//**\brief This API Reads fifo status register byte
 *                          from location 0Eh
 *
 *
 *
 *
 *  \param u8 *v_stat_fifo_u8 : Pointer holding the status of fifo
 *
 *
 *
 * \return Result of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_stat(
u8 *v_stat_fifo_u8);
/***************************************************************************
 * Description: *//**\brief This API Reads fifo frame count
 *       bits from location 0Eh
 *
 *
 *
 *
 * \param u8 *v_frame_count_u8 : Pointer holding the fifo frame count
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_frame_count(
u8 *v_frame_count_u8);
/***************************************************************************
 * Description: *//**\brief This API Reads fifo overrun bits
 *        from location 0Eh
 *
 *
 *
 *
 *\param u8 *v_fifo_overrun_u8 : Pointer holding the fifo overrun
 *
 *
 *
 * \return Result of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_overrun(
u8 *v_fifo_overrun_u8);
/****************************************************************************
 * Description: *//**\brief This API Reads interrupt status register byte
 *                          from location 09h
 *
 *
 *
 *
 *\param u8 * v_intr_stat_u8 : Pointer holding the interrupt status register
 *
 *
 *
 * \return Result of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_stat(
u8 *v_intr_stat_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 *                 the Ranges(g values) of the sensor in the register 0x0F
 *					bit from 0 to 3
 *
 *
 *
 *
 *  \param u8 * v_range_u8 : Pointer holding the Accel Range
 *                     v_range_u8 -> 3 -> BMA2x2_RANGE_2G
 *                              5 -> BMA2x2_RANGE_4G
 *                              8 -> BMA2x2_RANGE_8G
 *                              12 -> BMA2x2_RANGE_16G
 *
 *
 *
 * \return communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_range(u8 *v_range_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *  Ranges(g v_value_u8) of the sensor in the register 0x0F
 *					bit from 0 to 3
 *
 *
 *
 *
 *
 *  \param u8 v_range_u8 : The v_value_u8 of Range
 *             v_range_u8 ->   3 -> BMA2x2_RANGE_2G
 *                        5 -> BMA2x2_RANGE_4G
 *                        8 -> BMA2x2_RANGE_8G
 *                        12 -> BMA2x2_RANGE_16G
 *
 * \return communication results
 *
 *
 ****************************************************************************/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_range(u8 v_range_u8);
/***********************************************************************
 * Description: *//**\brief This API is used to get
 *       the bandwidth of the sensor in the register
 *			0x10 bit from 0 to 4
 *
 *
 *
 *
 *  \param  u8 * v_bw_u8 : Pointer holding the bandwidth
 *                v_bw_u8 ->	BMA2x2_BW_7_81HZ        0x08
 *						BMA2x2_BW_15_63HZ       0x09
 *						BMA2x2_BW_31_25HZ       0x0A
 *						BMA2x2_BW_62_50HZ       0x0B
 *						BMA2x2_BW_125HZ         0x0C
 *						BMA2x2_BW_250HZ         0x0D
 *						BMA2x2_BW_500HZ         0x0E
 *						BMA2x2_BW_1000HZ        0x0F
 *
 *
 *
 * \return	results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_bw(u8 *v_bw_u8);
/************************************************************************
 * Description: *//**\brief This API is used to set
 *           Bandwidth of the sensor in the register
 *			0x10 bit from 0 to 4
 *
 *
 *
 *
 *  \param u8 v_bw_u8: The v_value_u8 of Bandwidth
 *              v_bw_u8 ->	BMA2x2_BW_7_81HZ        0x08
 *						BMA2x2_BW_15_63HZ       0x09
 *						BMA2x2_BW_31_25HZ       0x0A
 *						BMA2x2_BW_62_50HZ       0x0B
 *						BMA2x2_BW_125HZ         0x0C
 *						BMA2x2_BW_250HZ         0x0D
 *						BMA2x2_BW_500HZ         0x0E
 *						BMA2x2_BW_1000HZ        0x0F
 *
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_bw(u8 v_bw_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get the operating
 *	modes of the sensor in the register 0x11 and 0x12
 *	Register 0x11 - bit from 5 to 7
 *	Register 0x12 - bit from 5 and 6
 *
 *
 *
 *
 *
 *  \param u8 * v_power_mode_u8 : Pointer holding the Mode
 *  v_power_mode_u8 ->   0 -> NORMAL
 *                       1 -> LOWPOWER1
 *                       2 -> SUSPEND
 *                       3 -> DEEP_SUSPEND
 *                       4 -> LOWPOWER2
 *                       5 -> STANDBY
 *
 *
 *
 * \return results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_power_mode(
u8 *v_power_mode_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the operating Modes of the sensor in the register 0x11 and 0x12
 *	Register 0x11 - bit from 5 to 7
 *	Register 0x12 - bit from 5 and 6
 *
 *
 *
 *
 *  \param u8 v_power_mode_u8: The v_value_u8 of Mode
 *  v_power_mode_u8 ->   0 -> NORMAL
 *                       1 -> LOWPOWER1
 *                       2 -> SUSPEND
 *                       3 -> DEEP_SUSPEND
 *                       4 -> LOWPOWER2
 *                       5 -> STANDBY
 *
 * \return results of bus communication function
 *
 *
 *****************************************************************************/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_power_mode(u8 v_power_mode_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to select the power mode
 *
 *  \param u8 v_power_mode_u8: The v_value_u8 of Mode
 *  v_power_mode_u8 ->   0 -> NORMAL
 *                       1 -> LOWPOWER1
 *                       2 -> SUSPEND
 *                       3 -> DEEP_SUSPEND
 *                       4 -> LOWPOWER2
 *                       5 -> STANDBY
 *
 * \return results of bus communication function
 *
 *
 *****************************************************************************/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_mode_value(u8 v_power_mode_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the sleep duration v_intr_stat_u8us of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *
 *
 *  \param  u8 *v_sleep_durn_u8 : Pointer holding the sleep duration time
 *	BMA2x2_SLEEP_DURN_0_5MS        0x05
 *	BMA2x2_SLEEP_DURN_1MS          0x06
 *	BMA2x2_SLEEP_DURN_2MS          0x07
 *	BMA2x2_SLEEP_DURN_4MS          0x08
 *	BMA2x2_SLEEP_DURN_6MS          0x09
 *	BMA2x2_SLEEP_DURN_10MS         0x0A
 *	BMA2x2_SLEEP_DURN_25MS         0x0B
 *	BMA2x2_SLEEP_DURN_50MS         0x0C
 *	BMA2x2_SLEEP_DURN_100MS        0x0D
 *	BMA2x2_SLEEP_DURN_500MS        0x0E
 *	BMA2x2_SLEEP_DURN_1S           0x0F
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
 **********************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_sleep_durn(u8 *v_sleep_durn_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	Sleep Duration of the sensor in the register 0x11
 *	Register 0x11 - bit from 0 to 3
 *
 *
 *
 *
 *  \param u8 v_sleep_durn_u8: The v_value_u8 of Sleep Duration time
 *        v_sleep_durn_u8 ->
 *	BMA2x2_SLEEP_DURN_0_5MS        0x05
 *	BMA2x2_SLEEP_DURN_1MS          0x06
 *	BMA2x2_SLEEP_DURN_2MS          0x07
 *	BMA2x2_SLEEP_DURN_4MS          0x08
 *	BMA2x2_SLEEP_DURN_6MS          0x09
 *	BMA2x2_SLEEP_DURN_10MS         0x0A
 *	BMA2x2_SLEEP_DURN_25MS         0x0B
 *	BMA2x2_SLEEP_DURN_50MS         0x0C
 *	BMA2x2_SLEEP_DURN_100MS        0x0D
 *	BMA2x2_SLEEP_DURN_500MS        0x0E
 *	BMA2x2_SLEEP_DURN_1S           0x0F
 *
 *
 * \return results of bus communication function
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleep_durn(u8 v_sleep_durn_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get the sleep timer mode
 *			in the register 0x12 bit 5
 *
 *
 *
 *
 *  \param  u8 *v_sleep_timer_u8 : Pointer holding the sleep_tmr
 *                  v_sleep_timer_u8 -> [0:1]
 *                  0 => enable EventDrivenSampling(EDT)
 *                  1 => enable Equidistant sampling v_power_mode_u8(EST)
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_sleep_timer_mode(
u8 *v_sleep_timer_u8);
/**************************************************************************
 * Description: *//**\brief This API is used to set
 *   the sleep timer mode in the register 0x12 bit 5
 *
 *
 *
 *
 *  \param u8 v_sleep_timer_u8:	The v_value_u8 of sleep timer mode
 *                  v_sleep_timer_u8 -> [0:1]
 *                  0 => enable EventDrivenSampling(EDT)
 *                  1 => enable Equidistant sampling mode(EST)
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleep_timer_mode(u8 v_sleep_timer_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get high bandwidth
 *		in the register 0x13 bit 7
 *
 *
 *
 *  \param u8 *v_high_bw_u8 : Pointer holding the high bandwidth
 *       v_high_bw_u8 ->  1 -> Unfiltered High Bandwidth
 *                   0 -> Filtered Low Bandwidth
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_high_bw(u8 *v_high_bw_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set high bandwidth
 *			in the register 0x13 bit 7
 *
 *
 *
 *  \param u8 v_high_bw_u8: The v_value_u8 of high bandwidth
 *       v_high_bw_u8 ->   1 -> Unfiltered High Bandwidth
 *                    0 -> Filtered Low Bandwidth
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_high_bw(u8 v_high_bw_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get shadow dis
 *		in the register 0x13 bit 6
 *
 *
 *
 *  \param u8 *v_shadow_dis_u8 : Pointer holding the shadow disable
 *           v_shadow_dis_u8 -> 1 -> No MSB Locking
 *                         0 -> MSB is Locked
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_shadow_dis(u8 *v_shadow_dis_u8);
/**************************************************************************
 * Description: *//**\brief This API is used to set shadow dis
 *		in the register 0x13 bit 6
 *
 *
 *
 *  \param u8 v_shadow_dis_u8: The v_value_u8 of shadow dis
 *          v_shadow_dis_u8 ->   1 -> No MSB Locking
 *                          0 -> MSB is Locked
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_shadow_dis(u8 v_shadow_dis_u8);
/*****************************************************************************
 * Description: *//**\brief
 *                      This function is used for the soft reset
 *     The soft reset register will be written with 0xB6 in the register 0x14.
 *
 *
 *
 *  \param None
 *
 *
 *
 * \return results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_soft_rst(void);
/**************************************************************************
 * Description: *//**\brief This API is used to update the register values
 *
 *
 *
 *
 *  param:  None
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_update_image(void);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *
 *
 *
 *
 *  \param u8 *v_intr_type_u8: The pointer holding
 *               the  interrupt type
 *                        0 -> BMA2x2_LOW_G_INTR
 *                        1 -> BMA2x2_HIGH_G_X_INTR
 *                        2 -> BMA2x2_HIGH_G_Y_INTR
 *                        3 -> BMA2x2_HIGH_G_Z_INTR
 *                        4 -> BMA2x2_DATA_ENABLE
 *                        5 -> SLOPE_X_INT
 *                        6 -> SLOPE_Y_INT
 *                        7 -> SLOPE_Z_INT
 *                        8 -> SINGLE_Tap_INT
 *                        9 -> DOUBLE_Tap_INT
 *                       10 -> ORIENT_INT
 *                       11 -> FLAT_INT
 *       u8 v_value_u8: The value of interrupts enable
 *					1 -> enable interrupt
 *					0 -> disable interrupt
 *
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_enable(u8 v_intr_type_u8,
u8 *v_value_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *
 *
 *
 *
 *  \param u8 v_intr_type_u8: The value of interrupt type
 *                        0 -> Low_G_INTR
 *                        1 -> High_G_X_INTR
 *                        2 -> High_G_Y_INTR
 *                        3 -> High_G_Z_INTR
 *                        4 -> DATA_EN
 *                        5 -> SLOPE_X_INTR
 *                        6 -> SLOPE_Y_INTR
 *                        7 -> SLOPE_Z_INTR
 *                        8 -> SINGLE_TAP_INTR
 *                        9 -> Double_TAP_INTR
 *                       10 -> Orient_INTR
 *                       11 -> Flat_INTR
 *       u8 v_value_u8: The value of interrupts enable
 *				1 -> enable interrupt
 *				0 -> disable interrupt
 *
 *
 *
 * \return results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_enable(u8 v_intr_type_u8,
u8 v_value_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the interrupt fifo full enable interrupt status in the register 0x17 bit 5
 *
 *
 *
 *
 *  \param u8 *v_fifo_full_u8 :Pointer holding the FIFO Full fifo_full bit
 *                    FIFO Full -> [0:1]
 *                              0 --> Clear
 *                              1 --> Set
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_fifo_full(u8 *v_fifo_full_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the interrupt fifo_full enable interrupt status in the register 0x17 bit 5
 *
 *
 *
 *
 *  \param u8 v_fifo_full_u8: The value of FIFO full interrupt status bit
 *            FIFO Full -> [0:1]
 *                      0 --> Clear
 *                      1 --> Set
 *
 *
 *  \return  results of bus communication function communication results
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_fifo_full(u8 v_fifo_full_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the interrupt fwm enable v_intr_stat_u8us in the register 0x17 bit 6
 *
 *
 *
 *
 *  \param u8 *v_fifo_wm_u8 : Pointer holding the FIFO Water Mark
 *                   v_fifo_wm_u8 -> [0:1]
 *                           0 --> Clear
 *                           1 --> Set
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_fifo_wm(u8 *v_fifo_wm_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the interrupt fwm enable v_intr_stat_u8us in the register 0x17 bit 6
 *
 *
 *
 *
 *  \param u8 v_fifo_wm_u8: The value of FIFO water mark
 *        FIFO Water Mark -> [0:1]
 *                0 --> Clear
 *                1 --> Set
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_fifo_wm(u8 v_fifo_wm_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of slow/no motion interrupt in the register 0x18.
 * bit from 0 to 3
 *
 *
 *
 *
 *  \param u8 v_channel_u8:
 *          The value of slow/no motion channel
 *           v_channel_u8 -->
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_X     ->     0
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Y     ->     1
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Z     ->     2
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_SEL   ->     3
 *      u8 *v_slow_no_motion_u8: Pointer holding the slow no motion interrupt
 *           v_slow_no_motion_u8
 *				enable --> 1
 *				disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_slow_no_motion(u8 v_channel_u8,
u8 *v_slow_no_motion_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the v_intr_stat_u8us of slow/no motion interrupt in the register 0x18.
 * bit from 0 to 3
 *
 *
 *
 *
 *  \param  u8 v_channel_u8:
 *          The value of slow/no motion channel
 *           v_channel_u8 -->
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_X     ->     0
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Y     ->     1
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_Z     ->     2
 *           BMA2x2_ACCEL_SLOW_NO_MOTION_ENABLE_SELECT ->     3
 *      u8 *v_slow_no_motion_u8: The value of slow/no motion v_intr_stat_u8us
 *           v_slow_no_motion_u8
 *				enable --> 1
 *				disable --> 0
 *
 *
 *
 *  \return  results of bus communication function communication results
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_slow_no_motion(u8 v_channel_u8,
u8 v_slow_no_motion_u8);
/****************************************************************************
 * Description: *//**\brief  This API is used to get
 * the v_intr_stat_u8us of low interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 0
 * _INTR2_ -> register 0x1B bit 0
 *
 *
 *
 *
 * param u8 v_channel_u8 : The value of low interrupt channel
 *                       v_channel_u8 -->
 *                       BMA2x2_ACCEL_INTR1_LOW_G     ->    0
 *                        BMA2x2_ACCEL_INTR2_LOW_G     ->    1
 *  u8 *v_intr_low_g_u8 : Pointer holding the low interrupt
 *                       v_intr_low_g_u8
 *							enable --> 1
 *							disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_low_g(u8 v_channel_u8,
u8 *v_intr_low_g_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the v_intr_stat_u8us of low interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 0
 * _INTR2_ -> register 0x1B bit 0
 *
 *
 *
 *
 * param u8 v_channel_u8 : The value of low interrupt channel
 *                       v_channel_u8 -->
 *                       BMA2x2_ACCEL_INTR1_LOW_G     ->    0
 *                       BMA2x2_ACCEL_INTR2_LOW_G     ->    1
 *  u8 *v_intr_low_u8 : The value of low interrupt
 *                       v_intr_low_u8
 *							enable --> 1
 *							disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
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
*****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_low_g(u8 v_channel_u8,
u8 v_intr_low_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of high interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 1
 * _INTR2_ -> register 0x1B bit 1
 *
 *
 *
 *
 *  \param u8 v_channel_u8: The value of high interrupt channel
 *                           v_channel_u8 -->
 *                           BMA2x2_ACCEL_INTR1_HIGH_G     ->    0
 *                           BMA2x2_ACCEL_INTR2_HIGH_G     ->    1
 *       u8 *v_intr_high_g_u8: Pointer holding the  high interrupt
 *                           high_g
 *                           enable --> 1
 *							 disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_high_g(u8 v_channel_u8,
u8 *v_intr_high_g_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the v_intr_stat_u8us of high interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 1
 * _INTR2_ -> register 0x1B bit 1
 *
 *
 *
 *
 *  \param u8 v_channel_u8: The value of high interrupt channel
 *                           v_channel_u8 -->
 *                           BMA2x2_ACCEL_INTR1_HIGH_G     ->    0
 *                           BMA2x2_ACCEL_INTR2_HIGH_G     ->    1
 *       u8 *v_int_high_g_u8: The value of high interrupt
 *                           v_int_high_g_u8
 *                           enable --> 1
 *							 disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_high_g(u8 v_channel_u8,
u8 v_intr_high_g_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of slope interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 2
 * _INTR2_ -> register 0x1B bit 2
 *
 *
 *
 ** \param u8 v_channel_u8: Pointer holding the slope channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_SLOPE ->    0
 *	BMA2x2_ACCEL_INTR2_SLOPE ->    1
 *
 *	u8 *v_intr_slope_u8: Pointer holding the slope value
 *	v_intr_slope_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_slope(u8 v_channel_u8,
u8 *v_intr_slope_u8);
/*************************************************************************
 * Description: *//**\brief This API is used to set
 * the v_intr_stat_u8us of slope interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 2
 * _INTR2_ -> register 0x1B bit 2
 *
 *
 *
 *
 * \param u8 v_channel_u8:The value of slope channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_SLOPE     ->    0
 *	BMA2x2_ACCEL_INTR2_SLOPE     ->    1
 *
 *  u8  v_intr_slope_u8: The slope v_intr_stat_u8us value
 *	v_intr_slope_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***********************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_slope(u8 v_channel_u8,
u8 v_intr_slope_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of slow/no motion interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 3
 * _INTR2_ -> register 0x1B bit 3
 *
 *
 *
 *
 * \param u8 v_channel_u8:
 *	The value of slow/no motion channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_SLOW_NO_MOTION  ->  0
 *					BMA2x2_ACCEL_INTR2_SLOW_NO_MOTION  ->  1
 *
 *	u8 *v_intr_slow_no_motion_u8: Pointer holding the slow/no value
 *	v_intr_slow_no_motion_u8
 *	enable --> 1
 *	disable --> 0
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_slow_no_motion(u8 v_channel_u8,
u8 *v_intr_slow_no_motion_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the v_intr_stat_u8us of slow/no motion interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 3
 * _INTR2_ -> register 0x1B bit 3
 *
 *
 *
 *
 *	\param u8 v_channel_u8:The value of slow/no motion v_channel_u8 number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_SLOW_NO_MOTION     ->    0
 *	BMA2x2_ACCEL_INTR2_SLOW_NO_MOTION     ->    1
 *
 *	u8 v_intr_slow_no_motion_u8:The slow/no motion v_intr_stat_u8us value
 *	v_intr_slow_no_motion_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_slow_no_motion(u8 v_channel_u8,
u8 v_intr_slow_no_motion_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get
 *  the v_intr_stat_u8us of double tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 4
 * _INTR2_ -> register 0x1B bit 4
 *
 *
 *
 *
 * \param u8 v_channel_u8:
 *	The value of double tap channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_DOUBLE_TAP ->  0
 *					BMA2x2_ACCEL_INTR2_DOUBLE_TAP ->  1
 *
 *	u8 *v_intr_double_tap_u8: Pointer holding the double tap interrupt value
 *	v_intr_double_tap_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_double_tap(u8 v_channel_u8,
u8 *v_intr_double_tap_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the v_intr_stat_u8us of double tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 4
 * _INTR2_ -> register 0x1B bit 4
 *
 *
 *
 *
 * \param u8 v_channel_u8:
 *	The value of double tap interrupt channel number
 *  v_channel_u8 --> BMA2x2_ACCEL_INTR1_DOUBLE_TAP -> 0
 *					 BMA2x2_ACCEL_INTR2_DOUBLE_TAP -> 1
 *
 *	u8  v_intr_double_tap_u8: The double tap interrupt status value
 *	v_intr_double_tap_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_double_tap(u8 v_channel_u8,
u8 v_intr_double_tap_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of single tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 5
 * _INTR2_ -> register 0x1B bit 5
 *
 *
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of single tap interrupt channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_SINGLE_TAP -> 0
 *					BMA2x2_ACCEL_INTR2_SINGLE_TAP -> 1
 *
 *  u8 *v_intr_single_tap_u8: Pointer holding the single tap interrupt value
 *  v_intr_single_tap_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_single_tap(u8 v_channel_u8,
u8 *v_intr_single_tap_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the status of single tap interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 5
 * _INTR2_ -> register 0x1B bit 5
 *
 *
 *
 *
 * \param u8 v_channel_u8:
 *	The value of single tap interrupt channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_SINGLE_TAP -> 0
 *					BMA2x2_ACCEL_INTR2_SINGLE_TAP -> 1
 *
 *  u8 v_intr_single_tap_u8: The single tap interrupt status value
 *  v_intr_single_tap_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_single_tap(u8 v_channel_u8,
u8 v_intr_single_tap_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the interrupt status of orient interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 6
 * _INTR2_ -> register 0x1B bit 6
 *
 *
 *
 *
 ** \param u8 v_channel_u8:
 *	The value of orient interrupt channel number
 *	v_channel_u8 -->	BMA2x2_ACCEL_INTR1_ORIENT     ->    0
 *				BMA2x2_ACCEL_INTR2_ORIENT     ->    1
 *
 *  u8 *v_intr_orient_u8: Pointer holding the orient interrupt value
 *	v_intr_orient_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_orient(u8 v_channel_u8,
u8 *v_intr_orient_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the interrupt status of orient interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 6
 * _INTR2_ -> register 0x1B bit 6
 *
 *
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of orient interrupt channel number
 *	v_channel_u8 -->	BMA2x2_ACCEL_INTR1_ORIENT     ->    0
 *				BMA2x2_ACCEL_INTR2_ORIENT     ->    1
 *
 *  u8  v_intr_orient_u8:
 *	The orient interrupt value
 *	v_intr_orient_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_orient(u8 v_channel_u8,
u8 v_intr_orient_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of flat interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 7
 * _INTR2_ -> register 0x1B bit 7
 *
 *
 *
 *
 *\param u8 v_channel_u8:
 *	The value of flat interrupt channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_FLAT     ->    0
 *					BMA2x2_ACCEL_INTR2_FLAT     ->    1
 *
 *  u8 *v_intr_flat_u8: Pointer holding the flat interrupt value
 *	v_intr_flat_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_flat(u8 v_channel_u8,
u8 *v_intr_flat_u8);
/**************************************************************************
 * Description: *//**\brief This API is used to set
 * the v_intr_stat_u8us of flat interrupt in the register 0x19 and 0x1B
 * INT1 -> register 0x19 bit 7
 * _INTR2_ -> register 0x1B bit 7
 *
 *
 *
 *
 * \param u8 v_channel_u8:
 *	The value of flat interrupt channel number
 *	v_channel_u8 --> BMA2x2_ACCEL_INTR1_FLAT     ->    0
 *				BMA2x2_ACCEL_INTR2_FLAT     ->    1
 *
 *	u8 v_intr_flat_u8:
 *	The flat interrupt v_intr_stat_u8us value
 *	v_intr_flat_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_flat(u8 v_channel_u8,
u8 v_intr_flat_u8);
/**************************************************************************
 * Description: *//**\brief This API is used to get
 * the interrupt status of new data in the register 0x19
 * INT1 -> register 0x19 bit 0
 * _INTR2_ -> register 0x19 bit 7
 *
 *
 *
 *	\param u8 v_channel_u8:
 *		The value of new data interrupt channel number
 *		v_channel_u8 -->BMA2x2_ACCEL_INTR1_NEWDATA ->  0
 *                      BMA2x2_ACCEL_INTR2_NEWDATA ->  1
 *
 *	u8 *intr_newdata_u8: Pointer holding the new data interrupt value
 *	intr_newdata_u8
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_new_data(u8 v_channel_u8,
u8 *intr_newdata_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 * the interrupt status of new data in the register 0x19
 * INT1 -> register 0x19 bit 0
 * _INTR2_ -> register 0x19 bit 7
 *
 *
 *
 * \param u8 v_channel_u8:
 *	The value of new data interrupt channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_NEWDATA -> 0
 *					BMA2x2_ACCEL_INTR2_NEWDATA -> 1
 *
 *	u8 intr_newdata_u8: The new data interrupt value
 *	intr_newdata_u8 :
 *	enable --> 1
 *	disable --> 0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_new_data(u8 v_channel_u8,
u8 intr_newdata_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get the fwm interrupt1 data
 * in the register 0x1A
 * INT1 -> register 0x1A bit 1
 *
 *  \param  u8 *v_intr1_fifo_wm_u8 :
 *	Pointer holding the interrupt1 FIFO watermark
 *	v_intr1_fifo_wm_u8 --> [0:1]
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr1_fifo_wm(u8 *v_intr1_fifo_wm_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set the fwm interrupt1 data
 *in the register 0x1A
 * INT1 -> register 0x1A bit 1
 *
 *
 *	u8 v_intr1_fifo_wm_u8:
 *	The fifo watermark  interrupt1 status value
 *	int1_fwm
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr1_fifo_wm(u8 v_intr1_fifo_wm_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the fwm interrupt1 v_data_u8 in the register 0x1A
 * _INTR2_ -> register 0x1A bit 6
 *
 *  \param  u8 *v_intr2_fifo_wm_u8 :
 *	pointer holding The fifo watermark  interrupt2 status value
 *	v_intr2_fifo_wm_u8 --> [0:1]
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr2_fifo_wm(u8 *v_intr2_fifo_wm_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the fwm interrupt1 v_data_u8 in the register 0x1A
 *	_INTR2_ -> register 0x1A bit 6
 *
 *
 *	u8 v_intr2_fifo_wm_u8:
 *	The fifo watermark  interrupt2 status value
 *	v_intr2_fifo_wm_u8
 *	0 --> disable
 *	1 --> enable
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr2_fifo_wm(u8 v_intr2_fifo_wm_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo_full interrupt1 v_data_u8 in the register 0x1A
 *	INT1 -> register 0x1A bit 2
 *
 *
 *
 *  \param u8 *v_intr1_fifo_full_u8 : Pointer holding the fifo full
 *	v_intr1_fifo_full_u8
 *	0 --> enable
 *	1 --> disable
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 *****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr1_fifo_full(
u8 *v_intr1_fifo_full_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the fifo_full interrupt1 v_data_u8 in the register 0x1A
 *	INT1 -> register 0x1A bit 2
 *
 *
 *
 *	u8 v_intr1_fifo_full_u8:
 *	The fifo_full interrupt1 status value
 *	v_intr1_fifo_full_u8
 *	0 --> enable
 *	1 --> disable
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr1_fifo_full(u8 v_intr1_fifo_full_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo_full interrupt1 v_data_u8 in the register 0x1A
 *	_INTR2_ -> register 0x1A bit 5
 *
 *
 *
 *  \param u8 *v_intr2_fifo_full_u8 : Pointer holding the fifo full
 *	v_intr2_fifo_full_u8
 *	0 --> enable
 *	1 --> disable
 *
 *
 *
 *  \return  results of bus communication function
 *
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr2_fifo_full(
u8 *v_intr2_fifo_full_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the fifo_full interrupt1 v_data_u8 in the register 0x1A
 *	_INTR2_ -> register 0x1A bit 5
 *
 *
 *
 *	u8 v_intr2_fifo_full_u8:
 *	The fifo_full interrupt1 status value
 *	v_intr2_fifo_full_u8
 *	0 --> enable
 *	1 --> disable
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr2_fifo_full(u8 v_intr2_fifo_full_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the source status v_data_u8 in the register 0x1E bit from 0 to 5
 *
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of source status channel number
 *	v_channel_u8 -->	BMA2x2_ACCEL_SOURCE_LOW_G         0
 *				BMA2x2_ACCEL_SOURCE_HIGH_G        1
 *				BMA2x2_ACCEL_SOURCE_SLOPE        2
 *				BMA2x2_ACCEL_SOURCE_SLOW_NO_MOTION   3
 *				BMA2x2_ACCEL_SOURCE_TAP          4
 *				BMA2x2_ACCEL_SOURCE_DATA         5
 *
 *	u8 *v_intr_source_u8: Pointer holding the source status value
 *	v_intr_source_u8
 *	0 --> enable
 *	1 --> disable
 *
 *	\return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_source(u8 v_channel_u8,
u8 *v_intr_source_u8);
/****************************************************************************
 * Description: *//**\brief  This API is used to set
 *	the source v_intr_stat_u8us v_data_u8 in the register 0x1E bit from 0 to 5
 *
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of source status channel number
 *	v_channel_u8 -->	BMA2x2_ACCEL_SOURCE_LOW_G         0
 *				BMA2x2_ACCEL_SOURCE_HIGH_G        1
 *				BMA2x2_ACCEL_SOURCE_SLOPE        2
 *				BMA2x2_ACCEL_SOURCE_SLOW_NO_MOTION   3
 *				BMA2x2_ACCEL_SOURCE_TAP          4
 *				BMA2x2_ACCEL_SOURCE_DATA         5
 *
 *  u8 v_intr_source_u8:
 *	The source status value
 *	v_intr_source_u8
 *	0 --> enable
 *	1 --> disable
 *
 *  \return  results of bus communication
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_source(u8 v_channel_u8,
u8 v_intr_source_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the output type v_intr_stat_u8us in the register 0x20.
 *	INT1 -> bit 1
 *	_INTR2_ -> bit 3
 *
 *	\param u8 v_channel_u8:
 *   The value of output type channel number
 *	v_channel_u8 --> BMA2x2_ACCEL_INTR1_OUTPUT    ->   0
 *				BMA2x2_ACCEL_INTR2_OUTPUT    ->   1
 *
 *	u8 *v_intr_output_type_u8: Pointer holding the output type value
 *	open drain   ->   1
 *	push pull    ->   0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_output_type(u8 v_channel_u8,
u8 *v_intr_output_type_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *	the output type v_intr_stat_u8us in the register 0x20.
 *	INT1 -> bit 1
 *	_INTR2_ -> bit 3
 *
 *	\param u8 v_channel_u8:
 *	The value of output type channel number
 *	v_channel_u8 -->	BMA2x2_ACCEL_INTR1_OUTPUT    ->   0
 *				BMA2x2_ACCEL_INTR2_OUTPUT    ->   1
 *
 *  u8  v_intr_output_type_u8: The output type value
 *	open drain   ->   1
 *	push pull    ->   0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_output_type(u8 v_channel_u8,
u8 v_intr_output_type_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	Active Level status in the register 0x20
 *	INTR1 -> bit 0
 *	INTR2 -> bit 2
 *
 *	\param u8 v_channel_u8:
 *	The value of Active Level channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_LEVEL ->    0
 *					BMA2x2_ACCEL_INTR2_LEVEL ->    1
 *
 *  u8 *v_intr_level_u8: Pointer holding the Active Level status value
 *	Active HIGH   ->   1
 *	Active LOW    ->   0
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_level(u8 v_channel_u8,
u8 *v_intr_level_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	Active Level v_intr_stat_u8us in the register 0x20
 *	INTR1 -> bit 0
 *	INTR2_ -> bit 2
 *
 *
 *
 *	\param u8 v_channel_u8:
 *        The value of Active Level channel number
 *       v_channel_u8 -->BMA2x2_ACCEL_INTR1_LEVEL    ->    0
 *                       BMA2x2_ACCEL_INTR2_LEVEL    ->    1
 *
 *  u8 v_intr_level_u8:
 *	The value of Active Level channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_INTR1_LEVEL ->    0
 *					BMA2x2_ACCEL_INTR2_LEVEL ->    1
 *	Active HIGH   ->   1
 *	Active LOW    ->   0
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_level(u8 v_channel_u8,
u8 v_intr_level_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the reset interrupt in the register 0x21 bit 7
 *
 *
 *
 *  \param u8 v_rst_intr_u8: Reset the interrupt
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_rst_intr(u8 v_rst_intr_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *	\param u8 *v_latch_intr_u8:
 *    Pointer holding the latch duration value
 *      v_latch_intr_u8 -->
 *			BMA2x2_LATCH_DURN_NON_LATCH    0x00
 *			BMA2x2_LATCH_DURN_250MS        0x01
 *			BMA2x2_LATCH_DURN_500MS        0x02
 *			BMA2x2_LATCH_DURN_1S           0x03
 *			BMA2x2_LATCH_DURN_2S           0x04
 *			BMA2x2_LATCH_DURN_4S           0x05
 *			BMA2x2_LATCH_DURN_8S           0x06
 *			BMA2x2_LATCH_DURN_LATCH        0x07
 *			BMA2x2_LATCH_DURN_NON_LATCH1   0x08
 *			BMA2x2_LATCH_DURN_250US        0x09
 *			BMA2x2_LATCH_DURN_500US        0x0A
 *			BMA2x2_LATCH_DURN_1MS          0x0B
 *			BMA2x2_LATCH_DURN_12_5MS       0x0C
 *			BMA2x2_LATCH_DURN_25MS         0x0D
 *			BMA2x2_LATCH_DURN_50MS         0x0E
 *			BMA2x2_LATCH_DURN_LATCH1       0x0F
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_latch_intr(u8 *v_latch_intr_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the latch duration in the register 0x21 bit from 0 to 3
 *
 *
 *
 *	u8 v_latch_intr_u8:
 *	The latch duration value
 *          BMA2x2_LATCH_DURN_NON_LATCH    0x00
 *			BMA2x2_LATCH_DURN_250MS        0x01
 *			BMA2x2_LATCH_DURN_500MS        0x02
 *			BMA2x2_LATCH_DURN_1S           0x03
 *			BMA2x2_LATCH_DURN_2S           0x04
 *			BMA2x2_LATCH_DURN_4S           0x05
 *			BMA2x2_LATCH_DURN_8S           0x06
 *			BMA2x2_LATCH_DURN_LATCH        0x07
 *			BMA2x2_LATCH_DURN_NON_LATCH1   0x08
 *			BMA2x2_LATCH_DURN_250US        0x09
 *			BMA2x2_LATCH_DURN_500US        0x0A
 *			BMA2x2_LATCH_DURN_1MS          0x0B
 *			BMA2x2_LATCH_DURN_12_5MS       0x0C
 *			BMA2x2_LATCH_DURN_25MS         0x0D
 *			BMA2x2_LATCH_DURN_50MS         0x0E
 *			BMA2x2_LATCH_DURN_LATCH1       0x0F
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_latch_intr(u8 v_latch_intr_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_DURN		-> register 0x22 bit form 0 to 7
 *	HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *	\param u8 v_channel_u8:
 *	The value of duration v_channel_u8 number
 *	v_channel_u8 --> BMA2x2_ACCEL_LOW_DURN            0
 *				BMA2x2_ACCEL_HIGH_DURN           1
 *				BMA2x2_ACCEL_SLOPE_DURN          2
 *				BMA2x2_ACCEL_SLOW_NO_MOTION_DURN     3
 *
 *	u8 *v_durn_u8: Pointer holding the duration value
 *
 *
 *
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_durn(u8 v_channel_u8,
u8 *v_durn_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to set the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_DURN		-> register 0x22 bit form 0 to 7
 *	HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *	\param u8 v_channel_u8:
 *	The value of duration v_channel_u8 number
 *	v_channel_u8 --> BMA2x2_ACCEL_LOW_DURN            0
 *				BMA2x2_ACCEL_HIGH_DURN           1
 *				BMA2x2_ACCEL_SLOPE_DURN          2
 *				BMA2x2_ACCEL_SLOW_NO_MOTION_DURN     3
 *
 *
 *	u8 v_durn_u8: duration value
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_durn(u8 v_channel_u8,
u8 v_durn_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_THRES		-> register 0x23 bit form 0 to 7
 *	HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *	\param u8 v_channel_u8:
 *    The value of threshold v_channel_u8 number
 *    v_channel_u8 -->BMA2x2_ACCEL_LOW_THRES            0
 *               BMA2x2_ACCEL_HIGH_THRES           1
 *               BMA2x2_ACCEL_SLOPE_THRES          2
 *               BMA2x2_ACCEL_SLOW_NO_MOTION_THRES     3
 *
 *  u8 *v_thres_u8: Pointer holding the threshold value of threshold
 *
 *
 *
 *  \return  results of bus communication function
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_thres(u8 v_channel_u8,
u8 *v_thres_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	LOW_THRES		-> register 0x23 bit form 0 to 7
 *	HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of threshold v_channel_u8 number
 *	v_channel_u8 -->	BMA2x2_ACCEL_LOW_THRES            0
 *				BMA2x2_ACCEL_HIGH_THRES           1
 *				BMA2x2_ACCEL_SLOPE_THRES          2
 *				BMA2x2_ACCEL_SLOW_NO_MOTION_THRES     3
 *
 *  u8  v_thres_u8: The threshold interrupt status value
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_thres(u8 v_channel_u8,
u8 v_thres_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the low high hysteresis in the registers 0x24
 *	LOW_G_HYST  -> bit form 0 to 1
 *	HIGH_G_HYST  -> bit from 6 to 7(it depends on the v_range_u8 selection)
 *
 * \param u8 v_channel_u8: The value of selection of hysteresis channel number
 *	v_channel_u8 -->BMA2x2_ACCEL_LOW_G_HYST	    0
 *					BMA2x2_ACCEL_HIGH_G_HYST    1
 *
 *  u8 *v_hyst_u8: Pointer holding the hysteresis value
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_low_high_g_hyst(u8 v_channel_u8,
u8 *v_hyst_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the low high hysteresis in the registers 0x24
 *	LOW_G_HYST  -> bit form 0 to 1
 *	HIGH_G_HYST  -> bit from 6 to 7(it depends on the v_range_u8 selection)
 *
 *
 *
 *	\param u8 v_channel_u8: The value of hysteresis v_channel_u8 number
 *	v_channel_u8 -->	BMA2x2_ACCEL_LOW_G_HYST  ->    0
 *				BMA2x2_ACCEL_HIGH_G_HYST ->    1
 *
 *	u8 v_hyst_u8: The hysteresis value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_low_high_g_hyst(u8 v_channel_u8,
u8 v_hyst_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	low_g  v_power_mode_u8 in the registers 0x24 bit 2
 *
 *
 *	\param u8 *v_low_g_mode_u8: Pointer holding the Low_G mode value
 *	v_low_g_mode_u8:
 *	0 -> single v_power_mode_u8
 *	1 -> sum v_power_mode_u8
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
*****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 ******************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_low_g_mode(u8 *v_low_g_mode_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	low_g  v_power_mode_u8 in the registers 0x24 bit 2
 *
 *
 *
 *	\param u8 v_low_g_mode_u8: The Low g  mode value
 *	v_low_g_mode_u8:
 *	0 -> single v_power_mode_u8
 *	1 -> sum v_power_mode_u8
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_low_g_mode(u8 v_low_g_mode_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap duration in the register 0x2A bit form 0 to 2
 *
 *
 *
 *	\param u8 *v_tap_durn_u8: Pointer holding the tap duration value
 *	v_tap_durn_u8 -->  0 -> 50ms
 *               1 -> 100ms
 *               2 -> 150ms
 *               3 -> 200ms
 *               4 -> 250ms
 *               5 -> 375ms
 *               6 -> 500ms
 *               7 -> 700ms
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_durn(u8 *v_tap_durn_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap duration in the register 0x2A bit form 0 to 2
 *
 *
 *
 *	\param u8  v_tap_durn_u8: The tap duration value
 *	v_tap_durn_u8	->	0 -> 50ms
 *				1 -> 100ms
 *				2 -> 150ms
 *				3 -> 200ms
 *				4 -> 250ms
 *				5 -> 375ms
 *				6 -> 500ms
 *				7 -> 700ms
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_durn(u8 v_tap_durn_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap shock form the register 0x2A bit 6
 *
 *
 *
 *	\param u8 *v_tap_shock_u8: Pointer holding the tap shock value
 *	v_tap_shock_u8
 *	1 -> 75ms
 *	0 -> 50ms
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_shock(u8 *v_tap_shock_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap shock form the register 0x2A bit 6
 *
 *
 *
 *	\param u8 u8 v_tap_shock_u8: The tap shock value
 *     v_tap_shock_u8 --> 0 -> 50ms
 *                   1 -> 75ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_shock(u8 v_tap_shock_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap quiet in the register 0x2A bit 7
 *
 *
 *
 *  \param  u8 *v_tap_quiet_u8 :Pointer holding the tap quiet value
 *          v_tap_quiet_u8 ->  0 -> 30ms
 *                        1 -> 20ms
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_quiet(u8 *v_tap_quiet_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap quiet in the register 0x2A bit 7
 *
 *
 *
 *  \param u8 v_tap_quiet_u8 : The tap quiet value
 *            v_tap_quiet_u8 ->    0 -> 30ms
 *                            1 -> 20ms
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_quiet(u8 v_tap_quiet_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap threshold in the register 0x2B bit from 0 to 4
 *
 *
 *
 *  \param u8 *v_tap_thres_u8 : Pointer holding the tap threshold
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_thres(u8 *v_tap_thres_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap thresheshold in the register 0x2B bit from 0 to 4
 *
 *
 *
 *  \param u8 v_tap_thres_u8: The tap thresheshold value
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_thres(u8 v_tap_thres_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the tap sample in the register 0x2B bit 6 and 7
 *
 *
 *
 *  \param u8  *v_tap_sample_u8 : Pointer holding the tap sample
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_sample(u8 *v_tap_sample_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the tap sample in the register 0x2B bit 6 and 7
 *
 *
 *
 *  \param u8 v_tap_sample_u8: The tap sample value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_sample(u8 v_tap_sample_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the orient v_power_mode_u8 in the register 0x2C bit 0 and 1
 *
 *
 *
 *  \param u8 *v_orient_mode_u8 : Pointer holding the orient mode value
 *	v_orient_mode_u8  ->
 *	00 -> 45' symmetrical
 *	01 -> 63' high asymmetrical
 *	10 -> 27' low asymmetrical
 *	11 -> reserved
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_mode(u8 *v_orient_mode_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the orient v_power_mode_u8 in the register 0x2C bit 0 and 1
 *
 *
 *
 *  \param u8 v_orient_mode_u8: The orient mode value
 *	v_orient_mode_u8 ->
 *	00	->	45' symmetrical
 *	01	->	63' high asymmetrical
 *	10	->	27' low asymmetrical
 *	11	->	reserved
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_mode(u8 v_orient_mode_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the orient block in the register 0x2C bit 2 and 3
 *
 *
 *
 *	\param u8 *v_orient_block_u8 : Pointer holding the orient block value
 *	v_orient_block_u8 ->
 *	00 -> disabled
 *	01 -> horizontal position or accel >1.75g
 *	10 -> horizontal position or accel >1.75g or slope > 0.2g
 *	11 -> horizontal position or accel >1.75g or slope > 0.4g or wait 100ms
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_block(
u8 *v_orient_block_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the orient block in the register 0x2C bit 2 and 3
 *
 *
 *
 *  \param u8 v_orient_block_u8: The orient block value
 *       v_orient_block_u8 ->
 *	00 -> disabled
 *	01 -> horizontal position or accel >1.75g
 *	10 -> horizontal position or accel >1.75g or slope > 0.2g
 *	11 -> horizontal position or accel >1.75g or slope > 0.4g or wait 100ms
 *
 *
 *  \return  results of bus communication function
 *
 *
 *************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_block(u8 v_orient_block_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the orient hysteresis in the register 0x2C bit 4 to 6
 *
 *
 *
 *  \param u8 *v_orient_hyst_u8 : Pointer holding the v_orient_hyst_u8
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_hyst(u8 *v_orient_hyst_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the orient hysteresis in the register 0x2C bit 4 to 6
 *
 *
 *
 *  \param u8 v_orient_hyst_u8: The orient hysteresis value
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_hyst(u8 v_orient_hyst_u8);
/***************************************************************************
 *	Description: *//**\brief  This API is used to get
 *	the theta value of orient and flat interrupts
 *	ORIENT_THETA -> register 0x2D bit 0 to 5
 *	FLAT_THETA -> register 0x2E bit 0 to 5
 *
 *	\param u8 v_channel_u8: The value of theta v_channel_u8 number
 *	v_channel_u8 -->	BMA2x2_ACCEL_ORIENT_THETA	0
 *				BMA2x2_ACCEL_FLAT_THETA		1
 *
 *  u8 *v_theta_u8: Pointer holding the v_theta_u8 value
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_theta(u8 v_channel_u8,
u8 *v_theta_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to set
 *	the v_theta_u8 value of orient and flat interrupts
 *	ORIENT_THETA -> register 0x2D bit 0 to 5
 *	FLAT_THETA -> register 0x2E bit 0 to 5
 *
 *	\param u8 v_channel_u8: The value of v_theta_u8 v_channel_u8 number
 *	v_channel_u8 -->	BMA2x2_ACCEL_ORIENT_THETA	0
 *				BMA2x2_ACCEL_FLAT_THETA		1
 *
 *	u8 v_theta_u8: The v_theta_u8 value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_theta(u8 v_channel_u8,
u8 v_theta_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *  the v_intr_stat_u8us of "orient_ud_en" enable in the register 0x2D bit 6
 *
 *
 *
 *
 *  \param u8 *v_orient_enable_u8 : Pointer holding the v_orient_enable_u8
 *	v_orient_enable_u8 ->	1 -> Generates Interrupt
 *					0 -> Do not generate interrupt
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_enable(
u8 *v_orient_enableable_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *  the "orient_ud_enable" enable in the register 0x2D bit 6
 *
 *
 *
 *
 *  \param u8 v_orient_enableable_u8: The orient enable value
 *	v_orient_enableable_u8 ->
 *	1	->	Generates Interrupt
 *	0	->	Do not generate interrupt
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_enable(u8 v_orient_enableable_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the v_intr_stat_u8us of flat hysteresis("flat_hy) in the register 0x2F bit 0 to 2
 *
 *
 *
 *
 *  \param u8 *v_flat_hyst_u8 : Pointer holding the flat hyst
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_flat_hyst(u8 *v_flat_hyst_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value flat hysteresis("flat_hy) in the register 0x2F bit 0 to 2
 *
 *
 *
 *  \param u8 v_flat_hyst_u8: The flat hysteresis value
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_flat_hyst(u8 v_flat_hyst_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *  the v_intr_stat_u8us of flat hold time(flat_hold_time)
 *	in the register 0x2F bit 4 and 5
 *
 *
 *
 *
 *  \param  u8 *v_flat_hold_time_u8 : Pointer holding the flat hold time
 *             v_flat_hold_time_u8 ->  00 -> disabled
 *                                01 -> 512ms
 *                                10 -> 1024ms
 *                                11 -> 2048ms
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_flat_hold_time(
u8 *v_flat_hold_time_u8);
/**************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of flat hold time(v_flat_hold_time_u8)
 *	in the register 0x2F bit 4 and 5
 *
 *
 *
 *
 *  \param u8 v_flat_hold_time_u8: The flat hold time value
 *              v_flat_hold_time_u8 -> 00 -> disabled
 *                                01 -> 512ms
 *                                10 -> 1024ms
 *                                11 -> 2048ms
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_flat_hold_time(
u8 v_flat_hold_time_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo water mark level trigger(fifo_water_mark_level_trigger_retain)
 *	v_intr_stat_u8us in the register 0x30 bit from 0 to 5
 *
 *
 *
 *
 *  \param u8 *fifo_wml_trig: Pointer holding the fifo_wml_trig
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_wml_trig(
u8 *fifo_wml_trig);
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the fifo water mark level trigger(fifo_water_mark_level_trigger_retain)
 *	value in the register 0x30 bit from 0 to 5
 *
 *
 *
 *
 *  \param u8 fifo_wml_trig: The fifo water mark level trigger value
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_wml_trig(
u8 fifo_wml_trig);
/****************************************************************************
 *	Description: *//**\brief This API is for to get
 *	the self test axis(self_test_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  \param u8 *v_selftest_axis_u8 : Pointer holding the v_selftest_axis_u8
 *
 *	0x00	-> self test disabled
 *	0x01	-> x-axis
 *	0x02	-> y-axis
 *	0x03	-> z-axis
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_selftest_axis(
u8 *v_selftest_axis_u8);
/***************************************************************************
 *	Description: *//**\brief This API is for to Set the value of
 *	the self test axis(selftest_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  \param u8 v_selftest_axis_u8: The self test axis value
 *
 *	0x00	-> self test disabled
 *	0x01	-> x-axis
 *	0x02	-> y-axis
 *	0x03	-> z-axis
 *
 *
 *
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_selftest_axis(
u8 v_selftest_axis_u8);
/***************************************************************************
 *	Description: *//**\brief This API is for to get
 *	the Self Test sign(selftest_sign) in the register 0x32 bit 2
 *
 *
 *
 *  \param u8 *selftest_sign : Pointer holding the selftest sign
 *	v_selftest_sign_u8 ->
 *	0 -> negative sign
 *	1 -> positive sign
 *
 *
 *
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_selftest_sign(
u8 *v_selftest_sign_u8);
/****************************************************************************
 *	Description: *//**\brief This API is for to Set the value of
 *	the self test axis(selftest_axis) in the register ox32 bit 0 to 2
 *
 *
 *
 *  \param u8 v_selftest_sign_u8: The self test sign value
 *
 *	0x00	-> self test disabled
 *	0x01	-> x-axis
 *	0x02	-> y-axis
 *	0x03	-> z-axis
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_selftest_sign(
u8 v_selftest_sign_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of nvm program mode(nvm_prog_mode)in the register 0x33 bit 0
 *
 *
 *
 *
 *  \param  u8 *v_nvmprog_mode_u8 : Pointer holding the nvmprog_mode
 *	v_nvmprog_mode_u8
 *	1 -> Enable program v_power_mode_u8
 *	0 -> Disable program v_power_mode_u8
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_mode(
u8 *v_nvmprog_mode_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set
 * the value of nvm program mode(nvm_prog_mode) in the register 0x33 bit 0
 *
 *
 *
 *  \param u8 v_nvmprog_mode_u8 : The nvw program mode value
 *                v_nvmprog_mode_u8 ->   1 -> Enable program mode
 *                             0 -> Disable program mode
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvmprog_mode(u8 v_nvmprog_mode_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of nvm program trig(nvm_prog_trig) in the register 0x33 bit 1
 *
 *
 *
 *
 *	\param u8 v_nvprog_trig_u8: The nvm program trig value
 *	v_nvprog_trig_u8
 *	1 -> trig program seq (wo)
 *	0 -> No Action
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvprog_trig(u8 v_nvprog_trig_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of nvmprogram ready(nvm_rdy) in the register bit 2
 *
 *
 *
 *
 *  \param u8 *v_nvprog_ready_u8: The pointer holding the nvmprogram ready value
 *	v_nvprog_ready_u8
 *	1 -> program seq finished
 *	0 -> program seq in progress
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_ready(u8 *v_nvprog_ready_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *  the v_intr_stat_u8us of nvm program remain(nvm_remain) in the register 0x33
 *	bit 4 to 7
 *
 *
 *
 *  \param u8 *v_nvprog_remain_u8:
 *        The pointer holding the nvm program remain value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_remain(u8 *v_nvprog_remain_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get the staus of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  \param  u8 *v_spi3_u8 : Pointer holding the v_spi3_u8 value
 *          v_spi3_u8 ->    0 -> v_spi3_u8
 *                     1 -> spi4(default)
 *
 *
 *
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_spi3(u8 *v_spi3_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to set the v_intr_stat_u8us of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  \param u8 v_spi3_u8: The spi3 value
 *        v_spi3_u8 -> 0 -> spi3
 *                1 -> spi4(default)
 *
 *
 *  \return  results of bus communication function
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
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_spi3(u8 v_spi3_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get the v_intr_stat_u8us of i2c
 *	wdt selection(i2c_wdt_sel) and wdt enable(i2c_wdt_enable) in the register
 *	0x36 bit 1 and 2
 *
 *
 *	\param u8 v_channel_u8: The value of i2c wdt v_channel_u8 number
 *	v_channel_u8 --> BMA2x2_ACCEL_I2C_SELECT          0
 *				BMA2x2_ACCEL_I2C_EN              1
 *
 *  u8 *v_i2c_wdt_u8: Pointer holding the mode of i2c value
 *       v_i2c_wdt_u8 --> x,0 ->OFF
 *                     0,1 ->1 ms
 *                     1,1 ->50ms
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_i2c_wdt(u8 v_channel_u8,
u8 *v_i2c_wdt_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set the value of i2c
 *	wdt selection(i2c_wdt_sel) and wdt enable(i2c_wdt_enable) in the register
 *	0x36 bit 1 and 2
 *
 *
 *	\param u8 v_channel_u8: The value of i2c wdt channel number
 *	v_channel_u8 --> BMA2x2_ACCEL_I2C_SELECT          0
 *				BMA2x2_ACCEL_I2C_EN              1
 *
 *  u8  v_i2c_wdt_u8: The vale of i2c watch dog timer
 *	v_i2c_wdt_u8 ->
 *	x,0 ->OFF
 *	0,1 ->1 ms
 *	1,1 ->50ms
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 **************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_i2c_wdt(u8 v_channel_u8,
u8 v_i2c_wdt_u8);
/**************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the v_intr_stat_u8us slow compensation(hp_x_enable, hp_y_enable and hp_z_enable)
 *	in the register 0x36 bit 0 to 2
 *	SLOW_COMP_X -> bit 0
 *	SLOW_COMP_Y -> bit 1
 *	SLOW_COMP_Z -> bit 2
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of slow compensation v_channel_u8 number
 *	v_channel_u8 --> BMA2x2_ACCEL_SLOW_COMP_X              0
 *				BMA2x2_ACCEL_SLOW_COMP_Y              1
 *				BMA2x2_ACCEL_SLOW_COMP_Z              2
 *
 *  u8 *v_slow_comp_u8: Pointer holding the slow compensation value
 *	v_slow_comp_u8 : 1 -> enable
 *				0 -> disable slow offset
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_slow_comp(u8 v_channel_u8,
u8 *v_slow_comp_u8);
/*************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the v_intr_stat_u8us slow compensation(hp_x_enable, hp_y_enable and hp_z_enable)
 *	in the register 0x36 bit 0 to 2
 *	SLOW_COMP_X -> bit 0
 *	SLOW_COMP_Y -> bit 1
 *	SLOW_COMP_Z -> bit 2
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of slow compensation v_channel_u8 number
 *	v_channel_u8 --> BMA2x2_ACCEL_SLOW_COMP_X              0
 *				BMA2x2_ACCEL_SLOW_COMP_Y              1
 *				BMA2x2_ACCEL_SLOW_COMP_Z              2
 *
 *
 *	u8 v_slow_comp_u8: The slow compensation value
 *	v_slow_comp_u8 : 1 -> enable
 *				0 -> disable slow offset
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_slow_comp(u8 v_channel_u8,
u8 v_slow_comp_u8);
/***********************************************************************
 *	Description: *//**\brief This API is used to get
 *	the v_intr_stat_u8us of fast offset compensation(cal_rdy) in the register 0x36
 *	bit 4(Read Only Possible)
 *
 *
 *
 *  \param u8 v_cal_rdy_u8: Pointer holding the cal rdy
 *
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_cal_rdy(u8 *v_cal_rdy_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the v_intr_stat_u8us of fast offset calculation(cal_trig) in the register 0x36
 *	bit 6(Write only possible)
 *
 *
 *
 *  \param u8 v_cal_trigger_u8: The value of call trig
 *
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_cal_trigger(u8 v_cal_trigger_u8);
/*********************************************************************
 *	Description: *//**\brief This API is used to set
 *	the v_intr_stat_u8us of offset reset(offset_reset) in the register 0x36
 *	bit 7(Write only possible)
 *
 *
 *
 *  \param u8 v_offset_rst_u8: The offset reset value
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 **************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset_rst(u8 v_offset_rst_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the v_intr_stat_u8us of offset target axis(offset_target_x, offset_target_y and
 *	offset_target_z) and cut_off in the register 0x37
 *	CUT_OFF -> bit 0
 *	OFFSET_TRIGGER_X -> bit 1 and 2
 *	OFFSET_TRIGGER_Y -> bit 3 and 4
 *	OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *
 *	\param u8 v_channel_u8:
 *	The value of offset axis selection v_channel_u8 number
 *	v_channel_u8 --> BMA2x2_ACCEL_CUT_OFF              ->	0
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_X     ->    1
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y     ->    2
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z     ->    3
 *
 * u8 v_offset_u8: The v_offset_u8 target value
 *	CUT_OFF -> 0 or 1
 *	v_offset_u8 -->	BMA2x2_ACCEL_OFFSET_TRIGGER_X	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z	-> 0,1,2,3
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_offset_target(u8 v_channel_u8,
u8 *v_offset_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of offset target axis(offset_target_x, offset_target_y and
 *	offset_target_z) and cut_off in the register 0x37
 *	CUT_OFF -> bit 0
 *	OFFSET_TRIGGER_X -> bit 1 and 2
 *	OFFSET_TRIGGER_Y -> bit 3 and 4
 *	OFFSET_TRIGGER_Z -> bit 5 and 6
 *
 *
 *	\param u8 offset:
 *	The value of v_offset_u8 axis selection offset number
 *	v_channel_u8 --> BMA2x2_ACCEL_CUT_OFF              ->	0
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_X     ->    1
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y     ->    2
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z     ->    3
 *
 * u8 v_offset_u8: The v_offset_u8 target value
 *	CUT_OFF -> 0 or 1
 *	v_offset_u8 -->	BMA2x2_ACCEL_OFFSET_TRIGGER_X	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Y	-> 0,1,2,3
 *				BMA2x2_ACCEL_OFFSET_TRIGGER_Z	-> 0,1,2,3
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset_target(u8 v_channel_u8,
u8 v_offset_u8);
/**************************************************************************
 *	Description: *//**\brief This API is used to get the v_intr_stat_u8us of v_offset_u8
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	offset_x -> register 0x38 bit 0 to 7
 *	offset_y -> register 0x39 bit 0 to 7
 *	offset_z -> register 0x3A bit 0 to 7
 *
 *
 *	\param u8 v_channel_u8: The value of v_offset_u8 channel number
 *	v_channel_u8 ->
 *		BMA2x2_ACCEL_X_AXIS     ->      0
 *		BMA2x2_ACCEL_Y_AXIS     ->      1
 *		BMA2x2_ACCEL_Z_AXIS     ->      2
 *
 *  u8 *v_offset_u8: Pointer holding the v_offset_u8 value
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_offset(u8 v_channel_u8,
s8 *v_offset_u8);
/***************************************************************************
 * Description: *//**\brief This API is used to set the value of v_offset_u8
 *	(offset_x, offset_y and offset_z) in the registers 0x38,0x39 and 0x3A
 *	offset_x -> register 0x38 bit 0 to 7
 *	offset_y -> register 0x39 bit 0 to 7
 *	offset_z -> register 0x3A bit 0 to 7
 *
 *
 *	\param u8 v_channel_u8: The value of offset channel number
 *	v_channel_u8 ->
 *		BMA2x2_ACCEL_X_AXIS     ->      0
 *		BMA2x2_ACCEL_Y_AXIS     ->      1
 *		BMA2x2_ACCEL_Z_AXIS     ->      2
 *
 *	u8 v_offset_u8: The v_offset_u8 value
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset(u8 v_channel_u8,
s8 v_offset_u8);
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the v_intr_stat_u8us of fifo v_power_mode_u8(fifo_mode) in the register 0x3E bit 6 and 7
 *
 *
 *
 *
 *  \param u8 *v_fifo_mode_u8 : Pointer holding the fifo mode
 *	v_fifo_mode_u8	0 --> Bypass
 *				1 --> FIFO
 *				2 --> Stream
 *				3 --> Reserved
 *
 *
 *
 *  \return  results of bus communication function
 *
 *
 ****************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_mode(u8 *v_fifo_mode_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used set to FIFO mode
 *	the value of fifo (fifo mode) in the register 0x3E bit 6 and 7
 *
 *
 *
 *  \param u8 v_fifo_mode_u8: The fifo mode value
 *	v_fifo_mode_u8	0 --> Bypass
 *				1 --> FIFO
 *				2 --> Stream
 *				3 --> Reserved
 *
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_mode(u8 v_fifo_mode_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of fifo v_data_u8 select in the register 0x3E bit 0 and 1
 *
 *
 *
 *
 *  \param u8 *v_fifo_data_select_u8 : Pointer holding the fifo data select
 *         v_data_u8_select --> [0:3]
 *         0 --> X,Y and Z (DEFAULT)
 *         1 --> Y only
 *         2 --> X only
 *         3 --> Z only
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_data_select(
u8 *v_fifo_data_select_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the value of fifo v_data_u8 select in the register 0x3E bit 0 and 1
 *
 *
 *
 *  \param u8 v_fifo_data_select_u8: The fifo data select value
 *         v_fifo_data_select_u8 --> [0:3]
 *         0 --> X,Y and Z (DEFAULT)
 *         1 --> Y only
 *         2 --> X only
 *         3 --> Z only
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_data_select(
u8 v_fifo_data_select_u8);
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo v_data_u8(fifo_v_data_u8_output_register) in the register 0x3F
 *
 *
 *
 *
 *
 *  \param  u8 *v_output_reg_u8 : Pointer holding the out reg
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_data_output_reg(
u8 *v_output_reg_u8);
/****************************************************************************
 * Description: *//**\brief This API is used to read the temp
 *
 *
 *
 *
 *  \param s8 *v_temp_s8:
 *                 Pointer holding the v_temp_s8 value
 *
 *
 *
 *  \return  results of bus communication function
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
 ****************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_temp(s8 *v_temp_s8);
/****************************************************************************
 * Description: *//**\brief This API reads accelerometer data X,Y,Z values and
 * temperature data from location 02h to 08h
 *
 *
 *
 *
 *  \param bma2x2_accel_data_temp * accel : Pointer holding the accel data
 *	and temperature
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_xyzt(
struct bma2x2_accel_data_temp *accel);
/****************************************************************************
 * Description: *//**\brief This API reads acceleration of 8 bit resolution
 *                          v_data_u8 of X,Y,Z values
 *                          from location 03h , 05h and 07h
 *
 *
 *
 *
 *  \param bma2x2accel_t * accel : pointer holding the data of accel
 *
 *
 *
 *  \return  results of bus communication function
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
 ***************************************************************************/
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_eight_resolution_xyzt(
struct bma2x2_accel_eight_resolution_temp *accel);
#endif
