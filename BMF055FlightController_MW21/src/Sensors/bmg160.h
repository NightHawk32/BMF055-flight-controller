/*
****************************************************************************
* Copyright (C) 2010 - 2014 Bosch Sensortec GmbH
*
* bmg160.h
* Date: 2014/10/17
* Revision: 2.0.2 $
*
* Usage: Sensor Driver for BMG160 sensor
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
/*****************************************************************************/
/*! \file BMG160.h
    \brief Header for BMG160 API */
/* user defined code to be added here ... */
#ifndef __BMG160_H__
#define __BMG160_H__

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

/**\brief defines the calling parameter types of the BMG160_WR_FUNCTION */
#define BMG160_BUS_WR_RETURN_TYPE s8

/**\brief links the order of parameters defined in
BMG160_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define BMG160_BUS_WR_PARAM_TYPES u8, u8,\
u8 *, u8

/**\brief links the order of parameters defined in
BMG160_BUS_WR_PARAM_TYPE to function calls used inside the API*/
#define BMG160_BUS_WR_PARAM_ORDER(device_addr, register_addr,\
register_data, wr_len)

/* never change this line */
#define BMG160_BUS_WRITE_FUNC(device_addr, register_addr,\
register_data, wr_len) bus_write(device_addr, register_addr,\
register_data, wr_len)
/**\brief defines the return parameter type of the BMG160_RD_FUNCTION
*/
#define BMG160_BUS_RD_RETURN_TYPE s8
/**\brief defines the calling parameter types of the BMG160_RD_FUNCTION
*/
#define BMG160_BUS_RD_PARAM_TYPES u8, u8,\
u8 *, u8
/**\brief links the order of parameters defined in \
BMG160_BUS_RD_PARAM_TYPE to function calls used inside the API
*/
#define BMG160_BUS_RD_PARAM_ORDER (device_addr, register_addr,\
register_data)
/* never change this line */
#define BMG160_BUS_READ_FUNC(device_addr, register_addr,\
register_data, rd_len)bus_read(device_addr, register_addr,\
register_data, rd_len)
/**\brief defines the return parameter type of the BMG160_RD_FUNCTION
*/
#define BMG160_BURST_RD_RETURN_TYPE s8
/**\brief defines the calling parameter types of the BMG160_RD_FUNCTION
*/
#define BMG160_BURST_RD_PARAM_TYPES u8,\
u8, u8 *, s32
/**\brief links the order of parameters defined in \
BMG160_BURST_RD_PARAM_TYPE to function calls used inside the API
*/
#define BMG160_BURST_RD_PARAM_ORDER (device_addr, register_addr,\
register_data)
/* never change this line */
#define BMG160_BURST_READ_FUNC(device_addr, register_addr,\
register_data, rd_len)burst_read(device_addr, \
register_addr, register_data, rd_len)
/**\brief defines the return parameter type of the BMG160_DELAY_FUNCTION
*/
#define BMG160_DELAY_RETURN_TYPE void
/* never change this line */
#define BMG160_DELAY_FUNC(delay_in_msec)\
		delay_func(delay_in_msec)

#define BMG160_RETURN_FUNCTION_TYPE			s8
/**< This refers BMG160 return type as signed */

#define	BMG160_I2C_ADDR1				0x68
#define	BMG160_I2C_ADDR2				0x69



/*Define of registers*/

/* Hard Wired */
#define BMG160_CHIP_ID_ADDR						 0x00
/**<Address of Chip ID Register*/

/* Data Register */
#define BMG160_RATE_X_LSB_ADDR                    0x02
/**<        Address of X axis Rate LSB Register       */
#define BMG160_RATE_X_MSB_ADDR                    0x03
/**<        Address of X axis Rate MSB Register       */
#define BMG160_RATE_Y_LSB_ADDR                    0x04
/**<        Address of Y axis Rate LSB Register       */
#define BMG160_RATE_Y_MSB_ADDR                     0x05
/**<        Address of Y axis Rate MSB Register       */
#define BMG160_RATE_Z_LSB_ADDR                     0x06
/**<        Address of Z axis Rate LSB Register       */
#define BMG160_RATE_Z_MSB_ADDR                     0x07
/**<        Address of Z axis Rate MSB Register       */
#define BMG160_TEMP_ADDR                           0x08
/**<        Address of Temperature Data LSB Register  */

/* Status Register */
#define BMG160_INTR_STAT0_ADDR                     0x09
/**<        Address of Interrupt status Register 0    */
#define BMG160_INTR_STAT1_ADDR                     0x0A
/**<        Address of Interrupt status Register 1    */
#define BMG160_INTR_STAT2_ADDR                     0x0B
/**<        Address of Interrupt status Register 2    */
#define BMG160_INTR_STAT3_ADDR                     0x0C
/**<        Address of Interrupt status Register 3    */
#define BMG160_FIFO_STAT_ADDR                      0x0E
/**<        Address of FIFO status Register           */

/* Control Register */
#define BMG160_RANGE_ADDR                         0x0F
/**<        Address of Range address Register     */
#define BMG160_BW_ADDR                            0x10
/**<        Address of Bandwidth Register         */
#define BMG160_MODE_LPM1_ADDR                     0x11
/**<        Address of Mode LPM1 Register         */
#define BMG160_MODE_LPM2_ADDR                     0x12
/**<        Address of Mode LPM2 Register         */
#define BMG160_HIGH_BW_ADDR                       0x13
/**<        Address of Rate HIGH_BW Register       */
#define BMG160_BGW_SOFT_RST_ADDR                  0x14
/**<        Address of BGW Softreset Register      */
#define BMG160_INTR_ENABLE0_ADDR                  0x15
/**<        Address of Interrupt Enable 0             */
#define BMG160_INTR_ENABLE1_ADDR                  0x16
/**<        Address of Interrupt Enable 1             */
#define BMG160_INTR_MAP_ZERO_ADDR                 0x17
/**<        Address of Interrupt MAP 0                */
#define BMG160_INTR_MAP_ONE_ADDR                  0x18
/**<        Address of Interrupt MAP 1                */
#define BMG160_INTR_MAP_TWO_ADDR                  0x19
/**<        Address of Interrupt MAP 2                */
#define BMG160_INTR_ZERO_ADDR                     0x1A
/**<        Address of Interrupt 0 register   */
#define BMG160_INTR_ONE_ADDR                      0x1B
/**<        Address of Interrupt 1 register   */
#define BMG160_INTR_TWO_ADDR                      0x1C
/**<        Address of Interrupt 2 register   */
#define BMG160_INTR_4_ADDR                        0x1E
/**<        Address of Interrupt 4 register   */
#define BMG160_RST_LATCH_ADDR                     0x21
/**<        Address of Reset Latch Register           */
#define BMG160_HIGHRATE_THRES_X_ADDR              0x22
/**<        Address of High Th x Address register     */
#define BMG160_HIGHRATE_DURN_X_ADDR               0x23
/**<        Address of High Dur x Address register    */
#define BMG160_HIGHRATE_THRES_Y_ADDR              0x24
/**<        Address of High Th y  Address register    */
#define BMG160_HIGHRATE_DURN_Y_ADDR               0x25
/**<        Address of High Dur y Address register    */
#define BMG160_HIGHRATE_THRES_Z_ADDR              0x26
/**<        Address of High Th z Address register  */
#define BMG160_HIGHRATE_DURN_Z_ADDR               0x27
/**<        Address of High Dur z Address register  */
#define BMG160_SOC_ADDR                           0x31
/**<        Address of SOC register        */
#define BMG160_A_FOC_ADDR                         0x32
/**<        Address of A_FOC Register        */
#define BMG160_TRIM_NVM_CTRL_ADDR                 0x33
/**<        Address of Trim NVM control register    */
#define BMG160_BGW_SPI3_WDT_ADDR                  0x34
/**<        Address of BGW SPI3,WDT Register           */


/* Trim Register */
#define BMG160_OFC1_ADDR                   0x36
/**<        Address of OFC1 Register          */
#define BMG160_OFC2_ADDR                   0x37
/**<        Address of OFC2 Register          */
#define BMG160_OFC3_ADDR                   0x38
/**<        Address of OFC3 Register          */
#define BMG160_OFC4_ADDR                   0x39
/**<        Address of OFC4 Register          */
#define BMG160_TRIM_GP0_ADDR               0x3A
/**<        Address of Trim GP0 Register              */
#define BMG160_TRIM_GP1_ADDR               0x3B
/**<        Address of Trim GP1 Register              */
#define BMG160_SELECTF_TEST_ADDR            0x3C
/**<        Address of BGW Self test Register           */

/* Control Register */
#define BMG160_FIFO_CGF1_ADDR              0x3D
/**<        Address of FIFO CGF0 Register             */
#define BMG160_FIFO_CGF0_ADDR              0x3E
/**<        Address of FIFO CGF1 Register             */

/* Data Register */
#define BMG160_FIFO_DATA_ADDR              0x3F
/**<        Address of FIFO Data Register             */



/* Rate X LSB Register */
#define BMG160_RATE_X_LSB_VALUEX__POS        0
/**< Last 8 bits of RateX LSB Registers */
#define BMG160_RATE_X_LSB_VALUEX__LEN        8
#define BMG160_RATE_X_LSB_VALUEX__MSK        0xFF
#define BMG160_RATE_X_LSB_VALUEX__REG        BMG160_RATE_X_LSB_ADDR

/* Rate Y LSB Register */
/**<  Last 8 bits of RateY LSB Registers */
#define BMG160_RATE_Y_LSB_VALUEY__POS        0
#define BMG160_RATE_Y_LSB_VALUEY__LEN        8
#define BMG160_RATE_Y_LSB_VALUEY__MSK        0xFF
#define BMG160_RATE_Y_LSB_VALUEY__REG        BMG160_RATE_Y_LSB_ADDR

/* Rate Z LSB Register */
/**< Last 8 bits of RateZ LSB Registers */
#define BMG160_RATE_Z_LSB_VALUEZ__POS        0
#define BMG160_RATE_Z_LSB_VALUEZ__LEN        8
#define BMG160_RATE_Z_LSB_VALUEZ__MSK        0xFF
#define BMG160_RATE_Z_LSB_VALUEZ__REG        BMG160_RATE_Z_LSB_ADDR

/* Interrupt status 0 Register */
   /**< 2th bit of Interrupt status 0 register */
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__POS     2
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__LEN     1
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__MSK     0x04
#define BMG160_INTR_STAT0_ANY_MOTION_INTR__REG     BMG160_INTR_STAT0_ADDR

/**< 1st bit of Interrupt status 0 register */
#define BMG160_INTR_STAT0_HIGHRATE_INTR__POS    1
#define BMG160_INTR_STAT0_HIGHRATE_INTR__LEN    1
#define BMG160_INTR_STAT0_HIGHRATE_INTR__MSK    0x02
#define BMG160_INTR_STAT0_HIGHRATE_INTR__REG    BMG160_INTR_STAT0_ADDR

 /**< 1st and 2nd bit of Interrupt status 0 register */
#define BMG160_INTR_STAT_ZERO__POS    1
#define BMG160_INTR_STAT_ZERO__LEN    2
#define BMG160_INTR_STAT_ZERO__MSK    0x06
#define BMG160_INTR_STAT_ZERO__REG    BMG160_INTR_STAT0_ADDR

/* Interrupt status 1 Register */
/**< 7th bit of Interrupt status 1 register */
#define BMG160_INTR_STAT1_DATA_INTR__POS           7
#define BMG160_INTR_STAT1_DATA_INTR__LEN           1
#define BMG160_INTR_STAT1_DATA_INTR__MSK           0x80
#define BMG160_INTR_STAT1_DATA_INTR__REG           BMG160_INTR_STAT1_ADDR

 /**< 6th bit of Interrupt status 1 register */
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__POS    6
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__LEN    1
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__MSK    0x40
#define BMG160_INTR_STAT1_AUTO_OFFSET_INTR__REG    BMG160_INTR_STAT1_ADDR

/**< 5th bit of Interrupt status 1 register */
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__POS    5
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__LEN    1
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__MSK    0x20
#define BMG160_INTR_STAT1_FAST_OFFSET_INTR__REG    BMG160_INTR_STAT1_ADDR

/**< 4th bit of Interrupt status 1 register */
#define BMG160_INTR_STAT1_FIFO_INTR__POS           4
#define BMG160_INTR_STAT1_FIFO_INTR__LEN           1
#define BMG160_INTR_STAT1_FIFO_INTR__MSK           0x10
#define BMG160_INTR_STAT1_FIFO_INTR__REG           BMG160_INTR_STAT1_ADDR

/**< MSB 4 bits of Interrupt status1 register */
#define BMG160_INTR_STAT_ONE__POS           4
#define BMG160_INTR_STAT_ONE__LEN           4
#define BMG160_INTR_STAT_ONE__MSK           0xF0
#define BMG160_INTR_STAT_ONE__REG           BMG160_INTR_STAT1_ADDR

/* Interrupt status 2 Register */
/**< 3th bit of Interrupt status 2 register */
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__POS     3
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__LEN     1
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__MSK     0x08
#define BMG160_INTR_STAT2_ANY_MOTION_SIGN_INTR__REG     BMG160_INTR_STAT2_ADDR

/**< 2th bit of Interrupt status 2 register */
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__POS   2
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__LEN   1
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__MSK   0x04
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTZ_INTR__REG   BMG160_INTR_STAT2_ADDR

/**< 1st bit of Interrupt status 2 register */
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__POS   1
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__LEN   1
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__MSK   0x02
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTY_INTR__REG   BMG160_INTR_STAT2_ADDR

/**< 0th bit of Interrupt status 2 register */
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__POS   0
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__LEN   1
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__MSK   0x01
#define BMG160_INTR_STAT2_ANY_MOTION_FIRSTX_INTR__REG   BMG160_INTR_STAT2_ADDR

/**< 4 bits of Interrupt status 2 register */
#define BMG160_INTR_STAT_TWO__POS   0
#define BMG160_INTR_STAT_TWO__LEN   4
#define BMG160_INTR_STAT_TWO__MSK   0x0F
#define BMG160_INTR_STAT_TWO__REG   BMG160_INTR_STAT2_ADDR

/* Interrupt status 3 Register */
/**< 3th bit of Interrupt status 3 register */
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__POS     3
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__LEN     1
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__MSK     0x08
#define BMG160_INTR_STAT3_HIGHRATE_SIGN_INTR__REG     BMG160_INTR_STAT3_ADDR

/**< 2th bit of Interrupt status 3 register */
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__POS   2
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__LEN   1
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__MSK   0x04
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTZ_INTR__REG  BMG160_INTR_STAT3_ADDR

/**< 1st bit of Interrupt status 3 register */
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__POS   1
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__LEN   1
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__MSK   0x02
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTY_INTR__REG   BMG160_INTR_STAT3_ADDR

/**< 0th bit of Interrupt status 3 register */
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__POS   0
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__LEN   1
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__MSK   0x01
#define BMG160_INTR_STAT3_HIGHRATE_FIRSTX_INTR__REG   BMG160_INTR_STAT3_ADDR

/**< LSB 4 bits of Interrupt status 3 register */
#define BMG160_INTR_STAT_THREE__POS   0
#define BMG160_INTR_STAT_THREE__LEN   4
#define BMG160_INTR_STAT_THREE__MSK   0x0F
#define BMG160_INTR_STAT_THREE__REG   BMG160_INTR_STAT3_ADDR

/* BMG160 FIFO Status Register */
/**< 7th bit of FIFO status Register */
#define BMG160_FIFO_STAT_OVERRUN__POS         7
#define BMG160_FIFO_STAT_OVERRUN__LEN         1
#define BMG160_FIFO_STAT_OVERRUN__MSK         0x80
#define BMG160_FIFO_STAT_OVERRUN__REG         BMG160_FIFO_STAT_ADDR

/**< First 7 bits of FIFO status Register */
#define BMG160_FIFO_STAT_FRAME_COUNTER__POS   0
#define BMG160_FIFO_STAT_FRAME_COUNTER__LEN   7
#define BMG160_FIFO_STAT_FRAME_COUNTER__MSK   0x7F
#define BMG160_FIFO_STAT_FRAME_COUNTER__REG   BMG160_FIFO_STAT_ADDR

/**< First 3 bits of range Registers */
#define BMG160_RANGE_ADDR_RANGE__POS           0
#define BMG160_RANGE_ADDR_RANGE__LEN           3
#define BMG160_RANGE_ADDR_RANGE__MSK           0x07
#define BMG160_RANGE_ADDR_RANGE__REG           BMG160_RANGE_ADDR

/**< First 3 bits of Bandwidth Registers */
#define BMG160_BW_ADDR__POS             0
#define BMG160_BW_ADDR__LEN             3
#define BMG160_BW_ADDR__MSK             0x07
#define BMG160_BW_ADDR__REG             BMG160_BW_ADDR

/**< 5th and 7th bit of LPM1 Register */
#define BMG160_MODE_LPM1__POS             5
#define BMG160_MODE_LPM1__LEN             3
#define BMG160_MODE_LPM1__MSK             0xA0
#define BMG160_MODE_LPM1__REG             BMG160_MODE_LPM1_ADDR

/**< 1st to 3rd bit of LPM1 Register */
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__POS              1
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__LEN              3
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__MSK              0x0E
#define BMG160_MODELPM1_ADDR_SLEEP_DURN__REG              \
BMG160_MODE_LPM1_ADDR

/**< 7th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__POS         7
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__LEN         1
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__MSK         0x80
#define BMG160_MODE_LPM2_ADDR_FAST_POWERUP__REG         \
BMG160_MODE_LPM2_ADDR

/**< 6th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__POS      6
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__LEN      1
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__MSK      0x40
#define BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING__REG      \
BMG160_MODE_LPM2_ADDR

/**< 4th & 5th bit of Mode LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__POS          4
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__LEN          2
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__MSK          0x30
#define BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__REG          \
BMG160_MODE_LPM2_ADDR

/**< 0th to 2nd bit of LPM2 Register */
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__POS  0
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__LEN  3
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__MSK  0x07
#define BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG  BMG160_MODE_LPM2_ADDR

/**< 7th bit of HIGH_BW Register */
#define BMG160_HIGH_BW__POS         7
#define BMG160_HIGH_BW__LEN         1
#define BMG160_HIGH_BW__MSK         0x80
#define BMG160_HIGH_BW__REG         BMG160_HIGH_BW_ADDR

/**< 6th bit of HIGH_BW Register */
#define BMG160_SHADOW_DIS__POS          6
#define BMG160_SHADOW_DIS__LEN          1
#define BMG160_SHADOW_DIS__MSK          0x40
#define BMG160_SHADOW_DIS__REG          BMG160_HIGH_BW_ADDR

/**< 7th bit of Interrupt Enable 0 Registers */
#define BMG160_INTR_ENABLE0_DATA__POS               7
#define BMG160_INTR_ENABLE0_DATA__LEN               1
#define BMG160_INTR_ENABLE0_DATA__MSK               0x80
#define BMG160_INTR_ENABLE0_DATA__REG               BMG160_INTR_ENABLE0_ADDR

/**< 6th bit of Interrupt Enable 0 Registers */
#define BMG160_INTR_ENABLE0_FIFO__POS               6
#define BMG160_INTR_ENABLE0_FIFO__LEN               1
#define BMG160_INTR_ENABLE0_FIFO__MSK               0x40
#define BMG160_INTR_ENABLE0_FIFO__REG               BMG160_INTR_ENABLE0_ADDR

/**< 2nd bit of Interrupt Enable 0 Registers */
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__POS        2
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__LEN        1
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__MSK        0x04
#define BMG160_INTR_ENABLE0_AUTO_OFFSET__REG        BMG160_INTR_ENABLE0_ADDR

/**< 3rd bit of Interrupt Enable 1 Registers */
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__POS               3
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__LEN               1
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__MSK               0x08
#define BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__REG               \
BMG160_INTR_ENABLE1_ADDR

/**< 2nd bit of Interrupt Enable 1 Registers */
#define BMG160_INTR_ENABLE1_IT2_LEVEL__POS              2
#define BMG160_INTR_ENABLE1_IT2_LEVEL__LEN              1
#define BMG160_INTR_ENABLE1_IT2_LEVEL__MSK              0x04
#define BMG160_INTR_ENABLE1_IT2_LEVEL__REG              \
BMG160_INTR_ENABLE1_ADDR

/**< 1st bit of Interrupt Enable 1 Registers */
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__POS               1
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__LEN               1
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__MSK               0x02
#define BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__REG               \
BMG160_INTR_ENABLE1_ADDR

/**< 0th bit of Interrupt Enable 1 Registers */
#define BMG160_INTR_ENABLE1_IT1_LEVEL__POS              0
#define BMG160_INTR_ENABLE1_IT1_LEVEL__LEN              1
#define BMG160_INTR_ENABLE1_IT1_LEVEL__MSK              0x01
#define BMG160_INTR_ENABLE1_IT1_LEVEL__REG              \
BMG160_INTR_ENABLE1_ADDR

/**< 3rd bit of Interrupt MAP 0 Registers */
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__POS            3
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__LEN            1
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__MSK            0x08
#define BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__REG            \
BMG160_INTR_MAP_ZERO_ADDR

/**< 1st bit of Interrupt MAP 0 Registers */
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__POS             1
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__LEN             1
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__MSK             0x02
#define BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__REG             \
BMG160_INTR_MAP_ZERO_ADDR

/**< 7th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_DATA__POS                  7
#define BMG160_MAP_ONE_INTR2_DATA__LEN                  1
#define BMG160_MAP_ONE_INTR2_DATA__MSK                  0x80
#define BMG160_MAP_ONE_INTR2_DATA__REG                  \
BMG160_INTR_MAP_ONE_ADDR


/**< 6th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__POS           6
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__LEN           1
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__MSK           0x40
#define BMG160_MAP_ONE_INTR2_FAST_OFFSET__REG           \
BMG160_INTR_MAP_ONE_ADDR

/**< 5th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_FIFO__POS                  5
#define BMG160_MAP_ONE_INTR2_FIFO__LEN                  1
#define BMG160_MAP_ONE_INTR2_FIFO__MSK                  0x20
#define BMG160_MAP_ONE_INTR2_FIFO__REG                  \
BMG160_INTR_MAP_ONE_ADDR

/**< 4th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__POS           4
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__LEN           1
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__MSK           0x10
#define BMG160_MAP_ONE_INTR2_AUTO_OFFSET__REG           \
BMG160_INTR_MAP_ONE_ADDR

/**< 3rd bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__POS           3
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__LEN           1
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__MSK           0x08
#define BMG160_MAP_ONE_INTR1_AUTO_OFFSET__REG           \
BMG160_INTR_MAP_ONE_ADDR

/**< 2nd bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_FIFO__POS                  2
#define BMG160_MAP_ONE_INTR1_FIFO__LEN                  1
#define BMG160_MAP_ONE_INTR1_FIFO__MSK                  0x04
#define BMG160_MAP_ONE_INTR1_FIFO__REG                  \
BMG160_INTR_MAP_ONE_ADDR

/**< 1st bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__POS           1
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__LEN           1
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__MSK           0x02
#define BMG160_MAP_ONE_INTR1_FAST_OFFSET__REG           \
BMG160_INTR_MAP_ONE_ADDR

/**< 0th bit of MAP_1Registers */
#define BMG160_MAP_ONE_INTR1_DATA__POS                  0
#define BMG160_MAP_ONE_INTR1_DATA__LEN                  1
#define BMG160_MAP_ONE_INTR1_DATA__MSK                  0x01
#define BMG160_MAP_ONE_INTR1_DATA__REG                  \
BMG160_INTR_MAP_ONE_ADDR

/**< 3rd bit of Interrupt Map 2 Registers */
#define BMG160_INTR_MAP_TWO_INT2_HIGHRATE__POS            3
#define BMG160_INTR_MAP_TWO_INT2_HIGHRATE__LEN            1
#define BMG160_INTR_MAP_TWO_INT2_HIGHRATE__MSK            0x08
#define BMG160_INTR_MAP_TWO_INT2_HIGHRATE__REG            \
BMG160_INTR_MAP_TWO_ADDR

/**< 1st bit of Interrupt Map 2 Registers */
#define BMG160_INTR_MAP_TWO_INT2_ANY_MOTION__POS             1
#define BMG160_INTR_MAP_TWO_INT2_ANY_MOTION__LEN             1
#define BMG160_INTR_MAP_TWO_INT2_ANY_MOTION__MSK             0x02
#define BMG160_INTR_MAP_TWO_INT2_ANY_MOTION__REG             \
BMG160_INTR_MAP_TWO_ADDR

/**< 5th bit of Interrupt 0 Registers */
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__POS          5
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__LEN          1
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__MSK          0x20
#define BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__REG          \
BMG160_INTR_ZERO_ADDR

/**< 3rd bit of Interrupt 0 Registers */
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__POS            3
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__LEN            1
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__MSK            0x08
#define BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__REG            \
BMG160_INTR_ZERO_ADDR

/**< 1st bit of Interrupt 0 Registers */
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__POS             1
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__LEN             1
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__MSK             0x02
#define BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__REG             \
BMG160_INTR_ZERO_ADDR

/**< 7th bit of INT_1  Registers */
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__POS            7
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__LEN            1
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__MSK            0x80
#define BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__REG            \
BMG160_INTR_ONE_ADDR

/**< First 7 bits of INT_1  Registers */
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__POS                       0
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__LEN                       7
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__MSK                       0x7F
#define BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__REG                       \
BMG160_INTR_ONE_ADDR

/**< Last 2 bits of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__POS          6
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__LEN          2
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__MSK          0xC0
#define BMG160_INTR_TWO_ADDR_AWAKE_DURN__REG          BMG160_INTR_TWO_ADDR

/**< 4th & 5th bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__POS      4
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__LEN      2
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__MSK      0x30
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__REG      \
BMG160_INTR_TWO_ADDR

/**< 2nd bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__POS           2
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__LEN           1
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__MSK           0x04
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__REG           \
BMG160_INTR_TWO_ADDR

/**< 1st bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__POS           1
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__LEN           1
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__MSK           0x02
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__REG           \
BMG160_INTR_TWO_ADDR

/**< 0th bit of INT 2Registers */
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__POS           0
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__LEN           1
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__MSK           0x01
#define BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__REG           \
BMG160_INTR_TWO_ADDR

/**< Last bit of INT 4 Registers */
#define BMG160_INTR_4_FIFO_WM_ENABLE__POS           7
#define BMG160_INTR_4_FIFO_WM_ENABLE__LEN           1
#define BMG160_INTR_4_FIFO_WM_ENABLE__MSK           0x80
#define BMG160_INTR_4_FIFO_WM_ENABLE__REG           BMG160_INTR_4_ADDR

/**< Last bit of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_RST_INTR__POS           7
#define BMG160_RST_LATCH_ADDR_RST_INTR__LEN           1
#define BMG160_RST_LATCH_ADDR_RST_INTR__MSK           0x80
#define BMG160_RST_LATCH_ADDR_RST_INTR__REG           BMG160_RST_LATCH_ADDR

/**< 6th bit of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__POS        6
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__LEN        1
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__MSK        0x40
#define BMG160_RST_LATCH_ADDR_OFFSET_RST__REG        BMG160_RST_LATCH_ADDR

/**< 4th bit of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__POS        4
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__LEN        1
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__MSK        0x10
#define BMG160_RST_LATCH_ADDR_LATCH_STAT__REG        BMG160_RST_LATCH_ADDR

/**< First 4 bits of Reset Latch Registers */
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__POS           0
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__LEN           4
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__MSK           0x0F
#define BMG160_RST_LATCH_ADDR_LATCH_INTR__REG           BMG160_RST_LATCH_ADDR

/**< Last 2 bits of HIGHRATE_THRES_X Registers */
#define BMG160_HIGHRATE_HYST_X__POS        6
#define BMG160_HIGHRATE_HYST_X__LEN        2
#define BMG160_HIGHRATE_HYST_X__MSK        0xC0
#define BMG160_HIGHRATE_HYST_X__REG        BMG160_HIGHRATE_THRES_X_ADDR

/**< 5 bits of HIGHRATE_THRES_X Registers */
#define BMG160_HIGHRATE_THRES_X__POS        1
#define BMG160_HIGHRATE_THRES_X__LEN        5
#define BMG160_HIGHRATE_THRES_X__MSK        0x3E
#define BMG160_HIGHRATE_THRES_X__REG        BMG160_HIGHRATE_THRES_X_ADDR

/**< 0th bit of HIGHRATE_THRES_X Registers */
#define BMG160_HIGHRATE_ENABLE_X__POS        0
#define BMG160_HIGHRATE_ENABLE_X__LEN        1
#define BMG160_HIGHRATE_ENABLE_X__MSK        0x01
#define BMG160_HIGHRATE_ENABLE_X__REG        BMG160_HIGHRATE_THRES_X_ADDR

/**< Last 2 bits of HIGHRATE_THRES_Y Registers */
#define BMG160_HIGHRATE_HYST_Y__POS        6
#define BMG160_HIGHRATE_HYST_Y__LEN        2
#define BMG160_HIGHRATE_HYST_Y__MSK        0xC0
#define BMG160_HIGHRATE_HYST_Y__REG        BMG160_HIGHRATE_THRES_Y_ADDR

/**< 5 bits of HIGHRATE_THRES_Y Registers */
#define BMG160_HIGHRATE_THRES_Y__POS        1
#define BMG160_HIGHRATE_THRES_Y__LEN        5
#define BMG160_HIGHRATE_THRES_Y__MSK        0x3E
#define BMG160_HIGHRATE_THRES_Y__REG        BMG160_HIGHRATE_THRES_Y_ADDR

/**< 0th bit of HIGHRATE_THRES_Y Registers */
#define BMG160_HIGHRATE_ENABLE_Y__POS        0
#define BMG160_HIGHRATE_ENABLE_Y__LEN        1
#define BMG160_HIGHRATE_ENABLE_Y__MSK        0x01
#define BMG160_HIGHRATE_ENABLE_Y__REG        BMG160_HIGHRATE_THRES_Y_ADDR

/**< Last 2 bits of HIGHRATE_THRES_Z Registers */
#define BMG160_HIGHRATE_HYST_Z__POS        6
#define BMG160_HIGHRATE_HYST_Z__LEN        2
#define BMG160_HIGHRATE_HYST_Z__MSK        0xC0
#define BMG160_HIGHRATE_HYST_Z__REG        BMG160_HIGHRATE_THRES_Z_ADDR

/**< 5 bits of HIGHRATE_THRES_Z Registers */
#define BMG160_HIGHRATE_THRES_Z__POS        1
#define BMG160_HIGHRATE_THRES_Z__LEN        5
#define BMG160_HIGHRATE_THRES_Z__MSK        0x3E
#define BMG160_HIGHRATE_THRES_Z__REG        BMG160_HIGHRATE_THRES_Z_ADDR

/**< 0th bit of HIGHRATE_THRES_Z Registers */
#define BMG160_HIGHRATE_ENABLE_Z__POS        0
#define BMG160_HIGHRATE_ENABLE_Z__LEN        1
#define BMG160_HIGHRATE_ENABLE_Z__MSK        0x01
#define BMG160_HIGHRATE_ENABLE_Z__REG        BMG160_HIGHRATE_THRES_Z_ADDR

/**< Last 3 bits of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_THRES__POS          6
#define BMG160_SLOW_OFFSET_THRES__LEN          2
#define BMG160_SLOW_OFFSET_THRES__MSK          0xC0
#define BMG160_SLOW_OFFSET_THRES__REG          BMG160_SOC_ADDR

/**< 2  bits of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_DURN__POS         3
#define BMG160_SLOW_OFFSET_DURN__LEN         3
#define BMG160_SLOW_OFFSET_DURN__MSK         0x38
#define BMG160_SLOW_OFFSET_DURN__REG         BMG160_SOC_ADDR

/**< 2nd bit of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_ENABLE_Z__POS        2
#define BMG160_SLOW_OFFSET_ENABLE_Z__LEN        1
#define BMG160_SLOW_OFFSET_ENABLE_Z__MSK        0x04
#define BMG160_SLOW_OFFSET_ENABLE_Z__REG        BMG160_SOC_ADDR

/**< 1st bit of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_ENABLE_Y__POS        1
#define BMG160_SLOW_OFFSET_ENABLE_Y__LEN        1
#define BMG160_SLOW_OFFSET_ENABLE_Y__MSK        0x02
#define BMG160_SLOW_OFFSET_ENABLE_Y__REG        BMG160_SOC_ADDR

/**< 0th bit of INT OFF0 Registers */
#define BMG160_SLOW_OFFSET_ENABLE_X__POS        0
#define BMG160_SLOW_OFFSET_ENABLE_X__LEN        1
#define BMG160_SLOW_OFFSET_ENABLE_X__MSK        0x01
#define BMG160_SLOW_OFFSET_ENABLE_X__REG        BMG160_SOC_ADDR

/**< Last 2 bits of INT OFF1 Registers */
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__POS        6
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__LEN        2
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__MSK        0xC0
#define BMG160_AUTO_OFFSET_WORD_LENGHTH__REG        BMG160_A_FOC_ADDR

/**< 2  bits of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_WORD_LENGHTH__POS        4
#define BMG160_FAST_OFFSET_WORD_LENGHTH__LEN        2
#define BMG160_FAST_OFFSET_WORD_LENGHTH__MSK        0x30
#define BMG160_FAST_OFFSET_WORD_LENGHTH__REG        BMG160_A_FOC_ADDR

/**< 3nd bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE__POS        3
#define BMG160_FAST_OFFSET_ENABLE__LEN        1
#define BMG160_FAST_OFFSET_ENABLE__MSK        0x08
#define BMG160_FAST_OFFSET_ENABLE__REG        BMG160_A_FOC_ADDR

/**< 2nd bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_Z__POS      2
#define BMG160_FAST_OFFSET_ENABLE_Z__LEN      1
#define BMG160_FAST_OFFSET_ENABLE_Z__MSK      0x04
#define BMG160_FAST_OFFSET_ENABLE_Z__REG      BMG160_A_FOC_ADDR

/**< 1st bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_Y__POS      1
#define BMG160_FAST_OFFSET_ENABLE_Y__LEN      1
#define BMG160_FAST_OFFSET_ENABLE_Y__MSK      0x02
#define BMG160_FAST_OFFSET_ENABLE_Y__REG      BMG160_A_FOC_ADDR

/**< 0th bit of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_X__POS      0
#define BMG160_FAST_OFFSET_ENABLE_X__LEN      1
#define BMG160_FAST_OFFSET_ENABLE_X__MSK      0x01
#define BMG160_FAST_OFFSET_ENABLE_X__REG      BMG160_A_FOC_ADDR

/**< 0 to 2 bits of INT OFF1 Registers */
#define BMG160_FAST_OFFSET_ENABLE_XYZ__POS      0
#define BMG160_FAST_OFFSET_ENABLE_XYZ__LEN      3
#define BMG160_FAST_OFFSET_ENABLE_XYZ__MSK      0x07
#define BMG160_FAST_OFFSET_ENABLE_XYZ__REG      BMG160_A_FOC_ADDR

/**< Last 4 bits of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__POS        4
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__LEN        4
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__MSK        0xF0
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__REG        \
BMG160_TRIM_NVM_CTRL_ADDR

/**< 3rd bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__POS          3
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__LEN          1
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__MSK          0x08
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__REG          \
BMG160_TRIM_NVM_CTRL_ADDR

/**< 2nd bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__POS           2
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__LEN           1
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__MSK           0x04
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__REG           \
BMG160_TRIM_NVM_CTRL_ADDR

 /**< 1st bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__POS     1
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__LEN     1
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__MSK     0x02
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__REG     \
BMG160_TRIM_NVM_CTRL_ADDR

/**< 0th bit of Trim NVM control Registers */
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__POS     0
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__LEN     1
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__MSK     0x01
#define BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__REG     \
BMG160_TRIM_NVM_CTRL_ADDR

 /**< 2nd bit of SPI3 WDT Registers */
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__POS      2
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__LEN      1
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__MSK      0x04
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__REG      \
BMG160_BGW_SPI3_WDT_ADDR

 /**< 1st bit of SPI3 WDT Registers */
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__POS     1
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__LEN     1
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__MSK     0x02
#define BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__REG     \
BMG160_BGW_SPI3_WDT_ADDR

/**< 0th bit of SPI3 WDT Registers */
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__POS            0
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__LEN            1
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__MSK            0x01
#define BMG160_BGW_SPI3_WDT_ADDR_SPI3__REG            \
BMG160_BGW_SPI3_WDT_ADDR

/**< 4th bit of Self test Registers */
#define BMG160_SELECTF_TEST_ADDR_RATEOK__POS            4
#define BMG160_SELECTF_TEST_ADDR_RATEOK__LEN            1
#define BMG160_SELECTF_TEST_ADDR_RATEOK__MSK            0x10
#define BMG160_SELECTF_TEST_ADDR_RATEOK__REG            \
BMG160_SELECTF_TEST_ADDR

/**< 2nd bit of Self test Registers */
#define BMG160_SELECTF_TEST_ADDR_BISTFAIL__POS          2
#define BMG160_SELECTF_TEST_ADDR_BISTFAIL__LEN          1
#define BMG160_SELECTF_TEST_ADDR_BISTFAIL__MSK          0x04
#define BMG160_SELECTF_TEST_ADDR_BISTFAIL__REG          \
BMG160_SELECTF_TEST_ADDR

/**< 1st bit of Self test Registers */
#define BMG160_SELECTF_TEST_ADDR_BISTRDY__POS           1
#define BMG160_SELECTF_TEST_ADDR_BISTRDY__LEN           1
#define BMG160_SELECTF_TEST_ADDR_BISTRDY__MSK           0x02
#define BMG160_SELECTF_TEST_ADDR_BISTRDY__REG           \
BMG160_SELECTF_TEST_ADDR

/**< 0th bit of Self test Registers */
#define BMG160_SELECTF_TEST_ADDR_TRIGBIST__POS          0
#define BMG160_SELECTF_TEST_ADDR_TRIGBIST__LEN          1
#define BMG160_SELECTF_TEST_ADDR_TRIGBIST__MSK          0x01
#define BMG160_SELECTF_TEST_ADDR_TRIGBIST__REG          \
BMG160_SELECTF_TEST_ADDR

/**< 7th bit of FIFO CGF1 Registers */
#define BMG160_FIFO_CGF1_ADDR_TAG__POS     7
#define BMG160_FIFO_CGF1_ADDR_TAG__LEN     1
#define BMG160_FIFO_CGF1_ADDR_TAG__MSK     0x80
#define BMG160_FIFO_CGF1_ADDR_TAG__REG     BMG160_FIFO_CGF1_ADDR

/**< First 7 bits of FIFO CGF1 Registers */
#define BMG160_FIFO_CGF1_ADDR_WML__POS     0
#define BMG160_FIFO_CGF1_ADDR_WML__LEN     7
#define BMG160_FIFO_CGF1_ADDR_WML__MSK     0x7F
#define BMG160_FIFO_CGF1_ADDR_WML__REG     BMG160_FIFO_CGF1_ADDR

/**< Last 2 bits of FIFO CGF0 Addr Registers */
#define BMG160_FIFO_CGF0_ADDR_MODE__POS         6
#define BMG160_FIFO_CGF0_ADDR_MODE__LEN         2
#define BMG160_FIFO_CGF0_ADDR_MODE__MSK         0xC0
#define BMG160_FIFO_CGF0_ADDR_MODE__REG         BMG160_FIFO_CGF0_ADDR

/**< First 2 bits of FIFO CGF0 Addr Registers */
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__POS     0
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__LEN     2
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__MSK     0x03
#define BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG     BMG160_FIFO_CGF0_ADDR

 /**< Last 2 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_X__POS       6
#define BMG160_OFC1_ADDR_OFFSET_X__LEN       2
#define BMG160_OFC1_ADDR_OFFSET_X__MSK       0xC0
#define BMG160_OFC1_ADDR_OFFSET_X__REG       BMG160_OFC1_ADDR

/**< 3 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_Y__POS       3
#define BMG160_OFC1_ADDR_OFFSET_Y__LEN       3
#define BMG160_OFC1_ADDR_OFFSET_Y__MSK       0x38
#define BMG160_OFC1_ADDR_OFFSET_Y__REG       BMG160_OFC1_ADDR

/**< First 3 bits of INL Offset MSB Registers */
#define BMG160_OFC1_ADDR_OFFSET_Z__POS       0
#define BMG160_OFC1_ADDR_OFFSET_Z__LEN       3
#define BMG160_OFC1_ADDR_OFFSET_Z__MSK       0x07
#define BMG160_OFC1_ADDR_OFFSET_Z__REG       BMG160_OFC1_ADDR

/**< 4 bits of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_GP0__POS            4
#define BMG160_TRIM_GP0_ADDR_GP0__LEN            4
#define BMG160_TRIM_GP0_ADDR_GP0__MSK            0xF0
#define BMG160_TRIM_GP0_ADDR_GP0__REG            BMG160_TRIM_GP0_ADDR

/**< 2 bits of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__POS       2
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__LEN       2
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__MSK       0x0C
#define BMG160_TRIM_GP0_ADDR_OFFSET_X__REG       BMG160_TRIM_GP0_ADDR

/**< 1st bit of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__POS       1
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__LEN       1
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__MSK       0x02
#define BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG       BMG160_TRIM_GP0_ADDR

/**< First bit of Trim GP0 Registers */
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__POS       0
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__LEN       1
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__MSK       0x01
#define BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG       BMG160_TRIM_GP0_ADDR

/* For Axis Selection   */
/**< It refers BMG160 X-axis */
#define BMG160_X_AXIS           0
/**< It refers BMG160 Y-axis */
#define BMG160_Y_AXIS           1
/**< It refers BMG160 Z-axis */
#define BMG160_Z_AXIS           2

/* For Mode Settings    */
#define BMG160_MODE_NORMAL              0
#define BMG160_MODE_DEEPSUSPEND         1
#define BMG160_MODE_SUSPEND             2
#define BMG160_MODE_FASTPOWERUP			3
#define BMG160_MODE_ADVANCEDPOWERSAVING 4

/* get bit slice  */
#define BMG160_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)

/* Set bit slice */
#define BMG160_SET_BITSLICE(regvar, bitname, val)\
((regvar&~bitname##__MSK)|((val<<bitname##__POS)&bitname##__MSK))
/* Constants */

#define BMG160_NULL                             0
/**< constant declaration of NULL */
#define BMG160_DISABLE                          0
/**< It refers BMG160 disable */
#define BMG160_ENABLE                           1
/**< It refers BMG160 enable */
#define BMG160_OFF                              0
/**< It refers BMG160 OFF state */
#define BMG160_ON                               1
/**< It refers BMG160 ON state  */


#define BMG160_TURN1                               0
/**< It refers BMG160 TURN1 */
#define BMG160_TURN2                               1
/**< It refers BMG160 TURN2 */

#define BMG160_INTR1                              0
/**< It refers BMG160 INT1 */
#define BMG160_INTR2                              1
/**< It refers BMG160 INT2 */

#define BMG160_SLOW_OFFSET                         0
/**< It refers BMG160 Slow Offset */
#define BMG160_AUTO_OFFSET                         1
/**< It refers BMG160 Auto Offset */
#define BMG160_FAST_OFFSET                         2
/**< It refers BMG160 Fast Offset */
#define BMG160_S_TAP                               0
/**< It refers BMG160 Single Tap */
#define BMG160_D_TAP                               1
/**< It refers BMG160 Double Tap */
#define BMG160_INTR1_DATA                          0
/**< It refers BMG160 Int1 Data */
#define BMG160_INTR2_DATA                          1
/**< It refers BMG160 Int2 Data */
#define BMG160_TAP_UNFILT_DATA                     0
/**< It refers BMG160 Tap unfilt data */
#define BMG160_HIGHRATE_UNFILT_DATA                1
/**< It refers BMG160 High unfilt data */
#define BMG160_CONST_UNFILT_DATA                   2
/**< It refers BMG160 Const unfilt data */
#define BMG160_ANY_MOTION_UNFILT_DATA              3
/**< It refers BMG160 Any unfilt data */
#define BMG160_SHAKE_UNFILT_DATA                   4
/**< It refers BMG160 Shake unfilt data */
#define BMG160_SHAKE_TH                            0
/**< It refers BMG160 Shake Threshold */
#define BMG160_SHAKE_TH2                           1
/**< It refers BMG160 Shake Threshold2 */
#define BMG160_AUTO_OFFSET_WORD_LENGHTH            0
/**< It refers BMG160 Auto Offset word length */
#define BMG160_FAST_OFFSET_WORD_LENGHTH            1
/**< It refers BMG160 Fast Offset word length */
#define BMG160_I2C_WDT_ENABLE                      0
/**< It refers BMG160 I2C WDT En */
#define BMG160_I2C_WDT_SELECT                      1
/**< It refers BMG160 I2C WDT Sel */
#define BMG160_EXT_MODE                         0
/**< It refers BMG160 Ext Mode */
#define BMG160_EXT_PAGE                         1
/**< It refers BMG160 Ext page */
#define BMG160_START_ADDR                       0
/**< It refers BMG160 Start Address */
#define BMG160_STOP_ADDR                        1
/**< It refers BMG160 Stop Address */
#define BMG160_SLOW_CMD                         0
/**< It refers BMG160 Slow Command */
#define BMG160_FAST_CMD                         1
/**< It refers BMG160 Fast Command */
#define BMG160_TRIM_VRA                         0
/**< It refers BMG160 Trim VRA */
#define BMG160_TRIM_VRD                         1
/**< It refers BMG160 Trim VRD */
#define BMG160_LOGBIT_EM                        0
/**< It refers BMG160 LogBit Em */
#define BMG160_LOGBIT_VM                        1
/**< It refers BMG160 LogBit VM */
#define BMG160_GP0                              0
/**< It refers BMG160 GP0 */
#define BMG160_GP1                              1
/**< It refers BMG160 GP1*/
#define BMG160_LOW_SPEED                        0
/**< It refers BMG160 Low Speed Oscillator */
#define BMG160_HIGHRATE_SPEED                   1
/**< It refers BMG160 High Speed Oscillator */
#define BMG160_DRIVE_OFFSET_P                   0
/**< It refers BMG160 Drive Offset P */
#define BMG160_DRIVE_OFFSET_N                   1
/**< It refers BMG160 Drive Offset N */
#define BMG160_TEST_MODE_ENABLE                 0
/**< It refers BMG160 Test Mode Enable */
#define BMG160_TEST_MODE_REG                    1
/**< It refers BMG160 Test Mode reg */
#define BMG160_IBIAS_DRIVE_TRIM                 0
/**< It refers BMG160 IBIAS Drive Trim */
#define BMG160_IBIAS_RATE_TRIM                  1
/**< It refers BMG160 IBIAS Rate Trim */
#define BMG160_BAA_MODE                         0
/**< It refers BMG160 BAA Mode Trim */
#define BMG160_BMA_MODE                         1
/**< It refers BMG160 BMA Mode Trim */
#define BMG160_PI_KP                            0
/**< It refers BMG160 PI KP */
#define BMG160_PI_KI                            1
/**< It refers BMG160 PI KI */


#define C_BMG160_SUCCESS						0
/**< It refers BMG160 operation is success */
#define C_BMG160_FAILURE						1
/**< It refers BMG160 operation is Failure */

#define BMG160_SPI_RD_MASK                      0x80
/**< Read mask **/
#define BMG160_READ_SET                         0x01
/**< Setting for reading data **/

#define BMG160_SHIFT_ONE_POSITION                 1
/**< Shift bit by 1 Position **/
#define BMG160_SHIFT_TWO_POSITION                 4
/**< Shift bit by 2 Position **/
#define BMG160_SHIFT_FOUR_POSITION                   4
/**< Shift bit by 4 Position **/
#define BMG160_SHIFT_EIGHT_POSITION                   8
/**< Shift bit by 8 Position **/

#define         C_BMG160_NULL_U8X				((u8)0)
#define         C_BMG160_ZERO_U8X				((u8)0)
#define         C_BMG160_ONE_U8X				((u8)1)
#define         C_BMG160_TWO_U8X				((u8)2)
#define         C_BMG160_FOUR_U8X				((u8)4)
#define         C_BMG160_FIVE_U8X				((u8)5)
#define         C_BMG160_EIGHT_U8X				((u8)8)
#define         C_BMG160_ONETWENTYEIGHT_U8X		((u8)128)

#define E_BMG160_NULL_PTR               ((s8)-127)
#define E_BMG160_OUT_OF_RANGE           ((s8)-2)
#define BMG160_ERROR					((s8)-1)

#define C_BMG160_NO_FILTER_U8X			0
#define	C_BMG160_BW_230HZ_U8X			1
#define	C_BMG160_BW_116HZ_U8X			2
#define	C_BMG160_BW_47HZ_U8X			3
#define	C_BMG160_BW_23HZ_U8X			4
#define	C_BMG160_BW_12HZ_U8X			5
#define	C_BMG160_BW_64HZ_U8X			6
#define	C_BMG160_BW_32HZ_U8X			7


#define C_BMG160_NO_AUTO_SLEEP_DURN_U8X	0
#define	C_BMG160_4MS_AUTO_SLEEP_DURN_U8X	1
#define	C_BMG160_5MS_AUTO_SLEEP_DURN_U8X	2
#define	C_BMG160_8MS_AUTO_SLEEP_DURN_U8X	3
#define	C_BMG160_10MS_AUTO_SLEEP_DURN_U8X	4
#define	C_BMG160_15MS_AUTO_SLEEP_DURN_U8X	5
#define	C_BMG160_20MS_AUTO_SLEEP_DURN_U8X	6
#define	C_BMG160_40MS_AUTO_SLEEP_DURN_U8X	7


#define BMG160_WR_FUNC_PTR s8 (*bus_write)\
(u8, u8, u8 *, u8)
#define BMG160_RD_FUNC_PTR s8 (*bus_read)\
(u8, u8, u8 *, u8)
#define BMG160_BRD_FUNC_PTR s8 (*burst_read)\
(u8, u8, u8 *, s32)
#define BMG160_MDELAY_DATA_TYPE u32


/*user defined Structures*/
struct bmg160_data_t {
	s16 datax;
	s16 datay;
	s16 dataz;
	char intstatus[5];
};


struct bmg160_offset_t {
	u16 datax;
	u16 datay;
	u16 dataz;
};


struct bmg160_t {
	u8 chip_id;
	u8 dev_addr;
	BMG160_BRD_FUNC_PTR;
	BMG160_WR_FUNC_PTR;
	BMG160_RD_FUNC_PTR;
	void(*delay_msec)(BMG160_MDELAY_DATA_TYPE);
};
/*****************************************************************************
 *	Description: *//**\brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the gyro
 *	chip id is read in the register 0x00 bit from 0 to 7
 *
 *	 \param bmg160_t *p_bmg160 structure pointer.
 *
 *	While changing the parameter of the bmg160_t
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_init(struct bmg160_t *bmg160);
/*****************************************************************************
 * Description: *//**brief Reads Rate dataX in the registers 02h and 03h
 *
 *
 *
 *
 *  \param
 *      s16  *v_data_x_s16   :  Pointer holding the data x
 *
 *
 *  \return
 *      result of communication routines
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_X(s16 *v_data_x_s16);
/*****************************************************************************
 * Description: *//**brief Reads Rate dataY in the registers 04h and 05h
 *
 *
 *
 *
 *  \param
 *      s16  *v_data_y_s16   :  Pointer holding the data y
 *
 *
 *  \return
 *      result of communication routines
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_Y(s16 *v_data_y_s16);
/*****************************************************************************
 *	Description: *//**brief Reads Rate dataZ in the registers 06h and 07h
 *
 *
 *
 *
 *  \param
 *      s16  *v_data_z_s16   :  Pointer holding the data z
 *
 *
 *  \return
 *      result of communication routines
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_Z(s16 *v_data_z_s16);
/*****************************************************************************
 * Description: *//**brief Reads data X,Y and Z from location 02h to 07h
 *
 *
 *
 *
 *	\param
 *      bmg160_data_t *data :  Pointer holding the bmg160_data_t
 *
 *
 *  \return
 *      results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_XYZ(struct bmg160_data_t *data);
/*****************************************************************************
 * Description: *//**brief Reads data X,Y,Z and Interrupts
 *							from location 02h to 07h
 *
 *
 *
 *
 *	\param
 *      bmg160_data_t *data   :  Pointer holding the bmg160_data_t
 *
 *
 *  \return
 *      result of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_XYZI(struct bmg160_data_t *data);
/*****************************************************************************
 * Description: *//**brief Reads Temperature from location 08h
 *
 *
 *
 *
 *  \param
 *      s8 *v_temp_s8   :  Address of temperature
 *
 *
 *  \return
 *      result of communication routines
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_temp(s8 *v_temp_s8);
/*****************************************************************************
 * Description: *//**brief This API reads the data from the given register
 *
 *
 *
 *
 *\param u8 v_addr_u8, u8 *v_data_u8 u8 len
 *       v_addr_u8 -> Address of the register
 *       v_data_u8 -> address of the variable, read value will be
 *	     kept
 *		v_len_u8 -> No of byte to be read.
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_read_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8);
/*****************************************************************************
 * Description: *//**\brief This API reads the data from
 *           the given register address
 *
 *
 *
 *
 *  \param u8 v_addr_u8, u8 *data, u8 v_len_u8
 *         v_addr_u8 -> Address of the register
 *         v_data_u8 -> address of the variable, read value will be kept
 *         v_len_u8  -> Length of the data
 *
 *
 *
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_burst_read(u8 v_addr_u8,
u8 *v_data_u8, s32 v_len_u8);
/*****************************************************************************
 * Description: *//**brief This API given data to the given register
 *
 *
 *
 *
 *\param u8 v_addr_u8, u8 data,u8 v_len_u8
 *                   v_addr_u8 -> Address of the register
 *                   v_data_u8 -> Data to be written to the register
 *					v_len_u8 -> No of byte to be read.
 *
 *  \return Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_write_register(u8 v_addr_u8,
u8 *v_data_u8, u8 v_len_u8);
/*****************************************************************************
 *	Description: *//** This api used to reads interrupt status of
 *	any motion and high rate in the register 09h
 *	any motion bit	->	2
 *	high rate bit	->	1
 *
 *
 *
 *	\param
 *	u8 *v_stat0_data_u8 : Pointer holding the interrupt status of
 *	any motion and high rate
 *
 *
 *  \return
 *      Result of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_stat_reg_zero(
u8 *v_stat0_data_u8);
/*****************************************************************************
 *	Description: *//** This api used to reads the interrupt status of
 *	data, auto_offset, fast_offset and fifo_int in the register 0Ah
 *	data bit	->	7
 *	auto_offset bit	->	6
 *	fast_offset bit	->	5
 *	fifo_int bit	->	4
 *
 *
 *
 *	\param
 *	u8 *v_stat1_data_u8 : Pointer holding the the interrupt status of
 *	data, auto_offset, fast_offset and fifo_int
 *
 *
 *  \return
 *      Result of bus communication function
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
 *****************************************************************************/

BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_stat_reg_one(
u8 *v_stat1_data_u8);
/*****************************************************************************
 *	Description: *//** This api used to reads the interrupt status of
 *	any_sign, any_first_z, any_first_x and any_first_y in the register 0Bh
 *	any_sign bit	->	3
 *	any_first_z bit	->	2
 *	any_first_x bit	->	1
 *	any_first_y bit	->	0
 *
 *
 *
 *	\param
 *	u8 *v_stat2_data_u8 : Pointer holding the the interrupt status of
 *	any_sign, any_first_z, any_first_x and any_first_y
 *
 *
 *  \return
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_stat_reg_two(
u8 *v_stat2_data_u8);
/*****************************************************************************
 *	Description: *//** This api used to reads the interrupt status of
 *	high_sign, high_first_z, high_first_x and high_first_y in the register 0Ch
 *	high_sign bit	->	3
 *	high_first_z bit	->	2
 *	high_first_x bit	->	1
 *	high_first_y bit	->	0
 *
 *
 *
 *	\param
 *	u8 *v_stat3_data_u8 : Pointer holding the the interrupt status of
 *	high_sign, high_first_z, high_first_x and high_first_y
 *
 *
 *  \return
 *      Result of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_stat_reg_three(
u8 *v_stat3_data_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get
 *	the range in the register 0x0Fh bits from 0 to 2
 *
 *	\param u8 v_range_u8 :  Pointer holding the gyro range
 *	v_range_u8[0....7]
 *	0x00 2000/s
 *	0x01 1000/s
 *	0x02 500/s
 *	0x03 250/s
 *	0x04 125/s
 *
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_range_reg(u8 *v_range_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of
 *	the gyro v_range_u8 in the register 0x0Fh bits from 0 to 2
 *
 *	\param u8: Pointer holding the gyro v_range_u8
 *	v_range_u8[0....7]
 *	0x00 2000/s
 *	0x01 1000/s
 *	0x02 500/s
 *	0x03 250/s
 *	0x04 125/s
 *
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_range_reg(u8 v_range_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the gyro bandwidth
 *	in the register 0x10 bits from 0 to 3
 *
 *
 *
 *
 *
 *	\param  u8 *v_bw_u8: pointer holding the value of gyro bandwidth
 *	v_bw_u8->
 *	0x00 -> no filter(523 Hz)
 *	0x01 -> 230Hz
 *	0x02 -> 116Hz
 *	0x03 -> 47Hz
 *	0x04 -> 23Hz
 *	0x05 -> 12Hz
 *	0x06 -> 64Hz
 *	0x07 -> 32Hz
 *
 *
 *	\return Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_bw(u8 *v_bw_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of
 *	gyro bandwidth in the register 0x10 bits from 0 to 3
 *
 *
 *
 *
 *
 *	\param  u8 v_bw_u8: the value of gyro bandwidth
 *	v_bw_u8->
 *	0x00 -> no filter(523 Hz)
 *	0x01 -> 230Hz
 *	0x02 -> 116Hz
 *	0x03 -> 47Hz
 *	0x04 -> 23Hz
 *	0x05 -> 12Hz
 *	0x06 -> 64Hz
 *	0x07 -> 32Hz
 *
 *
 *	\return Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_bw(u8 v_bw_u8);
/*****************************************************************************
 *	Description: *//**brief This API used to get the status of
 *	External Trigger selection in the register 0x12h bits from 4 to 5
 *
 *
 *
 *
 *	\param u8 *v_pwu_ext_tri_select_u8:
 *	Pointer holding the External Trigger selection
 *	v_pwu_ext_tri_select_u8		Trigger source
 *		0x00				No
 *		0x01				INT1 pin
 *		0x02				INT2 pin
 *		0x03				SDO pin(SPI3 mode)
 *
 *
 *	\return Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_pmu_ext_tri_select(
u8 *v_pwu_ext_tri_select_u8);
/*****************************************************************************
 *	Description: *//**brief This API used to set the value of
 *	External Trigger selection in the register 0x12h bits from 4 to 5
 *
 *
 *
 *
 *	\param u8 v_pwu_ext_tri_select_u8:
 *	Pointer holding the External Trigger selection
 *	v_pwu_ext_tri_select_u8		Trigger source
 *		0x00				No
 *		0x01				INT1 pin
 *		0x02				INT2 pin
 *		0x03				SDO pin(SPI3 mode)
 *
 *
 *	\return Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_pmu_ext_tri_select(
u8 v_pwu_ext_tri_select_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to get data high bandwidth
 *	in the register 0x13 bit 7
 *
 *
 *
 *	\param u8 *v_high_bw_u8 : Pointer holding the high bandwidth
 *	1	->	unfiltered
 *	0	->	filtered
 *
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_high_bw(u8 *v_high_bw_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to set data high bandwidth
 *	in the register 0x13 bit 7
 *
 *
 *
 *	\param u8 v_high_bw_u8 : the value of v_high_bw_u8
 *	1	->	unfiltered
 *	0	->	filtered
 *
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_high_bw(u8 v_high_bw_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to get the shadow dis
 *	in the register 0x13 bit 6
 *
 *
 *
 *	\param u8 *v_shadow_dis_u8 : Pointer holding the value of shadow dis
 *	1	->	enable
 *	0	->	disable
 *
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_shadow_dis(u8 *v_shadow_dis_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to set the value of shadow dis
 *	in the register 0x13 bit 6
 *
 *
 *
 *	\param u8 *v_shadow_dis_u8 : the value of shadow dis
 *	1	->	enable
 *	0	->	disable
 *
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_shadow_dis(u8 v_shadow_dis_u8);
/*****************************************************************************
 *	This function is used for the soft reset
 *	The soft reset register will be written with 0xB6 in the register 0x14.
 *
 *
 *
 *
 *	\param  None
 *
 *
 *
 *	\return Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_soft_rst(void);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get the data(data_enable)
 *	interrupt enable bits of the sensor in the registers 0x15 bit 7
 *
 *
 *
 *
 *	\param u8 *v_data_enable_u8 : Pointer holding the data enable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_enable(u8 *v_data_enable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	data(v_data_enable_u8) interrupt enable bits of the sensor in
 *	the registers 0x15 bit 7
 *
 *
 *	\param u8 v_data_enable_u8: The value of data interrupt enable
 *
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_data_enable(u8 v_data_enable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get the fifo(fifo_enable)
 *	interrupt enable bits of the sensor in the registers 0x15 bit 6
 *
 *
 *
 *
 *	\param u8 *v_fifo_enable_u8 : Pointer holding the fifo enable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_enable(u8 *v_fifo_enable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	fifo(v_fifo_enable_u8) interrupt enable bits of the sensor in
 *	the registers 0x15 bit 6
 *
 *
 *	\param u8 v_fifo_enable_u8: The value of fifo interrupt enable
 *
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_enable(u8 v_fifo_enable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the auto offset(auto_offset_enable) interrupt enable bits of
 *	the sensor in the registers 0x15 bit 3
 *
 *
 *
 *	\param u8 *v_offset_enable_u8 : Pointer holding the offset enable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_auto_offset_enable(
u8 *v_offset_enable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the auto offset(auto_offset_enable) interrupt enable bits of
 *	the sensor in the registers 0x15 bit 3
 *
 *
 *
 *	\param u8 v_offset_enable_u8 : the value of auto offset enable/disable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_auto_offset_enable(
u8 v_offset_enable_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the output type status in the register 0x16.
 *	INT1 -> bit 1
 *	INT2 -> bit 3
 *
 *	\param u8 channel:
 *   The value of output type param number
 *	v_param_u8 -->	BMG160_INTR1    ->   0
 *				BMG160_INTR2    ->   1
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_output_type(u8 v_param_u8,
u8 *v_intr_output_type_u8);
/*****************************************************************************
 * Description: *//**\brief This API is used to set
 *	the output type status in the register 0x16.
 *	INT1 -> bit 1
 *	INT2 -> bit 3
 *
 *	\param u8 channel:
 *	The value of output type status v_param_u8 number
 *	v_param_u8 -->	BMG160_INTR1    ->   0
 *				BMG160_INTR2    ->   1
 *
 *  u8  v_intr_output_type_u8: The output type status value
 *	open drain   ->   1
 *	push pull    ->   0
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr_output_type(u8 v_param_u8,
u8 v_intr_output_type_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	Active Level status in the register 0x16
 *	INT1 -> bit 0
 *	INT2 -> bit 2
 *
 *	\param  u8 v_param_u8:
 *	The value of Active Level param number
 *	v_param_u8 -->	BMG160_INTR1    ->    0
 *				BMG160_INTR1    ->    1
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_level(u8 v_param_u8,
u8 *v_intr_level_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	Active Level status in the register 0x16
 *	INT1 -> bit 0
 *	INT2 -> bit 2
 *
 *	\param u8 v_param_u8:
 *	The value of Active Level param number
 *	v_param_u8 -->	BMG160_INTR1    ->    0
 *				BMG160_INTR2    ->    1
 *
 *  u8 v_intr_level_u8: The value of Active Level status
 *	Active HIGH   ->   1
 *	Active LOW    ->   0
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr_level(u8 v_param_u8,
u8 v_intr_level_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the high rate(int1_high) interrupt1 enable bits of
 *	the sensor in the registers 0x17 bit 3
 *
 *
 *
 *	\param u8 *v_intr1__u8 : Pointer holding the int1 high value
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr1_highrate(u8 *v_intr1__u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the high rate(v_intr1_high_u8) interrupt1 enable bits of
 *	the sensor in the registers 0x17 bit 3
 *
 *
 *
 *	\param u8 v_intr1__u8 : the value of  enable/disable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr1_highrate(
u8 v_intr1__u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the any motion(int1_any) interrupt1 enable bits of
 *	the sensor in the registers 0x17 bit 1
 *
 *
 *
 *	\param u8 *v_int1r_any_motion_u8 : Pointer holding the interrupt
 *	anymotion value
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr1_any_motion(
u8 *v_int1r_any_motion_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the any motion(int1_any) interrupt1 enable bits of
 *	the sensor in the registers 0x17 bit 1
 *
 *
 *
 *	\param u8 *v_int1r_any_motion_u8 :
 *	the value of interrupt1 enable/disable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr1_any_motion(
u8 v_int1r_any_motion_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the data interrupt1 and interrupt2(int1_data and int2_data)
 *	in the register 0x18
 *	INT1 -> bit 0
 *	INT2 -> bit 7
 *
 *	\param u8 v_axis_u8:
 *	The value of Active Level axis number
 *	v_axis_u8 -->	BMG160_INTR1_DATA    ->    0
 *				BMG160_INT2_DATA    ->    1
 *
 *  u8 *v_intr_data_u8:
 *	Pointer holding the value of data interrupt1 or interrupt2
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_data(u8 v_axis_u8,
u8 *v_intr_data_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the data interrupt1 and interrupt2(int1_data and int2_data)
 *	in the register 0x18
 *	INT1 -> bit 0
 *	INT2 -> bit 7
 *
 *	\param u8 v_axis_u8:
 *	The value of Active Level v_axis_u8 number
 *	v_axis_u8 -->	BMG160_INTR1_DATA    ->    0
 *				BMG160_INTR2_DATA    ->    1
 *
 *  u8 v_intr_data_u8: The value of interrupt data enable/disable
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr_data(u8 v_axis_u8,
u8 v_intr_data_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the fast offset(int2_fast_offset) and auto offset(int2_auto_offset)
 *	of interrupt2 in the register 0x18
 *	int2_fast_offset -> bit 6
 *	int2_auto_offset -> bit 4
 *
 *	\param u8 v_axis_u8: The value of fast/auto offset interrupts
 *	v_axis_u8 -->	BMG160_FAST_OFFSET    ->    1
 *				BMG160_AUTO_OFFSET    ->    2
 *
 *  u8 v_intr2_offset_u8: the value of fast/auto offset enable/disable
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr2_offset(u8 v_axis_u8,
u8 v_intr2_offset_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fast offset(int1_fast_offset) and auto offset(int1_auto_offset)
 *	of interrupt1 in the register 0x18
 *	int1_fast_offset -> bit 1
 *	int1_auto_offset -> bit 3
 *
 *	\param u8 v_axis_u8: The value of fast/auto offset interrupts
 *	v_axis_u8 -->	BMG160_FAST_OFFSET    ->    1
 *				BMG160_AUTO_OFFSET    ->    2
 *
 *  u8 *v_intr1_offset_u8: Pointer holding the value of fast/auto offset
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr_offset(u8 v_axis_u8,
u8 *v_intr1_offset_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the fast offset(int1_fast_offset) and auto offset(int1_auto_offset)
 *	of interrupt1 in the register 0x18
 *	int1_fast_offset -> bit 1
 *	int1_auto_offset -> bit 3
 *
 *	\param u8 v_axis_u8: The value of fast/auto offset interrupts
 *	v_axis_u8 -->	BMG160_FAST_OFFSET    ->    1
 *				BMG160_AUTO_OFFSET    ->    2
 *
 *  u8 v_intr1_offset_u8: the value of fast/auto offset enable/disable
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr1_offset(u8 v_axis_u8,
u8 v_intr1_offset_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo(int2_fifo) interrupt2 enable bits of
 *	the sensor in the registers 0x18 bit 5
 *
 *
 *
 *	\param u8 *v_intr_fifo_u8 : Pointer holding the interrupt2 fifo value
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr2_fifo(u8 *v_intr_fifo_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fifo(int1_fifo) interrupt1 enable bits of
 *	the sensor in the registers 0x18 bit 2
 *
 *
 *
 *	\param u8 *v_intr_fifo_u8 : Pointer holding the v_intr_fifo_u8 value
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
 *
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr1_fifo(u8 *v_intr_fifo_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the fifo interrupt1 and interrupt2(int1_fifo and int2_fifo)
 *	in the register 0x18
 *	int1_fifo -> bit 2
 *	int2_fifo -> bit 5
 *
 *	\param u8 v_axis_u8: The value of int1_fifo/int2_fifo interrupts
 *	v_axis_u8 -->	BMG160_INTR1    ->    0
 *					BMG160_INTR2    ->    1
 *
 *  u8 v_intr_fifo_u8: the value of int1_fifo/int2_fifo enable/disable
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr_fifo(u8 v_axis_u8,
u8 v_intr_fifo_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the high rate(int2_high) interrupt2 enable bits of
 *	the sensor in the registers 0x19 bit 3
 *
 *
 *
 *	\param u8 *v_intr2_highrate_u8 :
 *	Pointer holding the interrupt2 high value
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr2_highrate(
u8 *v_intr2_highrate_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the high rate(v_intr2_high_u8) interrupt2 enable bits of
 *	the sensor in the registers 0x19 bit 3
 *
 *
 *
 *	\param u8 v_intr2_highrate_u8 : the value interrupt2 enable/disable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr2_highrate(
u8 v_intr2_highrate_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the any motion(int2_any_motion) interrupt1 enable bits of
 *	the sensor in the registers 0x19 bit 1
 *
 *
 *
 *	\param u8 *v_intr2_any_motion_u8 :
 *	Pointer holding the int2 any_motion value
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr2_any_motion(
u8 *v_intr2_any_motion_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the any motion(v_intr2_any_motion_u8) interrupt2 enable bits of
 *	the sensor in the registers 0x19 bit 1
 *
 *
 *
 *	\param u8 *v_intr2_any_motion_u8 :
 *	the value of  intr2 any_motion_u8 enable/disable
 *	0 -> interrupt disable
 *	1 -> interrupt enable
 *
 *
 *	\return	Results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_intr2_any_motion(
u8 v_intr2_any_motion_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the slow offset(slow_offset_unfilt) and fast offset(fast_offset_unfilt)
 *	unfilt data in the register 0x1A and 1B
 *	slow_offset_unfilt -> 0x1A bit 5
 *	fast_offset_unfilt -> 0x1B bit 7
 *
 *	\param u8 v_param_u8: The value of fast/slow offset unfilt data
 *	v_param_u8 -->	BMG160_SLOW_OFFSET    ->    0
 *					BMG160_FAST_OFFSET    ->    2
 *
 *  u8 *v_offset_unfilt_u8: Pointer holding the value of fast/slow offset
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_offset_unfilt(u8 v_param_u8,
u8 *v_offset_unfilt_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the slow offset(slow_offset_unfilt) and fast offset(fast_offset_unfilt)
 *	unfilt data in the register 0x1A and 1B
 *	slow_offset_unfilt -> 0x1A bit 5
 *	fast_offset_unfilt -> 0x1B bit 7
 *
 *	\param u8 v_param_u8: The value of fast/slow offset unfilt data
 *	v_param_u8 -->	BMG160_SLOW_OFFSET    ->    0
 *				BMG160_FAST_OFFSET    ->    2
 *
 *  u8 v_offset_unfilt_u8: the value of fast/slow offset enable/disable
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_offset_unfilt(u8 v_param_u8,
u8 v_offset_unfilt_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the any motion(any_unfilt_data) and high rate(high_unfilt_data)
 *	unfilt data in the register 0x1A
 *	any_unfilt_data -> bit 1
 *	high_unfilt_data -> bit 3
 *
 *	\param u8 v_param_u8: The value of any/high offset unfilt data
 *	v_param_u8 -->	BMG160_HIGHRATE_UNFILT_DATA    ->    1
 *					BMG160_ANY_MOTION_UNFILT_DATA    ->    3
 *
 *  u8 *v_unfilt_data_u8: Pointer holding the value of any/high unfilt data
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_unfilt_data(u8 v_param_u8,
u8 *v_unfilt_data_u8);
/*****************************************************************************
 *	Description: *//**\brief This API is used to set the value of
 *	the any motion(any_unfilt_data) and high rate(high_unfilt_data)
 *	unfilt data in the register 0x1A
 *	any_unfilt_data -> bit 1
 *	high_unfilt_data -> bit 3
 *
 *	\param u8 v_param_u8: The value of any/high offset unfilt data
 *	v_param_u8 -->	BMG160_HIGHRATE_UNFILT_DATA    ->    1
 *				BMG160_ANY_MOTION_UNFILT_DATA    ->    3
 *
 *  u8 v_unfilt_data_u8: the value of any/high unfilt data
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_unfilt_data(u8 v_param_u8,
u8 v_unfilt_data_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get Any Threshold
 *	in the register 0x1B bit from 0 to 6
 *
 *
 *
 *	\param u8 *v_any_motion_thres_u8 :
 *	Pointer holding the any_motion Threshold value
 *
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_any_motion_thres(
u8 *v_any_motion_thres_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of
 *	Any Threshold in the register 0x1B bit from 0 to 6
 *
 *
 *
 *	\param u8 any_motion_thres :
 *	the value of any_motion Threshold
 *
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_any_motion_thres(
u8 any_motion_thres);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the awake Duration
 *	in the register 0x1C bit 6 and 7
 *
 *
 *
 *	\param u8 *v_awake_durn_u8 : Pointer holding the awake Duration
 *	v_awake_durn_u8 ->
 *	0x00 -> 8 samples
 *	0x01 -> 16 samples
 *	0x02 -> 32 samples
 *	0x03 -> 64 samples
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_awake_durn(u8 *v_awake_durn_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of
 *	awake Duration in the register 0x1C bit 6 and 7
 *
 *
 *
 *	\param u8 v_awake_durn_u8 : The value of awake duration
 *	v_awake_durn_u8 ->
 *	0x00 -> 8 samples
 *	0x01 -> 16 samples
 *	0x02 -> 32 samples
 *	0x03 -> 64 samples
 *
 *	\return results of bus communication function
 *
 *
 *****************************************************************************
 * Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_awake_durn(u8 v_awake_durn_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get
 *	the any motion Duration samples in the register 0x1C bit 4 and 5
 *
 *
 *
 *	\param u8 *v_durn_sample_u8 : Pointer holding the dursample
 *	v_durn_sample_u8 ->
 *	0x00 -> 4 samples
 *	0x01 -> 8 samples
 *	0x02 -> 12 samples
 *	0x03 -> 16 samples
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_any_motion_durn_sample(
u8 *v_durn_sample_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of
 *	the any motion Duration samples in the register 0x1C bit 4 and 5
 *
 *
 *
 *	\param u8 v_durn_sample_u8 : The value of duration sample
 *	v_durn_sample_u8 ->
 *	0x00 -> 4 samples
 *	0x01 -> 8 samples
 *	0x02 -> 12 samples
 *	0x03 -> 16 samples
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_any_motion_durn_sample(
u8 v_durn_sample_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of
 *	Any motion interrupt axis(X,Y,Z) enable channel
 *	BMG160_X_AXIS -> bit 0
 *	BMG160_Y_AXIS -> bit 1
 *	BMG160_Z_AXIS -> bit 2
 *
 *	\param u8 v_channel_u8 : The value of Any Enable channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 *data: Pointer holding the Any Enable value
 *	v_data_u8 :
 *	Enable  -> 1
 *	disable -> 0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_any_motion_enable_axis(u8 v_channel_u8,
u8 *v_data_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of
 *	Any motion interrupt v_axis_u8(X,Y,Z) enable channel
 *	BMG160_X_AXIS -> bit 0
 *	BMG160_Y_AXIS -> bit 1
 *	BMG160_Z_AXIS -> bit 2
 *
 *	\param u8 v_channel_u8 : The value of Any Enable channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 data: The value of Any Enable
 *	v_data_u8 :
 *	Enable  -> 1
 *	disable -> 0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_any_motion_enable_axis(u8 v_channel_u8,
u8 v_data_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get
 *	the status of fifo water mark in the register 0x1E bit 7
 *
 *
 *
 *	\param u8 *v_fifo_wm_enable_u8 :
 *	Pointer holding the fifo water mark enable
 *	v_fifo_wm_enable_u8 ->
 *	0 -> disable
 *	1 -> enable
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_wm_enable(
u8 *v_fifo_wm_enable_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set
 *	the value of fifo water mark in the register 0x1E bit 7
 *
 *
 *
 *	\param u8 v_fifo_wm_enable_u8 : The value of fifo water mark enable
 *	v_fifo_wm_enable_u8 ->
 *	0 -> disable
 *	1 -> enable
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_wm_enable(
u8 v_fifo_wm_enable_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the Interrupt Reset
 *	in the register 0x21 bit 7
 *
 *
 *
 *	\param u8 v_rst_int_u8: the value of reset interrupt
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_rst_intr(u8 v_rst_int_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the offset Reset
 *	in the register 0x21 bit 6
 *
 *
 *
 *	\param u8 v_offset_rst_u8: the value of reset offset
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_offset_rst(
u8 v_offset_rst_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the Latch Status
 *	in the register 0x21 bit 4
 *
 *
 *
 *	\param u8 *v_latch_stat_u8 : Pointer holding the latch_status
 *	v_latch_stat_u8 ->
 *	0 -> disable
 *	1 -> enable
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_latch_stat(
u8 *v_latch_stat_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the Latch value
 *	in the register 0x21 bit 4
 *
 *
 *
 *	\param u8 v_latch_stat_u8 : the value of latch status
 *	v_latch_stat_u8 ->
 *	0 -> disable
 *	1 -> enable
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_latch_stat(
u8 v_latch_stat_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the Latch interrupt
 *	in the register 0x21 bit from 0 to 3
 *
 *
 *
 *	\param u8 *v_latch_int_u8 : Pointer holding the value of latch interrupt
 *	v_latch_intr_u8 ->
 *	0x00	-> non-latched
 *	0x01	-> 500ms
 *	0x02	-> 2s
 *	0x03	-> 8s
 *	0x04	-> non-latched
 *	0x05	-> 500 micro sec
 *	0x06	-> 12.5ms
 *	0x07	-> 50ms
 *	0x08	-> 250ms
 *	0x09	-> 1s
 *	0x0A	-> 4s
 *	0x0B	-> latched
 *	0x0C	-> 250 micro sec
 *	0x0D	-> 1ms
 *	0x0E	-> 25ms
 *	0x0F	-> latched
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_latch_intr(u8 *v_latch_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the Latch interrupt
 *	in the register 0x21 bit from 0 to 3
 *
 *
 *
 *	\param u8 v_latch_intr_u8 : the value of latch interrupt
 *	v_latch_intr_u8 ->
 *	0x00	-> non-latched
 *	0x01	-> 500ms
 *	0x02	-> 2s
 *	0x03	-> 8s
 *	0x04	-> non-latched
 *	0x05	-> 500 micro sec
 *	0x06	-> 12.5ms
 *	0x07	-> 50ms
 *	0x08	-> 250ms
 *	0x09	-> 1s
 *	0x0A	-> 4s
 *	0x0B	-> latched
 *	0x0C	-> 250 micro sec
 *	0x0D	-> 1ms
 *	0x0E	-> 25ms
 *	0x0F	-> latched
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_latch_intr(u8 v_latch_intr_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of High
 *	Hysteresis X,Y,Z in the registers 0x22,0x24 and 0x26
 *	X_AXIS - 0x22 bit 6 and 7
 *	Y_AXIS - 0x24 bit 6 and 7
 *	Z_AXIS - 0x26 bit 6 and 7
 *
 *	\param u8 v_channel_u8: The value of high Hysteresis channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 *v_highrate_hyst_u8: the pointer holding the high Hysteresis value
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_highrate_hyst(u8 v_channel_u8,
u8 *v_highrate_hyst_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of High
 *	Hysteresis X,Y,Z in the registers 0x22,0x24 and 0x26
 *	X_AXIS - 0x22 bit 6 and 7
 *	Y_AXIS - 0x24 bit 6 and 7
 *	Z_AXIS - 0x26 bit 6 and 7
 *
 *	\param u8 v_channel_u8: The value of high Hysteresis channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 v__hyst_u8:  high Hysteresis value
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_highrate_hyst(u8 v_channel_u8,
u8 v_highrate_hyst_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of High
 *	Threshold X,Y,Z in the registers 0x22, 0x24 and 0x26
 *	X_AXIS - 0x22 bit from 1 to 5
 *	Y_AXIS - 0x24 bit from 1 to 5
 *	Z_AXIS - 0x26 bit from 1 to 5
 *
 *	\param u8 v_channel_u8 : The value of high threshold channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 *v__thres_u8: the pointer holding the high threshold value
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_highrate_thres(u8 v_channel_u8,
u8 *v_highrate_thres_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of High
 *	Threshold X,Y,Z in the registers 0x22, 0x24 and 0x26
 *	X_AXIS - 0x22 bit from 1 to 5
 *	Y_AXIS - 0x24 bit from 1 to 5
 *	Z_AXIS - 0x26 bit from 1 to 5
 *
 *	\param u8 v_channel_u8 : The value of high threshold channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 v_highrate_thres_u8: the high threshold value
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_highrate_thres(u8 v_channel_u8,
u8 v_highrate_thres_u8);
/*****************************************************************************
 * Description: *//**brief This API is used to get the status of High Enable
 * Channel X,Y,Z in the registers 0x22, 0x24 and 0x26
 *	X_AXIS - 0x22 bit 0
 *	Y_AXIS - 0x24 bit 0
 *	Z_AXIS - 0x26 bit 0
 *
 *  \param u8 v_channel_u8 : The value of high enable channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 *v__enable_u8: pointer holding the high enable
 *	v__enable_u8 :
 *	Enable  -> 1
 *	disable -> 0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_highrate_enable_axis(u8 v_channel_u8,
u8 *v__enable_u8);
/*****************************************************************************
 * Description: *//**brief This API is used to set the value of High Enable
 * Channel X,Y,Z in the registers 0x22, 0x24 and 0x26
 *	X_AXIS - 0x22 bit 0
 *	Y_AXIS - 0x24 bit 0
 *	Z_AXIS - 0x26 bit 0
 *
 *  \param u8 v_channel_u8 : The value of high enable channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 v__enable_u8: the value of high enable
 *	v__enable_u8 :
 *	Enable  -> 1
 *	disable -> 0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_highrate_enable_axis(u8 v_channel_u8,
u8 v__enable_u8);
/*****************************************************************************
 * Description: *//**brief This API is used to get the status
 * of High duration X,Y,Z in the registers 0x23, 0x25 and 0x27
 *	X_AXIS - 0x23 bit form 0 to 7
 *	Y_AXIS - 0x25 bit form 0 to 7
 *	Z_AXIS - 0x27 bit form 0 to 7
 *
 *
 *
 *	\param u8 v_channel_u8: The value of High Duration channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 *v__durn_axis_u8: pointer holding the high duration value
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_highrate_durn_axis(u8 v_channel_u8,
u8 *v_highrate_durn_axis_u8);
/*****************************************************************************
 * Description: *//**brief This API is used to set the value
 * of High duration X,Y,Z in the registers 0x23, 0x25 and 0x27
 *	X_AXIS - 0x23 bit form 0 to 7
 *	Y_AXIS - 0x25 bit form 0 to 7
 *	Z_AXIS - 0x27 bit form 0 to 7
 *
 *
 *
 *	\param u8 v_channel_u8: The value of High Duration channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 v_highrate_durn_axis_u8: the high duration value
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_highrate_durn_axis(u8 v_channel_u8,
u8 v_highrate_durn_axis_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get Slow Offset Threshold
 *	status in the register 0x31 bit 6 and 7
 *
 *
 *
 *	\param u8 *v_offset_thres_u8 : Pointer holding the offset Threshold
 *	v_offset_thres_u8 ->
 *	0x00 -> 0.1 degree/sec
 *	0x01 -> 0.2 degree/sec
 *	0x02 -> 0.5 degree/sec
 *	0x03 -> 1 degree/sec *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_slow_offset_thres(
u8 *v_offset_thres_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set Slow Offset Threshold
 *	value in the register 0x31 bit 6 and 7
 *	offset_th ->
 *	0x00 -> 0.1 degree/sec
 *	0x01 -> 0.2 degree/sec
 *	0x02 -> 0.5 degree/sec
 *	0x03 -> 1 degree/sec
 *
 *
 *	\param u8 v_offset_thres_u8 : The value of Offset Threshold
 *
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_slow_offset_thres(u8 v_offset_thres_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get Slow Offset duration
 *	status in the register 0x31 bit 4,5 and 6
 *
 *
 *
 *	\param u8 *v_offset_durn_u8 : Pointer holding the Slow Offset duration
 *	v_offset_durn_u8 ->
 *	0x00 -> 40ms
 *	0x01 -> 80ms
 *	0x02 -> 160ms
 *	0x03 -> 320ms
 *	0x04 -> 640ms
 *	0x05 -> 1280ms
 *	0x06 -> unused
 *	0x07 -> unused
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_slow_offset_durn(
u8 *v_offset_durn_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set Slow Offset duration
 *	value in the register 0x31 bit 4,5 and 6
 *
 *
 *
 *	\param u8 v_offset_durn_u8 : the value of Slow Offset duration
 *	v_offset_durn_u8 ->
 *	0x00 -> 40ms
 *	0x01 -> 80ms
 *	0x02 -> 160ms
 *	0x03 -> 320ms
 *	0x04 -> 640ms
 *	0x05 -> 1280ms
 *	0x06 -> unused
 *	0x07 -> unused
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_slow_offset_durn(
u8 v_offset_durn_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get Slow Offset Enable channel
 *	X,Y,Z in the register 0x31
 *	X_AXIS -> bit 0
 *	Y_AXIS -> bit 1
 *	Z_AXIS -> bit 2
 *
 *
 *	\param the u8 v_channel_u8: The value of slow offset channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 *v_slow_offset_u8: pointer holding the slow offset value
 *	v_slow_offset_u8 :
 *	Enable  -> 1
 *	disable -> 0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_slow_offset_enable_axis(
u8 v_channel_u8, u8 *v_slow_offset_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set Slow Offset Enable
 *	v_axis_u8 X,Y,Z in the register 0x31
 *	X_AXIS -> bit 0
 *	Y_AXIS -> bit 1
 *	Z_AXIS -> bit 2
 *
 *
 *	\param the u8 v_channel_u8: The value of slow offset channel number
 *	v_channel_u8 :
 *	BMG160_X_AXIS -> 0
 *	BMG160_Y_AXIS -> 1
 *	BMG160_Z_AXIS -> 2
 *	u8 v_slow_offset_u8: the slow offset value
 *	v_slow_offset_u8 :
 *	Enable  -> 1
 *	disable -> 0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_slow_offset_enable_axis(
u8 v_channel_u8, u8 v_slow_offset_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get
 *	Fast Offset WordLength and Auto Offset WordLength in the register 0x32
 *	fast_offset_wordlength -> bit 4 and 5
 *	auto_offset_wordlength -> bit 6 and 7
 *
 *
 *
 *	\param the u8 v_channel_u8: The value of WordLengthchannel number
 *	v_channel_u8 :
 *	BMG160_AUTO_OFFSET_WL -> 0
 *	BMG160_FAST_OFFSET_WL -> 1
 *	u8 *v_offset_word_length_u8: The pointer holding the offset word length
 *	word length ->
 *	0x00 -> 32 samples
 *	0x01 -> 64 samples
 *	0x02 -> 128 samples
 *	0x03 -> 256 samples
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_offset_word_length(u8 v_channel_u8,
u8 *v_offset_word_length_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of
 *	Fast Offset WordLength and Auto Offset WordLength in the register 0x32
 *	fast_offset_wordlength -> bit 4 and 5
 *	auto_offset_wordlength -> bit 6 and 7
 *
 *
 *
 *	\param the u8 v_channel_u8: The value of WordLengthchannel number
 *	v_channel_u8 :
 *	BMG160_AUTO_OFFSET_WORD_LENGHTH -> 0
 *	BMG160_FAST_OFFSET_WORD_LENGHTH -> 1
 *	u8 v_offset_word_length_u8: The value of offset word length
 *	word length ->
 *	0x00 -> 32 samples
 *	0x01 -> 64 samples
 *	0x02 -> 128 samples
 *	0x03 -> 256 samples
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_offset_word_length(
u8 v_channel_u8, u8 v_offset_word_length_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to enable fast offset
 *	in the register 0x32 bit 3 it is a write only register
 *
 *
 *
 *	\param None
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_enable_fast_offset(void);
/*****************************************************************************
 *	Description: *//**brief This API read the Fast offset enable
 *	v_axis_u8(X,Y and Z) in the register 0x32h
 *	X_AXIS -> bit 0
 *	Y_AXIS -> bit 1
 *	Z_AXIS -> bit 2
 *
 *
 *
 *  \param u8 v_fast_offset_u8: Pointer holding the fast offset
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fast_offset_enable_axis(
u8 *v_fast_offset_u8);
/*****************************************************************************
 *	Description: *//**brief This API used to set
 *	the Fast offset enable v_axis_u8 in the register 0x32h
 *	X_AXIS -> bit 0
 *	Y_AXIS -> bit 1
 *	Z_AXIS -> bit 2
 *
 *
 *
 *	\param   u8 v_channel_u8: The value of output type status channel number
 *	v_channel_u8 --> BMG160_X_AXIS 0
 *				BMG160_Y_AXIS 1
 *				BMG160_Z_AXIS 2
 *
 *	u8 v_fast_offset_u8: The value of Fast offset
 *	v_fast_offset_u8 --> 0 - Disable
 *					1 - Enable
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fast_offset_enable_axis(
u8 v_channel_u8, u8 v_fast_offset_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of nvm program
 *	remain in the register 0x33 bit from 4 to 7
 *
 *
 *
 *
 *	\param u8 *v_nvm_remain_u8: The pointer holding the nvm program
 *
 *
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_nvm_remain(u8 *v_nvm_remain_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the status of nvm load
 *	in the register 0x33 bit 3
 *
 *
 *
 *	\param u8 v_nvm_load_u8: The value of nvm load
 *	v_nvm_load_u8 ->
 *	1 -> load offset value from NVM
 *	0 -> no action
 *
 *
 *
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_nvm_load(u8 v_nvm_load_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of nvm
 *	ready in the register 0x33 bit 1
 *
 *
 *
 *
 *	\param u8 *v_nvm_rdy_u8: The pointer holding the nvmprogram
 *	v_nvm_rdy_u8->   1 -> program seq finished
 *              0 -> program seq in progress
 *
 *
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_nvm_rdy(u8 *v_nvm_rdy_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set
 *	the status of nvm program trigger in the register 0x33 bit 0
 *
 *
 *
 *
 *	\param u8 nvm_prog_trig: The value of nvm program
 *	nvm_prog_trig ->
 *	1 -> trig program seq (wo)
 *	0 -> No Action
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_nvm_prog_trig(u8 nvm_prog_trig);
/*****************************************************************************
 *	Description: *//**brief This API is used to get
 *	the status of nvm program trigger in the register 0x33 bit 0
 *
 *
 *
 *
 *	\param u8 *nvm_prog_mode: pointer holding the value of nvm program
 *	nvm_prog_mode ->
 *	1 -> trig program seq (wo)
 *	0 -> No Action
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_nvm_prog_mode(u8 *nvm_prog_mode);
/******************************************************************************
 *	Description: *//**brief This API is used to set the status of nvmprogram
 *	mode in the register 0x33 bit 0
 *
 *
 *
 *
 *  \param u8 nvm_prog_mode: The value of nvmprogram
 *                   1 -> Enable program mode
 *                   0 -> Disable program mode
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_nvm_prog_mode(u8 nvm_prog_mode);
/*****************************************************************************
 *	Description: *//**brief This API is used to get
 *	the status of i2c wdt select and enable in the register 0x34
 *	i2c_wdt_select -> bit 1
 *	i2c_wdt_enable -> bit 2
 *
 *
 *	\param  u8 v_channel_u8: The value of i2c wdt channel number
 *	channel->
 *	BMG160_I2C_WDT_EN               1
 *	BMG160_I2C_WDT_SELECT                0
 *	u8 *v_i2c_wdt_u8: Pointer holding the i2c wdt
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_i2c_wdt(u8 v_channel_u8,
u8 *v_i2c_wdt_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set
 *	the value of i2c wdt select and enable in the register 0x34
 *	i2c_wdt_select -> bit 1
 *	i2c_wdt_enable -> bit 2
 *
 *
 *	\param  u8 v_channel_u8: The value of i2c wdt channel number
 *	v_channel_u8->
 *	BMG160_I2C_WDT_EN               1
 *	BMG160_I2C_WDT_SELECT           0
 *	u8 v_i2c_wdt_u8: the value of i2c wdt
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_i2c_wdt(u8 v_channel_u8,
u8 v_i2c_wdt_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to get the status of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  \param  u8 *v_spi3_u8 : Pointer holding the spi3
 *
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_spi3(u8 *v_spi3_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of spi3
 *	in the register 0x34 bit 0
 *
 *
 *
 *  \param u8 v_spi3_u8: The value of spi3
 *
 *
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_spi3(u8 v_spi3_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to get the status of FIFO tag
 *	in the register 0x3D bit 7
 *
 *
 *
 *	\param  u8 *v_fifo_tag_u8 : Pointer holding the FIFO tag
 *	v_fifo_tag_u8 ->
 *	0 -> disable
 *	1 -> enable
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_tag(u8 *v_fifo_tag_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to set the value of FIFO tag
 *	in the register 0x3D bit 7
 *
 *
 *
 *	\param  u8 v_fifo_tag_u8 : the value of FIFO tag
 *	v_fifo_tag_u8 ->
 *	0 -> disable
 *	1 -> enable
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_tag(u8 v_fifo_tag_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get Water Mark Level
 *	in the register 0x3D bit from 0 to 6
 *
 *
 *
 *	\param u8 *v_fifo_wml_u8 : Pointer holding the water_mark_level
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_wm_level(
u8 *v_fifo_wml_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set Water Mark Level
 *	in the register 0x3D bit from 0 to 6
 *
 *
 *
 *	\param u8 water_mark_level : the value of water_mark_level
 *
 *
 *
 *	\return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_wm_level(
u8 v_fifo_wml_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the value of offset
 *	X, Y and Z in the registers 0x36, 0x37, 0x38, 0x39 and 0x3A
 *	the offset is a 12bit value
 *	X_AXIS ->
 *		bit 0 and 1 is available in the register 0x3A bit 2 and 3
 *		bit 2 and 3 is available in the register 0x36 bit 6 and 7
 *		bit 4 to 11 is available in the register 0x37 bit 0 to 7
 *	Y_AXIS ->
 *		bit 0 is available in the register 0x3A bit 1
 *		bit 1,2 and 3 is available in the register 0x36 bit 3,4 and 5
 *		bit 4 to 11 is available in the register 0x38 bit 0 to 7
 *	Z_AXIS ->
 *		bit 0 is available in the register 0x3A bit 0
 *		bit 1,2 and 3 is available in the register 0x36 bit 0,1 and 3
 *		bit 4 to 11 is available in the register 0x39 bit 0 to 7
 *
 *  \param  u8 v_axis_u8 : The value of offset v_axis_u8
 *                   v_axis_u8 ->
 *                   BMG160_X_AXIS     ->      0
 *                   BMG160_Y_AXIS     ->      1
 *                   BMG160_Z_AXIS     ->      2
 *   u8 *v_offset_s16: The pointer holding the v_offset_s16 value
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_offset(u8 v_axis_u8,
s16 *v_offset_s16);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the value of offset
 *	X, Y and Z in the registers 0x36, 0x37, 0x38, 0x39 and 0x3A
 *	the offset is a 12bit value
 *	X_AXIS ->
 *		bit 0 and 1 is available in the register 0x3A bit 2 and 3
 *		bit 2 and 3 is available in the register 0x36 bit 6 and 7
 *		bit 4 to 11 is available in the register 0x37 bit 0 to 7
 *	Y_AXIS ->
 *		bit 0 is available in the register 0x3A bit 1
 *		bit 1,2 and 3 is available in the register 0x36 bit 3,4 and 5
 *		bit 4 to 11 is available in the register 0x38 bit 0 to 7
 *
 *	Z_AXIS ->
 *		bit 0 is available in the register 0x3A bit 0
 *		bit 1,2 and 3 is available in the register 0x36 bit 0,1 and 3
 *		bit 4 to 11 is available in the register 0x39 bit 0 to 7
 *
 *  \param  u8 v_axis_u8 : The value of offset
 *                   v_axis_u8 ->
 *                   BMG160_X_AXIS     ->      0
 *                   BMG160_Y_AXIS     ->      1
 *                   BMG160_Z_AXIS     ->      2
 *   u8 *v_offset_s16: The pointer holding the offset value
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_offset(
u8 v_axis_u8, s16 v_offset_s16);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of general
 *	purpose register in the register 0x3A and 0x3B
 *
 *
 *
 *
 *  \param Pointer holding the u8 param
 *          u8 *v_gp_u8
 *               v_param_u8 ->
 *              BMG160_GP0                      0
 *              BMG160_GP0                      1
 *
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_gp(u8 v_param_u8,
u8 *v_gp_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the status of general
 *	purpose register in the register 0x3A and 0x3B
 *
 *
 *
 *
 *  \param Pointer holding the u8 param
 *          u8 v_gp_u8: the value of gp
 *               v_param_u8 ->
 *              BMG160_GP0                      0
 *              BMG160_GP0                      1
 *
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_gp(u8 v_param_u8,
u8 v_gp_u8);
/*****************************************************************************
 * Description: *//**brief Reads FIFI data from location 3Fh
 *
 *
 *
 *
 *  \param
 *      u8 *v_fifo_data_u8 : Address of FIFO data bits
 *
 *
 *
 *
 *  \return result of communication routines
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_FIFO_data_reg(u8 *v_fifo_data_u8);
/*****************************************************************************
 *	Description: *//** this api is used to read the fifo status
 *	of frame_counter and overrun in the register 0Eh
 *	frame_counter bit	->	from 0 to 6
 *	overrun bit	->	7
 *
 *
 *
 *	\param
 *      u8 *v_fifo_stat_u8 : pointer holding the fifo status
 *	of frame_counter and overrun
 *
 *
 *  \return
 *      Result of bus communication function
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
 *****************************************************************************/

BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_stat_reg(
u8 *v_fifo_stat_u8);
/*****************************************************************************
 *	Description: *//**brief this API is used to get the fifo frame counter
 *	in the register 0x0E bit 0 to 6
 *
 *
 *
 *	\param u8 *v_fifo_frame_count_u8: Pointer holding the FIFO frame counter
 *
 *
 *	\return Result of bus communication function
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
 *****************************************************************************/

BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_frame_count(
u8 *v_fifo_frame_count_u8);
/*****************************************************************************
 *	Description: *//**brief this API is used to get the fifo over run
 *	in the register 0x0E bit 7
 *
 *
 *
 *	\param u8 *v_fifo_overrun_u8: Pointer holding the FIFO over run
 *
 *
 *	\return Result of bus communication function
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
 *****************************************************************************/

BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_overrun(
u8 *v_fifo_overrun_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of fifo mode
 *	in the register 0x3E bit 6 and 7
 *
 *
 *
 *	\param u8 *v_fifo_mode_u8 : Pointer holding the FIFO mode
 *	v_fifo_mode_u8
 *	0 --> Bypass
 *	1 --> FIFO
 *	2 --> Stream
 *	3 --> Reserved
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_mode(u8 *v_fifo_mode_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of fifo mode
 *	in the register 0x3E bit 6 and 7
 *
 *
 *
 *	\param u8 v_fifo_mode_u8 :the value of FIFO mode
 *	v_fifo_mode_u8
 *	0 --> Bypass
 *	1 --> FIFO
 *	2 --> Stream
 *	3 --> Reserved
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_mode(u8 v_fifo_mode_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the status of fifo
 *	data select in the register 0x3E bit 0 and 1
 *
 *
 *	\param u8 *v_fifo_data_select_u8 : Pointer holding the data select
 *	v_fifo_data_select_u8 --> [0:3]
 *	0 --> X,Y and Z (DEFAULT)
 *	1 --> X only
 *	2 --> Y only
 *	3 --> Z only
 *
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_data_select(
u8 *v_fifo_data_select_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the value of fifo
 *	data select in the register 0x3E bit 0 and 1
 *
 *
 *	\param u8 *v_fifo_data_select_u8 : Pointer holding the fifo data_select
 *	v_fifo_data_select_u8 --> [0:3]
 *	0 --> X,Y and Z (DEFAULT)
 *	1 --> X only
 *	2 --> Y only
 *	3 --> Z only
 *
 *
 *
 *
 *
 *  \return results of bus communication function
 *
 *
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_data_select(
u8 v_fifo_data_select_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to get the operating modes of the
 *	sensor in the registers 0x11 and 0x12
 *
 *
 *
 *
 *  \param u8 * v_power_mode_u8 :Pointer holding the operating Mode
 *              v_power_mode_u8->   0 -> NORMAL
 *                       1 -> SUSPEND
 *                       2 -> DEEP SUSPEND
 *						 3 -> FAST POWERUP
 *						 4 -> ADVANCED POWERSAVING
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_power_mode(u8 *v_power_mode_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to set the operating modes of the
 *	sensor in the registers 0x11 and 0x12
 *
 *
 *
 *
 *  \param u8 power_mode :the value of operating Mode
 *              power_mode->   0 -> NORMAL
 *                       1 -> SUSPEND
 *                       2 -> DEEP SUSPEND
 *						 3 -> FAST POWERUP
 *						 4 -> ADVANCED POWERSAVING
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_power_mode(u8 v_power_mode_u8);
/*****************************************************************************
 *	Description: *//**brief This API is used to to do selftest to sensor
 *	sensor in the register 0x3C
 *
 *
 *
 *
 *  \param u8 *v_result_u8: Pointer holding the selftest value
 *
 *
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_seleftest(u8 *v_result_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to get the auto sleep duration
 *	in the register 0x12 bit 0 to 2
 *
 *
 *
 *  \param u8 *durn : Pointer holding the auto sleep duration
 *	durn ->
 *	C_BMG160_NO_FILTER_U8X			0
 *	C_BMG160_BW_230HZ_U8X			1
 *	C_BMG160_BW_116HZ_u8X			2
 *	C_BMG160_BW_47HZ_u8X			3
 *	C_BMG160_BW_23HZ_u8X			4
 *	C_BMG160_BW_12HZ_u8X			5
 *	C_BMG160_BW_64HZ_u8X			6
 *	C_BMG160_BW_32HZ_u8X			7
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_auto_sleep_durn(u8 *durn);
/*****************************************************************************
 *	Description: *//**brief  This API is used to set the auto sleep duration
 *	in the register 0x12 bit 0 to 2
 *
 *
 *
 *  \param u8 dur : Pointer holding the auto sleep duration
 *	dur ->
 *	C_BMG160_NO_FILTER_U8X			0
 *	C_BMG160_BW_230HZ_U8X			1
 *	C_BMG160_BW_116HZ_u8X			2
 *	C_BMG160_BW_47HZ_u8X			3
 *	C_BMG160_BW_23HZ_u8X			4
 *	C_BMG160_BW_12HZ_u8X			5
 *	C_BMG160_BW_64HZ_u8X			6
 *	C_BMG160_BW_32HZ_u8X			7
 *
 *	u8 bandwidth:
 *			Value to be written passed as a parameter
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_auto_sleep_durn(u8 durn,
u8 v_bw_u8);
/*****************************************************************************
 *	Description: *//**brief  This API is used to get the sleep duration
 *	in the register 0x11 bit 1 to 3
 *
 *
 *
 *	\param u8 *durn : Pointer holding the sleep duration
 *	durn ->
 *	0x00 -> 2ms
 *	0x01 -> 4ms
 *	0x02 -> 5ms
 *	0x03 -> 8ms
 *	0x04 -> 10ms
 *	0x05 -> 15ms
 *	0x06 -> 18ms
 *	0x07 -> 20ms
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_sleep_durn(u8 *durn);
/*****************************************************************************
 *	Description: *//**brief  This API is used to set the sleep duration
 *	in the register 0x11 bit 1 to 3
 *
 *
 *
 *	\param u8 duration : the value of sleep duration
 *	duration ->
 *	0x00 -> 2ms
 *	0x01 -> 4ms
 *	0x02 -> 5ms
 *	0x03 -> 8ms
 *	0x04 -> 10ms
 *	0x05 -> 15ms
 *	0x06 -> 18ms
 *	0x07 -> 20ms
 *
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_set_sleep_durn(u8 durn);

/*****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the fast offset(int2_fast_offset) and auto offset(int2_auto_offset)
 *	of interrupt2 in the register 0x18
 *	int2_fast_offset -> bit 6
 *	int2_auto_offset -> bit 4
 *
 *	\param u8 v_axis_u8: The value of fast/auto offset interrupts
 *	v_axis_u8 -->	BMG160_FAST_OFFSET    ->    1
 *				BMG160_AUTO_OFFSET    ->    2
 *
 *  u8 *v_intr1_offset_u8: Pointer holding the value of fast/auto offset
 *	Interrupt enable	->	1
 *	Interrupt disable	->	0
 *
 *  \return results of bus communication function
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
 *****************************************************************************/
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr2_offset(u8 v_axis_u8,
u8 *v_intr2_offset_u8);


#endif
