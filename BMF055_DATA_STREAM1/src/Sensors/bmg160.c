/*
****************************************************************************
* Copyright (C) 2010 - 2014 Bosch Sensortec GmbH
*
* bmg160.c
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
/*! file bmg160.c
    brief Driver for BMG160 */
#include "bmg160.h"
static struct bmg160_t *p_bmg160;


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
BMG160_RETURN_FUNCTION_TYPE bmg160_init(struct bmg160_t *bmg160)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	p_bmg160 = bmg160;
	/*Read CHIP_ID */
	comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
	 BMG160_CHIP_ID_ADDR, &v_data_u8, 1);
	p_bmg160->chip_id = v_data_u8;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_X(s16 *v_data_x_s16)
{
	BMG160_RETURN_FUNCTION_TYPE comres  = C_BMG160_ZERO_U8X;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RATE_X_LSB_VALUEX__REG, v_data_u8, 2);
		v_data_u8[0] = BMG160_GET_BITSLICE(v_data_u8[0],
		BMG160_RATE_X_LSB_VALUEX);
		*v_data_x_s16 = (s16)
		((((s32)((s8)v_data_u8[1])) <<
		BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[0]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_Y(s16 *v_data_y_s16)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RATE_Y_LSB_VALUEY__REG, v_data_u8, 2);
		v_data_u8[0] = BMG160_GET_BITSLICE(v_data_u8[0],
		BMG160_RATE_Y_LSB_VALUEY);
		*v_data_y_s16 = (s16)
		((((s32)((s8)v_data_u8[1]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[0]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_Z(s16 *v_data_z_s16)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8[2] = {0, 0};
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		 BMG160_RATE_Z_LSB_VALUEZ__REG, v_data_u8, 2);
		v_data_u8[0] = BMG160_GET_BITSLICE(v_data_u8[0],
		BMG160_RATE_Z_LSB_VALUEZ);
		*v_data_z_s16 = (s16)
		((((s32)((s8)v_data_u8[1]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[0]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_XYZ(struct bmg160_data_t *data)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8[6] = {0, 0, 0, 0, 0, 0};
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		 BMG160_RATE_X_LSB_VALUEX__REG, v_data_u8, 6);
		/* Data X */
		v_data_u8[0] =
		BMG160_GET_BITSLICE(v_data_u8[0], BMG160_RATE_X_LSB_VALUEX);
		data->datax = (s16)
		((((s32)((s8)v_data_u8[1]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[0]));
		/* Data Y */
		v_data_u8[2] = BMG160_GET_BITSLICE(v_data_u8[2],
		BMG160_RATE_Y_LSB_VALUEY);
		data->datay = (s16)
		((((s32)((s8)v_data_u8[3]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[2]));
		/* Data Z */
		v_data_u8[4] = BMG160_GET_BITSLICE(v_data_u8[4],
		BMG160_RATE_Z_LSB_VALUEZ);
		data->dataz = (s16)
		((((s32)((s8)v_data_u8[5]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[4]));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_XYZI(struct bmg160_data_t *data)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		 BMG160_RATE_X_LSB_VALUEX__REG, v_data_u8, 12);
		/* Data X */
		v_data_u8[0] = BMG160_GET_BITSLICE(v_data_u8[0],
		BMG160_RATE_X_LSB_VALUEX);
		data->datax = (s16)
		((((s32)((s8)v_data_u8[1]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[0]));
		/* Data Y */
		v_data_u8[2] = BMG160_GET_BITSLICE(v_data_u8[2],
		BMG160_RATE_Y_LSB_VALUEY);
		data->datay = (s16)
		((((s32)((s8)v_data_u8[3]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[2]));
		/* Data Z */
		v_data_u8[4] = BMG160_GET_BITSLICE(v_data_u8[4],
		BMG160_RATE_Z_LSB_VALUEZ);
		data->dataz = (s16)
		((((s32)((s8)v_data_u8[5]))
		<< BMG160_SHIFT_EIGHT_POSITION) | (v_data_u8[4]));
		data->intstatus[0] = v_data_u8[7];
		data->intstatus[1] = v_data_u8[8];
		data->intstatus[2] = v_data_u8[9];
		data->intstatus[3] = v_data_u8[10];
		data->intstatus[4] = v_data_u8[11];
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_temp(s8 *v_temp_s8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_TEMP_ADDR, &v_data_u8, 1);
		*v_temp_s8 = v_data_u8;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*****************************************************************************
 * Description: *//**brief This API reads the data from the given register
 *
 *
 *
 *
 *\param u8 v_addr_u8, u8 *v_data_u8 u8 len
 *           v_addr_u8 -> Address of the register
 *           v_data_u8 -> address of the variable, read value will be
 *			  kept
 *           v_len_u8 -> No of byte to be read.
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
u8 *v_data_u8, u8 v_len_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr, v_addr_u8, v_data_u8, v_len_u8);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_data_u8, s32 v_len_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BURST_READ_FUNC(p_bmg160->dev_addr,
		v_addr_u8, v_data_u8, v_len_u8);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_data_u8, u8 v_len_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_WRITE_FUNC
		(p_bmg160->dev_addr, v_addr_u8, v_data_u8, v_len_u8);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_stat0_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr,
		BMG160_INTR_STAT_ZERO__REG, &v_data_u8, 1);
		*v_stat0_data_u8 =
		BMG160_GET_BITSLICE(v_data_u8, BMG160_INTR_STAT_ZERO);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_stat1_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr, BMG160_INTR_STAT_ONE__REG,
		&v_data_u8, 1);
		*v_stat1_data_u8 =
		BMG160_GET_BITSLICE(v_data_u8, BMG160_INTR_STAT_ONE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_stat2_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr,
		BMG160_INTR_STAT_TWO__REG, &v_data_u8, 1);
		*v_stat2_data_u8 =
		BMG160_GET_BITSLICE(v_data_u8, BMG160_INTR_STAT_TWO);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_stat3_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr,
		BMG160_INTR_STAT_THREE__REG, &v_data_u8, 1);
		*v_stat3_data_u8 =
		BMG160_GET_BITSLICE(v_data_u8, BMG160_INTR_STAT_THREE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_range_reg(u8 *v_range_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr,
		BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8, 1);
		*v_range_u8 =
		BMG160_GET_BITSLICE(v_data_u8, BMG160_RANGE_ADDR_RANGE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_range_reg(u8 v_range_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_range_u8 < C_BMG160_FIVE_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_RANGE_ADDR_RANGE,
			v_range_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_RANGE_ADDR_RANGE__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_bw(u8 *v_bw_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr, BMG160_BW_ADDR__REG, &v_data_u8, 1);
		*v_bw_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_BW_ADDR);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_bw(u8 v_bw_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	u8 v_mode_u8r  = C_BMG160_ZERO_U8X;
	u8 v_auto_sleep_dur = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_bw_u8 < C_BMG160_EIGHT_U8X) {
			bmg160_get_power_mode(&v_mode_u8r);
			if (v_mode_u8r == BMG160_MODE_ADVANCEDPOWERSAVING) {
				bmg160_get_auto_sleep_durn(&v_auto_sleep_dur);
				bmg160_set_auto_sleep_durn(v_auto_sleep_dur,
				v_bw_u8);
			}
				comres = p_bmg160->BMG160_BUS_READ_FUNC
				(p_bmg160->dev_addr,
				BMG160_BW_ADDR__REG, &v_data_u8, 1);
				v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
				BMG160_BW_ADDR, v_bw_u8);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
				BMG160_BW_ADDR__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_pwu_ext_tri_select_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__REG, &v_data_u8, 1);
		*v_pwu_ext_tri_select_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_pwu_ext_tri_select_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT, v_pwu_ext_tri_select_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR_EXT_TRI_SELECT__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_high_bw(u8 *v_high_bw_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_HIGH_BW__REG, &v_data_u8, 1);
		*v_high_bw_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_HIGH_BW);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_high_bw(u8 v_high_bw_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_high_bw_u8 < C_BMG160_TWO_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGH_BW__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGH_BW, v_high_bw_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGH_BW__REG,
			&v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_shadow_dis(u8 *v_shadow_dis_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_SHADOW_DIS__REG, &v_data_u8, 1);
		*v_shadow_dis_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_SHADOW_DIS);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_shadow_dis(u8 v_shadow_dis_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_shadow_dis_u8 < C_BMG160_TWO_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_SHADOW_DIS__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_SHADOW_DIS, v_shadow_dis_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_SHADOW_DIS__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_soft_rst(void)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_soft_rst_u8  = C_BMG160_ZERO_U8X;
	v_soft_rst_u8 = 0xB6;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_BGW_SOFT_RST_ADDR, &v_soft_rst_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_data_enable(u8 *v_data_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_DATA__REG, &v_data_u8, 1);
		*v_data_enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_ENABLE0_DATA);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_data_enable(u8 v_data_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_DATA__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_ENABLE0_DATA, v_data_enable_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC
		(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_DATA__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_enable(u8 *v_fifo_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_FIFO__REG, &v_data_u8, 1);
		*v_fifo_enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_ENABLE0_FIFO);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_enable(u8 v_fifo_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_fifo_enable_u8 < C_BMG160_TWO_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE0_FIFO__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE0_FIFO, v_fifo_enable_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE0_FIFO__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_offset_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_AUTO_OFFSET__REG, &v_data_u8, 1);
		*v_offset_enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_ENABLE0_AUTO_OFFSET);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_auto_offset_enable(u8 v_offset_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_AUTO_OFFSET__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_ENABLE0_AUTO_OFFSET, v_offset_enable_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ENABLE0_AUTO_OFFSET__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_output_type_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_INTR1:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			 BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__REG,
			 &v_data_u8, 1);
			*v_intr_output_type_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE);
			break;
		case BMG160_INTR2:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			*v_intr_output_type_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_output_type_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_INTR1:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE,
			v_intr_output_type_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT1_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			break;
		case BMG160_INTR2:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE,
			v_intr_output_type_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT2_OUTPUT_TYPE__REG,
			&v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_level_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_INTR1:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT1_LEVEL__REG, &v_data_u8, 1);
			*v_intr_level_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT1_LEVEL);
			break;
		case BMG160_INTR2:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT2_LEVEL__REG, &v_data_u8, 1);
			*v_intr_level_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT2_LEVEL);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_level_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_INTR1:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT1_LEVEL__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT1_LEVEL, v_intr_level_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT1_LEVEL__REG, &v_data_u8, 1);
			break;
		case BMG160_INTR2:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT2_LEVEL__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ENABLE1_IT2_LEVEL, v_intr_level_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ENABLE1_IT2_LEVEL__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr1_highrate(u8 *v_intr1__u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__REG, &v_data_u8, 1);
		*v_intr1__u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr1__u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE, v_intr1__u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_ZERO_INTR1_HIGHRATE__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_int1r_any_motion_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__REG, &v_data_u8, 1);
		*v_int1r_any_motion_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_int1r_any_motion_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION, v_int1r_any_motion_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_ZERO_INTR1_ANY_MOTION__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_INTR1_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_DATA__REG, &v_data_u8, 1);
			*v_intr_data_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR1_DATA);
			break;
		case BMG160_INTR2_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_DATA__REG, &v_data_u8, 1);
			*v_intr_data_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR2_DATA);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	}   else {
			switch (v_axis_u8) {
			case BMG160_INTR1_DATA:
				comres = p_bmg160->BMG160_BUS_READ_FUNC
				(p_bmg160->dev_addr,
				BMG160_MAP_ONE_INTR1_DATA__REG, &v_data_u8, 1);
				v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
				BMG160_MAP_ONE_INTR1_DATA, v_intr_data_u8);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
				BMG160_MAP_ONE_INTR1_DATA__REG, &v_data_u8, 1);
				break;
			case BMG160_INTR2_DATA:
				comres = p_bmg160->BMG160_BUS_READ_FUNC
				(p_bmg160->dev_addr,
				BMG160_MAP_ONE_INTR2_DATA__REG, &v_data_u8, 1);
				v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
				BMG160_MAP_ONE_INTR2_DATA, v_intr_data_u8);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
				BMG160_MAP_ONE_INTR2_DATA__REG, &v_data_u8, 1);
				break;
			default:
				comres = E_BMG160_OUT_OF_RANGE;
				break;
			}
		}
		return comres;
	}

/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr2_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_FAST_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_FAST_OFFSET__REG, &v_data_u8, 1);
			*v_intr2_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR2_FAST_OFFSET);
			break;
		case BMG160_AUTO_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_AUTO_OFFSET__REG, &v_data_u8, 1);
			*v_intr2_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR2_AUTO_OFFSET);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr2_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_FAST_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_FAST_OFFSET__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR2_FAST_OFFSET, v_intr2_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_FAST_OFFSET__REG, &v_data_u8, 1);
			break;
		case BMG160_AUTO_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_AUTO_OFFSET__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR2_AUTO_OFFSET, v_intr2_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_AUTO_OFFSET__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr1_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_FAST_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_FAST_OFFSET__REG, &v_data_u8, 1);
			*v_intr1_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR1_FAST_OFFSET);
			break;
		case BMG160_AUTO_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_AUTO_OFFSET__REG, &v_data_u8, 1);
			*v_intr1_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR1_AUTO_OFFSET);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr1_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_FAST_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_FAST_OFFSET__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR1_FAST_OFFSET, v_intr1_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_FAST_OFFSET__REG, &v_data_u8, 1);
			break;
		case BMG160_AUTO_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_AUTO_OFFSET__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR1_AUTO_OFFSET, v_intr1_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_AUTO_OFFSET__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr2_fifo(u8 *v_intr_fifo_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_MAP_ONE_INTR2_FIFO__REG, &v_data_u8, 1);
		*v_intr_fifo_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_MAP_ONE_INTR2_FIFO);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_intr1_fifo(u8 *v_intr_fifo_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_MAP_ONE_INTR1_FIFO__REG, &v_data_u8, 1);
		*v_intr_fifo_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_MAP_ONE_INTR1_FIFO);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_fifo_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_INTR1:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_FIFO__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR1_FIFO, v_intr_fifo_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR1_FIFO__REG, &v_data_u8, 1);
			break;
		case BMG160_INTR2:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_FIFO__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MAP_ONE_INTR2_FIFO, v_intr_fifo_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_MAP_ONE_INTR2_FIFO__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr2_highrate_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_TWO_INT2_HIGHRATE__REG, &v_data_u8, 1);
		*v_intr2_highrate_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_TWO_INT2_HIGHRATE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr2_highrate_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_TWO_INT2_HIGHRATE__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_TWO_INT2_HIGHRATE, v_intr2_highrate_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_TWO_INT2_HIGHRATE__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr2_any_motion_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_TWO_INT2_ANY_MOTION__REG, &v_data_u8, 1);
		*v_intr2_any_motion_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_TWO_INT2_ANY_MOTION);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr2_any_motion_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_TWO_INT2_ANY_MOTION__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_MAP_TWO_INT2_ANY_MOTION, v_intr2_any_motion_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_MAP_TWO_INT2_ANY_MOTION__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_offset_unfilt_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_SLOW_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__REG,
			&v_data_u8, 1);
			*v_offset_unfilt_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT);
			break;
		case BMG160_FAST_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__REG,
			&v_data_u8, 1);
			*v_offset_unfilt_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_offset_unfilt_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_SLOW_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT,
			v_offset_unfilt_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_SLOW_OFFSET_UNFILT__REG,
			&v_data_u8, 1);
			break;
		case BMG160_FAST_OFFSET:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT,
			v_offset_unfilt_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ONE_ADDR_FAST_OFFSET_UNFILT__REG,
			&v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_unfilt_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_HIGHRATE_UNFILT_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__REG,
			&v_data_u8, 1);
			*v_unfilt_data_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA);
			break;
		case BMG160_ANY_MOTION_UNFILT_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__REG,
			&v_data_u8, 1);
			*v_unfilt_data_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_unfilt_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_HIGHRATE_UNFILT_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA,
			v_unfilt_data_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_HIGHRATE_UNFILT_DATA__REG,
			&v_data_u8, 1);
			break;
		case BMG160_ANY_MOTION_UNFILT_DATA:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA,
			v_unfilt_data_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_ZERO_ADDR_ANY_MOTION_UNFILT_DATA__REG,
			&v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_any_motion_thres_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__REG, &v_data_u8, 1);
		*v_any_motion_thres_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 any_motion_thres)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES, any_motion_thres);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_ONE_ADDR_ANY_MOTION_THRES__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_awake_durn(u8 *v_awake_durn_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_TWO_ADDR_AWAKE_DURN__REG, &v_data_u8, 1);
		*v_awake_durn_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_TWO_ADDR_AWAKE_DURN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_awake_durn(u8 v_awake_durn_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_TWO_ADDR_AWAKE_DURN__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_TWO_ADDR_AWAKE_DURN, v_awake_durn_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_TWO_ADDR_AWAKE_DURN__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_durn_sample_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__REG,
		&v_data_u8, 1);
		*v_durn_sample_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_durn_sample_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__REG,
		&v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE,
		v_durn_sample_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(
		p_bmg160->dev_addr,
		BMG160_INTR_TWO_ADDR_ANY_MOTION_DURN_SAMPLE__REG,
		&v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data1_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__REG,
			&v_data1_u8, 1);
			*v_data_u8 = BMG160_GET_BITSLICE(v_data1_u8,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__REG,
			&v_data1_u8, 1);
			*v_data_u8 = BMG160_GET_BITSLICE(v_data1_u8,
				BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__REG,
			&v_data1_u8, 1);
			*v_data_u8 = BMG160_GET_BITSLICE(v_data1_u8,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data1_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__REG,
			&v_data1_u8, 1);
			v_data1_u8 = BMG160_SET_BITSLICE(v_data1_u8,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X, v_data_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_X__REG,
			&v_data_u8, 1);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__REG,
			&v_data_u8, 1);
			v_data1_u8 = BMG160_SET_BITSLICE(v_data1_u8,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y, v_data_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Y__REG,
			&v_data_u8, 1);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__REG,
			&v_data_u8, 1);
			v_data1_u8 = BMG160_SET_BITSLICE(v_data1_u8,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z, v_data_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_TWO_ADDR_ANY_MOTION_ENABLE_Z__REG,
			&v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_wm_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_INTR_4_FIFO_WM_ENABLE__REG, &v_data_u8, 1);
		*v_fifo_wm_enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_INTR_4_FIFO_WM_ENABLE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_fifo_wm_enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_fifo_wm_enable_u8 < C_BMG160_TWO_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_4_FIFO_WM_ENABLE__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_INTR_4_FIFO_WM_ENABLE, v_fifo_wm_enable_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_INTR_4_FIFO_WM_ENABLE__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_rst_intr(u8 v_rst_int_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_RST_INTR__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_RST_LATCH_ADDR_RST_INTR, v_rst_int_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_RST_INTR__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_offset_rst_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_OFFSET_RST__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_RST_LATCH_ADDR_OFFSET_RST, v_offset_rst_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_OFFSET_RST__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_latch_stat_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_LATCH_STAT__REG, &v_data_u8, 1);
		*v_latch_stat_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_RST_LATCH_ADDR_LATCH_STAT);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_latch_stat_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_LATCH_STAT__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_RST_LATCH_ADDR_LATCH_STAT, v_latch_stat_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_LATCH_STAT__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_latch_intr(u8 *v_latch_intr_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_LATCH_INTR__REG, &v_data_u8, 1);
		*v_latch_intr_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_RST_LATCH_ADDR_LATCH_INTR);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_latch_intr(u8 v_latch_intr_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_LATCH_INTR__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_RST_LATCH_ADDR_LATCH_INTR, v_latch_intr_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_RST_LATCH_ADDR_LATCH_INTR__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_highrate_hyst_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_X__REG, &v_data_u8, 1);
			*v_highrate_hyst_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_HYST_X);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_Y__REG, &v_data_u8, 1);
			*v_highrate_hyst_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_HYST_Y);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_Z__REG, &v_data_u8, 1);
			*v_highrate_hyst_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_HYST_Z);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_highrate_hyst_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_X__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_HYST_X, v_highrate_hyst_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_X__REG, &v_data_u8, 1);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_Y__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_HYST_Y, v_highrate_hyst_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_Y__REG, &v_data_u8, 1);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_Z__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_HYST_Z, v_highrate_hyst_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_HYST_Z__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_highrate_thres_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_X__REG, &v_data_u8, 1);
			*v_highrate_thres_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_THRES_X);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Y__REG, &v_data_u8, 1);
			*v_highrate_thres_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_THRES_Y);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Z__REG, &v_data_u8, 1);
			*v_highrate_thres_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_THRES_Z);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_highrate_thres_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_X__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_THRES_X, v_highrate_thres_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_X__REG, &v_data_u8, 1);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Y__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_THRES_Y, v_highrate_thres_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Y__REG, &v_data_u8, 1);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Z__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_THRES_Z, v_highrate_thres_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Z__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v__enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_X__REG, &v_data_u8, 1);
			*v__enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_ENABLE_X);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_Y__REG, &v_data_u8, 1);
			*v__enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_ENABLE_Y);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_Z__REG, &v_data_u8, 1);
			*v__enable_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_ENABLE_Z);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v__enable_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_X__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_ENABLE_X, v__enable_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_X__REG, &v_data_u8, 1);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_Y__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_ENABLE_Y, v__enable_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_Y__REG, &v_data_u8, 1);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_Z__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_HIGHRATE_ENABLE_Z, v__enable_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_ENABLE_Z__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_highrate_durn_axis_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_DURN_X_ADDR, &v_data_u8, 1);
			*v_highrate_durn_axis_u8 = v_data_u8;
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Y_ADDR, &v_data_u8, 1);
			*v_highrate_durn_axis_u8 = v_data_u8;
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Z_ADDR, &v_data_u8, 1);
			*v_highrate_durn_axis_u8 = v_data_u8;
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_highrate_durn_axis_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			v_data_u8 = v_highrate_durn_axis_u8;
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_X_ADDR, &v_data_u8, 1);
			break;
		case BMG160_Y_AXIS:
			v_data_u8 = v_highrate_durn_axis_u8;
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Y_ADDR, &v_data_u8, 1);
			break;
		case BMG160_Z_AXIS:
			v_data_u8 = v_highrate_durn_axis_u8;
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_HIGHRATE_THRES_Z_ADDR, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_offset_thres_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_SLOW_OFFSET_THRES__REG, &v_data_u8, 1);
		*v_offset_thres_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_SLOW_OFFSET_THRES);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_slow_offset_thres(u8 v_offset_thres_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_SLOW_OFFSET_THRES__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_SLOW_OFFSET_THRES, v_offset_thres_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_SLOW_OFFSET_THRES__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_offset_durn_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_SLOW_OFFSET_DURN__REG, &v_data_u8, 1);
		*v_offset_durn_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_SLOW_OFFSET_DURN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_offset_durn_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_SLOW_OFFSET_DURN__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_SLOW_OFFSET_DURN, v_offset_durn_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_SLOW_OFFSET_DURN__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_channel_u8, u8 *v_slow_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_X__REG, &v_data_u8, 1);
			*v_slow_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_SLOW_OFFSET_ENABLE_X);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_Y__REG, &v_data_u8, 1);
			*v_slow_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_SLOW_OFFSET_ENABLE_Y);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_Z__REG, &v_data_u8, 1);
			*v_slow_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_SLOW_OFFSET_ENABLE_Z);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_channel_u8, u8 v_slow_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_X__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_SLOW_OFFSET_ENABLE_X, v_slow_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_X__REG, &v_data_u8, 1);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_Y__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_SLOW_OFFSET_ENABLE_Y, v_slow_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_Y__REG, &v_data_u8, 1);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_Z__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_SLOW_OFFSET_ENABLE_Z,
			v_slow_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_SLOW_OFFSET_ENABLE_Z__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_offset_word_length_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_AUTO_OFFSET_WORD_LENGHTH:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_AUTO_OFFSET_WORD_LENGHTH__REG, &v_data_u8, 1);
			*v_offset_word_length_u8 =
			BMG160_GET_BITSLICE(v_data_u8,
			BMG160_AUTO_OFFSET_WORD_LENGHTH);
			break;
		case BMG160_FAST_OFFSET_WORD_LENGHTH:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_WORD_LENGHTH__REG, &v_data_u8, 1);
			*v_offset_word_length_u8 =
			BMG160_GET_BITSLICE(v_data_u8,
			BMG160_FAST_OFFSET_WORD_LENGHTH);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_channel_u8, u8 v_offset_word_length_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_AUTO_OFFSET_WORD_LENGHTH:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_AUTO_OFFSET_WORD_LENGHTH__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_AUTO_OFFSET_WORD_LENGHTH,
			v_offset_word_length_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_AUTO_OFFSET_WORD_LENGHTH__REG,
			&v_data_u8, 1);
			break;
		case BMG160_FAST_OFFSET_WORD_LENGHTH:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_WORD_LENGHTH__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FAST_OFFSET_WORD_LENGHTH,
			v_offset_word_length_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_WORD_LENGHTH__REG,
			&v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_enable_fast_offset(void)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FAST_OFFSET_ENABLE__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_FAST_OFFSET_ENABLE, 1);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_FAST_OFFSET_ENABLE__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fast_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr,
		BMG160_FAST_OFFSET_ENABLE_XYZ__REG, &v_data_u8, 1);
		*v_fast_offset_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FAST_OFFSET_ENABLE_XYZ);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_channel_u8, u8 v_fast_offset_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_ENABLE_X__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FAST_OFFSET_ENABLE_X, v_fast_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_ENABLE_X__REG, &v_data_u8, 1);
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_ENABLE_Y__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FAST_OFFSET_ENABLE_Y, v_fast_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_ENABLE_Y__REG, &v_data_u8, 1);
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_ENABLE_Z__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FAST_OFFSET_ENABLE_Z, v_fast_offset_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FAST_OFFSET_ENABLE_Z__REG, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_nvm_remain(u8 *v_nvm_remain_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN__REG, &v_data_u8, 1);
		*v_nvm_remain_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_REMAIN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_nvm_load(u8 v_nvm_load_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD, v_nvm_load_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_LOAD__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_nvm_rdy(u8 *v_nvm_rdy_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY__REG, &v_data_u8, 1);
		*v_nvm_rdy_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_RDY);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_nvm_prog_trig(u8 nvm_prog_trig)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG, nvm_prog_trig);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_TRIG__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_nvm_prog_mode(u8 *nvm_prog_mode)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__REG, &v_data_u8, 1);
		*nvm_prog_mode = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_nvm_prog_mode(u8 nvm_prog_mode)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE, nvm_prog_mode);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_TRIM_NVM_CTRL_ADDR_NVM_PROG_MODE__REG, &v_data_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_i2c_wdt_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_I2C_WDT_ENABLE:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__REG,
			&v_data_u8, 1);
			*v_i2c_wdt_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE);
			break;
		case BMG160_I2C_WDT_SELECT:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__REG,
			&v_data_u8, 1);
			*v_i2c_wdt_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_i2c_wdt_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_channel_u8) {
		case BMG160_I2C_WDT_ENABLE:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE, v_i2c_wdt_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_ENABLE__REG,
			&v_data_u8, 1);
			break;
		case BMG160_I2C_WDT_SELECT:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT, v_i2c_wdt_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_BGW_SPI3_WDT_ADDR_I2C_WDT_SELECT__REG,
			&v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_spi3(u8 *v_spi3_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_BGW_SPI3_WDT_ADDR_SPI3__REG, &v_data_u8, 1);
		*v_spi3_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_BGW_SPI3_WDT_ADDR_SPI3);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_spi3(u8 v_spi3_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_BGW_SPI3_WDT_ADDR_SPI3__REG, &v_data_u8, 1);
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_BGW_SPI3_WDT_ADDR_SPI3, v_spi3_u8);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
		BMG160_BGW_SPI3_WDT_ADDR_SPI3__REG, &v_data_u8, 1);
	}
	return comres;
}
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_tag(u8 *v_fifo_tag_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_CGF1_ADDR_TAG__REG, &v_data_u8, 1);
		*v_fifo_tag_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF1_ADDR_TAG);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_tag(u8 v_fifo_tag_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_fifo_tag_u8 < C_BMG160_TWO_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_TAG__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF1_ADDR_TAG, v_fifo_tag_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_TAG__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_wm_level_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_CGF1_ADDR_WML__REG, &v_data_u8, 1);
		*v_fifo_wm_level_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF1_ADDR_WML);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_fifo_wm_level_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_fifo_wm_level_u8 < C_BMG160_ONETWENTYEIGHT_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_WML__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF1_ADDR_WML, v_fifo_wm_level_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF1_ADDR_WML__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s16 *v_offset_s16)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data1_u8r = C_BMG160_ZERO_U8X;
	u8 v_data2_u8r = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_X_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_OFFSET_X__REG, &v_data1_u8r, 1);
			v_data1_u8r = BMG160_GET_BITSLICE(v_data1_u8r,
			BMG160_TRIM_GP0_ADDR_OFFSET_X);
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC1_ADDR_OFFSET_X__REG, &v_data2_u8r, 1);
			v_data2_u8r = BMG160_GET_BITSLICE(v_data2_u8r,
			BMG160_OFC1_ADDR_OFFSET_X);
			v_data2_u8r = ((v_data2_u8r <<
			BMG160_SHIFT_TWO_POSITION) | v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr, BMG160_OFC2_ADDR, &v_data1_u8r, 1);
			*v_offset_s16 = (s16)((((s16)
			((s8)v_data1_u8r))
			<< BMG160_SHIFT_FOUR_POSITION) | (v_data2_u8r));
			break;
		case BMG160_Y_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG, &v_data1_u8r, 1);
			v_data1_u8r = BMG160_GET_BITSLICE(v_data1_u8r,
			BMG160_TRIM_GP0_ADDR_OFFSET_Y);
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC1_ADDR_OFFSET_Y__REG, &v_data2_u8r, 1);
			v_data2_u8r = BMG160_GET_BITSLICE(v_data2_u8r,
			BMG160_OFC1_ADDR_OFFSET_Y);
			v_data2_u8r = ((v_data2_u8r <<
			BMG160_SHIFT_ONE_POSITION) | v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC3_ADDR, &v_data1_u8r, 1);
			*v_offset_s16 = (s16)((((s16)
			((s8)v_data1_u8r))
			<< BMG160_SHIFT_FOUR_POSITION) | (v_data2_u8r));
			break;
		case BMG160_Z_AXIS:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG, &v_data1_u8r, 1);
			v_data1_u8r = BMG160_GET_BITSLICE(v_data1_u8r,
			BMG160_TRIM_GP0_ADDR_OFFSET_Z);
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC1_ADDR_OFFSET_Z__REG, &v_data2_u8r, 1);
			v_data2_u8r = BMG160_GET_BITSLICE(v_data2_u8r,
			BMG160_OFC1_ADDR_OFFSET_Z);
			v_data2_u8r = ((v_data2_u8r <<
			BMG160_SHIFT_ONE_POSITION)
			| v_data1_u8r);
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC4_ADDR, &v_data1_u8r, 1);
			*v_offset_s16 = (s16)((((s16)
			((s8)v_data1_u8r))
			<< BMG160_SHIFT_FOUR_POSITION) | (v_data2_u8r));
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_axis_u8, s16 v_offset_s16)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data1_u8r = C_BMG160_ZERO_U8X;
	u8 v_data2_u8r = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_axis_u8) {
		case BMG160_X_AXIS:
			v_data1_u8r = ((s8) (v_offset_s16 & 0x0FF0))
			>> BMG160_SHIFT_FOUR_POSITION;
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC2_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (u8) (v_offset_s16 & 0x000C);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,
			BMG160_OFC1_ADDR_OFFSET_X, v_data1_u8r);
			comres += p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC1_ADDR_OFFSET_X__REG, &v_data2_u8r, 1);

			v_data1_u8r = (u8) (v_offset_s16 & 0x0003);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,
			BMG160_TRIM_GP0_ADDR_OFFSET_X, v_data1_u8r);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_OFFSET_X__REG, &v_data2_u8r, 1);
			break;
		case BMG160_Y_AXIS:
			v_data1_u8r = ((s8) (v_offset_s16 & 0x0FF0)) >>
			BMG160_SHIFT_FOUR_POSITION;
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC3_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (u8) (v_offset_s16 & 0x000E);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,
			BMG160_OFC1_ADDR_OFFSET_Y, v_data1_u8r);
			comres += p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC1_ADDR_OFFSET_Y__REG, &v_data2_u8r, 1);

			v_data1_u8r = (u8) (v_offset_s16 & 0x0001);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,
			BMG160_TRIM_GP0_ADDR_OFFSET_Y, v_data1_u8r);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_OFFSET_Y__REG, &v_data2_u8r, 1);
			break;
		case BMG160_Z_AXIS:
			v_data1_u8r = ((s8) (v_offset_s16 & 0x0FF0)) >>
			BMG160_SHIFT_FOUR_POSITION;
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC4_ADDR, &v_data1_u8r, 1);

			v_data1_u8r = (u8) (v_offset_s16 & 0x000E);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,
			BMG160_OFC1_ADDR_OFFSET_Z, v_data1_u8r);
			comres += p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_OFC1_ADDR_OFFSET_Z__REG, &v_data2_u8r, 1);

			v_data1_u8r = (u8) (v_offset_s16 & 0x0001);
			v_data2_u8r = BMG160_SET_BITSLICE(v_data2_u8r,
			BMG160_TRIM_GP0_ADDR_OFFSET_Z, v_data1_u8r);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_OFFSET_Z__REG, &v_data2_u8r, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_gp_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_GP0:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_GP0__REG, &v_data_u8, 1);
			*v_gp_u8 = BMG160_GET_BITSLICE(v_data_u8,
			BMG160_TRIM_GP0_ADDR_GP0);
			break;
		case BMG160_GP1:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP1_ADDR, &v_data_u8, 1);
			*v_gp_u8 = v_data_u8;
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_gp_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		switch (v_param_u8) {
		case BMG160_GP0:
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_GP0__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_TRIM_GP0_ADDR_GP0, v_gp_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP0_ADDR_GP0__REG, &v_data_u8, 1);
			break;
		case BMG160_GP1:
			v_data_u8 = v_gp_u8;
			comres = p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_TRIM_GP1_ADDR, &v_data_u8, 1);
			break;
		default:
			comres = E_BMG160_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_FIFO_data_reg(u8 *v_fifo_data_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_DATA_ADDR, &v_data_u8, 1);
		*v_fifo_data_u8 = v_data_u8;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_stat_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_STAT_ADDR, v_fifo_stat_u8, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_frame_count_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8  = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_STAT_FRAME_COUNTER__REG, &v_data_u8, 1);
		*v_fifo_frame_count_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_STAT_FRAME_COUNTER);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_overrun_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_STAT_OVERRUN__REG, &v_data_u8, 1);
		*v_fifo_overrun_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_STAT_OVERRUN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_fifo_mode(u8 *v_fifo_mode_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_CGF0_ADDR_MODE__REG, &v_data_u8, 1);
		*v_fifo_mode_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF0_ADDR_MODE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_fifo_mode(u8 v_fifo_mode_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_fifo_mode_u8 < C_BMG160_FOUR_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_MODE__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF0_ADDR_MODE, v_fifo_mode_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_MODE__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_data_select_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG, &v_data_u8, 1);
		*v_fifo_data_select_u8 = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_FIFO_CGF0_ADDR_DATA_SELECT);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_fifo_data_select_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_fifo_data_select_u8 < C_BMG160_FOUR_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG, &v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_FIFO_CGF0_ADDR_DATA_SELECT,
			v_fifo_data_select_u8);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_FIFO_CGF0_ADDR_DATA_SELECT__REG, &v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_power_mode(u8 *v_power_mode_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 data1 = C_BMG160_ZERO_U8X;
	u8 data2 = C_BMG160_ZERO_U8X;
	u8 data3 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == C_BMG160_ZERO_U8X) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_ONE_U8X);
		comres += p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR, &data2, C_BMG160_ONE_U8X);
		data1  = (data1 & 0xA0) >> 5;
		data3  = (data2 & 0x40) >> 6;
		data2  = (data2 & 0x80) >> 7;
		if (data3 == 0x01) {
			*v_power_mode_u8  = BMG160_MODE_ADVANCEDPOWERSAVING;
		} else {
			if ((data1 == 0x00) && (data2 == 0x00)) {
				*v_power_mode_u8  = BMG160_MODE_NORMAL;
				} else {
				if ((data1 == 0x01) || (data1 == 0x05)) {
					*v_power_mode_u8  =
					BMG160_MODE_DEEPSUSPEND;
					} else {
					if ((data1 == 0x04) &&
					(data2 == 0x00)) {
						*v_power_mode_u8  =
						BMG160_MODE_SUSPEND;
					} else {
					if ((data1 == 0x04) &&
						(data2 == 0x01))
							*v_power_mode_u8  =
							BMG160_MODE_FASTPOWERUP;
						}
					}
				}
			}
		}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_power_mode(u8 v_power_mode_u8)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 data1 = C_BMG160_ZERO_U8X;
	u8 data2 = C_BMG160_ZERO_U8X;
	u8 data3 = C_BMG160_ZERO_U8X;
	u8 v_autosleepduration = C_BMG160_ZERO_U8X;
	u8 v_bw_u8r = C_BMG160_ZERO_U8X;
	if (p_bmg160 == C_BMG160_ZERO_U8X) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (v_power_mode_u8 < C_BMG160_FIVE_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_ONE_U8X);
			comres += p_bmg160->BMG160_BUS_READ_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data2, C_BMG160_ONE_U8X);
			switch (v_power_mode_u8) {
			case BMG160_MODE_NORMAL:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, C_BMG160_ZERO_U8X);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				C_BMG160_ZERO_U8X);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				C_BMG160_ZERO_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_ONE_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_ONE_U8X);
				break;
			case BMG160_MODE_DEEPSUSPEND:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, C_BMG160_ONE_U8X);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				C_BMG160_ZERO_U8X);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				C_BMG160_ZERO_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_ONE_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_ONE_U8X);
				break;
			case BMG160_MODE_SUSPEND:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, C_BMG160_FOUR_U8X);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				C_BMG160_ZERO_U8X);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				C_BMG160_ZERO_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_ONE_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_ONE_U8X);
				break;
			case BMG160_MODE_FASTPOWERUP:
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, C_BMG160_FOUR_U8X);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				C_BMG160_ONE_U8X);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				C_BMG160_ZERO_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_ONE_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_ONE_U8X);
				break;
			case BMG160_MODE_ADVANCEDPOWERSAVING:
				/* Configuring the proper settings for auto
				sleep duration */
				bmg160_get_bw(&v_bw_u8r);
				bmg160_get_auto_sleep_durn(
					&v_autosleepduration);
				bmg160_set_auto_sleep_durn(v_autosleepduration,
				v_bw_u8r);
				comres += p_bmg160->BMG160_BUS_READ_FUNC
					(p_bmg160->dev_addr,
				BMG160_MODE_LPM2_ADDR, &data2,
				C_BMG160_ONE_U8X);
				/* Configuring the advanced power saving mode*/
				data1  = BMG160_SET_BITSLICE(data1,
				BMG160_MODE_LPM1, C_BMG160_ZERO_U8X);
				data2  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_FAST_POWERUP,
				C_BMG160_ZERO_U8X);
				data3  = BMG160_SET_BITSLICE(data2,
				BMG160_MODE_LPM2_ADDR_ADV_POWERSAVING,
				C_BMG160_ONE_U8X);
				comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM1_ADDR, &data1, C_BMG160_ONE_U8X);
			p_bmg160->delay_msec(1);/*A minimum delay of atleast
			450us is required for Multiple write.*/
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
				(p_bmg160->dev_addr,
			BMG160_MODE_LPM2_ADDR, &data3, C_BMG160_ONE_U8X);
				break;
				}
		} else {
		comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_seleftest(u8 *v_result_u8)
	{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 data1 = C_BMG160_ZERO_U8X;
	u8 data2 = C_BMG160_ZERO_U8X;

	comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
	BMG160_SELECTF_TEST_ADDR, &data1, C_BMG160_ONE_U8X);
	data2  = BMG160_GET_BITSLICE(data1, BMG160_SELECTF_TEST_ADDR_RATEOK);
	data1  = BMG160_SET_BITSLICE(data1, BMG160_SELECTF_TEST_ADDR_TRIGBIST,
	C_BMG160_ONE_U8X);
	comres += p_bmg160->BMG160_BUS_WRITE_FUNC(p_bmg160->dev_addr,
	BMG160_SELECTF_TEST_ADDR_TRIGBIST__REG, &data1, C_BMG160_ONE_U8X);

	/* Waiting time to complete the selftest process */
	p_bmg160->delay_msec(10);

	/* Reading Selftest v_result_u8 bir bist_failure */
	comres += p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
	BMG160_SELECTF_TEST_ADDR_BISTFAIL__REG, &data1, C_BMG160_ONE_U8X);
	data1  = BMG160_GET_BITSLICE(data1, BMG160_SELECTF_TEST_ADDR_BISTFAIL);
	if ((data1 == 0x00) && (data2 == 0x01))
		*v_result_u8 = C_BMG160_SUCCESS;
	else
		*v_result_u8 = C_BMG160_FAILURE;
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_auto_sleep_durn(u8 *durn)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		 BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG, &v_data_u8, 1);
		*durn = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_bw_u8)
{
BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
u8 v_data_u8 = C_BMG160_ZERO_U8X;
u8 v_auto_sleep_durn_u8r = C_BMG160_ZERO_U8X;
if (p_bmg160 == BMG160_NULL) {
	return  E_BMG160_NULL_PTR;
} else {
	comres = p_bmg160->BMG160_BUS_READ_FUNC
		(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG,
		&v_data_u8, 1);
		if (durn < C_BMG160_EIGHT_U8X) {
			switch (v_bw_u8) {
			case C_BMG160_NO_FILTER_U8X:
				if (durn >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_230HZ_U8X:
				if (durn >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_116HZ_U8X:
				if (durn >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_47HZ_U8X:
				if (durn >
				C_BMG160_5MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_5MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_23HZ_U8X:
				if (durn >
				C_BMG160_10MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_10MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_12HZ_U8X:
				if (durn >
				C_BMG160_20MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
				v_auto_sleep_durn_u8r =
				C_BMG160_20MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_64HZ_U8X:
				if (durn >
				C_BMG160_10MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_10MS_AUTO_SLEEP_DURN_U8X;
				break;
			case C_BMG160_BW_32HZ_U8X:
				if (durn >
				C_BMG160_20MS_AUTO_SLEEP_DURN_U8X)
					v_auto_sleep_durn_u8r =
					durn;
				else
					v_auto_sleep_durn_u8r =
					C_BMG160_20MS_AUTO_SLEEP_DURN_U8X;
				break;
			default:
			if (durn >
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X)
				v_auto_sleep_durn_u8r =
					durn;
				else
				v_auto_sleep_durn_u8r =
				C_BMG160_4MS_AUTO_SLEEP_DURN_U8X;
				break;
			}
		v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN,
		v_auto_sleep_durn_u8r);
		comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
		BMG160_MODE_LPM2_ADDR_AUTO_SLEEP_DURN__REG,
		&v_data_u8, 1);
	} else {
		comres = E_BMG160_OUT_OF_RANGE;
	}
}
return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_get_sleep_durn(u8 *durn)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		comres = p_bmg160->BMG160_BUS_READ_FUNC(p_bmg160->dev_addr,
		 BMG160_MODELPM1_ADDR_SLEEP_DURN__REG, &v_data_u8, 1);
		*durn = BMG160_GET_BITSLICE(v_data_u8,
		BMG160_MODELPM1_ADDR_SLEEP_DURN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMG160_RETURN_FUNCTION_TYPE bmg160_set_sleep_durn(u8 durn)
{
	BMG160_RETURN_FUNCTION_TYPE comres = BMG160_ERROR;
	u8 v_data_u8 = C_BMG160_ZERO_U8X;
	if (p_bmg160 == BMG160_NULL) {
		return  E_BMG160_NULL_PTR;
	} else {
		if (durn < C_BMG160_EIGHT_U8X) {
			comres = p_bmg160->BMG160_BUS_READ_FUNC
			(p_bmg160->dev_addr,
			BMG160_MODELPM1_ADDR_SLEEP_DURN__REG,
			&v_data_u8, 1);
			v_data_u8 = BMG160_SET_BITSLICE(v_data_u8,
			BMG160_MODELPM1_ADDR_SLEEP_DURN, durn);
			comres += p_bmg160->BMG160_BUS_WRITE_FUNC
			(p_bmg160->dev_addr,
			BMG160_MODELPM1_ADDR_SLEEP_DURN__REG,
			&v_data_u8, 1);
		} else {
			comres = E_BMG160_OUT_OF_RANGE;
		}
	}
	return comres;
}

