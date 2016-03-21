/*
****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* bmm050.c
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
/*! \file <BMM050 >
    \brief <Sensor driver for BMM050> */

#include "bmm050.h"

static struct bmm050 *p_bmm050;

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
BMM050_RETURN_FUNCTION_TYPE bmm050_init(struct bmm050 *bmm050)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8[2] = {0, 0};
	p_bmm050 = bmm050;

	/* set device from suspend into sleep mode */
	com_rslt = bmm050_set_power_mode(BMM050_ON);

	/* wait two millisecond for bmc to settle */
	p_bmm050->delay_msec(BMM050_DELAY_SETTLING_TIME);

	/*Read CHIP_ID and REv. info */
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_CHIP_ID, v_data_u8, 1);
	p_bmm050->company_id = v_data_u8[0];

	/* Function to initialise trim values */
	com_rslt += bmm050_init_trim_registers();
	com_rslt += bmm050_set_presetmode(BMM050_PRESETMODE_REGULAR);
	return com_rslt;
}
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
u8 *v_presetmode_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_rate_u8 = BMM050_ZERO_U8X;
	u8 rep_xy = BMM050_ZERO_U8X;
	u8 rep_z = BMM050_ZERO_U8X;

	/* Get the current data rate */
	com_rslt = bmm050_get_data_rate(&v_data_rate_u8);
	/* Get the preset number of XY Repetitions */
	com_rslt += bmm050_get_rep_XY(&rep_xy);
	/* Get the preset number of Z Repetitions */
	com_rslt += bmm050_get_rep_Z(&rep_z);
	if ((v_data_rate_u8 == BMM050_LOWPOWER_DR) && (
		rep_xy == BMM050_LOWPOWER_REPXY) && (
		rep_z == BMM050_LOWPOWER_REPZ)) {
		*v_presetmode_u8 = BMM050_PRESETMODE_LOWPOWER;
	} else {
		if ((v_data_rate_u8 == BMM050_REGULAR_DR) && (
			rep_xy == BMM050_REGULAR_REPXY) && (
			rep_z == BMM050_REGULAR_REPZ)) {
			*v_presetmode_u8 = BMM050_PRESETMODE_REGULAR;
		} else {
			if ((v_data_rate_u8 == BMM050_HIGHACCURACY_DR) && (
				rep_xy == BMM050_HIGHACCURACY_REPXY) && (
				rep_z == BMM050_HIGHACCURACY_REPZ)) {
					*v_presetmode_u8 =
					BMM050_PRESETMODE_HIGHACCURACY;
			} else {
				if ((v_data_rate_u8 == BMM050_ENHANCED_DR) && (
				rep_xy == BMM050_ENHANCED_REPXY) && (
				rep_z == BMM050_ENHANCED_REPZ)) {
					*v_presetmode_u8 =
					BMM050_PRESETMODE_ENHANCED;
				} else {
					*v_presetmode_u8 =
					E_BMM050_UNDEFINED_MODE;
				}
			}
		}
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_set_presetmode(u8 v_presetmode_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	switch (v_presetmode_u8) {
	case BMM050_PRESETMODE_LOWPOWER:
		/* Set the data rate for Low Power mode */
		com_rslt = bmm050_set_data_rate(BMM050_LOWPOWER_DR);
		/* Set the XY-repetitions number for Low Power mode */
		com_rslt += bmm050_set_rep_XY(BMM050_LOWPOWER_REPXY);
		/* Set the Z-repetitions number  for Low Power mode */
		com_rslt += bmm050_set_rep_Z(BMM050_LOWPOWER_REPZ);
		break;
	case BMM050_PRESETMODE_REGULAR:
		/* Set the data rate for Regular mode */
		com_rslt = bmm050_set_data_rate(BMM050_REGULAR_DR);
		/* Set the XY-repetitions number for Regular mode */
		com_rslt += bmm050_set_rep_XY(BMM050_REGULAR_REPXY);
		/* Set the Z-repetitions number  for Regular mode */
		com_rslt += bmm050_set_rep_Z(BMM050_REGULAR_REPZ);
		break;
	case BMM050_PRESETMODE_HIGHACCURACY:
		/* Set the data rate for High Accuracy mode */
		com_rslt = bmm050_set_data_rate(BMM050_HIGHACCURACY_DR);
		/* Set the XY-repetitions number for High Accuracy mode */
		com_rslt += bmm050_set_rep_XY(BMM050_HIGHACCURACY_REPXY);
		/* Set the Z-repetitions number  for High Accuracy mode */
		com_rslt += bmm050_set_rep_Z(BMM050_HIGHACCURACY_REPZ);
		break;
	case BMM050_PRESETMODE_ENHANCED:
		/* Set the data rate for Enhanced Accuracy mode */
		com_rslt = bmm050_set_data_rate(BMM050_ENHANCED_DR);
		/* Set the XY-repetitions number for High Enhanced mode */
		com_rslt += bmm050_set_rep_XY(BMM050_ENHANCED_REPXY);
		/* Set the Z-repetitions number  for High Enhanced mode */
		com_rslt += bmm050_set_rep_Z(BMM050_ENHANCED_REPZ);
		break;
	default:
		com_rslt = E_BMM050_OUT_OF_RANGE;
		break;
	}
	return com_rslt;
}
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
u8 v_functional_state_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	} else {
		switch (v_functional_state_u8) {
		case BMM050_NORMAL_MODE:
			com_rslt = bmm050_get_power_mode(&v_data_u8);
			if (v_data_u8 == BMM050_OFF) {
				com_rslt += bmm050_set_power_mode(BMM050_ON);
				p_bmm050->delay_msec(
				BMM050_DELAY_SUSPEND_SLEEP);
			}
			com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_OPERATION_MODE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMM050_SET_BITSLICE(
			v_data_u8,
			BMM050_CONTROL_OPERATION_MODE,
			BMM050_NORMAL_MODE);
			com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_OPERATION_MODE__REG,
			&v_data_u8, 1);
			break;
		case BMM050_SUSPEND_MODE:
			com_rslt = bmm050_set_power_mode(BMM050_OFF);
			break;
		case BMM050_FORCED_MODE:
			com_rslt = bmm050_get_power_mode(&v_data_u8);
			if (v_data_u8 == BMM050_OFF) {
				com_rslt = bmm050_set_power_mode(BMM050_ON);
				p_bmm050->delay_msec(
				BMM050_DELAY_SUSPEND_SLEEP);
			}
			com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_OPERATION_MODE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMM050_SET_BITSLICE(
			v_data_u8,
			BMM050_CONTROL_OPERATION_MODE, BMM050_ON);
			com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_OPERATION_MODE__REG,
			&v_data_u8, 1);
			break;
		case BMM050_SLEEP_MODE:
			bmm050_get_power_mode(&v_data_u8);
			if (v_data_u8 == BMM050_OFF) {
				com_rslt = bmm050_set_power_mode(BMM050_ON);
				p_bmm050->delay_msec(
				BMM050_DELAY_SUSPEND_SLEEP);
			}
			com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_OPERATION_MODE__REG,
			&v_data_u8, 1);
			v_data_u8 = BMM050_SET_BITSLICE(
			v_data_u8,
			BMM050_CONTROL_OPERATION_MODE,
			BMM050_SLEEP_MODE);
			com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_OPERATION_MODE__REG,
			&v_data_u8, 1);
			break;
		default:
			com_rslt = E_BMM050_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
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
u8 *v_functional_state_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_CONTROL_OPERATION_MODE__REG,
		&v_data_u8, 1);
		*v_functional_state_u8 = BMM050_GET_BITSLICE(
		v_data_u8, BMM050_CONTROL_OPERATION_MODE);
	}
	return com_rslt;
}
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
struct bmm050_mag_data_s16_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;

	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	struct {
		s16 raw_data_x;
		s16 raw_data_y;
		s16 raw_data_z;
		u16 raw_data_r;
	} raw_data_xyz_t;

	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_DATAX_LSB, v_data_u8, 8);

		/* Reading data for X axis */
		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_DATAX_LSB_VALUEX);
		raw_data_xyz_t.raw_data_x = (s16)((((s32)
		((s8)v_data_u8[1])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[0]);

		/* Reading data for Y axis */
		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_DATAY_LSB_VALUEY);
		raw_data_xyz_t.raw_data_y = (s16)((((s32)
		((s8)v_data_u8[3])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[2]);

		/* Reading data for Z axis */
		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		raw_data_xyz_t.raw_data_z = (s16)((((s32)
		((s8)v_data_u8[5])) <<
		SHIFT_LEFT_7_POSITION) | v_data_u8[4]);

		/* Reading data for Resistance*/
		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		raw_data_xyz_t.raw_data_r = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);

		/* Compensation for X axis */
		mag_data->datax = bmm050_compensate_X(raw_data_xyz_t.raw_data_x,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Y axis */
		mag_data->datay = bmm050_compensate_Y(raw_data_xyz_t.raw_data_y,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Z axis */
		mag_data->dataz = bmm050_compensate_Z(raw_data_xyz_t.raw_data_z,
		raw_data_xyz_t.raw_data_r);

	    /* Output raw resistance value */
	    mag_data->resistance = raw_data_xyz_t.raw_data_r;
	}
	return com_rslt;
}
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
struct bmm050_remapped_mag_s16_data_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;

	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	struct {
		s16 raw_data_x;
		s16 raw_data_y;
		s16 raw_data_z;
		u16 raw_data_r;
	} raw_data_xyz_t;

	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_BMX055_REMAPPED_DATAY_LSB, v_data_u8, 8);

		/* Reading data for Y axis */
		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		raw_data_xyz_t.raw_data_y = (s16)((((s32)
		((s8)v_data_u8[1])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[0]);


		/* Reading data for X axis */
		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		raw_data_xyz_t.raw_data_x = (s16)((((s32)
		((s8)v_data_u8[3])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[2]);
		raw_data_xyz_t.raw_data_x = -raw_data_xyz_t.raw_data_x;

		/* Reading data for Z axis */
		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		raw_data_xyz_t.raw_data_z = (s16)((((s32)
		((s8)v_data_u8[5])) <<
		SHIFT_LEFT_7_POSITION) | v_data_u8[4]);

		/* Reading data for Resistance*/
		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		raw_data_xyz_t.raw_data_r = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);

		/* Compensation for X axis */
		mag_data->datax = bmm050_compensate_X(raw_data_xyz_t.raw_data_x,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Y axis */
		mag_data->datay = bmm050_compensate_Y(raw_data_xyz_t.raw_data_y,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Z axis */
		mag_data->dataz = bmm050_compensate_Z(raw_data_xyz_t.raw_data_z,
		raw_data_xyz_t.raw_data_r);

	    /* Output raw resistance value */
	    mag_data->resistance = raw_data_xyz_t.raw_data_r;
	}
	return com_rslt;
}
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
struct bmm050_mag_s32_data_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;

	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	struct {
		s16 raw_data_x;
		s16 raw_data_y;
		s16 raw_data_z;
		u16 raw_data_r;
	} raw_data_xyz_t;

	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_DATAX_LSB, v_data_u8, 8);

		/* Reading data for X axis */
		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_DATAX_LSB_VALUEX);
		raw_data_xyz_t.raw_data_x = (s16)((((s32)
		((s8)v_data_u8[1])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[0]);

		/* Reading data for Y axis */
		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_DATAY_LSB_VALUEY);
		raw_data_xyz_t.raw_data_y = (s16)((((s32)
		((s8)v_data_u8[3])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[2]);

		/* Reading data for Z axis */
		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		raw_data_xyz_t.raw_data_z = (s16)((((s32)
		((s8)v_data_u8[5])) <<
		SHIFT_LEFT_7_POSITION) | v_data_u8[4]);

		/* Reading data for Resistance*/
		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		raw_data_xyz_t.raw_data_r = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);

		/* Compensation for X axis */
		mag_data->datax = bmm050_compensate_X_s32(
		raw_data_xyz_t.raw_data_x,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Y axis */
		mag_data->datay = bmm050_compensate_Y_s32(
		raw_data_xyz_t.raw_data_y,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Z axis */
		mag_data->dataz = bmm050_compensate_Z_s32(
		raw_data_xyz_t.raw_data_z,
		raw_data_xyz_t.raw_data_r);

	    /* Output raw resistance value */
	    mag_data->resistance = raw_data_xyz_t.raw_data_r;
	}
	return com_rslt;
}
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
struct bmm050_remapped_mag_s32_data_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;

	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0 };

	struct {
		s16 raw_data_x;
		s16 raw_data_y;
		s16 raw_data_z;
		u16 raw_data_r;
	} raw_data_xyz_t;

	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_BMX055_REMAPPED_DATAY_LSB, v_data_u8, 8);

		/* Reading data for Y axis */
		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		raw_data_xyz_t.raw_data_y = (s16)((((s32)
		((s8)v_data_u8[1])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[0]);


		/* Reading data for X axis */
		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		raw_data_xyz_t.raw_data_x = (s16)((((s32)
		((s8)v_data_u8[3])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[2]);
		raw_data_xyz_t.raw_data_x = -raw_data_xyz_t.raw_data_x;

		/* Reading data for Z axis */
		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		raw_data_xyz_t.raw_data_z = (s16)((((s32)
		((s8)v_data_u8[5])) <<
		SHIFT_LEFT_7_POSITION) | v_data_u8[4]);

		/* Reading data for Resistance*/
		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		raw_data_xyz_t.raw_data_r = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);

		/* Compensation for X axis */
		mag_data->datax = bmm050_compensate_X_s32(
		raw_data_xyz_t.raw_data_x,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Y axis */
		mag_data->datay = bmm050_compensate_Y_s32(
		raw_data_xyz_t.raw_data_y,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Z axis */
		mag_data->dataz = bmm050_compensate_Z_s32(
		raw_data_xyz_t.raw_data_z,
		raw_data_xyz_t.raw_data_r);

		/* Output raw resistance value */
		mag_data->resistance = raw_data_xyz_t.raw_data_r;
	}
	return com_rslt;
}
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
struct bmm050_mag_data_float_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;

	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	struct {
		s16 raw_data_x;
		s16 raw_data_y;
		s16 raw_data_z;
		s16 raw_data_r;
	} raw_data_xyz_t;

	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_DATAX_LSB, v_data_u8, 8);

		/* Reading data for X axis */
		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_DATAX_LSB_VALUEX);
		raw_data_xyz_t.raw_data_x = (s16)((((s32)
		((s8)v_data_u8[1])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[0]);

		/* Reading data for Y axis */
		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_DATAY_LSB_VALUEY);
		raw_data_xyz_t.raw_data_y = (s16)((((s32)
		((s8)v_data_u8[3])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[2]);

		/* Reading data for Z axis */
		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		raw_data_xyz_t.raw_data_z = (s16)((((s32)
		((s8)v_data_u8[5])) <<
		SHIFT_LEFT_7_POSITION) | v_data_u8[4]);

		/* Reading data for Resistance*/
		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		raw_data_xyz_t.raw_data_r = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);

		/* Compensation for X axis */
		mag_data->datax = bmm050_compensate_X_float(
		raw_data_xyz_t.raw_data_x,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Y axis */
		mag_data->datay = bmm050_compensate_Y_float(
		raw_data_xyz_t.raw_data_y,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Z axis */
		mag_data->dataz = bmm050_compensate_Z_float(
		raw_data_xyz_t.raw_data_z,
		raw_data_xyz_t.raw_data_r);

		/* Output raw resistance value */
		mag_data->resistance = raw_data_xyz_t.raw_data_r;
	}
	return com_rslt;
}
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
struct bmm050_remapped_mag_data_float_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;

	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	struct {
		s16 raw_data_x;
		s16 raw_data_y;
		s16 raw_data_z;
		u16 raw_data_r;
	} raw_data_xyz_t;

	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_BMX055_REMAPPED_DATAY_LSB, v_data_u8, 8);

		/* Reading data for Y axis */
		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		raw_data_xyz_t.raw_data_y = (s16)((((s32)
		((s8)v_data_u8[1])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[0]);

		/* Reading data for X axis */
		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		raw_data_xyz_t.raw_data_x = (s16)((((s32)
		((s8)v_data_u8[3])) <<
		SHIFT_LEFT_5_POSITION) | v_data_u8[2]);
		raw_data_xyz_t.raw_data_x = -raw_data_xyz_t.raw_data_x;

		/* Reading data for Z axis */
		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		raw_data_xyz_t.raw_data_z = (s16)((((s32)
		((s8)v_data_u8[5])) <<
		SHIFT_LEFT_7_POSITION) | v_data_u8[4]);

		/* Reading data for Resistance*/
		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		raw_data_xyz_t.raw_data_r = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);

		/* Compensation for X axis */
		mag_data->datax = bmm050_compensate_X_float(
		raw_data_xyz_t.raw_data_x,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Y axis */
		mag_data->datay = bmm050_compensate_Y_float(
		raw_data_xyz_t.raw_data_y,
		raw_data_xyz_t.raw_data_r);

		/* Compensation for Z axis */
		mag_data->dataz = bmm050_compensate_Z_float(
		raw_data_xyz_t.raw_data_z,
		raw_data_xyz_t.raw_data_r);

		/* Output raw resistance value */
		mag_data->resistance = raw_data_xyz_t.raw_data_r;
	}
	return com_rslt;
}
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
u8 *v_data_u8, u8 v_len_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		v_addr_u8, v_data_u8, v_len_u8);
	}
	return com_rslt;
}
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
u8 *v_data_u8, u8 v_len_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
		v_addr_u8, v_data_u8, v_len_u8);
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_set_selftest(u8 v_selftest_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr, BMM050_CONTROL_SELFTEST__REG,
		&v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(
		v_data_u8, BMM050_CONTROL_SELFTEST, v_selftest_u8);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr, BMM050_CONTROL_SELFTEST__REG,
		&v_data_u8, 1);
	}
	return com_rslt;
}
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
u8 *v_selftest_xyz)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8[5] = {0, 0, 0, 0, 0};
	u8 v_result_u8 = 0x00;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr, BMM050_DATAX_LSB_TESTX__REG,
		v_data_u8, 5);

		v_result_u8 = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_TESTZ);

		v_result_u8 = (v_result_u8 << 1);
		v_result_u8 = (v_result_u8 | BMM050_GET_BITSLICE(
		v_data_u8[2], BMM050_DATAY_LSB_TESTY));

		v_result_u8 = (v_result_u8 << 1);
		v_result_u8 = (v_result_u8 | BMM050_GET_BITSLICE(
		v_data_u8[0], BMM050_DATAX_LSB_TESTX));

		*v_selftest_xyz = v_result_u8;
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_set_spi3(u8 v_value_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_SPI3_ENABLE__REG, &v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(v_data_u8,
		BMM050_POWER_CONTROL_SPI3_ENABLE, v_value_u8);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_SPI3_ENABLE__REG, &v_data_u8, 1);
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_set_data_rate(u8 v_data_rate_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_CONTROL_DATA_RATE__REG,
		&v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(v_data_u8,
		BMM050_CONTROL_DATA_RATE, v_data_rate_u8);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr,
		BMM050_CONTROL_DATA_RATE__REG,
		&v_data_u8, 1);
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_get_data_rate(u8 *v_data_rate_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_CONTROL_DATA_RATE__REG,
		&v_data_u8, 1);
		*v_data_rate_u8 = BMM050_GET_BITSLICE(v_data_u8,
		BMM050_CONTROL_DATA_RATE);
	}
	return com_rslt;
}
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
s16 *v_diff_z_s16)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	s16 result_positive = BMM050_ZERO_U8X;
	s16 result_negative  = BMM050_ZERO_U8X;
	struct bmm050_mag_data_s16_t mag_data = {0, 0, 0, 0};
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		/* set sleep mode to prepare for forced measurement.
		* If sensor is off, this will turn it on
		* and respect needed delays. */
		com_rslt = bmm050_set_functional_state(BMM050_SLEEP_MODE);

		/* set normal accuracy mode */
		com_rslt += bmm050_set_rep_Z(BMM050_LOWPOWER_REPZ);
		/* 14 repetitions Z in normal accuracy mode */

		/* disable X, Y channel */
		com_rslt += bmm050_set_control_measurement_x(
		BMM050_CHANNEL_DISABLE);
		com_rslt += bmm050_set_control_measurement_y(
		BMM050_CHANNEL_DISABLE);

		/* enable positive current and force a
		* measurement with positive field */
		com_rslt += bmm050_set_advanced_selftest(
		BMM050_ADVANCED_SELFTEST_POSITIVE);
		com_rslt += bmm050_set_functional_state(BMM050_FORCED_MODE);
		/* wait for measurement to complete */
		p_bmm050->delay_msec(4);

		/* read result from positive field measurement */
		com_rslt += bmm050_read_mag_data_XYZ(&mag_data);
		result_positive = mag_data.dataz;

		/* enable negative current and force a
		* measurement with negative field */
		com_rslt += bmm050_set_advanced_selftest(
		BMM050_ADVANCED_SELFTEST_NEGATIVE);
		com_rslt += bmm050_set_functional_state(BMM050_FORCED_MODE);
		p_bmm050->delay_msec(4); /* wait for measurement to complete */

		/* read result from negative field measurement */
		com_rslt += bmm050_read_mag_data_XYZ(&mag_data);
		result_negative = mag_data.dataz;

		/* turn off self test current */
		com_rslt += bmm050_set_advanced_selftest(
		BMM050_ADVANCED_SELFTEST_OFF);

		/* enable X, Y channel */
		com_rslt += bmm050_set_control_measurement_x(
		BMM050_CHANNEL_ENABLE);
		com_rslt += bmm050_set_control_measurement_y(
		BMM050_CHANNEL_ENABLE);

		/* write out difference in positive and negative field.
		* This should be ~ 200 mT = 3200 LSB */
		*v_diff_z_s16 = (result_positive - result_negative);
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_init_trim_registers(void)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8[2] = {0, 0};
	com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_X1, (u8 *)&p_bmm050->dig_x1, 1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_Y1, (u8 *)&p_bmm050->dig_y1, 1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_X2, (u8 *)&p_bmm050->dig_x2, 1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_Y2, (u8 *)&p_bmm050->dig_y2, 1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_XY1, (u8 *)&p_bmm050->dig_xy1, 1);
	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_XY2, (u8 *)&p_bmm050->dig_xy2, 1);

	/* shorts can not be recast into (u8*)
	* due to possible mix up between trim data
	* arrangement and memory arrangement */

	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_Z1_LSB, v_data_u8, 2);
	p_bmm050->dig_z1 = (u16)((((u32)((u8)
	v_data_u8[1])) <<
	SHIFT_LEFT_8_POSITION) | v_data_u8[0]);

	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_Z2_LSB, v_data_u8, 2);
	p_bmm050->dig_z2 = (s16)((((s32)(
	(s8)v_data_u8[1])) <<
	SHIFT_LEFT_8_POSITION) | v_data_u8[0]);

	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_Z3_LSB, v_data_u8, 2);
	p_bmm050->dig_z3 = (s16)((((s32)(
	(s8)v_data_u8[1])) <<
	SHIFT_LEFT_8_POSITION) | v_data_u8[0]);

	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_Z4_LSB, v_data_u8, 2);
	p_bmm050->dig_z4 = (s16)((((s32)(
	(s8)v_data_u8[1])) <<
	SHIFT_LEFT_8_POSITION) | v_data_u8[0]);

	com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
	BMM050_DIG_XYZ1_LSB, v_data_u8, 2);
	v_data_u8[1] = BMM050_GET_BITSLICE(v_data_u8[1], BMM050_DIG_XYZ1_MSB);
	p_bmm050->dig_xyz1 = (u16)((((u32)
	((u8)v_data_u8[1])) <<
	SHIFT_LEFT_8_POSITION) | v_data_u8[0]);
	return com_rslt;
}
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
u8 v_advanced_selftest_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		switch (v_advanced_selftest_u8) {
		case BMM050_ADVANCED_SELFTEST_OFF:
			com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_ADVANCED_SELFTEST__REG,
			&v_data_u8, 1);
			v_data_u8 = BMM050_SET_BITSLICE(
			v_data_u8,
			BMM050_CONTROL_ADVANCED_SELFTEST,
			BMM050_ADVANCED_SELFTEST_OFF);
			com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_ADVANCED_SELFTEST__REG,
			&v_data_u8, 1);
			break;
		case BMM050_ADVANCED_SELFTEST_POSITIVE:
			com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_ADVANCED_SELFTEST__REG,
			&v_data_u8, 1);
			v_data_u8 = BMM050_SET_BITSLICE(
			v_data_u8,
			BMM050_CONTROL_ADVANCED_SELFTEST,
			BMM050_ADVANCED_SELFTEST_POSITIVE);
			com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_ADVANCED_SELFTEST__REG,
			&v_data_u8, 1);
			break;
		case BMM050_ADVANCED_SELFTEST_NEGATIVE:
			com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_ADVANCED_SELFTEST__REG,
			&v_data_u8, 1);
			v_data_u8 = BMM050_SET_BITSLICE(
			v_data_u8,
			BMM050_CONTROL_ADVANCED_SELFTEST,
			BMM050_ADVANCED_SELFTEST_NEGATIVE);
			com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_CONTROL_ADVANCED_SELFTEST__REG,
			&v_data_u8, 1);
			break;
		default:
		break;
		}
	}
	return com_rslt;
}
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
u8 *v_advanced_selftest_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_CONTROL_ADVANCED_SELFTEST__REG, &v_data_u8, 1);
		*v_advanced_selftest_u8 = BMM050_GET_BITSLICE(v_data_u8,
		BMM050_CONTROL_ADVANCED_SELFTEST);
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_get_power_mode(u8 *v_power_mode_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_POWER_CONTROL_BIT__REG,
		&v_data_u8, 1);
		*v_power_mode_u8 = BMM050_GET_BITSLICE(v_data_u8,
		BMM050_POWER_CONTROL_POWER_CONTROL_BIT);
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_set_power_mode(u8 v_power_mode_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_POWER_CONTROL_BIT__REG,
		&v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(v_data_u8,
		BMM050_POWER_CONTROL_POWER_CONTROL_BIT, v_power_mode_u8);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_POWER_CONTROL_BIT__REG,
		&v_data_u8, 1);
	}
	return com_rslt;
}
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
u8 *v_rep_xy_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_REP_XY,
		&v_data_u8, 1);
		*v_rep_xy_u8 = v_data_u8;
	}
	return com_rslt;
}
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
u8 v_rep_xy_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		v_data_u8 = v_rep_xy_u8;
		com_rslt = p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr,
		BMM050_REP_XY,
		&v_data_u8, 1);
	}
	return com_rslt;
}
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
u8 *v_rep_z_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_REP_Z,
		&v_data_u8, 1);
		*v_rep_z_u8 = v_data_u8;
	}
	return com_rslt;
}
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
u8 v_rep_z_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		v_data_u8 = v_rep_z_u8;
		com_rslt = p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
		BMM050_REP_Z, &v_data_u8, 1);
	}
	return com_rslt;
}
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
s16 bmm050_compensate_X(s16 mag_data_x, u16 data_r)
{
	s16 inter_retval = BMM050_ZERO_U8X;
	if (mag_data_x != BMM050_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((s16)(((u16)
		((((s32)p_bmm050->dig_xyz1) << 14)/
		 (data_r != 0 ? data_r : p_bmm050->dig_xyz1))) -
		((u16)0x4000)));
		inter_retval = ((s16)((((s32)mag_data_x) *
				((((((((s32)p_bmm050->dig_xy2) *
				((((s32)inter_retval) *
				((s32)inter_retval)) >> 7)) +
			     (((s32)inter_retval) *
			      ((s32)(((s16)p_bmm050->dig_xy1)
			      << 7)))) >> 9) +
			   ((s32)0x100000)) *
			  ((s32)(((s16)p_bmm050->dig_x2) +
			  ((s16)0xA0)))) >> 12)) >> 13)) +
			(((s16)p_bmm050->dig_x1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}
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
s32 bmm050_compensate_X_s32(s16 mag_data_x, u16 data_r)
{
	s32 retval = BMM050_ZERO_U8X;

	retval = bmm050_compensate_X(mag_data_x, data_r);
	if (retval == (s32)BMM050_OVERFLOW_OUTPUT)
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	return retval;
}
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
float bmm050_compensate_X_float(s16 mag_data_x, u16 data_r)
{
	float inter_retval = BMM050_ZERO_U8X;
	if (mag_data_x != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
	   ) {
		if (data_r != 0) {
			inter_retval = ((((float)p_bmm050->dig_xyz1)*16384.0f
				/data_r)-16384.0f);
		} else {
			inter_retval = 0;
		}
		inter_retval = (((mag_data_x * ((((((float)p_bmm050->dig_xy2) *
			(inter_retval*inter_retval / 268435456.0f) +
			inter_retval*((float)p_bmm050->dig_xy1)/16384.0f))
			+ 256.0f) *	(((float)p_bmm050->dig_x2) + 160.0f)))
			/ 8192.0f) + (((float)p_bmm050->dig_x1) * 8.0f))/16.0f;
	} else {
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
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
s16 bmm050_compensate_Y(s16 mag_data_y, u16 data_r)
{
	s16 inter_retval = BMM050_ZERO_U8X;
	if (mag_data_y != BMM050_FLIP_OVERFLOW_ADCVAL  /* no overflow */
	   ) {
		inter_retval = ((s16)(((u16)(((
			(s32)p_bmm050->dig_xyz1) << 14)/
			(data_r != 0 ?
			 data_r : p_bmm050->dig_xyz1))) -
			((u16)0x4000)));
		inter_retval = ((s16)((((s32)mag_data_y) * ((((((((s32)
			p_bmm050->dig_xy2) * ((((s32) inter_retval) *
			((s32)inter_retval)) >> 7)) + (((s32)inter_retval) *
			((s32)(((s16)p_bmm050->dig_xy1) << 7)))) >> 9) +
			((s32)0x100000)) * ((s32)(((s16)p_bmm050->dig_y2)
			+ ((s16)0xA0)))) >> 12)) >> 13)) +
			(((s16)p_bmm050->dig_y1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}
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
s32 bmm050_compensate_Y_s32(s16 mag_data_y, u16 data_r)
{
	s32 retval = BMM050_ZERO_U8X;

	retval = bmm050_compensate_Y(mag_data_y, data_r);
	if (retval == BMM050_OVERFLOW_OUTPUT)
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	return retval;
}
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
float bmm050_compensate_Y_float(s16 mag_data_y, u16 data_r)
{
	float inter_retval = BMM050_ZERO_U8X;
	if (mag_data_y != BMM050_FLIP_OVERFLOW_ADCVAL /* no overflow */
	   ) {
		if (data_r != 0) {
			inter_retval = ((((float)p_bmm050->dig_xyz1)*16384.0f
			/data_r)-16384.0f);
		} else {
			inter_retval = 0;
		}
		inter_retval = (((mag_data_y * ((((((float)p_bmm050->dig_xy2) *
			(inter_retval*inter_retval / 268435456.0f) +
			inter_retval * ((float)p_bmm050->dig_xy1)/16384.0f)) +
			256.0f) * (((float)p_bmm050->dig_y2) + 160.0f)))
			/ 8192.0f) + (((float)p_bmm050->dig_y1) * 8.0f))/16.0f;
	} else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
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
s16 bmm050_compensate_Z(s16 mag_data_z, u16 data_r)
{
	s32 retval = BMM050_ZERO_U8X;
	if ((mag_data_z != BMM050_HALL_OVERFLOW_ADCVAL)	/* no overflow */
	   ) {
		retval = (((((s32)(mag_data_z - p_bmm050->dig_z4)) << 15) -
			((((s32)p_bmm050->dig_z3) * ((s32)(((s16)data_r) -
			((s16) p_bmm050->dig_xyz1))))>>2))/
			(p_bmm050->dig_z2 + ((s16)(((((s32)
			p_bmm050->dig_z1) * ((((s16)data_r) << 1)))+
			(1<<15))>>16))));
		/* saturate result to +/- 2 mT */
		if (retval > BMM050_POSITIVE_SATURATION_Z) {
			retval =  BMM050_POSITIVE_SATURATION_Z;
		} else {
			if (retval < BMM050_NEGATIVE_SATURATION_Z)
				retval = BMM050_NEGATIVE_SATURATION_Z;
		}
	} else {
		/* overflow */
		retval = BMM050_OVERFLOW_OUTPUT;
	}
	return (s16)retval;
}
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
s32 bmm050_compensate_Z_s32(s16 mag_data_z, u16 data_r)
{
	s32 retval = BMM050_ZERO_U8X;
	if (mag_data_z != BMM050_HALL_OVERFLOW_ADCVAL) {
		retval = (((((s32)(mag_data_z - p_bmm050->dig_z4)) << 15) -
		((((s32)p_bmm050->dig_z3) *
		((s32)(((s16)data_r) -
		((s16)p_bmm050->dig_xyz1))))>>2))/
		(p_bmm050->dig_z2 +
		((s16)(((((s32)p_bmm050->dig_z1) *
		((((s16)data_r) << 1)))+(1<<15))>>16))));
	} else {
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	}
		return retval;
}
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
float bmm050_compensate_Z_float (s16 mag_data_z, u16 data_r)
{
	float inter_retval = BMM050_ZERO_U8X;
	if (mag_data_z != BMM050_HALL_OVERFLOW_ADCVAL /* no overflow */
	   ) {
		inter_retval = ((((((float)mag_data_z)-
		((float)p_bmm050->dig_z4))*
		131072.0f)-(((float)p_bmm050->dig_z3)*(((float)data_r)-
		((float)p_bmm050->dig_xyz1))))/((((float)p_bmm050->dig_z2)+
		((float)p_bmm050->dig_z1)*((float)data_r)/32768.0)*4.0))/16.0;
	} else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}
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
u8 v_enable_disable_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_SENS_CONTROL_CHANNELX__REG,
		&v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(v_data_u8,
		BMM050_SENS_CONTROL_CHANNELX,
		v_enable_disable_u8);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr,
		BMM050_SENS_CONTROL_CHANNELX__REG,
		&v_data_u8, 1);
	}
	return com_rslt;
}
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
u8 v_enable_disable_u8)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_SENS_CONTROL_CHANNELY__REG,
		&v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(
		v_data_u8,
		BMM050_SENS_CONTROL_CHANNELY,
		v_enable_disable_u8);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr,
		BMM050_SENS_CONTROL_CHANNELY__REG,
		&v_data_u8, 1);
	}
	return com_rslt;
}
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
BMM050_RETURN_FUNCTION_TYPE bmm050_soft_rst(void)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8 = BMM050_ZERO_U8X;
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		v_data_u8 = BMM050_ON;

		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_SOFT_RST_7__REG,
		&v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(v_data_u8,
		BMM050_POWER_CONTROL_SOFT_RST_7,
		BMM050_SOFT_RESET7_ON);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_SOFT_RST_7__REG, &v_data_u8, 1);

		com_rslt += p_bmm050->BMM050_BUS_READ_FUNC(
		p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_SOFT_RST_1__REG,
		&v_data_u8, 1);
		v_data_u8 = BMM050_SET_BITSLICE(v_data_u8,
		BMM050_POWER_CONTROL_SOFT_RST_1,
		BMM050_SOFT_RESET1_ON);
		com_rslt += p_bmm050->BMM050_BUS_WRITE_FUNC(
		p_bmm050->dev_addr,
		BMM050_POWER_CONTROL_SOFT_RST_1__REG,
		&v_data_u8, 1);

		p_bmm050->delay_msec(BMM050_DELAY_SOFTRESET);
	}
	return com_rslt;
}
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
struct bmm050_mag_data_s16_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_DATAX_LSB, v_data_u8, 8);

		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_DATAX_LSB_VALUEX);
		mag_data->datax = (s16)((((s32)
		((s8)v_data_u8[1]))
		<< SHIFT_LEFT_5_POSITION)
		| v_data_u8[0]);

		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_DATAY_LSB_VALUEY);
		mag_data->datay = (s16)((((s32)
		((s8)v_data_u8[3]))
		<< SHIFT_LEFT_5_POSITION)
		| v_data_u8[2]);

		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		mag_data->dataz = (s16)((((s32)
		((s8)v_data_u8[5]))
		<< SHIFT_LEFT_7_POSITION)
		| v_data_u8[4]);

		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		mag_data->resistance = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);
	}
	return com_rslt;
}
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
struct bmm050_remapped_mag_s16_data_t *mag_data)
{
	BMM050_RETURN_FUNCTION_TYPE com_rslt = BMM050_ERROR;
	u8 v_data_u8[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	if (p_bmm050 == BMM050_NULL) {
		return  E_BMM050_NULL_PTR;
	} else {
		com_rslt = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
		BMM050_BMX055_REMAPPED_DATAY_LSB, v_data_u8, 8);

		v_data_u8[0] = BMM050_GET_BITSLICE(v_data_u8[0],
		BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		mag_data->datay = (s16)((((s32)
		((s8)v_data_u8[1]))
		<< SHIFT_LEFT_5_POSITION)
		| v_data_u8[0]);


		v_data_u8[2] = BMM050_GET_BITSLICE(v_data_u8[2],
		BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		mag_data->datax = (s16)((((s32)
		((s8)v_data_u8[3]))
		<< SHIFT_LEFT_5_POSITION)
		| v_data_u8[2]);
		mag_data->datax = -mag_data->datax;

		v_data_u8[4] = BMM050_GET_BITSLICE(v_data_u8[4],
		BMM050_DATAZ_LSB_VALUEZ);
		mag_data->dataz = (s16)((((s32)
		((s8)v_data_u8[5]))
		<< SHIFT_LEFT_7_POSITION)
		| v_data_u8[4]);

		v_data_u8[6] = BMM050_GET_BITSLICE(v_data_u8[6],
		BMM050_R_LSB_VALUE);
		mag_data->resistance = (u16)((((u32)
		v_data_u8[7]) <<
		SHIFT_LEFT_6_POSITION) | v_data_u8[6]);
	}
	return com_rslt;
}
