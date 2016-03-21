/*
****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* bma2x2.c
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
/*! file <BMA2x2 >
    brief <Sensor driver for BMA2x2> */
#include "bma2x2.h"
/* user defined code to be added here ... */
static struct bma2x2_t *p_bma2x2;
/* Based on Bit resolution v_value_u8 should be modified */
u8 V_BMA2x2RESOLUTION_U8 = BMA2x2_14_RESOLUTION;
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_data_u8, u32 v_len_u32)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BURST_READ_FUNC
			(p_bma2x2->dev_addr, v_addr_u8, v_data_u8, v_len_u32);
		}
	return com_rslt;
}
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_init(struct bma2x2_t *bma2x2)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	u8 v_config_data_u8 = C_BMA2x2_ZERO_U8X;

	p_bma2x2 = bma2x2;       /* assign bma2x2 ptr */
	com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
	(p_bma2x2->dev_addr,
	BMA2x2_CHIP_ID__REG, &v_data_u8, 1);/* read Chip Id */
	p_bma2x2->chip_id = v_data_u8;    /* get bit slice */
	com_rslt += bma2x2_read_reg(BMA2x2_FIFO_MODE_REG, &v_config_data_u8, 1);
	p_bma2x2->fifo_config = v_config_data_u8;
	return com_rslt;
}
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
u8 *v_data_u8, u8 v_len_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, v_adr_u8, v_data_u8, v_len_u8);
	}
	return com_rslt;
}
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
u8 *v_data_u8, u8 v_len_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, v_adr_u8, v_data_u8, v_len_u8);
		}
	return com_rslt;
}
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_x(s16 *v_accel_x_s16)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8	data[2] = {0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		case BMA2x2_12_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_X12_LSB__REG, data, 2);
			*v_accel_x_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xF0));
			*v_accel_x_s16 = *v_accel_x_s16 >> C_BMA2x2_FOUR_U8X;
		break;
		case BMA2x2_10_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_X10_LSB__REG, data, 2);
			*v_accel_x_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xC0));
			*v_accel_x_s16 = *v_accel_x_s16 >> C_BMA2x2_SIX_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_X14_LSB__REG, data, 2);
			*v_accel_x_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xFC));
			*v_accel_x_s16 = *v_accel_x_s16 >> C_BMA2x2_TWO_U8X;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s8 *v_accel_x_s8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8	data = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_X_AXIS_MSB_REG, &data, 1);
			*v_accel_x_s8 = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACCEL_X_MSB);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_y(s16 *v_accel_y_s16)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 data[2] = {0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		case BMA2x2_12_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_Y12_LSB__REG, data, 2);
			*v_accel_y_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xF0));
			*v_accel_y_s16 = *v_accel_y_s16 >> C_BMA2x2_FOUR_U8X;
		break;
		case BMA2x2_10_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_Y10_LSB__REG, data, 2);
			*v_accel_y_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xC0));
			*v_accel_y_s16 = *v_accel_y_s16 >> C_BMA2x2_SIX_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_Y14_LSB__REG, data, 2);
			*v_accel_y_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xFC));
			*v_accel_y_s16 = *v_accel_y_s16 >> C_BMA2x2_TWO_U8X;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s8 *v_accel_y_s8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8	data = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Y_AXIS_MSB_REG, &data, 1);
			*v_accel_y_s8 = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACCEL_Y_MSB);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_accel_z(s16 *v_accel_z_s16)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 data[2] = {0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		case BMA2x2_12_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_Z12_LSB__REG, data, 2);
			*v_accel_z_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xF0));
			*v_accel_z_s16 = *v_accel_z_s16 >> C_BMA2x2_FOUR_U8X;
		break;
		case BMA2x2_10_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_Z10_LSB__REG, data, 2);
			*v_accel_z_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xC0));
			*v_accel_z_s16 = *v_accel_z_s16 >> C_BMA2x2_SIX_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ACCEL_Z14_LSB__REG, data, 2);
			*v_accel_z_s16 = (s16)((((s32)((s8)data[1]))
			<< C_BMA2x2_EIGHT_U8X) | (data[0] & 0xFC));
			*v_accel_z_s16 = *v_accel_z_s16 >> C_BMA2x2_TWO_U8X;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s8 *v_accel_z_s8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8	data = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Z_AXIS_MSB_REG, &data, 1);
			*v_accel_z_s8 = BMA2x2_GET_BITSLICE(data,
			BMA2x2_ACCEL_Z_MSB);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
struct bma2x2_accel_data *accel)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8[6] = {0, 0, 0, 0, 0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		case BMA2x2_12_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X12_LSB__REG,
			v_data_u8, 6);
			/* read the x v_data_u8*/
			accel->x = (s16)((((s32)((s8)v_data_u8[1]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[0] & 0xF0));
			accel->x = accel->x >> C_BMA2x2_FOUR_U8X;

			/* read the y v_data_u8*/
			accel->y = (s16)((((s32)((s8)v_data_u8[3]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[2] & 0xF0));
			accel->y = accel->y >> C_BMA2x2_FOUR_U8X;

			/* read the z v_data_u8*/
			accel->z = (s16)((((s32)((s8)v_data_u8[5]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[4] & 0xF0));
			accel->z = accel->z >> C_BMA2x2_FOUR_U8X;

		break;
		case BMA2x2_10_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X10_LSB__REG,
			v_data_u8, 6);
			/* read the x v_data_u8*/
			accel->x = (s16)((((s32)((s8)v_data_u8[1]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[0] & 0xC0));
			accel->x = accel->x >> C_BMA2x2_SIX_U8X;

			/* read the y v_data_u8*/
			accel->y = (s16)((((s32)((s8)v_data_u8[3]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[2] & 0xC0));
			accel->y = accel->y >> C_BMA2x2_SIX_U8X;

			/* read the z v_data_u8*/
			accel->z = (s16)((((s32)((s8)v_data_u8[5]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[4] & 0xC0));
			accel->z = accel->z >> C_BMA2x2_SIX_U8X;
		break;
		case BMA2x2_14_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X14_LSB__REG,
			v_data_u8, 6);

			/* read the x v_data_u8*/
			accel->x = (s16)((((s32)((s8)v_data_u8[1]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[0] & 0xFC));
			accel->x = accel->x >> C_BMA2x2_TWO_U8X;

			/* read the y v_data_u8*/
			accel->y = (s16)((((s32)((s8)v_data_u8[3]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[2] & 0xFC));
			accel->y = accel->y >> C_BMA2x2_TWO_U8X;

			/* read the z v_data_u8*/
			accel->z = (s16)((((s32)((s8)v_data_u8[5]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[4] & 0xFC));
			accel->z = accel->z >> C_BMA2x2_TWO_U8X;
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
struct bma2x2_accel_eight_resolution *accel)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8	v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_X_AXIS_MSB_REG, &v_data_u8, 1);
		accel->x = BMA2x2_GET_BITSLICE(v_data_u8,
		BMA2x2_ACCEL_X_MSB);

		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_Y_AXIS_MSB_REG, &v_data_u8, 1);
		accel->y = BMA2x2_GET_BITSLICE(v_data_u8,
		BMA2x2_ACCEL_Y_MSB);

		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_Z_AXIS_MSB_REG, &v_data_u8, 1);
		accel->z = BMA2x2_GET_BITSLICE(v_data_u8,
		BMA2x2_ACCEL_Z_MSB);
		}
	return com_rslt;
}
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
u8 *v_stat_tap_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_STAT_TAP_SLOPE_REG,
			v_stat_tap_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_stat_orient_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_STAT_ORIENT_HIGH_REG,
			v_stat_orient_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_stat_fifo_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_STAT_FIFO_REG,
			v_stat_fifo_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_frame_count_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_FRAME_COUNT_STAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_frame_count_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_FIFO_FRAME_COUNT_STAT);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_overrun_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_OVERRUN_STAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_fifo_overrun_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_FIFO_OVERRUN_STAT);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_stat_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_STAT1_REG, v_intr_stat_u8, C_BMA2x2_FOUR_U8X);
		}
	return com_rslt;
}
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_range(u8 *v_range_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(p_bma2x2->dev_addr,
		BMA2x2_RANGE_SELECT__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		v_data_u8 = BMA2x2_GET_BITSLICE(v_data_u8, BMA2x2_RANGE_SELECT);
		*v_range_u8 = v_data_u8;
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_range(u8 v_range_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if ((v_range_u8 == C_BMA2x2_THREE_U8X) ||
		(v_range_u8 == C_BMA2x2_FIVE_U8X) ||
		(v_range_u8 == C_BMA2x2_EIGHT_U8X) ||
		(v_range_u8 == C_BMA2x2_TWELVE_U8X)) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RANGE_SELECT_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			switch (v_range_u8) {
			case BMA2x2_RANGE_2G:
				v_data_u8  = BMA2x2_SET_BITSLICE(v_data_u8,
				BMA2x2_RANGE_SELECT, C_BMA2x2_THREE_U8X);
			break;
			case BMA2x2_RANGE_4G:
				v_data_u8  = BMA2x2_SET_BITSLICE(v_data_u8,
				BMA2x2_RANGE_SELECT, C_BMA2x2_FIVE_U8X);
			break;
			case BMA2x2_RANGE_8G:
				v_data_u8  = BMA2x2_SET_BITSLICE(v_data_u8,
				BMA2x2_RANGE_SELECT, C_BMA2x2_EIGHT_U8X);
			break;
			case BMA2x2_RANGE_16G:
				v_data_u8  = BMA2x2_SET_BITSLICE(v_data_u8,
				BMA2x2_RANGE_SELECT, C_BMA2x2_TWELVE_U8X);
			break;
			default:
			break;
			}
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RANGE_SELECT_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_bw(u8 *v_bw_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BW__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_GET_BITSLICE(v_data_u8, BMA2x2_BW);
			*v_bw_u8 = v_data_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_bw(u8 v_bw_u8)
{
BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
u8 v_data_bw_u8 = C_BMA2x2_ZERO_U8X;
if (p_bma2x2 == BMA2x2_NULL) {
	return E_BMA2x2_NULL_PTR;
	} else {
	if (p_bma2x2->chip_id == 0xFB) {
		if (v_bw_u8 > C_BMA2x2_SEVEN_U8X &&
		v_bw_u8 < C_BMA2x2_FIFETEEN_U8X) {
			switch (v_bw_u8) {
			case BMA2x2_BW_7_81HZ:
				v_data_bw_u8 = BMA2x2_BW_7_81HZ;

				/*  7.81 Hz      64000 uS   */
			break;
			case BMA2x2_BW_15_63HZ:
				v_data_bw_u8 = BMA2x2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case BMA2x2_BW_31_25HZ:
				v_data_bw_u8 = BMA2x2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case BMA2x2_BW_62_50HZ:
				v_data_bw_u8 = BMA2x2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case BMA2x2_BW_125HZ:
				v_data_bw_u8 = BMA2x2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case BMA2x2_BW_250HZ:
				v_data_bw_u8 = BMA2x2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case BMA2x2_BW_500HZ:
				v_data_bw_u8 = BMA2x2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
			default:
			break;
			}
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BW__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_BW, v_data_bw_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BW__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			} else {
			com_rslt = E_OUT_OF_RANGE;
			}
		} else {
		if (v_bw_u8 > C_BMA2x2_SEVEN_U8X &&
		v_bw_u8 < C_BMA2x2_SIXTEEN_U8X) {
			switch (v_bw_u8) {
			case BMA2x2_BW_7_81HZ:
				v_data_bw_u8 = BMA2x2_BW_7_81HZ;

			/*  7.81 Hz      64000 uS   */
			break;
			case BMA2x2_BW_15_63HZ:
				v_data_bw_u8 = BMA2x2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
			case BMA2x2_BW_31_25HZ:
				v_data_bw_u8 = BMA2x2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
			case BMA2x2_BW_62_50HZ:
				v_data_bw_u8 = BMA2x2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
			case BMA2x2_BW_125HZ:
				v_data_bw_u8 = BMA2x2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
			case BMA2x2_BW_250HZ:
				v_data_bw_u8 = BMA2x2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
			case BMA2x2_BW_500HZ:
				v_data_bw_u8 = BMA2x2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
			case BMA2x2_BW_1000HZ:
				v_data_bw_u8 = BMA2x2_BW_1000HZ;

			/*  1000 Hz      500 uS   */
			break;
			default:
			break;
			}
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BW__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_BW, v_data_bw_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_BW__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			} else {
			com_rslt = E_OUT_OF_RANGE;
			}
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_power_mode_u8)
{
BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
u8 v_data2_u8 = C_BMA2x2_ZERO_U8X;
if (p_bma2x2 == BMA2x2_NULL) {
	return E_BMA2x2_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		com_rslt += p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_LOW_NOISE_CTRL_REG,
		&v_data2_u8, C_BMA2x2_ONE_U8X);

		v_data_u8  = (v_data_u8 & 0xE0) >> 5;
		v_data2_u8  = (v_data2_u8 & 0x40) >> 6;

	if ((v_data_u8 == 0x00) &&
	(v_data2_u8 == 0x00)) {
		*v_power_mode_u8  = BMA2x2_MODE_NORMAL;
		} else {
		if ((v_data_u8 == 0x02) &&
		(v_data2_u8 == 0x00)) {
			*v_power_mode_u8  =
			BMA2x2_MODE_LOWPOWER1;
			} else {
			if ((v_data_u8 == 0x04
			|| v_data_u8 == 0x06) &&
			(v_data2_u8 == 0x00)) {
				*v_power_mode_u8  =
				BMA2x2_MODE_SUSPEND;
				} else {
				if (((v_data_u8 & 0x01) == 0x01)) {
					*v_power_mode_u8  =
					BMA2x2_MODE_DEEP_SUSPEND;
					} else {
					if ((v_data_u8 == 0x02)
					&& (v_data2_u8 == 0x01)) {
						*v_power_mode_u8  =
						BMA2x2_MODE_LOWPOWER2;
					} else {
					if ((v_data_u8 == 0x04) &&
						(v_data2_u8 == 0x01))
							*v_power_mode_u8  =
							BMA2x2_MODE_STANDBY;
					else
						*v_power_mode_u8 =
						BMA2x2_MODE_DEEP_SUSPEND;
						}
					}
				}
			}
		}
	}
	p_bma2x2->v_power_mode_u8 = *v_power_mode_u8;
return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_power_mode(u8 v_power_mode_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 mode_ctr_eleven_reg = C_BMA2x2_ZERO_U8X;
	u8 mode_ctr_twel_reg = C_BMA2x2_ZERO_U8X;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	u8 v_data2_u8 = C_BMA2x2_ZERO_U8X;
	u8 v_pre_fifo_config_data = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_MODE_CTRL_REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		com_rslt += p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_LOW_POWER_MODE__REG,
		&v_data2_u8, C_BMA2x2_ONE_U8X);

		com_rslt += bma2x2_set_mode_value(v_power_mode_u8);
		mode_ctr_eleven_reg = p_bma2x2->ctrl_mode_reg;
		mode_ctr_twel_reg =  p_bma2x2->low_mode_reg;
		/* write the power mode to
		register 0x12*/
		v_data2_u8  = BMA2x2_SET_BITSLICE
		(v_data2_u8, BMA2x2_LOW_POWER_MODE,
		mode_ctr_twel_reg);
		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, BMA2x2_LOW_POWER_MODE__REG,
		&v_data2_u8, C_BMA2x2_ONE_U8X);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay_msec(1);
		/* Enter the power mode to suspend*/
		v_data_u8  = BMA2x2_SET_BITSLICE
		(v_data_u8, BMA2x2_MODE_CTRL,
		C_BMA2x2_FOUR_U8X);
		/* write the power mode to suspend*/
		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay_msec(1);
		/* write the previous FIFO mode and data select*/
		v_pre_fifo_config_data = p_bma2x2->fifo_config;
		com_rslt += bma2x2_write_reg(BMA2x2_FIFO_MODE_REG,
		&v_pre_fifo_config_data, 1);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay_msec(1);
		com_rslt += p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_MODE_CTRL_REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		/* write the power mode to 11th register*/
		v_data_u8  = BMA2x2_SET_BITSLICE
		(v_data_u8, BMA2x2_MODE_CTRL,
		mode_ctr_eleven_reg);
		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, BMA2x2_MODE_CTRL_REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		/*A minimum delay of
		atleast 450us is required for
		the low power modes,
		as per the data sheet.*/
		p_bma2x2->delay_msec(1);
	}
	return com_rslt;
}
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_mode_value(u8 v_power_mode_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_SUCCESS;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
	if (v_power_mode_u8 < C_BMA2x2_SIX_U8X) {
		switch (v_power_mode_u8)	{
		case BMA2x2_MODE_NORMAL:
			p_bma2x2->ctrl_mode_reg = 0x00;
			p_bma2x2->low_mode_reg = 0x00;
		break;
		case BMA2x2_MODE_LOWPOWER1:
			p_bma2x2->ctrl_mode_reg = 0x02;
			p_bma2x2->low_mode_reg = 0x00;
		break;
		case BMA2x2_MODE_LOWPOWER2:
			p_bma2x2->ctrl_mode_reg = 0x02;
			p_bma2x2->low_mode_reg = 0x01;
		break;
		case BMA2x2_MODE_SUSPEND:
			p_bma2x2->ctrl_mode_reg = 0x04;
			p_bma2x2->low_mode_reg = 0x00;
		break;
		case BMA2x2_MODE_STANDBY:
			p_bma2x2->ctrl_mode_reg = 0x04;
			p_bma2x2->low_mode_reg = 0x01;
		break;
		case BMA2x2_MODE_DEEP_SUSPEND:
			p_bma2x2->ctrl_mode_reg = 0x01;
		break;
		}
		} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_sleep_durn(u8 *v_sleep_durn_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			/*SLEEP DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_sleep_durn_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_SLEEP_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleep_durn(u8 v_sleep_durn_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_sleep_durn_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_sleep_durn_u8 > C_BMA2x2_FOUR_U8X &&
		v_sleep_durn_u8 < C_BMA2x2_SIXTEEN_U8X) {
			switch (v_sleep_durn_u8) {
			case BMA2x2_SLEEP_DURN_0_5MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_0_5MS;

				/*  0.5 MS   */
			break;
			case BMA2x2_SLEEP_DURN_1MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_1MS;

				/*  1 MS  */
			break;
			case BMA2x2_SLEEP_DURN_2MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_2MS;

				/*  2 MS  */
			break;
			case BMA2x2_SLEEP_DURN_4MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_4MS;

				/*  4 MS   */
			break;
			case BMA2x2_SLEEP_DURN_6MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_6MS;

				/*  6 MS  */
			break;
			case BMA2x2_SLEEP_DURN_10MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_10MS;

				/*  10 MS  */
			break;
			case BMA2x2_SLEEP_DURN_25MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_25MS;

				/*  25 MS  */
			break;
			case BMA2x2_SLEEP_DURN_50MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_50MS;

				/*  50 MS   */
			break;
			case BMA2x2_SLEEP_DURN_100MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_100MS;

				/*  100 MS  */
			break;
			case BMA2x2_SLEEP_DURN_500MS:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_500MS;

				/*  500 MS   */
			break;
			case BMA2x2_SLEEP_DURN_1S:
				v_data_sleep_durn_u8 = BMA2x2_SLEEP_DURN_1S;

				/*  1 SECS   */
			break;
			default:
			break;
			}
			/*SLEEP DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_SLEEP_DURN, v_data_sleep_durn_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
 *                  1 => enable Equidistant sampling mode(EST)
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
u8 *v_sleep_timer_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			/*SLEEP TIMER MODE*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_TIMER__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_sleep_timer_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_SLEEP_TIMER);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_sleep_timer_mode(u8 v_sleep_timer_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_sleep_timer_u8 < C_BMA2x2_TWO_U8X) {
			/*SLEEP TIMER MODE*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_TIMER__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_SLEEP_TIMER, v_sleep_timer_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLEEP_TIMER__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_high_bw(u8 *v_high_bw_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ENABLE_DATA_HIGH_BW__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_high_bw_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_DATA_HIGH_BW);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_high_bw(u8 v_high_bw_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		}  else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ENABLE_DATA_HIGH_BW__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_DATA_HIGH_BW, v_high_bw_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_DATA_HIGH_BW__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_shadow_dis(u8 *v_shadow_dis_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_DIS_SHADOW_PROC__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_shadow_dis_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_DIS_SHADOW_PROC);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_shadow_dis(u8 v_shadow_dis_u8)
{

	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_DIS_SHADOW_PROC__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_DIS_SHADOW_PROC, v_shadow_dis_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_DIS_SHADOW_PROC__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_soft_rst(void)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = BMA2x2_ENABLE_SOFT_RESET_VALUE;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		}  else {
			/* To reset the sensor
			0xB6 v_value_u8 will be written */
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_RST_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_update_image(void)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_UPDATE_IMAGE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_UPDATE_IMAGE, C_BMA2x2_ONE_U8X);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_UPDATE_IMAGE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_value_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_intr_type_u8) {
		case BMA2x2_LOW_G_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_LOW_G_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_LOW_G_INTR);
		break;
		case BMA2x2_HIGH_G_X_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_HIGH_G_X_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_HIGH_G_X_INTR);
		break;
		case BMA2x2_HIGH_G_Y_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_HIGH_G_Y_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_HIGH_G_Y_INTR);
		break;
		case BMA2x2_HIGH_G_Z_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_HIGH_G_Z_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_HIGH_G_Z_INTR);
		break;
		case BMA2x2_DATA_ENABLE:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_NEW_DATA_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_NEW_DATA_INTR);
		break;
		case BMA2x2_SLOPE_X_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOPE_X_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SLOPE_X_INTR);
		break;
		case BMA2x2_SLOPE_Y_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOPE_Y_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SLOPE_Y_INTR);
		break;
		case BMA2x2_SLOPE_Z_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOPE_Z_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SLOPE_Z_INTR);
		break;
		case BMA2x2_SINGLE_TAP_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SINGLE_TAP_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SINGLE_TAP_INTR);
		break;
		case BMA2x2_DOUBLE_TAP_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_DOUBLE_TAP_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_DOUBLE_TAP_INTR);
		break;
		case BMA2x2_ORIENT_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_ORIENT_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_ORIENT_INTR);
		break;
		case BMA2x2_FLAT_INTR:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_FLAT_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_value_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_FLAT_INTR);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
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
u8 v_value_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	u8 v_data2_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE1_REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		com_rslt += p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE2_REG,
		&v_data2_u8, C_BMA2x2_ONE_U8X);
		v_value_u8 = v_value_u8 & C_BMA2x2_ONE_U8X;
		switch (v_intr_type_u8) {
		case BMA2x2_LOW_G_INTR:
			/* Low G Interrupt  */
			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
			BMA2x2_ENABLE_LOW_G_INTR, v_value_u8);
		break;
		case BMA2x2_HIGH_G_X_INTR:
			/* High G X Interrupt */
			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
			BMA2x2_ENABLE_HIGH_G_X_INTR, v_value_u8);
		break;
		case BMA2x2_HIGH_G_Y_INTR:
			/* High G Y Interrupt */
			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
			BMA2x2_ENABLE_HIGH_G_Y_INTR, v_value_u8);
		break;
		case BMA2x2_HIGH_G_Z_INTR:
			/* High G Z Interrupt */
			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
			BMA2x2_ENABLE_HIGH_G_Z_INTR, v_value_u8);
		break;
		case BMA2x2_DATA_ENABLE:
			/*Data En Interrupt  */
			v_data2_u8 = BMA2x2_SET_BITSLICE(v_data2_u8,
			BMA2x2_ENABLE_NEW_DATA_INTR, v_value_u8);
		break;
		case BMA2x2_SLOPE_X_INTR:
			/* Slope X Interrupt */
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_SLOPE_X_INTR, v_value_u8);
		break;
		case BMA2x2_SLOPE_Y_INTR:
			/* Slope Y Interrupt */
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_SLOPE_Y_INTR, v_value_u8);
		break;
		case BMA2x2_SLOPE_Z_INTR:
			/* Slope Z Interrupt */
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_SLOPE_Z_INTR, v_value_u8);
		break;
		case BMA2x2_SINGLE_TAP_INTR:
			/* Single Tap Interrupt */
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
				BMA2x2_ENABLE_SINGLE_TAP_INTR, v_value_u8);
		break;
		case BMA2x2_DOUBLE_TAP_INTR:
			/* Double Tap Interrupt */
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
				BMA2x2_ENABLE_DOUBLE_TAP_INTR, v_value_u8);
		break;
		case BMA2x2_ORIENT_INTR:
			/* Orient Interrupt  */
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_ORIENT_INTR, v_value_u8);
		break;
		case BMA2x2_FLAT_INTR:
			/* Flat Interrupt */
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_FLAT_INTR, v_value_u8);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE1_REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr, BMA2x2_INTR_ENABLE2_REG,
		&v_data2_u8, C_BMA2x2_ONE_U8X);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_fifo_full(u8 *v_fifo_full_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_FIFO_FULL_ENABLE_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_fifo_full_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_INTR_FIFO_FULL_ENABLE_INTR);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_fifo_full(u8 v_fifo_full_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_fifo_full_u8 < C_BMA2x2_TWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_FIFO_FULL_ENABLE_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_INTR_FIFO_FULL_ENABLE_INTR,
			v_fifo_full_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_FIFO_FULL_ENABLE_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr_fifo_wm(u8 *v_fifo_wm_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_FIFO_WM_ENABLE_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_fifo_wm_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR_FIFO_WM_ENABLE_INTR);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr_fifo_wm(u8 v_fifo_wm_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_fifo_wm_u8 < C_BMA2x2_TWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_FIFO_WM_ENABLE_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_INTR_FIFO_WM_ENABLE_INTR,
			v_fifo_wm_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_FIFO_WM_ENABLE_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_slow_no_motion_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_SLOW_NO_MOTION_ENABLE_X:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_slow_no_motion_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR);
		break;
		case BMA2x2_SLOW_NO_MOTION_ENABLE_Y:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_slow_no_motion_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR);
		break;
		case BMA2x2_SLOW_NO_MOTION_ENABLE_Z:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_slow_no_motion_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR);
		break;
		case BMA2x2_SLOW_NO_MOTION_ENABLE_SELECT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_slow_no_motion_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_slow_no_motion_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_SLOW_NO_MOTION_ENABLE_X:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR,
			v_slow_no_motion_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_X_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOW_NO_MOTION_ENABLE_Y:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR,
			v_slow_no_motion_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Y_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOW_NO_MOTION_ENABLE_Z:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR,
			v_slow_no_motion_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_Z_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOW_NO_MOTION_ENABLE_SELECT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR,
			v_slow_no_motion_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR_SLOW_NO_MOTION_ENABLE_SELECT_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_low_g_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_LOW_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_low_g_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_LOW_G);
		break;
		case BMA2x2_INTR2_LOW_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_low_g_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_LOW_G);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_low_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_LOW_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_LOW_G, v_intr_low_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_LOW_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_LOW_G,
			v_intr_low_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_high_g_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_HIGH_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_high_g_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_HIGH_G);
		break;
		case BMA2x2_INTR2_HIGH_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_high_g_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_HIGH_G);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_high_g_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_HIGH_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_HIGH_G,
			v_intr_high_g_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_HIGH_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_HIGH_G,
			v_intr_high_g_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_slope_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_SLOPE:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_slope_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_SLOPE);
		break;
		case BMA2x2_INTR2_SLOPE:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_slope_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_SLOPE);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_slope_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_SLOPE:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_SLOPE,
			v_intr_slope_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_SLOPE:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_SLOPE,
			v_intr_slope_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
 *	v_channel_u8 -->BMA2x2_INTR1_SLOW_NO_MOTION  ->  0
 *					BMA2x2_INTR2_SLOW_NO_MOTION  ->  1
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
u8 *v_intr_slow_no_motion_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_SLOW_NO_MOTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_slow_no_motion_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION);
		break;
		case BMA2x2_INTR2_SLOW_NO_MOTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_slow_no_motion_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
 *	v_channel_u8 -->BMA2x2_INTR1_SLOW_NO_MOTION ->    0
 *					BMA2x2_INTR2_SLOW_NO_MOTION ->    1
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
u8 v_intr_slow_no_motion_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_SLOW_NO_MOTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION,
			v_intr_slow_no_motion_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_SLOW_NO_MOTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION,
			v_intr_slow_no_motion_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_double_tap_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_DOUBLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_double_tap_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP);
		break;
		case BMA2x2_INTR2_DOUBLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_double_tap_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_double_tap_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_DOUBLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP,
			v_intr_double_tap_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_DOUBLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_DOUBLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP,
			v_intr_double_tap_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_DOUBLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_single_tap_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_SINGLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_single_tap_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP);
		break;
		case BMA2x2_INTR2_SINGLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_single_tap_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_single_tap_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_SINGLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP,
			v_intr_single_tap_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_SINGLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_SINGLE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP,
			v_intr_single_tap_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_SINGLE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_orient_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_ORIENT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_ORIENT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_orient_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_ORIENT);
		break;
		case BMA2x2_INTR2_ORIENT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_ORIENT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_orient_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_ORIENT);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_orient_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_ORIENT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_ORIENT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_ORIENT, v_intr_orient_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_ORIENT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_ORIENT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_ORIENT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_ORIENT, v_intr_orient_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_ORIENT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_flat_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_FLAT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_flat_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_FLAT);
		break;
		case BMA2x2_INTR2_FLAT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_flat_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_FLAT);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_flat_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_FLAT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_FLAT, v_intr_flat_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_FLAT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_FLAT, v_intr_flat_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *intr_newdata_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_NEWDATA:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_NEWDATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*intr_newdata_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_NEWDATA);
		break;
		case BMA2x2_INTR2_NEWDATA:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_NEWDATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*intr_newdata_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_NEWDATA);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 intr_newdata_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_NEWDATA:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_NEWDATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_NEWDATA, intr_newdata_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_NEWDATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_NEWDATA:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_NEWDATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_NEWDATA, intr_newdata_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_NEWDATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr1_fifo_wm(u8 *v_intr1_fifo_wm_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FIFO_WM__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr1_fifo_wm_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_FIFO_WM);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr1_fifo_wm(u8 v_intr1_fifo_wm_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_intr1_fifo_wm_u8 < C_BMA2x2_TWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FIFO_WM__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR1_PAD_FIFO_WM, v_intr1_fifo_wm_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FIFO_WM__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr2_fifo_wm(u8 *v_intr2_fifo_wm_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_WM__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr2_fifo_wm_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_FIFO_WM);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr2_fifo_wm(u8 v_intr2_fifo_wm_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_intr2_fifo_wm_u8 < C_BMA2x2_TWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_WM__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_WM, v_intr2_fifo_wm_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_WM__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr1_fifo_full(u8 *v_intr1_fifo_full_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr1_fifo_full_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr1_fifo_full(u8 v_intr1_fifo_full_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_intr1_fifo_full_u8 < C_BMA2x2_TWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL,
			v_intr1_fifo_full_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR1_PAD_FIFO_FULL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			} else {
			com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_intr2_fifo_full(u8 *v_intr2_fifo_full_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr2_fifo_full_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_intr2_fifo_full(u8 v_intr2_fifo_full_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_intr2_fifo_full_u8 < C_BMA2x2_TWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL,
			v_intr2_fifo_full_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_INTR2_PAD_FIFO_FULL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			} else {
			com_rslt = E_OUT_OF_RANGE;
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_source_u8)
{
		u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_SOURCE_LOW_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_source_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_LOW_G);
		break;
		case BMA2x2_SOURCE_HIGH_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_source_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_HIGH_G);
		break;
		case BMA2x2_SOURCE_SLOPE:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_source_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_SLOPE);
		break;
		case BMA2x2_SOURCE_SLOW_NO_MOTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_source_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION);
		break;
		case BMA2x2_SOURCE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_source_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_TAP);
		break;
		case BMA2x2_SOURCE_DATA:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_DATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_source_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_DATA);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
			}
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_source_u8)
{
		u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_SOURCE_LOW_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_UNFILT_INTR_SOURCE_LOW_G, v_intr_source_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_LOW_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SOURCE_HIGH_G:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_UNFILT_INTR_SOURCE_HIGH_G, v_intr_source_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_HIGH_G__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SOURCE_SLOPE:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_UNFILT_INTR_SOURCE_SLOPE, v_intr_source_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_SLOPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SOURCE_SLOW_NO_MOTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION,
			v_intr_source_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_SLOW_NO_MOTION__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SOURCE_TAP:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_TAP,
			v_intr_source_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_TAP__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SOURCE_DATA:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_DATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_UNFILT_INTR_SOURCE_DATA,
			v_intr_source_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_UNFILT_INTR_SOURCE_DATA__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_output_type_u8)
{
		u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_OUTPUT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR1_PAD_OUTPUT_TYPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_output_type_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR1_PAD_OUTPUT_TYPE);
		break;
		case BMA2x2_INTR2_OUTPUT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR2_PAD_OUTPUT_TYPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_output_type_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR2_PAD_OUTPUT_TYPE);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_output_type_u8)
{
		u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		}  else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_OUTPUT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR1_PAD_OUTPUT_TYPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR1_PAD_OUTPUT_TYPE, v_intr_output_type_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR1_PAD_OUTPUT_TYPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_OUTPUT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR2_PAD_OUTPUT_TYPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR2_PAD_OUTPUT_TYPE, v_intr_output_type_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR2_PAD_OUTPUT_TYPE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_intr_level_u8)
{
		u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_LEVEL:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR1_PAD_ACTIVE_LEVEL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_level_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR1_PAD_ACTIVE_LEVEL);
		break;
		case BMA2x2_INTR2_LEVEL:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR2_PAD_ACTIVE_LEVEL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_intr_level_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_INTR2_PAD_ACTIVE_LEVEL);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_intr_level_u8)
{
		u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
		BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
		if (p_bma2x2 == BMA2x2_NULL) {
			return  E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_INTR1_LEVEL:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR1_PAD_ACTIVE_LEVEL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR1_PAD_ACTIVE_LEVEL, v_intr_level_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR1_PAD_ACTIVE_LEVEL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_INTR2_LEVEL:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR2_PAD_ACTIVE_LEVEL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_INTR2_PAD_ACTIVE_LEVEL, v_intr_level_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_INTR2_PAD_ACTIVE_LEVEL__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_rst_intr(u8 v_rst_intr_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_RESET_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_RESET_INTR, v_rst_intr_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_RESET_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_latch_intr(u8 *v_latch_intr_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LATCH_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_latch_intr_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_LATCH_INTR);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_latch_intr(u8 v_latch_intr_u8)
{
u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
u8 v_latch_durn_u8 = C_BMA2x2_ZERO_U8X;
if (p_bma2x2 == BMA2x2_NULL)  {
		return E_BMA2x2_NULL_PTR;
		} else  {
		if (v_latch_intr_u8 < C_BMA2x2_SIXTEEN_U8X) {
			switch (v_latch_intr_u8) {
			case BMA2x2_LATCH_DURN_NON_LATCH:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_NON_LATCH;

				/*  NON LATCH   */
			break;
			case BMA2x2_LATCH_DURN_250MS:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_250MS;

				/*  250 MS  */
			break;
			case BMA2x2_LATCH_DURN_500MS:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_500MS;

				/*  500 MS  */
			break;
			case BMA2x2_LATCH_DURN_1S:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_1S;

				/*  1 S   */
			break;
			case BMA2x2_LATCH_DURN_2S:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_2S;

				/*  2 S  */
			break;
			case BMA2x2_LATCH_DURN_4S:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_4S;

				/*  4 S  */
			break;
			case BMA2x2_LATCH_DURN_8S:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_8S;

				/*  8 S  */
			break;
			case BMA2x2_LATCH_DURN_LATCH:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_LATCH;

				/*  LATCH  */
			break;
			case BMA2x2_LATCH_DURN_NON_LATCH1:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_NON_LATCH1;

				/*  NON LATCH1  */
			break;
			case BMA2x2_LATCH_DURN_250US:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_250US;

				/*  250 US   */
			break;
			case BMA2x2_LATCH_DURN_500US:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_500US;

				/*  500 US   */
			break;
			case BMA2x2_LATCH_DURN_1MS:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_1MS;

				/*  1 MS   */
			break;
			case BMA2x2_LATCH_DURN_12_5MS:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_12_5MS;

				/*  12.5 MS   */
			break;
			case BMA2x2_LATCH_DURN_25MS:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_25MS;

				/*  25 MS   */
			break;
			case BMA2x2_LATCH_DURN_50MS:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_50MS;

				/*  50 MS   */
			break;
			case BMA2x2_LATCH_DURN_LATCH1:
				v_latch_durn_u8 = BMA2x2_LATCH_DURN_LATCH1;

				/*  LATCH1   */
			break;
			default:
			break;
			}
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LATCH_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_LATCH_INTR, v_latch_durn_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LATCH_INTR__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_durn_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_LOW_DURN:
			/*LOW DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_DURN_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_durn_u8 = v_data_u8;
		break;
		case BMA2x2_HIGH_DURN:
			/*HIGH DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_HIGH_DURN_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_durn_u8 = v_data_u8;
		break;
		case BMA2x2_SLOPE_DURN:
			/*SLOPE DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_SLOPE_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_durn_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_SLOPE_DURN);
		break;
		case BMA2x2_SLOW_NO_MOTION_DURN:
			/*SLO NO MOT DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOW_NO_MOTION_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_durn_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_SLOW_NO_MOTION_DURN);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_durn_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL)  {
		return E_BMA2x2_NULL_PTR;
		}  else  {
		switch (v_channel_u8)   {
		case BMA2x2_LOW_DURN:
			/*LOW DURATION*/
			v_data_u8 = v_durn_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_DURN_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_HIGH_DURN:
			/*HIGH DURATION*/
			v_data_u8 = v_durn_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_DURN_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOPE_DURN:
			/*SLOPE DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_SLOPE_DURN, v_durn_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOW_NO_MOTION_DURN:
			/*SLO NO MOT DURATION*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOW_NO_MOTION_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_SLOW_NO_MOTION_DURN, v_durn_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOW_NO_MOTION_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_thres_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_LOW_THRES:
			/*LOW THRESHOLD*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_THRES_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_thres_u8 = v_data_u8;
		break;
		case BMA2x2_HIGH_THRES:
			/*HIGH THRESHOLD*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_THRES_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_thres_u8 = v_data_u8;
		break;
		case BMA2x2_SLOPE_THRES:
			/*SLOPE THRESHOLD*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_THRES_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_thres_u8 = v_data_u8;
		break;
		case BMA2x2_SLOW_NO_MOTION_THRES:
			/*SLO NO MOT THRESHOLD*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOW_NO_MOTION_THRES_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_thres_u8 = v_data_u8;
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_thres_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_LOW_THRES:
			/*LOW THRESHOLD*/
			v_data_u8 = v_thres_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_THRES_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_HIGH_THRES:
			/*HIGH THRESHOLD*/
			v_data_u8 = v_thres_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_THRES_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOPE_THRES:
			/*SLOPE THRESHOLD*/
			v_data_u8 = v_thres_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOPE_THRES_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOW_NO_MOTION_THRES:
			/*SLO NO MOT THRESHOLD*/
			v_data_u8 = v_thres_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_SLOW_NO_MOTION_THRES_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_hyst_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_LOW_G_HYST:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_G_HYST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_hyst_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_LOW_G_HYST);
		break;
		case BMA2x2_HIGH_G_HYST:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_G_HYST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_hyst_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_HIGH_G_HYST);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_hyst_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_LOW_G_HYST:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_G_HYST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_LOW_G_HYST, v_hyst_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_G_HYST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_HIGH_G_HYST:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_G_HYST__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_HIGH_G_HYST, v_hyst_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_HIGH_G_HYST__REG,
			&v_data_u8,  C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	low_g  mode in the registers 0x24 bit 2
 *
 *
 *	\param u8 *v_low_g_mode_u8: Pointer holding the Low_G mode value
 *	v_low_g_mode_u8:
 *	0 -> single mode
 *	1 -> sum mode
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_low_g_mode(u8 *v_low_g_mode_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_LOW_G_INTR_MODE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_low_g_mode_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_LOW_G_INTR_MODE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	low_g  v_power_mode_u8 in the registers 0x24 bit 2
 *
 *
 *
 *	\param u8 v_low_g_mode_u8: The Low g  mode value
 *	v_low_g_mode_u8:
 *	0 -> single mode
 *	1 -> sum mode
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_low_g_mode(u8 v_low_g_mode_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_G_INTR_MODE__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_LOW_G_INTR_MODE, v_low_g_mode_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_LOW_G_INTR_MODE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_durn(u8 *v_tap_durn_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_DURN__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_tap_durn_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_TAP_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_durn(u8 v_tap_durn_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_TAP_DURN__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_TAP_DURN, v_tap_durn_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_DURN__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_shock(u8 *v_tap_shock_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SHOCK_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_tap_shock_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_TAP_SHOCK_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_shock(u8 v_tap_shock_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SHOCK_DURN__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_TAP_SHOCK_DURN, v_tap_shock_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SHOCK_DURN__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_quiet(u8 *v_tap_quiet_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_QUIET_DURN__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			*v_tap_quiet_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_TAP_QUIET_DURN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_quiet(u8 v_tap_quiet_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_QUIET_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_TAP_QUIET_DURN, v_tap_quiet_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_QUIET_DURN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_thres(u8 *v_tap_thres_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_THRES__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_tap_thres_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_TAP_THRES);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_thres(u8 v_tap_thres_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_THRES__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_TAP_THRES, v_tap_thres_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_THRES__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_tap_sample(u8 *v_tap_sample_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SAMPLES__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_tap_sample_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_TAP_SAMPLES);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_tap_sample(u8 v_tap_sample_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_TAP_SAMPLES__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_TAP_SAMPLES, v_tap_sample_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_TAP_SAMPLES__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the orient mode in the register 0x2C bit 0 and 1
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_mode(u8 *v_orient_mode_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_ORIENT_MODE__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_orient_mode_u8 = BMA2x2_GET_BITSLICE(
			v_data_u8, BMA2x2_ORIENT_MODE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/****************************************************************************
 *	Description: *//**\brief This API is used to set
 *	the orient mode in the register 0x2C bit 0 and 1
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_mode(u8 v_orient_mode_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_MODE__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ORIENT_MODE, v_orient_mode_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_MODE__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_orient_block_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_BLOCK__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_orient_block_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ORIENT_BLOCK);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_block(u8 v_orient_block_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_BLOCK__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ORIENT_BLOCK, v_orient_block_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_BLOCK__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_hyst(u8 *v_orient_hyst_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_HYST__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_orient_hyst_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ORIENT_HYST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_hyst(u8 v_orient_hyst_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_HYST__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ORIENT_HYST, v_orient_hyst_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_ORIENT_HYST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_theta_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_ORIENT_THETA:
			/*ORIENT THETA*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_BLOCK__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			*v_theta_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_THETA_BLOCK);
		break;
		case BMA2x2_FLAT_THETA:
			/*FLAT THETA*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_theta_u8 = v_data_u8;
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_theta_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_ORIENT_THETA:
			/*ORIENT THETA*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_BLOCK__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_THETA_BLOCK, v_theta_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_BLOCK__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_FLAT_THETA:
			/*FLAT THETA*/
			v_data_u8 = v_theta_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_THETA_FLAT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_orient_enable(u8 *v_orient_enableable_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_UD_ENABLE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_orient_enableable_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ORIENT_UD_ENABLE);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_orient_enable(u8 v_orient_enableable_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_UD_ENABLE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_ORIENT_UD_ENABLE, v_orient_enableable_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ORIENT_UD_ENABLE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_flat_hyst(u8 *v_flat_hyst_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HYST__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_flat_hyst_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_FLAT_HYST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_flat_hyst(u8 v_flat_hyst_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FLAT_HYST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_FLAT_HYST, v_flat_hyst_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HYST__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_flat_hold_time_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HOLD_TIME__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_flat_hold_time_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_FLAT_HOLD_TIME);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_flat_hold_time_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HOLD_TIME__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_FLAT_HOLD_TIME, v_flat_hold_time_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FLAT_HOLD_TIME__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *fifo_wml_trig)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_WML_TRIG_RETAIN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*fifo_wml_trig = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_FIFO_WML_TRIG_RETAIN);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 fifo_wml_trig)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (fifo_wml_trig < C_BMA2x2_THIRTYTWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_WML_TRIG_RETAIN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_FIFO_WML_TRIG_RETAIN,
			fifo_wml_trig);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_WML_TRIG_RETAIN__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_selftest_axis_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SELFTEST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_selftest_axis_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SELFTEST);
		}
	return com_rslt;
}
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
u8 v_selftest_axis_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_selftest_axis_u8 < C_BMA2x2_FOUR_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SELFTEST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SELFTEST, v_selftest_axis_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SELFTEST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		 } else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_selftest_sign_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELFTEST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_selftest_sign_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_NEG_SELFTEST);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_selftest_sign_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_selftest_sign_u8 < C_BMA2x2_TWO_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELFTEST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_NEG_SELFTEST, v_selftest_sign_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_NEG_SELFTEST__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to get
 * the v_intr_stat_u8us of nvm program mode(nvm_prog_mode)in the register 0x33 bit 0
 *
 *
 *
 *
 *  \param  u8 *v_nvmprog_mode_u8 : Pointer holding the nvmprog_mode
 *	v_nvmprog_mode_u8
 *	1 -> Enable program mode
 *	0 -> Disable program mode
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
u8 *v_nvmprog_mode_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return  E_BMA2x2_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_UNLOCK_EE_PROG_MODE__REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		*v_nvmprog_mode_u8 = BMA2x2_GET_BITSLICE
		(v_data_u8, BMA2x2_UNLOCK_EE_PROG_MODE);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvmprog_mode(u8 v_nvmprog_mode_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_UNLOCK_EE_PROG_MODE__REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		v_data_u8 = BMA2x2_SET_BITSLICE
		(v_data_u8, BMA2x2_UNLOCK_EE_PROG_MODE, v_nvmprog_mode_u8);
		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_UNLOCK_EE_PROG_MODE__REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_nvprog_trig(u8 v_nvprog_trig_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_START_EE_PROG_TRIG__REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		v_data_u8 = BMA2x2_SET_BITSLICE
		(v_data_u8, BMA2x2_START_EE_PROG_TRIG, v_nvprog_trig_u8);
		com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_START_EE_PROG_TRIG__REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_ready(u8 *v_nvprog_ready_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_EE_PROG_READY__REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		*v_nvprog_ready_u8 = BMA2x2_GET_BITSLICE
		(v_data_u8, BMA2x2_EE_PROG_READY);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_nvmprog_remain(u8 *v_nvprog_remain_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
	} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_EE_REMAIN__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		*v_nvprog_remain_u8 = BMA2x2_GET_BITSLICE
		(v_data_u8, BMA2x2_EE_REMAIN);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_spi3(u8 *v_spi3_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SPI_MODE_3__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_spi3_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SPI_MODE_3);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_spi3(u8 v_spi3_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SPI_MODE_3__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SPI_MODE_3, v_spi3_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SPI_MODE_3__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_i2c_wdt_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_I2C_SELECT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_I2C_WDT_PERIOD__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_i2c_wdt_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_I2C_WDT_PERIOD);
		break;
		case BMA2x2_I2C_EN:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_I2C_WDT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_i2c_wdt_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_I2C_WDT);
		break;
		default:
		com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 * Description: *//**\brief This API is used to set the value of i2c
 *	wdt selection(i2c_wdt_sel) and wdt enable(i2c_wdt_enable) in the register
 *	0x36 bit 1 and 2
 *
 *
 *	\param u8 v_channel_u8: The value of i2c wdt channel number
 *	v_channel_u8 --> BMA2x2_I2C_SELECT          0
 *				BMA2x2_I2C_EN              1
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
u8 v_i2c_wdt_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_I2C_SELECT:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_I2C_WDT_PERIOD__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_I2C_WDT_PERIOD, v_i2c_wdt_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_I2C_WDT_PERIOD__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_I2C_EN:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_I2C_WDT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_I2C_WDT, v_i2c_wdt_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_I2C_WDT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_slow_comp_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_SLOW_COMP_X:
			/*SLOW COMP X*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_X__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_slow_comp_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SLOW_COMP_X);
		break;
		case BMA2x2_SLOW_COMP_Y:
			/*SLOW COMP Y*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_Y__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_slow_comp_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SLOW_COMP_Y);
		break;
		case BMA2x2_SLOW_COMP_Z:
			/*SLOW COMP Z*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_Z__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_slow_comp_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_ENABLE_SLOW_COMP_Z);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_slow_comp_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_SLOW_COMP_X:
			/*SLOW COMP X*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_X__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_SLOW_COMP_X, v_slow_comp_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_X__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOW_COMP_Y:
			/*SLOW COMP Y*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_Y__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_SLOW_COMP_Y, v_slow_comp_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_Y__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_SLOW_COMP_Z:
			/*SLOW COMP Z*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_Z__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_ENABLE_SLOW_COMP_Z, v_slow_comp_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_ENABLE_SLOW_COMP_Z__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_cal_rdy(u8 *v_cal_rdy_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
		(p_bma2x2->dev_addr,
		BMA2x2_FAST_CAL_RDY_STAT__REG,
		&v_data_u8, C_BMA2x2_ONE_U8X);
		*v_cal_rdy_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
		BMA2x2_FAST_CAL_RDY_STAT);
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_cal_trigger(u8 v_cal_trigger_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_CAL_TRIGGER__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_CAL_TRIGGER, v_cal_trigger_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_CAL_TRIGGER__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_offset_rst(u8 v_offset_rst_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RST_OFFSET__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_RST_OFFSET,
			v_offset_rst_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_RST_OFFSET__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_offset_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_CUT_OFF:
			/*CUT-OFF*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_CUTOFF__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			*v_offset_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_COMP_CUTOFF);
		break;
		case BMA2x2_OFFSET_TRIGGER_X:
			/*OFFSET TRIGGER X*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_X__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_offset_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_COMP_TARGET_OFFSET_X);
		break;
		case BMA2x2_OFFSET_TRIGGER_Y:
			/*OFFSET TRIGGER Y*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Y__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_offset_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_COMP_TARGET_OFFSET_Y);
		break;
		case BMA2x2_OFFSET_TRIGGER_Z:
			/*OFFSET TRIGGER Z*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Z__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_offset_u8 = BMA2x2_GET_BITSLICE
			(v_data_u8, BMA2x2_COMP_TARGET_OFFSET_Z);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_offset_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_CUT_OFF:
			/*CUT-OFF*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_CUTOFF__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_COMP_CUTOFF, v_offset_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_CUTOFF__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_OFFSET_TRIGGER_X:
			/*OFFSET TARGET X*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_X__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_COMP_TARGET_OFFSET_X, v_offset_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_X__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_OFFSET_TRIGGER_Y:
			/*OFFSET TARGET Y*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Y__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_COMP_TARGET_OFFSET_Y, v_offset_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Y__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_OFFSET_TRIGGER_Z:
			/*OFFSET TARGET Z*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Z__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8, BMA2x2_COMP_TARGET_OFFSET_Z, v_offset_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_COMP_TARGET_OFFSET_Z__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s8 *v_offset_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_X_AXIS:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_X_AXIS_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_offset_u8 = (s8)v_data_u8;
		break;
		case BMA2x2_Y_AXIS:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Y_AXIS_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_offset_u8 = (s8)v_data_u8;
		break;
		case BMA2x2_Z_AXIS:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Z_AXIS_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			*v_offset_u8 = (s8)v_data_u8;
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
s8 v_offset_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (v_channel_u8) {
		case BMA2x2_X_AXIS:
			v_data_u8 = v_offset_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_X_AXIS_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_Y_AXIS:
			v_data_u8 = v_offset_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Y_AXIS_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		case BMA2x2_Z_AXIS:
			v_data_u8 = v_offset_u8;
			com_rslt = p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_OFFSET_Z_AXIS_REG, &v_data_u8, C_BMA2x2_ONE_U8X);
		break;
		default:
			com_rslt = E_OUT_OF_RANGE;
		break;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/***************************************************************************
 *	Description: *//**\brief This API is used to get
 *	the v_intr_stat_u8us of fifo (fifo_mode) in the register 0x3E bit 6 and 7
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_get_fifo_mode(u8 *v_fifo_mode_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			*v_fifo_mode_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_FIFO_MODE);
		}
	return com_rslt;
}
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_set_fifo_mode(u8 v_fifo_mode_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_config_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_fifo_mode_u8 < C_BMA2x2_FOUR_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE__REG, &v_data_u8, C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE(v_data_u8,
			BMA2x2_FIFO_MODE, v_fifo_mode_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_MODE__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			if (com_rslt == BMA2x2_SUCCESS) {
				com_rslt += bma2x2_read_reg(
				BMA2x2_FIFO_MODE_REG,
				&v_config_data_u8, 1);
				p_bma2x2->fifo_config = v_config_data_u8;
			}
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_fifo_data_select_u8)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_fifo_data_select_u8 = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_FIFO_DATA_SELECT);
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 v_fifo_data_select_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_config_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		if (v_fifo_data_select_u8 < C_BMA2x2_FOUR_U8X) {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT__REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			v_data_u8 = BMA2x2_SET_BITSLICE
			(v_data_u8,
			BMA2x2_FIFO_DATA_SELECT, v_fifo_data_select_u8);
			com_rslt += p_bma2x2->BMA2x2_BUS_WRITE_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_SELECT__REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			if (com_rslt == BMA2x2_SUCCESS) {
				com_rslt += bma2x2_read_reg(
				BMA2x2_FIFO_MODE_REG,
				 &v_config_data_u8, 1);
				p_bma2x2->fifo_config = v_config_data_u8;
			}
		} else {
		com_rslt = E_OUT_OF_RANGE;
		}
	}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
u8 *v_output_reg_u8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			/*GET FIFO DATA OUTPUT REGISTER*/
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_FIFO_DATA_OUTPUT_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_output_reg_u8 = v_data_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
BMA2x2_RETURN_FUNCTION_TYPE bma2x2_read_temp(s8 *v_temp_s8)
{
	u8 v_data_u8 = C_BMA2x2_ZERO_U8X;
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_TEMP_REG,
			&v_data_u8, C_BMA2x2_ONE_U8X);
			*v_temp_s8 = (s8)v_data_u8;
		}
	return com_rslt;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
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
struct bma2x2_accel_data_temp *accel)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8 v_data_u8[7] = {0, 0, 0, 0, 0, 0, 0};
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		case BMA2x2_12_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X12_LSB__REG,
			v_data_u8, 7);

			/* read x v_data_u8*/
			accel->x = (s16)((((s32)((s8)v_data_u8[1]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[0] & 0xF0));
			accel->x = accel->x >> C_BMA2x2_FOUR_U8X;

			/* read y v_data_u8*/
			accel->y = (s16)((((s32)((s8)v_data_u8[3]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[2] & 0xF0));
			accel->y = accel->y >> C_BMA2x2_FOUR_U8X;

			/* read z v_data_u8*/
			accel->z = (s16)((((s32)((s8)v_data_u8[5]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[4] & 0xF0));
			accel->z = accel->z >> C_BMA2x2_FOUR_U8X;

			accel->temp = (s8)v_data_u8[6];
		break;
		case BMA2x2_10_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X10_LSB__REG,
			v_data_u8, 7);

			/* read x v_data_u8*/
			accel->x = (s16)((((s32)((s8)v_data_u8[1]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[0] & 0xC0));
			accel->x = accel->x >> C_BMA2x2_SIX_U8X;

			/* read y v_data_u8*/
			accel->y = (s16)((((s32)((s8)v_data_u8[3]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[2] & 0xC0));
			accel->y = accel->y >> C_BMA2x2_SIX_U8X;

			/* read z v_data_u8*/
			accel->z = (s16)((((s32)((s8)v_data_u8[5]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[4] & 0xC0));
			accel->z = accel->z >> C_BMA2x2_SIX_U8X;

			/* read v_temp_s8 v_data_u8*/
			accel->temp = (s8)v_data_u8[6];
		break;
		case BMA2x2_14_RESOLUTION:
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr, BMA2x2_ACCEL_X14_LSB__REG,
			v_data_u8, 7);

			/* read x v_data_u8*/
			accel->x = (s16)((((s32)((s8)v_data_u8[1]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[0] & 0xFC));
			accel->x = accel->x >> C_BMA2x2_TWO_U8X;

			/* read y v_data_u8*/
			accel->y = (s16)((((s32)((s8)v_data_u8[3]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[2] & 0xFC));
			accel->y = accel->y >> C_BMA2x2_TWO_U8X;

			/* read z v_data_u8*/
			accel->z = (s16)((((s32)((s8)v_data_u8[5]))
			<< C_BMA2x2_EIGHT_U8X) | (v_data_u8[4] & 0xFC));
			accel->z = accel->z >> C_BMA2x2_TWO_U8X;

			/* read temp v_data_u8*/
			accel->temp = (s8)v_data_u8[6];
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
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
struct bma2x2_accel_eight_resolution_temp *accel)
{
	BMA2x2_RETURN_FUNCTION_TYPE com_rslt = BMA2x2_ERROR;
	u8	v_data_u8 = C_BMA2x2_ZERO_U8X;
	if (p_bma2x2 == BMA2x2_NULL) {
		return E_BMA2x2_NULL_PTR;
		} else {
			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_X_AXIS_MSB_REG, &v_data_u8, 1);
			accel->x = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_ACCEL_X_MSB);

			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Y_AXIS_MSB_REG, &v_data_u8, 1);
			accel->y = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_ACCEL_Y_MSB);

			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC
			(p_bma2x2->dev_addr,
			BMA2x2_Z_AXIS_MSB_REG, &v_data_u8, 1);
			accel->z = BMA2x2_GET_BITSLICE(v_data_u8,
			BMA2x2_ACCEL_Z_MSB);

			com_rslt = p_bma2x2->BMA2x2_BUS_READ_FUNC(
			p_bma2x2->dev_addr,
			BMA2x2_TEMP_REG, &v_data_u8,
			C_BMA2x2_ONE_U8X);
			accel->temp = (s8)v_data_u8;
		}
	return com_rslt;
}
