/*
 * This file is part of the MW21 adaption for BMF055.
 *
 * BMF055 flight controller is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BMF055 flight controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BMF055 flight controller.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*!
 *
 * @file		bmf055.c
 * @author	Bosch Sensortec
 *
 * @brief	Functions declared in bmf055.h file are defined here.
 *
 *
 */

/************************************************************************/
/* Include Own Header                                                   */
/************************************************************************/

#include "bmf055.h"

/************************************************************************/
/* Local Variables                                                     */
/************************************************************************/

/*! This structure holds acceleration data of x, y and z axes. */
static struct bma2x2_accel_data bma2x2_accel_data;
/*! This structure holds angular velocity data of x, y and z axes. */
static struct bmg160_data_t bmg160_gyro_data;
/*! This structure holds magnetic field data of x, y and z axes. */
static struct bmm050_mag_data_s16_t bmm050_mag_data;


/************************************************************************/
/* Function Definitions                                                 */
/************************************************************************/

/*!
 * @brief		Initializes the internal sensors of BMF055.
 *
 * @param [in]	NULL
 *
 * @param [out]	NULL
 *
 * @return		NULL
 *
 */
void bmf055_sensors_initialize (void){
	/* Initialize BMA280 */
	bma_init();

	/* Initialize BMG160 */
	bmg_init();

	/* Initialize BMM150 */
	//bmm_init();
}


/*!
 * @brief		Reads output data of the internal sensors and sends sensor data via USART.
 *
 * @param [in]	NULL
 *
 * @param [out]	NULL
 *
 * @return		NULL
 *
 */
void bmf055_sensors_data_print (void)
{
	/* Read accelerometer's data */
	bma2x2_read_accel_xyz(&bma2x2_accel_data);

	/* Read gyroscope's data */
	bmg160_get_data_XYZ(&bmg160_gyro_data);

	/* Read magnetometer's data */
	bmm050_get_raw_xyz(&bmm050_mag_data);

	uint8_t usart_buffer_tx[81] = {0};

	/* Convert integer values to string and form the packet to be sent on USART */
	sprintf((char *)usart_buffer_tx, "\r\nAcc:%6d %6d %6d\r\nGyr:%6d %6d %6d\r\nMag:%6d %6d %6d\r\n",
			bma2x2_accel_data.x, bma2x2_accel_data.y, bma2x2_accel_data.z,
			bmg160_gyro_data.datax, bmg160_gyro_data.datay, bmg160_gyro_data.dataz,
			bmm050_mag_data.datax, bmm050_mag_data.datay, bmm050_mag_data.dataz);

	/* Print data on USART */
	usart_write_buffer_wait(&usart_instance, usart_buffer_tx,sizeof(usart_buffer_tx));
}


/*!
 * @brief		Processes USART input commands.
 *
 * @param [in]	bmf055_usart_input_buf	Holds the received byte to be processed
 *
 * @param [out]	NULL
 *
 * @return		NULL
 *
 */
void bmf055_usart_read_process(uint16_t bmf055_usart_input_buf)
{
	/* These variables hold intermediate values to update sensors' configuration */
	static uint8_t acc_range;
	static uint8_t acc_bw;
	static uint8_t acc_mode;
	static uint8_t gyr_range;
	static uint8_t gyr_bw;
	static uint8_t gyr_mode;
	static uint8_t mag_preset;
	static uint8_t mag_bw;
	static uint8_t mag_mode;

	usart_write_wait(&usart_instance, '\r');
	usart_write_wait(&usart_instance, '\n');

	/* This switch-case implements the state machine to process input commands */
	switch (bmf055_input_state)
	{
	case USART_INPUT_STATE_PRINT_DATA:
		if (bmf055_usart_input_buf == 0)
		{
			tc6_stop_counter();
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Stop Stream\r\n",13);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_STOPPED:
		if(bmf055_usart_input_buf == 1)
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Start Stream\r\n",14);
			tc6_start_counter();
			bmf055_input_state = USART_INPUT_STATE_PRINT_DATA;
		}
		else if (bmf055_usart_input_buf == 0xAA)
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Acc Range:\r\n",12);
			bmf055_input_state = USART_INPUT_STATE_ACC_RANGE;
		}
		else if (bmf055_usart_input_buf == 0xBB)
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Gyr Range:\r\n",12);
			bmf055_input_state = USART_INPUT_STATE_GYR_RANGE;
		}
		else if (bmf055_usart_input_buf == 0xCC)
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Mag Preset:\r\n",13);
			bmf055_input_state = USART_INPUT_STATE_MAG_PRESET;
		}
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_ACC_RANGE:
		if (bmf055_usart_input_buf == BMA2x2_RANGE_2G
				|| bmf055_usart_input_buf == BMA2x2_RANGE_4G
				|| bmf055_usart_input_buf == BMA2x2_RANGE_8G
				|| bmf055_usart_input_buf == BMA2x2_RANGE_16G
				|| bmf055_usart_input_buf == 0xFF)
		{
			acc_range = bmf055_usart_input_buf;
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Acc BW:\r\n",9);
			bmf055_input_state = USART_INPUT_STATE_ACC_BW;
		}
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_ACC_BW:
		if ((bmf055_usart_input_buf >= BMA2x2_BW_7_81HZ && bmf055_usart_input_buf <= BMA2x2_BW_1000HZ)
				|| bmf055_usart_input_buf == 0xFF)
		{
			acc_bw = bmf055_usart_input_buf;
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Acc Mode:\r\n",11);
			bmf055_input_state = USART_INPUT_STATE_ACC_MODE;
		}
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_ACC_MODE:
		if ((bmf055_usart_input_buf >= BMA2x2_MODE_NORMAL && bmf055_usart_input_buf <= BMA2x2_MODE_STANDBY)
				|| bmf055_usart_input_buf == 0xFF)
		{
			acc_mode = bmf055_usart_input_buf;
			if (acc_range != 0xFF)
			{
				bma2x2_set_range(acc_range);
			}
			if (acc_bw != 0xFF)
			{
				bma2x2_set_bw(acc_bw);
			}
			if (acc_mode != 0xFF)
			{
				bma2x2_set_power_mode(acc_mode);
			}			
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Acc Updated!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_GYR_RANGE:
		if ((bmf055_usart_input_buf >= 0x1 && bmf055_usart_input_buf <= 0x5) || bmf055_usart_input_buf == 0xFF)
		{
			gyr_range = bmf055_usart_input_buf;
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Gyr BW:\r\n", 9);
			bmf055_input_state = USART_INPUT_STATE_GYR_BW;

		}
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_GYR_BW:
		if ((bmf055_usart_input_buf >= C_BMG160_NO_FILTER_U8X && bmf055_usart_input_buf <= C_BMG160_BW_32HZ_U8X)
				|| bmf055_usart_input_buf == 0xFF)
		{
			gyr_bw = bmf055_usart_input_buf;
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Gyr Mode:\r\n", 11);
			bmf055_input_state = USART_INPUT_STATE_GYR_MODE;
		}
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_GYR_MODE:
		if ((bmf055_usart_input_buf >= BMG160_MODE_NORMAL && bmf055_usart_input_buf <= BMG160_MODE_ADVANCEDPOWERSAVING)
				|| bmf055_usart_input_buf == 0xFF)
		{
			gyr_mode = bmf055_usart_input_buf;
			if (gyr_range != 0xFF)
			{
				bmg160_set_range_reg(gyr_range);
			}
			if (gyr_bw != 0xFF)
			{
				bmg160_set_bw(gyr_bw);
			}
			if (gyr_mode != 0xFF)
			{
				bmg160_set_power_mode(gyr_mode);
			}
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Gyr Updated!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		} 
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_MAG_PRESET:
		if ((bmf055_usart_input_buf >= BMM050_PRESETMODE_LOWPOWER && BMM050_PRESETMODE_ENHANCED <= 4)
				|| bmf055_usart_input_buf == 0xFF)
		{
			mag_preset = bmf055_usart_input_buf;
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Mag BW:\r\n", 9);
			bmf055_input_state = USART_INPUT_STATE_MAG_BW;
		} 
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_MAG_BW:
		if ((bmf055_usart_input_buf >= BMM050_DR_10HZ && bmf055_usart_input_buf <= BMM050_DR_30HZ)
				|| bmf055_usart_input_buf == 0xFF)
		{
			mag_bw = bmf055_usart_input_buf;
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Mag Mode:\r\n", 11);
			bmf055_input_state = USART_INPUT_STATE_MAG_MODE;

		}
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;

	case USART_INPUT_STATE_MAG_MODE:
		if ((bmf055_usart_input_buf >= BMM050_NORMAL_MODE && bmf055_usart_input_buf <= BMM050_SLEEP_MODE)
				|| bmf055_usart_input_buf == 0xFF)
		{
			mag_mode = bmf055_usart_input_buf;

			if (mag_bw != 0xFF)
			{
				/* datarate rewritten if a preset mode is selected */
				bmm050_set_data_rate(mag_bw);
			}
			if (mag_preset != 0xFF)
			{
				bmm050_set_presetmode(mag_preset);
			}
			if (mag_mode != 0xFF)
			{
				bmm050_set_functional_state(mag_mode);
			}
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Mag Updated!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		} 
		else
		{
			usart_write_buffer_wait(&usart_instance, (uint8_t *)"Wrong Input!\r\n",14);
			bmf055_input_state = USART_INPUT_STATE_STOPPED;
		}
		break;
	}
}

