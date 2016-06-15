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

#ifndef IMU_H_
#define IMU_H_


void computeIMU (void);


typedef struct {
	float X;
	float Y;
	float Z;
} t_fp_vector_def,fp_vector;

typedef union {
	float   A[3];
	t_fp_vector_def V;
} t_fp_vector;

int16_t _atan2(float y, float x);
void rotateV(fp_vector *v,float* delta);
void getEstimatedAttitude(void);
float InvSqrt (float x);
float fsq(float x);

#endif
