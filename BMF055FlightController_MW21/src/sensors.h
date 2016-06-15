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

#ifndef SENSORS_H_
#define SENSORS_H_

#include "bma2x2.h"
#include "bmg160.h"
  
void GYRO_Common(void);
void ACC_Common(void);

void Gyro_init(void);
void Gyro_getADC (void);
void ACC_init (void);
void ACC_getADC (void);
void initSensors(void);

#endif
