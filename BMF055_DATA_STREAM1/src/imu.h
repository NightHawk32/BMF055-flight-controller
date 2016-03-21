/*
 * imu.h
 *
 * Created: 06.03.2016 19:23:41
 *  Author: Lukas
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
