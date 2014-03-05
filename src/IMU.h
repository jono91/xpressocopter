/*
 * IMU.h
 *
 *  Created on: 19/03/2013
 *      Author: Jono
 */

#ifndef IMU_H_
#define IMU_H_
#include "type.h"
#include "config.h"
#include "def.h"


#define PI 3.14159265359




typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;



void computeIMU () ;

#ifdef MAYHONY
float _atan2(float y, float x);
#else
int16_t _atan2(float y, float x);
#endif

void rotateV(struct fp_vector *v,float* delta) ;
void getEstimatedAttitude();
void MayhonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MayhonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);
static void getAttitude(void);
uint8_t getEstimatedAltitude();


#endif /* IMU_H_ */
