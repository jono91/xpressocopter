/*
 * IMU.c
 *
 *  Created on: 19/03/2013
 *      Author: Jono
 */
#include "IMU.h"
#include "sensors.h"
#include "microsec.h"
#include "common.h"
#include "math.h"
#include "serial.h"
#include "gpio.h"
#include "medianFilter.h"

static void getAttitude(void);

#ifdef SONAR
extern filterHistory_t SonarFilter;
#endif

//#undef MAG

extern struct flags_struct f;
extern struct conf_def conf;

extern uint32_t currentTime;

extern uint16_t acc_1G;             // this is the 1G measured acceleration
extern int16_t  acc_25deg;
extern int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];

#ifdef MAYHONY
#define AccFactorDef	(5.0f)	// 2 * proportional gain
#define MagFactorDef (1.0f)

#ifdef MAG
	#define twoKpDef (2.0f * 0.4f)
	#define twoKpDefMin (2.0f * 0.015f)//0.12
#else
	#define twoKpDef (2.0f * 0.50f)
	#define twoKpDefMin (2.0f * 0.03f)
#endif

#define twoKiDef	(2.0f * 0.008f)	// 2 * integral gain
#define GYRO_SCALE_MAYHONY ((2000 * PI)/((32767.0f / 4.0f ) * 180.0f))

extern int16_t gyroAHRS[3];
volatile float AccFactor = AccFactorDef;
volatile float MagFactor = MagFactorDef;
volatile float twoKp = twoKpDef;// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float samplePeriod = 0;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float halfvx, halfvy, halfvz; // Estimated direction of gravity and vector perpendicular to magnetic flux from quaterinion
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
volatile uint32_t  convergedT = 0;
volatile uint32_t accelT = 0;
	#ifdef AHRSDEBUG
		volatile uint32_t updateTime = 0;
	#endif
#endif


extern int16_t  annex650_overrun_count;

extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t gyroData[3];
extern int16_t gyroZero[3];
extern int16_t angle[2];  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
extern int16_t  heading;
extern int16_t debug[4];

extern int32_t BaroAlt;
extern int32_t  EstAlt;
extern int32_t  AltHold;
extern int16_t  errorAltitudeI;
extern int16_t  BaroPID;
extern int16_t  vario;              // variometer in cm/s
extern int16_t  sonarAlt;

#if BARO
  extern int32_t baroPressure;
  extern int32_t baroTemperature;
  extern int32_t baroPressureSum;
#endif

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  #if defined(NUNCHUCK)
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    ACC_getADC();
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    f.NUNCHUKDATA = 1;
    while(f.NUNCHUKDATA) ACC_getADC(); // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis])/4;
      gyroADCprevious[axis] = gyroADC[axis];
    }
  #else
    #if ACC
      ACC_getADC();
      getEstimatedAttitude();
    #endif
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();

    if ((micros()-timeInterleave)>650) {
       annex650_overrun_count++;
    } else {
       while((micros()-timeInterleave)<650) ; //empirical, interleaving delay between 2 consecutive reads
    }
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      if (!ACC) accADC[axis]=0;
    }
  #endif
  #if defined(GYRO_SMOOTHING)
    static int16_t gyroSmooth[3] = {0,0,0};
    for (axis = 0; axis < 3; axis++) {
      gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis]-1) )+gyroData[axis]+1 ) / conf.Smoothing[axis]);
      gyroSmooth[axis] = gyroData[axis];
    }
  #elif defined(TRI)
    static int16_t gyroYawSmooth = 0;
    gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW])/3;
    gyroYawSmooth = gyroData[YAW];
  #endif
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 100.0f
#endif

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef MG_LPF_FACTOR
  //#define MG_LPF_FACTOR 4
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 400.0f
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#ifndef GYR_CMPFM_FACTOR
  #define GYR_CMPFM_FACTOR 200.0f
#endif

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
  #define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
  // +-2000/sec deg scale
  //#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
  #define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

#ifdef obsolete
//Hastings approximation for atan2()
#ifdef MAYHONY
float _atan2(float y, float x){
#else
int16_t _atan2(float y, float x){
#endif
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = fabs((int16_t)(z * 100));
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10)
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
#ifndef MAYHONY
  z *= (180.0f / PI * 10);
#endif
  return z;
}
#endif

#ifndef MAYHONY
// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  struct fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}
#endif

void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;

#ifndef MAYHONY
  static t_fp_vector EstG;
	#if MAG
	  static t_fp_vector EstM;
	#endif
#endif

#if defined(MG_LPF_FACTOR)
  static int16_t mgSmooth[3];
#endif

#if defined(ACC_LPF_FACTOR)
  static float accLPF[3];
#endif

  static uint32_t previousT;
  uint32_t currentT = micros();

#ifndef MAYHONY
  float scale;
  float deltaGyroAngle[3];

  scale = (currentT - previousT) * GYRO_SCALE;
#else
  float GyroRate[3];
  samplePeriod = (currentT-previousT)/1000000.0f;
#endif


  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
#ifndef MAYHONY
    deltaGyroAngle[axis] = gyroADC[axis]  * scale;
#endif
    #if defined(ACC_LPF_FACTOR)
      accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
      accSmooth[axis] = accLPF[axis];
      #define ACC_VALUE accSmooth[axis]
    #else
      accSmooth[axis] = accADC[axis];
      #define ACC_VALUE accADC[axis]
    #endif
    accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);
    accMag += (int32_t)ACC_VALUE*ACC_VALUE ;
    #if MAG
      #if defined(MG_LPF_FACTOR)
        mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
        #define MAG_VALUE mgSmooth[axis]
      #else
        #define MAG_VALUE magADC[axis]
      #endif
    #endif
  }
  accMag = accMag*100/((int32_t)acc_1G*acc_1G);



#if defined(MAYHONY)
  if(calibratingA == 0 && calibratingG == 0)
  {
	  for(axis = 0; axis < 3; axis++)
	  {
		  GyroRate[axis] = gyroAHRS[axis]*GYRO_SCALE_MAYHONY;
	  }
		#ifdef MAG
		  MayhonyAHRSupdate(GyroRate[Xaxis], GyroRate[Yaxis], GyroRate[Zaxis], (float)accSmooth[Xaxis], (float)accSmooth[Yaxis], (float)accSmooth[Zaxis], (float)magADC[Xaxis], (float)magADC[Yaxis], (float)magADC[Zaxis]);
		#else
		  MayhonyAHRSupdateIMU(GyroRate[Xaxis], GyroRate[Yaxis], GyroRate[Zaxis], (float)accSmooth[Xaxis], (float)accSmooth[Yaxis], (float)accSmooth[Zaxis]);
		#endif
  }

#else
  rotateV(&EstG.V,deltaGyroAngle);
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);
  #endif

  if ( fabs(accSmooth[ROLL])<acc_25deg && fabs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro

	  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 )
		for (axis = 0; axis < 3; axis++) {
		  int16_t acc = ACC_VALUE;
		  EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
		}
	  #if MAG
		for (axis = 0; axis < 3; axis++)
		  EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
	  #endif

  // Attitude of the estimated vector
  angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
  angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;

  #if MAG
    // Attitude of the cross product vector GxM
    heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );
    heading += MAG_DECLINIATION * 10; //add declination
    heading = heading /10;
    if ( heading > 180)      heading = heading - 360;
    else if (heading < -180) heading = heading + 360;
  #endif
#endif//multiwii fusion
}


/*-----------------------------------------------------------------------------------
 * Quaternion representation of Mahony's DCM complimentary filter written by Sebastian Madgwick
 * 		- modified to include complimentary filter between mag and acc to reduce cross coupling
 * 		- gains are scheduled to reduce errors associated with external/centripetal accelerations
 * 		- uses fast inverse square root algorithm for normalisation
 * 		- quaternions are converted to ypr body centred tait bryan angles for attitude control
 * 		- note all quaternions represent rotation of earth relative to sensor
 *
 * 		http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

#if defined(MAYHONY)
#if defined(MAG)
void MayhonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	uint32_t currentT;

	//save time at current iteration
	currentT = millis();

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		//compute inverse squareroot of accelerometer readings
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);

		//gain scheduling for feedback based on external accelerations
		if((recipNorm>0.0017755f)&&(recipNorm<0.00217f))	//high gain if normal is between 1.1 and 0.9G
		{

			if(currentT < 5000) //give AHRS time to converge to initial conditions
			{
				f.OK_TO_ARM = 0;
				f.ARMED = 0;
				if(currentT > convergedT)
				{
					LEDPIN_TOGGLE;
					convergedT = currentT +50;
				}
				twoKp = 40;//accelerate AHRS towards initial conditions while multicopter is held steady
			}
			else if(currentT > accelT)//normal operation
			{
				twoKp = twoKpDef;
				twoKi = twoKiDef;
			}
		}
		else
		{
			accelT = currentT + 500; //reduce gains for 200ms after acceleration detected
			twoKp = twoKpDefMin; //reduce proportional gain to reduce effects of erroneous corrections in pitch and roll
			twoKi = 0; //reset integral to prevent windup and remove unwanted effects of large accel errors
		}

		// Normalise accelerometer measurement
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field, finding how much inclination should exist by translating to estimated earth frame
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2); //translate the reading back into sensor frame
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is weighted sum of cross product between estimated direction and measured direction of field vectors
		halfex = 2*(AccFactor*(ay * halfvz - az * halfvy) + MagFactor*(my * halfwz - mz * halfwy))/(AccFactor+MagFactor);//reduce cross coupling in pitch and roll
		halfey = 2*(AccFactor*(az * halfvx - ax * halfvz) + MagFactor*(mz * halfwx - mx * halfwz))/(AccFactor+MagFactor);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);//more emphasis in compass yaw rotation

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * samplePeriod;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * samplePeriod;
			integralFBz += twoKi * halfez * samplePeriod;

			constrain(integralFBx,-0.008f,0.008f); //constrain integral error to 0.5 degs/sample to prevent integral windup
			constrain(integralFBy,-0.008f,0.008f); //(roughly error of 1 rad for 8/gain milliseconds to saturate = 800milliseconds at 0.01 ki)
			constrain(integralFBz,-0.008f,0.008f);

			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += halfex*twoKp;
		gy += halfey*twoKp;
		gz += halfez*twoKp;
	}



	// Integrate rate of change of quaternion
	gx *= (0.5f * samplePeriod);		// pre-multiply common factors
	gy *= (0.5f * samplePeriod);
	gz *= (0.5f * samplePeriod);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	//convert to tait bryan angles and store
	getAttitude();
}
#endif//9dof fusion

//---------------------------------------------------------------------------------------------------
// IMU algorithm update
#if !defined(MAG)
void MayhonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// find inverse squareroot of accelerometer data
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);

		//gain scheduling for measured acceleration
		if((recipNorm>0.0017755f)&&(recipNorm<0.00217f))	//only use accelerometer if normal is between 1.1 and 0.9G
		{
			currentT = millis();
			if(currentT < 5000) //give AHRS time to converge to initial conditions
			{
				f.OK_TO_ARM = 0;
				f.ARMED = 0;
				if(currentT > convergedT)
				{
					LEDPIN_TOGGLE;
					convergedT = currentT +50;
				}
				twoKp = 40;//accelerate AHRS towards initial conditions while multicopter is held steady
			}
			else//normal operation
			{
				twoKp = twoKpDef;
			}
		}
		else
		{
			twoKp = twoKpDefMin; //reduce proportional gain to reduce effects of erroneous corrections in pitch and roll
		}

		//Normalise accelerometer reading
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * samplePeriod;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * samplePeriod;
			integralFBz += twoKi * halfez * samplePeriod;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * samplePeriod);		// pre-multiply common factors
	gy *= (0.5f * samplePeriod);
	gz *= (0.5f * samplePeriod);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	//convert to tait bryan angles and store
	getAttitude();
}
#endif//6dof fusion


//modified fast inverse square root http://www.diydrones.com/forum/topics/madgwick-imu-ahrs-and-fast-inverse-square-root
float invSqrt(float x) {
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float tmp = *(float*)&i;
   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}


//convert quaternion -> rotation matrix -> tait bryan angles (see madgwick paper and project documentation)
static void getAttitude(void)
{
	float test =  q1*q3 - q0*q2; //check for singularity


	/*NOTE! MultiWii pitch is defined nose down positive (against convention)
	 * converts to intrinsic tait bryan angles for pitch & roll
	 * from output using quaternion conjugate(board is referenece for rotation axis not earth)
	 * roll is positive as sensor roll axis aligns with aeronautical sequence
	 * pitch positive as multiwii is backwards for this and sensor pitch axis is negative of aero sequence (double negative)
	 * yaw is negative as sensor yaw axis is negative of aeronautical sequence
	 */
	if(test>0.497) //pitch down 90deg case
	{
		angle[ROLL] = 0;
		angle[PITCH] = asin(2*(q0*q2 - q1*q3))*(180.0f / PI*10 );

		#if MAG
			heading = -atan2((q0*q3-q1*q2),-(q1*q3+q0*q2))* (180.0f / PI);
		#endif
	}
	else if(test<-0.497) //pitch up 90deg case
	{
		angle[ROLL] = 0;
		angle[PITCH] = asin(2*(q0*q2 - q1*q3))*(180.0f / PI*10 );

		#if MAG
			heading = atan2((q1*q2-q0*q3),(q1*q3+q0*q2))* (180.0f / PI);
		#endif
	}
	else //no singularity case
	{
	// Attitude of the estimated vector
	angle[ROLL] =  atan2(2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2))*(180.0f / PI *10 );
	angle[PITCH] =  asin(2*(q0*q2 - q1*q3))*(180.0f / PI *10 );
	#if MAG
		heading =  -atan2(2*(q1*q2 + q0*q3), 1 - 2*(q2*q2 + q3*q3))* (180.0f / PI);
	#endif
	}

	#if MAG
		//fix heading
		heading += MAG_DECLINIATION;

		if ( heading > 180)      heading = heading - 360;
		else if (heading < -180) heading = heading + 360;
	#endif

#ifdef AHRSDEBUG

	uint32_t newTime = micros();
	if(newTime - updateTime > 20000)
	{
		//for use with freeimu processing program as visualisation
		float quat[4];
		quat[0] = q0;
		quat[1] = q1;
		quat[2] = q2;
		quat[3] = q3;

		serialPrintFloatArr(&quat, 4);
		char c1 = '\n';
		UARTSend(&c1, 1);
		updateTime = newTime;
	}
#endif
}

#endif

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define BARO_TAB_SIZE   21
#define ACC_Z_DEADBAND 5 // was 40 instead of 32 now


#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }

#ifdef BARO
uint8_t getEstimatedAltitude(){
  static int32_t baroGroundPressure;
  static uint32_t previousT;
  uint32_t currentT = micros();
  uint32_t dTime;

  if(currentT<1000000) return 0;

  dTime = currentT - previousT;
  if (dTime < UPDATE_INTERVAL) return 0;
  previousT = currentT;

  if(calibratingB > 0) {
    baroGroundPressure = baroPressureSum/(BARO_TAB_SIZE - 1);
    calibratingB--;
  }

  // pressure relative to ground pressure with temperature compensation (fast!)
  // baroGroundPressure is not supposed to be 0 here
  // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
  BaroAlt = log( baroGroundPressure * (BARO_TAB_SIZE - 1)/ (float)baroPressureSum ) * (baroTemperature+27315) * 29.271267f; // in cemtimeter



#ifdef MEDFILTER
#ifdef SONAR
  //filter sonar output and compensate for tilt
  debug[3] = applyMedFilter(&SonarFilter,sonarAlt) * halfvz * 2;
	static int16_t sonarOffset = 0; // = acc_1G*6; //58 bytes saved and convergence is fast enough to omit init
	if (!f.ARMED) {
	  sonarOffset -= sonarOffset>>3;
	  sonarOffset += debug[3];
	}
	debug[3] -= sonarOffset>>3;
	if(EstAlt < 400)
		EstAlt = (EstAlt * 2 + debug[3] * 6) >> 3;// complimentary filter to include sonar data
	else
		EstAlt = (EstAlt * 6 + BaroAlt * 2) >> 3;
#endif
#else
  EstAlt = (EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)
#endif

  #if (defined(VARIOMETER) && (VARIOMETER != 2)) || !defined(SUPPRESS_BARO_ALTHOLD)
	//P
	int16_t error16 = constrain(AltHold - EstAlt, -300, 300);
	applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
	BaroPID = constrain((conf.P8[PIDALT] * error16 >>7), -150, +150);

	//I
	errorAltitudeI += conf.I8[PIDALT] * error16 >>6;
	errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
	BaroPID += errorAltitudeI>>9; //I in range +/-60

	// projection of ACC vector to global Z, with 1G subtructed
	// Math: accZ = A * G  - 1G
	int16_t accZ = 2*(accSmooth[Xaxis] * halfvx + accSmooth[Yaxis] * halfvy + accSmooth[Zaxis] * halfvz);// scale of 2 as vx,vy and vz are a unit vector

	static int16_t accZoffset = 0; // = acc_1G*6; //58 bytes saved and convergence is fast enough to omit init
	if (!f.ARMED) {
	  accZoffset -= accZoffset>>3;
	  accZoffset += accZ;
	}
	accZ -= accZoffset>>3;
	//applyDeadband(accZ, ACC_Z_DEADBAND);

	//debug[1] = BaroAlt;


	if((calibratingA == 0) && (calibratingB == 0))
	  {
		static float vel = 0.0f;
		static float accVelScale = 9.80665f / 10000.0f / 512 ;

		/*test Complimentary filter for altitude using dead reckoning*/
		static float acc_alt = 0.0f;

		acc_alt += vel * dTime / 1000000.0f;

		acc_alt = acc_alt * 0.85f + BaroAlt * 0.15f;

		//debug[2] = acc_alt;//cm
		/*----------------------------------------------------------*/

		// Integrator - velocity, cm/sec
		vel += accZ * accVelScale * dTime;

		static int32_t lastBaroAlt = 0;
		int16_t baroVel = (EstAlt - lastBaroAlt) * 1000000.0f / dTime;
		lastBaroAlt = EstAlt;

		baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
		applyDeadband(baroVel, 10); // to reduce noise near zero

		// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
		// By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
		vel = vel * 0.985f + baroVel * 0.015f;

		//debug[3] = vel;// mm/s



		//D
		int16_t vel_tmp = vel;
		applyDeadband(vel_tmp, 5);
		vario = vel_tmp;
		BaroPID -= constrain(conf.D8[PIDALT] * vel_tmp >>4, -150, 150);
	  #endif
  }
  return 1;
}
#endif //BARO
