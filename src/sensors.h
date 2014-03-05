/*
 * sensors.h
 *
 *  Created on: 2/02/2013
 *      Author: Jono
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "config.h"
#include "def.h"
#include "type.h"
// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION)
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION)
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION)
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif


#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
  //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

#if defined(BMP085)
#define BMP085_ADDRESS 0x77
#endif

#if !defined(MS561101BA_ADDRESS)
  #define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
  //#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)
#endif

#if defined(HMC5883)
	#define MAG_ADDRESS 0x1E
	#define MAG_DATA_REGISTER 0x03
#endif



//MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_DLPF_CFG   5
  #endif
  #if defined(MPU6050_LPF_5HZ)
    #define MPU6050_DLPF_CFG   6
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define MPU6050_DLPF_CFG   0
#endif


#define SONARCHANNEL 6
#define SONARSCALE 0.3175f



//function prototype definitions
uint32_t i2c_writeReg(uint8_t add,uint8_t reg, uint8_t val);
uint32_t i2c_burstWrite(uint8_t add,uint8_t reg, uint8_t* val, uint8_t bytes);
uint32_t i2c_writeCommand(uint8_t add, uint8_t val);
uint32_t i2c_readReg( uint8_t add, uint8_t reg);
uint32_t i2c_getSixRawADC(uint8_t add, uint8_t reg);
uint32_t i2c_read(uint8_t add, uint8_t reg, uint8_t bytes);
void Gyro_init();
void Gyro_getADC ();
void ACC_init ();
void ACC_getADC();
void Device_Mag_getADC();
void Mag_init();
uint32_t getCompADC();
void i2c_BMP085_readCalibration();
void Baro_init();
uint8_t Baro_update();

#ifdef BMP085
	void i2c_BMP085_UT_Start();
	void i2c_BMP085_UP_Start ();
	void i2c_BMP085_UP_Read ();
	void i2c_BMP085_UT_Read() ;
	void i2c_BMP085_Calculate();
#endif

#ifdef MS561101BA
	void i2c_MS561101BA_reset();
	void i2c_MS561101BA_readCalibration();
	void i2c_MS561101BA_UT_Start();
	void i2c_MS561101BA_UP_Start ();
	void i2c_MS561101BA_UP_Read ();
	void i2c_MS561101BA_UT_Read();
	void i2c_MS561101BA_Calculate();
#endif
void Sonar_update(void);
void GYRO_Common(void);
void ACC_Common(void);
uint8_t Mag_getADC(void);
void Baro_Common();
void initSensors();
void initError(void);

#endif /* SENSORS_H_ */
