/****************************************************************************
 *   $Id:: eeprom.c 7171 2011-04-26 20:23:50Z nxp28548                      $
 *   Project: NXP LPC11Axx EEPROM example
 *
 *   Description:
 *     This file contains EEPROM code example which include EEPROM
 *     initialization, EEPROM interrupt handler, and APIs for EEPROM access.
 *
 ****************************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.

* Permission to use, copy, modify, and distribute this software and its
* documentation is hereby granted, under NXP Semiconductors’
* relevant copyright in the software, without fee, provided that it
* is used in conjunction with NXP Semiconductors microcontrollers.  This
* copyright, permission, and disclaimer notice must appear in all copies of
* this code.

****************************************************************************/

#include "LPC13Uxx.h"
#include "EEPROM.h"
#include "config.h"
#include "def.h"
#include "common.h"
#ifdef GPS
	#include "GPS.h"
#endif

#define IAP_LOCATION 0x1FFF1FF1
typedef void (*IAP) (	unsigned int command[],
						unsigned int result[] );
static const IAP iap_entry = (IAP) IAP_LOCATION;

extern struct flags_struct f;
extern struct conf_def conf;
extern int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
extern int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

//1) EEprom Write
//
//Command code: 61
//Param0: eeprom address (byte, half-word or word aligned)
//Param1: RAM address (byte, half-word or word aligned)
//Param2: Number of bytes to be written ( Byte, Half-words write are ok)
//Param3: System Clock Frequency (CCLK) in kHz
//
//Return Code CMD_SUCCESS | SRC_ADDR_NOT_MAPPED | DST_ADDR_NOT_MAPPED
void eeprom_write_block( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount )
{
	unsigned int command[5], result[4];

	command[0] = 61;
	command[1] = (uint32_t) eeAddress;
	command[2] = (uint32_t) buffAddress;
	command[3] = byteCount;
	command[4] = SystemCoreClock/1000;

	/* Invoke IAP call...*/
	iap_entry(command, result);
	if (0 != result[0])
	{
		//Trap error
		while(1);
	}
	return;
}

//2) EEprom Read
//Command code: 62
//Param0: eeprom address (byte, half-word or word aligned)
//Param1: RAM address (byte, half-word or word aligned)
//Param2: Number of bytes to be read ( Byte, Half-words read are ok)
//Param3: System Clock Frequency (CCLK) in kHz
//
//Return Code CMD_SUCCESS | SRC_ADDR_NOT_MAPPED | DST_ADDR_NOT_MAPPED
void eeprom_read_block( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount )
{
	unsigned int command[5], result[4];

	command[0] = 62;
	command[1] = (uint32_t) eeAddress;
	command[2] = (uint32_t) buffAddress;
	command[3] = byteCount;
	command[4] = SystemCoreClock/1000;

	/* Invoke IAP call...*/
  	iap_entry( command, result);
	if (0 != result[0])
	{
		//Trap error
		while(1);
	}
	return;
}

void readEEPROM(void) {
  uint8_t i;

  eeprom_read_block( (uint8_t*)0x00,  (uint8_t*)&conf,  sizeof(conf));
  for(i=0;i<6;i++) {
    lookupPitchRollRC[i] = (2500+conf.rcExpo8*(i*i-25))*i*(int32_t)conf.rcRate8/2500;
  }
  for(i=0;i<11;i++) {
    int16_t tmp = 10*i-conf.thrMid8;
    uint8_t y = 1;
    if (tmp>0) y = 100-conf.thrMid8;
    if (tmp<0) y = conf.thrMid8;
    lookupThrottleRC[i] = 10*conf.thrMid8 + tmp*( 100-conf.thrExpo8+(int32_t)conf.thrExpo8*(tmp*tmp)/(y*y) )/10; // [0;1000]
    lookupThrottleRC[i] = MINTHROTTLE + (int32_t)(MAXTHROTTLE-MINTHROTTLE)* lookupThrottleRC[i]/1000;            // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
  }

  #if defined(POWERMETER)
    pAlarm = (uint32_t) conf.powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
  #endif
  #ifdef FLYING_WING
    #ifdef LCD_CONF
      conf.wing_left_mid  = constrain(conf.wing_left_mid, WING_LEFT_MIN,  WING_LEFT_MAX); //LEFT
      conf.wing_right_mid = constrain(conf.wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
    #else // w.o LCD support user may not find this value stored in eeprom, so always use the define value
      conf.wing_left_mid  = WING_LEFT_MID;
      conf.wing_right_mid = WING_RIGHT_MID;
    #endif
  #endif
  #ifdef TRI
    #ifdef LCD_CONF
      conf.tri_yaw_middle = constrain(conf.tri_yaw_middle, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
    #else // w.o LCD support user may not find this value stored in eeprom, so always use the define value
      conf.tri_yaw_middle = TRI_YAW_MIDDLE;
    #endif
  #endif
  #if GPS
    if (f.I2C_INIT_DONE) GPS_set_pids();
  #endif
}


void writeParams(uint8_t b) {
  feedWatchDog();
  conf.checkNewConf = EEPROM_CONF_VERSION; // make sure we write the current version into eeprom
  eeprom_write_block( (uint8_t*)0x00,  (uint8_t*)&conf, sizeof(conf));
  readEEPROM();
  if (b == 1) blinkLED(15,20,1);
}

void checkFirstTime(void) {
  if (EEPROM_CONF_VERSION == conf.checkNewConf) return;
  uint8_t i = 0;

#if PIDcontroller == 1
	conf.P8[ROLL]     = 33; conf.I8[ROLL]     = 30; conf.D8[ROLL]     = 23;
	conf.P8[PITCH]    = 33; conf.I8[PITCH]    = 30; conf.D8[PITCH]    = 23;
	conf.P8[PIDLEVEL] = 90; conf.I8[PIDLEVEL] = 10; conf.D8[PIDLEVEL] = 100;
#elif PIDcontroller == 2
	conf.P8[ROLL]     = 28;  conf.I8[ROLL]    = 10; conf.D8[ROLL]     = 7;
	conf.P8[PITCH]    = 28; conf.I8[PITCH]    = 10; conf.D8[PITCH]    = 7;
	conf.P8[PIDLEVEL] = 30; conf.I8[PIDLEVEL] = 0;  conf.D8[PIDLEVEL] = 100;
#endif

  conf.P8[YAW]      = 68;  conf.I8[YAW]     = 45;  conf.D8[YAW]     = 0;

  conf.P8[PIDALT]   = 16; conf.I8[PIDALT]   = 15; conf.D8[PIDALT]   = 7;

  conf.P8[PIDPOS]  = POSHOLD_P * 100;     conf.I8[PIDPOS]    = POSHOLD_I * 100;       conf.D8[PIDPOS]    = 0;
  conf.P8[PIDPOSR] = POSHOLD_RATE_P * 10; conf.I8[PIDPOSR]   = POSHOLD_RATE_I * 100;  conf.D8[PIDPOSR]   = POSHOLD_RATE_D * 1000;
  conf.P8[PIDNAVR] = NAV_P * 10;          conf.I8[PIDNAVR]   = NAV_I * 100;           conf.D8[PIDNAVR]   = NAV_D * 1000;

  conf.P8[PIDMAG] = 40;

  conf.P8[PIDVEL] = 0;  conf.I8[PIDVEL] = 0;  conf.D8[PIDVEL] = 0;

  conf.rcRate8 = 90; conf.rcExpo8 = 65;
  conf.rollPitchRate = 0;
  conf.yawRate = 0;
  conf.dynThrPID = 0;
  conf.thrMid8 = 50; conf.thrExpo8 = 0;
  for(i=0;i<CHECKBOXITEMS;i++) {conf.activate[i] = 0;}
  conf.angleTrim[0] = 0; conf.angleTrim[1] = 0;
  conf.powerTrigger1 = 0;
  #ifdef FLYING_WING
    conf.wing_left_mid  = WING_LEFT_MID;
    conf.wing_right_mid = WING_RIGHT_MID;
  #endif
  #ifdef FIXEDWING
    conf.dynThrPID = 50;
    conf.rcExpo8   =  0;
  #endif
  #ifdef TRI
    conf.tri_yaw_middle = TRI_YAW_MIDDLE;
  #endif
  #if defined HELICOPTER || defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
    {
      int16_t s[8] = SERVO_OFFSET;
      for(uint8_t i=0;i<8;i++) conf.servoTrim[i] = s[i];
    }
  #endif
  #if defined(GYRO_SMOOTHING)
    {
      uint8_t s[3] = GYRO_SMOOTHING;
      for(uint8_t i=0;i<3;i++) conf.Smoothing[i] = s[i];
    }
  #endif
  writeParams(0); // this will also (p)reset checkNewConf with the current version number again.
}
/******************************************************************************
**                            End Of File
******************************************************************************/
