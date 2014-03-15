/*
 * def.h
 *
 *  Created on: 4/02/2013
 *      Author: Jono
 */

#ifndef DEF_H_
#define DEF_H_
#include "type.h"

#define  VERSION  210

//global defines
/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDPOS     4
#define PIDPOSR    5
#define PIDNAVR    6
#define PIDLEVEL   7
#define PIDMAG     8
#define PIDVEL     9 // not used currently

#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10
#define BOXLEDMAX    11 // we want maximum illumination
#define BOXLLIGHTS   12 // enable landing lights at any altitude
#define BOXHEADADJ   13 // acquire heading for HEADFREE mode
#define BOXFLOWHOLD  14

#define Yaxis 0
#define Xaxis 1
#define Zaxis 2


#define PIDITEMS 10
#define CHECKBOXITEMS 15

// LPCXpresso eval board LED
#define LED_PORT 0		// Port for led
#define LED_BIT 7		// Bit on port for led
#define LED_ON 1		// Level to set port to turn on led
#define LED_OFF 0		// Level to set port to turn off led

#define SHIELD_PORT 0
#define SHIELD_BIT 8


struct conf_def{
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t accZero[3];
  int16_t magZero[3];
  int16_t angleTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
  #ifdef FLYING_WING
    uint16_t wing_left_mid;
    uint16_t wing_right_mid;
  #endif
  #if defined(TRI) || defined(TRI_DIGITAL)
    uint16_t tri_yaw_middle;
  #endif
  #if defined HELICOPTER || defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
    int16_t servoTrim[8];
  #endif
  #if defined(GYRO_SMOOTHING)
    uint8_t Smoothing[3];
  #endif
};

struct flags_struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ACC_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
  uint8_t FLOW_HOLD_MODE :1;
};

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(x) ((x)>0?(x):-(x))
#define true 1
#define false 0

#define MINCHECK 1100
#define MAXCHECK 1900
#define FAILSAFE_DETECT_TRESHOLD  950

#define LAT  0
#define LON  1
// default POSHOLD control gains
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           0.2f        // m/s

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //
#define NAV_IMAX               20        // degrees


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial GPS only variables
//navigation mode
#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2

//definitions for IMU boards
#if defined(GY_86)
  #define MPU6050
  #define HMC5883
  #define MS561101BA
	#if defined(MAYHONY)
		#define ACC_ORIENTATION(X, Y, Z)  {accADC[Xaxis]  = Y; accADC[Yaxis]  = -X; accADC[Zaxis]  =  Z;}
		#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z; }
		#define MAG_ORIENTATION(X, Y, Z)  {magADC[Xaxis]  =  Y; magADC[Yaxis]  =  -X; magADC[Zaxis]  = Z;}
		#define GYRO_AHRS_ORIENTATION(R, P, Y) {gyroAHRS[Xaxis] = R; gyroAHRS[Yaxis] = P; gyroAHRS[Zaxis] = -Y;}
	#else
	  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
	  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
	  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
	#endif
  #define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050

#undef INTERNAL_I2C_PULLUPS
#endif

#if defined(GY_87)
  #define MPU6050
  #define HMC5883
  #define BMP085
	#if defined(MAYHONY)
#define ACC_ORIENTATION(X, Y, Z)  {accADC[Xaxis]  = Y; accADC[Yaxis]  = -X; accADC[Zaxis]  =  Z;}
		#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z; }
		#define MAG_ORIENTATION(X, Y, Z)  {magADC[Xaxis]  =  Y; magADC[Yaxis]  =  -X; magADC[Zaxis]  = Z;}
		#define GYRO_AHRS_ORIENTATION(R, P, Y) {gyroAHRS[Xaxis] = R; gyroAHRS[Yaxis] = P; gyroAHRS[Zaxis] = -Y;}
	#else
	  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
	  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
	  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
	#endif
	#define MPU6050_I2C_AUX_MASTER // MAG connected to the AUX I2C bus of MPU6050
  #undef INTERNAL_I2C_PULLUPS
#endif

/**************************************************************************************/
/***************              Sensor Type definitions              ********************/
/**************************************************************************************/

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(MMA7455) || defined(ADCACC) || defined(LIS3LV02) || defined(LSM303DLx_ACC) || defined(MPU6050) || defined(NUNCHUCK)
  #define ACC 1
#else
  #define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) || defined(MAG3110)
  #define MAG 1
#else
  #define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(MPU6050) || defined(WMP)
  #define GYRO 1
#else
  #define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA)
  #define BARO 1
#else
  #define BARO 0
#endif

#if defined(GPS_PROMINI_SERIAL)
  #define GPS_SERIAL 0
  #define GPS_PROMINI
  #define GPS_BAUD   GPS_PROMINI_SERIAL
#endif

#if defined(GPS_SERIAL)  || defined(I2C_GPS) || defined(GPS_FROM_OSD) || defined(TINY_GPS) || defined(I2C_NAV)
  #define GPS 1
#else
  #define GPS 0
#endif

#if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235) || defined(TINY_GPS_SONAR) || defined(MAXSONAR) || defined(MAXSONAR_PWM)|| defined(FLOW)
  #define SONAR 1
#else
  #define SONAR 0
#endif


/**************************************************************************************/
/***************      Multitype decleration for the GUI's          ********************/
/**************************************************************************************/
#if defined(TRI)
  #define MULTITYPE 1
#elif defined(QUADP)
  #define MULTITYPE 2
#elif defined(QUADX)
  #define MULTITYPE 3
#elif defined(BI)
  #define MULTITYPE 4
#elif defined(GIMBAL)
  #define MULTITYPE 5
#elif defined(Y6)
  #define MULTITYPE 6
#elif defined(HEX6)
  #define MULTITYPE 7
#elif defined(FLYING_WING)
  #define MULTITYPE 8
#elif defined(Y4)
  #define MULTITYPE 9
#elif defined(HEX6X)
  #define MULTITYPE 10
#elif defined(OCTOX8)
  #define MULTITYPE 11   //the JAVA GUI is the same for all 8 motor configs
#elif defined(OCTOFLATP)
  #define MULTITYPE 12   //12  for MultiWinGui
#elif defined(OCTOFLATX)
  #define MULTITYPE 13   //13  for MultiWinGui
#elif defined(AIRPLANE)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
  #define MULTITYPE 14
#elif defined (HELI_120_CCPM)
  #define MULTITYPE 15
#elif defined (HELI_90_DEG)
  #define MULTITYPE 16
#elif defined(VTAIL4)
 #define MULTITYPE 17
#endif

/**************************************************************************************/
/***************          Some unsorted "chain" defines            ********************/
/**************************************************************************************/
#if defined (AIRPLANE) || defined(FLYING_WING)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
  #define FIXEDWING
#endif

#if defined (AIRPLANE) || defined(HELICOPTER)|| defined(SINGLECOPTER)|| defined(DUALCOPTER) && defined(PROMINI)
  #if defined(D12_POWER)
    #define SERVO_4_PINMODE            ;  // D12
    #define SERVO_4_PIN_HIGH           ;
    #define SERVO_4_PIN_LOW            ;
  #else
    #define POWERPIN_PINMODE           ;
    #define POWERPIN_ON                ;
    #define POWERPIN_OFF               ;
  #endif
#endif

#if defined(HELI_120_CCPM) || defined(HELI_90_DEG)
  #define HELICOPTER
#endif

#if defined(POWERMETER_HARD) || defined(POWERMETER_SOFT)
  #define POWERMETER
#endif

//all new Special RX's must be added here
//this is to avoid confusion :)
#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(SBUS) && !defined(RCSERIALERIAL)
  #define STANDARD_RX
#endif


// Spektrum Satellite
#if defined(SPEKTRUM)
  #define SPEK_MAX_CHANNEL 7
  #define SPEK_FRAME_SIZE 16
  #if (SPEKTRUM == 1024)
    #define SPEK_CHAN_SHIFT  2       // Assumes 10 bit frames, that is 1024 mode.
    #define SPEK_CHAN_MASK   0x03    // Assumes 10 bit frames, that is 1024 mode.
  #endif
  #if (SPEKTRUM == 2048)
    #define SPEK_CHAN_SHIFT  3       // Assumes 11 bit frames, that is 2048 mode.
    #define SPEK_CHAN_MASK   0x07    // Assumes 11 bit frames, that is 2048 mode.
  #endif
#endif


/**************************************************************************************/
/***************             motor and servo numbers               ********************/
/**************************************************************************************/
#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(AIRPLANE) || defined(CAMTRIG) || defined(HELICOPTER) || defined(SERVO_MIX_TILT)|| defined(SINGLECOPTER)|| defined(DUALCOPTER)
  #define SERVO
#else
	#undef SERVO
#endif

#if defined(GIMBAL)
  #define NUMBER_MOTOR     0
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2
#elif defined(FLYING_WING)
  #define NUMBER_MOTOR     1
  #define PRI_SERVO_FROM   1 // use servo from 1 to 2
  #define PRI_SERVO_TO     2

#elif defined(SINGLECOPTER)
  #define NUMBER_MOTOR     1
  #define PRI_SERVO_FROM   4 // use servo from 4 to 7
  #define PRI_SERVO_TO     7
#elif defined(DUALCOPTER)
  #define NUMBER_MOTOR     2
  #define PRI_SERVO_FROM   4 // use servo from 5 to 6
  #define PRI_SERVO_TO     6

#elif defined(AIRPLANE)
  #define NUMBER_MOTOR     0
    #if defined(FLAPS)
      #define PRI_SERVO_FROM   3 // use servo from 3 to 8
      #undef CAMTRIG             // Disable Camtrig on A2
    #else
      #define PRI_SERVO_FROM   4 // use servo from 4 to 8
    #endif
  #define PRI_SERVO_TO     8
#elif defined(BI)
  #define NUMBER_MOTOR     2
  #define PRI_SERVO_FROM   5 // use servo from 5 to 6
  #define PRI_SERVO_TO     6

#elif defined(TRI)
  #define NUMBER_MOTOR     3
  #define PRI_SERVO_FROM   6 // use only servo 6
  #define PRI_SERVO_TO     6
#elif defined(QUADP) || defined(QUADX) || defined(Y4)|| defined(VTAIL4)
  #define NUMBER_MOTOR     4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
  #define NUMBER_MOTOR     6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
  #define NUMBER_MOTOR     8
#elif defined(HELICOPTER)
  #ifdef HELI_USE_SERVO_FOR_THROTTLE
    #define NUMBER_MOTOR     0 // use servo to drive throttle output
    #define PRI_SERVO_FROM   4 // use servo from 4 to 8
    #define PRI_SERVO_TO     8
  #else
    #define NUMBER_MOTOR     1 // use 1 motor for throttle
    #define PRI_SERVO_FROM   4 // use servo from 4 to 7
    #define PRI_SERVO_TO     7
  #endif
#endif


#if (defined(SERVO_TILT)|| defined(SERVO_MIX_TILT))&& defined(CAMTRIG)
  #define SEC_SERVO_FROM   1 // use servo from 1 to 3
  #define SEC_SERVO_TO     3
#else
  #if defined(SERVO_TILT)|| defined(SERVO_MIX_TILT)
    // if A0 and A1 is taken by motors, we can use A2 and 12 for Servo tilt
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      #define SEC_SERVO_FROM   3 // use servo from 3 to 4
      #define SEC_SERVO_TO     4
    #else
      #if !defined(MEGA_HW_GIMBAL) // if HW Gimbal is active we dont need the SW PWM defines
        #define SEC_SERVO_FROM   1 // use servo from 1 to 2
        #define SEC_SERVO_TO     2
      #endif
    #endif
  #endif
  #if defined(CAMTRIG)
    #define SEC_SERVO_FROM   3 // use servo 3
    #define SEC_SERVO_TO     3
  #endif
#endif

#if !(defined(DISPLAY_2LINES)) && !(defined(DISPLAY_MULTILINE))
  #if (defined(LCD_VT100)) || (defined(OLED_I2C_128x64))
    #define DISPLAY_MULTILINE
  #else
    #define DISPLAY_2LINES
  #endif
#endif

#if (defined(LCD_VT100))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 6
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 9
  #endif
#elif (defined(OLED_I2C_128x64))
  #if !(defined(MULTILINE_PRE))
    #define MULTILINE_PRE 3
  #endif
  #if !(defined(MULTILINE_POST))
    #define MULTILINE_POST 5
  #endif
#endif

#if !defined(ALT_HOLD_THROTTLE_NEUTRAL_ZONE)
  #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 20
#endif

/**************************************************************************************/
/***************                       I2C GPS                     ********************/
/**************************************************************************************/
#if defined(I2C_GPS)||defined(I2C_NAV)
  #define I2C_GPS_ADDRESS                         0x20 //7 bits
///////////////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS NAV registers
///////////////////////////////////////////////////////////////////////////////////////////////////
//
#define I2C_GPS_STATUS_00                            00 //(Read only)
        #define I2C_GPS_STATUS_NEW_DATA       0x01      // New data is available (after every GGA frame)
        #define I2C_GPS_STATUS_2DFIX          0x02      // 2dfix achieved
        #define I2C_GPS_STATUS_3DFIX          0x04      // 3dfix achieved
        #define I2C_GPS_STATUS_WP_REACHED     0x08      // Active waypoint has been reached (not cleared until new waypoint is set)
        #define I2C_GPS_STATUS_NUMSATS        0xF0      // Number of sats in view

#define I2C_GPS_COMMAND                              01 // (write only)
        #define I2C_GPS_COMMAND_POSHOLD       0x01      // Start position hold at the current gps positon
        #define I2C_GPS_COMMAND_START_NAV     0x02      // get the WP from the command and start navigating toward it
        #define I2C_GPS_COMMAND_SET_WP        0x03      // copy current position to given WP
        #define I2C_GPS_COMMAND_UPDATE_PIDS   0x04      // update PI and PID controllers from the PID registers, this must be called after a pid register is changed
        #define I2C_GPS_COMMAND_NAV_OVERRIDE  0x05      // do not nav since we tring to controll the copter manually (not implemented yet)
        #define I2C_GPS_COMMAND_STOP_NAV      0x06      // Stop navigation (zeroes out nav_lat and nav_lon
        #define I2C_GPS_COMMAND__7            0x07
        #define I2C_GPS_COMMAND__8            0x08
        #define I2C_GPS_COMMAND__9            0x09
        #define I2C_GPS_COMMAND__a            0x0a
        #define I2C_GPS_COMMAND__b            0x0b
        #define I2C_GPS_COMMAND__c            0x0c
        #define I2C_GPS_COMMAND__d            0x0d
        #define I2C_GPS_COMMAND__e            0x0e
        #define I2C_GPS_COMMAND__f            0x0f

        #define I2C_GPS_COMMAND_WP_MASK       0xF0       // Waypoint number

#define I2C_GPS_WP_REG                              02   // Waypoint register (Read only)
        #define I2C_GPS_WP_REG_ACTIVE_MASK    0x0F       // Active Waypoint lower 4 bits
        #define I2C_GPS_WP_REG_PERVIOUS_MASK  0xF0       // pervious Waypoint upper 4 bits

#define I2C_GPS_REG_VERSION                         03   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_REG_RES2                            04   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES3                            05   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES4                            06   // reserved for future use (uint8_t)


#define I2C_GPS_LOCATION                            07   // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_NAV_LAT                             15   // Desired banking towards north/south int16_t
#define I2C_GPS_NAV_LON                             17   // Desired banking toward east/west    int16_t
#define I2C_GPS_WP_DISTANCE                         19   // Distance to current WP in cm uint32
#define I2C_GPS_WP_TARGET_BEARING                   23   // bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_NAV_BEARING                         25   // crosstrack corrected bearing towards current wp 1deg = 1000 int16_t
#define I2C_GPS_HOME_TO_COPTER_BEARING              27   // bearing from home to copter 1deg = 1000 int16_t
#define I2C_GPS_DISTANCE_TO_HOME                    29   // distance to home in m int16_t

#define I2C_GPS_GROUND_SPEED                        31   // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE                            33   // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE                       35   // GPS ground course (uint16_t)
#define I2C_GPS_RES1                                37   // reserved for future use (uint16_t)
#define I2C_GPS_TIME                                39   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)

//Writeable registers from here

#define I2C_GPS_CROSSTRACK_GAIN                     43    // Crosstrack gain *100 (1 - 0.01 100 - 1) uint8_t
#define I2C_GPS_SPEED_MIN                           44    // Minimum navigation speed cm/s uint8_t
#define I2C_GPS_SPEED_MAX                           45    // Maximum navigation speed cm/s uint16_t
#define I2C_GPS_RESERVED                            47    // Reserved for future use
#define I2C_GPS_WP_RADIUS                           49    // Radius of the wp in cm, within this radius we consider the wp reached (uint16_t)

#define I2C_GPS_NAV_FLAGS                           51    // Controls various functions of the I2C-GPS-NAV module
        #define I2C_NAV_FLAG_GPS_FILTER          0x80     // If this bit set GPS coordinates are filtered via a 5 element moving average filter
        #define I2C_NAV_FLAG_LOW_SPEED_D_FILTER  0x40     // If speed below .5m/s ignore D term in POSHOLD_RATE, this supposed to filter out noise

#define I2C_GPS_HOLD_P                              52    // poshold_P  *100 uint16_t
#define I2C_GPS_HOLD_I                              53    // poshold_I  *100 uint16_t
#define I2C_GPS_HOLD_IMAX                           54    // poshold_IMAX *1 uint8_t

#define I2C_GPS_HOLD_RATE_P                         55    // poshold_rate_P  *10 uint16_t
#define I2C_GPS_HOLD_RATE_I                         56    // poshold_rate_I  *100 uint16_t
#define I2C_GPS_HOLD_RATE_D                         57    // poshold_rate_D  *1000 uint16_t
#define I2C_GPS_HOLD_RATE_IMAX                      58    // poshold_rate_IMAX *1 uint8_t

#define I2C_GPS_NAV_P                               59    // nav_P  *10 uint16_t
#define I2C_GPS_NAV_I                               60    // nav_I  *100 uint16_t
#define I2C_GPS_NAV_D                               61    // nav_D  *1000 uint16_t
#define I2C_GPS_NAV_IMAX                            62    // nav_IMAX *1 uint8_t

#define I2C_GPS_WP0                                 63   //Waypoint 0 used for RTH location      (R/W)
#define I2C_GPS_WP1                                 74
#define I2C_GPS_WP2                                 85
#define I2C_GPS_WP3                                 96
#define I2C_GPS_WP4                                 107
#define I2C_GPS_WP5                                 118
#define I2C_GPS_WP6                                 129
#define I2C_GPS_WP7                                 140
#define I2C_GPS_WP8                                 151
#define I2C_GPS_WP9                                 162
#define I2C_GPS_WP10                                173
#define I2C_GPS_WP11                                184
#define I2C_GPS_WP12                                195
#define I2C_GPS_WP13                                206
#define I2C_GPS_WP14                                217
#define I2C_GPS_WP15                                228
///////////////////////////////////////////////////////////////////////////////////////////////////
// End register definition
///////////////////////////////////////////////////////////////////////////////////////////////////

#endif

/**************************************************************************************/
/***************               Error Checking Section              ********************/
/**************************************************************************************/

#ifndef NUMBER_MOTOR
        #error "NUMBER_MOTOR is not set, most likely you have not defined any type of multicopter"
#endif

#if (defined(LCD_CONF) || defined(LCD_TELEMETRY)) && !(defined(LCD_SERIAL3W) || defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) )
  #error "LCD_CONF or LCD_TELEMETRY defined, and choice of LCD not defined.  Uncomment one of LCD_SERIAL3W or LCD_TEXTSTAR or LCD_VT100 or LCD_ETPP or LCD_LCD03 or OLED_I2C_128x64"
#endif

#if defined(POWERMETER) && !(defined(VBAT))
        #error "to use powermeter, you must also define and configure VBAT"
#endif

#if defined(LCD_TELEMETRY_AUTO) && !(defined(LCD_TELEMETRY))
        #error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif

#if defined(LCD_TELEMETRY_STEP) && !(defined(LCD_TELEMETRY))
        #error "to use single step telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif

#if defined(VBAT) && !(defined(BUZZER))
        //#error "to use VBAT, you must also configure BUZZER"
#endif


#endif /* DEF_H_ */
