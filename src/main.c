/*****************************************************************************
 *   i2ctest.c:  main C entry file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.07.20  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#include "LPC13Uxx.h"			/* LPC13xx Peripheral Registers */
#include "type.h"
#include "config.h"
#include "def.h"
#include "i2c.h"
#include "gpio.h"
#include "serial.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "microsec.h"
#include "sensors.h"
#include "PWM.h"
#include "RX.h"
#include "EEPROM.h"
#include "IMU.h"
#include "common.h"
#include "math.h"
#if GPS
	#include "GPS.h"
#endif
#ifdef MEDFILTER
#include "medianFilter.h"
#endif
#ifdef FLOW
#include "FLOW.h"
#endif

#ifdef MAYHONY
volatile int16_t gyroAHRS[3];
#endif

#define GYRO_I_MAX 256



volatile uint32_t currentTime = 0;
volatile uint32_t previousTime = 0;
volatile uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
volatile uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
volatile uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
volatile uint16_t calibratingG;
volatile uint16_t acc_1G;             // this is the 1G measured acceleration
volatile int16_t  acc_25deg;
volatile int16_t  headFreeModeHold;
volatile int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
volatile int16_t  heading,magHold;
volatile uint8_t  vbat;               // battery voltage in 0.1V steps
volatile uint8_t  rcOptions[CHECKBOXITEMS];
volatile int32_t  BaroAlt;
volatile int32_t  EstAlt;             // in cm
volatile int16_t  BaroPID = 0;
volatile int32_t  AltHold;
volatile int16_t  errorAltitudeI = 0;
volatile int16_t  vario = 0;              // variometer in cm/s

#if defined(BUZZER)
  volatile uint8_t  toggleBeep = 0;
#endif
#if defined(ARMEDTIMEWARNING)
  volatile uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

volatile int16_t  debug[4];
volatile int16_t  sonarAlt; //to think about the unit

#ifdef MEDFILTER
#ifdef SONAR
volatile filterHistory_t SonarFilter;
#endif
#ifdef FLOW
volatile filterHistory_t FlowFiltLon;
volatile filterHistory_t FlowFiltLat;
#endif
#endif

volatile struct flags_struct f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  volatile uint16_t cycleTimeMax = 0;       // highest ever cycle timen
  volatile uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
  volatile uint16_t powerMax = 0;           // highest ever current
  volatile uint32_t armedTime = 0;
  volatile int32_t  BAROaltStart = 0;       // offset value from powerup
  volatile int32_t	BAROaltMax = 0;	        // maximum value
#endif

volatile int16_t  i2c_errors_count = 0;
volatile int16_t  annex650_overrun_count = 0;

// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
  volatile uint16_t InflightcalibratingA = 0;
  volatile int16_t AccInflightCalibrationArmed;
  volatile uint16_t AccInflightCalibrationMeasurementDone = 0;
  volatile uint16_t AccInflightCalibrationSavetoEEProm = 0;
  volatile uint16_t AccInflightCalibrationActive = 0;
#endif

// **********************
// power meter
// **********************
#if defined(POWERMETER)
#define PMOTOR_SUM 8                     // index into pMeter[] for sum
  volatile uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  volatile uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  volatile uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  volatile uint16_t powerValue = 0;          // last known current
#endif
volatile uint16_t intPowerMeterSum, intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
  volatile uint8_t telemetry = 0;
  volatile uint8_t telemetry_auto = 0;
#endif
// ******************
// rc functions
// ******************


volatile int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

volatile int16_t rcData[8];          // interval [1000;2000]
volatile int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
volatile int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
volatile int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
volatile uint8_t rcFrameComplete; // for serial rc receiver Spektrum

#if defined(OPENLRSv2MULTI)
  volatile uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
#endif

// **************
// gyro+acc IMU
// **************
volatile int16_t gyroData[3] = {0,0,0};
volatile int16_t gyroZero[3] = {0,0,0};
volatile int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// *************************
// motor and servo functions
// *************************
volatile int16_t axisPID[3];
volatile int16_t motor[NUMBER_MOTOR];
#ifdef SERVO
  volatile int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
#endif

// ************************
// EEPROM Layout definition
// ************************
volatile uint8_t dynP8[3], dynD8[3];


volatile struct conf_def conf;

// **********************
// GPS common variables
// **********************
volatile int32_t  GPS_coord[2];
int32_t  GPS_home[2];
int32_t  GPS_hold[2];
volatile uint8_t  GPS_numSat;
volatile uint16_t GPS_distanceToHome;                          // distance to home in meters
volatile int16_t  GPS_directionToHome;                         // direction to home in degrees
volatile uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
volatile uint8_t  GPS_update = 0;                              // it's a binary toogle to distinct a GPS position update
volatile int16_t  GPS_angle[2] = { 0, 0};                      // it's the angles that must be applied for GPS correction
volatile uint16_t GPS_ground_course = 0;                       // degrees*10
volatile uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
volatile uint8_t  GPS_Enable  = 0;


// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
volatile int16_t  nav[2];
volatile int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial GPS only variables
//navigation mode
#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
volatile uint8_t nav_mode = NAV_MODE_NONE;            //Navigation mode

#ifdef FLOW
    volatile uint16_t frame_count;// counts created I2C frames
    volatile int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep
    volatile int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep
    volatile uint8_t qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
    volatile uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
    volatile int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
#endif



#if BARO
    volatile int32_t baroPressure;
    volatile int32_t baroTemperature;
    volatile int32_t baroPressureSum;
#endif


void setup() {
#if !defined(GPS_PROMINI)
    UARTInit(115200);
#endif
    LEDPIN_PINMODE;
    SHIELDLED_PINMODE;
    //POWERPIN_PINMODE;
    //BUZZERPIN_PINMODE;
    //STABLEPIN_PINMODE;
    //POWERPIN_OFF;


    /* Initialize GPIO (sets up clock) */
    GPIOInit();
    init_microsec();
    enable_microsec();
    init_timer16PWM();
    enable_PWMtimer();

    /********  special version of MultiWii to calibrate all attached ESCs ************/
#if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    delayMs(3000);
    writeAllMotors(ESC_CALIB_LOW);
    delayMs(500);
    while (1) {
        delayMs(5000);
        blinkLED(2,20, 2);
    }
    // statement never reached
#endif

    writeAllMotors(MINCOMMAND);
    delayMs(300);

    readEEPROM();
    checkFirstTime();
    configureReceiver();
#if defined(OPENLRSv2MULTI)
    initOpenLRS();
#endif
    initSensors();
#if defined(I2C_GPS) || defined(GPS_SERIAL) || defined(GPS_FROM_OSD)||defined(I2C_NAV)
    GPS_set_pids();
#endif
    previousTime = micros();
#if defined(GIMBAL)
    calibratingA = 400;
#endif
    calibratingG = 400;
    calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
#if defined(POWERMETER)
    for(uint8_t i=0;i<=PMOTOR_SUM;i++)
        pMeter[i]=0;
#endif
#if defined(ARMEDTIMEWARNING)
    ArmedTimeWarningMicroSeconds = (ARMEDTIMEWARNING *1000000);
#endif
    /************************************/
#if defined(GPS_SERIAL)
    SerialOpen(GPS_SERIAL,GPS_BAUD);
    delay(400);
    for(uint8_t i=0;i<=5;i++){
        GPS_NewData();
        LEDPIN_ON
        delay(20);
        LEDPIN_OFF
        delay(80);
    }
    if(!GPS_Present){
        SerialEnd(GPS_SERIAL);
        SerialOpen(0,SERIAL_COM_SPEED);
    }
#if !defined(GPS_PROMINI)
    GPS_Present = 1;
#endif
    GPS_Enable = GPS_Present;
#endif
    /************************************/

#if defined(I2C_GPS) || defined(TINY_GPS) || defined(GPS_FROM_OSD)|| defined(I2C_NAV)
    GPS_Enable = 1;
#endif

#if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64)
    initLCD();
#endif
#ifdef LCD_TELEMETRY_DEBUG
    telemetry_auto = 1;
#endif
#ifdef LCD_CONF_DEBUG
    configurationLoop();
#endif
#ifdef LANDING_LIGHTS_DDR
    init_landing_lights();
#endif

#if defined(LED_FLASHER)
    init_led_flasher();
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
#endif
    f.SMALL_ANGLES_25=1; // important for gyro only conf

    //initialise median filter structures
#ifdef MEDFILTER
#ifdef SONAR
    initMedianFilter(&SonarFilter, 5);
#endif
#ifdef FLOW
    initMedianFilter(&FlowFiltLon, 3);
    initMedianFilter(&FlowFiltLat, 3);
#endif
#endif

    //initWatchDog();
}


/*******************************************************************************
**   Main Function  main()
*******************************************************************************/
int main (void)
{
    /* Basic chip initialization is taken care of in SystemInit() called
     * from the startup code. SystemInit() and chip settings are defined
     * in the CMSIS system_<part family>.c file.
     */

    setup();

    static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    uint8_t axis,i;

    int16_t delta,deltaSum;
    int16_t PTerm,ITerm,DTerm;
    static int16_t lastGyro[3] = {0,0,0};
    static int16_t delta1[3],delta2[3];

    static int16_t errorAngleI[2] = {0,0};

    static uint32_t rcTime  = 0;
    static int16_t initialThrottleHold;
    int16_t error,errorAngle =0;

#if PIDcontroller ==1
    static int16_t errorGyroI[3] = {0,0,0};
#elif PIDcontroller == 2
    int32_t AngleRateTmp = 0, RateError = 0, AngleITerm = 0;
    static int32_t errorGyroI[3] = {0,0,0};
#endif

#define RC_FREQ 50

    while(1)
    {
#ifdef LCD_TELEMETRY_STEP
        static char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
        static uint8_t telemetryStepIndex = 0;
#endif

#if defined(SPEKTRUM)
        if (rcFrameComplete) computeRC();
#endif
#if defined(OPENLRSv2MULTI)
        Read_OpenLRS_RC();
#endif



        if (currentTime > rcTime ) { // 50Hz
            rcTime = currentTime + 20000;
            computeRC();
            // Failsafe routine - added by MIS
#if defined(FAILSAFE)
            if ( failsafeCnt > (5*FAILSAVE_DELAY) && f.ARMED) {                  // Stabilize, and set Throttle to specified level
                for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
                rcData[THROTTLE] = FAILSAVE_THROTTLE;
                if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
                    f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                    f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
                }
                failsafeEvents++;
            }
            if ( failsafeCnt > (5*FAILSAVE_DELAY) && !f.ARMED) {  //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
                f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
            }
            failsafeCnt++;
#endif
            // end of failsave routine - next change is made with RcOptions setting
            if (rcData[THROTTLE] < MINCHECK) {
                errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
                errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
                rcDelayCommand++;
                if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && !f.ARMED) {
                    if (rcDelayCommand == 20) {
                        calibratingG=400;
#if GPS
                        GPS_reset_home_position();
#endif
                    }
                } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
                    if (rcDelayCommand == 20) {
#ifdef TRI
                        servo[5] = 1500; // we center the yaw servo in conf mode
                        writeServos();
#endif
#ifdef FLYING_WING
                        servo[0]  = conf.wing_left_mid;
                        servo[1]  = conf.wing_right_mid;
                        writeServos();
#endif
#ifdef AIRPLANE
                        for(i = 4; i<7 ;i++) servo[i] = 1500;
                        writeServos();
#endif
#if defined(LCD_CONF)
                        configurationLoop(); // beginning LCD configuration
#endif
                        previousTime = micros();
                    }
                }
#if defined(INFLIGHT_ACC_CALIBRATION)
                else if (!f.ARMED && rcData[YAW] < MINCHECK && rcData[PITCH] > MAXCHECK && rcData[ROLL] > MAXCHECK){
                    if (rcDelayCommand == 20){
                        if (AccInflightCalibrationMeasurementDone){                // trigger saving into eeprom after landing
                            AccInflightCalibrationMeasurementDone = 0;
                            AccInflightCalibrationSavetoEEProm = 1;
                        }else{
                            AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
#if defined(BUZZER)
                            if (AccInflightCalibrationArmed){
                                toggleBeep = 2;
                            } else {
                                toggleBeep = 3;
                            }
#endif
                        }
                    }
                }
#endif
                else if (conf.activate[BOXARM] > 0) {
                    if ( rcOptions[BOXARM] && f.OK_TO_ARM
#if defined(FAILSAFE)
                            && failsafeCnt <= 1
#endif
                    ) {
                        f.ARMED = 1;
                        headFreeModeHold = heading;
                    } else if (f.ARMED) f.ARMED = 0;
                    rcDelayCommand = 0;
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
                } else if ( (rcData[YAW] < MINCHECK )  && f.ARMED) {
                    if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
                } else if ( (rcData[YAW] > MAXCHECK ) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
                    if (rcDelayCommand == 20) {
                        f.ARMED = 1;
                        headFreeModeHold = heading;
                    }
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
                } else if ( (rcData[ROLL] < MINCHECK)  && f.ARMED) {
                    if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
                } else if ( (rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
                    if (rcDelayCommand == 20) {
                        f.ARMED = 1;
                        headFreeModeHold = heading;
                    }
#endif
#ifdef LCD_TELEMETRY_AUTO
                } else if (rcData[ROLL] < MINCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
                    if (rcDelayCommand == 20) {
                        if (telemetry_auto) {
                            telemetry_auto = 0;
                            telemetry = 0;
                        } else
                            telemetry_auto = 1;
                    }
#endif
#ifdef LCD_TELEMETRY_STEP
                } else if (rcData[ROLL] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
                    if (rcDelayCommand == 20) {
                        telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
                        LCDclear(); // make sure to clear away remnants
                    }
#endif
                } else
                    rcDelayCommand = 0;
            } else if (rcData[THROTTLE] > MAXCHECK && !f.ARMED) {
                if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {        // throttle=max, yaw=left, pitch=min
                    if (rcDelayCommand == 20) calibratingA=400;
                    rcDelayCommand++;
                } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) { // throttle=max, yaw=right, pitch=min
                    if (rcDelayCommand == 20) f.CALIBRATE_MAG = 1; // MAG calibration request
                    rcDelayCommand++;
                } else if (rcData[PITCH] > MAXCHECK) {
                    conf.angleTrim[PITCH]+=2;writeParams(1);
#if defined(LED_RING)
                    blinkLedRing();
#endif
                } else if (rcData[PITCH] < MINCHECK) {
                    conf.angleTrim[PITCH]-=2;writeParams(1);
#if defined(LED_RING)
                    blinkLedRing();
#endif
                } else if (rcData[ROLL] > MAXCHECK) {
                    conf.angleTrim[ROLL]+=2;writeParams(1);
#if defined(LED_RING)
                    blinkLedRing();
#endif
                } else if (rcData[ROLL] < MINCHECK) {
                    conf.angleTrim[ROLL]-=2;writeParams(1);
#if defined(LED_RING)
                    blinkLedRing();
#endif
                } else {
                    rcDelayCommand = 0;
                }
            }
#if defined(LED_FLASHER)
            led_flasher_autoselect_sequence();
#endif

#if defined(INFLIGHT_ACC_CALIBRATION)
            if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ){ // Copter is airborne and you are turning it off via boxarm : start measurement
                InflightcalibratingA = 50;
                AccInflightCalibrationArmed = 0;
            }
            if (rcOptions[BOXPASSTHRU]) {      // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
                if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone){
                    InflightcalibratingA = 50;
                }
            }else if(AccInflightCalibrationMeasurementDone && !f.ARMED){
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
            }
#endif

            uint16_t auxState = 0;
            for(i=0;i<4;i++)
                auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
            for(i=0;i<CHECKBOXITEMS;i++)
                rcOptions[i] = (auxState & conf.activate[i])>0;

            // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
            if (( rcOptions[BOXACC] || (failsafeCnt > 5*FAILSAVE_DELAY) ) && ACC ) {
                // bumpless transfer to Level mode
                if (!f.ACC_MODE) {
                    errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
                    f.ACC_MODE = 1;
                }
            } else {
                // failsafe support
                f.ACC_MODE = 0;
            }

            if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
            //if (f.ACC_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}

#if BARO
            if (rcOptions[BOXBARO]) {
                if (!f.BARO_MODE) {
                    f.BARO_MODE = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                    BaroPID=0;
                }
            } else {
                f.BARO_MODE = 0;
            }
#endif
#if MAG
            if (rcOptions[BOXMAG]) {
                if (!f.MAG_MODE) {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            } else {
                f.MAG_MODE = 0;
            }
            if (rcOptions[BOXHEADFREE]) {
                if (!f.HEADFREE_MODE) {
                    f.HEADFREE_MODE = 1;
                }
            } else {
                f.HEADFREE_MODE = 0;
            }
            if (rcOptions[BOXHEADADJ]) {
                headFreeModeHold = heading; // acquire new heading
            }
#endif

#if GPS
#if defined(I2C_NAV)
            static uint8_t GPSNavReset = 1;
            if (f.GPS_FIX && GPS_numSat >= 5 ) {
                if (!rcOptions[BOXGPSHOME] && !rcOptions[BOXGPSHOLD] )
                {    //Both boxes are unselected
                    if (GPSNavReset == 0 ) {
                        GPSNavReset = 1;
                        GPS_reset_nav();
                    }
                }
                if (rcOptions[BOXGPSHOME]) {
                    if (!f.GPS_HOME_MODE)  {
                        f.GPS_HOME_MODE = 1;
                        GPSNavReset = 0;
                        GPS_I2C_command(I2C_GPS_COMMAND_START_NAV,0);        //waypoint zero
                    }
                } else {
                    f.GPS_HOME_MODE = 0;
                }
                if (rcOptions[BOXGPSHOLD]) {
                    if (!f.GPS_HOLD_MODE & !f.GPS_HOME_MODE) {
                        f.GPS_HOLD_MODE = 1;
                        GPSNavReset = 0;
                        GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD,0);
                    }
                } else {
                    f.GPS_HOLD_MODE = 0;
                }
            }
#endif
#if defined(GPS_SERIAL) || defined(TINY_GPS) || defined(GPS_FROM_OSD)||defined(I2C_GPS)
            if (f.GPS_FIX && GPS_numSat >= 5 ) {
                if (rcOptions[BOXGPSHOME]) {
                    if (!f.GPS_HOME_MODE)  {
                        f.GPS_HOME_MODE = 1;
                        GPS_set_next_wp(&GPS_home[LAT],&GPS_home[LON]);
                        nav_mode    = NAV_MODE_WP;
                    }
                } else {
                    f.GPS_HOME_MODE = 0;
                }
                if (rcOptions[BOXGPSHOLD]) {
                    if (!f.GPS_HOLD_MODE) {
                        f.GPS_HOLD_MODE = 1;
                        GPS_hold[LAT] = GPS_coord[LAT];
                        GPS_hold[LON] = GPS_coord[LON];
                        GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
                        nav_mode = NAV_MODE_POSHOLD;
                    }
                } else {
                    f.GPS_HOLD_MODE = 0;
                }
            }
#endif
#endif
#ifdef FLOW
            if(rcOptions[BOXFLOWHOLD])
            {
                if (!f.FLOW_HOLD_MODE)
                {
                    f.FLOW_HOLD_MODE = 1;
                    GPS_hold[LAT] = GPS_coord[LAT];
                    GPS_hold[LON] = GPS_coord[LON];
                    flow_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
                    nav_mode = NAV_MODE_POSHOLD;
                }
                //debug[1] = 1;
            }
            else
            {
                f.FLOW_HOLD_MODE = 0;
                //debug[1] = 0;
            }

#endif
            if (rcOptions[BOXPASSTHRU]) {f.PASSTHRU_MODE = 1;}
            else {f.PASSTHRU_MODE = 0;}

#ifdef FIXEDWING
            f.HEADFREE_MODE = 0;
#endif
        } else { // not in rc loop
            static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
            if(taskOrder>4) taskOrder-=5;
            switch (taskOrder) {
            case 0:
                taskOrder++;
#if MAG
                if (Mag_getADC()) break; // max 350 µs (HMC5883) // only break when we actually did something
#endif
            case 1:
                taskOrder++;
#if BARO
                if (Baro_update() != 0 ) break;
#endif
            case 2:
                taskOrder++;
#if BARO
                if (getEstimatedAltitude() !=0) break;
#endif
            case 3:
                taskOrder++;
                if(flowUpdate() !=0) break;
#if GPS
                if(GPS_Enable) GPS_NewData();
                break;
#endif
            case 4:
                taskOrder++;
#if SONAR
#if !defined(MAXSONAR_PWM) && !defined(FLOW)
                Sonar_update();
#endif
                //debug[2] = sonarAlt;
#endif
#ifdef LANDING_LIGHTS_DDR
                auto_switch_landing_lights();
#endif
#ifdef VARIOMETER
                if (f.VARIO_MODE) vario_signaling();
#endif
                break;
            }
        }


        computeIMU();
        // Measure loop rate just afer reading the sensors
        currentTime = micros();
        cycleTime = currentTime - previousTime;
        previousTime = currentTime;


        //delayMs(500);
        //reset watchdog timer
        if(millis()>3000)//wait 3s before watchdog starts
            feedWatchDog();

        //if(cycleTime > 3150)
        //  debug[0]++;

#if MAG
        if (fabs(rcCommand[YAW]) <70 && f.MAG_MODE) {
            int16_t dif = heading - magHold;
            if (dif <= - 180) dif += 360;
            if (dif >= + 180) dif -= 360;
            if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif*conf.P8[PIDMAG]/30;  // 18 deg
        } else magHold = heading;
#endif

#if BARO
        if (f.BARO_MODE) {
            if (fabs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
                f.BARO_MODE = 0; // so that a new althold reference is defined
            }
            rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
        }
#endif
#if GPS
        if ( (!f.GPS_HOME_MODE && !f.GPS_HOLD_MODE) || !f.GPS_FIX_HOME ) {
            GPS_reset_nav(); // If GPS is not activated. Reset nav loops and all nav related parameters
            GPS_angle[ROLL]   = 0;
            GPS_angle[PITCH]  = 0;
        } else {
            float sin_yaw_y = sinf(heading*0.0174532925f);
            float cos_yaw_x = cosf(heading*0.0174532925f);

#if defined(NAV_SLEW_RATE)
            nav_rated[LON] += constrain(wrap_18000(nav[LON]-nav_rated[LON]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
            nav_rated[LAT] += constrain(wrap_18000(nav[LAT]-nav_rated[LAT]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
            GPS_angle[ROLL]   = (nav_rated[LON]*cos_yaw_x - nav_rated[LAT]*sin_yaw_y) /10;
            GPS_angle[PITCH]  = (nav_rated[LON]*sin_yaw_y + nav_rated[LAT]*cos_yaw_x) /10;
#else
            GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
            GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
#endif
        }
        //debug[2] = GPS_angle[ROLL];
        //debug[3] = GPS_angle[PITCH];
#endif

#ifdef FLOW
        if (!f.FLOW_HOLD_MODE) {
            flow_reset_nav(); // If GPS is not activated. Reset nav loops and all nav related parameters
            GPS_angle[ROLL]   = 0;
            GPS_angle[PITCH]  = 0;
        } else {
            float sin_yaw_y = sinf(heading*0.0174532925f);
            float cos_yaw_x = cosf(heading*0.0174532925f);

#if defined(NAV_SLEW_RATE)
            nav_rated[LON] += constrain(wrap_18000(nav[LON]-nav_rated[LON]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
            nav_rated[LAT] += constrain(wrap_18000(nav[LAT]-nav_rated[LAT]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
            GPS_angle[ROLL]   = (nav_rated[LON]*cos_yaw_x - nav_rated[LAT]*sin_yaw_y) /10;
            GPS_angle[PITCH]  = (nav_rated[LON]*sin_yaw_y + nav_rated[LAT]*cos_yaw_x) /10;
#else
            GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
            GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
#endif
        }
        //debug[2] = GPS_angle[ROLL];
        //debug[3] = GPS_angle[PITCH];
#endif


#if PIDcontroller == 1
        //**** PITCH & ROLL & YAW PID ****
        for(axis=0;axis<3;axis++) {
            if (f.ACC_MODE && axis<2 ) { //LEVEL MODE
                // 50 degrees max inclination
                errorAngle = constrain(2*rcCommand[axis] + GPS_angle[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
#ifdef LEVEL_PDF
                PTerm      = -(int32_t)angle[axis]*conf.P8[PIDLEVEL]/100 ;
#else
                PTerm      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
#endif
                PTerm = constrain(PTerm,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

                errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
                ITerm              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
            } else { //ACRO MODE or YAW axis
                if (fabs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
                error -= gyroData[axis];

                PTerm = rcCommand[axis];

                errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
                if (fabs(gyroData[axis])>640) errorGyroI[axis] = 0;
                ITerm = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
            }
            if (fabs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
            else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation

            delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastGyro[axis] = gyroData[axis];
            deltaSum       = delta1[axis]+delta2[axis]+delta;
            delta2[axis]   = delta1[axis];
            delta1[axis]   = delta;

            if (fabs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result
            else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation

            axisPID[axis] =  PTerm + ITerm - DTerm;
        }

#elif PIDcontroller == 2
        // ----------PID controller----------
        for (axis = 0; axis < 3; axis++)
        {
            //--------------attitude outer loop------------------------
            // -----Get the desired angle rate depending on flight mode
            if (f.ACC_MODE && axis < 2 )
            { // MODE relying on ACC
                // calculate error and limit the angle to 50 degrees max inclination
                errorAngle = constrain((rcCommand[axis] << 1) + GPS_angle[axis], -500, +500) - angle[axis] + conf.angleTrim[axis]; // 16 bits is ok here
            }
            if (axis == 2)
            { // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
                AngleRateTmp = (((int32_t)(conf.yawRate + 27) * rcCommand[2]) >> 5);
            }
            else
            {
                if (!f.ACC_MODE)
                { //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                    AngleRateTmp = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
                    errorAngleI[axis] = 0;
                    AngleITerm = 0;
                }
                else
                {
                    // ANGLE mode - control is angle based, so control loop is needed
                    //proportional term constrained to Level D
                    AngleRateTmp = (errorAngle * conf.P8[PIDLEVEL]) >> 4;
                    AngleRateTmp = constrain(AngleRateTmp,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

                    //Integral term
                    errorAngleI[axis]  = constrain(errorAngleI[axis]+((errorAngle*cycleTime)>>11),-10000,+10000);// WindUp     //16 bits is ok here //normalised to 2048ms
                    AngleITerm         = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
                }
            }

            // --------rate inner loop PID. ----------
            // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
            // -----calculate scaled error.AngleRates
            // multiplication of rcCommand corresponds to changing the sticks scaling here
            RateError = AngleRateTmp - gyroData[axis];
            RateError += AngleITerm;

            // -----calculate P component
            PTerm = (RateError * dynP8[axis]) >> 7;
            // -----calculate I component
            // there should be no division before accumulating the error to integrator, because the precision would be reduced.
            // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
            // Time correction (to avoid different I scaling for different builds based on average cycle time)
            // is normalized to cycle time = 2048.
            errorGyroI[axis] = errorGyroI[axis] + ((RateError * cycleTime) >> 11) * conf.I8[axis];

            // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
            // I coefficient (I8) moved before integration to make limiting independent from PID settings
            errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t)-GYRO_I_MAX << 13, (int32_t)+GYRO_I_MAX << 13);
            ITerm = errorGyroI[axis] >> 13;

            //-----calculate D-term based on process variable to remove derivative kick
            delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastGyro[axis] = gyroData[axis];


            // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
            // would be scaled by different dt each time. Division by dT fixes that.
            delta = (delta * ((uint16_t)0xFFFF / (cycleTime >> 4))) >> 6;	//0xFFFF>>6 = 1024 ie converting cycletime to secs


            // add moving average here to reduce noise - consider median filter
            deltaSum = delta1[axis] + delta2[axis] + delta;
            delta2[axis] = delta1[axis];
            delta1[axis] = delta;
            DTerm = (deltaSum * dynD8[axis]) >> 8;

            // -----calculate total PID output
            axisPID[axis] = PTerm + ITerm - DTerm; //D subtracted as using PV based D
        }

#endif	//pidcontroller

        mixTable();
        if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
        writeMotors();
    }
    return 0;
}

void HardFault_Handler(void) {
    //fall out of sky
    writeAllMotors(MINCOMMAND);
    while (1) {
        delayMs(2000);
        LEDPIN_ON;
        delayMs(2000);
        LEDPIN_OFF;
    }
}



/******************************************************************************
 **                            End Of File
 ******************************************************************************/
