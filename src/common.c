/*
 * common.c
 *
 *  Created on: 27/03/2013
 *      Author: Jono
 */
#include "LPC13Uxx.h"
#include "common.h"
#include "config.h"
#include "def.h"
#include "microsec.h"
#include "gpio.h"
#include "serial.h"
#include "math.h"
#include "PWM.h"
#include "adc.h"

extern int16_t rcData[8];
extern int16_t rcCommand[4];
extern int16_t lookupPitchRollRC[6];
extern int16_t lookupThrottleRC[11];
extern int16_t  heading;
extern int16_t  headFreeModeHold;
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern uint32_t currentTime;
extern uint8_t  GPS_numSat;
extern uint8_t vbat;               // battery voltage in 0.1V steps

extern uint8_t dynP8[3], dynD8[3];

extern struct flags_struct f;
extern struct conf_def conf;

void initWatchDog(void)
{
	int i = 0;
	//enable watchdog clock
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<15);


	LPC_SYSCON->WDTOSCCTRL = 0xF3; //watch dog oscillator set to: fclkana = 2Mhz, DIVSEL = 19 which is 0.05Mhz clock

	//power up watchdog clock
	LPC_SYSCON->PDRUNCFG     &= ~(1 << 6);          /* Power-up WDT Clock       */
	for (i = 0; i < 200; i++) __NOP();

	LPC_WWDT->CLKSEL = 1;//select watchdog oscillator
	LPC_WWDT->TC = 12500;//set watchdog interval to 125ms*2*4 = 1000ms
	LPC_WWDT->MOD = 0x1; //enable watchdog counter without reset
	LPC_WWDT->WARNINT = 0;

	NVIC_EnableIRQ(WDT_IRQn);

	//feedWatchDog();

}

void feedWatchDog(void)
{
	__disable_irq();
	LPC_WWDT->FEED = 0xAA;
	LPC_WWDT->FEED = 0x55;
	__enable_irq();
}

void WDT_IRQHandler(void)
{
	//fall out of sky
	int i = 0;
	writeAllMotors(MINCOMMAND);
	while (1) {
		for(i=0;i<10000000;i++);

		LEDPIN_TOGGLE;
	}
}

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      #if defined(LED_FLASHER)
        switch_led_flasher(1);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(1);
      #endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      //BUZZERPIN_ON;
      delayMs(wait);
      //BUZZERPIN_OFF;
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(0);
      #endif
    }
    delayMs(60);
  }
}


void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds

  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  #if defined(BUZZER)
    static uint8_t  buzzerFreq;         // delay between buzzer ring
  #endif
  uint8_t axis,prop1,prop2;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE]<BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(fabs(rcData[axis]-MIDRC),499);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp/100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100;//-(uint16_t)conf.rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100;//-(uint16_t)conf.yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp/100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  #if defined(POWERMETER_HARD)
    uint16_t pMeterRaw;               // used for current reading
    static uint16_t psensorTimer = 0;
    if (! (++psensorTimer % PSENSORFREQ)) {
      pMeterRaw =  analogRead(PSENSORPIN);
      powerValue = ( PSENSORNULL > pMeterRaw ? PSENSORNULL - pMeterRaw : pMeterRaw - PSENSORNULL); // do not use abs(), it would induce implicit cast to uint and overrun
      if ( powerValue < 333) {  // only accept reasonable values. 333 is empirical
      #ifdef LCD_TELEMETRY
        if (powerValue > powerMax) powerMax = powerValue;
      #endif
      } else {
        powerValue = 333;
      }
      pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
    }
  #endif

  #if defined(VBAT)
    uint8_t i=0;
    static uint8_t vbatTimer = 0;
    static uint8_t ind = 0;
    uint16_t vbatRaw = 0;
    static uint16_t vbatRawArray[8];
    if (! (++vbatTimer % VBATFREQ)) {
    	vbatRawArray[(ind++)%8] = AnalogRead(V_BATPIN);
    	for (i=0;i<8;i++) vbatRaw += vbatRawArray[i];
    	vbat = vbatRaw / (VBATSCALE);                  // result is Vbatt in 0.1V steps
    }
	#ifdef BUZZER
		if ( ( (vbat>VBATLEVEL1_3S)
		#if defined(POWERMETER)
							 && ( (pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0) )
		#endif
						   )  || (NO_VBAT>vbat)                              ) // ToLuSe
		{                                          // VBAT ok AND powermeter ok, buzzer off
		  buzzerFreq = 0;
		#if defined(POWERMETER)
		} else if (pMeter[PMOTOR_SUM] > pAlarm) {                             // sound alarm for powermeter
		  buzzerFreq = 4;
		#endif
		} else if (vbat>VBATLEVEL2_3S) buzzerFreq = 1;
		else if (vbat>VBATLEVEL3_3S)   buzzerFreq = 2;
		else                           buzzerFreq = 4;


		buzzer(buzzerFreq); // external buzzer routine that handles buzzer events globally now
	#endif
   #endif

  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  #if defined(LED_RING)
    static uint32_t LEDTime;
    if ( currentTime > LEDTime ) {
      LEDTime = currentTime + 50000;
      i2CLedRingState();
    }
  #endif

  #if defined(LED_FLASHER)
    auto_switch_led_flasher();
  #endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 500000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  #if defined(GPS_PROMINI)
    if(GPS_Enable == 0) {serialCom();}
  #else
    serialCom();
  #endif

  #if defined(POWERMETER)
    intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
    intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE;
  #endif

  #ifdef LCD_TELEMETRY_AUTO
    static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
    static uint8_t telemetryAutoIndex = 0;
    static uint16_t telemetryAutoTimer = 0;
    if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ){
      telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
      LCDclear(); // make sure to clear away remnants
    }
  #endif
  #ifdef LCD_TELEMETRY
    static uint16_t telemetryTimer = 0;
    if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
      #if (LCD_TELEMETRY_DEBUG+0 > 0)
        telemetry = LCD_TELEMETRY_DEBUG;
      #endif
      if (telemetry) lcd_telemetry();
    }
  #endif

  #if GPS & defined(GPS_LED_INDICATOR)
    static uint32_t GPSLEDTime;
    if ( currentTime > GPSLEDTime && (GPS_numSat >= 5)) {
      GPSLEDTime = currentTime + 150000;
      SHIELDLED_TOGGLE;
    }
  #endif

  #if defined(LOG_VALUES) && (LOG_VALUES == 2)
    if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
    if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
  #endif
  #ifdef LCD_TELEMETRY
    if (f.ARMED) armedTime += (uint32_t)cycleTime;
    #if BARO
      if (!f.ARMED) {
        BAROaltStart = BaroAlt;
        BAROaltMax = BaroAlt;
      } else {
        if (BaroAlt > BAROaltMax) BAROaltMax = BaroAlt;
      }
    #endif
  #endif
}


