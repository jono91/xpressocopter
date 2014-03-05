/****************************************************************************
 *   $Id:: timer16.c 6950 2011-03-23 22:09:44Z usb00423                     $
 *   Project: NXP LPC13Uxx 16-bit timer example
 *
 *   Description:
 *     This file contains 16-bit timer code example which include timer 
 *     initialization, timer interrupt handler, and related APIs for 
 *     timer setup.
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
* documentation is hereby granted, under NXP Semiconductors'
* relevant copyright in the software, without fee, provided that it
* is used in conjunction with NXP Semiconductors microcontrollers.  This
* copyright, permission, and disclaimer notice must appear in all copies of
* this code.
****************************************************************************/
#include "LPC13Uxx.h"
#include "config.h"
#include "def.h"
#include "PWM.h"
#include "gpio.h"
//#include "nmi.h"



volatile uint32_t PWM_BUFFER[6] = {1502*4,1502*4,1502*4,1502*4,1502*4,1502*4};
volatile uint8_t PWM_PIN[8] = {1,2,3,4,5,6,7,8};
extern int16_t motor[NUMBER_MOTOR];

#if defined(SERVO)
	extern int16_t servo[8];
#endif

extern int16_t rcData[8];
extern int16_t rcCommand[4];
extern int16_t axisPID[3];

extern struct flags_struct f;
extern struct conf_def conf;


/***************************set up refresh rate****************************************/
#define MOTOR_RFR_VALUE 8163	//490hz refresh rate on motors
#define MOTOR_SCALE_FACTOR 4


#ifdef SERVO_RFR_50HZ
	#define SERVO_SCALE_FACTOR 2
	#define SERVO_RFR_VALUE 40000  //prescaled to tick twice every micro rather than 4 times **special case**
#else
	#define SERVO_SCALE_FACTOR 4
#endif
#ifdef SERVO_RFR_160HZ
	#define SERVO_RFR_VALUE 25000
#endif
#ifdef SERVO_RFR_250HZ
	#define SERVO_RFR_VALUE 16000
#endif
#ifdef SERVO_RFR_333HZ
	#define SERVO_RFR_VALUE 12012
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos(void) {


	#if defined(SERVO)
	uint8_t i,j = 0;
		#if defined(PRI_SERVO_FROM)    // write primary servos
			for( i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++,j++){

				#if (NUMBER_MOTOR < 4)
				if(j == 0)
					PWM_BUFFER[3] = servo[i]*SERVO_SCALE_FACTOR;
				else if (j == 1)
					PWM_BUFFER[4] = servo[i]*SERVO_SCALE_FACTOR;
				else if (j == 2)
					PWM_BUFFER[5] = servo[i]*SERVO_SCALE_FACTOR;

				#endif
			}
		#endif
	#endif
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors(void)
{ // [1000;2000] => [16000;12000]


    #if (NUMBER_MOTOR > 0)
		PWM_BUFFER[0] = motor[0]*MOTOR_SCALE_FACTOR;
    #endif
    #if (NUMBER_MOTOR > 1)
		PWM_BUFFER[1] = motor[1]*MOTOR_SCALE_FACTOR;
    #endif
    #if (NUMBER_MOTOR > 2)
		PWM_BUFFER[2] = motor[2]*MOTOR_SCALE_FACTOR;
    #endif
    #if (NUMBER_MOTOR > 3)
		PWM_BUFFER[3] = motor[3]*MOTOR_SCALE_FACTOR;
    #endif
    #if (NUMBER_MOTOR > 4)
		PWM_BUFFER[4] = motor[4]*MOTOR_SCALE_FACTOR;
		PWM_BUFFER[5] = motor[5]*MOTOR_SCALE_FACTOR;
    #endif


}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
	uint8_t i = 0;
	for ( i =0;i<NUMBER_MOTOR;i++) {
		motor[i]=mc;
	}
	writeMotors();
}

/******************************************************************************
** Function name:		CT16B0_IRQHandler
**
** Descriptions:		Timer/CounterX and CaptureX interrupt handler
**
** parameters:			None
** Returned value:		None
**
******************************************************************************/
void CT16B0_IRQHandler(void)
{
  if ( LPC_CT16B0->IR & (0x1<<0) )
  {
	LPC_CT16B0->IR = 0x1<<0;			/* clear interrupt flag */
	//GPIOSetBitValue( M1_PORT, M1_BIT, 0 );
	//LPC_CT16B0->MR0 = motorVal[0];
  }
  if ( LPC_CT16B0->IR & (0x1<<1) )
  {
	LPC_CT16B0->IR = 0x1<<1;			/* clear interrupt flag */
	//GPIOSetBitValue( M2_PORT, M2_BIT, 0 );
	//LPC_CT16B0->MR1 = motorVal[1];
  }
  if ( LPC_CT16B0->IR & (0x1<<2) )
  {
	LPC_CT16B0->IR = 0x1<<2;			/* clear interrupt flag */
	//GPIOSetBitValue( M3_PORT, M3_BIT, 0 );
	//LPC_CT16B0->MR2 = motorVal[2];
  }
  if ( LPC_CT16B0->IR & (0x1<<3) )
  {
	  LPC_CT16B0->EMR |= (MATCH0)|(MATCH1)|(MATCH2);	//set output to high manually
	  LPC_CT16B0->TCR = 1;						//restart counter

	  //write next PWM value immediately after rising edge (1 pulse train lag)
	  LPC_CT16B0->MR0 = PWM_BUFFER[3];
	  LPC_CT16B0->MR1 = PWM_BUFFER[4];
	  LPC_CT16B0->MR2 = PWM_BUFFER[5];

	  LPC_CT16B0->IR = 0x1<<3;			/* clear interrupt flag */
  }
  if ( LPC_CT16B0->IR & (0x1<<4) )
  {
	LPC_CT16B0->IR = 0x1<<4;		/* clear interrupt flag */
  }
  if ( LPC_CT16B0->IR & (0x1<<5) )
  {
	LPC_CT16B0->IR = 0x1<<5;		/* clear interrupt flag */
  }
  if ( LPC_CT16B0->IR & (0x1<<6) )
  {
	LPC_CT16B0->IR = 0x1<<6;		/* clear interrupt flag */
  }
  if ( LPC_CT16B0->IR & (0x1<<7) )
  {
	LPC_CT16B0->IR = 0x1<<7;		/* clear interrupt flag */
  }
  return;
}
/******************************************************************************
** Function name:		CT32B0_IRQHandler
**
** Descriptions:		Timer/CounterX and captureX interrupt handler
**
** parameters:			None
** Returned value:		None
**
******************************************************************************/
void CT32B0_IRQHandler(void)
{
  if ( LPC_CT32B0->IR & (0x01<<0) )
  {
	LPC_CT32B0->IR = 0x1<<0;			/* clear interrupt flag */
  }
  if ( LPC_CT32B0->IR & (0x01<<1) )
  {
	LPC_CT32B0->IR = 0x1<<1;			/* clear interrupt flag */
  }
  if ( LPC_CT32B0->IR & (0x01<<2) )
  {
	  LPC_CT32B0->EMR |= (MATCH0)|(MATCH1)|(MATCH3);	//set output to high manually
	  LPC_CT32B0->TCR = 1;						//restart counter

	  //write next PWM value immediately after rising edge (1 pulse train lag)
	  LPC_CT32B0->MR3 = PWM_BUFFER[0];	//check board layout (ct32b0 mr3 is first pin)
	  LPC_CT32B0->MR0 = PWM_BUFFER[1];
	  LPC_CT32B0->MR1 = PWM_BUFFER[2];


	LPC_CT32B0->IR = 0x1<<2;			/* clear interrupt flag */
  }
  if ( LPC_CT32B0->IR & (0x01<<3) )
  {
	LPC_CT32B0->IR = 0x1<<3;			/* clear interrupt flag */
  }
  if ( LPC_CT32B0->IR & (0x1<<4) )
  {
	LPC_CT32B0->IR = 0x1<<4;			/* clear interrupt flag */
  }
  if ( LPC_CT32B0->IR & (0x1<<5) )
  {
	LPC_CT32B0->IR = 0x1<<5;			/* clear interrupt flag */
  }
  if ( LPC_CT32B0->IR & (0x1<<6) )
  {
	LPC_CT32B0->IR = 0x1<<6;			/* clear interrupt flag */
  }
  if ( LPC_CT32B0->IR & (0x1<<7) )
  {
	LPC_CT32B0->IR = 0x1<<7;			/* clear interrupt flag */
  }
  return;
}

void enable_PWMtimer(void)
{

    LPC_CT16B0->TCR = 1;
    LPC_CT32B0->TCR = 1;
    return;
}

/******************************************************************************
** Function name:		disable_timer
**
** Descriptions:		Disable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void disable_PWMtimer(void)
{

    LPC_CT16B0->TCR = 0;
    LPC_CT32B0->TCR = 0;
    return;
}

/******************************************************************************
** Function name:		reset_timer
**
** Descriptions:		Reset timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void reset_PWMtimer(void)
{
	uint32_t regVal;


    regVal = LPC_CT16B0->TCR;
    regVal |= 0x02;
    LPC_CT16B0->TCR = regVal;

    regVal = LPC_CT32B0->TCR;
	regVal |= 0x02;
	LPC_CT32B0->TCR = regVal;
    return;
}


/******************************************************************************
** Function name:		init_timer16PWM
**
** Descriptions:		Initialize timer as PWM
**
** parameters:			timer number, period and match enable:
**						match_enable[0] = PWM for MAT0 
**						match_enable[1] = PWM for MAT1
**						match_enable[2] = PWM for MAT2
**			
** Returned value:	None
** 
******************************************************************************/
void init_timer16PWM(void)
{	
	disable_PWMtimer();



	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);//start CT16B0 peripheral clock source
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);//start CT32B0 peripheral clock source

	//Set timers to tick every 0.25 microseconds
	LPC_CT16B0->PR = 17;
	LPC_CT32B0->PR = 17;
		
	//special case for 50hz servos, needs to tick every 0.5 microseconds
	#ifdef SERVO
		#ifdef SERVO_RFR_50HZ
			LPC_CT16B0->PR = 35;
		#endif
	#endif

	/* Setup the external match register */
	LPC_CT16B0->EMR = (1<<EMC3)|(1<<EMC2)|(1<<EMC1)|(1<<EMC0)|(1<<3)|(MATCH0)|(MATCH1)|(MATCH2);
	LPC_CT32B0->EMR = (1<<EMC3)|(1<<EMC2)|(1<<EMC1)|(1<<EMC0)|(1<<3)|(MATCH0)|(MATCH1)|(MATCH2);

	/* Setup the outputs */
	/* If match0 is enabled, set the output */
	LPC_IOCON->PIO1_13           &= ~0x07;
	LPC_IOCON->PIO1_13           |= 0x02;		/* Timer0_16 MAT0 */
	//GPIOSetDir(M1_PORT, M1_BIT, 1 );

			
	LPC_IOCON->PIO1_14           &= ~0x07;
	LPC_IOCON->PIO1_14           |= 0x02;		/* Timer0_16 MAT1 */
	//GPIOSetDir(M2_PORT, M2_BIT, 1 );

	LPC_IOCON->PIO1_15           &= ~0x07;
	LPC_IOCON->PIO1_15           |= 0x02;		/* Timer0_16 MAT2 */
	//GPIOSetDir(M3_PORT, M3_BIT, 1 );

	LPC_IOCON->PIO0_18           &= ~0x07;
	LPC_IOCON->PIO0_18           |= 0x02;		/* Timer0_32 MAT0 */

	LPC_IOCON->PIO0_19           &= ~0x07;
	LPC_IOCON->PIO0_19           |= 0x02;		/* Timer0_32 MAT1 */

	LPC_IOCON->TDI_PIO0_11       &= ~0x07;
	LPC_IOCON->TDI_PIO0_11       |= 0x03;		/* Timer0_32 MAT3 */

	/* disable PWM mode use external match mode */
	LPC_CT16B0->PWMC = 0;
	LPC_CT32B0->PWMC = 0;
		
	/* Setup the match registers */
	/* set the period value to a global variable */


#ifdef SERVO
		LPC_CT16B0->MR3 = SERVO_RFR_VALUE;
#else
		LPC_CT16B0->MR3 = MOTOR_RFR_VALUE;
#endif
	LPC_CT16B0->MR0 = (0);
	LPC_CT16B0->MR1 = (0);
	LPC_CT16B0->MR2 = (0);
		
	LPC_CT32B0->MR2 = MOTOR_RFR_VALUE;
	LPC_CT32B0->MR0 = (0);
	LPC_CT32B0->MR1 = (0);
	LPC_CT32B0->MR3 = (0);
	/* Set the match control register */
	LPC_CT16B0->MCR = (1<<MR3S|1<<MR3R|1<<MR3I);				/* Stop, Reset and interrupt on MR3 */
	LPC_CT32B0->MCR = (1<<MR2S|1<<MR2R|1<<MR2I);				/* Stop, Reset and interrupt on MR2 */ //reset is on mr2 due to pin constraints
		
	/* Enable the TIMER1 Interrupt */
	NVIC_EnableIRQ(CT16B0_IRQn);
	NVIC_EnableIRQ(CT32B0_IRQn);



}


void mixTable(void) {
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  /****************                   main Mix Table                ******************/
  #ifdef BI
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT
    servo[4]  = constrain(1500 + (YAW_DIRECTION * axisPID[YAW]) + axisPID[PITCH], 1020, 2000); //LEFT
    servo[5]  = constrain(1500 + (YAW_DIRECTION * axisPID[YAW]) - axisPID[PITCH], 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = constrain(conf.tri_yaw_middle + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
  #ifdef QUADP
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #endif
  #ifdef Y4
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #endif
  #ifdef Y6
    motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT
  #endif
  #ifdef HEX6
    motor[0] = PIDMIX(-7/8,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-7/8,-1/2,-1); //FRONT_R
    motor[2] = PIDMIX(+7/8,+1/2,+1); //REAR_L
    motor[3] = PIDMIX(+7/8,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
    motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
  #endif
  #ifdef HEX6X
    motor[0] = PIDMIX(-1/2,+7/8,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-7/8,+1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+7/8,-1); //REAR_L
    motor[3] = PIDMIX(+1/2,-7/8,-1); //FRONT_L
    motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
    motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
  #endif




  /****************                Filter the Motors values                ******************/
  maxMotor=motor[0];
  for(i=1;i< NUMBER_MOTOR;i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
    if ((rcData[THROTTLE]) < MINCHECK)
      #ifndef MOTOR_STOP
        motor[i] = MINTHROTTLE;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (!f.ARMED)
      motor[i] = MINCOMMAND;
  }
  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES == 2) || defined(POWERMETER_SOFT)
    uint32_t amp;
    /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500 */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};

    if (vbat) { // by all means - must avoid division by zero
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
  	    #if (LOG_VALUES == 2)
           pMeter[i]+= amp; // sum up over time the mapped ESC input
        #endif
        #if defined(POWERMETER_SOFT)
           pMeter[PMOTOR_SUM]+= amp; // total sum over all motors
        #endif
      }
    }
  #endif
}
/******************************************************************************
**                            End Of File
******************************************************************************/
