/****************************************************************************
 *   $Id:: timer16.h 6956 2011-03-23 23:03:25Z usb00423                     $
 *   Project: NXP LPC13Uxx software example
 *
 *   Description:
 *     This file contains definition and prototype for 16-bit timer 
 *     configuration.
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
****************************************************************************/
#ifndef __PWM_H
#define __PWM_H

#include "type.h"

/* The test is either MAT_OUT or CAP_IN. Default is MAT_OUT. */
#define TIMER_MATCH		0


#define EMC0	4
#define EMC1	6
#define EMC2	8
#define EMC3	10

#define MATCH0	(1<<0)
#define MATCH1	(1<<1)
#define MATCH2	(1<<2)
#define MATCH3	(1<<3)

#define M1_PORT 1		// Port for led
#define M1_BIT 13		// Bit on port for led
#define M1_ON 1		// Level to set port to turn on led
#define M1_OFF 0		// Level to set port to turn off led


#define M2_PORT 1		// Port for led
#define M2_BIT 14		// Bit on port for led
#define M2_ON 1		// Level to set port to turn on led
#define M2_OFF 0		// Level to set port to turn off led

#define M3_PORT 1		// Port for led
#define M3_BIT 15		// Bit on port for led
#define M3_ON 1		// Level to set port to turn on led
#define M3_OFF 0		// Level to set port to turn off led

#define MR0I 0
#define MR1I 3
#define MR2I 6
#define MR2R 7
#define MR2S 8
#define MR3I 9
#define MR3R 10
#define MR3S 11



///* For 16-bit timer, make sure that TIME_INTERVAL should be no
//greater than 0xFFFF. */
#ifndef TIME_INTERVAL
#define TIME_INTERVAL	(SystemCoreClock/1000 - 1)
#endif

void writeServos(void);
void writeMotors(void);
void writeAllMotors(int16_t mc);
void CT16B0_IRQHandler(void);
void CT32B0_IRQHandler(void);
void enable_PWMtimer(void);
void disable_PWMtimer(void);
void reset_PWMtimer(void);
void init_timer16PWM(void);
void mixTable(void);


#endif /* end __PWM_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
