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
#include "microsec.h"
//#include "nmi.h"

static volatile uint32_t millisCount = 0;
volatile int delayCount = 0;




/******************************************************************************
** Function name:		CT16B1_IRQHandler
**
** Descriptions:		Timer/CounterX and CaptureX interrupt handler
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void CT16B1_IRQHandler(void)
{
  if ( LPC_CT16B1->IR & (0x1<<0) )
  {  
	LPC_CT16B1->IR = 0x1<<0;			/* clear interrupt flag */

	if(delayCount <=0)
		delayCount++;
	millisCount++;
  }
  if ( LPC_CT16B1->IR & (0x1<<1) )
  {  
	LPC_CT16B1->IR = 0x1<<1;			/* clear interrupt flag */
	//timer16_1_counter[1]++;
  }
  if ( LPC_CT16B1->IR & (0x1<<2) )
  {  
	LPC_CT16B1->IR = 0x1<<2;			/* clear interrupt flag */
	//timer16_1_counter[2]++;
  }
  if ( LPC_CT16B1->IR & (0x1<<3) )
  {  
	LPC_CT16B1->IR = 0x1<<3;			/* clear interrupt flag */
	//timer16_1_counter[3]++;
  }
  if ( LPC_CT16B1->IR & (0x1<<4) )
  {
	LPC_CT16B1->IR = 0x1<<4;		/* clear interrupt flag */
	//timer16_1_capture[0]++;
  }
  if ( LPC_CT16B1->IR & (0x1<<5) )
  {
	LPC_CT16B1->IR = 0x1<<5;		/* clear interrupt flag */
	//timer16_1_capture[1]++;
  }
  if ( LPC_CT16B1->IR & (0x1<<6) )
  {
	LPC_CT16B1->IR = 0x1<<6;		/* clear interrupt flag */
	//timer16_1_capture[2]++;
  }
  if ( LPC_CT16B1->IR & (0x1<<7) )
  {
	LPC_CT16B1->IR = 0x1<<7;		/* clear interrupt flag */
	//timer16_1_capture[3]++;
  }
  return;
}

/******************************************************************************
** Function name:		enable_timer
**
** Descriptions:		Enable timer
**
** parameters:			timer number: 0 or 1
** Returned value:		None
** 
******************************************************************************/
void enable_microsec(void)
{


  LPC_CT16B1->TCR = 1;

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
void disable_microsec(void)
{

  LPC_CT16B1->TCR = 0;

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
void reset_microsec(void)
{
  uint32_t regVal;

  millisCount = 0;
  regVal = LPC_CT16B1->TCR;
  regVal |= 0x02;
  LPC_CT16B1->TCR = regVal;

  return;
}




/******************************************************************************
** Function name:		init_timer
**
** Descriptions:		Initialize timer, set timer interval, reset timer,
**						install timer interrupt handler
**
** parameters:			timer number and timer interval
** Returned value:		None
** 
******************************************************************************/
void init_microsec(void)
{
	millisCount = 0;

   /* Some of the I/O pins need to be clearfully planned if
   you use below module because JTAG and TIMER CAP/MAT pins are muxed. */
   LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);
   LPC_CT16B1->PR = 71;
   LPC_CT16B1->MR0 = 1000;

   LPC_CT16B1->MCR = (0x3<<0);  /* Interrupt and Reset on MR0 and MR1 */

   /* Enable the TIMER1 Interrupt */
//#if NMI_ENABLED
   //NVIC_DisableIRQ(CT16B1_IRQn);
   //NMI_Init( CT16B1_IRQn );
//#else
   NVIC_EnableIRQ(CT16B1_IRQn);
//#endif

  return;
}

void delayMs(uint32_t ms)
{
	delayCount = -ms;
	while (delayCount <0);

}

uint32_t micros(void)
{
	return (millisCount*1000 + LPC_CT16B1->TC);
}

uint32_t millis(void)
{
	return millisCount;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
