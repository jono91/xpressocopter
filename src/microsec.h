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
#ifndef __MICROSEC_H
#define __MICROSEC_H



#define EMC0	4
#define EMC1	6
#define EMC2	8
#define EMC3	10

#define MATCH0	(1<<0)
#define MATCH1	(1<<1)
#define MATCH2	(1<<2)
#define MATCH3	(1<<3)




void CT16B1_IRQHandler(void);
void enable_microsec(void);
void disable_microsec(void);
void reset_microsec(void);
void init_microsec(void);
void delayMs(uint32_t ms);
uint32_t micros(void);
uint32_t millis(void);

#endif /* end __MICROSEC_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
