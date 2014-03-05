/*
 * RX.h
 *
 *  Created on: 3/02/2013
 *      Author: Jono
 */

#ifndef RX_H_
#define RX_H_

#include "type.h"


#define edgeTrig 0
#define bothEdges 2

#define RX0_CH 0		// interrupt channel for RX pin 0
#define RX0_PORT 0		// Port for RX pin 0
#define RX0_BIT 2		// Bit on port for RX pin 0

#define RX1_CH 1		// interrupt channel for RX pin 1
#define RX1_PORT 0		// Port for RX pin 1
#define RX1_BIT 20		// Bit on port for RX pin 1

#define RX2_CH 2		// interrupt channel for RX pin 2
#define RX2_PORT 1		// Port for RX pin 2
#define RX2_BIT 19		// Bit on port for RX pin 2

#define RX3_CH 3		// interrupt channel for RX pin 3
#define RX3_PORT 1		// Port for RX pin 3
#define RX3_BIT 25		// Bit on port for RX pin 3

#define RX4_CH 4		// interrupt channel for RX pin 4
#define RX4_PORT 1		// Port for RX pin 4
#define RX4_BIT 16		// Bit on port for RX pin 4

#define RX5_CH 5		// interrupt channel for RX pin 5
#define RX5_PORT 0		// Port for RX pin 5
#define RX5_BIT 17		// Bit on port for RX pin 5

#define SONARPWM_CH 6 		//interrupt channel for sonar pwm input
#define SONARPWM_PORT 0
#define SONARPWM_BIT  22


void configureReceiver(void);
void PIN_INT0_IRQHandler(void);
void PIN_INT1_IRQHandler(void);
void PIN_INT2_IRQHandler(void);
void PIN_INT3_IRQHandler(void);
void PIN_INT4_IRQHandler(void);
void PIN_INT5_IRQHandler(void);
void PIN_INT6_IRQHandler(void);
void PIN_INT7_IRQHandler(void);
void computeRC();
#endif /* RX_H_ */
