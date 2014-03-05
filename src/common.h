/*
 * common.h
 *
 *  Created on: 27/03/2013
 *      Author: Jono
 */

#ifndef COMMON_H_
#define COMMON_H_
#include "type.h"

void initWatchDog(void);
void feedWatchDog(void);
void WDT_IRQHandler(void);
void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat);
void annexCode();

#endif /* COMMON_H_ */
