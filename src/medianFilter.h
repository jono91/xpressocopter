/*
 * medianFilter.h
 *
 *  Created on: 9/03/2014
 *      Author: jono
 */

#ifndef MEDIANFILTER_H_
#define MEDIANFILTER_H_
#include "type.h"

typedef struct filterHistory{
    uint8_t     histLength;
    uint8_t     index;
    int16_t     *filterValues;
} filterHistory_t;

void initMedianFilter(filterHistory_t *history, uint8_t historyLength);
int16_t applyMedFilter(filterHistory_t *history, uint16_t newValue);

#endif /* MEDIANFILTER_H_ */
