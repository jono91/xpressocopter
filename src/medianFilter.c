/*
 * medianFilter.c
 *
 *  Created on: 9/03/2014
 *      Author: jono
 */

#include "stdlib.h"
#include "medianFilter.h"
#include "common.h"
#include "config.h"
#include "def.h"

#ifdef MEDFILTER
void initMedianFilter(filterHistory_t *history, uint8_t historyLength)
{
    //max number of values to use is 7 or else too much lag is introduced
    if(historyLength >7)
        historyLength = 7;

    history->histLength = historyLength;
    history->filterValues = calloc(historyLength, sizeof(history->filterValues));
    if(history->filterValues == NULL)
    {
        WDT_IRQHandler();
    }

    history->index = 0;

}

int16_t applyMedFilter(filterHistory_t *history, int16_t newValue)
{

    int16_t sortTab[7];
    uint8_t rdy = 0, sortidx = history->index;
    uint8_t maxsortidx = history->histLength - 1;
    int16_t tmp;

    history->filterValues[sortidx] = newValue;
    history->index ++;
    if(history->index >= history->histLength)
    {
        history->index = 0;
    }

    //copy to sorting array
    for(sortidx=0;sortidx<history->histLength;sortidx++)
    {
        sortTab[sortidx] = history->filterValues[sortidx];
    }

    //optimised bubble sort
    while(rdy == 0){
        rdy = 1;
        for (sortidx = 0; sortidx < maxsortidx; sortidx++){
            tmp = sortTab[sortidx];
            if (tmp > sortTab[sortidx+1])
            {
                sortTab[sortidx] = sortTab[sortidx+1];
                sortTab[sortidx+1] = tmp;
                rdy = 0;
            }//check and swap
        }
        maxsortidx --;
    }

    tmp = sortTab[(history->histLength / 2)];

    //free(sortTab);
    return tmp;
}
#endif
