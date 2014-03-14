/*
 * FLOW.c
 *
 *  Created on: 13/03/2014
 *      Author: jono
 */


#include "math.h"
#include "FLOW.h"
#include "config.h"
#include "def.h"
#include "i2c.h"
#include "sensors.h"

#ifdef FLOW


    extern uint16_t frame_count;// counts created I2C frames
    extern int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep
    extern int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep
    extern int16_t qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
    extern uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
    extern int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance






#endif
