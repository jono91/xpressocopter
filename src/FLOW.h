/*
 * FLOW.h
 *
 *  Created on: 13/03/2014
 *      Author: jono
 */

#ifndef FLOW_H_
#define FLOW_H_

#include "type.h"
#include "stdbool.h"

typedef struct PID_PARAM_ {
float kP;
float kI;
float kD;
float Imax;
} PID_PARAM;

typedef struct PID_ {
float   integrator; // integrator value
int32_t last_input; // last input for derivative
float   lastderivative; // last derivative for low-pass filter
float   output;
float   derivative;
} PID;


#define RADX100                    0.000174532925
#define CROSSTRACK_GAIN            1
#define NAV_SPEED_MIN              50    // cm/sec
#define NAV_SPEED_MAX              90    // cm/sec
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX 3000        //30deg max banking when navigating (just for security and testing)

#define XOFFSET 0.07f //m from centre of frame
#define YOFFSET 0.0f

#define INT16OVF 65535
uint8_t flowUpdate(void);
void flow_reset_nav(void);
void flow_set_pids(void);
void flow_set_next_wp(int32_t* lat, int32_t* lon);
void flow_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing);
int32_t wrap_18000(int32_t ang);
int32_t wrap_36000(int32_t ang);
#endif /* FLOW_H_ */
