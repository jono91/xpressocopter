/*
 * GPS.h
 *
 *  Created on: 3/07/2013
 *      Author: Jono
 */

#ifndef GPS_H_
#define GPS_H_
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


#define _X 1
#define _Y 0

#define RADX100                    0.000174532925
#define CROSSTRACK_GAIN            1
#define NAV_SPEED_MIN              100    // cm/sec
#define NAV_SPEED_MAX              300    // cm/sec
#define NAV_SLOW_NAV               true
#define NAV_BANK_MAX 3000        //30deg max banking when navigating (just for security and testing)




void GPS_I2C_command(uint8_t command, uint8_t wp);
void GPS_NewData();
void GPS_reset_home_position();
void GPS_reset_nav();
void GPS_set_pids();
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
static bool check_missed_wp();
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing);
static void GPS_calc_velocity();
int16_t medianFilter(int16_t data[]);
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng );
static void GPS_calc_poshold();
static void GPS_calc_nav_rate(uint16_t max_speed);
static void GPS_update_crosstrack(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);
int32_t wrap_18000(int32_t ang);
int32_t wrap_36000(int32_t ang);
#endif /* GPS_H_ */
