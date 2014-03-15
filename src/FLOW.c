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


#define GYRO_SCALE_FLOW ((2000 * PI)/((32767.0f / 4.0f ) * 180.0f))
#define PI 3.14159265359

extern uint16_t frame_count;// counts created I2C frames
extern int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep
extern int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep
extern uint8_t qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
extern uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
extern int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
uint16_t prev_frame_count = 0;//number of i2c frames updated in previous probe
uint16_t missedFrames = 0;
float globalFlowVel[2] = {0,0}; //change in position between updates in mm in ENU frame

extern int16_t gyroData[3];
extern int16_t heading;
extern int16_t debug[4];
extern int16_t sonarAlt;

float sinHeading, cosHeading;

// ************************
// EEPROM Layout definition
// ************************
extern struct conf_def conf;
extern struct flags_struct f;


// **********************
// GPS common variables
// **********************
extern int32_t  GPS_coord[2];//position in mm from initial position in earth frame (east north up)
extern int32_t  GPS_home[2];
extern int32_t  GPS_hold[2];
extern uint16_t GPS_distanceToHome;                          // distance to home in meters
extern int16_t  GPS_directionToHome;                         // direction to home in degrees
extern int16_t  GPS_angle[2];                      // it's the angles that must be applied for GPS correction
extern uint16_t GPS_ground_course;                       // degrees*10
extern uint8_t  GPS_numSat;



// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
extern int16_t  nav[2];
extern int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial GPS only variables
//navigation mode
#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
extern uint8_t nav_mode;            //Navigation mode


int16_t actual_speed[2] = {0,0};

static void flow_calc_velocity(void);
static void flow_calc_pos(void);

/**
 * updates the position and velocity info from PX4Flow
 *
 * Integrates flow info and calculates velocity when new information is present
 *
 * @param  none
 *
 * @return returns 0 if data is stale, 1 if position update has occured
 */
uint8_t flowUpdate(void)
{
    int32_t frameCountDiff = 0;

    f.GPS_FIX = TRUE;
    GPS_numSat = 5;

    probeFlowSensor();

    if (frame_count == prev_frame_count)//info is stale, so do nothing
    {
        return 0;
    }

    getFlowData();
    if(ground_distance >= 0)
    {
        sonarAlt = ground_distance/10;
    }

    frameCountDiff = frame_count - prev_frame_count;
    prev_frame_count = frame_count;
    if (frameCountDiff != 1)//check frames havent been missed
    {
        if(-frameCountDiff != INT16OVF)//check if value overflowed but are still consecutive numbers
        {
            missedFrames++;
            return 0; //return and do nothing with data as missing information will induced errors in integration
        }
    }
    debug[0] = missedFrames;
    debug[1] = sonar_timestamp;
    debug[2] = actual_speed[LON];
    debug[3] = actual_speed[LAT];

    //update trig functions
    sinHeading = sinf(heading*0.0174532925f);
    cosHeading = cosf(heading*0.0174532925f);

    flow_calc_velocity();
    flow_calc_pos();


    return 1;

}

/**
 * calculate velocity in cm/s based on flow data
 *
 * takes flow data in sensor frame, differentiate over time and convert
 * to earth frame (East North Up). Also uses 2 val moving avg filter
 *
 * @param  none
 */
static void flow_calc_velocity(void)
{
    static int16_t speed_old[2] = {0,0};
    static uint8_t init = 0;
    float GyroRate = gyroData[Zaxis] * GYRO_SCALE_FLOW;
    //compensate for z axis rotation
    flow_comp_m_x -= GyroRate * YOFFSET;
    flow_comp_m_y += GyroRate * XOFFSET;

    //convert vel to ENU frame
    globalFlowVel[LON] = ((float)flow_comp_m_x * sinHeading + (float)flow_comp_m_y * cosHeading);
    globalFlowVel[LAT] = ((float)flow_comp_m_x * cosHeading - (float)flow_comp_m_y * sinHeading);

    if (init) {

        actual_speed[LON] = globalFlowVel[LON] * 0.1f;
        actual_speed[LAT] = globalFlowVel[LAT] * 0.1f;

        actual_speed[LON] = (actual_speed[LON] + speed_old[LON]) / 2;
        actual_speed[LAT] = (actual_speed[LAT] + speed_old[LAT]) / 2;

        speed_old[LON] = actual_speed[LON];
        speed_old[LAT] = actual_speed[LAT];
    }
    init=1;

}

/**
 * calculate position in mm/s based on flow data
 *
 * takes flow data in sensor frame, convert
 * to earth frame (East North Up).
 *
 * @param  none
 */
static void flow_calc_pos(void)
{
    float tmp = sonar_timestamp * 0.001f;//milliseconds


    GPS_coord[LON] += globalFlowVel[LON] * tmp;
    GPS_coord[LAT] += globalFlowVel[LAT] * tmp;

    GPS_distanceToHome = sqrt(GPS_coord[LON]*GPS_coord[LON] + GPS_coord[LAT]*GPS_coord[LAT])/1000;

    GPS_directionToHome = 90.000f + atan2(GPS_coord[LAT], -GPS_coord[LON]) * 57.2957795f;      //Convert the output redians to 100xdeg
    if (GPS_directionToHome < 0) GPS_directionToHome += 360;
}


#endif
