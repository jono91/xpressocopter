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
#include "medianFilter.h"

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
extern int16_t heading, magHold;
extern int16_t debug[4];
extern int16_t sonarAlt;

#ifdef MEDFILTER
extern filterHistory_t FlowFiltLon;
extern filterHistory_t FlowFiltLat;
#endif
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


PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;


PID posholdPID[2];
PID poshold_ratePID[2];
PID navPID[2];

int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
    return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
    pid->integrator += ((float)error * pid_param->kI) * *dt;
    pid->integrator = constrain(pid->integrator,-pid_param->Imax,pid_param->Imax);
    return pid->integrator;
}

int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
    pid->derivative = (input - pid->last_input) / *dt;

    /// Low pass filter cut frequency for derivative calculation.
    float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
    // Examples for _filter:
    // f_cut = 10 Hz -> _filter = 15.9155e-3
    // f_cut = 15 Hz -> _filter = 10.6103e-3
    // f_cut = 20 Hz -> _filter =  7.9577e-3
    // f_cut = 25 Hz -> _filter =  6.3662e-3
    // f_cut = 30 Hz -> _filter =  5.3052e-3

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    pid->derivative = pid->lastderivative + (*dt / ( filter + *dt)) * (pid->derivative - pid->lastderivative);
    // update state
    pid->last_input = input;
    pid->lastderivative    = pid->derivative;
    // add in derivative component
    return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_* pid) {
    pid->integrator = 0;
    pid->last_input = 0;
    pid->lastderivative = 0;
}

static uint16_t GPS_wp_radius    = GPS_WP_RADIUS;

// The difference between the desired rate of travel and the actual rate of travel
static int16_t rate_error[2];
static int32_t error[2];

//Currently used WP
static int32_t GPS_WP[2];

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t target_bearing;
////////////////////////////////////////////////////////////////////////////////
// Crosstrack
////////////////////////////////////////////////////////////////////////////////
// deg * 100, The original angle to the next_WP when the next_WP was set
// Also used to check when we pass a WP
static int32_t original_target_bearing;
// The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
static int16_t crosstrack_error;

// distance between plane and next_WP in cm
static uint32_t wp_distance;

// used for slow speed wind up when start navigation;
static uint16_t waypoint_speed_gov;

volatile uint8_t flow_pids_initialized;

// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;

static float  dTnav;            // Delta Time in milliseconds for navigation computations

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
static bool check_missed_wp(void);
static void flow_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* flow_lat, int32_t* flow_lng );
static void flow_calc_poshold(void);
static void flow_calc_nav_rate(uint16_t max_speed);
static void flow_update_crosstrack(void);
static uint16_t flow_calc_desired_speed(uint16_t max_speed, bool _slow);

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

    //check for new data
    probeFlowSensor();

    if (frame_count == prev_frame_count)//info is stale, so do nothing
    {
        return 0;
    }

    //get new data
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
    //debug[0] = missedFrames;
    //debug[1] = sonar_timestamp;
    debug[0] = (int16_t)GPS_coord[LON];
    debug[1] = (int16_t)GPS_coord[LAT];

    //update trig functions
    sinHeading = sinf(heading*0.0174532925f);
    cosHeading = cosf(heading*0.0174532925f);

    dTnav = (float)sonar_timestamp * 0.001f;

    flow_calc_velocity();
    flow_calc_pos();

    if (!flow_pids_initialized) {
        flow_set_pids();
        flow_pids_initialized = 1;
    }


    if (f.FLOW_HOLD_MODE )
    {    //ok we are navigating
        //do gps nav calculations here, these are common for nav and poshold

        flow_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
        flow_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);


        //int16_t speed = 0;

        switch (nav_mode) {
        case NAV_MODE_POSHOLD:
            //Desired output is in nav_lat and nav_lon
            flow_calc_poshold();
            break;
        case NAV_MODE_WP:
/*
            speed = flow_calc_desired_speed(NAV_SPEED_MAX, NAV_SLOW_NAV);      //slow navigation
            // use error as the desired rate towards the target
            //Desired output is in nav_lat and nav_lon where 1deg inclination is 100
            flow_calc_nav_rate(speed);

            //Tail control
            if (NAV_CONTROLS_HEADING) {
                if (NAV_TAIL_FIRST) {
                    magHold = wrap_18000(nav_bearing-18000)/100;
                } else {
                    magHold = nav_bearing/100;
                }
            }
            // Are we there yet ?(within 2 meters of the destination)
            if ((wp_distance <= GPS_wp_radius) || check_missed_wp()){         //if yes switch to poshold mode
                nav_mode = NAV_MODE_POSHOLD;
                if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
            }*/
            break;
        }
    } //end of g

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


    actual_speed[LON] = globalFlowVel[LON] * 0.1f;
    actual_speed[LAT] = globalFlowVel[LAT] * 0.1f;

    if (init)
    {
        actual_speed[LON] = (actual_speed[LON] + speed_old[LON]) / 2;
        actual_speed[LAT] = (actual_speed[LAT] + speed_old[LAT]) / 2;
    }

    speed_old[LON] = actual_speed[LON];
    speed_old[LAT] = actual_speed[LAT];

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
#ifdef MEDFILTER
	int16_t velSmooth[2];
	velSmooth[LON] = applyMedFilter(&FlowFiltLon, (int16_t)globalFlowVel[LON]);
	velSmooth[LAT] = applyMedFilter(&FlowFiltLat, (int16_t)globalFlowVel[LAT]);

	GPS_coord[LON] += velSmooth[LON] * dTnav;//millimeters
	GPS_coord[LAT] += velSmooth[LAT] * dTnav;
#else
    GPS_coord[LON] += globalFlowVel[LON] * dTnav;//millimeters
    GPS_coord[LAT] += globalFlowVel[LAT] * dTnav;
#endif

    GPS_distanceToHome = sqrt(GPS_coord[LON]*GPS_coord[LON] + GPS_coord[LAT]*GPS_coord[LAT])/1000;

    GPS_directionToHome = 90.000f + atan2(GPS_coord[LAT], -GPS_coord[LON]) * 57.2957795f;      //Convert the output redians to 100xdeg
    if (GPS_directionToHome < 0) GPS_directionToHome += 360;
}

void flow_set_pids() {
    posholdPID_PARAM.kP   = (float)conf.P8[PIDPOS]/100.0;
    posholdPID_PARAM.kI   = (float)conf.I8[PIDPOS]/1000.0;//smaller than default by 1/10 to increase tuning resolution
    posholdPID_PARAM.Imax = POSHOLD_IMAX*100;//max integral induced speed

    poshold_ratePID_PARAM.kP   = (float)conf.P8[PIDPOSR]/10.0;
    poshold_ratePID_PARAM.kI   = (float)conf.I8[PIDPOSR]/100.0;
    poshold_ratePID_PARAM.kD   = (float)conf.D8[PIDPOSR]/1000.0;
    poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;

    navPID_PARAM.kP   = (float)conf.P8[PIDNAVR]/10.0;
    navPID_PARAM.kI   = (float)conf.I8[PIDNAVR]/100.0;
    navPID_PARAM.kD   = (float)conf.D8[PIDNAVR]/1000.0;
    navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
}


void flow_reset_nav(void) {
    uint8_t i;

    for(i=0;i<2;i++) {
        nav_rated[i] = 0;
        nav[i] = 0;

        reset_PID(&posholdPID[i]);
        reset_PID(&poshold_ratePID[i]);
        reset_PID(&navPID[i]);
        nav_mode = NAV_MODE_NONE;

    }
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void flow_set_next_wp(int32_t* lat, int32_t* lon) {
    GPS_WP[LAT] = *lat;
    GPS_WP[LON] = *lon;

    flow_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);

    nav_bearing = target_bearing;
    flow_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
    original_target_bearing = target_bearing;
    waypoint_speed_gov = NAV_SPEED_MIN;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp(void) {
    int32_t temp;
    temp = target_bearing - original_target_bearing;
    temp = wrap_18000(temp);
    return (abs(temp) > 10000);   // we passed the waypoint by 100 degrees
}

/**
 * calculate distance between 2 points in cm
 *
 *
 * @param  int32_t* lat1 - position 1 latitude coord(mm from start point)
 * @param  int32_t* lon1 - position 1 longitude coord(mm)
 * @param  int32_t* lat2 - position 2 latitude coord(mm from start point)
 * @param  int32_t* lon2 - position 2 longitude coord(mm)
 *
 * @param  uint32_t* dist - output distance in cm
 * @param  int32_t* bearing - output bearing in deg*100
 *
 */
void flow_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
    float dLat = *lat2 - *lat1;                                    // difference of latitude in mm
    float dLon = *lon2 - *lon1;
    *dist = sqrt(dLat*dLat + dLon*dLon)/10;

    *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
    if (*bearing < 0) *bearing += 36000;
}


/**
 * calculate location error between two ENU coordinates
 * ported from GPS - now uses centimeters instead of degrees,
 * translate to roughly the same value to make PID routine
 * portable
 *
 * @param  int32_t* target_lat - target position latitude coord(mm from start point)
 * @param  int32_t* target_lng - target position longitude coord(mm)
 * @param  int32_t* flow_lat - actual position latitude coord(mm from start point)
 * @param  int32_t* flow_lng - actual position longitude coord(mm)
 */
static void flow_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* flow_lat, int32_t* flow_lng ) {
    error[LON] = (*target_lng - *flow_lng)/10;  // X Error
    error[LAT] = (*target_lat - *flow_lat)/10; // Y Error
    //debug[0] = sqrt(error[LON]*error[LON]+error[LAT]*error[LAT]);

}

/**
 * PID routine for position hold
 *
 * calculates the nav angle for x and y based on position error and velocity
 * @param  none
 */
static void flow_calc_poshold(void) {
    int32_t d;
    int32_t target_speed;
    uint8_t axis;

    for (axis=0;axis<2;axis++) {
        target_speed = get_P(error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
        target_speed = constrain(target_speed,-100,100);      // Constrain the target speed in poshold mode to 1m/s it helps avoid runaways..
        rate_error[axis] = target_speed - actual_speed[axis]; // calc the speed error
        rate_error[axis] += get_I(error[axis], &dTnav, &posholdPID[axis], &posholdPID_PARAM); //add position integral term

        //rate controller

        nav[axis]      =
                get_P(rate_error[axis],                                               &poshold_ratePID_PARAM)
                +get_I(rate_error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

        d = get_D(error[axis],                    &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

        d = constrain(d, -2000, 2000);
        // get rid of noise
        if(abs(actual_speed[axis]) < 50) d = 0;

        nav[axis] +=d;
        nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        navPID[axis].integrator = poshold_ratePID[axis].integrator;

        //debug[axis] = actual_speed[axis];
    }


}
////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void flow_calc_nav_rate(uint16_t max_speed) {
    float trig[2];
    uint8_t axis;
    // push us towards the original track
    flow_update_crosstrack();

    // nav_bearing includes crosstrack
    float temp = (9000l - nav_bearing) * RADX100;
    trig[LON] = cosf(temp);
    trig[LAT] = sinf(temp);

    for (axis=0;axis<2;axis++) {
        rate_error[axis] = (trig[axis] * max_speed) - actual_speed[axis];
        rate_error[axis] = constrain(rate_error[axis], -1000, 1000);
        // P + I + D
        nav[axis]      =
                get_P(rate_error[axis],                        &navPID_PARAM)
                +get_I(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM)
                +get_D(rate_error[axis], &dTnav, &navPID[axis], &navPID_PARAM);

        nav[axis]      = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
        poshold_ratePID[axis].integrator = navPID[axis].integrator;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line
// when flying to a waypoint.
//
static void flow_update_crosstrack(void) {
    if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {  // If we are too far off or too close we don't do track following
        float temp = (target_bearing - original_target_bearing) * RADX100;
        crosstrack_error = sinf(temp) * (wp_distance * CROSSTRACK_GAIN);  // Meters we are off track line
        nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
        nav_bearing = wrap_36000(nav_bearing);
    }else{
        nav_bearing = target_bearing;
    }
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow
// speed rampup when starting a navigation
//flow sensor can only sense up to 90cm/s
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                50   |   90      90
//                 |                                        +|+
//                 |< we should slow to 0.5 m/s as we hit the target
//
static uint16_t flow_calc_desired_speed(uint16_t max_speed, bool _slow) {
    // max_speed is default 400 or 4m/s
    if(_slow){
        max_speed = min(max_speed, wp_distance / 2);
        //max_speed = max(max_speed, 0);
    }else{
        max_speed = min(max_speed, wp_distance);
        max_speed = max(max_speed, NAV_SPEED_MIN);  // go at least 100cm/s
    }

    // limit the ramp up of the speed
    // waypoint_speed_gov is reset to 0 at each new WP command
    if(max_speed > waypoint_speed_gov){
        waypoint_speed_gov += (int)(100.0 * dTnav); // increase at .5/ms
        max_speed = waypoint_speed_gov;
    }
    return max_speed;
}


//It was mobed here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
    if (ang > 18000)  ang -= 36000;
    if (ang < -18000) ang += 36000;
    return ang;
}
////////////////////////////////////////////////////////////////////////////////////
// Utilities
//

int32_t wrap_36000(int32_t ang) {
    if (ang > 36000) ang -= 36000;
    if (ang < 0)     ang += 36000;
    return ang;
}

#endif
