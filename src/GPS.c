/*
 * GPS.c
 *
 *  Created on: 3/07/2013
 *      Author: Jono
 */
#include "GPS.h"
#include "config.h"
#include "def.h"
#include "i2c.h"
#include "sensors.h"
#include "microsec.h"
#include "gpio.h"
#include "LPC13Uxx.h"
#include "math.h"

#if GPS
static bool check_missed_wp();
static void GPS_calc_velocity();
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng );
static void GPS_calc_poshold();
static void GPS_calc_nav_rate(uint16_t max_speed);
static void GPS_update_crosstrack(void);
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow);


extern uint8_t I2CMasterBuffer[I2CBUFSIZE];
extern uint8_t I2CSlaveBuffer[I2CBUFSIZE];

extern uint8_t nav_mode;

extern int32_t  GPS_coord[2];
extern int32_t  GPS_home[2];
extern int32_t  GPS_hold[2];
extern uint8_t  GPS_numSat;
extern uint16_t GPS_distanceToHome;                          // distance to home in meters
extern int16_t  GPS_directionToHome;                         // direction to home in degrees
extern uint16_t GPS_altitude,GPS_speed;                      // altitude in 0.1m and speed in 0.1m/s
extern uint8_t  GPS_update;                              // it's a binary toogle to distinct a GPS position update
extern int16_t  GPS_angle[2];                      // it's the angles that must be applied for GPS correction
extern uint16_t GPS_ground_course;                       // degrees*10
extern uint8_t  GPS_Present;                             // Checksum from Gps serial
extern uint8_t  GPS_Enable;

extern struct conf_def conf;
extern struct flags_struct f;

extern int16_t heading, magHold;
extern uint16_t i2c_errors_count;

// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
extern int16_t  nav[2];
extern int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother
extern int16_t debug[4];

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


static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static uint16_t GPS_wp_radius    = GPS_WP_RADIUS;
static int16_t actual_speed[2] = {0,0};
static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
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
////////////////////////////////////////////////////////////////////////////////
// The location of the copter in relation to home, updated every GPS read (1deg - 100)
// static int32_t home_to_copter_bearing; /* unused */
// distance between plane and home in cm
// static int32_t home_distance; /* unused */
// distance between plane and next_WP in cm
static uint32_t wp_distance;

// used for slow speed wind up when start navigation;
static uint16_t waypoint_speed_gov;

////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
//

#define GPS_FILTER_VECTOR_LENGTH 5

static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
static uint16_t fraction3[2];





volatile uint8_t GPS_pids_initialized;
volatile uint8_t _i2c_gps_status;
volatile uint8_t dataSet=0;


// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t nav_takeoff_bearing;


#if defined(I2C_NAV)
/////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS helper functions
//
// Send a command to the I2C GPS module, first parameter command, second parameter wypoint number
void GPS_I2C_command(uint8_t command, uint8_t wp) {
	uint8_t _cmd;
	uint32_t status;

	_cmd = (wp << 4) + command;

	status = i2c_writeReg(I2C_GPS_ADDRESS,I2C_GPS_COMMAND,_cmd);
	if(status!=I2CSTATE_ACK)
		i2c_errors_count++;

	uint32_t timeMark = micros() + 15;
	while (micros() < timeMark);
}
#endif



void GPS_NewData() {

	//uint16_t timeStamp;

#if defined(I2C_NAV)
	if(millis()>3000)
	{
		GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;
		if( i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS_00) == I2CSTATE_ACK)                //Get status register
		{
			_i2c_gps_status = I2CSlaveBuffer[0];
		}
		else
		{
			i2c_errors_count++;
		}

		if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
			f.GPS_FIX = 1;

#if !defined(DONT_RESET_HOME_AT_ARM)
			if (!f.ARMED) {f.GPS_FIX_HOME = 0;}                                           //Clear home position if disarmed
#endif

			if (!f.GPS_FIX_HOME && f.ARMED) {        //if home is not set set home position to WP#0 and activate it
				GPS_reset_home_position();
			}
			if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                                //Check about new data
				if (GPS_update) { GPS_update = 0;} else { GPS_update = 1;}                    //Fancy flash on GUI :D
				if (!GPS_pids_initialized) {
					GPS_set_pids();
					GPS_pids_initialized = 1;
				}


				//Read GPS data for distance, heading and gps position alternating each cycle
				//always read nav data
				switch (dataSet%3){
				case 0:
					if(i2c_read(I2C_GPS_ADDRESS,I2C_GPS_NAV_BEARING,6)==I2CSTATE_ACK)
					{
						nav_bearing = I2CSlaveBuffer[1]<<8 | I2CSlaveBuffer[0];
						GPS_directionToHome = I2CSlaveBuffer[3]<<8 | I2CSlaveBuffer[2];
						GPS_distanceToHome = I2CSlaveBuffer[5]<<8 | I2CSlaveBuffer[4];

						//fix direction home
						GPS_directionToHome = GPS_directionToHome / 100;  // 1deg =1000 in the reg, downsize
						GPS_directionToHome += 180; // fix (see http://www.multiwii.com/forum/viewtopic.php?f=8&t=2892)
						if (GPS_directionToHome>180) GPS_directionToHome -= 360;
						//fix distance
						GPS_distanceToHome = GPS_distanceToHome / 100;      //register is in CM, we need in meter. max= 655 meters with this way
					}
					else
						i2c_errors_count++;

					if(i2c_read(I2C_GPS_ADDRESS,I2C_GPS_NAV_LAT,4)==I2CSTATE_ACK)
					{
						nav[LAT] = I2CSlaveBuffer[1]<<8 | I2CSlaveBuffer[0];
						nav[LON] = I2CSlaveBuffer[3]<<8 | I2CSlaveBuffer[2];
					}
					else
						i2c_errors_count++;
					break;
				case 1:
					//timeStamp = micros();
					if(i2c_read(I2C_GPS_ADDRESS,I2C_GPS_LOCATION,12)==I2CSTATE_ACK)
					{
						GPS_coord[LAT] = I2CSlaveBuffer[3]<<24 | I2CSlaveBuffer[2]<<16 | I2CSlaveBuffer[1]<<8 |I2CSlaveBuffer[0];
						GPS_coord[LON] = I2CSlaveBuffer[7]<<24 | I2CSlaveBuffer[6]<<16 | I2CSlaveBuffer[5]<<8 |I2CSlaveBuffer[4];
						nav[LAT] = I2CSlaveBuffer[9]<<8 | I2CSlaveBuffer[8];
						nav[LON] = I2CSlaveBuffer[11]<<8 | I2CSlaveBuffer[10];
					}
					else
						i2c_errors_count++;
					//debug[0] = micros()-timeStamp;

					break;
				case 2:
					if(i2c_read(I2C_GPS_ADDRESS,I2C_GPS_GROUND_SPEED,6)==I2CSTATE_ACK)
					{
						GPS_speed = I2CSlaveBuffer[1]<<8 | I2CSlaveBuffer[0];
						GPS_altitude = I2CSlaveBuffer[3]<<8 | I2CSlaveBuffer[2];
						GPS_ground_course = I2CSlaveBuffer[5]<<8 | I2CSlaveBuffer[4];
					}
					else
						i2c_errors_count++;

					if(i2c_read(I2C_GPS_ADDRESS,I2C_GPS_NAV_LAT,4)==I2CSTATE_ACK)
					{
						nav[LAT] = I2CSlaveBuffer[1]<<8 | I2CSlaveBuffer[0];
						nav[LON] = I2CSlaveBuffer[3]<<8 | I2CSlaveBuffer[2];
					}
					else
						i2c_errors_count++;
					break;
				}
				dataSet++;

				if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
					GPS_distanceToHome = 0;
					GPS_directionToHome = 0;
				}

				//Adjust heading when navigating
				if (f.GPS_HOME_MODE) {
					if ( !(_i2c_gps_status & I2C_GPS_STATUS_WP_REACHED) ) {
						//Tail control
						if (NAV_CONTROLS_HEADING) {
							if (NAV_TAIL_FIRST) {
								magHold = nav_bearing/100-180;
								if (magHold > 180)  magHold -= 360;
								if (magHold < -180) magHold += 360;
							} else {
								magHold = nav_bearing/100;
							}
						}
					} else {        //Home position reached
						if (NAV_SET_TAKEOFF_HEADING) { magHold = nav_takeoff_bearing; }
					}
				}
			}
		} else {                                                                          //We don't have a fix zero out distance and bearing (for safety reasons)
			GPS_distanceToHome = 0;
			GPS_directionToHome = 0;
			GPS_numSat = 0;
			f.GPS_FIX = 0;
		}

	}
#endif
#if defined(GPS_SERIAL) || defined(TINY_GPS) || defined(GPS_FROM_OSD)|| defined (I2C_GPS)
	uint8_t axis = 0;
	static bool newData = 0;
	//timeStamp = micros();

#if defined(GPS_SERIAL)
	uint8_t c = SerialAvailable(GPS_SERIAL);
	while (c--) {
		//while (SerialAvailable(GPS_SERIAL)) {
		if (GPS_newFrame(SerialRead(GPS_SERIAL))) {
#elif defined(TINY_GPS)
			{
				{
					tinygps_query();
#elif defined(GPS_FROM_OSD)
					{
						if(GPS_update & 2) {  // Once second bit of GPS_update is set, indicate new GPS datas is readed from OSD - all in right format.
							GPS_update &= 1;    // We have: GPS_fix(0-2), GPS_numSat(0-15), GPS_coord[LAT & LON](signed, in 1/10 000 000 degres), GPS_altitude(signed, in meters) and GPS_speed(in cm/s)
#elif defined(I2C_GPS)

							_i2c_gps_status = 0;
							if( i2c_readReg(I2C_GPS_ADDRESS,I2C_GPS_STATUS_00) == I2CSTATE_ACK)                //Get status register
							{
								_i2c_gps_status = I2CSlaveBuffer[0];
							}
							else
							{
								i2c_errors_count++;
							}
							GPS_numSat = (_i2c_gps_status & 0xf0) >> 4;


							if (_i2c_gps_status & I2C_GPS_STATUS_3DFIX) {                                     //Check is we have a good 3d fix (numsats>5)
								f.GPS_FIX = 1;


								if (_i2c_gps_status & I2C_GPS_STATUS_NEW_DATA) {                                //Check about new data



									newData = 1;

									if (!GPS_pids_initialized) {
										GPS_set_pids();
										GPS_pids_initialized = 1;
									}

									if(i2c_read(I2C_GPS_ADDRESS,I2C_GPS_LOCATION,8)==I2CSTATE_ACK)
									{
										GPS_coord[LAT] = I2CSlaveBuffer[3]<<24 | I2CSlaveBuffer[2]<<16 | I2CSlaveBuffer[1]<<8 |I2CSlaveBuffer[0];
										GPS_coord[LON] = I2CSlaveBuffer[7]<<24 | I2CSlaveBuffer[6]<<16 | I2CSlaveBuffer[5]<<8 |I2CSlaveBuffer[4];
									}
									else
										i2c_errors_count++;


#endif

									if (GPS_update == 1) GPS_update = 0; else GPS_update = 1;
									if (f.GPS_FIX && GPS_numSat >= 5) {

#if !defined(DONT_RESET_HOME_AT_ARM)
										if (!f.ARMED) {f.GPS_FIX_HOME = 0;}
#endif
										if (!f.GPS_FIX_HOME && f.ARMED) {
											GPS_reset_home_position();
										}

										//Apply moving average filter to GPS data
#if defined(GPS_FILTERING)
										GPS_filter_index = (GPS_filter_index+1) % GPS_FILTER_VECTOR_LENGTH;
										for (axis = 0; axis< 2; axis++) {
											GPS_read[axis] = GPS_coord[axis]; //latest unfiltered data is in GPS_latitude and GPS_longitude
											GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t

											// How close we are to a degree line ? its the first three digits from the fractions of degree
											// later we use it to Check if we are close to a degree line, if yes, disable averaging,
											fraction3[axis] = (GPS_read[axis]- GPS_degree[axis]*10000000) / 10000;

											GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
											GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis]*10000000);
											GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
											GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
											if ( nav_mode == NAV_MODE_POSHOLD) {      //we use gps averaging only in poshold mode...
												if ( fraction3[axis]>1 && fraction3[axis]<999 ) GPS_coord[axis] = GPS_filtered[axis];
											}
										}
#endif


										//dTnav calculation
										//Time for calculating x,y speed and navigation pids
										static uint32_t nav_loopTimer;
										dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
										nav_loopTimer = millis();
										// prevent runup from bad GPS
										dTnav = min(dTnav, 1.0);
										//debug[3] = dTnav*1000;

										//calculate distance and bearings for gui and other stuff continously - From home to copter
										uint32_t dist;
										int32_t  dir;
										GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
										GPS_distanceToHome = dist/100;
										GPS_directionToHome = dir/100;

										if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
											GPS_distanceToHome = 0;
											GPS_directionToHome = 0;
										}

										//calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
										GPS_calc_velocity();

										if (f.GPS_HOLD_MODE || f.GPS_HOME_MODE){    //ok we are navigating
											//do gps nav calculations here, these are common for nav and poshold
#if defined(GPS_LEAD_FILTER)
											GPS_distance_cm_bearing(&GPS_coord_lead[LAT],&GPS_coord_lead[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
											GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord_lead[LAT],&GPS_coord_lead[LON]);
#else
											GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
											GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
#endif

											int16_t speed = 0;

											switch (nav_mode) {
											case NAV_MODE_POSHOLD:
												//Desired output is in nav_lat and nav_lon where 1deg inclination is 100
												GPS_calc_poshold();
												break;
											case NAV_MODE_WP:

												speed = GPS_calc_desired_speed(NAV_SPEED_MAX, NAV_SLOW_NAV);      //slow navigation
												// use error as the desired rate towards the target
												//Desired output is in nav_lat and nav_lon where 1deg inclination is 100
												GPS_calc_nav_rate(speed);

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
												}
												break;
											}
										} //end of gps calcs
									}//good fix
								}
								else if(newData == 1)//split up reads to reduce time spent on main gps loop
								{
									if(i2c_read(I2C_GPS_ADDRESS,I2C_GPS_GROUND_SPEED,6)==I2CSTATE_ACK)
									{
										GPS_speed = I2CSlaveBuffer[1]<<8 | I2CSlaveBuffer[0];
										GPS_altitude = I2CSlaveBuffer[3]<<8 | I2CSlaveBuffer[2];
										GPS_ground_course = I2CSlaveBuffer[5]<<8 | I2CSlaveBuffer[4];
										newData = 0;
									}
									else
										i2c_errors_count++;

								}
							}
							//debug[0] = micros()-timeStamp;
#endif

}


void GPS_reset_home_position() {
	if (f.GPS_FIX && GPS_numSat >= 5) {
#if defined(I2C_NAV)
		//set current position as home
		GPS_I2C_command(I2C_GPS_COMMAND_SET_WP,0);  //WP0 is the home position
#else
		GPS_home[LAT] = GPS_coord[LAT];
		GPS_home[LON] = GPS_coord[LON];
		GPS_calc_longitude_scaling(GPS_coord[LAT]);  //need an initial value for distance and bearing calc
#endif
		nav_takeoff_bearing = heading;             //save takeoff heading
		//Set ground altitude
		f.GPS_FIX_HOME = 1;
	}
}

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav() {
	uint8_t i;

	for(i=0;i<2;i++) {
		nav_rated[i] = 0;
		nav[i] = 0;
#if !defined(I2C_NAV)
		reset_PID(&posholdPID[i]);
		reset_PID(&poshold_ratePID[i]);
		reset_PID(&navPID[i]);
		nav_mode = NAV_MODE_NONE;
#endif
	}
#ifdef I2C_NAV
	GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
#endif

}

//Get the relevant P I D values and set the PID controllers
void GPS_set_pids() {
#if defined(GPS_SERIAL)  || defined(GPS_FROM_OSD) || defined(TINY_GPS) || defined (I2C_GPS)
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
#endif

#if defined(I2C_GPS)
	//clear all nav flags
	if( i2c_writeReg(I2C_GPS_ADDRESS,I2C_GPS_NAV_FLAGS,0) != I2CSTATE_ACK)
		initError(); //fatal error

	delayMs(1);
#endif

	//delays required as atmega8 is slow on storing data consider burst writing with delays in between
#if defined(I2C_NAV)
	uint32_t i2cStatus = 0;
	uint8_t writeBuffer[3];

	//write gps hold p and i gains
	writeBuffer[0] = conf.P8[PIDPOS];
	writeBuffer[1] = conf.I8[PIDPOS];

	i2cStatus = i2c_burstWrite(I2C_GPS_ADDRESS, I2C_GPS_HOLD_P, writeBuffer, 2);
	delayMs(1);

	//write gps hold rate pid
	writeBuffer[0] = conf.P8[PIDPOSR];
	writeBuffer[1] = conf.I8[PIDPOSR];
	writeBuffer[2] = conf.D8[PIDPOSR];

	i2cStatus |= i2c_burstWrite(I2C_GPS_ADDRESS, I2C_GPS_HOLD_RATE_P, writeBuffer, 3);
	delayMs(1);

	//write gps nav pid
	writeBuffer[0] = conf.P8[PIDNAVR];
	writeBuffer[1] = conf.I8[PIDNAVR];
	writeBuffer[2] = conf.D8[PIDNAVR];

	i2cStatus |= i2c_burstWrite(I2C_GPS_ADDRESS,I2C_GPS_NAV_P, writeBuffer, 3);
	delayMs(1);

	//save pids
	GPS_I2C_command(I2C_GPS_COMMAND_UPDATE_PIDS,0);

	delayMs(1);

	uint8_t nav_flags = 0;
#if defined(GPS_FILTERING)
	nav_flags += I2C_NAV_FLAG_GPS_FILTER;
#endif
#if defined(GPS_LOW_SPEED_D_FILTER)
	nav_flags += I2C_NAV_FLAG_LOW_SPEED_D_FILTER;
#endif

	//write nav flags
	i2cStatus |= i2c_writeReg(I2C_GPS_ADDRESS,I2C_GPS_NAV_FLAGS,nav_flags);
	delayMs(1);

	//write waypoint radius (16bits little endian)
	writeBuffer[0] = GPS_WP_RADIUS & 0x00FF;// lower eight bit
	writeBuffer[1] = GPS_WP_RADIUS >>8;// upper eight bit

	i2cStatus |= i2c_burstWrite(I2C_GPS_ADDRESS,I2C_GPS_WP_RADIUS, writeBuffer, 2);

	delayMs(1);
	if(i2cStatus != I2CSTATE_ACK){
		initError(); //fatal error
	}
#endif
}

#if defined (TINY_GPS)
int32_t GPS_coord_to_decimal(struct coord *c) {
#define GPS_SCALE_FACTOR 10000000L
	uint32_t deg = 0;
	uint8_t i;
	deg = (uint32_t)c->deg * GPS_SCALE_FACTOR;

	uint32_t min = 0;
	/* add up the BCD fractions */
	uint16_t divisor = 1000;
	for (i=0; i<NMEA_MINUTE_FRACTS; i++) {
		uint8_t b = c->frac[i/2];
		uint8_t n = (i%2 ? b>>4 : b&0x0F);
		min += n*(divisor);
		divisor /= 10;
	}
	min *= 1000; // <-- NEW
	min += (uint32_t)c->min * GPS_SCALE_FACTOR;
	/* now sum up degrees and minutes */
	return deg + min/60;
}
#endif

//OK here is the onboard GPS code
#if defined(GPS_SERIAL) || defined(GPS_FROM_OSD) || defined(TINY_GPS) || defined(I2C_GPS)

////////////////////////////////////////////////////////////////////////////////////
//PID based GPS navigation functions
//Author : EOSBandi
//Based on code and ideas from the Arducopter team: Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
//Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat) {
	float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
	GPS_scaleLonDown = cosf(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
//
void GPS_set_next_wp(int32_t* lat, int32_t* lon) {
	GPS_WP[LAT] = *lat;
	GPS_WP[LON] = *lon;

	GPS_calc_longitude_scaling(*lat);
	GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);

	nav_bearing = target_bearing;
	GPS_calc_location_error(&GPS_WP[LAT],&GPS_WP[LON],&GPS_coord[LAT],&GPS_coord[LON]);
	original_target_bearing = target_bearing;
	waypoint_speed_gov = NAV_SPEED_MIN;
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp() {
	int32_t temp;
	temp = target_bearing - original_target_bearing;
	temp = wrap_18000(temp);
	return (abs(temp) > 10000);   // we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
	float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
	float dLon = (float)(*lon2 - *lon1) * GPS_scaleLonDown;
	*dist = sqrt(dLat*dLat + dLon*dLon) * 1.113195;

	*bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
	if (*bearing < 0) *bearing += 36000;
}

#if defined(OBSOLATED)
////////////////////////////////////////////////////////////////////////////////////
// keep old calculation function for compatibility (could be removed later) distance in meters, bearing in degree
//
void GPS_distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, uint16_t* dist, int16_t* bearing) {
	uint32_t d1;
	int32_t  d2;
	GPS_distance_cm_bearing(&lat1,&lon1,&lat2,&lon2,&d1,&d2);
	*dist = d1 / 100;          //convert to meters
	*bearing = d2 /  100;      //convert to degrees
}
#endif


//*******************************************************************************************************
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
//*******************************************************************************************************
static void GPS_calc_velocity(){
	static int16_t speed_old[2] = {0,0};
	static int32_t last[2] = {0,0};
	static uint8_t init = 0;


	if (init) {
		float tmp = 1.0/dTnav;
		actual_speed[_X] = (float)(GPS_coord[LON] - last[LON]) *  GPS_scaleLonDown * tmp;
		actual_speed[_Y] = (float)(GPS_coord[LAT]  - last[LAT])  * tmp;

		//debug[2] = tmp;

#if !defined(GPS_LEAD_FILTER)

		actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
		actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;

		speed_old[_X] = actual_speed[_X];
		speed_old[_Y] = actual_speed[_Y];


#endif
	}
	init=1;

	last[LON] = GPS_coord[LON];
	last[LAT] = GPS_coord[LAT];



#if defined(GPS_LEAD_FILTER)
	GPS_coord_lead[LON] = xLeadFilter.get_position(GPS_coord[LON], actual_speed[_X], GPS_LAG);
	GPS_coord_lead[LAT] = yLeadFilter.get_position(GPS_coord[LAT], actual_speed[_Y], GPS_LAG);
#endif

}







////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) {
	error[LON] = (float)(*target_lng - *gps_lng) * GPS_scaleLonDown;  // X Error
	error[LAT] = *target_lat - *gps_lat; // Y Error
	//debug[0] = sqrt(error[LON]*error[LON]+error[LAT]*error[LAT]);

}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold() {
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
static void GPS_calc_nav_rate(uint16_t max_speed) {
	float trig[2];
	uint8_t axis;
	// push us towards the original track
	GPS_update_crosstrack();

	// nav_bearing includes crosstrack
	float temp = (9000l - nav_bearing) * RADX100;
	trig[_X] = cosf(temp);
	trig[_Y] = sinf(temp);

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
static void GPS_update_crosstrack(void) {
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
//
//      |< WP Radius
//      0  1   2   3   4   5   6   7   8m
//      ...|...|...|...|...|...|...|...|
//                100  |  200     300     400cm/s
//                 |                                        +|+
//                 |< we should slow to 1.5 m/s as we hit the target
//
static uint16_t GPS_calc_desired_speed(uint16_t max_speed, bool _slow) {
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
#endif

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




#endif // GPS

