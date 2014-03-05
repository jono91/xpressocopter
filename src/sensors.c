/*
 * sensors.c

 *
 *  Created on: 2/02/2013
 *      Author: Jono
 */
#include "sensors.h"
#include "LPC13Uxx.h"			/* LPC13xx Peripheral Registers */
#include "type.h"
#include "i2c.h"
#include "gpio.h"
#include "stdint.h"
#include "microsec.h"
//#include "math.h"
#include "EEPROM.h"
#include "gpio.h"
#include "adc.h"
#include "common.h"


extern struct conf_def conf;
extern struct flags_struct f;

extern volatile uint32_t ADCValue[ADC_NUM];
extern volatile uint32_t ADCIntDone;
extern volatile uint32_t OverRunCounter;

extern int16_t  sonarAlt;
extern int16_t  acc_25deg;
extern uint32_t currentTime;
extern uint32_t I2CCount;
extern uint8_t I2CMasterBuffer[I2CBUFSIZE];
extern uint8_t I2CSlaveBuffer[I2CBUFSIZE];
extern uint32_t I2CMasterState;
extern uint32_t I2CReadLength, I2CWriteLength;
extern int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
extern uint16_t acc_1G;
extern uint16_t i2c_errors_count;
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern int16_t gyroZero[3];
extern int32_t BaroAlt;
extern int16_t debug[4];

#if BARO
extern int32_t baroPressure;
extern int32_t baroTemperature;
extern int32_t baroPressureSum;
#endif

#ifdef MAYHONY
extern int16_t gyroAHRS[3];
#endif

float magCal[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
uint8_t magInit = 0;

uint32_t i2c_writeReg(uint8_t add,uint8_t reg, uint8_t val)
{
	/* In order to start the I2CEngine, the all the parameters
  must be set in advance, including I2CWriteLength, I2CReadLength,
  I2CCmd, and the I2cMasterBuffer which contains the stream
  command/data to the I2c slave device.
  (1) If it's a I2C write only, the number of bytes to be written is
  I2CWriteLength, I2CReadLength is zero, the content will be filled
  in the I2CMasterBuffer.
  (2) If it's a I2C read only, the number of bytes to be read is
  I2CReadLength, I2CWriteLength is 0, the read value will be filled
  in the I2CMasterBuffer.
  (3) If it's a I2C Write/Read with repeated start, specify the
  I2CWriteLength, fill the content of bytes to be written in
  I2CMasterBuffer, specify the I2CReadLength, after the repeated
  start and the device address with RD bit set, the content of the
  reading will be filled in I2CSlaveBuffer index at
  I2CSlaveBuffer[0].

  e.g. Start, DevAddr(W), WRByte1...WRByteN, Repeated-Start, DevAddr(R),
  RDByte1...RDByteN Stop. The content of the reading will be filled
  after (I2CWriteLength + two devaddr) bytes. */

	/* Write SLA(W), address and one data byte */
	I2CWriteLength = 3;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = add<<1;
	I2CMasterBuffer[1] = reg;		/* address */
	I2CMasterBuffer[2] = val;
	return I2CEngine();
}

uint32_t i2c_burstWrite(uint8_t add,uint8_t reg, uint8_t* val, uint8_t bytes)
{
	uint8_t i;
	/* Write SLA(W), address and number of data bytes specified by variable byte */
	I2CWriteLength = bytes+2;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = add<<1;
	I2CMasterBuffer[1] = reg;		/* address */

	for (i=2;i<(bytes+2);i++)
	{
		I2CMasterBuffer[i] = val[i-2];
	}
	return I2CEngine();
}

uint32_t i2c_writeCommand(uint8_t add, uint8_t val)//for writing a single command to a slave device
{
	/* Write SLA(W) and one data byte */
	I2CWriteLength = 2;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = add<<1;
	I2CMasterBuffer[1] = val;
	return I2CEngine();
}

//result is stored in I2CSlaveBuffer[0]
//move straight after this call
uint32_t i2c_readReg( uint8_t add, uint8_t reg)
{


	I2CSlaveBuffer[0] = 0x00;

	/* Write SLA(W), address, SLA(R), and read one byte back. */
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = add<<1;
	I2CMasterBuffer[1] = reg;		/* address */
	I2CMasterBuffer[2] = (add<<1) | RD_BIT;
	return I2CEngine();
}

//result is stored in I2CSlaveBuffer[0-5]
//move straight after this call
uint32_t i2c_getSixRawADC(uint8_t add, uint8_t reg)
{
	int i=0;
	for ( i = 0; i < 6; i++ )
	{
		I2CSlaveBuffer[i] = 0x00;
	}
	/* Write SLA(W), address, SLA(R), and read one byte back. */
	I2CWriteLength = 2;
	I2CReadLength = 6;
	I2CMasterBuffer[0] = add<<1;
	I2CMasterBuffer[1] = reg;		/* address */
	I2CMasterBuffer[2] = (add<<1) | RD_BIT;
	return I2CEngine();
}

//result is stored in I2CSlaveBuffer[0-(bytes -1)]
//move straight after this call
uint32_t i2c_read(uint8_t add, uint8_t reg, uint8_t bytes)
{
	int i=0;
	for ( i = 0; i < bytes; i++ )
	{
		I2CSlaveBuffer[i] = 0x00;
	}
	/* Write SLA(W), address, SLA(R), and read one byte back. */
	I2CWriteLength = 2;
	I2CReadLength = bytes;
	I2CMasterBuffer[0] = add<<1;
	I2CMasterBuffer[1] = reg;		/* address */
	I2CMasterBuffer[2] = (add<<1) | RD_BIT;
	return I2CEngine();
}



// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#if defined(MPU6050)

void Gyro_init() {

	uint32_t state;
	state = i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
	delayMs(5);
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
	// enable I2C bypass for AUX I2C
#if defined(MAG)
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
#endif

	if (state != I2CSTATE_ACK)
		initError();  //fatal error on setup
}

void Gyro_getADC () {
	if(i2c_getSixRawADC(MPU6050_ADDRESS, 0x43) == I2CSTATE_ACK)
	{
		GYRO_ORIENTATION( (int16_t)((I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1])/4 , // range: +/- 8192; +/- 2000 deg/sec
				(int16_t)((I2CSlaveBuffer[2]<<8) | I2CSlaveBuffer[3])/4 ,
				(int16_t)((I2CSlaveBuffer[4]<<8) | I2CSlaveBuffer[5])/4 );
		GYRO_Common();
	}
	else
	{
		i2c_errors_count++;
	}
}

void ACC_init () {
	uint32_t state;
	state = i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
	//note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
	//confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
#if defined(FREEIMUv04)
	acc_1G = 255;
#else
	acc_1G = 512;
#endif

#if defined(MPU6050_I2C_AUX_MASTER)
	//at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
	//now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
	state |= i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=6 (3x2 bytes)
#endif
	if (state != I2CSTATE_ACK)
		initError();  //fatal setup error
}

void ACC_getADC () {
	if(i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B) == I2CSTATE_ACK)
	{
		ACC_ORIENTATION( (int16_t)((I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1])/8 ,
				(int16_t)((I2CSlaveBuffer[2]<<8) | I2CSlaveBuffer[3])/8 ,
				(int16_t)((I2CSlaveBuffer[4]<<8) | I2CSlaveBuffer[5])/8 );
		ACC_Common();
	}
	else
	{
		i2c_errors_count++;
	}

}

//The MAG acquisition function must be replaced because we now talk to the MPU device
#if defined(MPU6050_I2C_AUX_MASTER)
void Device_Mag_getADC() {
	if(i2c_getSixRawADC(MPU6050_ADDRESS, 0x49) == I2CSTATE_ACK)	//0x49 is the first memory room for EXT_SENS_DATA
	{
#if defined(HMC5843)
		MAG_ORIENTATION( (int16_t)((I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1]) ,
				(int16_t)((I2CSlaveBuffer[2]<<8) | I2CSlaveBuffer[3]) ,
				(int16_t)((I2CSlaveBuffer[4]<<8) | I2CSlaveBuffer[5]) );
#endif
#if defined (HMC5883)
		MAG_ORIENTATION( (int16_t)((I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1]) ,
				(int16_t)((I2CSlaveBuffer[4]<<8) | I2CSlaveBuffer[5]) ,
				(int16_t)((I2CSlaveBuffer[2]<<8) | I2CSlaveBuffer[3]) );
#endif
#if defined (MAG3110)
		MAG_ORIENTATION( (int16_t)((I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1]) ,
				(int16_t)((I2CSlaveBuffer[2]<<8) | I2CSlaveBuffer[3]) ,
				(int16_t)((I2CSlaveBuffer[4]<<8) | I2CSlaveBuffer[5]) );
#endif
	}
	else
	{
		i2c_errors_count++;
	}
}
#endif
#endif

// ************************************************************************************************************
// I2C Magnetometer HMC5843/HMC5883L
// ************************************************************************************************************

#if defined(HMC5843) || defined(HMC5883)
void Mag_init() {
	uint32_t state;
	delayMs(100);
	// force positiveBias
	state = i2c_writeReg(MAG_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
	delayMs(50);
	// set gains for calibration
	state |= i2c_writeReg(MAG_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
	state |= i2c_writeReg(MAG_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode

	// read values from the compass -  self test operation
	// by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
	// The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
	delayMs(100);
	state |= getCompADC();
	delayMs(10);
#if defined(HMC5883)
	magCal[ROLL]  =  1160.0 / abs(magADC[ROLL]);
	magCal[PITCH] =  1160.0 / abs(magADC[PITCH]);
	magCal[YAW]   =  1080.0 / abs(magADC[YAW]);
#else
	magCal[ROLL]  =  1000.0 / abs(magADC[ROLL]);
	magCal[PITCH] =  1000.0 / abs(magADC[PITCH]);
	magCal[YAW]   =  1000.0 / abs(magADC[YAW]);
#endif

	// leave test mode
	state |= i2c_writeReg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	state |= i2c_writeReg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	state |= i2c_writeReg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

	magInit = 1;

	if(state != I2CSTATE_ACK)
		initError(); //fatal init error
}

uint32_t getCompADC() {
	uint32_t state;
	state = i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
	if( state == I2CSTATE_ACK)
	{
#if defined(HMC5843)
		MAG_ORIENTATION( (int16_t)((I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1]) ,
				(int16_t)((I2CSlaveBuffer[2]<<8) | I2CSlaveBuffer[3]) ,
				(int16_t)((I2CSlaveBuffer[4]<<8) | I2CSlaveBuffer[5]) );
#endif
#if defined (HMC5883)
		MAG_ORIENTATION( (int16_t)((I2CSlaveBuffer[0]<<8) | I2CSlaveBuffer[1]) ,
				(int16_t)((I2CSlaveBuffer[4]<<8) | I2CSlaveBuffer[5]) ,
				(int16_t)((I2CSlaveBuffer[2]<<8) | I2CSlaveBuffer[3]) );
#endif
	}
	else
		i2c_errors_count++;

	return state;

}

#ifndef MPU6050_I2C_AUX_MASTER
void Device_Mag_getADC() {
	getCompADC();
}
#endif
#endif

// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************


#if defined(BMP085)
#define BMP085_ADDRESS 0x77

typedef struct {
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
} bmp085_smd500_calibration_param_t;

typedef struct {
	bmp085_smd500_calibration_param_t cal_param;
	int16_t oversampling_setting;
	uint8_t state;
	uint32_t deadline;
} bmp085_t;

static bmp085_t bmp085 = { { 0, } };
static uint16_t bmp085_ut; // static result of temperature measurement
static uint32_t bmp085_up; // static result of pressure measurement



void i2c_BMP085_readCalibration(){
	delayMs(10);
	//read calibration data in one go
	if(i2c_read(BMP085_ADDRESS, 0xAA, 22) == I2CSTATE_ACK)
	{
		//move data while swapping endianness
		/*parameters AC1-AC6*/
		bmp085.cal_param.ac1 = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1];
		bmp085.cal_param.ac2 = (I2CSlaveBuffer[2] << 8) | I2CSlaveBuffer[3];
		bmp085.cal_param.ac3 = (I2CSlaveBuffer[4] << 8) | I2CSlaveBuffer[5];
		bmp085.cal_param.ac4 = (I2CSlaveBuffer[6] << 8) | I2CSlaveBuffer[7];
		bmp085.cal_param.ac5 = (I2CSlaveBuffer[8] << 8) | I2CSlaveBuffer[9];
		bmp085.cal_param.ac6 = (I2CSlaveBuffer[10] << 8) | I2CSlaveBuffer[11];

		/*parameters B1,B2*/
		bmp085.cal_param.b1 = (I2CSlaveBuffer[12] << 8) | I2CSlaveBuffer[13];
		bmp085.cal_param.b2 = (I2CSlaveBuffer[14] << 8) | I2CSlaveBuffer[15];

		/*parameters MB,MC,MD*/
		bmp085.cal_param.mb = (I2CSlaveBuffer[16] << 8) | I2CSlaveBuffer[17];
		bmp085.cal_param.mc = (I2CSlaveBuffer[18] << 8) | I2CSlaveBuffer[19];
		bmp085.cal_param.md = (I2CSlaveBuffer[20] << 8) | I2CSlaveBuffer[21];
	}
	else
		initError();
}

void  Baro_init() {
	delayMs(10);
	bmp085.oversampling_setting = 3;
	i2c_BMP085_readCalibration();
	i2c_BMP085_UT_Start();
	bmp085.deadline = currentTime+5000;
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start() {
	if(i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e) != I2CSTATE_ACK)
		i2c_errors_count++;
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
	if(i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(bmp085.oversampling_setting<<6)) != I2CSTATE_ACK) // control register value for oversampling setting 3
		i2c_errors_count++;
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
	if(i2c_read(BMP085_ADDRESS, 0xF6, 3) == I2CSTATE_ACK)
		bmp085_up = (((uint32_t)I2CSlaveBuffer[0] << 16) | ((uint32_t)I2CSlaveBuffer[1] << 8) | (uint32_t)I2CSlaveBuffer[2] ) >>(8 - bmp085.oversampling_setting);
	else
		i2c_errors_count++;
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
	if(i2c_read(BMP085_ADDRESS, 0xF6, 2) == I2CSTATE_ACK)
		bmp085_ut = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1];
	else
		i2c_errors_count++;
}

void i2c_BMP085_Calculate() {
	int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
	uint32_t b4, b7;
	// Temperature calculations
	x1 = ((int32_t)bmp085_ut - bmp085.cal_param.ac6) * bmp085.cal_param.ac5 >> 15;
	x2 = ((int32_t)bmp085.cal_param.mc << 11) / (x1 + bmp085.cal_param.md);
	b5 = x1 + x2;
	baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
	// Pressure calculations
	b6 = b5 - 4000;
	x1 = (bmp085.cal_param.b2 * (b6 * b6 >> 12)) >> 11;
	x2 = bmp085.cal_param.ac2 * b6 >> 11;
	x3 = x1 + x2;
	tmp = bmp085.cal_param.ac1;
	tmp = (tmp*4 + x3) << bmp085.oversampling_setting;
	b3 = (tmp+2)/4;
	x1 = bmp085.cal_param.ac3 * b6 >> 13;
	x2 = (bmp085.cal_param.b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (bmp085.cal_param.ac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = ((uint32_t) (bmp085_up - b3) * (50000 >> bmp085.oversampling_setting));
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	baroPressure = p + ((x1 + x2 + 3791) >> 4);
}


//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                   // first UT conversion is started in init procedure
	if (currentTime < bmp085.deadline) return 0;

	bmp085.deadline = currentTime+6000; // 1.5ms margin according to the spec (4.5ms T convetion time)


	if (bmp085.state == 0)
	{
		i2c_BMP085_UT_Read();
		i2c_BMP085_UP_Start();
		bmp085.state = 1;
		Baro_Common();
		bmp085.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
		return 1;
	}
	else
	{
		i2c_BMP085_UP_Read();
		i2c_BMP085_UT_Start();
		i2c_BMP085_Calculate();
		bmp085.state = 0;
		return 2;
	}
}
#endif

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

typedef struct {
	// sensor registers from the MS561101BA datasheet
	uint16_t c[7];
	uint32_t ut; //uncompensated T
	uint32_t up; //uncompensated P
	uint8_t  state;
	uint32_t deadline;
} ms561101ba_t;

static ms561101ba_t ms561101ba_ctx;

void i2c_MS561101BA_reset(){
	if(i2c_writeCommand(MS561101BA_ADDRESS, MS561101BA_RESET) != I2CSTATE_ACK)
		i2c_errors_count++;
}

void i2c_MS561101BA_readCalibration(){
	uint8_t i;
	uint32_t state = I2CSTATE_ACK;
	for(i=0;i<6;i++) {
		state |= i2c_read(MS561101BA_ADDRESS,0xA2+2*i,2);
		ms561101ba_ctx.c[i+1] = (I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1];
	}
	if(state != I2CSTATE_ACK)
		initError(); //fatal error
}

void  Baro_init() {
	delayMs(10);
	i2c_MS561101BA_reset();
	delayMs(100);
	i2c_MS561101BA_readCalibration();
	delayMs(10);
	i2c_MS561101BA_UT_Start();
	ms561101ba_ctx.deadline = currentTime+10000;
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start() {
	if(i2c_writeCommand(MS561101BA_ADDRESS, MS561101BA_TEMPERATURE + OSR) != I2CSTATE_ACK)
		i2c_errors_count++;
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start () {
	if(i2c_writeCommand(MS561101BA_ADDRESS, MS561101BA_PRESSURE + OSR) != I2CSTATE_ACK)
		i2c_errors_count++;
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read () {
	if(i2c_read(MS561101BA_ADDRESS,0x00,3) == I2CSTATE_ACK)
		ms561101ba_ctx.up = (I2CSlaveBuffer[0] << 16) | (I2CSlaveBuffer[1]<<8) | I2CSlaveBuffer[2];
	else
		i2c_errors_count++;
}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read() {
	if(i2c_read(MS561101BA_ADDRESS,0x00,3) ==I2CSTATE_ACK)
		ms561101ba_ctx.ut = (I2CSlaveBuffer[0] << 16) | (I2CSlaveBuffer[1]<<8) | I2CSlaveBuffer[2];
	else
		i2c_errors_count++;
}

void i2c_MS561101BA_Calculate() {
	int32_t off2,sens2,delt;

	int64_t dT       = (int32_t)ms561101ba_ctx.ut - ((int32_t)ms561101ba_ctx.c[5] << 8);
	baroTemperature  = 2000 + ((dT * ms561101ba_ctx.c[6])>>23);
	int64_t off      = ((uint32_t)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
	int64_t sens     = ((uint32_t)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) >> 8);

	if (baroTemperature < 2000) { // temperature lower than 20st.C
		delt = baroTemperature-2000;
		delt  = 5*delt*delt;
		off2  = delt>>1;
		sens2 = delt>>2;
		if (baroTemperature < -1500) { // temperature lower than -15st.C
			delt  = baroTemperature+1500;
			delt  = delt*delt;
			off2  += 7 * delt;
			sens2 += (11 * delt)>>1;
		}
		off  -= off2;
		sens -= sens2;
	}

	baroPressure     = (( (ms561101ba_ctx.up * sens ) >> 21) - off) >> 15;
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                            // first UT conversion is started in init procedure
	if (currentTime < ms561101ba_ctx.deadline) return 0;
	ms561101ba_ctx.deadline = currentTime+10000;  // UT and UP conversion take 8.5ms so we do next reading after 10ms

	if (ms561101ba_ctx.state == 0) {
		i2c_MS561101BA_UT_Read();
		i2c_MS561101BA_UP_Start();
		Baro_Common();                              // moved here for less timecycle spike
		ms561101ba_ctx.state = 1;
		return 1;
	} else {
		i2c_MS561101BA_UP_Read();
		i2c_MS561101BA_UT_Start();
		i2c_MS561101BA_Calculate();
		ms561101ba_ctx.state = 0;
		return 2;
	}
}
#endif

// ************************
// Analog sonar rangefinder
// ************************

#ifdef MAXSONAR
void Sonar_update(void)
{
	sonarAlt = AnalogRead(SONARCHANNEL)*SONARSCALE;
}


#endif




// ****************
// GYRO common part
// ****************
void GYRO_Common(void) {
	static int16_t previousGyroADC[3] = {0,0,0};
	static int32_t g[3];
	uint8_t axis;

#if defined MMGYRO
	// Moving Average Gyros by Magnetron1
	//---------------------------------------------------
	static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGHT];
	static int32_t mediaMobileGyroADCSum[3];
	static uint8_t mediaMobileGyroIDX;
	//---------------------------------------------------
#endif

	if (calibratingG>0) {
		for (axis = 0; axis < 3; axis++) {
			// Reset g[axis] at start of calibration
			if (calibratingG == 400) g[axis]=0;
			// Sum up 400 readings
			g[axis] +=gyroADC[axis];
			// Clear global variables for next reading
			gyroADC[axis]=0;
			gyroZero[axis]=0;
			if (calibratingG == 1) {
				gyroZero[axis]=g[axis]/400;
				blinkLED(10,15,1);
			}
		}
		calibratingG--;
	}

#ifdef MMGYRO
	mediaMobileGyroIDX = ++mediaMobileGyroIDX % MMGYROVECTORLENGHT;
	for (axis = 0; axis < 3; axis++) {
		gyroADC[axis]  -= gyroZero[axis];
		mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
		//anti gyro glitch, limit the variation between two consecutive readings
		mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
		mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
		gyroADC[axis] = mediaMobileGyroADCSum[axis] / MMGYROVECTORLENGHT;
#else
		for (axis = 0; axis < 3; axis++) {
			gyroADC[axis]  -= gyroZero[axis];
			//anti gyro glitch, limit the variation between two consecutive readings
			gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
#endif
			previousGyroADC[axis] = gyroADC[axis];
#ifdef MAYHONY
			GYRO_AHRS_ORIENTATION(gyroADC[ROLL], gyroADC[PITCH], gyroADC[YAW]);
#endif
		}
	}



	// ****************
	// ACC common part
	// ****************
	void ACC_Common(void) {
		static int32_t a[3];
		uint8_t axis = 0;
		if (calibratingA>0) {
			for ( axis = 0; axis < 3; axis++) {
				// Reset a[axis] at start of calibration
				if (calibratingA == 400) a[axis]=0;
				// Sum up 400 readings
				a[axis] +=accADC[axis];
				// Clear global variables for next reading
				accADC[axis]=0;
				conf.accZero[axis]=0;
			}
			// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
			if (calibratingA == 1) {
				conf.accZero[ROLL]  = a[ROLL]/400;
				conf.accZero[PITCH] = a[PITCH]/400;
				conf.accZero[YAW]   = a[YAW]/400-acc_1G; // for nunchuk 200=1G
				conf.angleTrim[ROLL]   = 0;
				conf.angleTrim[PITCH]  = 0;
				writeParams(1); // write accZero in EEPROM
			}
			calibratingA--;
		}
#if defined(INFLIGHT_ACC_CALIBRATION)
		static int32_t b[3];
		static int16_t accZero_saved[3]  = {0,0,0};
		static int16_t  angleTrim_saved[2] = {0, 0};
		//Saving old zeropoints before measurement
		if (InflightcalibratingA==50) {
			accZero_saved[ROLL]  = conf.accZero[ROLL] ;
			accZero_saved[PITCH] = conf.accZero[PITCH];
			accZero_saved[YAW]   = conf.accZero[YAW] ;
			angleTrim_saved[ROLL] = conf.angleTrim[ROLL] ;
			angleTrim_saved[PITCH] = conf.angleTrim[PITCH] ;
		}
		if (InflightcalibratingA>0) {
			for (uint8_t axis = 0; axis < 3; axis++) {
				// Reset a[axis] at start of calibration
				if (InflightcalibratingA == 50) b[axis]=0;
				// Sum up 50 readings
				b[axis] +=accADC[axis];
				// Clear global variables for next reading
				accADC[axis]=0;
				conf.accZero[axis]=0;
			}
			//all values are measured
			if (InflightcalibratingA == 1) {
				AccInflightCalibrationActive = 0;
				AccInflightCalibrationMeasurementDone = 1;
#if defined(BUZZER)
				toggleBeep = 2;      //buzzer for indicatiing the end of calibration
#endif
				// recover saved values to maintain current flight behavior until new values are transferred
				conf.accZero[ROLL]  = accZero_saved[ROLL] ;
				conf.accZero[PITCH] = accZero_saved[PITCH];
				conf.accZero[YAW]   = accZero_saved[YAW] ;
				conf.angleTrim[ROLL] = angleTrim_saved[ROLL] ;
				conf.angleTrim[PITCH] = angleTrim_saved[PITCH] ;
			}
			InflightcalibratingA--;
		}
		// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
		if (AccInflightCalibrationSavetoEEProm == 1){  //the copter is landed, disarmed and the combo has been done again
			AccInflightCalibrationSavetoEEProm = 0;
			conf.accZero[ROLL]  = b[ROLL]/50;
			conf.accZero[PITCH] = b[PITCH]/50;
			conf.accZero[YAW]   = b[YAW]/50-acc_1G; // for nunchuk 200=1G
			conf.angleTrim[ROLL]   = 0;
			conf.angleTrim[PITCH]  = 0;
			writeParams(1); // write accZero in EEPROM
		}
#endif
		accADC[ROLL]  -=  conf.accZero[ROLL] ;
		accADC[PITCH] -=  conf.accZero[PITCH];
		accADC[YAW]   -=  conf.accZero[YAW] ;
	}


	// ************************************************************************************************************
	// I2C Compass common function
	// ************************************************************************************************************
#if MAG


	uint8_t Mag_getADC(void) {
		static uint32_t t,tCal = 0;
		static int16_t magZeroTempMin[3];
		static int16_t magZeroTempMax[3];
		uint8_t axis;
		if ( currentTime < t ) return 0; //each read is spaced by 100ms (changed to 20ms)
		t = currentTime + 100000;

		Device_Mag_getADC();
		magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
		magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
		magADC[YAW]   = magADC[YAW]   * magCal[YAW];
		if (f.CALIBRATE_MAG) {
			tCal = t;
			for(axis=0;axis<3;axis++) {
				conf.magZero[axis] = 0;
				magZeroTempMin[axis] = magADC[axis];
				magZeroTempMax[axis] = magADC[axis];
			}
			f.CALIBRATE_MAG = 0;
		}
		if (magInit) { // we apply offset only once mag calibration is done
			magADC[ROLL]  -= conf.magZero[ROLL];
			magADC[PITCH] -= conf.magZero[PITCH];
			magADC[YAW]   -= conf.magZero[YAW];
		}

		if (tCal != 0) {
			if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
				GPIOToggleBitValue( LED_PORT, LED_BIT);
				for(axis=0;axis<3;axis++) {
					if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
					if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
				}
			} else {
				tCal = 0;
				for(axis=0;axis<3;axis++)
					conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
				writeParams(1);
			}
		}
		return 1;
	}
#endif
	// ************************************************************************************************************
	// I2C Barometer common function
	// ************************************************************************************************************

#if BARO

#define BARO_TAB_SIZE   21

	void Baro_Common()
	{
		static int32_t baroHistTab[BARO_TAB_SIZE];
		static uint8_t baroHistIdx;

		uint8_t indexplus1 = (baroHistIdx + 1);
		if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
		baroHistTab[baroHistIdx] = baroPressure;
		baroPressureSum += baroHistTab[baroHistIdx];
		baroPressureSum -= baroHistTab[indexplus1];
		baroHistIdx = indexplus1;
	}
#endif

	void initSensors() {
		delayMs(200);
		//POWERPIN_ON;
		delayMs(100);
		if ( I2CInit( (uint32_t)I2CMASTER ) == FALSE )	/* initialize I2c */
		{
			initError();				/* Fatal error */
		}
		delayMs(100);
		if (GYRO) Gyro_init();
		if (BARO) Baro_init();
		if (MAG) Mag_init();
		if (ACC) {ACC_init();acc_25deg = acc_1G * 0.423;}
#if defined(MAXSONAR) || defined(VBAT)
		ADCInit( ADC_CLK );
#endif
		f.I2C_INIT_DONE = 1;
	}

	void initError(void)
	{
		while (1)
		{
			delayMs(2000);
			LEDPIN_TOGGLE;
		}
	}
