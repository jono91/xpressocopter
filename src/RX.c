/*
 * RX.c

 *
 *  Created on: 3/02/2013
 *      Author: Jono
 */
#include "LPC13Uxx.h"
#include "RX.h"
#include "gpio.h"
#include "microsec.h"
#include "config.h"
#include "def.h"


static uint16_t edgeTime[8] = {0};
static uint16_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502};
static uint8_t rcChannel[8]  = {ROLL, PITCH, YAW, THROTTLE, AUX1,AUX2,AUX3,AUX4};
volatile uint8_t GoodPulses = 0;
extern uint16_t rcData[8];
extern int16_t failsafeCnt;
extern int16_t  sonarAlt;


void configureReceiver(void)
{
	GPIOSetDir( RX0_PORT, RX0_BIT, 0);
	GPIOSetPinInterrupt( RX0_CH, RX0_PORT, RX0_BIT, edgeTrig, bothEdges);
	GPIOPinIntEnable( RX0_CH, bothEdges );

	GPIOSetDir( RX1_PORT, RX1_BIT, 0);
	GPIOSetPinInterrupt( RX1_CH, RX1_PORT, RX1_BIT, edgeTrig, bothEdges);
	GPIOPinIntEnable( RX1_CH, bothEdges );

	GPIOSetDir( RX2_PORT, RX2_BIT, 0);
	GPIOSetPinInterrupt( RX2_CH, RX2_PORT, RX2_BIT, edgeTrig, bothEdges);
	GPIOPinIntEnable( RX2_CH, bothEdges );

	GPIOSetDir( RX3_PORT, RX3_BIT, 0);
	GPIOSetPinInterrupt( RX3_CH, RX3_PORT, RX3_BIT, edgeTrig, bothEdges);
	GPIOPinIntEnable( RX3_CH, bothEdges );

	GPIOSetDir( RX4_PORT, RX4_BIT, 0);
	GPIOSetPinInterrupt( RX4_CH, RX4_PORT, RX4_BIT, edgeTrig, bothEdges);
	GPIOPinIntEnable( RX4_CH, bothEdges );

	GPIOSetDir( RX5_PORT, RX5_BIT, 0);
	GPIOSetPinInterrupt( RX5_CH, RX5_PORT, RX5_BIT, edgeTrig, bothEdges);
	GPIOPinIntEnable( RX5_CH, bothEdges );

#ifdef MAXSONAR_PWM
	GPIOSetDir( SONARPWM_PORT, SONARPWM_BIT, 0);
	GPIOSetPinInterrupt( SONARPWM_CH, SONARPWM_PORT, SONARPWM_BIT, edgeTrig, bothEdges);
	GPIOPinIntEnable( SONARPWM_CH, bothEdges );
#endif
}

/*****************************************************************************
** Function name:		PIN_INT0_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT0_IRQHandler(void)
{
	uint16_t cTime,dTime;

	if ( LPC_GPIO_PIN_INT->IST & (0x1<<0) )
	{
		cTime = micros();

		if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<0) )
		{
			//level trigger
		}
		else
		{
			if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<0) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<0) ) )
			{
				edgeTime[0] = cTime;  			//save rising edge time
				LPC_GPIO_PIN_INT->RISE = 0x1<<0;
			}
			if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<0) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<0) ) )
			{
				dTime = cTime-edgeTime[0];      //calculate width
				if (900<dTime && dTime<2200)
					{
						rcValue[THROTTLE] = dTime; //save width if in range
						#ifdef FAILSAFE
							if(dTime > FAILSAFE_DETECT_TRESHOLD)
								GoodPulses |= (1<<THROTTLE);
						#endif
					}
				LPC_GPIO_PIN_INT->FALL = 0x1<<0;
			}

			#if defined(FAILSAFE) && !defined(PROMICRO)
			  if (GoodPulses==(1<<THROTTLE)+(1<<YAW)+(1<<ROLL)+(1<<PITCH)) {  // If all main four chanells have good pulses, clear FailSafe counter
				GoodPulses = 0;
				if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;
			  }
			#endif


			LPC_GPIO_PIN_INT->IST = 0x1<<0;
		}
	}
	return;
}

/*****************************************************************************
** Function name:		PIN_INT1_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT1_IRQHandler(void)
{
	uint16_t cTime,dTime;

  if ( LPC_GPIO_PIN_INT->IST & (0x1<<1) )
  {

	  cTime = micros();

	if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<1) )
	{
			//level trig
	}
	else
	{
	  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<1) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<1) ) )
	  {
		  edgeTime[1] = cTime;  			//save rising edge time
		LPC_GPIO_PIN_INT->RISE = 0x1<<1;
	  }
	  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<1) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<1) ) )
	  {
		  dTime = cTime-edgeTime[1];      //calculate width
		  if (900<dTime && dTime<2200)
		  {
			  rcValue[ROLL] = dTime; //save width if in range
				#ifdef FAILSAFE
					if(dTime > FAILSAFE_DETECT_TRESHOLD)
						GoodPulses |= (1<<ROLL);
				#endif
		  }
		LPC_GPIO_PIN_INT->FALL = 0x1<<1;
	  }
	  LPC_GPIO_PIN_INT->IST = 0x1<<1;
	}
  }
  return;
}

/*****************************************************************************
** Function name:		PIN_INT2_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT2_IRQHandler(void)
{

	uint16_t cTime,dTime;

  if ( LPC_GPIO_PIN_INT->IST & (0x1<<2) )
  {

	  cTime = micros();

	if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<2) )
	{
		//level
	}
	else
	{
	  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<2) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<2) ) )
	  {
		  edgeTime[2] = cTime;  			//save rising edge time
		LPC_GPIO_PIN_INT->RISE = 0x1<<2;
	  }
	  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<2) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<2) ) )
	  {
		  dTime = cTime-edgeTime[2];      //calculate width
		  if (900<dTime && dTime<2200)
			  {
			  	  rcValue[PITCH] = dTime; //save width if in range
					#ifdef FAILSAFE
						if(dTime > FAILSAFE_DETECT_TRESHOLD)
							GoodPulses |= (1<<PITCH);
					#endif
			  }
		LPC_GPIO_PIN_INT->FALL = 0x1<<2;
	  }
	  LPC_GPIO_PIN_INT->IST = 0x1<<2;
	}
  }
  return;
}

/*****************************************************************************
** Function name:		PIN_INT3_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT3_IRQHandler(void)
{

	uint16_t cTime,dTime;

  if ( LPC_GPIO_PIN_INT->IST & (0x1<<3) )
  {

	  cTime = micros();

	if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<3) )
	{
		//level
	}
	else
	{
	  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<3) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<3) ) )
	  {
		  edgeTime[3] = cTime;  			//save rising edge time
		LPC_GPIO_PIN_INT->RISE = 0x1<<3;
	  }
	  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<3) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<3) ) )
	  {
		  dTime = cTime-edgeTime[3];      //calculate width
		  if (900<dTime && dTime<2200)
			  {
			  	  rcValue[YAW] = dTime; //save width if in range
					#ifdef FAILSAFE
						if(dTime > FAILSAFE_DETECT_TRESHOLD)
							GoodPulses |= (1<<YAW);
					#endif
			  }
		LPC_GPIO_PIN_INT->FALL = 0x1<<3;
	  }
	  LPC_GPIO_PIN_INT->IST = 0x1<<3;
	}
  }
  return;
}

/*****************************************************************************
** Function name:		PIN_INT4_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT4_IRQHandler(void)
{

	uint16_t cTime,dTime;

  if ( LPC_GPIO_PIN_INT->IST & (0x1<<4) )
  {

	  cTime = micros();

	if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<4) )
	{
		//level
	}
	else
	{
	  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<4) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<4) ) )
	  {
		  edgeTime[4] = cTime;  			//save rising edge time
		LPC_GPIO_PIN_INT->RISE = 0x1<<4;
	  }
	  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<4) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<4) ) )
	  {
		  dTime = cTime-edgeTime[4];      //calculate width
		  if (900<dTime && dTime<2200) rcValue[AUX1] = dTime; //save width if in range
		LPC_GPIO_PIN_INT->FALL = 0x1<<4;
	  }
	  LPC_GPIO_PIN_INT->IST = 0x1<<4;
	}
  }
  return;
}

/*****************************************************************************
** Function name:		PIN_INT5_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT5_IRQHandler(void)
{

	uint16_t cTime,dTime;

  if ( LPC_GPIO_PIN_INT->IST & (0x1<<5) )
  {

	  cTime = micros();

	if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<5) )
	{
		//level
	}
	else
	{
	  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<5) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<5) ) )
	  {
		  edgeTime[5] = cTime;  			//save rising edge time
		LPC_GPIO_PIN_INT->RISE = 0x1<<5;
	  }
	  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<5) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<5) ) )
	  {
		  dTime = cTime-edgeTime[5];      //calculate width
		  if (900<dTime && dTime<2200) rcValue[AUX2] = dTime; //save width if in range
		LPC_GPIO_PIN_INT->FALL = 0x1<<5;
	  }
	  LPC_GPIO_PIN_INT->IST = 0x1<<5;
	}
  }
  return;
}

/*****************************************************************************
** Function name:		PIN_INT6_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT6_IRQHandler(void)
{
#ifdef MAXSONAR_PWM
	uint16_t cTime,dTime;

	  if ( LPC_GPIO_PIN_INT->IST & (0x1<<6) )
	  {

		  cTime = micros();

		if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<6) )
		{
			//level
		}
		else
		{
		  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<6) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<6) ) )
		  {
			  edgeTime[6] = cTime;  			//save rising edge time
			LPC_GPIO_PIN_INT->RISE = 0x1<<6;
		  }
		  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<6) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<6) ) )
		  {
			  dTime = cTime-edgeTime[6];      //calculate width
			  sonarAlt = dTime * 2.54f / 147;//save width if in range
			LPC_GPIO_PIN_INT->FALL = 0x1<<6;
		  }
		  LPC_GPIO_PIN_INT->IST = 0x1<<6;
		}
	  }
	  return;
#else
	  if ( LPC_GPIO_PIN_INT->IST & (0x1<<6) )
	    {
	  	if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<6) )
	  	{

	  	}
	  	else
	  	{
	  	  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<6) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<6) ) )
	  	  {

	  		LPC_GPIO_PIN_INT->RISE = 0x1<<6;
	  	  }
	  	  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<6) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<6) ) )
	  	  {

	  		LPC_GPIO_PIN_INT->FALL = 0x1<<6;
	  	  }
	  	  LPC_GPIO_PIN_INT->IST = 0x1<<6;
	  	}
	    }
	    return;
#endif
}

/*****************************************************************************
** Function name:		PIN_INT7_IRQHandler
**
** Descriptions:		Use one GPIO pin as interrupt source
**
** parameters:			None
**
** Returned value:		None
**
*****************************************************************************/
void PIN_INT7_IRQHandler(void)
{

  if ( LPC_GPIO_PIN_INT->IST & (0x1<<7) )
  {
	if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<7) )
	{

	}
	else
	{
	  if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<7) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<7) ) )
	  {

		LPC_GPIO_PIN_INT->RISE = 0x1<<7;
	  }
	  if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<7) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<7) ) )
	  {

		LPC_GPIO_PIN_INT->FALL = 0x1<<7;
	  }
	  LPC_GPIO_PIN_INT->IST = 0x1<<7;
	}
  }
  return;
}
/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/
uint16_t readRawRC(uint8_t chan) {
  uint16_t data;

  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  #if defined(SPEKTRUM)
    static uint32_t spekChannelData[SPEK_MAX_CHANNEL];
    if (rcFrameComplete) {
      for (uint8_t b = 3; b < SPEK_FRAME_SIZE; b += 2) {
        uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> SPEK_CHAN_SHIFT);
        if (spekChannel < SPEK_MAX_CHANNEL) spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & SPEK_CHAN_MASK) << 8) + spekFrame[b];
      }
      rcFrameComplete = 0;
    }
  #endif

  #if defined(SPEKTRUM)
    static uint8_t spekRcChannelMap[SPEK_MAX_CHANNEL] = {1,2,3,0,4,5,6};
    if (chan >= SPEK_MAX_CHANNEL) {
      data = 1500;
    } else {
      #if (SPEKTRUM == 1024)
        data = 988 + spekChannelData[spekRcChannelMap[chan]];          // 1024 mode
      #endif
      #if (SPEKTRUM == 2048)
        data = 988 + (spekChannelData[spekRcChannelMap[chan]] >> 1);   // 2048 mode
      #endif
    }
  #endif
  return data; // We return the value correctly copied when the IRQ's where disabled
}
/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
void computeRC() {
  static int16_t rcData4Values[8][4], rcDataMean[8];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  #if !(defined(RCSERIAL) || defined(OPENLRSv2MULTI)) // dont know if this is right here
    #if defined(SBUS)
      readSBus();
    #endif
    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) {
		#if defined(yourmum)
		  uint16_t rcval = readRawRC(chan);
		  if(rcval>FAILSAFE_DETECT_TRESHOLD || chan > 3) {        // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
			rcData4Values[chan][rc4ValuesIndex%4] = rcval;
		  }
		#else
		  rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
		#endif
      rcDataMean[chan] = 0;
      for (a=0;a<4;a++) rcDataMean[chan] += rcData4Values[chan][a];
      rcDataMean[chan]= (rcDataMean[chan]+2)/4;
      if ( rcDataMean[chan] < rcData[chan] -3)  rcData[chan] = rcDataMean[chan]+2;
      if ( rcDataMean[chan] > rcData[chan] +3)  rcData[chan] = rcDataMean[chan]-2;
    }
  #endif
}
