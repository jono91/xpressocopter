/****************************************************************************
 *   $Id:: uart.c 7125 2011-04-15 00:22:12Z usb01267                        $
 *   Project: NXP LPC13Uxx UART example
 *
 *   Description:
 *     This file contains UART code example which include UART 
 *     initialization, UART interrupt handler, and related APIs for 
 *     UART access.
 *
 ****************************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
* Permission to use, copy, modify, and distribute this software and its
* documentation is hereby granted, under NXP Semiconductors'
* relevant copyright in the software, without fee, provided that it
* is used in conjunction with NXP Semiconductors microcontrollers.  This
* copyright, permission, and disclaimer notice must appear in all copies of
* this code.
****************************************************************************/
#include "LPC13Uxx.h"
#include "type.h"
#include "serial.h"
#include "config.h"
#include "def.h"
#include "EEPROM.h"
#include "stdio.h"
#include "string.h"
#include "gpio.h"

uint32_t tailerr =0,txtailerr = 0;

volatile uint32_t UARTStatus;
volatile uint8_t  UARTTxEmpty = 1;
volatile uint8_t  serialBufferRX[RX_BUFFER_SIZE];
volatile uint8_t serialHeadRX = 0, serialTailRX = 0;

volatile uint8_t bufTX[TX_BUFFER_SIZE];
volatile uint8_t headTX,tailTX;

static uint8_t inBuf[INBUF_SIZE];

static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

extern struct conf_def conf;

#if AUTOBAUD_ENABLE
volatile uint32_t UARTAutoBaud = 0, AutoBaudTimeout = 0;
#endif

static const char boxnames[] = // names for dynamic generation of config GUI
  "ACC;"
  "BARO;"
  "MAG;"
  "CAMSTAB;"
  "CAMTRIG;"
  "ARM;"
  "GPS HOME;"
  "GPS HOLD;"
  "PASSTHRU;"
  "HEADFREE;"
  "BEEPER;"
  "LEDMAX;"
  "LLIGHTS;"
  "HEADADJ;"
  "OPTIC FLOW;"
;

static const char pidnames[] =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

extern int16_t rcData[8];          // interval [1000;2000]
extern uint16_t cycleTime;
extern int16_t  i2c_errors_count;
extern struct flags_struct f;
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern int16_t  accSmooth[3],magADC[3];
extern int16_t gyroData[3];
extern int16_t motor[NUMBER_MOTOR];
extern int16_t angle[2];
extern int16_t  heading;
extern int16_t  headFreeModeHold;
extern int32_t  EstAlt;
extern uint8_t  vbat;
extern uint16_t intPowerMeterSum, intPowerTrigger1;
extern uint8_t PWM_PIN[8];
extern uint16_t calibratingA;
extern int16_t  debug[4];
#ifdef SERVO
	extern int16_t servo[8];
#endif

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

uint32_t read32(void) {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16(void) {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8(void)  {
  return inBuf[indRX++]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply(void) {
  serialize8(checksum);UartSendData();
}

void serializeNames(const char* s, uint8_t length) {
	uint8_t i = 0;
	for (i = 0; i<length;i++)
	{
		serialize8(*s);
		s++;
	}
}

void serialCom(void) {
  uint8_t c;
  static uint8_t offset;
  static uint8_t dataSize;
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;

  while (SerialAvailable()) {
    uint8_t bytesTXBuff = 0;

   	bytesTXBuff = ((uint8_t)(headTX-tailTX))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer

    if (bytesTXBuff > TX_BUFFER_SIZE - 40 ) return; // ensure there is enough free TX buffer to go further (40 bytes margin)
    c = SerialRead();

    if (c_state == IDLE) {
      c_state = (c=='$') ? HEADER_START : IDLE;
      if (c_state == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
    } else if (c_state == HEADER_START) {
      c_state = (c=='M') ? HEADER_M : IDLE;
    } else if (c_state == HEADER_M) {
      c_state = (c=='<') ? HEADER_ARROW : IDLE;
    } else if (c_state == HEADER_ARROW) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        c_state = IDLE;
        continue;
      }
      dataSize = c;
      offset = 0;
      checksum = 0;
      indRX = 0;
      checksum ^= c;
      c_state = HEADER_SIZE;  // the command is to follow
    } else if (c_state == HEADER_SIZE) {
      cmdMSP = c;
      checksum ^= c;
      c_state = HEADER_CMD;
    } else if (c_state == HEADER_CMD && offset < dataSize) {
      checksum ^= c;
      inBuf[offset++] = c;
    } else if (c_state == HEADER_CMD && offset >= dataSize) {
      if (checksum == c) {  // compare calculated and transferred checksum
        evaluateCommand();  // we got a valid packet, evaluate it
      }
      c_state = IDLE;
    }
  }
}

void evaluateCommand(void) {
	uint8_t i = 0;
  switch(cmdMSP) {
   case MSP_SET_RAW_RC:
     for( i=0;i<8;i++) {
       rcData[i] = read16();
     }
     headSerialReply(0);
     break;
#if GPS
   case MSP_SET_RAW_GPS:
     f.GPS_FIX = read8();
     GPS_numSat = read8();
     GPS_coord[LAT] = read32();
     GPS_coord[LON] = read32();
     GPS_altitude = read16();
     GPS_speed = read16();
     GPS_update |= 2;              // New data signalisation to GPS functions
     headSerialReply(0);
     break;
#endif
   case MSP_SET_PID:
     for( i=0;i<PIDITEMS;i++) {
       conf.P8[i]=read8();
       conf.I8[i]=read8();
       conf.D8[i]=read8();
     }
     headSerialReply(0);
     break;
   case MSP_SET_BOX:
     for( i=0;i<CHECKBOXITEMS;i++) {
       conf.activate[i]=read16();
     }
     headSerialReply(0);
     break;
   case MSP_SET_RC_TUNING:
     conf.rcRate8 = read8();
     conf.rcExpo8 = read8();
     conf.rollPitchRate = read8();
     conf.yawRate = read8();
     conf.dynThrPID = read8();
     conf.thrMid8 = read8();
     conf.thrExpo8 = read8();
     headSerialReply(0);
     break;
   case MSP_SET_MISC:
     #if defined(POWERMETER)
       conf.powerTrigger1 = read16() / PLEVELSCALE;
     #endif
     headSerialReply(0);
     break;
   case MSP_IDENT:
     headSerialReply(7);
     serialize8(VERSION);   // multiwii version
     serialize8(MULTITYPE); // type of multicopter
     serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
     serialize32(0);        // "capability"
     break;
   case MSP_STATUS:
     headSerialReply(10);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
     serialize32(f.ACC_MODE<<BOXACC|f.BARO_MODE<<BOXBARO|f.MAG_MODE<<BOXMAG|f.ARMED<<BOXARM|
                 rcOptions[BOXCAMSTAB]<<BOXCAMSTAB | rcOptions[BOXCAMTRIG]<<BOXCAMTRIG |
                 f.GPS_HOME_MODE<<BOXGPSHOME|f.GPS_HOLD_MODE<<BOXGPSHOLD|f.HEADFREE_MODE<<BOXHEADFREE|
                 f.PASSTHRU_MODE<<BOXPASSTHRU|rcOptions[BOXBEEPERON]<<BOXBEEPERON|rcOptions[BOXLEDMAX]<<BOXLEDMAX|rcOptions[BOXLLIGHTS]<<BOXLLIGHTS|rcOptions[BOXHEADADJ]<<BOXHEADADJ);
     break;
   case MSP_RAW_IMU:
     headSerialReply(18);
     for( i=0;i<3;i++) serialize16(accSmooth[i]);
     for( i=0;i<3;i++) serialize16(gyroData[i]);
     for( i=0;i<3;i++) serialize16(magADC[i]);
     break;
   case MSP_SERVO:
     headSerialReply(16);
     for( i=0;i<8;i++)
       #if defined(SERVO)
       serialize16(servo[i]);
       #else
       serialize16(0);
       #endif
     break;
   case MSP_MOTOR:
     headSerialReply(16);
     for( i=0;i<8;i++) {
       serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
   case MSP_RC:
     headSerialReply(16);
     for( i=0;i<8;i++) serialize16(rcData[i]);
     break;
#if GPS
   case MSP_RAW_GPS:
     headSerialReply(14);
     serialize8(f.GPS_FIX);
     serialize8(GPS_numSat);
     serialize32(GPS_coord[LAT]);
     serialize32(GPS_coord[LON]);
     serialize16(GPS_altitude);
     serialize16(GPS_speed);
     break;
   case MSP_COMP_GPS:
     headSerialReply(5);
     serialize16(GPS_distanceToHome);
     serialize16(GPS_directionToHome);
     serialize8(GPS_update & 1);
     break;
#endif
   case MSP_ATTITUDE:
     headSerialReply(8);
     for( i=0;i<2;i++) serialize16(angle[i]);
     serialize16(heading);
     serialize16(headFreeModeHold);
     break;
   case MSP_ALTITUDE:
     headSerialReply(4);
     serialize32(EstAlt);
     break;
   case MSP_BAT:
     headSerialReply(3);
     serialize8(vbat);
     serialize16(intPowerMeterSum);
     break;
   case MSP_RC_TUNING:
     headSerialReply(7);
     serialize8(conf.rcRate8);
     serialize8(conf.rcExpo8);
     serialize8(conf.rollPitchRate);
     serialize8(conf.yawRate);
     serialize8(conf.dynThrPID);
     serialize8(conf.thrMid8);
     serialize8(conf.thrExpo8);
     break;
   case MSP_PID:
     headSerialReply(3*PIDITEMS);
     for( i=0;i<PIDITEMS;i++) {
       serialize8(conf.P8[i]);
       serialize8(conf.I8[i]);
       serialize8(conf.D8[i]);
     }
     break;
   case MSP_BOX:
     headSerialReply(2*CHECKBOXITEMS);
     for( i=0;i<CHECKBOXITEMS;i++) {
       serialize16(conf.activate[i]);
     }
     break;
   case MSP_BOXNAMES:
     headSerialReply(strlen(boxnames));
     serializeNames(boxnames,strlen(boxnames));
     break;
   case MSP_PIDNAMES:
     headSerialReply(strlen(pidnames));
     serializeNames(pidnames,strlen(boxnames));
     break;
   case MSP_MISC:
     headSerialReply(2);
     serialize16(intPowerTrigger1);
     break;
   case MSP_MOTOR_PINS:
     headSerialReply(8);
     for( i=0;i<8;i++) {
       serialize8(PWM_PIN[i]);
     }
     break;

#if defined(USE_MSP_WP)
   case MSP_WP:
     {
      uint8_t wp_no = read8();    //get the wp number
      headSerialReply(12);
      if (wp_no == 0) {
        serialize8(0);                   //wp0
        serialize32(GPS_home[LAT]);
        serialize32(GPS_home[LON]);
        serialize16(0);                  //altitude will come here
        serialize8(0);                   //nav flag will come here
      } else if (wp_no == 16)
      {
        serialize8(16);                  //wp16
        serialize32(GPS_hold[LAT]);
        serialize32(GPS_hold[LON]);
        serialize16(0);                  //altitude will come here
        serialize8(0);                   //nav flag will come here
      }
     }
     break;
#endif

   case MSP_RESET_CONF:
     conf.checkNewConf++;
     checkFirstTime();
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     calibratingA=400;
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     headSerialReply(8);
     for( i=0;i<4;i++) {
       serialize16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  switch (sr) {
  // Note: we may receive weird characters here which could trigger unwanted features during flight.
  //       this could lead to a crash easily.
  //       Please use if (!f.ARMED) where neccessary
    #ifdef LCD_CONF
    case 's':
    case 'S':
      if (!f.ARMED) configurationLoop();
      break;
    #endif
    #ifdef LCD_TELEMETRY
    case 'A': // button A press
      toggle_telemetry(1);
      break;
    case 'B': // button B press
      toggle_telemetry(2);
      break;
    case 'C': // button C press
      toggle_telemetry(3);
      break;
    case 'D': // button D press
      toggle_telemetry(4);
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    #if defined(LOG_VALUES) && defined(DEBUG)
    case 'R':
    #endif
    #ifdef DEBUG
    case 'F':
    #endif
      toggle_telemetry(sr);
      break;
    case 'a': // button A release
    case 'b': // button B release
    case 'c': // button C release
    case 'd': // button D release
      break;
    #endif // LCD_TELEMETRY
  }
}

// *******************************************************
// For Teensy 2.0, these function emulate the API used for ProMicro
// it cant have the same name as in the arduino API because it wont compile for the promini (eaven if it will be not compiled)
// *******************************************************
#if defined(TEENSY20)
  unsigned char T_USB_Available(){
    int n = Serial.available();
    if (n > 255) n = 255;
    return n;
  }
#endif
/*****************************************************************************
** Function name:		USART_IRQHandler
**
** Descriptions:		USART interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void USART_IRQHandler(void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;

  IIRValue = LPC_USART->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if (IIRValue == IIR_RLS)		/* Receive Line Status */
  {
    LSRValue = LPC_USART->LSR;
    /* Receive Line Status */
    if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
    {
      /* There are errors or break interrupt */
      /* Read LSR will clear the interrupt */
      UARTStatus = LSRValue;
      Dummy = LPC_USART->RBR;	/* Dummy read on RX to clear 
								interrupt, then bail out */
      return;
    }
    if (LSRValue & LSR_RDR)	/* Receive Data Ready */			
    {
      /* If no error on RLS, normal ready, save into the data buffer. */
      /* Note: read RBR will clear the interrupt */
    	uint8_t data = LPC_USART->RBR;
    	  uint8_t h = serialHeadRX;
    	  if (++h >= RX_BUFFER_SIZE) h = 0;
    	  if (h == serialTailRX)
    		  {
    		  	  tailerr++;
    		  	  return; // we did not bite our own tail?
    		  }
    	  serialBufferRX[serialHeadRX] = data;
    	  serialHeadRX = h;
    }
  }
  else if (IIRValue == IIR_RDA)	/* Receive Data Available */
  {
    /* Receive Data Available */
	  uint8_t data = LPC_USART->RBR;

	  uint8_t h = serialHeadRX;
	  if (++h >= RX_BUFFER_SIZE) h = 0;
	  if (h == serialTailRX)
		  {
		  	  tailerr++;
		  	  return; // we did not bite our own tail?
		  }
	  serialBufferRX[serialHeadRX] = data;
	  serialHeadRX = h;
  }
  else if (IIRValue == IIR_CTI)	/* Character timeout indicator */
  {
    /* Character Time-out indicator */
    UARTStatus |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if (IIRValue == IIR_THRE)	/* THRE, transmit holding register empty */
  {
    /* THRE interrupt */
	  LSRValue = LPC_USART->LSR; //clear interrupt

	  uint8_t t = tailTX;
	  if (headTX != t) {
		if (++t >= TX_BUFFER_SIZE) t = 0;
		LPC_USART->THR = bufTX[t];  // Transmit next byte in the ring
		tailTX = t;
	  }
	  if (t == headTX) LPC_USART->IER &= ~IER_THRE; // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#if AUTOBAUD_ENABLE
  if (LPC_USART->IIR & IIR_ABEO) /* End of Auto baud */
  {
	LPC_USART->IER &= ~IIR_ABEO;
	/* clear bit ABEOInt in the IIR by set ABEOIntClr in the ACR register */
	LPC_USART->ACR |= IIR_ABEO;
	UARTAutoBaud = 1;
  }
  else if (LPC_USART->IIR & IIR_ABTO)/* Auto baud time out */
  {
	LPC_USART->IER &= ~IIR_ABTO;
	AutoBaudTimeout = 1;
	/* clear bit ABTOInt in the IIR by set ABTOIntClr in the ACR register */
	LPC_USART->ACR |= IIR_ABTO;
  }
#endif
  return;
}

#if MODEM_TEST
/*****************************************************************************
** Function name:		ModemInit
**
** Descriptions:		Initialize UART0 port as modem, setup pin select.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void ModemInit( void )
{
  
  LPC_IOCON->PIO0_7 &= ~0x07;     /* UART I/O config */
  LPC_IOCON->PIO0_7 |= 0x01;      /* UART CTS */
  LPC_IOCON->PIO0_17 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO0_17 |= 0x01;     /* UART RTS */
#if 1
  LPC_IOCON->PIO1_13 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_13 |= 0x01;     /* UART DTR */ 
  LPC_IOCON->PIO1_14 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_14 |= 0x01;     /* UART DSR */
  LPC_IOCON->PIO1_15 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_15 |= 0x01;     /* UART DCD */
  LPC_IOCON->PIO1_16 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_16 |= 0x01;     /* UART RI */

#else
  LPC_IOCON->PIO1_19 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_19 |= 0x01;     /* UART DTR */
  LPC_IOCON->PIO1_20 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_20 |= 0x01;     /* UART DSR */
  LPC_IOCON->PIO1_21 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_21 |= 0x01;     /* UART DCD */
  LPC_IOCON->PIO1_22 &= ~0x07;    /* UART I/O config */
  LPC_IOCON->PIO1_22 |= 0x01;     /* UART RI */
#endif
  LPC_USART->MCR = 0xC0;          /* Enable Auto RTS and Auto CTS. */			
  return;
}
#endif

/***********************************************************************
 *
 * Function: uart_set_divisors
 *
 * Purpose: Determines best dividers to get a target clock rate
 *
 * Processing:
 *     See function.
 *
 * Parameters:
 *     UARTClk    : UART clock
 *     baudrate   : Desired UART baud rate
 *
 * Outputs:
 *	  baudrate : Sets the estimated buadrate value in DLL, DLM, and FDR.
 *
 * Returns: Error status.
 *
 * Notes: None
 *
 **********************************************************************/
uint32_t uart_set_divisors(uint32_t UARTClk, uint32_t baudrate)
{
  uint32_t uClk;
  uint32_t calcBaudrate = 0;
  uint32_t temp = 0;

  uint32_t mulFracDiv, dividerAddFracDiv;
  uint32_t diviser = 0 ;
  uint32_t mulFracDivOptimal = 1;
  uint32_t dividerAddOptimal = 0;
  uint32_t diviserOptimal = 0;

  uint32_t relativeError = 0;
  uint32_t relativeOptimalError = 100000;

  /* get UART block clock */
  uClk = UARTClk >> 4; /* div by 16 */
  /* In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
   * The formula is :
   * BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
   * It involves floating point calculations. That's the reason the formulae are adjusted with
   * Multiply and divide method.*/
  /* The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
   * 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15 */
  for (mulFracDiv = 1; mulFracDiv <= 15; mulFracDiv++)
  {
    for (dividerAddFracDiv = 0; dividerAddFracDiv <= 15; dividerAddFracDiv++)
    {
      temp = (mulFracDiv * uClk) / ((mulFracDiv + dividerAddFracDiv));
      diviser = temp / baudrate;
      if ((temp % baudrate) > (baudrate / 2))
        diviser++;

      if (diviser > 2 && diviser < 65536)
      {
        calcBaudrate = temp / diviser;

        if (calcBaudrate <= baudrate)
          relativeError = baudrate - calcBaudrate;
        else
          relativeError = calcBaudrate - baudrate;

        if ((relativeError < relativeOptimalError))
        {
          mulFracDivOptimal = mulFracDiv ;
          dividerAddOptimal = dividerAddFracDiv;
          diviserOptimal = diviser;
          relativeOptimalError = relativeError;
          if (relativeError == 0)
            break;
        }
      } /* End of if */
    } /* end of inner for loop */
    if (relativeError == 0)
      break;
  } /* end of outer for loop  */

  if (relativeOptimalError < (baudrate / 30))
  {
    /* Set the `Divisor Latch Access Bit` and enable so the DLL/DLM access*/
    /* Initialise the `Divisor latch LSB` and `Divisor latch MSB` registers */
    LPC_USART->DLM = (diviserOptimal >> 8) & 0xFF;
    LPC_USART->DLL = diviserOptimal & 0xFF;

    /* Initialise the Fractional Divider Register */
    LPC_USART->FDR = ((mulFracDivOptimal & 0xF) << 4) | (dividerAddOptimal & 0xF);
    return( TRUE );
  }
  return ( FALSE );
}

/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART0 port, setup pin select,
**				clock, parity, stop bits, FIFO, etc.
**
** parameters:			UART baudrate
** Returned value:		None
** 
*****************************************************************************/
void UARTInit(uint32_t baudrate)
{
#if !AUTOBAUD_ENABLE
  uint32_t Fdiv;
#endif
  volatile uint32_t regVal;

  UARTTxEmpty = 1;
  serialHeadRX = 0;
  
  NVIC_DisableIRQ(USART_IRQn);
  /* Select only one location from below. */
#if 0
  LPC_IOCON->PIO0_18 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO0_18 |= 0x01;     /* UART RXD */
  LPC_IOCON->PIO0_19 &= ~0x07;	
  LPC_IOCON->PIO0_19 |= 0x01;     /* UART TXD */
#endif
#if 0
  LPC_IOCON->PIO1_14 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO1_14 |= 0x03;     /* UART RXD */
  LPC_IOCON->PIO1_13 &= ~0x07;	
  LPC_IOCON->PIO1_13 |= 0x03;     /* UART TXD */
#endif
#if 0
  LPC_IOCON->PIO1_17 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO1_17 |= 0x02;     /* UART RXD */
  LPC_IOCON->PIO1_18 &= ~0x07;	
  LPC_IOCON->PIO1_18 |= 0x02;     /* UART TXD */
#endif
#if 1
  LPC_IOCON->PIO1_26 &= ~0x07;    /*  UART I/O config */
  LPC_IOCON->PIO1_26 |= 0x02;     /* UART RXD */
  LPC_IOCON->PIO1_27 &= ~0x07;	
  LPC_IOCON->PIO1_27 |= 0x02;     /* UART TXD */
#endif

  /* Enable UART clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
  LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

  LPC_USART->LCR = 0x83;            /* 8 bits, no Parity, 1 Stop bit */
#if !AUTOBAUD_ENABLE
#if FDR_CALIBRATION
	if ( uart_set_divisors(SystemCoreClock/LPC_SYSCON->UARTCLKDIV, baudrate) != TRUE )
	{
      Fdiv = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate ;	/*baud rate */
      LPC_USART->DLM = Fdiv / 256;							
      LPC_USART->DLL = Fdiv % 256;
	  LPC_USART->FDR = 0x10;		/* Default */
	}
#else
    Fdiv = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate ;	/*baud rate */
    LPC_USART->DLM = Fdiv / 256;							
    LPC_USART->DLL = Fdiv % 256;
	LPC_USART->FDR = 0x10;		/* Default */
#endif
#endif
  LPC_USART->LCR = 0x03;		/* DLAB = 0 */
  LPC_USART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

  /* Read to clear the line status. */
  regVal = LPC_USART->LSR;

  /* Ensure a clean start, no data in either TX or RX FIFO. */
  while (( LPC_USART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
  while ( LPC_USART->LSR & LSR_RDR )
  {
	regVal = LPC_USART->RBR;	/* Dump data from RX FIFO */
  }
 
  /* Enable the UART Interrupt */
  NVIC_EnableIRQ(USART_IRQn);

#if TX_INTERRUPT
  LPC_USART->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART interrupt */
#else
  LPC_USART->IER = IER_RBR | IER_RLS;	/* Enable UART interrupt */
#endif
#if AUTOBAUD_ENABLE
    LPC_USART->IER |= IER_ABEO | IER_ABTO;
#endif
  return;
}

#ifdef debugging
/*****************************************************************************
** Function name:		UartSend
**
** Descriptions:		sends strings
** parameters:		none
** Returned value:	None
**
*****************************************************************************/
void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{

  while ( Length != 0 )
  {
	  /* THRE status, contain valid data */

	  while ( !(LPC_USART->LSR & LSR_THRE) );
	  LPC_USART->THR = *BufferPtr;

      BufferPtr++;
      Length--;
  }
  return;
}
#endif
/*****************************************************************************
** Function name:		UartSendData
**
** Descriptions:		starts the interrupt driven TX output via ring buffer
** parameters:		none
** Returned value:	None
** 
*****************************************************************************/
void UartSendData(void)
{
	LPC_USART->IER |= IER_THRE;			//start tx holding register empty interrupt
	if(LPC_USART->LSR & LSR_THRE)		//if TX is idle, no interrupt > manually start transfer
	{
		  uint8_t t = tailTX;
		  if (headTX != t) {
			if (++t >= TX_BUFFER_SIZE) t = 0;
			LPC_USART->THR = bufTX[t];  // Transmit next byte in the ring
			tailTX = t;
		  }
	}

	return;
}
#ifdef debugging
/*****************************************************************************
** Function name:		print_string
**
** Descriptions:		print out string on the terminal
**
** parameters:			pointer to the string end with NULL char.
** Returned value:		none.
** 
*****************************************************************************/
void print_string( uint8_t *str_ptr )
{
  while(*str_ptr != 0x00)
  {
    while((LPC_USART->LSR & 0x60) != 0x60);
    LPC_USART->THR = *str_ptr;
    str_ptr++;
  }
  return;
}

/*****************************************************************************
** Function name:		get_key
**
** Descriptions:		Get a character from the terminal
**
** parameters:			None
** Returned value:		character, zero is none.
** 
*****************************************************************************/
uint8_t get_key( void )
{
  uint8_t dummy;
  
  while ( !(LPC_USART->LSR & 0x01) );  
  dummy = LPC_USART->RBR;
  if ((dummy>=65) && (dummy<=90))
  {
	/* convert capital to non-capital character, A2a, B2b, C2c. */ 
	dummy +=32;
  }
  /* echo */
  LPC_USART->THR = dummy;
  return(dummy);
}
#endif

/*****************************************************************************
** Function name:		SerialRead
**
** Descriptions:		Get a character RX ring buffer
**
** parameters:			None
** Returned value:		character
**
*****************************************************************************/

uint8_t SerialRead(void) {

  uint8_t t = serialTailRX;
  uint8_t c = serialBufferRX[t];
  if (serialHeadRX != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX = t;
  }
  return c;
}

uint8_t SerialAvailable(void) {

  return (serialHeadRX != serialTailRX);
}



void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = headTX;
  if(++t == tailTX)
  {
	  txtailerr++;
	  return;
  }
  if (t >= TX_BUFFER_SIZE) t = 0;
  bufTX[t] = a;
  checksum ^= a;
  headTX = t;
}
#ifdef AHRSDEBUG
void serialFloatPrint(float f) {
  uint8_t * b = (uint8_t *) &f;
  int i =0;
  for(i=0; i<4; i++) {

    uint8_t b1 = (b[i] >> 4) & 0x0f;
    uint8_t b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    UARTSend(&c1, 1);
    UARTSend(&c2,1);
  }
}

void serialPrintFloatArr(float * arr, int length) {
	int i =0;
  for(i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    char c1 = ',';
    UARTSend(&c1, 1);
  }
}
#endif
/******************************************************************************
**                            End Of File
******************************************************************************/
