/*
 * EEPROM.h
 *
 *  Created on: 4/02/2013
 *      Author: Jono
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "type.h"


#define EEPROM_CONF_VERSION 161



void eeprom_write_block( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount );
void eeprom_read_block( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount );
void readEEPROM(void);
void writeParams(uint8_t b);
void checkFirstTime(void);

#endif /* EEPROM_H_ */
