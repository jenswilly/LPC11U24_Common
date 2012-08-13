/*
 * eeprom.h
 *
 *  Created on: Aug 12, 2012
 *      Author: jenswilly
 * 
 *  Header file for EEPROM access methods.
 */

#ifndef __EEPROM_H
#define __EEPROM_H

void writeEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount );
void readEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount );

#endif /* end __EEPROM_H */
