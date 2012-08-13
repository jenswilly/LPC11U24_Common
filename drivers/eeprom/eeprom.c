/*
 * eeprom.c
 *
 *  Created on: Aug 12, 2012
 *      Author: jenswilly
 * 
 *  This file implements methods to store and read data in the EEPROM.
 *  Code is from AN11073 "Using LPC11Axx EEPROM with IAP": http://www.lpcware.com/content/nxpfile/an11073-using-lpc11axx-eeprom-iap-v10
 */

#include "LPC11Uxx.h"
#include "eeprom.h"

#define IAP_LOCATION 0x1FFF1FF1
typedef void (*IAP) (	unsigned int command[],
						unsigned int result[] );
static const IAP iap_entry = (IAP) IAP_LOCATION;
//1) EEprom Write
//
//Command code: 61
//Param0: eeprom address (byte, half-word or word aligned)
//Param1: RAM address (byte, half-word or word aligned)
//Param2: Number of bytes to be written ( Byte, Half-words write are ok)
//Param3: System Clock Frequency (CCLK) in kHz
//
//Return Code CMD_SUCCESS | SRC_ADDR_NOT_MAPPED | DST_ADDR_NOT_MAPPED
void writeEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount )
{
	unsigned int command[5], result[4];

	command[0] = 61;
	command[1] = (uint32_t) eeAddress;
	command[2] = (uint32_t) buffAddress;
	command[3] = byteCount;
	command[4] = SystemCoreClock/1000;

	/* Invoke IAP call...*/
	iap_entry(command, result);
	if (0 != result[0])
	{
		//Trap error
		while(1);
	}
	return;
}

//2) EEprom Read
//Command code: 62
//Param0: eeprom address (byte, half-word or word aligned)
//Param1: RAM address (byte, half-word or word aligned)
//Param2: Number of bytes to be read ( Byte, Half-words read are ok)
//Param3: System Clock Frequency (CCLK) in kHz
//
//Return Code CMD_SUCCESS | SRC_ADDR_NOT_MAPPED | DST_ADDR_NOT_MAPPED
void readEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount )
{
	unsigned int command[5], result[4];

	command[0] = 62;
	command[1] = (uint32_t) eeAddress;
	command[2] = (uint32_t) buffAddress;
	command[3] = byteCount;
	command[4] = SystemCoreClock/1000;

	/* Invoke IAP call...*/
  	iap_entry( command, result);
	if (0 != result[0])
	{
		//Trap error
		while(1);
	}
	return;
}
