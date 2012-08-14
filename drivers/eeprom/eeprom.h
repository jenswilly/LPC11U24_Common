/****************************************************************************
 *   $Id:: eeprom.h 7171 2011-04-26 20:23:50Z nxp28548                      $
 *   Project: NXP LPC11Axx EEPROM example
 *
 *   Description:
 *     This file contains EEPROM code header definition.
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
* documentation is hereby granted, under NXP Semiconductors’ 
* relevant copyright in the software, without fee, provided that it 
* is used in conjunction with NXP Semiconductors microcontrollers.  This 
* copyright, permission, and disclaimer notice must appear in all copies of 
* this code.

****************************************************************************/

#ifndef __EEPROM_H 
#define __EEPROM_H

void writeEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount );
void readEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount );

#endif /* end __EEPROM_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
