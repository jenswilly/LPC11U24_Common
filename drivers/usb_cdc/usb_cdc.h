/*
 * usb_cdc.h
 *
 *  Created on: Aug 12, 2012
 *      Author: jenswilly
 */

#ifndef USB_CDC_H_
#define USB_CDC_H_

#include "error.h"

ErrorCode_t USB_CDC_init(void);
void USB_CDC_send(uint8_t *bufferPtr, uint32_t length);
void USB_CDC_receive( uint8_t *bufferPtr, uint32_t length );	// Implement your own method to receive data

#endif /* USB_CDC_H_ */
