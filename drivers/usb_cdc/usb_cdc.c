/*
 * usb_cdc.c
 *
 *  Created on: Aug 12, 2012
 *      Author: jenswilly
 */

#include <string.h>
#include "LPC11Uxx.h"
#include "power_api.h"
#include "mw_usbd_rom_api.h"

extern uint8_t VCOM_DeviceDescriptor[];
extern uint8_t VCOM_StringDescriptor[];
extern uint8_t VCOM_ConfigDescriptor[];

USBD_API_T* pUsbApi;

/* VCOM defines */
#define VCOM_BUFFERS    4
#define VCOM_BUF_EMPTY_INDEX  (0xFF)
#define VCOM_BUF_FREE   0
#define VCOM_BUF_ALLOC  1
#define VCOM_BUF_USBTXQ  2
#define VCOM_BUF_UARTTXQ  3
#define VCOM_BUF_ALLOCU  4

struct VCOM_DATA;
typedef void (*VCOM_SEND_T) (struct VCOM_DATA* pVcom);

typedef struct VCOM_DATA {
  USBD_HANDLE_T hUsb;
  USBD_HANDLE_T hCdc;
  uint8_t* rxBuf;					// Buffer for data received on USB
  uint8_t* txBuf;					// Buffer for data received on UART to be sent on USB
  volatile uint8_t ser_pos;
  volatile uint16_t rxlen;			// How many bytes have been received on USB
  volatile uint16_t txlen;			// How many bytes have been received on UART
  VCOM_SEND_T send_fn;				// Function pointer to "send data" function
  volatile uint32_t sof_counter;
  volatile uint32_t last_ser_rx;
  volatile uint16_t break_time;
  volatile uint16_t usbrx_pend;		// If set, there is data in USB buffer that has not yet been shifted into rxBuf
} VCOM_DATA_T;

// Global data structure
VCOM_DATA_T g_vCOM;
uint8_t rxbuf[USB_HS_MAX_BULK_PACKET] __attribute__((section("USBRAM")));
uint8_t txbuf[USB_HS_MAX_BULK_PACKET] __attribute__((section("USBRAM")));

// Private function prototypes
ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event);
ErrorCode_t VCOM_SendBreak (USBD_HANDLE_T hCDC, uint16_t mstime);
ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event);

void USB_pin_clk_init(void);

/* Initialize USB CDC.
 */
ErrorCode_t USB_CDC_init(void)
{
	  USBD_API_INIT_PARAM_T usb_param;
	  USBD_CDC_INIT_PARAM_T cdc_param;
	  USB_CORE_DESCS_T desc;
	  USBD_HANDLE_T hUsb, hCdc;
	  ErrorCode_t ret = LPC_OK;
	  uint32_t ep_indx;

	  /* get USB API table pointer */
	  pUsbApi = (USBD_API_T*)((*(ROM **)(0x1FFF1FF8))->pUSBD);

	  /* enable clocks and pinmux for usb0 */
	  USB_pin_clk_init();

	  /* initialize call back structures */
	  memset((void*)&usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	  usb_param.usb_reg_base = LPC_USB_BASE;
//	  usb_param.mem_base = 0x10001000;	// Top of RAM is at 0x10001800 for LPC11U2x/301 devices
//	  usb_param.mem_size = 0x0800;		// 6 kb for LPC11U2x/301 devices
	  usb_param.mem_base = 0x20004800;
	  usb_param.mem_size = 0x0800;
	  usb_param.max_num_ep = 3;

	  /* init CDC params */
	  memset((void*)&cdc_param, 0, sizeof(USBD_CDC_INIT_PARAM_T));
	  memset((void*)&g_vCOM, 0, sizeof(VCOM_DATA_T));

	  /* user defined functions */
	#if defined(UART_BRIDGE)
	  cdc_param.SetLineCode = VCOM_SetLineCode;
	  usb_param.USB_SOF_Event = VCOM_sof_event;
	#endif
	  cdc_param.SendBreak = VCOM_SendBreak;

	  /* Initialize Descriptor pointers */
	  memset((void*)&desc, 0, sizeof(USB_CORE_DESCS_T));
	  desc.device_desc = (uint8_t *)&VCOM_DeviceDescriptor[0];
	  desc.string_desc = (uint8_t *)&VCOM_StringDescriptor[0];
	  desc.full_speed_desc = (uint8_t *)&VCOM_ConfigDescriptor[0];
	  desc.high_speed_desc = (uint8_t *)&VCOM_ConfigDescriptor[0];

	  /* USB Initialization */
	  ret = pUsbApi->hw->Init(&hUsb, &desc, &usb_param);
	  if( ret != LPC_OK )
		  return ret;

	// init CDC params
	cdc_param.mem_base = usb_param.mem_base;
	cdc_param.mem_size = usb_param.mem_size;
	cdc_param.cif_intf_desc = (uint8_t *)&VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE];
	cdc_param.dif_intf_desc = (uint8_t *)&VCOM_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE + \
							   USB_INTERFACE_DESC_SIZE + 0x0013 + USB_ENDPOINT_DESC_SIZE ];

	ret = pUsbApi->cdc->init(hUsb, &cdc_param, &hCdc);
	if( ret != LPC_OK )
		return ret;

	/* store USB handle */
	g_vCOM.hUsb = hUsb;
	g_vCOM.hCdc = hCdc;
//	g_vCOM.send_fn = VCOM_usb_send;

	/* allocate transfer buffers */
//	g_vCOM.rxBuf = (uint8_t*)(cdc_param.mem_base + (0 * USB_HS_MAX_BULK_PACKET));
//	g_vCOM.txBuf = (uint8_t*)(cdc_param.mem_base + (1 * USB_HS_MAX_BULK_PACKET));
	g_vCOM.rxBuf = rxbuf;
	g_vCOM.txBuf = txbuf;
	cdc_param.mem_size -= (4 * USB_HS_MAX_BULK_PACKET);

	/* register endpoint interrupt handler */
	ep_indx = (((USB_CDC_EP_BULK_IN & 0x0F) << 1) + 1);
	ret = pUsbApi->core->RegisterEpHandler( hUsb, ep_indx, VCOM_bulk_in_hdlr, &g_vCOM );
	if( ret != LPC_OK )
		return ret;

	/* register endpoint interrupt handler */
	ep_indx = ((USB_CDC_EP_BULK_OUT & 0x0F) << 1);
	ret = pUsbApi->core->RegisterEpHandler( hUsb, ep_indx, VCOM_bulk_out_hdlr, &g_vCOM );
	if( ret != LPC_OK )
		return ret;

	// Enable USB interrupts
	NVIC_EnableIRQ(USB_IRQn);

	/* USB Connect */
	pUsbApi->hw->Connect( hUsb, 1 );

	return LPC_OK;
}

/* Configures clock and pins for USB connection.
 * PIO0_3 is used as VBUS and PIO0_6 is used as SoftConnect.
 */
void USB_pin_clk_init(void)
{
  /* Enable AHB clock to the GPIO domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

  /* Enable AHB clock to the USB block and USB RAM. */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((0x1<<14)|(0x1<<27));

  /* Pull-down is needed, or internally, VBUS will be floating. This is to
  address the wrong status in VBUSDebouncing bit in CmdStatus register. It
  happens on the NXP Validation Board only that a wrong ESD protection chip is used. */
  LPC_IOCON->PIO0_3   &= ~0x1F;
  LPC_IOCON->PIO0_3   |= (0x01<<0);			/* Secondary function VBUS */
  LPC_IOCON->PIO0_6   &= ~0x07;
  LPC_IOCON->PIO0_6   |= (0x01<<0);			/* Secondary function SoftConn */

  return;
}

/* Sends data on USB CDC.
 */
void USB_CDC_send(uint8_t *bufferPtr, uint32_t length)
{
	VCOM_DATA_T* pVcom = &g_vCOM;
	pUsbApi->hw->WriteEP( pVcom->hUsb, USB_CDC_EP_BULK_IN, bufferPtr, length );
}

__attribute__((weak))void USB_CDC_receive( uint8_t *bufferPtr, uint32_t length )
{
	// Data received Ð make your own method that actually does something. Oh, and don't make it __weak__ too..
}

/* Sends data in pVcom->txBuf on USB.
 * Length is pVcom->txlen.
 */
#if 0
void VCOM_usb_send(VCOM_DATA_T* pVcom)
{
  /* data received send it back */
  pVcom->txlen -= pUsbApi->hw->WriteEP (pVcom->hUsb, USB_CDC_EP_BULK_IN, pVcom->txBuf, pVcom->txlen);
}
#endif

ErrorCode_t VCOM_SendBreak (USBD_HANDLE_T hCDC, uint16_t mstime)
{
  VCOM_DATA_T* pVcom = &g_vCOM;
  uint8_t lcr = LPC_USART->LCR;

  if ( mstime) {
    lcr |= (1 << 6);
  } else {
    lcr &= ~(1 << 6);
  }

  pVcom->break_time = mstime;
  LPC_USART->LCR = lcr;

  return LPC_OK;
}

/* USB handler for 'send USB data'.
 * Not used since data is written to USB endpoint immediately in VCOM_usb_send().
 */
ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event)
{
	/*
	VCOM_DATA_T* pVcom = (VCOM_DATA_T*) data;
	if (event == USB_EVT_IN)
	{
		// If we have data in the pxbuf, send it
		if( pVcom->txlen )
		{
			// Buffer non-empty: send it
			pUsbApi->hw->WriteEP( pVcom->hUsb, USB_CDC_EP_BULK_IN, pVcom->txBuf, pVcom->txlen );

			// Mark buffer as empty and ready for more data
			pVcom->txlen = 0;
		}
	}
	*/
	return LPC_OK;
}

/* USB handler for 'data received'.
 * If pVcom->rxBuf is empty, read from USB endpoint into pVcom->rxBuf immediately.
 * Otherwise, set flag to indicate we have unread data in the USB buffer for later reading.
 */
ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event)
{
  VCOM_DATA_T* pVcom = (VCOM_DATA_T*) data;

  switch (event) {
    case USB_EVT_OUT:

      if( pVcom->rxlen == 0 ) {
    	  // Data received: read from endpoint and call user's receive function
        pVcom->rxlen = pUsbApi->hw->ReadEP(hUsb, USB_CDC_EP_BULK_OUT, pVcom->rxBuf);
        USB_CDC_receive( pVcom->rxBuf, pVcom->rxlen );
        pVcom->rxlen = 0;
      } else {
        /* indicate bridge write buffer pending in USB buf */
        pVcom->usbrx_pend = 1;
      }
      break;
    default:
      break;
  }
  return LPC_OK;
}

/* USB interrupt.
 */
void USB_IRQHandler(void)
{
	// Forward request to appropriate method
	pUsbApi->hw->ISR(g_vCOM.hUsb);
}
