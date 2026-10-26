/**-------------------------------------------------------------------------
@file	board.h

@brief	EK-RA4M1 USB CDC loopback board definitions.

The EK-RA4M1 device USB connector uses the RA4M1 dedicated USBFS D-/D+ pair.
P407 is the USB_VBUS input and requires the USBFS peripheral function. D-/D+
are owned directly by USBFS and are not configured as ordinary GPIO pins.

@license MIT License
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__


// Supply USBFS with a dedicated 48 MHz clock from HOCO.
#define MCUOSC				{ \
	{ OSC_TYPE_RC, 48000000UL, 0U, 0U }, \
	{ OSC_TYPE_RC, 32768UL, 0U, 0U }, \
	true \
}


#define USB_VBUS_PORT			4
#define USB_VBUS_PIN			7
#define USB_VBUS_PINOP			IOPINOP_FUNC19

#define USB_PINS				{ \
	{USB_VBUS_PORT, USB_VBUS_PIN, USB_VBUS_PINOP, IOPINDIR_INPUT, \
	 IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif
