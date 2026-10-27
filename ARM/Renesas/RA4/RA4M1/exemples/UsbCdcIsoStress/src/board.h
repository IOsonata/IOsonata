/**-------------------------------------------------------------------------
@file	board.h

@brief	EK-RA4M1 USB CDC and ISO stress board definitions.

The EK-RA4M1 device USB connector uses the RA4M1 dedicated USBFS D-/D+ pair.
P407 is the USB_VBUS input and requires the USBFS peripheral function. D-/D+
are owned directly by USBFS and are not configured as ordinary GPIO pins.

@license MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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
// RA4M1 IOPinConfig uses raw PSEL: FUNC18 expands to 19 (USBFS).
#define USB_VBUS_PINOP			IOPINOP_FUNC18

#define USB_PINS				{ \
	{USB_VBUS_PORT, USB_VBUS_PIN, USB_VBUS_PINOP, IOPINDIR_INPUT, \
	 IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif
