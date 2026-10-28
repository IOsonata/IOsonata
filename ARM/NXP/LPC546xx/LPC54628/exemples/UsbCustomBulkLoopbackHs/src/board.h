/**-------------------------------------------------------------------------
@example	board.h

@brief LPC54628 USB1 high-speed custom bulk loopback configuration.

USB1, the high-speed controller, on the LPCXpresso54628: the 12 MHz crystal of
the board as the core oscillator. The controller reads VBUS on the USB1_VBUS
pin; USB_PINS can give a GPIO instead.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

// The 12 MHz crystal as the core oscillator: the USB PLL makes the 48 MHz
// USB1 clock from it.
#define MCUOSC { \
	{OSC_TYPE_XTAL, 12000000, 20, 0}, \
	{OSC_TYPE_RC, 32000, 0, 0}, true }

// USB1, the high-speed controller
#define USB_DEVNO		1

// No USB_PINS: USB1 runs from UsbEnable and reads VBUS on the USB1_VBUS pin.
// To keep USB1 off until a cable is attached, define USB_PINS with a GPIO
// input wired to the VBUS of the USB1 connector through a divider to about
// 3.3 V (for example 10 kOhm from VBUS and 20 kOhm to ground), such as P0_16
// (Arduino A0):
// #define USB_PINS {{0, 16, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}}

#endif
