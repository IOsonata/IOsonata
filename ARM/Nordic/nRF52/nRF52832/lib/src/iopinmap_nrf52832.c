/**-------------------------------------------------------------------------
@file	iopinmap_nrf52832.c

@brief	I/O pin map nRF52832

@author	Hoang Nguyen Hoan
@date	Jan. 30, 2036

@license

MIT

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
#include "iopinctrl.h"


#define IOPIN_MUX_COMMON	(IOPINMUX_GPIO | IOPINMUX_CLOCK_OUT | IOPINMUX_COMP_IN | \
							 IOPINMUX_I2C | IOPINMUX_I2S | IOPINMUX_PDM | IOPINMUX_PWM | \
							 IOPINMUX_QDEC | IOPINMUX_SPI | IOPINMUX_TIMER_CAP | IOPINMUX_UART)

__attribute__((weak)) PinMapEntry_t g_IOPinMap[IOPIN_MAX_COUNT] = {
	{0,  0, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_XTAL, IOPINMUX_XTAL},
	{0,  1, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_XTAL, IOPINMUX_XTAL},
	{0,  2, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC | IOPINMUX_EXTREF, 0},
	{0,  3, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC | IOPINMUX_EXTREF, 0},
	{0,  4, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC | IOPINMUX_EXTREF, 0},
	{0,  5, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC | IOPINMUX_EXTREF, 0},
	{0,  6, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  7, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  8, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  9, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_NFC, IOPINMUX_NFC},
	{0, 10, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_NFC, IOPINMUX_NFC},
	{0, 11, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 12, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 13, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 14, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 15, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 16, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 17, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 18, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 19, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 20, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 21, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_RESET, IOPINMUX_RESET},
	{0, 22, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 23, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 24, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 25, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 26, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 27, 0, 0, IOPIN_MUX_COMMON, 0},
	{0, 28, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC| IOPINMUX_EXTREF, 0},
	{0, 29, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC| IOPINMUX_EXTREF, 0},
	{0, 30, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC| IOPINMUX_EXTREF, 0},
	{0, 31, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC| IOPINMUX_EXTREF, 0}
};
#if 0
/**
 * @brief	Get pin capability
 *
 * This function return the capability (function) of a pin.
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @param	PinNo  	: Pin number (up to 32 pins)
 *
 * @return	PINMUX - orable bit field of the capability list.
 */
IOPINMUX IOPinGetCaps(uint8_t PortNo, uint8_t PinNo)
{
	if (PortNo > 0 || PinNo > 31)
	{
		return 0;
	}

	return g_IOPinMap[PinNo].Caps;
}

/**
 * @brief	Allocate pin usage
 *
 * Allocate pin for assignment to a function
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @param	PinNo  	: Pin number (up to 32 pins)
 * @param 	Fct		: Pin mux selected
 * @param	Id		: Pin identifier specific to each function
 *
 * @return	-1 - Pin in use by other function
 * 			index value in the pin assignment array.
 */
int IOPinAlloc(uint8_t PortNo, uint8_t PinNo, IOPINMUX Fct, uint8_t Id)
{
	if (PortNo > 0 || PinNo > 31 || (g_IOPinMap[PinNo].Conn != 0 && g_IOPinMap[PinNo].Conn != Fct))
	{
		return -1;
	}

	g_IOPinMap[PinNo].Conn = Fct;

	return PinNo;
}

void IOPinRelease(uint8_t PortNo, uint8_t PinNo)
{
	if (PortNo == 0 && PinNo < 32)
	{
		g_IOPinMap[PinNo].Conn = 0;
	}
}

int IOPinFind(IOPINMUX Fct)
{
	for (int i = 0; i < IOPIN_MAX_COUNT; i++)
	{
		if (g_IOPinMap[i].Caps & Fct)
		{
			return i;
		}
	}

	return -1;
}
#endif




