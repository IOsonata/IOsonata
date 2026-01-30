/**-------------------------------------------------------------------------
@file	iopinmap_nrf54l15.c

@brief	I/O pin map nRF54L15

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
	{0,  0, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  1, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  2, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  3, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  4, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  5, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  6, 0, 0, IOPIN_MUX_COMMON, 0},
	{1,  0, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_XTAL, IOPINMUX_XTAL},
	{1,  1, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_XTAL, IOPINMUX_XTAL},
	{1,  2, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_NFC, IOPINMUX_NFC},
	{1,  3, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_NFC, IOPINMUX_NFC},
	{1,  4, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1,  5, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1,  6, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1,  7, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1,  8, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_EXTREF, 0},
	{1,  9, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 10, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 11, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1, 12, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1, 13, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1, 14, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},
	{1, 15, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 16, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  0, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  1, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  2, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  3, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  4, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  5, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  6, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  7, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  8, 0, 0, IOPIN_MUX_COMMON, 0},
	{2,  9, 0, 0, IOPIN_MUX_COMMON, 0},
	{2, 10, 0, 0, IOPIN_MUX_COMMON, 0},
};

static const uint32_t s_IOPinMapPortIdx[GPIO_COUNT + 1] = {	0, IOPIN_P0_MAXCOUNT, IOPIN_P0_MAXCOUNT + IOPIN_P1_MAXCOUNT, IOPIN_MAX_COUNT};

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
	if (PortNo > 2)
	{
		return -1;
	}

	uint8_t idx = s_IOPinMapPortIdx[PortNo] + PinNo;

	if (idx >= s_IOPinMapPortIdx[PortNo + 1])
	{
		return 0;
	}

	return g_IOPinMap[idx].Caps;
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
	if (PortNo > 2)
	{
		return -1;
	}

	uint8_t idx = s_IOPinMapPortIdx[PortNo] + PinNo;

	if (idx >= s_IOPinMapPortIdx[PortNo + 1] || (g_IOPinMap[idx].Conn != 0 && g_IOPinMap[idx].Conn != Fct))
	{
		return -1;
	}

	g_IOPinMap[idx].Conn = Fct;

	return idx;
}

void IOPinRelease(uint8_t PortNo, uint8_t PinNo)
{
	if (PortNo > 2)
	{
		return;
	}

	uint8_t idx = s_IOPinMapPortIdx[PortNo] + PinNo;

	if (idx < s_IOPinMapPortIdx[PortNo + 1])
	{
		g_IOPinMap[idx].Conn = 0;
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





