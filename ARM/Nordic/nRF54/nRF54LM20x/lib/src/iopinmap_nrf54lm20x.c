/**-------------------------------------------------------------------------
@file	iopinmap_nrf54lm20x.c

@brief	I/O pin map nRF54LM20x (nRF54LM20A/B)

Pin function capability map for the nRF54LM20A/B.

Special function pins:
- XL1/XL2 (32.768 kHz crystal)   : P1.20, P1.21
- NFC1/NFC2 (NFC antenna)        : P1.01, P1.02
- SAADC analog inputs
  AIN0 : P1.00    AIN1 : P1.31    AIN2 : P1.30    AIN3 : P1.29
  AIN4 : P1.06    AIN5 : P1.05    AIN6 : P1.04    AIN7 : P1.03

@author	Hoang Nguyen Hoan
@date	Aug. 24, 2026

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
	// Port 0 : P0.00 - P0.09
	{0,  0, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  1, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  2, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  3, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  4, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  5, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  6, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  7, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  8, 0, 0, IOPIN_MUX_COMMON, 0},
	{0,  9, 0, 0, IOPIN_MUX_COMMON, 0},
	// Port 1 : P1.00 - P1.31
	{1,  0, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN0
	{1,  1, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_NFC, IOPINMUX_NFC},		// NFC1
	{1,  2, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_NFC, IOPINMUX_NFC},		// NFC2
	{1,  3, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN7
	{1,  4, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN6
	{1,  5, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN5
	{1,  6, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN4
	{1,  7, 0, 0, IOPIN_MUX_COMMON, 0},
	{1,  8, 0, 0, IOPIN_MUX_COMMON, 0},
	{1,  9, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 10, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 11, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 12, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 13, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 14, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 15, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 16, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 17, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 18, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 19, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 20, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_XTAL, IOPINMUX_XTAL},		// XL1
	{1, 21, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_XTAL, IOPINMUX_XTAL},		// XL2
	{1, 22, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 23, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 24, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 25, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 26, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 27, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 28, 0, 0, IOPIN_MUX_COMMON, 0},
	{1, 29, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN3
	{1, 30, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN2
	{1, 31, 0, 0, IOPIN_MUX_COMMON | IOPINMUX_ADC, 0},					// AIN1
	// Port 2 : P2.00 - P2.10, high speed pads
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
	// Port 3 : P3.00 - P3.12
	{3,  0, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  1, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  2, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  3, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  4, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  5, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  6, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  7, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  8, 0, 0, IOPIN_MUX_COMMON, 0},
	{3,  9, 0, 0, IOPIN_MUX_COMMON, 0},
	{3, 10, 0, 0, IOPIN_MUX_COMMON, 0},
	{3, 11, 0, 0, IOPIN_MUX_COMMON, 0},
	{3, 12, 0, 0, IOPIN_MUX_COMMON, 0},
};
