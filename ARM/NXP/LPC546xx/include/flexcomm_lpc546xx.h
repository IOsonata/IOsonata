/**-------------------------------------------------------------------------
@file	flexcomm_lpc546xx.h

@brief	LPC546xx Flexcomm interface selection

		Each Flexcomm is one USART, SPI or I2C at a time. The UART, SPI and
		I2C drivers use the Flexcomm number as their DevNo. These functions
		do the part common to all three: bus clock, reset, function select,
		function clock and interrupt number. The Flexcomm interrupt vectors
		dispatch through g_SharedIntrf, set with SharedIntrfSetIrqHandler.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#ifndef __FLEXCOMM_LPC546XX_H__
#define __FLEXCOMM_LPC546XX_H__

#include <stdint.h>
#include <stdbool.h>

#include "LPC546xx.h"

/// Flexcomm function, PSELID PERSEL value
typedef enum {
	LPC546XX_FLEXCOMM_USART = 1,
	LPC546XX_FLEXCOMM_SPI = 2,
	LPC546XX_FLEXCOMM_I2C = 3,
} LPC546XX_FLEXCOMM_FUNC;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Number of Flexcomm interfaces.
 */
int Lpc546xxFlexcommCount(void);

/**
 * @brief	Select the function of a Flexcomm.
 *
 * Turns the bus clock on, resets the Flexcomm, selects the function and the
 * 12 MHz FRO as function clock. All registers of the function are at their
 * reset value after this call.
 *
 * @param	FcNo	: Flexcomm number
 * @param	Func	: Function
 *
 * @return	Function clock frequency in Hz.
 * 			0 - no such Flexcomm, function not present on it, or the
 * 			Flexcomm is locked to another function.
 */
uint32_t Lpc546xxFlexcommSelect(int FcNo, LPC546XX_FLEXCOMM_FUNC Func);

/**
 * @brief	Turn the Flexcomm bus clock on or off.
 *
 * Registers keep their content while the clock is off.
 *
 * @param	FcNo	: Flexcomm number
 * @param	bOn		: true to turn on
 */
void Lpc546xxFlexcommClock(int FcNo, bool bOn);

/**
 * @brief	Get the Flexcomm interrupt number.
 *
 * @param	FcNo	: Flexcomm number, valid
 *
 * @return	Interrupt number
 */
IRQn_Type Lpc546xxFlexcommIrqNo(int FcNo);

#ifdef __cplusplus
}
#endif

#endif
