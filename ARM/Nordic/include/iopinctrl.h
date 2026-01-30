/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control implementation specific

This file must be named iopinctrl.h no matter which target

This is nRF5x implementation


@author	Hoang Nguyen Hoan
@date	June. 2, 2014

@license

MIT License

Copyright (c) 2014 I-SYST inc. All rights reserved.

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>

#include "nrf.h"

#if defined(NRF51)
#include "nrf_gpiote.h"
#else
#include "nrf_peripherals.h"
#endif

#include "coredev/iopincfg.h"


///
#define IOPIN_PORT_MAXCOUNT		GPIO_COUNT

#ifdef NRF52832_XXAA
#define IOPIN_P0_MAXCOUNT		P0_PIN_NUM
#define IOPIN_MAX_COUNT			(P0_PIN_NUM)
#elif defined(NRF52840_XXAA)
#define IOPIN_P0_MAXCOUNT		P0_PIN_NUM
#define IOPIN_P1_MAXCOUNT		P1_PIN_NUM
#define IOPIN_MAX_COUNT			(P0_PIN_NUM + P1_PIN_NUM)
#elif defined(NRF54L15_XXAA)
//
// Package		QFN40	QFN48	QFN52	CSP47
// Max GPIO		24		31		35		32
#define IOPIN_P0_MAXCOUNT		7
#define IOPIN_P1_MAXCOUNT		17
#define IOPIN_P2_MAXCOUNT		11

#define IOPIN_MAX_COUNT			(IOPIN_P0_MAXCOUNT + IOPIN_P1_MAXCOUNT + IOPIN_P2_MAXCOUNT)	// Define max gpio
#endif


#ifdef 	__cplusplus
extern "C" {
#endif

NRF_GPIO_Type *nRFGpioGetReg(int PortNo);

/**
 * @brief	Set gpio pin direction
 *
 * Change pin direction only without changing any other settings
 * for fast switching between In & Out
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 * @Param	Dir     : I/O direction
 */
static inline __attribute__((always_inline)) void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir) {
#if GPIO_COUNT > 1
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	NRF_GPIO_Type *reg;
	if (PortNo & 0x80)
	{
		reg = NRF_P0_NS;
	}
	else
	{
		reg = NRF_P0_S;
	}
#else
	NRF_GPIO_Type *reg = NRF_GPIO;
#endif

	if (reg == NULL || PinNo == -1)
	{
		return;
	}

	reg->PIN_CNF[PinNo] &= ~GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos;
	if (Dir == IOPINDIR_OUTPUT)
	{
		reg->PIN_CNF[PinNo] |= GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos;
		reg->DIRSET = (1 << PinNo);
	}
	else if (Dir == IOPINDIR_INPUT)
	{
		reg->DIRCLR = (1 << PinNo);
	}
}

/**
 * @brief	Read pin state
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 *
 * @return	Pin state 1 or 0
 */
static inline __attribute__((always_inline)) int IOPinRead(int PortNo, int PinNo) {
#if GPIO_COUNT > 1
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);
	return (reg->IN >> PinNo) & 1;
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	if (PortNo & 0x80)
	{
		return (NRF_P0_NS->IN >> PinNo) & 1;
	}
	return (NRF_P0_S->IN >> PinNo) & 1;
#else
	return (NRF_GPIO->IN >> PinNo) & 1;
#endif

}

/**
 * @brief	Set pin to high (1 logic)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo) {
#if GPIO_COUNT > 1
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	reg->OUTSET = (1 << PinNo);
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUTSET = (1 << PinNo);
	}
	else
	{
		NRF_P0_S->OUTSET = (1 << PinNo);
	}
#else
	NRF_GPIO->OUTSET = (1 << PinNo);
#endif
}

/**
 * @brief	Set pin to low (0 logic)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo) {
#if GPIO_COUNT > 1
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	reg->OUTCLR = (1 << PinNo);
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUTCLR = (1 << PinNo);
	}
	else
	{
		NRF_P0_S->OUTCLR = (1 << PinNo);
	}
#else
	NRF_GPIO->OUTCLR = (1 << PinNo);
#endif
}

/**
 * @brief	Toggle pin state (invert pin state)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo) {
#if GPIO_COUNT > 1
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	reg->OUT ^= (1 << PinNo);
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUT ^= (1 << PinNo);
	}
	else
	{
		NRF_P0_S->OUT ^= (1 << PinNo);
	}
#else
	NRF_GPIO->OUT ^= (1 << PinNo);
#endif
}

/**
 * @brief	Read all pins on port
 *
 * @Param 	PortNo	: Port number
 *
 * @return	Bit field pin states
 */
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo) {
#if GPIO_COUNT > 1
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	return reg->IN;
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	if (PortNo & 0x80)
	{
		return NRF_P0_NS->IN;
	}
	return NRF_P0_S->IN;
#else
	return NRF_GPIO->IN;
#endif
}

/**
 * @brief	Write state to all pin on port
 *
 * @Param 	PortNo	: Port number
 * @Param	Data	: Bit field state of all pins on port
 */
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data) {
#if GPIO_COUNT > 1
	NRF_GPIO_Type *reg = nRFGpioGetReg(PortNo);

	reg->OUT = Data;
#elif defined(NRF91_SERIES) || defined(NRF53_SERIES)
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUT = Data;
	}
	else
	{
		NRF_P0_S->OUT = Data;
	}
#else
	NRF_GPIO->OUT = Data;
#endif
}

#ifdef 	__cplusplus
}
#endif

#endif	// __IOPINCTRL_H__
