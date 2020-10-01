/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control implementation specific

This file must be named iopinctrl.h no matter which target

This is SAM4E implementation


@author	Hoang Nguyen Hoan
@date	May 29, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#include "sam4e.h"
#include "coredev/iopincfg.h"

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
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	if (Dir == IOPINDIR_OUTPUT)
	{
		reg->PIO_OER = 1 << PinNo;
		reg->PIO_OWER = 1 << PinNo;
		//reg->PIO_PUER = 1 << PinNo;
	}
	else
	{
		reg->PIO_ODR = 1 << PinNo;
		reg->PIO_OWDR = 1 << PinNo;
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
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	return (reg->PIO_PDSR >> PinNo) & 1;
}

/**
 * @brief	Set pin to high (1 logic)
 *
 * Note this chip has output write protect. Output direction must be set first to
 * enable write output, otherwise it would not work
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo) {
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

//	reg->PIO_OWER = (1 << PinNo);
	reg->PIO_SODR = (1 << PinNo);
	//reg->PIO_OWDR = (1 << PinNo);
}

/**
 * @brief	Set pin to low (0 logic)
 *
 * Note this chip has output write protect. Output direction must be set first to
 * enable write output, otherwise it would not work
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo) {
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	//reg->PIO_OWER = (1 << PinNo);
	reg->PIO_CODR = (1 << PinNo);
	//reg->PIO_OWDR = (1 << PinNo);
}

/**
 * @brief	Toggle pin state (invert pin state)
 *
 * Note this chip has output write protect. Output direction must be set first to
 * enable write output, otherwise it would not work
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo) {
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	//reg->PIO_OWER = (1 << PinNo);
	reg->PIO_ODSR ^= (1 << PinNo);
	//reg->PIO_OWDR = (1 << PinNo);
}

/**
 * @brief	Read all pins on port
 *
 * @Param 	PortNo	: Port number
 *
 * @return	Bit field pin states
 */
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo) {
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	return reg->PIO_PDSR;
}

/**
 * @brief	Write state to all pin on port
 *
 * Note this chip has output write protect. Output direction must be set first to
 * enable write output, otherwise it would not work
 *
 * @Param 	PortNo	: Port number
 * @Param	Data	: Bit field state of all pins on port
 */
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data) {
	Sam4ePio *reg = (Sam4ePio *)((uint32_t)SAM4E_PIOA + PortNo * 0x200);

	//reg->PIO_OWER = 0xFFFFFFFF;
	reg->PIO_ODSR = Data;
	//reg->PIO_OWDR = 0xFFFFFFFF;
}


#endif // __IOPINCTRL_H__

