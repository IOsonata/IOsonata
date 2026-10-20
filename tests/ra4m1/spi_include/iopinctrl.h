/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	Host test replacement of iopinctrl.h for spi_ra4m1.cpp.

		Only the pin calls of the SPI driver. The pins are simulated by
		spi_test.cpp.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include "coredev/iopincfg.h"

// Pin functions of the model. All pins of ports 0 to 9 exist.
static inline bool Ra4m1PinValid(int PortNo, int PinNo)
{
	return (unsigned)PortNo < 10U && (unsigned)PinNo < 16U;
}

#ifdef __cplusplus
extern "C" {
#endif
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type);
int IOPinRead(int PortNo, int PinNo);
void IOPinSet(int PortNo, int PinNo);
void IOPinClear(int PortNo, int PinNo);
#ifdef __cplusplus
}
#endif

#endif
