/**-------------------------------------------------------------------------
@file	shared_intrf.h

@brief	Shared device interface

This is a generic implementation for shared device interface.
Each MCU must implement it's own shared interface interrupt handler.

@author	Nguyen Hoan Hoang
@date	Mar. 12, 2020

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

#ifndef __SHARED_INTRF_H__
#define __SHARED_INTRF_H__

#include "device_intrf.h"

typedef void (*IrqHandler_t)(int DevNo, DevIntrf_t *pDev);

#pragma pack(push, 4)

typedef struct {
	DevIntrf_t *pIntrf;			//!< Device interface
	IrqHandler_t Handler ;		//!< Device interface interrupt handler
} SharedIntrf_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

void SharedIntrfSetIrqHandler(int DevNo, DevIntrf_t * const pDev, IrqHandler_t Handler);

#ifdef __cplusplus
}
#endif

extern const int g_SharedIntrfMaxCnt;	// Instance must be declared in MCU implementation
extern SharedIntrf_t g_SharedIntrf[];	// Instance must be declared in MCU implementation

#endif // __SHARED_INTRF_H__
