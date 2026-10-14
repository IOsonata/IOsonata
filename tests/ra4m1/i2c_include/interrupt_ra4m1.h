/**-------------------------------------------------------------------------
@file	interrupt_ra4m1.h

@brief	Host test replacement of interrupt_ra4m1.h for i2c_ra4m1.cpp.

		Same functions, implemented by the ICU model of i2c_test.cpp. The
		event ids are the ones of the RA4M1 header, copied into evt_ids.h by
		run_i2c_validation.py.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#ifndef __INTERRUPT_RA4M1_H__
#define __INTERRUPT_RA4M1_H__

#include <stdbool.h>

#include "ra4m1xxx.h"
#include "evt_ids.h"

typedef void (*Ra4m1IRQHandler_t)(int IntNo, void *pCtx);

IRQn_Type Ra4m1RegisterIntHandler(uint8_t EvtId, int Prio, Ra4m1IRQHandler_t pHandler, void *pCtx);
void Ra4m1UnregisterIntHandler(IRQn_Type IrqNo);
bool Ra4m1AcknowledgeInt(IRQn_Type IrqNo);

#endif
