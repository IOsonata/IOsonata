/**-------------------------------------------------------------------------
@file	nvm_timeslot_op.h

@brief	Nordic MPSL operation description for internal NVM writes.

Only the Nordic MCU NVM controller and MPSL timeslot implementation use
this type. Generic Nvm, NvmMcuIntrf and other MCU ports do not depend on it.

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
#ifndef __NVM_TIMESLOT_OP_H__
#define __NVM_TIMESLOT_OP_H__

#include <stdint.h>

/// One bounded unit of MCU NVM work to run when MPSL grants a radio timeslot.
/// The controller owns this descriptor until the operation is complete.
typedef struct __Nvm_Timeslot_Op {
	uint32_t (*Step)(void *pCtx);	//!< Run one step; return usec remaining
	uint32_t StepBudgetUs;		//!< Worst-case time needed for one step
	void *pCtx;				//!< Persistent controller operation context
	void (*Done)(void *pCtx, int Res);	//!< Optional deferred completion
} NvmTimeslotOp_t;

#endif // __NVM_TIMESLOT_OP_H__
