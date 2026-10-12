/*--------------------------------------------------------------------------
File   : flash17_shim.h

Desc   : Host clock-access hook using the LPC17xx vendor register type

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#pragma once
#include "../startup/system17_shim.h"

LPC_SC_TypeDef *TestSystemRegs(void);
#undef LPC_SC
#define LPC_SC TestSystemRegs()

void TestClockWaitCycle(void);
#undef __NOP
#define __NOP() TestClockWaitCycle()
