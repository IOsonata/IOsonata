/*--------------------------------------------------------------------------
File   : system17_shim.h

Desc   : Host-only SCB and barrier hooks for LPC17xx startup tests

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#pragma once
#include "LPC17xx.h"

// Only the field used by SystemInit is modeled. This is not a register map.
typedef struct {
	volatile uint32_t VTOR;
} TestScb_t;

extern TestScb_t g_TestScb;
void TestDSB(void);
void TestISB(void);

#define SCB (&g_TestScb)
#define __DSB() TestDSB()
#define __ISB() TestISB()

uint32_t TestGetPrimask(void);
void TestDisableIRQ(void);
void TestSetPrimask(uint32_t Mask);
#define __get_PRIMASK() TestGetPrimask()
#define __disable_irq() TestDisableIRQ()
#define __set_PRIMASK(Mask) TestSetPrimask(Mask)

#define __NOP() ((void)0)
