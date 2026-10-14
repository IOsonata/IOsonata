/* LPC1769 IRQ W1C model hooks. MIT License, I-SYST, 2026. */
#pragma once
#include "../startup/system17_shim.h"
void TestIrqClear(uint32_t Value);
void TestIrqSet(uint32_t Value);
