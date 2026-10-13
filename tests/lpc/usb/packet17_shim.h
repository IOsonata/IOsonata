/* LPC1769 EP0 FIFO model hooks. MIT License, I-SYST, 2026. */
#pragma once
#include "sie17_shim.h"
void TestPacketSize(uint32_t Value);
void TestPacketWrite(uint32_t Value);
uint32_t TestPacketRead(void);
