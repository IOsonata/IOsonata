/* Test-only USB MMIO write semantics. MIT License, I-SYST, 2026. */
#include <stdint.h>
void TestDeviceWrite(uint32_t Value);
void TestInterruptWrite(uint32_t Value);
void TestSkipWrite(uint32_t Value);
void TestInterruptSet(uint32_t Value);
