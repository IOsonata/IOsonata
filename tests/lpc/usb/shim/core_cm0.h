/* Host core only. Peripheral declarations remain the vendor's LPC11Uxx.h. */
#pragma once
#include "../../uart/shim/core_cm0.h"
void TestUsbDsb(void);
#define __DSB() TestUsbDsb()
#define __DMB() ((void)0)
