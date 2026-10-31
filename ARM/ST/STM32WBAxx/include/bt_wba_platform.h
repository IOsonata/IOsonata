/**-------------------------------------------------------------------------
@file	bt_wba_platform.h

@brief	STM32WBA BLE stack platform layer on IOsonata.

See bt_wba_platform.cpp. The platform functions the stack calls are declared
by the Cube headers. This file declares what the port code calls.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#ifndef __BT_WBA_PLATFORM_H__
#define __BT_WBA_PLATFORM_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Offer again the work that the Bluetooth event queue refused. Called from
/// the same place that retries the other owed work.
void BtWbaPlatformRetry(void);

/// Start HSE32 and select the radio sleep timer clock. Call before BleStack_Init.
void BtWbaPlatformInit(void);

/// Controller time in milliseconds, starting when the first port timer is created.
uint32_t BtWbaMsTick(void);

/// Reserve one periodic controller timer for one-second timeout checks.
/// Call after BleStack_Init. Callback runs in controller interrupt context.
bool BtWbaTickStart(void (*Callback)(const void *));

#ifdef __cplusplus
}
#endif

#endif
