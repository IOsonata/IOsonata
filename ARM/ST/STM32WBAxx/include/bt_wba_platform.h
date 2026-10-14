/**-------------------------------------------------------------------------
@file	bt_wba_platform.h

@brief	STM32WBA BLE stack platform layer on IOsonata.

See bt_wba_platform.cpp. The platform functions the stack calls are declared
by the Cube headers. This file declares what the port code calls.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#ifndef __BT_WBA_PLATFORM_H__
#define __BT_WBA_PLATFORM_H__

#ifdef __cplusplus
extern "C" {
#endif

/// Offer again the work that the Bluetooth event queue refused. Called from
/// the same place that retries the other owed work.
void BtWbaPlatformRetry(void);

/// Start HSE32 and select the radio sleep timer clock. Call before BleStack_Init.
void BtWbaPlatformInit(void);

#ifdef __cplusplus
}
#endif

#endif	// __BT_WBA_PLATFORM_H__
