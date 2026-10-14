/**-------------------------------------------------------------------------
@file	bt_wba_stack.h

@brief	STM32WBA BLE host stack headers with C linkage.

The headers of the STM32CubeWBA BLE stack (middleware release 1.9.0) declare
the aci_*, hci_* and BleStack_* functions without an extern "C" guard. The
stack library is C, so the C++ port sources include the stack through this file
and call the functions by their C names.

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
#ifndef __BT_WBA_STACK_H__
#define __BT_WBA_STACK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ble_types.h"
#include "ble_std.h"
#include "ble_defs.h"
#include "ble_core.h"
#include "ble_events.h"
#include "ble_bufsize.h"
#include "blestack.h"
#include "bpka.h"

#ifdef __cplusplus
}
#endif

#endif	// __BT_WBA_STACK_H__
