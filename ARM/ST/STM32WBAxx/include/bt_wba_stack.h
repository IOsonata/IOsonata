/**-------------------------------------------------------------------------
@file	bt_wba_stack.h

@brief	STM32WBA BLE host stack headers with C linkage.

The headers of the STM32CubeWBA BLE stack (middleware release 1.9.0) declare
the aci_*, hci_* and BleStack_* functions without an extern "C" guard. The
stack library is C, so the C++ port sources include the stack through this file
and call the functions by their C names.

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
#include "bleplat.h"
#include "pka_ctrl.h"
#include "ll_intf_cmn.h"

#ifdef __cplusplus
}
#endif

#endif
