/**-------------------------------------------------------------------------
@file usb_hw_lpc546xx.h
@brief Private LPC546xx USB0 full-speed hardware preparation.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

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
#ifndef __USB_HW_LPC546XX_H__
#define __USB_HW_LPC546XX_H__

#include <stdbool.h>
#include "usb_mem_lpc_fs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Requires a calibrated, powered FRO HF output at 48 or 96 MHz. Selects
// a 48 MHz USB clock and enables SOF trim. The system clock owner must not
// change the FRO while USB owns it. No host or USB1 controller may be active.
// Returns storage with DEV_EN, DCON, USB interrupts and NVIC disabled.
// Does not initialize the generic USB core or connect to the host.
LpcUsbFsMemory_t *Lpc546xxUsbHwPrepare(int IntPrio);

// The controller must disconnect, disable DEV_EN and retire all endpoint
// buffers first. Refuses release while hardware may still own a buffer.
// Restores the resources acquired by Prepare; a refusal retains ownership.
bool Lpc546xxUsbHwRelease(void);

#ifdef __cplusplus
}
#endif

#endif
