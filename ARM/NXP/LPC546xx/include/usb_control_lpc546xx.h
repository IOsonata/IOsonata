/**-------------------------------------------------------------------------
@file usb_control_lpc546xx.h
@brief Private LPC546xx USB0 full-speed control endpoint preparation.

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
#ifndef __USB_CONTROL_LPC546XX_H__
#define __USB_CONTROL_LPC546XX_H__

#include "usb_ctrlr_lpc.h"

#ifdef __cplusplus
extern "C" {
#endif

// Private port bring-up: prepare EP0 and its event sink. Leaves DCON and NVIC
// disabled. Used by UsbCtrlrStart; applications use the generic USB API.
bool Lpc546xxUsbControlPrepare(int IntPrio, void (*Event)(const UsbCtrlrEvt_t *));
// Disconnect and retire DMA before returning clocks/storage. Failure retains
// ownership; the caller may retry after the controller completes the skip.
bool Lpc546xxUsbControlRelease(void);
void USB0_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif
