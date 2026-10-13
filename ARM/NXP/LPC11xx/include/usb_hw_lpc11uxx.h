/**-------------------------------------------------------------------------
@file usb_hw_lpc11uxx.h
@brief Private LPC11Uxx USB hardware preparation; not an application API.

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
#ifndef __USB_HW_LPC11UXX_H__
#define __USB_HW_LPC11UXX_H__

#include <stdbool.h>
#include <stdint.h>

// UM10462 figure 20. Two words per physical endpoint, including the
// EP0 SETUP address at word 1 and the reserved EP0 IN word 3.
typedef struct __Lpc11_Usb_Memory {
	volatile uint32_t Command[20];
	uint8_t Reserved[48];
	uint8_t Setup[64];
	uint8_t Packet[10][64];
} Lpc11UsbMemory_t;

#ifdef __cplusplus
extern "C" {
#endif

// Requires SystemInit/SystemCoreClockSelect and a stopped controller. Leaves
// DEV_EN/DCON and all interrupts off. Returns NULL on failure. Caller must
// serialize access and must not change clocks while this storage is owned.
// This first storage layout supports 64-byte packets, not ISO payloads.
Lpc11UsbMemory_t *Lpc11UsbHwPrepare(int IntPrio);

// Disconnect and retire active single-buffer endpoints before releasing the
// memory. A timeout returns false and retains ownership and clocks: never
// recycle a buffer which the USB engine may still access. Retry after the
// hardware has completed EPSKIP, or reset the MCU.
bool Lpc11UsbHwRelease(void);

// Acknowledge only requested DEVCMDSTAT W1C flags. Preserve command bits and
// never generate remote wakeup as a side effect. Requires prepared hardware.
void Lpc11UsbHwAckStatus(uint32_t Flags);

// Enable EP0 and bus-event interrupt sources after hardware preparation.
// Leaves DCON and NVIC off. The controller lifecycle must configure board
// pins and connect only after the generic core and classes are initialized.
bool Lpc11UsbEp0Enable(void);
void USB_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif
