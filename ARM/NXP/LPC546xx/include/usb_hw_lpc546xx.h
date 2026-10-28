/**-------------------------------------------------------------------------
@file usb_hw_lpc546xx.h
@brief Private LPC546xx USB0 full-speed and USB1 high-speed hardware preparation.

Both controllers use the same storage in the USB RAM, one at a time.

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

// USB1 storage: twelve physical endpoints, two command words each in the
// high-speed list layout. EP0 OUT word 1 holds the SETUP buffer address.
// Buffers start on 64-byte boundaries: 64-byte EP0 packets, 512-byte data
// packets. This layout implements control, bulk and interrupt, not ISO.
typedef struct __Lpc546xx_Usb_Hs_Memory {
	volatile uint32_t Command[24];
	uint8_t Reserved[32];
	uint8_t Setup[64];
	uint8_t Packet0[2][64];
	uint8_t Packet[10][512];
} Lpc546xxUsbHsMemory_t;

#ifdef __cplusplus
extern "C" {
#endif

// Starts and calibrates the FRO HF output through ROM if it is off, keeping
// the selected 48/96 MHz rate. Selects a 48 MHz USB clock and enables SOF trim.
// The system clock owner must not change the FRO while USB owns it.
// No host or other device controller may be active.
// Returns storage with DEV_EN, DCON, USB interrupts and NVIC disabled.
// Does not initialize the generic USB core or connect to the host.
LpcUsbFsMemory_t *Lpc546xxUsbHwPrepare(int IntPrio);

// The controller must disconnect, disable DEV_EN and retire all endpoint
// buffers first. Refuses release while hardware may still own a buffer.
// Restores the resources acquired by Prepare; a refusal retains ownership.
bool Lpc546xxUsbHwRelease(void);

// USB1: powers the PHY supply, starts the USB PLL at 48 MHz from the 12 MHz
// system oscillator, which g_McuOsc must give as the core oscillator (crystal
// or TCXO), and selects it as the USB1 clock. The PLL supply stays powered
// after release: the other PLLs use it too.
// No host or other device controller may be active.
// Returns storage with DEV_EN, DCON, USB interrupts and NVIC disabled.
// Does not initialize the generic USB core or connect to the host.
Lpc546xxUsbHsMemory_t *Lpc546xxUsbHsHwPrepare(int IntPrio);

// Same conditions and behavior as Lpc546xxUsbHwRelease, for USB1.
bool Lpc546xxUsbHsHwRelease(void);

#ifdef __cplusplus
}
#endif

#endif
