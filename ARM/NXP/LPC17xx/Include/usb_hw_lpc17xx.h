/**-------------------------------------------------------------------------
@file	usb_hw_lpc17xx.h

@brief	Private LPC17xx USB clock preparation; no endpoint or pin setup.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef USB_HW_LPC17XX_H
#define USB_HW_LPC17XX_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// MainHz is the board's external oscillator frequency. The caller must keep
// that oscillator, the selected PLL and core clock stable until release.
// Requires exclusive ownership: an already-powered USB block is refused.
bool Lpc17UsbHwPrepare(uint32_t MainHz);
// Refuses release while endpoint, interrupt or DMA state remains enabled.
bool Lpc17UsbHwRelease(void);
// Private SIE operations for commands with zero or one data byte. Read output
// is unchanged on failure. Entire command/data sequences preserve PRIMASK.
// A timeout blocks further commands until release and successful preparation;
// the caller must treat controller state as unknown and reinitialize it.
// The USB ISR must leave CCEMPTY/CDFULL to these polling operations.
bool Lpc17UsbSieCommand(uint8_t Command);
bool Lpc17UsbSieWrite(uint8_t Command, uint8_t Data);
bool Lpc17UsbSieRead(uint8_t Command, uint8_t *pData);
#ifdef __cplusplus
}
#endif
#endif
