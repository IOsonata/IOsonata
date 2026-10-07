/**-------------------------------------------------------------------------
@file	tinyusb_combo_port.h

@brief	Chip glue used by the TinyUSB composite stress benchmark.

main.cpp is shared by all targets. Each target project implements these
functions in its own src folder, next to its tusb_config.h, the same way an
example takes its pin map from the project's board.h.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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
#ifndef __TINYUSB_COMBO_PORT_H__
#define __TINYUSB_COMBO_PORT_H__

#include <stdint.h>

#include "tusb.h"

// DCD scheduling counters read through vendor request 0x5B. Only a port with
// such counters in its DCD sets this to 1 in its tusb_config.h.
#ifndef TINYUSB_COMBO_ISO_DIAG
#define TINYUSB_COMBO_ISO_DIAG			0
#endif

#define TINYUSB_COMBO_DCD_DIAG_COUNT	11U

/**
 * @brief	Power the USB controller and set its interrupt priority.
 *
 * Called once, before tusb_init. The port file also defines the USB
 * interrupt handler, which calls tusb_int_handler(0, true).
 *
 * @return	true on success
 */
bool TinyUsbPortInit(void);

/**
 * @brief	Port work for each main loop pass, before tud_task_ext.
 *
 * Reports USB power changes to TinyUSB on ports that need it.
 */
void TinyUsbPortProcess(void);

/**
 * @brief	Read the 64-bit device identifier used as USB serial number.
 *
 * @param	Id : Receives the two identifier words, in serial string order
 */
void TinyUsbPortDeviceId(uint32_t Id[2]);

#if TINYUSB_COMBO_ISO_DIAG
/**
 * @brief	Read the DCD ISO scheduling counters.
 *
 * @param	Counts : Receives TINYUSB_COMBO_DCD_DIAG_COUNT values
 */
void TinyUsbPortIsoDiag(uint32_t Counts[TINYUSB_COMBO_DCD_DIAG_COUNT]);
#endif

#endif
