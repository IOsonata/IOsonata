/**-------------------------------------------------------------------------
@file	bsdlib_os_bare.h

@brief	bare metal port for nrf bsdlib


@author	Hoang Nguyen Hoan
@date	Sep. 26, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#ifndef __BSDLIB_OS_BARE_H__
#define __BSDLIB_OS_BARE_H__

#include <stdint.h>

#include "coredev/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TODO: add documentation in this file */

#define BSD_OS_NO_WAIT  0 /**< Zero time-out. */
#define BSD_OS_FOREVER -1 /**< Infinite time-out. */

/**
 * @brief	Initialize bsdlib
 *
 *
 */
int bsdlid_init(UARTDEV * const pUartDev, bool Trace);
void bsd_recoverable_error_handler(uint32_t error);

#ifdef __cplusplus
}
#endif

#endif // __BSDLIB_OS_BARE_H__

