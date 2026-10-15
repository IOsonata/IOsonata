/**-------------------------------------------------------------------------
@file	uarta_r9a02.h

@brief	R9A02G021 UART device numbers and the UARTA part of the UART driver

UARTCfg_t.DevNo:

	DevNo 0 to 2 : SAU UART0 to UART2 (uart_r9a02.cpp)
	DevNo 3 and 4: UARTA0 and UARTA1 (uarta_r9a02.cpp), 48-pin package only

UARTInit and UARTGetInstance in uart_r9a02.cpp select SAU or UARTA from
DevNo, without application linker flags or direct driver calls.

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
#ifndef __UARTA_R9A02_H__
#define __UARTA_R9A02_H__

#include "coredev/uart.h"

#define R9A02_UART_SAU_CNT			3					//!< DevNo 0 to 2
#define R9A02_UART_UARTA_CNT		2					//!< DevNo 3 and 4
#define R9A02_UART_UARTA_DEVNO(Chan)	(R9A02_UART_SAU_CNT + (Chan))
#define R9A02_UART_DEV_CNT			(R9A02_UART_SAU_CNT + R9A02_UART_UARTA_CNT)

#ifdef __cplusplus
extern "C" {
#endif

/// @brief	UARTInit for DevNo 3 and 4.
///
/// @param	pDev	: UART device data
/// @param	pCfg	: Configuration, DevNo 3 or 4
///
/// @return	false when a setting is not supported
bool R9a02UartaInit(UARTDev_t * const pDev, const UARTCfg_t *pCfg);

/// @brief	UARTGetInstance for DevNo 3 and 4.
///
/// @param	Chan	: UARTA channel, 0 or 1
///
/// @return	UART device data, NULL when the channel is not initialized
UARTDev_t const *R9a02UartaGetInstance(int Chan);

#ifdef __cplusplus
}
#endif

#endif
