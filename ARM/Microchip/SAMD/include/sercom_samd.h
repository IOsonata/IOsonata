/**-------------------------------------------------------------------------
@file	sercom_samd.h

@brief	SAM D11/D21 SERCOM common functions

The UART, I2C and SPI drivers share the SERCOM instances. DevNo is the
SERCOM number. The interrupt of each SERCOM goes to the driver registered
with SharedIntrfSetIrqHandler.

A SERCOM pin is selected by its PinOp: IOPINOP_PERIPHC for the SERCOM
function C, IOPINOP_PERIPHD for the SERCOM-ALT function D. SamdSercomPad
returns the pad of the pin on that function, which sets the pad routing of
the driver.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#ifndef __SERCOM_SAMD_H__
#define __SERCOM_SAMD_H__

#include <stdbool.h>

#include "samdxxx.h"
#include "coredev/iopincfg.h"
#include "device_intrf.h"

#ifdef SERCOM5_REGS
#define SAMD_SERCOM_CNT			6
#else
#define SAMD_SERCOM_CNT			3
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	SERCOM registers.
 *
 * @param	DevNo : SERCOM number
 *
 * @return	Register block, NULL for a bad number
 */
sercom_registers_t *SamdSercomGetReg(int DevNo);

/**
 * @brief	Bus clock and core clock of a SERCOM, from the main generator.
 *
 * @param	DevNo : SERCOM number
 *
 * @return	true - clocks running
 */
bool SamdSercomClockEnable(int DevNo);

/**
 * @brief	Stop the bus clock and the core clock of a SERCOM.
 *
 * @param	DevNo : SERCOM number
 */
void SamdSercomClockDisable(int DevNo);

/**
 * @brief	Pad of a pin on a SERCOM.
 *
 * @param	DevNo : SERCOM number
 * @param	pPin  : Pin with its PinOp, IOPINOP_PERIPHC or IOPINOP_PERIPHD
 *
 * @return	Pad 0 to 3, -1 if the pin is not on that SERCOM with that PinOp
 */
int SamdSercomPad(int DevNo, const IOPinCfg_t *pPin);

/**
 * @brief	Pin usable by a SERCOM in I2C mode.
 *
 * Only some pins have the I2C pad drive: PA14, PA15, PA22, PA23 on SAM D11,
 * PA08, PA09, PA12, PA13, PA16, PA17, PA22, PA23, PB12, PB13, PB16, PB17,
 * PB30, PB31 on SAM D21J.
 *
 * @param	pPin : Pin
 *
 * @return	true - I2C pin
 */
bool SamdSercomI2cPin(const IOPinCfg_t *pPin);

/**
 * @brief	SERCOM in use by another interface.
 *
 * The SERCOM is enabled and its interrupt is registered to another
 * interface with SharedIntrfSetIrqHandler.
 *
 * @param	DevNo	: SERCOM number
 * @param	pIntrf	: Interface to be initialized on it
 *
 * @return	true - in use by another interface, or bad number
 */
bool SamdSercomOwned(int DevNo, const DevIntrf_t *pIntrf);

static inline IRQn_Type SamdSercomIrq(int DevNo) { return (IRQn_Type)(SERCOM0_IRQn + DevNo); }

#ifdef __cplusplus
}
#endif

#endif // __SERCOM_SAMD_H__
