/**-------------------------------------------------------------------------
@file	analog_samd.h

@brief	SAM D11/D21 analog pins and voltage references shared by the ADC and
		the DAC, private definitions

The ADC and the DAC drivers can use the same pins (PA02: ADC AIN0 and DAC
VOUT, PA03: ADC AIN1 and VREFA of both) and the SYSCTRL VREF outputs
(BGOUTEN: the bandgap to the ADC, and to the INT1V reference as ASF does;
TSEN: the temperature sensor). Each driver says what it uses; a pin goes back
to its reset state, and a VREF output off, when no driver uses it any more.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#ifndef __ANALOG_SAMD_H__
#define __ANALOG_SAMD_H__

#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"

// Users of the shared analog resources
#define SAMD_ANALOG_ADC			0
#define SAMD_ANALOG_DAC			1
#define SAMD_ANALOG_USER_CNT	2

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Take or release an analog pin.
 *
 * Taken, the pin is on function B, the analog function, without input
 * buffer and pull. Released by its last user, it goes back to its reset
 * state (IOPinDisable).
 *
 * @param	User	: SAMD_ANALOG_ADC or SAMD_ANALOG_DAC
 * @param	Pin		: port x 32 + pin, the PIN_ number of the device header
 * @param	bOn		: true to take, false to release
 */
void SamdAnalogPin(int User, uint32_t Pin, bool bOn);

/**
 * @brief	SYSCTRL VREF outputs a user needs.
 *
 * VREF.TSEN and VREF.BGOUTEN are on while one user needs them.
 *
 * @param	User	: SAMD_ANALOG_ADC or SAMD_ANALOG_DAC
 * @param	Mask	: SYSCTRL_VREF_TSEN_Msk and SYSCTRL_VREF_BGOUTEN_Msk this
 *					  user needs now, 0 for none
 */
void SamdAnalogVref(int User, uint32_t Mask);

#ifdef __cplusplus
}
#endif

#endif // __ANALOG_SAMD_H__
