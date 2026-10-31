/**-------------------------------------------------------------------------
@file	dac_samd.h

@brief	SAM D11/D21 DAC implementation

The DAC of the SAM D11 and SAM D21, the same block on both: one channel,
10 bits, written by software. DacSamd is the DacDevice of IOsonata, in the
form of the R9A02 DAC driver: OpenChannel, WriteCode, WriteVolt (the
generic DacDevice has only Init).

Output: Vout = Code x reference / 1023, Code 0 to 1023, linear from 0.05 V
to VDDANA - 0.05 V, load of 5 kOhm or more and 100 pF or less. Channel 0
only. OpenChannel turns on, as asked (both by default):
	bPinOut		the output buffer (EOEN): VOUT pin PA02 (function B) and the
				ADC DAC input (ADC_INPUTCTRL_MUXPOS_DAC_Val of adc_samd.h)
	bInternal	the internal output (IOEN) to the analog comparators
The pins and the SYSCTRL VREF outputs are shared with the ADC driver through
analog_samd.h: PA02 is also the ADC AIN0, PA03 the ADC AIN1 and the VREFA of
both. A pin the ADC still uses stays on its analog function when the DAC
releases it, and the reverse.

Reference, Reference() with an AdcRefVolt_t of adc_device.h, also while the
channel is open (the DAC is stopped for the change):

	ADC_REFVOLT_TYPE_INTERNAL	INT1V, the 1.0 V internal reference, Voltage
								as the caller gives it. The bandgap output
								(SYSCTRL VREF.BGOUTEN) is on with it, as ASF.
	ADC_REFVOLT_TYPE_SUPPLY		VDDANA (REFSEL AVCC of the device header),
								Voltage is VDDANA. The default, 3.3 V.
	ADC_REFVOLT_TYPE_EXTERNAL	VREFA on PA03, Pin 0, 1.0 V to VDDANA - 0.6
								V. The pin is on its analog function while
								the channel is open.

Clock: the DAC generic clock on the main clock generator (0), the register
interface on the APBC bus. CTRLB (reference, outputs) is enable protected:
OpenChannel, CloseChannel, Reference and Enable write it with the DAC
disabled, then enable it, when the application has it on, and wait the
start-up time, 10 us (2.85 us above 2.6 V of VDDANA). DATA is written
synchronized: WriteCode returns when the DAC has the code, the output
follows in a conversion time, about 2.85 us, with no indication of the end.
The register sequences run with interrupts disabled: WriteCode can be
called from an interrupt. The voltage pump is on or off by itself (VPD 0).
DATABUF and the start conversion event are not used.

A synchronization that does not end makes the driver invalid; Init sets the
DAC up again.

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
#ifndef __DAC_SAMD_H__
#define __DAC_SAMD_H__

#include <stdint.h>

#include "samdxxx.h"
#include "converters/dac_device.h"
#include "converters/adc_device.h"

/** @addtogroup Converters
  * @{
  */

#define DAC_SAMD_CHAN_CNT		1		//!< Channels
#define DAC_SAMD_CODE_MAX		1023	//!< Highest code, 10 bits

/// DAC of the SAM D11 and SAM D21
class DacSamd : public DacDevice {
public:
	DacSamd();
	~DacSamd() override;

	/**
	 * @brief	Initialization, the channel stays closed.
	 *
	 * @param	Cfg		: DAC_MODE_SINGLE_ENDED
	 * @param	pIntrf	: nullptr, internal converter
	 *
	 * @return	false for another mode, an interface, a DAC owned by another
	 *			object or a clock that does not start
	 */
	bool Init(const DAC_CFG &Cfg, DeviceIntrf * const pIntrf = nullptr) override;

	/**
	 * @brief	Reference of the conversions, see dac_samd.h.
	 *
	 * @param	Ref : Type and Voltage, Pin 0 for EXTERNAL
	 *
	 * @return	false for a type or a voltage not supported
	 */
	bool Reference(const AdcRefVolt_t &Ref);
	float Reference() { return vRefVolt; }

	/**
	 * @brief	Open the channel at code 0, the DAC enabled.
	 *
	 * OpenChannel(0) turns on both outputs, OpenChannel(0, false) the
	 * internal output only, as the R9A02 driver.
	 *
	 * @param	Chan		: 0
	 * @param	bPinOut		: output buffer, VOUT pin PA02 and the ADC DAC input
	 * @param	bInternal	: internal output to the analog comparators
	 *
	 * @return	false if Chan is not valid, no output is asked or the DAC
	 *			did not start
	 */
	bool OpenChannel(int Chan, bool bPinOut = true, bool bInternal = true);
	void CloseChannel(int Chan);

	/// @brief	Output code, Vout = Code x reference / 1023.
	bool WriteCode(int Chan, uint16_t Code);

	/// @brief	Output voltage, the closest code from 0 to the reference.
	///
	/// @return	Voltage of the code set, negative on failure
	float WriteVolt(int Chan, float Volt);

	/// @brief	DAC on again with the last code, the channel open.
	bool Enable() override;

	/// @brief	DAC off, the output pin no longer driven, code kept. A
	///			Reference change keeps it off until Enable.
	void Disable() override;

	/// @brief	Code back to 0.
	void Reset() override;

	/// @brief	Channel closed, DAC and its clocks stopped.
	void PowerOff() override;

private:
	DacSamd(const DacSamd &) = delete;
	DacSamd &operator=(const DacSamd &) = delete;

	bool Owned();
	bool Stop();
	bool Start();
	void Resources(bool bOn);

	float vRefVolt;			//!< Reference voltage
	uint8_t vRefSel;		//!< CTRLB REFSEL value
	uint8_t vOut;			//!< CTRLB EOEN and IOEN of the open channel, 0 when closed
	uint16_t vCode;			//!< Last code
	bool vbOn;				//!< DAC enabled by the application (OpenChannel, Enable)
};

/** @} End of group Converters */

#endif // __DAC_SAMD_H__
