/**-------------------------------------------------------------------------
@file	adc_stm32l4.h

@brief	STM32L4 ADC1 polling implementation

ADC1 of the STM32L4 and STM32L4+ parts, software triggered scans read by
polling. Not supported, Init or OpenChannel fail on them: interrupt, DMA,
continuous mode, differential inputs, per channel oversampling (BurstMode),
FIFO memory, pin pull resistors, a Gain other than 0x100 (no gain stage). One object owns ADC1 until PowerOff or destruction. ADC2 and
ADC3 are not used; the RCC enable and reset bit is common to the three.

Logical channel Chan 0 to 15. PinP.PinNo is the ADC1 input channel:

	0		VREFINT, internal reference, 1.212 V typical
	1 - 4	PC0 - PC3
	5 - 12	PA0 - PA7
	13, 14	PC4, PC5
	15, 16	PB0, PB1
	17		Temperature sensor, 0.76 V typical at 30 C, 2.5 mV/C
	18		VBAT through the internal 1/3 bridge, Data is VBAT

The pin map is the same on every L4 and L4+ part (ST pin data), on the pins
the package has. Each input is used by one logical channel only. The pin is
put in analog mode, with its GPIOx_ASCR switch closed on the parts that have
it (L471, L475, L476, L485, L486). The internal inputs are enabled in
ADC_CCR while their channel is open.

Clock: synchronous, HCLK / 2, or HCLK / 4 when HCLK / 2 is above 80 MHz.
Sampling time: the shortest of 2.5 to 640.5 ADC clocks that covers AcqTime
in usec, and at least the datasheet minimum of the internal inputs: VREFINT
4 us, temperature 5 us, VBAT 12 us. OpenChannel fails when 640.5 clocks are
too short.

Reference: VREF+, supply or external, the caller gives its voltage. Data in
volt is Raw * Voltage / (2^Resolution - 1), times 3 for VBAT.

Resolution 6, 8, 10 or 12 bits: an odd value takes the next one up, a value
under 6 takes 6, 0 takes 12.

Cfg.OvrSample 2 to 256, power of 2, enables the hardware oversampler of the
regular group with the matching right shift: the result is the average at
the configured resolution. 0 or 1 is off.

Every scan starts with a conversion of its first input whose result is not
used: a conversion started more than 1 ms after the previous one or after
the calibration can be wrong (ES0250 2.8.11, workaround applied on every
part). The ADC waits for the data register to be read before the next
conversion of a scan (AUTDLY), so polling does not lose results. The
temperature sensor gives spurious values from spike noise (ES0250 2.8.12):
average many readings, or use oversampling.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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
#ifndef __ADC_STM32L4_H__
#define __ADC_STM32L4_H__

#include <stdint.h>

#include "converters/adc_device.h"

/** @addtogroup Converters
  * @{
  */

#define ADC_STM32L4_MAX_CHAN		16		//!< Logical channels, also the regular sequence length
#define ADC_STM32L4_INPUT_VREFINT	0		//!< Internal reference input
#define ADC_STM32L4_INPUT_TEMP		17		//!< Temperature sensor input
#define ADC_STM32L4_INPUT_VBAT		18		//!< VBAT / 3 input

class AdcSTM32L4 : public AdcDevice {
public:
	AdcSTM32L4();
	~AdcSTM32L4() override;
	bool Init(const AdcCfg_t &Cfg, Timer *pTimer = nullptr, DeviceIntrf *pIntrf = nullptr) override;
	bool OpenChannel(const AdcChanCfg_t *pCfg, int Count) override;
	void CloseChannel(int Chan) override;
	bool StartConversion() override;
	void StopConversion() override;
	bool UpdateData() override;
	int Read(AdcData_t *pBuff, int Len) override;
	bool Read(int Chan, AdcData_t *pBuff) override;
	bool Calibrate() override;
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	void PowerOff() override;
	uint16_t Resolution(uint16_t Val) override;
	using AdcDevice::Resolution;

private:
	AdcSTM32L4(const AdcSTM32L4 &) = delete;
	AdcSTM32L4 &operator=(const AdcSTM32L4 &) = delete;
	bool Owned();
	void Stop();
	void Release(int Input);
	bool Run(const uint8_t *pInput, int Count, uint16_t *pRaw, uint32_t Us);
	bool Scan(const int *pChan, int Count);

	uint8_t vInput[ADC_STM32L4_MAX_CHAN];	//!< ADC1 input + 1 of each logical channel, 0 when closed
	bool vbNew[ADC_STM32L4_MAX_CHAN];		//!< Result not read yet
	uint16_t vRaw[ADC_STM32L4_MAX_CHAN];	//!< Last result
	uint32_t vConvUs[ADC_STM32L4_MAX_CHAN];	//!< Conversion time of each logical channel, usec
	float vRef;								//!< Reference voltage
	uint32_t vAdcClk;						//!< ADC clock, Hz
	uint32_t vOvs;							//!< Oversampling ratio, 1 when off
};

/** @} End of group Converters */

#endif // __ADC_STM32L4_H__
