/**-------------------------------------------------------------------------
@file	adc_samd.h

@brief	SAM D11/D21 ADC polling implementation

The ADC of the SAM D11 and SAM D21, the same block on both: software
triggered single conversions read by polling, one input at a time, each with
its own reference, gain, sampling time and single ended or differential
mode. Not supported, Init or OpenChannel fail on them: interrupt, DMA,
events, continuous mode, per channel oversampling (BurstMode), FIFO memory,
pin pull resistors. One object owns the ADC until PowerOff or destruction.

Logical channel Chan 0 to 15. PinP.PinNo is the MUXPOS input of the device
header (ADC_INPUTCTRL_MUXPOS_..._Val):

	AIN		SAM D21		SAM D11		Internal inputs
	0		PA02		PA02		0x18	TEMP, temperature sensor
	1		PA03		PA03		0x19	BANDGAP, bandgap reference
	2		PB08		PA04		0x1A	SCALEDCOREVCC, VDDCORE / 4
	3		PB09		PA05		0x1B	SCALEDIOVCC, VDDIO / 4
	4		PA04		PA06		0x1C	DAC, DAC output
	5		PA05		PA07
	6		PA06		PA14
	7		PA07		PA15
	8		PB00		PA10
	9		PB01		PA11
	10 - 15	PB02 - PB07
	16 - 19	PA08 - PA11

The SAM D21 inputs on PA08 to PA11 are supplied by VDDIO. Data of
SCALEDCOREVCC and SCALEDIOVCC is VDDCORE and VDDIO. The DAC input reads the
DAC output buffer (DacSamd of dac_samd.h opened with bPinOut, which also
drives PA02). The temperature sensor (SYSCTRL VREF.TSEN) is on while its
channel is open, the bandgap output (VREF.BGOUTEN) while a channel converts
the bandgap or uses the INT1V reference (as ASF does); the datasheet gives
no start-up time for them, the driver does not wait. The pins and these
outputs are shared with the DAC driver through analog_samd.h: a pin or an
output the DAC still uses stays on. An input is the positive input of one logical channel only;
a pin can also be the negative input of differential channels.

Differential: Type ADC_CHAN_TYPE_DIFFERENTIAL, PinP an AIN input, PinN.PinNo
AIN 0 to 7 (MUXNEG). The result is signed, Data from -full scale to +full
scale. Single ended inputs are converted against the internal ground.

Pins: an AIN pin is on function B, the analog function, without input buffer
and pull while a channel uses it. CloseChannel and PowerOff put it back in
its reset state (IOPinDisable), unless the DAC uses it (PA02 is also the DAC
VOUT, PA03 the VREFA of the DAC).

References, AdcCfg_t pRefVolt, 1 to ADC_SAMD_MAX_REF, each channel picks one
with RefVoltIdx:

	ADC_REFVOLT_TYPE_INTERNAL	INT1V, the 1.0 V internal reference, Voltage
								as the caller gives it
	ADC_REFVOLT_TYPE_SUPPLY		VDDANA, Voltage is VDDANA. The ADC uses
								INTVCC1 (VDDANA / 2) and the gain stage at
								half the Gain asked. At 2.0 V or less, where
								INTVCC1 is not allowed, INTVCC0 (VDDANA /
								1.48) with the same gain stage.
	ADC_REFVOLT_TYPE_EXTERNAL	Pin 0: AREFA on PA03 (VREFA), Pin 1: AREFB on
								PA04 (VREFB), 1.0 V to VDDANA - 0.6 V. The pin
								is on its analog function from Init to
								PowerOff, its AIN input cannot be opened.

Gain: 0x002 is 1/2, 0x100, 0x200, 0x400, 0x800, 0x1000 are 1 to 16; on the
supply reference 0x100 to 0x2000, 1 to 32. Full scale is Voltage / Gain, 2 x
Voltage / (1.48 x Gain) on the supply at 2.0 V or less. An input is read up
to full scale, single ended from 0, and at most VDDANA.

Clock: the main clock generator (0) divided by the ADC prescaler, 4 to 512,
with the ADC clock from 30 kHz to 2.1 MHz (datasheet range): Init fails when
no prescaler gives it. Sampling time: 1 to 64 half ADC clocks (SAMPLEN), the
shortest that covers AcqTime in usec and the datasheet minimum of the input:
250 ns, 10 us for TEMP and BANDGAP, 3 us for DAC. The prescaler is the
smallest that covers the longest sampling time of the open channels with 64
half clocks; OpenChannel fails when the slowest ADC clock does not cover it.

Resolution 8, 10 or 12 bits: a value under 9 takes 8, under 11 takes 10, 0
takes 12. Cfg.OvrSample 2 to 1024, power of 2: the hardware averaging (16
bit mode, ADJRES), the result is the 12 bit average and Resolution is 12.
0 or 1 is off.

Data in volt is Raw x full scale / (2^Resolution - 1), (2^(Resolution - 1)
- 1) for a differential channel.

Calibration: Init loads the factory linearity and bias calibration of the
NVM software calibration row into CALIB, Calibrate loads it again and leaves
the ADC enabled or disabled as it was. The first
conversion after an enable or a reference change is not used, as the
datasheet asks. A conversion that does not end in time is flushed and makes
the driver invalid; Reset resets the ADC and sets it up again.

SAM D21 erratum 1.4.1 (DS80000760): single shot conversions have linearity
errors at +125 C. The workarounds are free running mode, not in this driver,
or VDDANA above 3 V.

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
#ifndef __ADC_SAMD_H__
#define __ADC_SAMD_H__

#include <stdint.h>

#include "samdxxx.h"
#include "converters/adc_device.h"

/** @addtogroup Converters
  * @{
  */

#define ADC_SAMD_MAX_CHAN		16		//!< Logical channels
#define ADC_SAMD_MAX_REF		5		//!< References in AdcCfg_t, one per REFSEL

/// ADC of the SAM D11 and SAM D21
class AdcSamd : public AdcDevice {
public:
	AdcSamd();
	~AdcSamd() override;

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
	AdcSamd(const AdcSamd &) = delete;
	AdcSamd &operator=(const AdcSamd &) = delete;

	bool Owned();
	bool Setup();
	void LoadCalib();
	uint16_t Ctrlb(bool bDiff);
	void Timing();
	bool RefPin(int Input);
	uint32_t HwGain(const AdcChanCfg_t &Cfg);
	bool Used(int Input, int Except);
	void Vref();
	bool Convert(int Chan, int16_t *pRaw);
	void Abort();
	bool Scan(const int *pChan, int Count);

	uint8_t vInput[ADC_SAMD_MAX_CHAN];		//!< MUXPOS + 1 of each logical channel, 0 when closed
	uint8_t vNeg[ADC_SAMD_MAX_CHAN];		//!< MUXNEG, GND for single ended
	uint32_t vInputCtrl[ADC_SAMD_MAX_CHAN];	//!< INPUTCTRL of each logical channel
	uint8_t vRefCtrl[ADC_SAMD_MAX_CHAN];	//!< REFCTRL of each logical channel
	uint8_t vSampLen[ADC_SAMD_MAX_CHAN];	//!< SAMPLEN at the current prescaler
	uint32_t vSampNs[ADC_SAMD_MAX_CHAN];	//!< Sampling time, ns
	float vFull[ADC_SAMD_MAX_CHAN];			//!< Full scale in volt
	int16_t vRaw[ADC_SAMD_MAX_CHAN];		//!< Last result
	bool vbNew[ADC_SAMD_MAX_CHAN];			//!< Result not read yet
	uint8_t vRefPin[2];						//!< VREFA, VREFB pin in use, 0xFF when not
	uint32_t vClk;							//!< Main clock generator, Hz
	uint8_t vPresc;							//!< PRESCALER code, ADC clock is vClk / (4 << vPresc)
	uint8_t vPrescMin;						//!< Fastest PRESCALER with the ADC clock in range
	uint8_t vPrescMax;						//!< Slowest PRESCALER with the ADC clock in range
	uint8_t vOvs;							//!< log2 of the averaged samples, 0 when off
	bool vbFirst;							//!< Next conversion is the first after an enable or a reference change
};

/** @} End of group Converters */

#endif // __ADC_SAMD_H__
