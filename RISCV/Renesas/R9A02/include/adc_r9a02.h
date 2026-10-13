/**-------------------------------------------------------------------------
@file	adc_r9a02.h

@brief	R9A02G021 12-bit A/D converter (ADC12)

AdcDevice implementation. Each AdcChanCfg_t is a logical channel, Chan 0 to
R9A02_ADC_MAX_CHAN - 1, converting the input given in PinP.PinNo:

	0 to 5		: ANI0 to ANI5 (P002, P003, P400, P401, P006, P007)
	16 to 19	: ANI16 to ANI19 (P000, P001, P105, P106)
	R9A02_ADC_INPUT_TEMP : temperature sensor
	R9A02_ADC_INPUT_VREF : internal reference voltage, 1.47 V typical

Channels are converted one after the other in select mode, one-shot,
software trigger (R9A02G021 User's Manual R01UH1036EJ0110, 29.4).

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
#ifndef __ADC_R9A02_H__
#define __ADC_R9A02_H__

#include <stdint.h>

#include "converters/adc_device.h"

#define R9A02_ADC_MAX_CHAN			8			//!< Logical channels
#define R9A02_ADC_INPUT_TEMP		0x80		//!< PinP.PinNo of the temperature sensor
#define R9A02_ADC_INPUT_VREF		0x81		//!< PinP.PinNo of the internal reference

/// ADC12 as an AdcDevice
class AdcR9a02 : public AdcDevice {
public:
	AdcR9a02();
	virtual ~AdcR9a02();
	AdcR9a02(AdcR9a02&);	// Copy ctor not allowed

	/// @brief	Initialization.
	///
	/// Supported: ADC_CONV_MODE_SINGLE, resolution 8, 10 or 12 bits, one
	/// reference, ADC_REFVOLT_TYPE_SUPPLY (VCC) or ADC_REFVOLT_TYPE_EXTERNAL
	/// (AVREFP on P002) with its voltage. Anything else fails.
	///
	/// @param	Cfg		: Configuration
	/// @param	pTimer	: Not used
	/// @param	pIntrf	: Not used, internal converter
	///
	/// @return	true on success
	virtual bool Init(const AdcCfg_t &Cfg, Timer * const pTimer = NULL, DeviceIntrf * const pIntrf = NULL);

	/// @brief	Set the resolution, 8, 10 or 12 bits, the closest is taken.
	///
	/// @return	Resolution set
	virtual uint16_t Resolution(uint16_t Val);
	virtual uint16_t Resolution() { return vResolution; }

	/// @brief	Configure logical channels.
	///
	/// Single ended only. AcqTime (us) selects the conversion clock and the
	/// operation voltage mode so that the sampling time is long enough.
	virtual bool OpenChannel(const AdcChanCfg_t * const pChanCfg, int NbChan);
	virtual void CloseChannel(int Chan);

	/// @brief	Convert every open channel once.
	///
	/// Without interrupt the conversions are done before it returns. With
	/// interrupt they run in the background, the event handler gets
	/// DEV_EVT_DATA_RDY when the last one is done.
	virtual bool StartConversion();
	virtual void StopConversion();

	/// @return	true if a channel has a result not read yet
	virtual bool UpdateData();

	/// @brief	Results not read yet, one per channel.
	virtual int Read(AdcData_t *pBuff, int Len);

	/// @brief	Result of one channel. Without interrupt the channel is
	/// converted now when it has no result waiting.
	virtual bool Read(int Chan, AdcData_t *pBuff);

	/// @brief	Zero and full scale measurement (ADTES), the results are
	/// corrected with them afterward.
	virtual bool Calibrate();

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual void PowerOff();

	/// Interrupt handler, internal use
	void IntHandler();

private:
	bool Configure();
	bool ConvertOne(int Chan);
	bool StartOne(int Chan);
	float ToVolt(int Chan, uint32_t Raw);
	int NextChan(int From);

	uint8_t vAds[R9A02_ADC_MAX_CHAN];		// ADS value, 0xFF when closed
	uint32_t vAcqNs[R9A02_ADC_MAX_CHAN];	// Sampling time needed
	uint16_t vRaw[R9A02_ADC_MAX_CHAN];		// Last result, right aligned
	bool vbNew[R9A02_ADC_MAX_CHAN];			// Result not read yet
	float vRefVolt;							// '+' reference voltage
	uint8_t vAdm0;							// FR and LV
	uint8_t vAdm1;							// ADLSP, one-shot
	uint8_t vAdm2;							// Resolution and reference
	uint32_t vFadHz;
	uint32_t vLoops;						// Poll loops for one conversion
	int vIntNo;								// IELSR of ADC_ENDI, -1 when none
	int vCurChan;							// Channel converting in background, -1 when none
	uint16_t vZero;							// Calibration, 12-bit scale
	uint16_t vFull;
	bool vbCal;
	bool vbInternal;						// ADS.ADISS currently set
	bool vbDiscard;							// Next result after an ADISS change is not valid
};

#endif
