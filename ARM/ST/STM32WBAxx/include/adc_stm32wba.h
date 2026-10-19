/**-------------------------------------------------------------------------
@file	adc_stm32wba.h

@brief	STM32WBA ADC4 polling implementation


@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#ifndef __ADC_STM32WBA_H__
#define __ADC_STM32WBA_H__
#include "converters/adc_device.h"

// ADC4, software-triggered single rounds, polling. No interrupt, DMA,
// oversampling, differential inputs or FIFO. Unsupported requests fail Init
// or OpenChannel. One object owns ADC4 until PowerOff or destruction.
// Logical Chan 0..7 maps to PinP.PinNo = ADC4 input 1..10:
// PA8, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0, PB9 (DS14736 table 23).
// Reference is VREF+: supply or external, caller supplies its voltage.
// HCLK/16 and 814.5 sampling cycles; OpenChannel checks AcqTime fits.
class AdcSTM32WBA : public AdcDevice {
public:
	AdcSTM32WBA();
	~AdcSTM32WBA() override;
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
	uint16_t Resolution() override { return vResolution; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate() override { return 0; }
	void Mode(ADC_CONV_MODE) override {} // Single rounds only
	ADC_CONV_MODE Mode() override { return ADC_CONV_MODE_SINGLE; }
private:
	AdcSTM32WBA(const AdcSTM32WBA &) = delete;
	AdcSTM32WBA &operator=(const AdcSTM32WBA &) = delete;
	bool Convert(int Chan);
	bool Owned();
	uint8_t vInput[8];
	bool vbNew[8];
	uint16_t vRaw[8];
	float vRef;
};
#endif
