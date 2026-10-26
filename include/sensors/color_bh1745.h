/**-------------------------------------------------------------------------
@file	color_bh1745.h

@brief	BH1745NUC RGBC sensor.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

/** BH1745NUC RGBC sensor. SPDX-License-Identifier: MIT */
#ifndef __COLOR_BH1745_H__
#define __COLOR_BH1745_H__
#include "device.h"

#define BH1745_I2C_ADDR0 0x38
#define BH1745_I2C_ADDR1 0x39

struct Bh1745Data_t
{
	uint16_t Red;
	uint16_t Green;
	uint16_t Blue;
	uint16_t Clear;
};

class ColorBh1745 : public Device
{
public:
	ColorBh1745();
	bool Init(DeviceIntrf *pIntrf, uint32_t Addr = BH1745_I2C_ADDR0);
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	bool SoftReset();
	// Time code 0..5: 160, 320, 640, 1280, 2560, 5120 ms; gain 1, 2, 16.
	bool Configure(uint8_t TimeCode, uint8_t Gain);
	// Returns false without modifying Data if not ready or transfer fails.
	bool Read(Bh1745Data_t &Data);
	// Source 0..3: R,G,B,C. Persistence 0..3: toggle, each, 4, 8 samples.
	bool ConfigureInterrupt(bool Enable, uint8_t Source, uint8_t Persistence,
		uint16_t Lower, uint16_t Upper, bool Latched = true);
	bool ClearInterrupt();
private:
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, const uint8_t *pData, int Len);
};
#endif
