/**-------------------------------------------------------------------------
@file	gas_ccs811.h

@brief	CCS811 gas sensor.

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

/** CCS811 gas sensor. SPDX-License-Identifier: MIT */
#ifndef __GAS_CCS811_H__
#define __GAS_CCS811_H__
#include "sensors/gas_sensor.h"

#define CCS811_I2C_ADDR0 0x5A
#define CCS811_I2C_ADDR1 0x5B

struct Ccs811Data_t {
	uint64_t Timestamp; // us; zero without a supplied timer
	uint16_t Eco2; // ppm; only meaningful when AlgorithmValid
	uint16_t Etvoc; // ppb; only meaningful when AlgorithmValid
	uint16_t RawAdc;
	uint8_t Current; // uA
	bool AlgorithmValid; // false in raw-only mode 4; does not certify conditioning
};

// Callback drives nWAKE low when Active=true. Must report GPIO transport failures.
typedef bool (*Ccs811WakeFn_t)(bool Active, void *pContext);

class GasCcs811 : public GasSensor {
public:
	GasCcs811();
	// Configure before Init. Without callback the board must hold nWAKE low.
	void WakeControl(Ccs811WakeFn_t Callback, void *pContext);
	// Freq is mHz, rounded to nearest: 1000, 100, 17 (one minute), 4000 (raw only).
	// Ties choose the lower rate; zero clamps to 17. Use SetDriveMode(0) for idle.
	bool Init(const GasSensorCfg_t &Cfg, DeviceIntrf * const pIntrf = nullptr,
		Timer * const pTimer = nullptr) override;
	bool Enable() override;
	void Disable() override;
	void Reset() override; // Returns to bootloader and invalidates; call Init afterwards.
	bool SoftReset();
	bool Read(Ccs811Data_t &Data);
	bool Read(GasSensorData_t &Data) override;
	bool StartSampling() override;
	bool UpdateData() override;
	void IntHandler() override
	{
		DataReadySet();
	} // No I2C in ISR.
	bool SetHeatingProfile(int Count, const GasSensorHeater_t * const pProfile) override;
	bool Mode(SENSOR_OPMODE Mode, uint32_t Freq) override;
	using Sensor::Mode;
	uint32_t SamplingFrequency(uint32_t Freq) override;
	using Sensor::SamplingFrequency;
	// Modes 0..4. To lower sample rate: enter idle, wait >=600000ms,
	// then pass the actual elapsed idle time (caller-owned monotonic clock).
	bool SetDriveMode(uint8_t Mode, uint32_t IdleMs = 0);
	bool SetEnvironment(int16_t TemperatureCentiC, uint16_t HumidityCentiPercent);
	bool ConfigureInterrupt(bool Enable, bool ThresholdOnly = false);
	bool SetThresholds(uint16_t LowPpm, uint16_t HighPpm);
	bool ReadBaseline(uint16_t &Baseline);
	bool WriteBaseline(uint16_t Baseline); // Caller observes 20-minute run-in.
	bool ReadError(uint8_t &Error); // Reading ERROR_ID clears hardware ERROR.
	bool FirmwareVersion(uint16_t &Boot, uint16_t &Application);
private:
	bool Transfer(uint8_t Reg, uint8_t *pData, int Len, bool Write);
	bool ReadReg(uint8_t Reg, uint8_t *pData, int Len);
	bool WriteReg(uint8_t Reg, const uint8_t *pData, int Len);
	bool AppStart();
	Ccs811WakeFn_t vWake;
	void *vpWakeContext;
	uint8_t vDrive;
	uint8_t vLastDrive;
	uint8_t vResumeDrive;
	Ccs811Data_t vData;
};
#endif
