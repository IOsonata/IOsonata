/**-------------------------------------------------------------------------
@file	pm_npm1300_fuel.h

@brief	IOsonata pm npm1300 fuel.

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

/** nPM1300 fuel gauge using Nordic's secondary-cell algorithm.
 * MIT License. Copyright (c) 2026 I-SYST inc.
 * Link pm_npm1300_fuel.cpp and the matching sdk-nrfxlib fuel-gauge archive.
 * Written for the API of sdk-nrfxlib v3.4.0 and later, where every function
 * returns an error code.
 * The Nordic library has one global state: only one instance can own it.
 * Call from one application context, never from an ISR.
 */
#ifndef __PM_NPM1300_FUEL_H__
#define __PM_NPM1300_FUEL_H__

#include "pwrmgnt/pm_npm1300.h"

struct battery_model;

class PmnPM1300FuelGauge : public PmnPM1300
{
public:
	PmnPM1300FuelGauge();
	~PmnPM1300FuelGauge();
	PmnPM1300FuelGauge(const PmnPM1300FuelGauge &) = delete;
	PmnPM1300FuelGauge &operator=(const PmnPM1300FuelGauge &) = delete;

	bool Init(DeviceIntrf * const pIntrf, uint32_t DevAddr = NPM1300_I2C_7BITS_ADDR);
	bool Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf) override;
	bool Init(const FuelGaugeCfg_t &Cfg, DeviceIntrf * const pIntrf,
		PowerMgnt * const pPwrMnt) override;
	// Model must match the installed battery and remain alive. State is optional;
	// restore only with the same library version, model and firmware addresses.
	// The library refuses a state saved by another version: start without it.
	bool StartFuelGauge(const battery_model &Model, uint32_t NowMs,
		const void *pState = NULL, size_t StateSize = 0);
	bool UpdateFuelGauge(uint32_t NowMs);
	bool FuelGaugeIdle(int32_t AverageDischargeUa);
	bool SaveFuelGaugeState(void *pState, size_t Size);
	static size_t FuelGaugeStateSize();
	void StopFuelGauge();
	uint16_t Level() override;
	float TimeToEmpty(); // seconds; NAN if unavailable
	float TimeToFull();
	void Reset() override;
	void PowerOff() override;
	bool EnterShipMode();

private:
	bool InformFuelGauge(const Npm1300BatterySample_t &Sample);
	bool SetLevel(float Percent);
	uint32_t vSampleTimeMs;
	uint16_t vLevel;
	bool vbLowBattery;
};

#endif
