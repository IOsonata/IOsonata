/**-------------------------------------------------------------------------
@file	pm_npm1300_fuel.cpp

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

/** nPM1300 / nRF Fuel Gauge integration.
 * MIT License. Copyright (c) 2026 I-SYST inc.
 */
#include <limits.h>
#include <math.h>
#include "nrf_fuel_gauge.h"
#include "pwrmgnt/pm_npm1300_fuel.h"

static PmnPM1300FuelGauge *s_pOwner;

PmnPM1300FuelGauge::PmnPM1300FuelGauge() :
	vSampleTimeMs(0), vLevel(UINT16_MAX), vbLowBattery(false)
{
}

PmnPM1300FuelGauge::~PmnPM1300FuelGauge()
{
	StopFuelGauge();
}

void PmnPM1300FuelGauge::StopFuelGauge()
{
	if (s_pOwner == this)
		s_pOwner = NULL;
	vLevel = UINT16_MAX;
	vbLowBattery = false;
}

bool PmnPM1300FuelGauge::Init(DeviceIntrf * const pIntrf, uint32_t DevAddr)
{
	StopFuelGauge();
	return PmnPM1300::Init(pIntrf, DevAddr);
}

bool PmnPM1300FuelGauge::Init(const PwrMgntCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	StopFuelGauge();
	return PmnPM1300::Init(Cfg, pIntrf);
}

bool PmnPM1300FuelGauge::Init(const FuelGaugeCfg_t &Cfg, DeviceIntrf * const pIntrf,
	PowerMgnt * const pPwrMnt)
{
	StopFuelGauge();
	return PmnPM1300::Init(Cfg, pIntrf, pPwrMnt);
}

bool PmnPM1300FuelGauge::InformFuelGauge(const Npm1300BatterySample_t &Sample)
{
	nrf_fuel_gauge_ext_state_info_data data = {};

	if (nrf_fuel_gauge_ext_state_update((Sample.VbusStatus & 1U) != 0 ?
		NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED :
		NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED, NULL) != 0)
		return false;
	data.charge_current_limit = (float)Sample.ChargeLimitMa / 1000.0f;
	if (nrf_fuel_gauge_ext_state_update(
		NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_LIMIT, &data) != 0)
		return false;
	data.charge_term_current = (float)Sample.ChargeLimitMa *
		(float)Sample.TerminationPercent / 100000.0f;
	if (nrf_fuel_gauge_ext_state_update(
		NRF_FUEL_GAUGE_EXT_STATE_INFO_TERM_CURRENT, &data) != 0)
		return false;
	if ((Sample.ChargerStatus & (1U << 1)) != 0)
		data.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_COMPLETE;
	else if ((Sample.ChargerStatus & (1U << 2)) != 0)
		data.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_TRICKLE;
	else if ((Sample.ChargerStatus & (1U << 3)) != 0)
		data.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CC;
	else if ((Sample.ChargerStatus & (1U << 4)) != 0)
		data.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_CV;
	else
		data.charge_state = NRF_FUEL_GAUGE_CHARGE_STATE_IDLE;
	return nrf_fuel_gauge_ext_state_update(
		NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_STATE_CHANGE, &data) == 0;
}

bool PmnPM1300FuelGauge::SetLevel(float Percent)
{
	if (!isfinite(Percent) || Percent < 0.0f || Percent > 100.0f)
	{
		vLevel = UINT16_MAX;
		return false;
	}
	vLevel = (uint16_t)lroundf(Percent * 10.0f);
	if (vLevel <= 100 && !vbLowBattery)
	{
		vbLowBattery = true;
		LowBatAlert();
	}
	else if (vLevel >= 150)
		vbLowBattery = false;
	return true;
}

bool PmnPM1300FuelGauge::StartFuelGauge(const battery_model &Model, uint32_t NowMs,
	const void *pState, size_t StateSize)
{
	Npm1300BatterySample_t sample;
	nrf_fuel_gauge_init_parameters params = {};
	float soc;

	if (s_pOwner != NULL && s_pOwner != this)
		return false;
	StopFuelGauge();
	if (!Valid() || (pState != NULL && StateSize != nrf_fuel_gauge_state_size) ||
		(pState == NULL && StateSize != 0) || !ReadBatterySample(sample))
		return false;
	params.v0 = (float)sample.VoltageMv / 1000.0f;
	params.i0 = (float)sample.CurrentUa / 1000000.0f;
	params.t0 = (float)sample.TemperatureDeciC / 10.0f;
	params.model = &Model;
	params.state = pState;
	if (nrf_fuel_gauge_init(&params, NULL) != 0 || !InformFuelGauge(sample))
		return false;
	soc = nrf_fuel_gauge_process(params.v0, params.i0, params.t0, 0.0f, NULL);
	s_pOwner = this;
	vSampleTimeMs = NowMs;
	if (!SetLevel(soc))
	{
		StopFuelGauge();
		return false;
	}
	return true;
}

bool PmnPM1300FuelGauge::UpdateFuelGauge(uint32_t NowMs)
{
	Npm1300BatterySample_t sample;
	uint32_t elapsed;
	float soc;

	vLevel = UINT16_MAX;
	if (s_pOwner != this || !Valid())
		return false;
	elapsed = NowMs - vSampleTimeMs;
	// Unsigned subtraction handles timer rollover; reject backwards timestamps.
	if (elapsed == 0 || elapsed > INT32_MAX || !ReadBatterySample(sample) ||
		!InformFuelGauge(sample))
		return false;
	soc = nrf_fuel_gauge_process((float)sample.VoltageMv / 1000.0f,
		(float)sample.CurrentUa / 1000000.0f,
		(float)sample.TemperatureDeciC / 10.0f, (float)elapsed / 1000.0f,
		NULL);
	vSampleTimeMs = NowMs;
	return SetLevel(soc);
}

uint16_t PmnPM1300FuelGauge::Level()
{
	return s_pOwner == this && Valid() ? vLevel : UINT16_MAX;
}

bool PmnPM1300FuelGauge::FuelGaugeIdle(int32_t AverageDischargeUa)
{
	Npm1300BatterySample_t sample;

	if (s_pOwner != this || !Valid() || AverageDischargeUa < 0 ||
		AverageDischargeUa > 1120000 || !ReadBatterySample(sample))
		return false;
	nrf_fuel_gauge_idle_set((float)sample.VoltageMv / 1000.0f,
		(float)sample.TemperatureDeciC / 10.0f, (float)AverageDischargeUa / 1000000.0f);
	return true;
}

size_t PmnPM1300FuelGauge::FuelGaugeStateSize()
{
	return nrf_fuel_gauge_state_size;
}

bool PmnPM1300FuelGauge::SaveFuelGaugeState(void *pState, size_t Size)
{
	return s_pOwner == this && Valid() && vLevel != UINT16_MAX && pState != NULL &&
		Size >= nrf_fuel_gauge_state_size && nrf_fuel_gauge_state_get(pState, Size) == 0;
}

float PmnPM1300FuelGauge::TimeToEmpty()
{
	return Level() == UINT16_MAX ? NAN : nrf_fuel_gauge_tte_get();
}

float PmnPM1300FuelGauge::TimeToFull()
{
	return Level() == UINT16_MAX ? NAN : nrf_fuel_gauge_ttf_get();
}

void PmnPM1300FuelGauge::Reset()
{
	StopFuelGauge();
	PmnPM1300::Reset();
}

bool PmnPM1300FuelGauge::EnterShipMode()
{
	if (!PmnPM1300::EnterShipMode())
		return false;
	StopFuelGauge();
	return true;
}

void PmnPM1300FuelGauge::PowerOff()
{
	EnterShipMode();
}
