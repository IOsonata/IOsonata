/** nPM1300 fuel gauge using Nordic's secondary-cell algorithm.
 * MIT License. Copyright (c) 2026 I-SYST inc.
 * Link pm_npm1300_fuel.cpp and the matching sdk-nrfxlib fuel-gauge archive.
 * The Nordic library has one global state: only one instance can own it.
 * Call from one application context, never from an ISR.
 */
#ifndef __PM_NPM1300_FUEL_H__
#define __PM_NPM1300_FUEL_H__

#include "pwrmgnt/pm_npm1300.h"

struct battery_model;

class PmnPM1300FuelGauge : public PmnPM1300 {
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
