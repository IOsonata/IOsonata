/**-------------------------------------------------------------------------
@file	nrf_fuel_gauge.h

@brief	Host test support: nrf fuel gauge.

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

// Test-only declarations for the sdk-nrfxlib v3.4.0 and later fuel-gauge API:
// every function returns an error code, values come back through pointers.
// This mock tests IOsonata integration, not the Nordic estimation algorithm.
#ifndef NPM1300_TEST_FUEL_GAUGE_H
#define NPM1300_TEST_FUEL_GAUGE_H
#include <stdbool.h>
#include <stddef.h>
extern "C" {
struct battery_model
{
	int Dummy;
};
struct battery_model_primary;
struct nrf_fuel_gauge_state_info;
struct nrf_fuel_gauge_config_parameters;
struct nrf_fuel_gauge_init_parameters {
	float v0, i0, t0;
	union {
		const struct battery_model *model;
		const struct battery_model_primary *model_primary;
	};
	const struct nrf_fuel_gauge_config_parameters *opt_params;
	int (*lock_func)(const void *context, bool lock);
	const void *lock_context;
	const void *state;
	size_t state_size;
};
enum nrf_fuel_gauge_charge_state {
	NRF_FUEL_GAUGE_CHARGE_STATE_IDLE,
	NRF_FUEL_GAUGE_CHARGE_STATE_TRICKLE,
	NRF_FUEL_GAUGE_CHARGE_STATE_CC,
	NRF_FUEL_GAUGE_CHARGE_STATE_CC_LIMITED,
	NRF_FUEL_GAUGE_CHARGE_STATE_CV,
	NRF_FUEL_GAUGE_CHARGE_STATE_COMPLETE
};
enum nrf_fuel_gauge_ext_state_info_type {
	NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_CONNECTED,
	NRF_FUEL_GAUGE_EXT_STATE_INFO_VBUS_DISCONNECTED,
	NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_STATE_CHANGE,
	NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_CORRECTION,
	NRF_FUEL_GAUGE_EXT_STATE_INFO_DISCHARGE_CURRENT_CORRECTION,
	NRF_FUEL_GAUGE_EXT_STATE_INFO_CHARGE_CURRENT_LIMIT,
	NRF_FUEL_GAUGE_EXT_STATE_INFO_TERM_CURRENT,
	NRF_FUEL_GAUGE_EXT_STATE_INFO_BATTERY_REPLACED
};
union nrf_fuel_gauge_ext_state_info_data {
	enum nrf_fuel_gauge_charge_state charge_state;
	float current_correction_factor;
	float charge_current_limit;
	float charge_term_current;
};
extern const size_t nrf_fuel_gauge_state_size;
int nrf_fuel_gauge_init(const struct nrf_fuel_gauge_init_parameters *, float *);
int nrf_fuel_gauge_process(float, float, float, float, float *, struct nrf_fuel_gauge_state_info *);
int nrf_fuel_gauge_tte_get(float *);
int nrf_fuel_gauge_ttf_get(float *);
int nrf_fuel_gauge_idle_set(float, float, float);
int nrf_fuel_gauge_state_get(void *, size_t);
int nrf_fuel_gauge_ext_state_update(enum nrf_fuel_gauge_ext_state_info_type,
	union nrf_fuel_gauge_ext_state_info_data *);
}
#endif
