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

// Test-only declarations for the sdk-nrfxlib v3.2.0 fuel-gauge API.
// This mock tests IOsonata integration, not the Nordic estimation algorithm.
#ifndef NPM1300_TEST_FUEL_GAUGE_H
#define NPM1300_TEST_FUEL_GAUGE_H
#include <stddef.h>
extern "C" {
struct battery_model
{
	int Dummy;
};
struct nrf_fuel_gauge_state_info;
struct nrf_fuel_gauge_init_parameters {
	float v0, i0, t0;
	const struct battery_model *model;
	const void *opt_params;
	const void *state;
};
enum nrf_fuel_gauge_charge_state {
	NRF_FUEL_GAUGE_CHARGE_STATE_IDLE,
	NRF_FUEL_GAUGE_CHARGE_STATE_TRICKLE,
	NRF_FUEL_GAUGE_CHARGE_STATE_CC,
	NRF_FUEL_GAUGE_CHARGE_STATE_CC_VBUS_LIMITED,
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
	NRF_FUEL_GAUGE_EXT_STATE_INFO_TERM_CURRENT
};
union nrf_fuel_gauge_ext_state_info_data {
	enum nrf_fuel_gauge_charge_state charge_state;
	float current_correction_factor;
	float charge_current_limit;
	float charge_term_current;
};
extern const size_t nrf_fuel_gauge_state_size;
int nrf_fuel_gauge_init(const struct nrf_fuel_gauge_init_parameters *, float *);
float nrf_fuel_gauge_process(float, float, float, float, struct nrf_fuel_gauge_state_info *);
float nrf_fuel_gauge_tte_get(void);
float nrf_fuel_gauge_ttf_get(void);
void nrf_fuel_gauge_idle_set(float, float, float);
int nrf_fuel_gauge_state_get(void *, size_t);
int nrf_fuel_gauge_ext_state_update(enum nrf_fuel_gauge_ext_state_info_type,
	union nrf_fuel_gauge_ext_state_info_data *);
}
#endif
