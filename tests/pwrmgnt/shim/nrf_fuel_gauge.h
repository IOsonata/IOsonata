// Test-only declarations for the sdk-nrfxlib v3.2.0 fuel-gauge API.
// This mock tests IOsonata integration, not the Nordic estimation algorithm.
#ifndef NPM1300_TEST_FUEL_GAUGE_H
#define NPM1300_TEST_FUEL_GAUGE_H
#include <stddef.h>
extern "C" {
struct battery_model { int Dummy; };
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
