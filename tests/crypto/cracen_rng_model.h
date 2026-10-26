/**-------------------------------------------------------------------------
@file	cracen_rng_model.h

@brief	Host test support: cracen rng model.

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

#pragma once
#include <stdint.h>
struct NRF_CRACEN_Type
{
	uint32_t ENABLE;
};
struct NRF_CRACENCORE_Type
{
	uint32_t PK[0x10000/4];
};
extern NRF_CRACEN_Type g_Cracen;
extern NRF_CRACENCORE_Type g_Core;
#define NRF_CRACEN (&g_Cracen)
#define NRF_CRACENCORE (&g_Core)
#define CRACEN_ENABLE_CRYPTOMASTER_Msk 1U
#define CRACEN_ENABLE_PKEIKG_Msk 2U
#define CRACEN_ENABLE_RNG_Msk 4U
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define TRNG_CLK_DIV 7U
#define NRF_CRACEN_RNG_HAS_IDLE_TIMER 1
#define NRF_CRACEN_RNG_HAS_FIFOWRITESTARTUP 1
#define NRF_CRACEN_RNG_HAS_BLENDING 1
#define NRF_CRACEN_RNG_HAS_REPEATTHRESHOLD 1
#define NRF_CRACEN_RNG_HAS_PROPTESTCUTOFF 1
#define NRF_CRACEN_RNG_HAS_DISABLEOSC 1
#define NRF_CRACEN_RNG_HAS_COOLDOWNPERIOD 1
#ifdef NRF54LM20A_XXAA
#define NRF_CRACEN_HAS_RECOMMENDED_CONF 1
#else
#define NRF_CRACEN_HAS_RECOMMENDED_CONF 0
#endif
#define CRACENCORE_RNGCONTROL_SAMPLINGPERIOD_SAMPLINGPERIOD_Recommended 9U
#define CRACENCORE_RNGCONTROL_WARMUPPERIOD_WARMUPPERIOD_Recommended 512U
#define CRACENCORE_RNGCONTROL_CONTROL_NB128BITBLOCKS_Recommended 4U
#define CRACENCORE_RNGCONTROL_CONTROL_FIFOWRITESTARTUP_Recommended 0U
#define CRACENCORE_RNGCONTROL_REPEATTHRESHOLD_REPEATTHRESHOLD_Recommended 4U
#define CRACENCORE_RNGCONTROL_PROPTESTCUTOFF_PROPTESTCUTOFF_Recommended 8U
#define CRACENCORE_RNGCONTROL_DISABLEOSC_DISABLEOSC_Recommended 0U
#define CRACENCORE_RNGCONTROL_COOLDOWNPERIOD_COOLDOWNPERIOD_Recommended 4U
#define NRF_CRACEN_RNG_BLENDING_CONCATENATION 0
struct nrf_cracen_rng_control_t {
	bool enable, fifo_full_int_en, soft_reset, fifo_write_startup;
	uint8_t number_128_blocks;
	int blending_method;
};
enum nrf_cracen_rng_fsm_state_t {
	NRF_CRACEN_RNG_FSM_STATE_RESET, NRF_CRACEN_RNG_FSM_STATE_STARTUP,
	NRF_CRACEN_RNG_FSM_STATE_IDLE_READY, NRF_CRACEN_RNG_FSM_STATE_ERROR
};
void nrf_cracen_rng_control_set(NRF_CRACENCORE_Type *, const nrf_cracen_rng_control_t *);
void nrf_cracen_rng_key_set(NRF_CRACENCORE_Type *, uint8_t, uint32_t);
nrf_cracen_rng_fsm_state_t nrf_cracen_rng_fsm_state_get(const NRF_CRACENCORE_Type *);
uint32_t nrf_cracen_rng_fifo_level_get(const NRF_CRACENCORE_Type *);
uint32_t nrf_cracen_rng_fifo_get(const NRF_CRACENCORE_Type *);
inline void nrf_cracen_rng_off_timer_set(NRF_CRACENCORE_Type *, uint16_t)
{
}
inline void nrf_cracen_rng_clk_div_set(NRF_CRACENCORE_Type *, uint16_t)
{
}
inline void nrf_cracen_rng_init_wait_val_set(NRF_CRACENCORE_Type *, uint16_t)
{
}
inline void nrf_cracen_rng_repeatthreshold_set(NRF_CRACENCORE_Type *, uint32_t)
{
}
inline void nrf_cracen_rng_proptestcutoff_set(NRF_CRACENCORE_Type *, uint32_t)
{
}
inline void nrf_cracen_rng_disableosc_set(NRF_CRACENCORE_Type *, uint8_t, uint32_t)
{
}
inline void nrf_cracen_rng_cooldown_period_set(NRF_CRACENCORE_Type *, uint32_t)
{
}
#define NRF_SUCCESS 0
int sd_softdevice_is_enabled(uint8_t *);
int sd_rand_application_bytes_available_get(uint8_t *);
int sd_rand_application_vector_get(uint8_t *, uint8_t);
