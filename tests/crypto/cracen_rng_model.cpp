/**-------------------------------------------------------------------------
@file	cracen_rng_model.cpp

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

#include "cracen_rng_model.h"
#include "crypto_rng_nrf.h"
#include "crypto/ba414ep.h"
#include "cracen_intrf.h"
#define main CmModelMain
#include "cm_model.cpp"
#undef main

NRF_CRACEN_Type g_Cracen = {};
NRF_CRACENCORE_Type g_Core = {};
static bool s_Active, s_Empty, s_Startup, s_SoftDevice, s_SdFail;
static unsigned s_Words, s_TotalWords, s_FailAfter, s_KeysWritten, s_Polls;
static uint32_t s_Keys[4];
static Ba414ep s_PkaLock;

void nrf_cracen_rng_control_set(NRF_CRACENCORE_Type *, const nrf_cracen_rng_control_t *cfg)
{
	s_Active = cfg->enable;
	if (s_Active)
	{
		assert(cfg->number_128_blocks == 4 && !cfg->soft_reset);
		assert(!cfg->fifo_full_int_en && !cfg->fifo_write_startup);
		s_Words = s_KeysWritten = 0;
	}
}
void nrf_cracen_rng_key_set(NRF_CRACENCORE_Type *, uint8_t index, uint32_t word)
{
	assert(index < 4);
	s_Keys[index] = word;
	if (s_Active)
		s_KeysWritten++;
}
nrf_cracen_rng_fsm_state_t nrf_cracen_rng_fsm_state_get(const NRF_CRACENCORE_Type *)
{
	s_Polls++;
	if (!s_Active)
		return NRF_CRACEN_RNG_FSM_STATE_RESET;
	if (s_FailAfter && s_Words >= s_FailAfter)
		return NRF_CRACEN_RNG_FSM_STATE_ERROR;
	return s_Startup ? NRF_CRACEN_RNG_FSM_STATE_STARTUP : NRF_CRACEN_RNG_FSM_STATE_IDLE_READY;
}
uint32_t nrf_cracen_rng_fifo_level_get(const NRF_CRACENCORE_Type *)
{
	return s_Empty ? 0 : 1;
}
uint32_t nrf_cracen_rng_fifo_get(const NRF_CRACENCORE_Type *)
{
	uint32_t word;
	assert(!s_Cm.OpAcquire());
	assert(!s_PkaLock.OpAcquire());
	assert(s_Active && !s_Empty && !s_Startup);
	if (s_Words >= 4)
		assert(s_KeysWritten == 4);
	assert(RAND_bytes((uint8_t *)&word, sizeof(word)) == 1);
	s_Words++;
	s_TotalWords++;
	return word;
}
int sd_softdevice_is_enabled(uint8_t *out)
{
	*out = s_SoftDevice;
	return 0;
}
int sd_rand_application_bytes_available_get(uint8_t *out)
{
	*out = s_SdFail ? 0 : 7;
	return 0;
}
int sd_rand_application_vector_get(uint8_t *out, uint8_t n)
{
	assert(!s_Cm.OpAcquire() && !s_PkaLock.OpAcquire());
	return RAND_bytes(out, n) == 1 ? 0 : 1;
}

int main()
{
	CryptoRngNrf *rng;
	CracenIntrf *intrf;
	uint8_t out[70003];
	unsigned before, i;
	Ba414ep secondPka;
	assert(CmModelMain() == 0);
	rng = CryptoRngNrfInstance();
	intrf = CracenIntrfInstance();
	assert(rng && intrf);
	assert(g_Cracen.ENABLE == 7);
	assert(s_PkaLock.OpAcquire());
	assert(!secondPka.OpAcquire() && !s_Cm.OpAcquire());
	assert(rng->Random(out, 16) == CRYPTO_STATUS_BUSY);
	s_PkaLock.OpRelease();
	assert(s_Cm.OpAcquire());
	assert(rng->Random(out, 16) == CRYPTO_STATUS_BUSY);
	s_Cm.OpRelease();
	assert(intrf->StartRx(CRACEN_ADDR_RNG));
	assert(!s_Cm.OpAcquire() && !s_PkaLock.OpAcquire());
	intrf->StopRx();
	assert(s_Cm.OpAcquire());
	s_Cm.OpRelease();
	for (i = 0; i < 3; i++)
	{
		memset(out, 0xA5, sizeof(out));
		s_Empty = i == 0;
		s_Startup = i == 1;
		s_FailAfter = i == 2 ? 7 : 0;
		before = s_Polls;
		assert(rng->Random(out+1, 21) == CRYPTO_STATUS_FAIL);
		assert(s_Polls-before < 64);
		assert(out[0] == 0xA5 && out[22] == 0xA5);
		for (unsigned j = 1; j <= 21; j++)
			assert(out[j] == 0);
		assert(!s_Active);
		for (unsigned j = 0; j < 4; j++)
			assert(s_Keys[j] == 0);
	}
	s_Empty = s_Startup = false;
	s_FailAfter = 0;
	assert(rng->Random(out+1, 70001) == CRYPTO_STATUS_OK);
	assert(s_Words == 4 + (70001+3)/4);
	assert(out[0] == 0xA5 && out[70002] == 0xA5);
	s_SoftDevice = true;
	before = s_TotalWords;
	assert(rng->Random(out, 21) == CRYPTO_STATUS_OK);
	assert(s_TotalWords == before);
	s_SdFail = true;
	memset(out, 0xA5, 21);
	assert(rng->Random(out, 21) == CRYPTO_STATUS_FAIL);
	for (i = 0; i < 21; i++)
		assert(out[i] == 0);
	s_SdFail = false;
	s_SoftDevice = false;
	assert(rng->Random(out, 21) == CRYPTO_STATUS_OK);
	assert(g_Cracen.ENABLE == 7);
	printf("CRACEN: bounded TRNG health/startup/FIFO failures, 70001-byte draw, SoftDevice and AES/PKE exclusion passed\n");
}
