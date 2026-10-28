/**-------------------------------------------------------------------------
@file	audpll_test.c

@brief	LPC546xx audio PLL regression

Builds the real system_LPC546xx.c with SYSCON and the voltage registers in
mapped host memory. Checks the audio PLL output for the audio rates the I2S
driver asks, closest outputs against a full search of the dividers, the
register values with the NXP decoders, the clock input, power down, VD3 and
lock failures, and that no other SYSCON register changes. The lock and VD3
ready bits are supplied by the test, the analog PLL is not simulated.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "LPC546xx.h"

// SystemInit is not run, SCB and the vector table only need to exist
static struct { uint32_t CPACR, VTOR; } s_Scb;
#define SCB (&s_Scb)
void *__Vectors;

#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
#include "../../../ARM/NXP/LPC546xx/src/system_LPC546xx.c"

#define MAP_BASE	0x40000000UL
#define MAP_SIZE	0x21000UL

#define CCO_MIN		275000000ULL
#define CCO_MAX		550000000ULL
#define REF_MIN		10000ULL

#define VD3_RDY		(1UL << 5)
#define AUD_PD		SYSCON_PDRUNCFG_PDEN_AUD_PLL_MASK

static volatile uint32_t *VdStatus(void) { return (volatile uint32_t *)0x40020054UL; }

// The PLL setting in the registers
typedef struct {
	uint32_t N, M, P;
	uint64_t Num, Den;		// Output = Num / Den
} Setting_t;

static Setting_t Decode(void)
{
	Setting_t s;

	assert(SYSCON->AUDPLLNDEC & SYSCON_AUDPLLNDEC_NREQ_MASK);
	assert(SYSCON->AUDPLLPDEC & SYSCON_AUDPLLPDEC_PREQ_MASK);
	assert(SYSCON->AUDPLLMDEC & SYSCON_AUDPLLMDEC_MREQ_MASK);
	s.N = pllDecodeN(SYSCON->AUDPLLNDEC & SYSCON_AUDPLLNDEC_NDEC_MASK);
	s.P = pllDecodeP(SYSCON->AUDPLLPDEC & SYSCON_AUDPLLPDEC_PDEC_MASK);
	s.M = pllDecodeM(SYSCON->AUDPLLMDEC & SYSCON_AUDPLLMDEC_MDEC_MASK);
	assert(s.N >= 1 && s.N <= 256 && s.P >= 1 && s.P <= 32 && s.M >= 1 && s.M <= 32768);

	return s;
}

// Lowest distance from Freq of any output, and the lowest and highest outputs
static void Search(uint64_t Fin, uint64_t Freq, uint64_t *pBest, uint64_t *pLo, uint64_t *pHi)
{
	*pBest = UINT64_MAX;
	*pLo = UINT64_MAX;
	*pHi = 0;
	for (uint64_t p = 1; p <= 32; p++)
	{
		for (uint64_t n = 1; n <= 256 && Fin / n >= REF_MIN; n++)
		{
			for (uint64_t m = CCO_MIN * n / (2 * Fin); m <= 32768; m++)
			{
				uint64_t cco2 = Fin * 2 * m;	// CCO x n

				if (cco2 < CCO_MIN * n)
				{
					continue;
				}
				if (cco2 > CCO_MAX * n)
				{
					break;
				}

				uint64_t f = cco2 / n / (2 * p);
				uint64_t d = f > Freq ? f - Freq : Freq - f;

				*pBest = d < *pBest ? d : *pBest;
				*pLo = f < *pLo ? f : *pLo;
				*pHi = f > *pHi ? f : *pHi;
			}
		}
	}
}

// The registers of a started PLL
static void CheckSetting(uint32_t Fin, uint32_t ClkSel, uint32_t Out)
{
	Setting_t s = Decode();
	uint64_t cco2 = (uint64_t)Fin * 2 * s.M;

	assert(Out != 0);
	assert(cco2 >= CCO_MIN * s.N && cco2 <= CCO_MAX * s.N);
	assert(Fin / s.N >= REF_MIN);
	assert(Out == cco2 / s.N / (2 * s.P));
	assert(SYSCON->AUDPLLCTRL == Lpc546xxPllCtrl(s.M, false));
	assert((SYSCON->AUDPLLCTRL & (SYSCON_AUDPLLCTRL_DIRECTO_MASK | SYSCON_AUDPLLCTRL_DIRECTI_MASK |
								  SYSCON_AUDPLLCTRL_BYPASS_MASK | SYSCON_AUDPLLCTRL_UPLIMOFF_MASK)) == 0);
	assert(SYSCON->AUDPLLCLKSEL == ClkSel);
	assert(SYSCON->AUDPLLFRAC == SYSCON_AUDPLLFRAC_SEL_EXT_MASK);
	assert(SYSCON->PDRUNCFGCLR[0] == SYSCON_PDRUNCFG_PDEN_VD3_MASK);
	assert(SYSCON->PDRUNCFGCLR[1] == AUD_PD && SYSCON->PDRUNCFGSET[1] == AUD_PD);
	assert(Lpc546xxAudioPllFreq() == Out);
}

static uint8_t s_Before[MAP_SIZE];

// Every SYSCON word but the audio PLL ones and the power controls unchanged
static void Snapshot(void)
{
	memcpy(s_Before, (const void *)MAP_BASE, MAP_SIZE);
}

static void CheckOthers(void)
{
	static const size_t own[] = {
		offsetof(SYSCON_Type, AUDPLLCLKSEL), offsetof(SYSCON_Type, AUDPLLCTRL), offsetof(SYSCON_Type, AUDPLLNDEC),
		offsetof(SYSCON_Type, AUDPLLPDEC), offsetof(SYSCON_Type, AUDPLLMDEC), offsetof(SYSCON_Type, AUDPLLFRAC),
		offsetof(SYSCON_Type, PDRUNCFGCLR[0]), offsetof(SYSCON_Type, PDRUNCFGCLR[1]),
		offsetof(SYSCON_Type, PDRUNCFGSET[1]),
	};

	for (size_t o = 0; o < MAP_SIZE; o += 4)
	{
		bool skip = false;

		for (size_t i = 0; i < sizeof(own) / sizeof(own[0]); i++)
		{
			skip |= o == own[i];
		}
		if (!skip && memcmp(s_Before + o, (const uint8_t *)MAP_BASE + o, 4) != 0)
		{
			fprintf(stderr, "SYSCON offset 0x%zx changed\n", o);
			assert(0);
		}
	}
}

static void Fresh(void)
{
	memset((void *)MAP_BASE, 0, MAP_SIZE);
	// Values that must stay
	SYSCON->SYSPLLCTRL = 0x12345;
	SYSCON->MAINCLKSELA = 3;
	SYSCON->PDRUNCFG[1] = 0x55;
	SYSCON->FCLKSEL[6] = 7;
	*VdStatus() = VD3_RDY;
	SYSCON->AUDPLLSTAT = SYSCON_AUDPLLSTAT_LOCK_MASK;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	g_McuOsc.CoreOsc.Freq = 12000000;
	Lpc546xxAudioPllStop();
	SYSCON->PDRUNCFGSET[1] = 0;
	Snapshot();
}

// The PLL frequencies the I2S driver asks: 256 x Fs x K, 12 MHz or more
static void TestAudioRates(void)
{
	static const uint32_t fs[] = { 8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
								   64000, 88200, 96000, 176400, 192000, 15625, 1000, 100 };
	static const struct { OSC_TYPE Type; uint32_t Freq, ClkSel; } osc[] = {
		{ OSC_TYPE_RC, 12000000, 0 }, { OSC_TYPE_XTAL, 12000000, 1 }, { OSC_TYPE_XTAL, 16000000, 1 },
		{ OSC_TYPE_XTAL, 24000000, 1 }, { OSC_TYPE_XTAL, 24576000, 1 },
	};

	for (size_t o = 0; o < sizeof(osc) / sizeof(osc[0]); o++)
	{
		for (size_t i = 0; i < sizeof(fs) / sizeof(fs[0]); i++)
		{
			uint32_t mclk = fs[i] * 256;
			uint32_t k = (12000000 + mclk - 1) / mclk;
			uint32_t want = mclk * (k > 256 ? 256 : k);

			Fresh();
			g_McuOsc.CoreOsc.Type = osc[o].Type;
			g_McuOsc.CoreOsc.Freq = osc[o].Freq;
			Snapshot();

			uint32_t out = Lpc546xxAudioPllStart(want);

			if (out != want)
			{
				fprintf(stderr, "Fin %u: asked %u, got %u\n", osc[o].Freq, want, out);
			}
			assert(out == want);
			CheckSetting(osc[o].Freq, osc[o].ClkSel, out);
			CheckOthers();
		}
	}
	printf("audio rates: PASS\n");
}

// Outputs that are not exact: the closest, the ends of the range
static void TestClosest(void)
{
	// 140671875 is exact only with N 256
	static const uint32_t req[] = { 1, 65536, 4000000, 4300000, 12345678, 22222222, 100000001, 140671875,
									274999999, 275000000, 300000000, 0xFFFFFFFFU };

	for (size_t i = 0; i < sizeof(req) / sizeof(req[0]); i++)
	{
		uint64_t best, lo, hi;

		Fresh();
		Search(12000000, req[i], &best, &lo, &hi);

		uint32_t out = Lpc546xxAudioPllStart(req[i]);
		uint64_t d = out > req[i] ? out - req[i] : req[i] - out;

		if (d != best)
		{
			fprintf(stderr, "asked %u, got %u, best distance %llu\n", req[i], out, (unsigned long long)best);
		}
		assert(d == best);
		if (req[i] < lo)
		{
			assert(out == lo);
		}
		if (req[i] > hi)
		{
			assert(out == hi);
		}
		CheckSetting(12000000, 0, out);
		CheckOthers();
	}
	printf("closest: PASS\n");
}

static volatile bool s_Watched;

static void *Watcher(void *pArg)
{
	(void)pArg;
	s_Watched = false;
	for (long i = 0; i < 2000000000L && SYSCON->PDRUNCFGCLR[1] != AUD_PD; i++);
	if (SYSCON->PDRUNCFGCLR[1] == AUD_PD && SYSCON->PDRUNCFGSET[1] == AUD_PD)
	{
		SYSCON->PDRUNCFGSET[1] = 0;
		s_Watched = true;
	}

	return NULL;
}

static void TestFailures(void)
{
	// Freq 0: off
	Fresh();
	assert(Lpc546xxAudioPllStart(48000 * 256) == 48000 * 256);
	SYSCON->PDRUNCFGSET[1] = 0;
	assert(Lpc546xxAudioPllStart(0) == 0 && Lpc546xxAudioPllFreq() == 0 && SYSCON->PDRUNCFGSET[1] == AUD_PD);

	// VD3 not ready: no PLL register written, the PLL off
	Fresh();
	assert(Lpc546xxAudioPllStart(12288000) == 12288000);
	*VdStatus() = 0;
	SYSCON->AUDPLLCTRL = 0xA5A5;
	SYSCON->AUDPLLMDEC = 0x5A5A;
	SYSCON->PDRUNCFGSET[1] = 0;
	SYSCON->PDRUNCFGCLR[1] = 0;
	assert(Lpc546xxAudioPllStart(12288000) == 0 && Lpc546xxAudioPllFreq() == 0);
	assert(SYSCON->AUDPLLCTRL == 0xA5A5 && SYSCON->AUDPLLMDEC == 0x5A5A);
	assert(SYSCON->PDRUNCFGSET[1] == AUD_PD && SYSCON->PDRUNCFGCLR[1] == 0);
	assert(SYSCON->PDRUNCFGCLR[0] == SYSCON_PDRUNCFG_PDEN_VD3_MASK);

	// No lock: powered down again. The watcher clears the power down of the
	// first Stop once the PLL is powered, during the lock wait.
	Fresh();
	SYSCON->AUDPLLSTAT = 0;
	SYSCON->PDRUNCFGCLR[1] = 0;
	pthread_t th;

	assert(pthread_create(&th, NULL, Watcher, NULL) == 0);
	assert(Lpc546xxAudioPllStart(22579200) == 0 && Lpc546xxAudioPllFreq() == 0);
	assert(pthread_join(th, NULL) == 0 && s_Watched);
	assert(SYSCON->PDRUNCFGSET[1] == AUD_PD && SYSCON->PDRUNCFGCLR[1] == AUD_PD);
	// Lock again
	SYSCON->AUDPLLSTAT = SYSCON_AUDPLLSTAT_LOCK_MASK;
	assert(Lpc546xxAudioPllStart(22579200) == 22579200);
	CheckSetting(12000000, 0, 22579200);
	CheckOthers();

	// Stop
	SYSCON->PDRUNCFGSET[1] = 0;
	Lpc546xxAudioPllStop();
	assert(Lpc546xxAudioPllFreq() == 0 && SYSCON->PDRUNCFGSET[1] == AUD_PD);
	CheckOthers();

	// A clock input of 300 MHz, out of the clk_in range: N 1 gives no M in
	// the CCO range
	Fresh();
	g_McuOsc.CoreOsc.Type = OSC_TYPE_TCXO;
	g_McuOsc.CoreOsc.Freq = 300000000;
	assert(Lpc546xxAudioPllStart(150000000) == 150000000);
	CheckSetting(300000000, 1, 150000000);
	assert(Lpc546xxAudioPllStart(1) != 0);
	CheckSetting(300000000, 1, Lpc546xxAudioPllFreq());
	// 300 MHz from N 1 and M 1 is a CCO of 600 MHz: not used
	assert(Lpc546xxAudioPllStart(300000000) != 0);
	CheckSetting(300000000, 1, Lpc546xxAudioPllFreq());

	// A clock input below 10 kHz x 1: no setting
	Fresh();
	g_McuOsc.CoreOsc.Type = OSC_TYPE_XTAL;
	g_McuOsc.CoreOsc.Freq = 9999;
	assert(Lpc546xxAudioPllStart(12288000) == 0 && Lpc546xxAudioPllFreq() == 0);
	printf("failures: PASS\n");
}

int main(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	assert(mmap((void *)MAP_BASE, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) ==
		   (void *)MAP_BASE);
	TestAudioRates();
	TestClosest();
	TestFailures();
	printf("audio pll: PASS\n");

	return 0;
}
