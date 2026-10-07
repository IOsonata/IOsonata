/**-------------------------------------------------------------------------
@file	cc3xx_intrf.cpp

@brief	DeviceIntrf over the Arm CryptoCell CC3xx register file.

		Implementation notes. StartTx / StartRx save the DevAddr selector and
		open a fresh address phase; the first bytes of the Tx data (or the
		TxSrData of a read) latch a little-endian register offset; the data
		phase then moves 32-bit words at base plus offset. All CC3xx access
		is word access. OpHold / OpRelease is the operation-level exclusion,
		distinct from the per transfer busy the framework manages. Enable and
		Disable hooks power the CryptoCell wrapper by the interface reference
		count.

		An Rx transfer on CC3XX_ADDR_RNG runs the true random generator the
		way the Arm CryptoCell runtime runs it in its full entropy mode
		(llf_rnd_fetrng.c): software reset, sample count, ring oscillator
		length, hardware tests on, watchdog, then one 192 bit sample at a
		time. A sample with a test failure or a watchdog timeout is dropped and
		the next, slower, oscillator setting is tried. A repetition test
		failure drops the sample as well, where the Arm runtime ignores it
		outside its FIPS mode. The transfer fails when the slowest setting
		fails too.

@author	Hoang Nguyen Hoan
@date	Jul. 17, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include "istddef.h"
#include "cc3xx_intrf.h"

static atomic_flag s_HoldFlag = ATOMIC_FLAG_INIT;
static const void *s_pHoldOwner;
static uint32_t s_Offset;				// byte offset latched from address phase
static bool s_bAddrLatched;

static uintptr_t s_Base;				// register file base, cached at Init
static bool s_bRngXfer;					// Rx transfer on CC3XX_ADDR_RNG

// True random generator registers, offsets in the register file
enum : uint32_t {
	REG_RNG_IMR = 0x100U,
	REG_RNG_ISR = 0x104U,
	REG_RNG_ICR = 0x108U,
	REG_TRNG_CONFIG = 0x10CU,
	REG_EHR_DATA = 0x114U,				// 6 words
	REG_NOISE_SOURCE = 0x12CU,
	REG_SAMPLE_CNT = 0x130U,
	REG_TRNG_DEBUG = 0x138U,
	REG_RNG_SW_RESET = 0x140U,
	REG_RNG_CLK = 0x1C4U,
	REG_RNG_WATCHDOG_VAL = 0x1D8U,
};

#define CC3XX_RNG_ISR_EHR_VALID		(1UL << 0)
#define CC3XX_RNG_ISR_AUTOCORR_ERR	(1UL << 1)
#define CC3XX_RNG_ISR_CRNGT_ERR		(1UL << 2)
#define CC3XX_RNG_ISR_VN_ERR		(1UL << 3)
#define CC3XX_RNG_ISR_WATCHDOG		(1UL << 4)

// RNG_IMR: everything masked but EHR_VALID, AUTOCORR_ERR and WATCHDOG
#define CC3XX_RNG_IMR				0xFFFFFFECUL

// TRNG_CONFIG ring oscillator length field
#define CC3XX_RNG_ROSC_LEN_MSK		0x3UL

// Words of one 192 bit sample
#define CC3XX_RNG_EHR_WORDS			6

// Watchdog of one sample in RNG clock cycles, over the sample count: the Arm
// runtime value, 4 times the expected time of 192 bits at 4 samples a bit
// (von Neumann corrector)
#define CC3XX_RNG_WATCHDOG			3072UL

// Writes of the sample count after the RNG reset before giving up
#define CC3XX_RNG_RESET_TRIES		1000

// Sample counts of the ring oscillator lengths, clock cycles between two noise
// samples, from the fastest oscillator: the full entropy mode values of the
// Arm runtime (cc_pal_trng.c). The fourth length is not used there either.
static const uint32_t s_Cc3xxRngSampleCnt[] = { 5000, 1000, 500 };

static inline volatile uint32_t *Cc3xxWord(uint32_t Offset)
{
	return (volatile uint32_t *)(s_Base + Offset);
}

// Clear a copy of random data. Volatile, so the stores are not dropped as
// unused.
static void Cc3xxRngClear(void *p, size_t Len)
{
	volatile uint8_t *d = (volatile uint8_t *)p;

	while (Len-- > 0)
	{
		*d++ = 0;
	}
}

// One 192 bit sample with ring oscillator length Rosc
static bool Cc3xxRngSample(uint32_t Rosc, uint32_t pEhr[CC3XX_RNG_EHR_WORDS])
{
	const uint32_t cnt = s_Cc3xxRngSampleCnt[Rosc];
	const uint32_t wd = cnt * CC3XX_RNG_WATCHDOG;
	const uint32_t done = CC3XX_RNG_ISR_EHR_VALID | CC3XX_RNG_ISR_AUTOCORR_ERR |
						  CC3XX_RNG_ISR_WATCHDOG;
	const uint32_t fail = CC3XX_RNG_ISR_AUTOCORR_ERR | CC3XX_RNG_ISR_CRNGT_ERR |
						  CC3XX_RNG_ISR_VN_ERR | CC3XX_RNG_ISR_WATCHDOG;
	int tries = CC3XX_RNG_RESET_TRIES;

	*Cc3xxWord(REG_RNG_CLK) = 1U;
	*Cc3xxWord(REG_RNG_SW_RESET) = 1U;

	// The reset is over once the sample count holds; it clears the clock
	// enable meanwhile
	do {
		if (tries-- <= 0)
		{
			return false;
		}
		*Cc3xxWord(REG_RNG_CLK) = 1U;
		*Cc3xxWord(REG_SAMPLE_CNT) = cnt;
	} while (*Cc3xxWord(REG_SAMPLE_CNT) != cnt);

	*Cc3xxWord(REG_NOISE_SOURCE) = 0U;
	*Cc3xxWord(REG_RNG_ICR) = 0xFFFFFFFFUL;
	*Cc3xxWord(REG_RNG_IMR) = CC3XX_RNG_IMR;
	*Cc3xxWord(REG_TRNG_CONFIG) = Rosc & CC3XX_RNG_ROSC_LEN_MSK;
	*Cc3xxWord(REG_TRNG_DEBUG) = 0U;
	*Cc3xxWord(REG_RNG_WATCHDOG_VAL) = wd;
	*Cc3xxWord(REG_NOISE_SOURCE) = 1U;

	// The hardware watchdog ends a sample that takes too long; the poll count
	// is a bound in case the RNG clock does not run, longer than it as a poll
	// takes more than a cycle
	uint32_t isr = 0;

	for (uint32_t i = 0; i < wd && (isr & done) == 0; i++)
	{
		isr = *Cc3xxWord(REG_RNG_ISR);
	}

	*Cc3xxWord(REG_NOISE_SOURCE) = 0U;
	*Cc3xxWord(REG_RNG_ICR) = 0xFFFFFFFFUL;

	if ((isr & CC3XX_RNG_ISR_EHR_VALID) == 0 || (isr & fail) != 0)
	{
		return false;
	}

	for (int i = 0; i < CC3XX_RNG_EHR_WORDS; i++)
	{
		pEhr[i] = *Cc3xxWord(REG_EHR_DATA + i * sizeof(uint32_t));
	}

	return true;
}

// Fill pBuff with Len bytes of entropy, zeros and false on failure
static bool Cc3xxRngRead(uint8_t *pBuff, int Len)
{
	uint32_t ehr[CC3XX_RNG_EHR_WORDS];
	uint8_t *p = pBuff;
	int rem = Len;
	uint32_t rosc = 0;
	bool res = true;

	while (rem > 0 && res)
	{
		// From the oscillator length of the last good sample to the slowest
		res = false;
		while (rosc < sizeof(s_Cc3xxRngSampleCnt) / sizeof(s_Cc3xxRngSampleCnt[0]) && res == false)
		{
			res = Cc3xxRngSample(rosc, ehr);
			if (res == false)
			{
				rosc++;
			}
		}

		if (res)
		{
			int n = rem < (int)sizeof(ehr) ? rem : (int)sizeof(ehr);

			memcpy(p, ehr, n);
			p += n;
			rem -= n;
		}
	}

	// Reset the generator before its clock goes off, as before a sample, so
	// that the last sample is not left in EHR_DATA
	*Cc3xxWord(REG_NOISE_SOURCE) = 0U;
	*Cc3xxWord(REG_RNG_ICR) = 0xFFFFFFFFUL;
	*Cc3xxWord(REG_RNG_CLK) = 1U;
	*Cc3xxWord(REG_RNG_SW_RESET) = 1U;
	*Cc3xxWord(REG_RNG_CLK) = 0U;

	Cc3xxRngClear(ehr, sizeof(ehr));
	if (res == false)
	{
		Cc3xxRngClear(pBuff, Len);
	}

	return res;
}

static void Cc3xxIntrfDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	Cc3xxDisable();
}

static void Cc3xxIntrfEnable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	(void)Cc3xxEnable();
}

static uint32_t Cc3xxGetRate(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; return 0; }
static uint32_t Cc3xxSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate) {
	(void)pDevIntrf; (void)Rate; return 0;
}

static bool Cc3xxStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	if (DevAddr == CC3XX_ADDR_RNG)
	{
		// The generator needs the CryptoCell powered, which an engine Enable
		// does through the reference count
		if (atomic_load(&pIntrf->EnCnt) == 0)
		{
			return false;
		}

		// A draw takes tens of msec: it holds the CryptoCell like an engine
		// operation, so that an operation is refused instead of losing its
		// register transfers to the draw. Engines draw their random values
		// before they hold it.
		if (atomic_flag_test_and_set(&s_HoldFlag))
		{
			return false;
		}
		s_pHoldOwner = &s_bRngXfer;

		// A PKA fault powers the CryptoCell off while the count says on
		if (!Cc3xxEnable())
		{
			s_pHoldOwner = nullptr;
			atomic_flag_clear(&s_HoldFlag);

			return false;
		}
		s_bRngXfer = true;

		return true;
	}

	// Restart condition inside a read: keep the offset latched by the
	// preceding address phase.
	s_bRngXfer = false;

	return true;
}

static int Cc3xxRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	(void)pIntrf;
	if (pBuff == nullptr || BuffLen <= 0)
	{
		return 0;
	}
	if (s_bRngXfer)
	{
		return Cc3xxRngRead(pBuff, BuffLen) ? BuffLen : 0;
	}
	if (!s_bAddrLatched)
	{
		return 0;
	}
	volatile uint32_t *w = Cc3xxWord(s_Offset);
	for (int i = 0; i + 4 <= BuffLen; i += 4)
	{
		uint32_t v = w[i / 4];
		pBuff[i] = (uint8_t)v;
		pBuff[i + 1] = (uint8_t)(v >> 8);
		pBuff[i + 2] = (uint8_t)(v >> 16);
		pBuff[i + 3] = (uint8_t)(v >> 24);
	}
	return BuffLen;
}

static void Cc3xxStopRx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	s_bAddrLatched = false;
	if (s_bRngXfer)
	{
		s_bRngXfer = false;
		s_pHoldOwner = nullptr;
		atomic_flag_clear(&s_HoldFlag);
	}
}

static bool Cc3xxStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	// The random generator is read only
	if (DevAddr == CC3XX_ADDR_RNG)
	{
		return false;
	}
	// Single register file: the selector is saved for protocol symmetry and
	// a fresh address phase opens.
	s_bAddrLatched = false;
	s_bRngXfer = false;
	return true;
}

static int Cc3xxTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	(void)pIntrf;
	if (pData == nullptr || DataLen <= 0)
	{
		return 0;
	}
	int consumed = 0;
	if (!s_bAddrLatched)
	{
		int n = (DataLen < 4) ? DataLen : 4;
		uint32_t off = 0U;
		for (int i = 0; i < n; i++)
		{
			off |= (uint32_t)pData[i] << (8 * i);
		}
		s_Offset = off;
		s_bAddrLatched = true;
		consumed = n;
	}
	int len = DataLen - consumed;
	const uint8_t *p = &pData[consumed];
	volatile uint32_t *w = Cc3xxWord(s_Offset);
	for (int i = 0; i + 4 <= len; i += 4)
	{
		w[i / 4] = (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8) |
				   ((uint32_t)p[i + 2] << 16) | ((uint32_t)p[i + 3] << 24);
	}
	return DataLen;
}

static int Cc3xxTxSrData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	return Cc3xxTxData(pIntrf, pData, DataLen);
}

static void Cc3xxStopTx(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	s_bAddrLatched = false;
}

static void Cc3xxReset(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; }
static void Cc3xxPowerOff(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
	Cc3xxDisable();
}
static void *Cc3xxGetHandle(DevIntrf_t * const pDevIntrf) { (void)pDevIntrf; return nullptr; }

bool Cc3xxIntrf::Init(void)
{
	vDevIntrf.pDevData = this;
	vDevIntrf.EvtCB = nullptr;
	atomic_flag_clear(&vDevIntrf.bBusy);
	vDevIntrf.MaxRetry = 5;
	// The user count starts at zero: engines are the users. The first engine
	// Enable powers the wrapper (0 to 1) and the last engine Disable powers
	// it off, through the DeviceIntrf reference count.
	atomic_store(&vDevIntrf.EnCnt, 0);
	vDevIntrf.Type = DEVINTRF_TYPE_CRYPTO;
	vDevIntrf.bDma = false;
	vDevIntrf.bIntEn = false;
	atomic_store(&vDevIntrf.bTxReady, true);
	atomic_store(&vDevIntrf.bNoStop, false);
	vDevIntrf.Disable = Cc3xxIntrfDisable;
	vDevIntrf.Enable = Cc3xxIntrfEnable;
	vDevIntrf.GetRate = Cc3xxGetRate;
	vDevIntrf.SetRate = Cc3xxSetRate;
	vDevIntrf.StartRx = Cc3xxStartRx;
	vDevIntrf.RxData = Cc3xxRxData;
	vDevIntrf.StopRx = Cc3xxStopRx;
	vDevIntrf.StartTx = Cc3xxStartTx;
	vDevIntrf.TxData = Cc3xxTxData;
	vDevIntrf.TxSrData = Cc3xxTxSrData;
	vDevIntrf.StopTx = Cc3xxStopTx;
	vDevIntrf.Reset = Cc3xxReset;
	vDevIntrf.PowerOff = Cc3xxPowerOff;
	vDevIntrf.GetHandle = Cc3xxGetHandle;

	// Wiring only: cache the target base and leave the wrapper unpowered.
	// Power follows the user count.
	s_Base = Cc3xxBase();
	return true;
}

bool Cc3xxIntrf::OpHold(const void *pOwner)
{
	if (pOwner == nullptr || atomic_flag_test_and_set(&s_HoldFlag))
	{
		return false;
	}
	s_pHoldOwner = pOwner;
	return true;
}

bool Cc3xxIntrf::OpRelease(const void *pOwner)
{
	if (pOwner == nullptr || s_pHoldOwner != pOwner)
	{
		return false;
	}
	s_pHoldOwner = nullptr;
	atomic_flag_clear(&s_HoldFlag);
	return true;
}

Cc3xxIntrf *Cc3xxIntrfInstance(void)
{
	static Cc3xxIntrf s_Cc3xxIntrf;
	static bool s_bInit = false;

	if (!s_bInit)
	{
		s_bInit = s_Cc3xxIntrf.Init();
	}
	return s_bInit ? &s_Cc3xxIntrf : nullptr;
}
