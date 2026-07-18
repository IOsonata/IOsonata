/**-------------------------------------------------------------------------
@file	cracen_intrf.cpp

@brief	Device interface to the Nordic on-die crypto core (CRACEN).

		Leaf DeviceIntrf that owns the CRACENCORE register and operand memory
		access. CRACEN is not a crypto engine; it is the access path the Silex
		engines sit on. An engine reaches it through the inherited Device Read /
		Write, exactly as a sensor reaches an SPI or I2C bus.

		The transfer follows the same pattern as SPI and I2C: StartTx or StartRx
		receives the DevAddr and saves it. TxSrData latches the byte offset from
		the address phase; TxData writes and RxData reads at the saved base plus
		the latched offset. Registers are accessed as 32-bit words and operand
		memory byte-wise, matching the hardware.

		Every transfer is self-addressing through the DevAddr module
		selector, so no cross-operation state lives here. Module power is
		owned by the transport Enable/Disable through the enable reference
		count. Normal engine recovery toggles only the selected engine; the
		interface Reset remains the whole-wrapper recovery path.

@author	Hoang Nguyen Hoan
@date	Jul 2026

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
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <string.h>

#include "nrf.h"

#include "cracen_intrf.h"
#include "cracen_ba414e_ucode.h"
#include "crypto/ba414ep.h"
#include "crypto/cryptomaster.h"

#include "nrfx_cracen.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#ifndef CRACEN_SD_RNG_POLL_LIMIT
#define CRACEN_SD_RNG_POLL_LIMIT	1000000U
#endif

#ifndef CRACEN_RESET_LOCK_SPINS
#define CRACEN_RESET_LOCK_SPINS		100000U
#endif

#define CRACEN_ENABLE_ALL_Msk	(CRACEN_ENABLE_CRYPTOMASTER_Msk | \
								 CRACEN_ENABLE_PKEIKG_Msk | \
								 CRACEN_ENABLE_RNG_Msk)

// PKE code RAM offset from the CRACENCORE base.
#define BA414EP_CODE_OFFSET		0xC000U

static volatile uint8_t *s_pPkeRegBase;
static volatile uint8_t *s_pCmRegBase;
static volatile uint8_t *s_pMemBase;

static volatile uint8_t *s_pXferBase;
static bool s_bXferMem;
static uint32_t s_Offset;
static bool s_bAddrLatched;
static bool s_bRngXfer;
static bool s_bDrbgInit;


static void CracenSelectBase(uint32_t DevAddr)
{
	s_bXferMem = (DevAddr == BA414EP_ADDR_MEM);
	s_pXferBase = s_bXferMem ? s_pMemBase :
				  (DevAddr == CRYPTOMASTER_ADDR_REG) ? s_pCmRegBase : s_pPkeRegBase;
}

static bool CracenStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	CracenSelectBase(DevAddr);
	s_bAddrLatched = false;
	return true;
}

static bool CracenSdEnabled(void)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	uint8_t en = 0;
	(void)sd_softdevice_is_enabled(&en);
	return en != 0;
#else
	return false;
#endif
}

static bool CracenStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	if (DevAddr == CRACEN_ADDR_RNG)
	{
		s_bRngXfer = true;
		return true;
	}

	s_bRngXfer = false;
	CracenSelectBase(DevAddr);
	return true;
}

static int CracenTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
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
	if (s_bXferMem)
	{
		volatile uint8_t *d = s_pXferBase + s_Offset;
		for (int i = 0; i < len; i++)
		{
			d[i] = p[i];
		}
	}
	else
	{
		volatile uint32_t *w = (volatile uint32_t *)(s_pXferBase + s_Offset);
		for (int i = 0; i + 4 <= len; i += 4)
		{
			w[i / 4] = (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8) |
					   ((uint32_t)p[i + 2] << 16) | ((uint32_t)p[i + 3] << 24);
		}
	}
	return DataLen;
}

static int CracenTxSrData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	return CracenTxData(pIntrf, pData, DataLen);
}

static int CracenSdRandFill(uint8_t *pBuff, int BuffLen)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	int idx = 0;
	uint32_t polls = CRACEN_SD_RNG_POLL_LIMIT;
	while (idx < BuffLen && polls-- > 0U)
	{
		uint8_t avail = 0;
		if (sd_rand_application_bytes_available_get(&avail) != NRF_SUCCESS ||
			avail == 0U)
		{
			continue;
		}

		int n = BuffLen - idx;
		if (n > (int)avail)
		{
			n = avail;
		}
		if (sd_rand_application_vector_get(&pBuff[idx], (uint8_t)n) == NRF_SUCCESS)
		{
			idx += n;
		}
	}
	return idx == BuffLen ? BuffLen : 0;
#else
	(void)pBuff;
	(void)BuffLen;
	return 0;
#endif
}

static int CracenRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	(void)pIntrf;
	if (pBuff == nullptr || BuffLen <= 0)
	{
		return 0;
	}

	if (s_bRngXfer)
	{
		if (CracenSdEnabled())
		{
			return CracenSdRandFill(pBuff, BuffLen);
		}

		int filled = BuffLen;
		if (!s_bDrbgInit)
		{
			int rc = nrfx_cracen_init();
			if (rc == 0 || rc == -EALREADY)
			{
				s_bDrbgInit = true;
			}
			else
			{
				filled = 0;
			}
		}
		if (filled != 0 &&
			nrfx_cracen_ctr_drbg_random_get(pBuff, (size_t)BuffLen) != 0)
		{
			filled = 0;
		}

		// The wrapped nrfx driver disables the RNG and CryptoMaster module
		// bits as a side effect of every draw. Repair the power state the
		// transport Enable established, at the one place the damage occurs.
		if (atomic_load(&pIntrf->EnCnt) > 0)
		{
			NRF_CRACEN->ENABLE |= CRACEN_ENABLE_ALL_Msk;
			__DMB();
		}
		return filled;
	}

	if (!s_bAddrLatched)
	{
		return 0;
	}

	if (s_bXferMem)
	{
		const volatile uint8_t *d = s_pXferBase + s_Offset;
		for (int i = 0; i < BuffLen; i++)
		{
			pBuff[i] = d[i];
		}
	}
	else
	{
		const volatile uint32_t *w =
			(const volatile uint32_t *)(s_pXferBase + s_Offset);
		for (int i = 0; i + 4 <= BuffLen; i += 4)
		{
			uint32_t v = w[i / 4];
			pBuff[i]     = (uint8_t)v;
			pBuff[i + 1] = (uint8_t)(v >> 8);
			pBuff[i + 2] = (uint8_t)(v >> 16);
			pBuff[i + 3] = (uint8_t)(v >> 24);
		}
	}
	return BuffLen;
}

static void CracenStopTx(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	s_bAddrLatched = false;
}

static void CracenStopRx(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	if (s_bRngXfer)
	{
		s_bRngXfer = false;
		return;
	}
	s_bAddrLatched = false;
}

// PKE code RAM lives at NRF_CRACENCORE + BA414EP_CODE_OFFSET. The Silex
// BA414EP on this die is the microcoded configuration: the engine cannot
// sequence any operation until its code RAM is filled. Initialization verifies
// the complete retained image before taking the fast path, so corruption in a
// middle word cannot be accepted as a valid program.
static bool CracenLoadPkeMicrocode(void)
{
	volatile uint32_t *pCode = (volatile uint32_t *)
		((uintptr_t)NRF_CRACENCORE + BA414EP_CODE_OFFSET);
	const size_t words = CRACEN_BA414E_UCODE_WORDS;

	bool match = true;
	for (size_t i = 0; i < words; i++)
	{
		if (pCode[i] != s_CracenBa414eUcode[i])
		{
			match = false;
			break;
		}
	}
	if (match)
	{
		return true;
	}

	for (size_t i = 0; i < words; i++)
	{
		pCode[i] = s_CracenBa414eUcode[i];
	}
	__DSB();
	for (size_t i = 0; i < words; i++)
	{
		if (pCode[i] != s_CracenBa414eUcode[i])
		{
			return false;
		}
	}
	return true;
}

// Transport power, driven by the EnCnt reference count: the first engine
// Enable powers the whole core (all three modules, the way the sdk-nrf
// cracen_acquire does), the last Disable powers it off. Retained state
// (microcode RAM, handover, DRBG) survives these cycles. NRF_CRACEN is the
// Nordic wrapper and lives only here; the Silex engines never touch it.
static void CracenCoreDisable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	NRF_CRACEN->ENABLE &= ~CRACEN_ENABLE_ALL_Msk;
	__DMB();
}

static void CracenCoreEnable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	NRF_CRACEN->ENABLE |= CRACEN_ENABLE_ALL_Msk;
	__DMB();
}
static uint32_t CracenGetRate(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	return 0U;
}
static uint32_t CracenSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	(void)pIntrf;
	(void)Rate;
	return 0U;
}

static bool CracenResetModule(DevIntrf_t * const pIntrf, uint32_t Mask)
{
	if (pIntrf == nullptr || Mask == 0U)
	{
		return false;
	}

	uint32_t attempts = __get_IPSR() == 0U ? CRACEN_RESET_LOCK_SPINS : 1U;
	while (atomic_flag_test_and_set(&pIntrf->bBusy))
	{
		if (attempts-- == 0U)
		{
			return false;
		}
		__asm volatile("" ::: "memory");
	}

	bool ok = true;
	NRF_CRACEN->ENABLE &= ~Mask;
	__DMB();
	if (atomic_load(&pIntrf->EnCnt) > 0)
	{
		NRF_CRACEN->ENABLE |= Mask;
		__DMB();
		ok = (NRF_CRACEN->ENABLE & Mask) == Mask;
		if (ok && (Mask & CRACEN_ENABLE_PKEIKG_Msk) != 0U)
		{
			ok = CracenLoadPkeMicrocode();
		}
	}

	s_bAddrLatched = false;
	s_bRngXfer = false;
	if ((Mask & CRACEN_ENABLE_RNG_Msk) != 0U)
	{
		s_bDrbgInit = false;
	}
	atomic_flag_clear(&pIntrf->bBusy);
	return ok;
}

// Interface reset is the wrapper-level escape hatch. Engine drivers use the
// strong platform hooks below so normal recovery does not disturb another
// engine that is operating concurrently.
static void CracenReset(DevIntrf_t * const pIntrf)
{
	(void)CracenResetModule(pIntrf, CRACEN_ENABLE_ALL_Msk);
}
static void CracenPowerOff(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	// Module power is owned by the transport Enable/Disable (whole core on).
}
static void *CracenGetHandle(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	return (void *)NRF_CRACENCORE;
}

bool CracenIntrf::Init(void)
{
	s_pPkeRegBase = (volatile uint8_t *)&NRF_CRACENCORE->PK;
	s_pCmRegBase = (volatile uint8_t *)NRF_CRACENCORE;
	s_pMemBase = (volatile uint8_t *)((uintptr_t)NRF_CRACENCORE +
									 BA414EP_CRYPTORAM_OFFSET);

	// Load the BA414EP microcode once, the way the Nordic power-up init does
	// (cracen_init in sdk-nrf hardware.c): power the PKE module, fill and
	// verify the code RAM, restore the module power state. The code RAM is
	// retained across ENABLE cycles; a chip reset clears it and also re-runs
	// Init. The IK handover and readiness are the generic engine's concern,
	// proven by its first Enable.
	const uint32_t prev = NRF_CRACEN->ENABLE & CRACEN_ENABLE_PKEIKG_Msk;
	NRF_CRACEN->ENABLE |= CRACEN_ENABLE_PKEIKG_Msk;
	__DMB();
	const bool bUcodeOk = CracenLoadPkeMicrocode();
	if (prev == 0U)
	{
		NRF_CRACEN->ENABLE &= ~CRACEN_ENABLE_PKEIKG_Msk;
		__DMB();
	}
	if (!bUcodeOk)
	{
		return false;
	}

	s_pXferBase = s_pPkeRegBase;
	s_bXferMem = false;
	s_Offset = 0U;
	s_bAddrLatched = false;
	s_bRngXfer = false;
	s_bDrbgInit = false;

	memset(&vDevIntrf, 0, sizeof(vDevIntrf));
	vDevIntrf.pDevData = this;
	vDevIntrf.IntPrio = 0;
	vDevIntrf.EvtCB = nullptr;
	vDevIntrf.MaxRetry = 5;
	vDevIntrf.Type = DEVINTRF_TYPE_CRYPTO;
	vDevIntrf.bDma = false;
	vDevIntrf.bIntEn = false;
	vDevIntrf.MaxTrxLen = 0;
	atomic_flag_clear(&vDevIntrf.bBusy);
	atomic_store(&vDevIntrf.EnCnt, 0);
	atomic_store(&vDevIntrf.bTxReady, true);
	atomic_store(&vDevIntrf.bNoStop, false);

	vDevIntrf.Disable = CracenCoreDisable;
	vDevIntrf.Enable = CracenCoreEnable;
	vDevIntrf.GetRate = CracenGetRate;
	vDevIntrf.SetRate = CracenSetRate;
	vDevIntrf.StartRx = CracenStartRx;
	vDevIntrf.RxData = CracenRxData;
	vDevIntrf.StopRx = CracenStopRx;
	vDevIntrf.StartTx = CracenStartTx;
	vDevIntrf.TxData = CracenTxData;
	vDevIntrf.TxSrData = CracenTxSrData;
	vDevIntrf.StopTx = CracenStopTx;
	vDevIntrf.Reset = CracenReset;
	vDevIntrf.PowerOff = CracenPowerOff;
	vDevIntrf.GetHandle = CracenGetHandle;
	return true;
}

CracenIntrf *CracenIntrfInstance(void)
{
	static CracenIntrf s_Instance;
	static bool s_bInit = false;
	if (!s_bInit)
	{
		s_bInit = s_Instance.Init();
	}
	return s_bInit ? &s_Instance : nullptr;
}

extern "C" bool Ba414epPlatformReset(DeviceIntrf *pIntrf)
{
	CracenIntrf *pCracen = CracenIntrfInstance();
	if (pIntrf == nullptr || pCracen == nullptr || pIntrf != pCracen)
	{
		return false;
	}
	DevIntrf_t *pHandle = *pCracen;
	return CracenResetModule(pHandle, CRACEN_ENABLE_PKEIKG_Msk);
}

extern "C" bool CryptoMasterPlatformReset(DeviceIntrf *pIntrf)
{
	CracenIntrf *pCracen = CracenIntrfInstance();
	if (pIntrf == nullptr || pCracen == nullptr || pIntrf != pCracen)
	{
		return false;
	}
	DevIntrf_t *pHandle = *pCracen;
	return CracenResetModule(pHandle, CRACEN_ENABLE_CRYPTOMASTER_Msk);
}
