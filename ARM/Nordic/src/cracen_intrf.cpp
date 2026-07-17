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

		ModuleHold provides operation-level exclusion across CryptoMaster,
		BA414EP and the RNG. It is separate from the DeviceIntrf transfer busy
		flag because one crypto operation contains many register transfers.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "nrf.h"

#include "cracen_intrf.h"

#include "nrfx_cracen.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#define BA414EP_CRYPTORAM_OFFSET	0x8000U

#ifndef CRACEN_SD_RNG_POLL_LIMIT
#define CRACEN_SD_RNG_POLL_LIMIT	1000000U
#endif

static volatile uint8_t *s_pPkeRegBase;
static volatile uint8_t *s_pCmRegBase;
static volatile uint8_t *s_pMemBase;
static volatile uint8_t *s_pRegBase;

static volatile uint8_t *s_pXferBase;
static bool s_bXferMem;
static uint32_t s_Offset;
static bool s_bAddrLatched;
static bool s_bRngXfer;
static bool s_bRngHeld;
static bool s_bDrbgInit;

// One operation may own CRACEN at a time. s_HeldMask is zero when there is no
// owner. s_PreviousEnable remembers whether the selected module was already
// enabled before the hold, so release does not power down another subsystem's
// pre-existing setup.
static atomic_flag s_HoldFlag = ATOMIC_FLAG_INIT;
static uint32_t s_HeldMask;
static uint32_t s_PreviousEnable;
static const void *s_pHoldOwner;

static void CracenSelectBase(uint32_t DevAddr)
{
	s_bXferMem = (DevAddr == CRACEN_ADDR_MEM);
	s_pXferBase = s_bXferMem ? s_pMemBase : s_pRegBase;
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
	if (DevAddr == CRACEN_ADDR_RNG)
	{
		bool held = false;
		if (!CracenSdEnabled())
		{
			CracenIntrf *p = (CracenIntrf *)pIntrf->pDevData;
			if (p == nullptr || !p->ModuleHold(CRACEN_MODULE_RNG, pIntrf))
			{
				return false;
			}
			held = true;
		}

		// Publish transfer state only after acquisition succeeds. A failed
		// StartRx is not followed by StopRx, so stale state must not be left.
		s_bRngHeld = held;
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
		if (!s_bRngHeld)
		{
			return CracenSdRandFill(pBuff, BuffLen);
		}

		if (!s_bDrbgInit)
		{
			int rc = nrfx_cracen_init();
			if (rc != 0 && rc != -EALREADY)
			{
				return 0;
			}
			s_bDrbgInit = true;
		}

		if (nrfx_cracen_ctr_drbg_random_get(pBuff, (size_t)BuffLen) != 0)
		{
			return 0;
		}
		return BuffLen;
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
	if (s_bRngXfer)
	{
		if (s_bRngHeld)
		{
			CracenIntrf *p = (CracenIntrf *)pIntrf->pDevData;
			if (p != nullptr)
			{
				(void)p->ModuleRelease(pIntrf);
			}
			s_bRngHeld = false;
		}
		s_bRngXfer = false;
		return;
	}
	s_bAddrLatched = false;
}

static uint32_t CracenModuleMask(uint32_t Module)
{
	switch (Module)
	{
	case CRACEN_MODULE_CRYPTOMASTER:
		return CRACEN_ENABLE_CRYPTOMASTER_Msk;
	case CRACEN_MODULE_PKEIKG:
		return CRACEN_ENABLE_PKEIKG_Msk;
	case CRACEN_MODULE_RNG:
		return CRACEN_ENABLE_RNG_Msk;
	default:
		return 0U;
	}
}

bool CracenIntrf::ModuleHold(uint32_t Module, const void *pOwner)
{
	const uint32_t mask = CracenModuleMask(Module);
	if (mask == 0U || pOwner == nullptr || atomic_flag_test_and_set(&s_HoldFlag))
	{
		return false;
	}

	s_HeldMask = mask;
	s_PreviousEnable = NRF_CRACEN->ENABLE & mask;
	s_pHoldOwner = pOwner;
	s_pRegBase = (Module == CRACEN_MODULE_CRYPTOMASTER) ? s_pCmRegBase
											 : s_pPkeRegBase;
	NRF_CRACEN->ENABLE |= mask;
	__DMB();
	return true;
}

// Release only the hold owned by pOwner and restore the prior module-enable
// state. A stray release can no longer clear another engine's exclusion flag.
static bool CracenHoldRelease(const void *pOwner)
{
	const uint32_t mask = s_HeldMask;
	if (mask == 0U || pOwner == nullptr || s_pHoldOwner != pOwner)
	{
		return false;
	}

	if (s_PreviousEnable == 0U)
	{
		NRF_CRACEN->ENABLE &= ~mask;
		__DMB();
	}
	s_HeldMask = 0U;
	s_PreviousEnable = 0U;
	s_pHoldOwner = nullptr;
	atomic_flag_clear(&s_HoldFlag);
	return true;
}

bool CracenIntrf::ModuleRelease(const void *pOwner)
{
	return CracenHoldRelease(pOwner);
}

bool CracenIntrf::ModuleReset(const void *pOwner)
{
	const uint32_t mask = s_HeldMask;
	if (mask == 0U || pOwner == nullptr || s_pHoldOwner != pOwner)
	{
		return false;
	}

	// The caller retains the global hold while the selected module is forced
	// off and back on. No sibling engine can enter during timeout recovery.
	NRF_CRACEN->ENABLE &= ~mask;
	__DMB();
	NRF_CRACEN->ENABLE |= mask;
	__DMB();
	return true;
}

static void CracenDummyDisable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
static void CracenDummyEnable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
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
static void CracenReset(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	s_bAddrLatched = false;
	s_bRngXfer = false;
	// Owner-aware abort: when the interface's own RNG transfer holds the
	// module, that hold must be released here, or the single-owner flag and
	// mask would be stranded with no owner left to release them. A hold
	// owned by another component (an operation-level user) is not touched.
	if (s_bRngHeld)
	{
		s_bRngHeld = false;
		(void)CracenHoldRelease(pIntrf);
	}
}
static void CracenPowerOff(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	// Operation owners control module power through ModuleHold/ModuleRelease.
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
	s_pRegBase = s_pPkeRegBase;
	s_pXferBase = s_pPkeRegBase;
	s_bXferMem = false;
	s_Offset = 0U;
	s_bAddrLatched = false;
	s_bRngXfer = false;
	s_bRngHeld = false;
	s_bDrbgInit = false;
	s_HeldMask = 0U;
	s_PreviousEnable = 0U;

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
	atomic_flag_clear(&s_HoldFlag);
	atomic_store(&vDevIntrf.EnCnt, 0);
	atomic_store(&vDevIntrf.bTxReady, true);
	atomic_store(&vDevIntrf.bNoStop, false);

	vDevIntrf.Disable = CracenDummyDisable;
	vDevIntrf.Enable = CracenDummyEnable;
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
