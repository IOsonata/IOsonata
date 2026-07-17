/**-------------------------------------------------------------------------
@file	cracen_intrf.cpp

@brief	Device interface to the Nordic on-die crypto core (CRACEN).

		Leaf DeviceIntrf that owns the CRACENCORE register and operand memory
		access. CRACEN is not a crypto engine; it is the access path the Silex
		engines sit on. An engine reaches it through the inherited Device Read /
		Write, exactly as a sensor reaches an SPI or I2C bus.

		The transfer follows the same pattern as SPI and I2C: StartTx or StartRx
		receives the DevAddr and saves it (here, which sub-block base the access
		lands in, the way SPI saves the chip-select). TxSrData latches the byte
		offset from the address phase; TxData writes and RxData reads at the saved
		base plus the latched offset. Registers are accessed as 32-bit words and
		operand memory byte-wise, matching the hardware.

		ModuleHold enables a module for the whole operation and takes the busy
		flag so the engines serialize; ModuleRelease disables and releases. The
		held module also selects which register sub-block the register base is.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf.h"

#include "cracen_intrf.h"

// The random generator is drawn through the vendor CTR-DRBG driver. When a
// SoftDevice (S115/S145) is enabled it owns the entropy hardware; draws must
// then go through the SoftDevice entropy pool instead of the driver.
#include "nrfx_cracen.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

// Operand memory sub-block offset from the CRACENCORE base.
#define BA414EP_CRYPTORAM_OFFSET	0x8000U

// The two fixed register sub-block bases and the operand memory base.
static volatile uint8_t *s_pPkeRegBase;
static volatile uint8_t *s_pCmRegBase;
static volatile uint8_t *s_pMemBase;

// Active register base (the held module's sub-block).
static volatile uint8_t *s_pRegBase;

// Saved transfer state, set by StartTx / StartRx and the address phase, used by
// TxData / RxData. This mirrors how SPI saves the chip-select in StartTx.
static volatile uint8_t *s_pXferBase;	// base saved from DevAddr
static bool s_bXferMem;					// operand memory space (byte access)
static uint32_t s_Offset;				// byte offset latched from address phase
static bool s_bAddrLatched;
static bool s_bRngXfer;					// current Rx is an entropy read
static bool s_bRngHeld;					// RNG module held for this entropy read
static bool s_bDrbgInit;				// CTR-DRBG brought up once

static void CracenSelectBase(uint32_t DevAddr)
{
	s_bXferMem = (DevAddr == CRACEN_ADDR_MEM);
	s_pXferBase = s_bXferMem ? s_pMemBase : s_pRegBase;
}

static bool CracenStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	// Save which sub-block this transfer lands in, and start a fresh address
	// phase. The module is already held enabled by ModuleHold.
	CracenSelectBase(DevAddr);
	s_bAddrLatched = false;
	return true;
}

// True when a SoftDevice is present in the build and enabled at run time. The
// SoftDevice owns the entropy hardware while enabled; direct access would
// assert the stack, so draws must go through its entropy pool.
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
		// Entropy read. With the SoftDevice enabled the draw goes through its
		// pool and no module hold is needed. Otherwise hold the RNG module for
		// the transfer; when another engine holds the core this fails and the
		// framework retry (MaxRetry) reattempts the whole transfer.
		s_bRngXfer = true;
		s_bRngHeld = false;
		if (!CracenSdEnabled())
		{
			CracenIntrf *p = (CracenIntrf *)pIntrf->pDevData;
			if (!p->ModuleHold(CRACEN_MODULE_RNG))
			{
				return false;
			}
			s_bRngHeld = true;
		}
		return true;
	}
	// Restart condition inside a read: keep the offset latched by the preceding
	// address phase; only re-select the saved base.
	s_bRngXfer = false;
	CracenSelectBase(DevAddr);
	return true;
}

// Address phase: latch the byte offset (little-endian) the engine sends before
// the data. Used for both the read address phase (TxSrData) and a write where
// the address leads the data in one buffer.
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

// Fill the buffer from the SoftDevice entropy pool. The pool refills at a
// finite rate, so take what is available until the request is filled.
static int CracenSdRandFill(uint8_t *pBuff, int BuffLen)
{
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	int idx = 0;
	while (idx < BuffLen)
	{
		uint8_t avail = 0;
		(void)sd_rand_application_bytes_available_get(&avail);
		if (avail == 0)
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
	return BuffLen;
#else
	(void)pBuff; (void)BuffLen;
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
		// Bring the CTR-DRBG up once, then draw.
		if (!s_bDrbgInit)
		{
			if (nrfx_cracen_init() != 0)
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
		const volatile uint32_t *w = (const volatile uint32_t *)(s_pXferBase + s_Offset);
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
			p->ModuleRelease();
			s_bRngHeld = false;
		}
		s_bRngXfer = false;
		return;
	}
	s_bAddrLatched = false;
}

// Map a CRACEN_MODULE to its enable bit.
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

// Operation-level hold, separate from the per-transfer busy flag. ModuleHold
// takes this for the whole operation so the engines serialize against one
// another; each Device Read / Write inside the operation still takes and
// releases the transfer busy flag through StartTx / StopTx.
static atomic_flag s_HoldFlag;

bool CracenIntrf::ModuleHold(uint32_t Module)
{
	const uint32_t mask = CracenModuleMask(Module);
	if (mask == 0U)
	{
		return false;
	}
	if (atomic_flag_test_and_set(&s_HoldFlag))
	{
		return false;
	}
	// Point register access at the held module's sub-block. The public-key and
	// symmetric engines are separate register sub-blocks on this die.
	s_pRegBase = (Module == CRACEN_MODULE_CRYPTOMASTER) ? s_pCmRegBase
													    : s_pPkeRegBase;
	NRF_CRACEN->ENABLE |= mask;
	return true;
}

void CracenIntrf::ModuleRelease(void)
{
	NRF_CRACEN->ENABLE &= ~(CRACEN_ENABLE_CRYPTOMASTER_Msk |
							CRACEN_ENABLE_PKEIKG_Msk |
							CRACEN_ENABLE_RNG_Msk);
	atomic_flag_clear(&s_HoldFlag);
}

static void CracenDummyDisable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
static void CracenDummyEnable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
static uint32_t CracenGetRate(DevIntrf_t * const pIntrf) { (void)pIntrf; return 0U; }
static uint32_t CracenSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	(void)pIntrf; (void)Rate; return 0U;
}

bool CracenIntrf::Init(void)
{
	// Register sub-block bases: the public-key engine and the symmetric engine
	// are separate register blocks; operand memory is the public-key operand RAM.
	s_pPkeRegBase = (volatile uint8_t *)&NRF_CRACENCORE->PK;
	s_pCmRegBase = (volatile uint8_t *)NRF_CRACENCORE;
	s_pMemBase = (volatile uint8_t *)((uintptr_t)NRF_CRACENCORE +
									 BA414EP_CRYPTORAM_OFFSET);
	s_pRegBase = s_pPkeRegBase;
	s_bAddrLatched = false;

	vDevIntrf.pDevData = this;
	vDevIntrf.IntPrio = 0;
	vDevIntrf.EvtCB = nullptr;
	vDevIntrf.MaxRetry = 0;
	vDevIntrf.Type = DEVINTRF_TYPE_CRYPTO;
	vDevIntrf.bDma = false;
	vDevIntrf.bIntEn = false;
	vDevIntrf.MaxTrxLen = 0;
	atomic_flag_clear(&vDevIntrf.bBusy);
	atomic_flag_clear(&s_HoldFlag);
	atomic_store(&vDevIntrf.EnCnt, 0);

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
	return true;
}

CracenIntrf *CracenIntrfInstance(void)
{
	static CracenIntrf s_Instance;
	static bool s_bInit = false;
	if (!s_bInit)
	{
		s_Instance.Init();
		s_bInit = true;
	}
	return &s_Instance;
}
