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

		CoreAcquire provides operation-level exclusion across CryptoMaster,
		BA414EP and the RNG. It is separate from the DeviceIntrf transfer busy
		flag because one crypto operation contains many register transfers.
		Module power is owned by the transport Enable/Disable, not the lock.

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
// owner. Module power is held on by the transport Enable (whole core on); the
// lock below only serializes the engines and selects the sub-block base.
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
			if (p == nullptr || !p->CoreAcquire(CRACEN_MODULE_RNG, pIntrf))
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
				(void)p->CoreRelease(pIntrf);
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

// The nRF54L15 pairs the BA414EP with the identity key generator in one
// PKEIKG module. After the first power-on following a reset the IKG runs a
// startup sequence on the shared public-key engine and its operand memory; a
// command issued before it finishes executes against clobbered operands
// (observed as an instant INVALID_MODULUS on the first operation, with the
// IK activity bit raised in the status word). Wait for both the PKE and the
// IK busy flags the way the sdk-nrf acquire does before handing the module
// to an engine. Once the startup has completed the wait costs two reads.
#define CRACEN_PK_REG_STATUS			0x0CU
#define CRACEN_PK_STATUS_BUSY			(1UL << 16)
#define CRACEN_IK_REG_PK_STATUS			0x1024U
#define CRACEN_IK_PK_BUSY_MSK			0x00050000UL

#ifndef CRACEN_PKEIKG_READY_POLL_LIMIT
#define CRACEN_PKEIKG_READY_POLL_LIMIT	4000000U
#endif

static bool CracenPkeIkgWaitReady(void)
{
	const volatile uint32_t *pkstat =
		(const volatile uint32_t *)(s_pPkeRegBase + CRACEN_PK_REG_STATUS);
	const volatile uint32_t *ikstat =
		(const volatile uint32_t *)(s_pPkeRegBase + CRACEN_IK_REG_PK_STATUS);

	for (uint32_t i = 0; i < CRACEN_PKEIKG_READY_POLL_LIMIT; i++)
	{
		if ((*pkstat & CRACEN_PK_STATUS_BUSY) == 0U &&
			(*ikstat & CRACEN_IK_PK_BUSY_MSK) == 0U)
		{
			return true;
		}
	}
	return false;
}

// PKE code RAM lives at NRF_CRACENCORE + 0xC000. The Silex BA414EP on this die
// is the microcoded configuration: the engine cannot sequence any operation
// until its code RAM is filled. Init owns the load; the fast path here is the
// two-word verify passing. The full copy reruns only if the RAM was cleared.
#define BA414EP_CODE_OFFSET 0xC000U
static bool CracenLoadPkeMicrocode(void)
{
	volatile uint32_t *pCode = (volatile uint32_t *)
		((uintptr_t)NRF_CRACENCORE + BA414EP_CODE_OFFSET);
	const size_t words = CRACEN_BA414E_UCODE_WORDS;

	if (pCode[0] == s_CracenBa414eUcode[0] &&
		pCode[words - 1U] == s_CracenBa414eUcode[words - 1U])
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

bool CracenIntrf::CoreAcquire(uint32_t Module, const void *pOwner)
{
	const uint32_t mask = CracenModuleMask(Module);
	if (mask == 0U || pOwner == nullptr || atomic_flag_test_and_set(&s_HoldFlag))
	{
		return false;
	}

	// Power only this sub-block for the operation (the hardware runs one module
	// at a time), serialize against the other engines, select the register base.
	s_HeldMask = mask;
	s_PreviousEnable = NRF_CRACEN->ENABLE & mask;
	s_pHoldOwner = pOwner;
	s_pRegBase = (Module == CRACEN_MODULE_CRYPTOMASTER) ? s_pCmRegBase
											 : s_pPkeRegBase;
	NRF_CRACEN->ENABLE |= mask;
	__DMB();

	if (Module == CRACEN_MODULE_PKEIKG &&
		(!CracenPkeIkgWaitReady() || !CracenLoadPkeMicrocode()))
	{
		NRF_CRACEN->ENABLE &= ~mask;
		s_HeldMask = 0U;
		s_PreviousEnable = 0U;
		s_pHoldOwner = nullptr;
		atomic_flag_clear(&s_HoldFlag);
		return false;
	}
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

bool CracenIntrf::CoreRelease(const void *pOwner)
{
	return CracenHoldRelease(pOwner);
}

bool CracenIntrf::CoreReset(const void *pOwner)
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

// Transport enable/disable placeholders. On this silicon only one CRACEN
// sub-block may be powered at a time, so module power is owned by the operation
// lock (CoreAcquire/CoreRelease), not a whole-core enable. NRF_CRACEN is the
// Nordic wrapper and lives only here; the Silex engines never touch it.
static void CracenCoreDisable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
static void CracenCoreEnable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
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
	// Module power is owned by the transport Enable/Disable (whole core on).
}
static void *CracenGetHandle(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
	return (void *)NRF_CRACENCORE;
}

// Registers used by the handover probe. Offsets match the Silex BA414EP
// register file (silexpk regs_addr.h).
#define CRACEN_PK_REG_CONFIG		0x00U
#define CRACEN_PK_REG_COMMAND		0x04U
#define CRACEN_PK_REG_CONTROL		0x08U
#define CRACEN_PK_CTRL_START		0x00000001UL
#define CRACEN_PK_CTRL_CLEAR_IRQ	0x00000002UL

// After a reset the PKEIKG module comes up on its identity-key side: the
// first public-key START is consumed by the IK to PK handover and completes
// instantly with a bogus error and the IK activity bit (bit 17, the
// SX_PK_OP_IK marker) raised in the status word. The next START runs clean.
// The old lazy-load flow masked this because its first acquire performed the
// full code RAM load and the operation in one powered session, and retained
// state hid it across warm debugger restarts. Run one small self-checking
// operation here, retrying once to absorb the handover, so the engine is
// proven to be computing in PK mode before Init returns: (7 + 8) mod 13,
// modular add, 4 byte operands, expected result 2.
static bool CracenPkeHandoverProbe(void)
{
	volatile uint32_t *cfg = (volatile uint32_t *)(s_pPkeRegBase + CRACEN_PK_REG_CONFIG);
	volatile uint32_t *cmd = (volatile uint32_t *)(s_pPkeRegBase + CRACEN_PK_REG_COMMAND);
	volatile uint32_t *ctrl = (volatile uint32_t *)(s_pPkeRegBase + CRACEN_PK_REG_CONTROL);
	const volatile uint32_t *stat =
		(const volatile uint32_t *)(s_pPkeRegBase + CRACEN_PK_REG_STATUS);

	// Big-endian operands sit at the end of their 0x200 slot. Modular add
	// uses the modulus in slot 0 and the default A, B, C pointers 6, 8, 10.
	volatile uint8_t *n = s_pMemBase + 0x200U - 4U;
	volatile uint8_t *a = s_pMemBase + 6U * 0x200U + 0x200U - 4U;
	volatile uint8_t *b = s_pMemBase + 8U * 0x200U + 0x200U - 4U;
	const volatile uint8_t *c = s_pMemBase + 10U * 0x200U + 0x200U - 4U;

	// Command 0x01 modular add, operand size 4 (3 in bits 8-15), big-endian
	// operands (bit 28), recompute r square (bit 31).
	const uint32_t probecmd = 0x01UL | (3UL << 8) | (1UL << 28) | (1UL << 31);

	for (uint32_t attempt = 0U; attempt < 2U; attempt++)
	{
		n[0] = 0U; n[1] = 0U; n[2] = 0U; n[3] = 13U;
		a[0] = 0U; a[1] = 0U; a[2] = 0U; a[3] = 7U;
		b[0] = 0U; b[1] = 0U; b[2] = 0U; b[3] = 8U;
		*cfg = (6UL) | (8UL << 8) | (10UL << 16);
		*cmd = probecmd;
		__DMB();
		*ctrl = CRACEN_PK_CTRL_START | CRACEN_PK_CTRL_CLEAR_IRQ;

		uint32_t st = CRACEN_PK_STATUS_BUSY;
		for (uint32_t i = 0U; i < CRACEN_PKEIKG_READY_POLL_LIMIT; i++)
		{
			st = *stat;
			if ((st & CRACEN_PK_STATUS_BUSY) == 0U)
			{
				break;
			}
		}
		*ctrl = CRACEN_PK_CTRL_CLEAR_IRQ;

		if ((st & (CRACEN_PK_STATUS_BUSY | 0xFFF0UL)) == 0U &&
			c[0] == 0U && c[1] == 0U && c[2] == 0U && c[3] == 2U)
		{
			return true;
		}
	}
	return false;
}

bool CracenIntrf::Init(void)
{
	s_pPkeRegBase = (volatile uint8_t *)&NRF_CRACENCORE->PK;
	s_pCmRegBase = (volatile uint8_t *)NRF_CRACENCORE;
	s_pMemBase = (volatile uint8_t *)((uintptr_t)NRF_CRACENCORE +
									 BA414EP_CRYPTORAM_OFFSET);

	// Load the BA414EP microcode once here, the way the Nordic power-up init
	// does (cracen_init in sdk-nrf hardware.c): power the PKE module, fill
	// the code RAM, restore the module power state. The code RAM retains its
	// content across the ENABLE cycles that CoreAcquire and CoreRelease
	// perform; a chip reset clears it and also re-runs Init. CoreAcquire
	// keeps a cheap two-word verify with reload as a guard.
	const uint32_t prev = NRF_CRACEN->ENABLE & CRACEN_ENABLE_PKEIKG_Msk;
	NRF_CRACEN->ENABLE |= CRACEN_ENABLE_PKEIKG_Msk;
	__DMB();
	bool bUcodeOk = CracenLoadPkeMicrocode();
	if (bUcodeOk)
	{
		bUcodeOk = CracenPkeIkgWaitReady() && CracenPkeHandoverProbe();
	}
	if (prev == 0U)
	{
		NRF_CRACEN->ENABLE &= ~CRACEN_ENABLE_PKEIKG_Msk;
		__DMB();
	}
	if (!bUcodeOk)
	{
		return false;
	}

	s_pRegBase = s_pPkeRegBase;
	s_pXferBase = s_pPkeRegBase;
	s_bXferMem = false;
	s_Offset = 0U;
	s_bAddrLatched = false;
	s_bRngXfer = false;
	s_bRngHeld = false;
	s_bDrbgInit = false;
	s_HeldMask = 0U;

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
