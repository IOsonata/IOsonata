/**-------------------------------------------------------------------------
@file	cracen_intrf.cpp

@brief	Device interface to the Nordic on-die crypto core (CRACEN).

		Leaf DeviceIntrf that owns the CRACENCORE register and operand memory
		access. A crypto engine reaches it through Device Read / Write, the same
		way a sensor reaches an SPI or I2C bus.

		DevAddr selects the base to access, the way SPI uses it as a chip-select
		index: CRACEN_ADDR_REG for the engine registers, CRACEN_ADDR_MEM for the
		operand memory. The transfer address is the byte offset within that base.
		StartTx enables the addressed engine module and takes the busy flag (from
		the base layer) so the engines serialize; StopTx disables and releases.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf.h"

#include "cracen_intrf.h"

// Operand memory sub-block offset from the CRACENCORE base.
#define BA414EP_CRYPTORAM_OFFSET	0x8000U

// Selected base and latched offset for the current transfer.
static volatile uint8_t *s_pSelBase;
static uint32_t s_Offset;
static bool s_bAddrLatched;

static volatile uint8_t *s_pRegBase;
static volatile uint8_t *s_pMemBase;

// Direct word access used within a held operation. Registers and operand memory
// require 32-bit accesses; offsets from the engine are word aligned.
uint32_t CracenIntrf::RegRead(uint32_t Offset) const
{
	return *(volatile uint32_t *)(vpRegBase + Offset);
}

void CracenIntrf::RegWrite(uint32_t Offset, uint32_t Value)
{
	*(volatile uint32_t *)(vpRegBase + Offset) = Value;
}

void CracenIntrf::MemRead(uint32_t Offset, uint8_t *pDst, size_t Len) const
{
	const volatile uint32_t *w = (const volatile uint32_t *)(vpMemBase + Offset);
	size_t words = Len / 4;
	for (size_t i = 0; i < words; i++)
	{
		uint32_t v = w[i];
		pDst[i * 4]     = (uint8_t)v;
		pDst[i * 4 + 1] = (uint8_t)(v >> 8);
		pDst[i * 4 + 2] = (uint8_t)(v >> 16);
		pDst[i * 4 + 3] = (uint8_t)(v >> 24);
	}
}

void CracenIntrf::MemWrite(uint32_t Offset, const uint8_t *pSrc, size_t Len)
{
	volatile uint32_t *w = (volatile uint32_t *)(vpMemBase + Offset);
	size_t words = Len / 4;
	for (size_t i = 0; i < words; i++)
	{
		w[i] = (uint32_t)pSrc[i * 4] | ((uint32_t)pSrc[i * 4 + 1] << 8) |
			   ((uint32_t)pSrc[i * 4 + 2] << 16) |
			   ((uint32_t)pSrc[i * 4 + 3] << 24);
	}
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

// Hold a module enabled for the whole operation and take the busy flag so the
// engines serialize. The module stays powered across every access until
// ModuleRelease. Returns false if another engine already holds the core.
bool CracenIntrf::ModuleHold(uint32_t Module)
{
	const uint32_t mask = CracenModuleMask(Module);
	if (mask == 0U)
	{
		return false;
	}
	if (atomic_flag_test_and_set(&vDevIntrf.bBusy))
	{
		return false;
	}
	NRF_CRACEN->ENABLE |= mask;
	return true;
}

void CracenIntrf::ModuleRelease(void)
{
	NRF_CRACEN->ENABLE &= ~(CRACEN_ENABLE_CRYPTOMASTER_Msk |
							CRACEN_ENABLE_PKEIKG_Msk |
							CRACEN_ENABLE_RNG_Msk);
	atomic_flag_clear(&vDevIntrf.bBusy);
}

static void CracenCoreEnable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
}

static void CracenCoreDisable(DevIntrf_t * const pIntrf)
{
	(void)pIntrf;
}
static uint32_t CracenGetRate(DevIntrf_t * const pIntrf) { (void)pIntrf; return 0U; }
static uint32_t CracenSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	(void)pIntrf; (void)Rate; return 0U;
}

static bool CracenStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	// Select the base for this access and clear any prior address latch. The
	// module is already enabled by Enable for the duration of the operation.
	s_pSelBase = (DevAddr == CRACEN_ADDR_MEM) ? s_pMemBase : s_pRegBase;
	s_bAddrLatched = false;
	return true;
}

static bool CracenStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf;
	// Restart condition inside a read: keep the offset latched by the preceding
	// address phase; only re-select the base.
	s_pSelBase = (DevAddr == CRACEN_ADDR_MEM) ? s_pMemBase : s_pRegBase;
	return true;
}

// The Device Write path combines the address and data into one TxData call:
// the first CRACEN_ADDR_BYTES bytes are the byte offset, the remainder is the
// value written at the selected base. The Read path sends the address alone
// (no stop) via TxSrData, then reads with RxData.
#define CRACEN_ADDR_BYTES	4

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
		int n = (DataLen < CRACEN_ADDR_BYTES) ? DataLen : CRACEN_ADDR_BYTES;
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
	// The BA414EP registers and operand RAM require single 32-bit word accesses.
	// All engine offsets are word aligned and all transfers are a word multiple.
	// Little-endian packing reproduces the byte image the engine intends.
	volatile uint32_t *w = (volatile uint32_t *)(s_pSelBase + s_Offset);
	for (int i = 0; i + 4 <= len; i += 4)
	{
		w[i / 4] = (uint32_t)p[i] | ((uint32_t)p[i + 1] << 8) |
				   ((uint32_t)p[i + 2] << 16) | ((uint32_t)p[i + 3] << 24);
	}
	return DataLen;
}

static int CracenTxSrData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	return CracenTxData(pIntrf, pData, DataLen);
}

static int CracenRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	(void)pIntrf;
	if (pBuff == nullptr || BuffLen <= 0 || !s_bAddrLatched)
	{
		return 0;
	}
	// Word access, matching the write path.
	volatile uint32_t *w = (volatile uint32_t *)(s_pSelBase + s_Offset);
	for (int i = 0; i + 4 <= BuffLen; i += 4)
	{
		uint32_t v = w[i / 4];
		pBuff[i]     = (uint8_t)v;
		pBuff[i + 1] = (uint8_t)(v >> 8);
		pBuff[i + 2] = (uint8_t)(v >> 16);
		pBuff[i + 3] = (uint8_t)(v >> 24);
	}
	return BuffLen;
}

static void CracenStop(void)
{
	// Module stays enabled for the operation; Disable powers it down at the end.
	s_bAddrLatched = false;
}

static void CracenStopTx(DevIntrf_t * const pIntrf) { (void)pIntrf; CracenStop(); }
static void CracenStopRx(DevIntrf_t * const pIntrf) { (void)pIntrf; CracenStop(); }

bool CracenIntrf::Init(void)
{
	vpRegBase = (volatile uint8_t *)&NRF_CRACENCORE->PK;
	vpMemBase = (volatile uint8_t *)((uintptr_t)NRF_CRACENCORE +
									 BA414EP_CRYPTORAM_OFFSET);
	s_pRegBase = vpRegBase;
	s_pMemBase = vpMemBase;
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
	atomic_store(&vDevIntrf.EnCnt, 0);

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
