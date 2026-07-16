/**-------------------------------------------------------------------------
@file	cracen_intrf.cpp

@brief	Device interface to the Nordic on-die crypto core (CRACEN).

		Leaf DeviceIntrf that owns the CRACENCORE register and operand memory
		access. CRACEN is not a crypto engine; it is the access path the Silex
		engines sit on. The public-key engine, the symmetric engine and the
		random generator each reach the die through it, and it holds the shared
		enable and the base addresses.

		An engine holds its module for the duration of an operation with
		ModuleHold and releases it with ModuleRelease. ModuleHold takes the busy
		flag so the engines serialize against one another, enables the module,
		and points register access at that module's sub-block. Within the held
		operation the engine reaches registers and operand memory through the
		direct offset accessors, which do not re-take the busy flag.

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

// Active register base (the held module's sub-block), the operand memory base,
// and the two fixed sub-block register bases.
static volatile uint8_t *s_pRegBase;
static volatile uint8_t *s_pMemBase;
static volatile uint8_t *s_pPkeRegBase;
static volatile uint8_t *s_pCmRegBase;

// Resolve the base a transfer addresses: operand memory, or the held module's
// register sub-block.
static volatile uint8_t *CracenBase(uint32_t DevAddr)
{
	return (DevAddr == CRACEN_ADDR_MEM) ? s_pMemBase : s_pRegBase;
}

// Byte offset carried in the address bytes (little-endian), as the engine sends
// it through Device Read / Write.
static uint32_t CracenOffset(const uint8_t *pAdCmd, int AdCmdLen)
{
	uint32_t off = 0U;
	for (int i = 0; i < AdCmdLen && i < 4; i++)
	{
		off |= (uint32_t)pAdCmd[i] << (8 * i);
	}
	return off;
}

// Memory-mapped read. The two working paths access registers as 32-bit words
// and operand memory byte-wise; both are proven on hardware, so this reproduces
// them per space. DevAddr selects which.
int CracenIntrf::Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen,
					  uint8_t *pBuff, int BuffLen)
{
	volatile uint8_t *base = CracenBase(DevAddr);
	uint32_t off = CracenOffset(pAdCmd, AdCmdLen);
	if (DevAddr == CRACEN_ADDR_MEM)
	{
		const volatile uint8_t *p = base + off;
		for (int i = 0; i < BuffLen; i++)
		{
			pBuff[i] = p[i];
		}
	}
	else
	{
		const volatile uint32_t *w = (const volatile uint32_t *)(base + off);
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

int CracenIntrf::Write(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen,
					   const uint8_t *pData, int DataLen)
{
	volatile uint8_t *base = CracenBase(DevAddr);
	uint32_t off = CracenOffset(pAdCmd, AdCmdLen);
	if (DevAddr == CRACEN_ADDR_MEM)
	{
		volatile uint8_t *p = base + off;
		for (int i = 0; i < DataLen; i++)
		{
			p[i] = pData[i];
		}
	}
	else
	{
		volatile uint32_t *w = (volatile uint32_t *)(base + off);
		for (int i = 0; i + 4 <= DataLen; i += 4)
		{
			w[i / 4] = (uint32_t)pData[i] | ((uint32_t)pData[i + 1] << 8) |
					   ((uint32_t)pData[i + 2] << 16) |
					   ((uint32_t)pData[i + 3] << 24);
		}
	}
	return DataLen;
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
	atomic_flag_clear(&vDevIntrf.bBusy);
}

// The DeviceIntrf table below is the leaf interface plumbing. Register and
// operand access uses the direct accessors above, so the byte-stream transfer
// hooks are minimal: the interface carries no serial data phase.
static void CracenDummyDisable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
static void CracenDummyEnable(DevIntrf_t * const pIntrf) { (void)pIntrf; }
static uint32_t CracenGetRate(DevIntrf_t * const pIntrf) { (void)pIntrf; return 0U; }
static uint32_t CracenSetRate(DevIntrf_t * const pIntrf, uint32_t Rate)
{
	(void)pIntrf; (void)Rate; return 0U;
}
static bool CracenStartTx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf; (void)DevAddr; return true;
}
static int CracenTxData(DevIntrf_t * const pIntrf, const uint8_t *pData, int DataLen)
{
	(void)pIntrf; (void)pData; return DataLen;
}
static void CracenStopTx(DevIntrf_t * const pIntrf) { (void)pIntrf; }
static bool CracenStartRx(DevIntrf_t * const pIntrf, uint32_t DevAddr)
{
	(void)pIntrf; (void)DevAddr; return true;
}
static int CracenRxData(DevIntrf_t * const pIntrf, uint8_t *pBuff, int BuffLen)
{
	(void)pIntrf; (void)pBuff; (void)BuffLen; return 0;
}
static void CracenStopRx(DevIntrf_t * const pIntrf) { (void)pIntrf; }

bool CracenIntrf::Init(void)
{
	// Register sub-block bases: the public-key engine and the symmetric engine
	// are separate register blocks; operand memory is the public-key operand RAM.
	s_pPkeRegBase = (volatile uint8_t *)&NRF_CRACENCORE->PK;
	s_pCmRegBase = (volatile uint8_t *)NRF_CRACENCORE;
	s_pMemBase = (volatile uint8_t *)((uintptr_t)NRF_CRACENCORE +
									 BA414EP_CRYPTORAM_OFFSET);
	s_pRegBase = s_pPkeRegBase;

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

	vDevIntrf.Disable = CracenDummyDisable;
	vDevIntrf.Enable = CracenDummyEnable;
	vDevIntrf.GetRate = CracenGetRate;
	vDevIntrf.SetRate = CracenSetRate;
	vDevIntrf.StartRx = CracenStartRx;
	vDevIntrf.RxData = CracenRxData;
	vDevIntrf.StopRx = CracenStopRx;
	vDevIntrf.StartTx = CracenStartTx;
	vDevIntrf.TxData = CracenTxData;
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
