/**-------------------------------------------------------------------------
@file	dfu_nrfx.cpp

@brief	DFU target layer for Nordic: NVMC (nRF52, nRF53, nRF91), RRAMC
		(nRF54L) and MRAM (nRF54H).

On nRF52 with an MBR and a SoftDevice, the application is started the way
the nRF5 SDK bootloader starts it: MBR interrupt forwarding set to the
SoftDevice, then the SoftDevice reset handler, which starts the application
at the end of the SoftDevice. Everywhere else VTOR points at slot 0.

On nRF54H20 the secure domain firmware owns MRAM and decides which core may
write which partition; the layouts assume each core may write its own.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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
#include "nrf.h"

#if defined(NRF52_SERIES) || defined(NRF53_SERIES) || defined(NRF91_SERIES)
#define DFU_TGT_NVMC			1
#elif defined(NRF54L_SERIES) || defined(NRF54L15_XXAA) || \
	  defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#include "hal/nrf_rramc.h"
#define DFU_TGT_RRAMC			1
#elif defined(NRF54H_SERIES) || defined(NRF54H20_XXAA)
#define DFU_TGT_MRAM			1
#else
#error "dfu_nrfx: unsupported MCU model"
#endif

#include "dfu/dfu_target.h"
#include "dfu_cm.h"

/** @addtogroup DFU
  * @{
  */

/// Polls of a ready flag before giving up, so a controller that never comes
/// ready fails the operation instead of hanging the boot. A page erase takes
/// under 100 ms on these parts; this is seconds at the fastest core clock.
#define DFU_TGT_WAIT_MAX		0x04000000UL

// ---------------------------------------------------------------------------
// Internal memory
// ---------------------------------------------------------------------------

#if defined(DFU_TGT_NVMC)

// Page erased flash, a mode register and a READY flag, the geometry in FICR:
// nRF52, nRF53 (both cores) and nRF91. Registers are written directly, the
// same accesses nrf_nvmc.h makes, so the file needs no nrfx configuration
// header, which some of these libraries do not have.
//
// nRF53 and nRF91 name their peripherals by security state. The boot and an
// application without TrustZone run secure; the nRF5340 network core has no
// secure alias.
#if defined(NRF_NVMC)
#define DFU_TGT_NVMC_REG		NRF_NVMC
#elif defined(NRF_TRUSTZONE_NONSECURE) || defined(NRF5340_XXAA_NETWORK)
#define DFU_TGT_NVMC_REG		NRF_NVMC_NS
#else
#define DFU_TGT_NVMC_REG		NRF_NVMC_S
#endif

#if defined(NRF_FICR)
#define DFU_TGT_FICR_REG		NRF_FICR
#elif defined(NRF5340_XXAA_NETWORK)
#define DFU_TGT_FICR_REG		NRF_FICR_NS
#else
#define DFU_TGT_FICR_REG		NRF_FICR_S
#endif

uint32_t DfuTgtWriteUnit(void)
{
	return 4;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	(void)Addr;

#if defined(FICR_INFO_CODEPAGESIZE_CODEPAGESIZE_Msk)
	return DFU_TGT_FICR_REG->INFO.CODEPAGESIZE;
#else
	return DFU_TGT_FICR_REG->CODEPAGESIZE;
#endif
}

// Mode register. On a part with an instruction cache (nRF53, nRF91), write
// or erase mode keeps the cache invalidated, so nothing stale is read back.
static void DfuTgtMode(uint32_t Mode)
{
	DFU_TGT_NVMC_REG->CONFIG = Mode;
	__DSB();
}

static bool DfuTgtWait(void)
{
	for (uint32_t i = 0; i < DFU_TGT_WAIT_MAX; i++)
	{
		if (DFU_TGT_NVMC_REG->READY & NVMC_READY_READY_Msk)
		{
			return true;
		}
	}

	return false;
}

bool DfuTgtErase(uintptr_t Addr)
{
	if ((Addr % DfuTgtEraseUnit(Addr)) != 0)
	{
		return false;
	}

	DfuTgtMode(NVMC_CONFIG_WEN_Een);
#if defined(NVMC_ERASEPAGE_ERASEPAGE_Msk)
	DFU_TGT_NVMC_REG->ERASEPAGE = (uint32_t)Addr;
#else
	// nRF53 and nRF91: in erase mode, a word written in a page erases it.
	*(volatile uint32_t *)Addr = 0xFFFFFFFFUL;
#endif
	bool res = DfuTgtWait();
	DfuTgtMode(NVMC_CONFIG_WEN_Ren);

	return res;
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	if ((Addr & 3) != 0 || (Len & 3) != 0)
	{
		return false;
	}

	const uint8_t *p = (const uint8_t *)pData;
	volatile uint32_t *dst = (volatile uint32_t *)Addr;
	bool res = true;

	DfuTgtMode(NVMC_CONFIG_WEN_Wen);
	for (uint32_t i = 0; i < Len / 4 && res; i++, p += 4)
	{
		// The source may be anywhere, flash included, and unaligned.
		dst[i] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
				 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
		res = DfuTgtWait();
	}
	DfuTgtMode(NVMC_CONFIG_WEN_Ren);

	return res;
}

#else	// RRAMC, MRAM

// No erase on these memories: an erase writes the erased pattern, the way
// nvm_nrfx does. The unit only sets how much at a time, and where regions
// may start and end: 2 KB, which the nRF54L layouts keep to around the
// SoftDevice. RRAM takes words, MRAM its 128 bit word.
#define DFU_TGT_NOERASE_UNIT	0x800U

#if defined(DFU_TGT_RRAMC)
#define DFU_TGT_WR_UNIT			4U
#else
#define DFU_TGT_WR_UNIT			16U
#endif

uint32_t DfuTgtWriteUnit(void)
{
	return DFU_TGT_WR_UNIT;
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	(void)Addr;

	return DFU_TGT_NOERASE_UNIT;
}

#if defined(DFU_TGT_RRAMC)
static void DfuTgtWriteEnable(bool bEnable)
{
	nrf_rramc_config_t cfg;

	nrf_rramc_config_get(NRF_RRAMC, &cfg);
	cfg.mode_write = bEnable;
	nrf_rramc_config_set(NRF_RRAMC, &cfg);
}
#endif

#if defined(DFU_TGT_RRAMC)
static bool DfuTgtRramcWait(bool bEmpty)
{
	for (uint32_t i = 0; i < DFU_TGT_WAIT_MAX; i++)
	{
		if (bEmpty ? nrf_rramc_empty_buffer_check(NRF_RRAMC) :
			nrf_rramc_write_ready_check(NRF_RRAMC))
		{
			return true;
		}
	}

	return false;
}
#else
// The core's caches in front of MRAM, from the device header. The data cache
// may hold written lines back (it has a clean task), and the instruction
// cache may still hold lines of the application that ran before the reset:
// clean the one, invalidate the other, before anything is read or run from
// what was written. Nothing to do on a cache the startup left disabled.
static bool DfuTgtCacheTask(NRF_CACHE_Type *pCache, volatile uint32_t *pTask)
{
	if ((pCache->ENABLE & CACHE_ENABLE_ENABLE_Msk) == 0)
	{
		return true;
	}

	*pTask = 1;
	for (uint32_t i = 0; i < DFU_TGT_WAIT_MAX; i++)
	{
		if ((pCache->STATUS & CACHE_STATUS_BUSY_Msk) == 0)
		{
			return true;
		}
	}

	return false;
}

static bool DfuTgtCacheSync(void)
{
	bool res = true;

#if defined(NRF_DCACHE)
	res = DfuTgtCacheTask(NRF_DCACHE, &NRF_DCACHE->TASKS_CLEANCACHE) && res;
#endif
#if defined(NRF_ICACHE)
	res = DfuTgtCacheTask(NRF_ICACHE, &NRF_ICACHE->TASKS_INVALIDATECACHE) && res;
#endif

	return res;
}
#endif

static bool DfuTgtWords(uintptr_t Addr, const uint8_t *p, uint32_t Words,
						bool bFill)
{
	volatile uint32_t *dst = (volatile uint32_t *)Addr;
	bool res = true;

#if defined(DFU_TGT_RRAMC)
	DfuTgtWriteEnable(true);
#endif
	for (uint32_t i = 0; i < Words && res; i++)
	{
		dst[i] = bFill ? 0xFFFFFFFFUL :
				 (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
				 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
		if (bFill == false)
		{
			p += 4;
		}
#if defined(DFU_TGT_RRAMC)
		res = DfuTgtRramcWait(false);
#endif
	}

#if defined(DFU_TGT_RRAMC)
	// Committed once the buffer has drained.
	res = DfuTgtRramcWait(true) && res;
	DfuTgtWriteEnable(false);
#else
	// MRAM takes the stores directly, a 128 bit word committed once it is
	// whole, which unit aligned writes always make it; the secure domain set
	// up its controller for the partition this core owns.
	__DSB();
	res = DfuTgtCacheSync() && res;
#endif

	return res;
}

bool DfuTgtErase(uintptr_t Addr)
{
	if ((Addr % DFU_TGT_NOERASE_UNIT) != 0)
	{
		return false;
	}

	return DfuTgtWords(Addr, nullptr, DFU_TGT_NOERASE_UNIT / 4, true);
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	if ((Addr % DFU_TGT_WR_UNIT) != 0 || (Len % DFU_TGT_WR_UNIT) != 0)
	{
		return false;
	}

	return DfuTgtWords(Addr, (const uint8_t *)pData, Len / 4, false);
}

#endif

// ---------------------------------------------------------------------------
// Start and reset
// ---------------------------------------------------------------------------

bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize)
{
	return DfuCmEntryValid(pImg, RunAddr, ImgSize, DFU_CM_SRAM_START,
						   DFU_CM_SRAM_END);
}

void DfuTgtReset(void)
{
	NVIC_SystemReset();
}

#if defined(NRF52_SERIES)

// SoftDevice information structure, at a fixed offset in every SoftDevice.
// The nRF5 SDK names these in nrf_bootloader_info.h and nrf_sdm.h, neither of
// which the boot otherwise needs.
#define DFU_TGT_SD_INFO_OFFSET		0x2000
#define DFU_TGT_SD_MAGIC_OFFSET		(DFU_TGT_SD_INFO_OFFSET + 0x04)
#define DFU_TGT_SD_SIZE_OFFSET		(DFU_TGT_SD_INFO_OFFSET + 0x08)
#define DFU_TGT_SD_MAGIC			0x51B1E5DBUL

// MBR call, as nrf_mbr.h defines it: SVC 0x18 with a command block, here IRQ
// forward address set. Written out because a library build without a
// SoftDevice has no MBR header on its path, while the call itself is only
// made when the MBR and a SoftDevice are found in flash.
#define DFU_TGT_MBR_SIZE			0x1000
#define DFU_TGT_MBR_IRQ_FORWARD		6

typedef struct {
	uint32_t Cmd;
	uint32_t Addr;
} DfuTgtMbrCmd_t;

static uint32_t DfuTgtMbrCall(DfuTgtMbrCmd_t *pCmd)
{
	register uint32_t r0 __asm("r0") = (uint32_t)pCmd;

	__asm volatile("svc 0x18" : "+r" (r0) : : "memory");

	return r0;
}

#endif

void DfuTgtStart(uintptr_t RunAddr)
{
	DfuCmQuiesce();

#if defined(NRF52_SERIES)
	uint32_t sdmagic = *(const uint32_t *)(DFU_TGT_MBR_SIZE +
										   DFU_TGT_SD_MAGIC_OFFSET);

	if (sdmagic == DFU_TGT_SD_MAGIC)
	{
		// The SoftDevice starts the application at its end, so that is where
		// slot 0 has to be. The layout says so; check it rather than start
		// something the SoftDevice will not forward to. The size field is
		// the end address, MBR included, as nrf_dfu_bank0_start_addr reads
		// it: rounded up to a page.
		uint32_t page = DfuTgtEraseUnit(RunAddr);
		uint32_t sdend = *(const uint32_t *)(DFU_TGT_MBR_SIZE +
											 DFU_TGT_SD_SIZE_OFFSET);
		sdend = (sdend + page - 1) & ~(page - 1);
		if (sdend != RunAddr)
		{
			for (;;)
			{
				__WFE();
			}
		}

		DfuTgtMbrCmd_t cmd = { DFU_TGT_MBR_IRQ_FORWARD, DFU_TGT_MBR_SIZE };
		(void)DfuTgtMbrCall(&cmd);

		// The SoftDevice reset handler, through the MBR vector table at the
		// start of the SoftDevice, starts the application at its end.
		DfuCmStart((const uint32_t *)DFU_TGT_MBR_SIZE, false);
	}
#endif

	DfuCmStart((const uint32_t *)RunAddr, true);
}

/** @} */
