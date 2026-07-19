/**-------------------------------------------------------------------------
@file	bt_pds_stm32wba.cpp

@brief	bt_pds NVM implementation over STM32WBA on-chip flash.

	Provides the NvmIO medium primitives (Read/Write/Erase) that the
	generic log-structured KV store (src/bluetooth/bt_pds.cpp) uses to
	persist bond records. This is the WBA equivalent of the Nordic
	bt_pds_nvm_nvmc_sdc.cpp implementation.

	Layering (same opaque-blob seam as Nordic):
	    bt_smp_bond.cpp (RAM table, weak hooks)
	      -> bt_smp_bond_stm32wba.cpp (strong hooks: slot <-> bt_pds key)
	        -> bt_pds.cpp (log structured KV over a region)
	          -> THIS FILE (NvmIO over WBA flash)

	STATUS:
	  [VERIFIED] BtPdsNvm_t layout + BtPdsInit signature: IOsonata
	             include/bluetooth/bt_pds.h.
	  [HARDWARE] flash addresses, sector size, and HAL erase/write timing
	             MUST be confirmed against the target part linker script and
	             the STM32WBAxx flash reference manual before trusting on
	             hardware. Values below are placeholders flagged TODO.

@author	Hoang Nguyen Hoan (port skeleton)
@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "stm32wbaxx.h"
#include "stm32wbaxx_hal.h"

#include "bluetooth/bt_pds.h"

// ---------------------------------------------------------------------------
// Region definition.
//
// [HARDWARE][TODO] These MUST match the reserved bond region in the target
// linker script. The region must be flash-sector aligned and sit OUTSIDE the
// code/stack image and OUTSIDE any area the ST stack uses for its own SNVMA.
//
// WBA flash is page-organised. Confirm FLASH_PAGE_SIZE for the part from the
// reference manual (do NOT assume - it differs across WBA2/5/6 density). The
// write granularity on WBA flash is a quad-word (16 bytes) for the main array;
// confirm and set WBA_FLASH_WRITE_GRAN accordingly.
// ---------------------------------------------------------------------------
#ifndef WBA_BOND_REGION_ADDR
#define WBA_BOND_REGION_ADDR	0x080F0000u		// TODO confirm vs ldscript
#endif
#ifndef WBA_BOND_REGION_SIZE
#define WBA_BOND_REGION_SIZE	0x00002000u		// TODO confirm (2 pages?)
#endif
#ifndef WBA_FLASH_PAGE_SIZE
#define WBA_FLASH_PAGE_SIZE		0x00001000u		// TODO confirm per part
#endif
#ifndef WBA_FLASH_WRITE_GRAN
#define WBA_FLASH_WRITE_GRAN	16u				// TODO confirm (quad-word)
#endif

// Absolute base of the region in the flash address space.
static const uint32_t s_RegionBase = WBA_BOND_REGION_ADDR;

// ---------------------------------------------------------------------------
// STM32WBA internal flash bond region as an NvmIO. Flash is memory mapped for
// read; programming is quad-word (16 byte) and erase is page based.
// ---------------------------------------------------------------------------
class BtPdsWbaNvm : public NvmIO {
public:
	BtPdsWbaNvm() { Region(s_RegionBase, WBA_BOND_REGION_SIZE); }
	bool Enable(void) override { return true; }
	void Disable(void) override {}
	void Reset(void) override {}
	uint32_t EraseSize(void) const override { return WBA_FLASH_PAGE_SIZE; }
	uint32_t WriteGran(void) const override { return WBA_FLASH_WRITE_GRAN; }

	int Read(uint64_t Off, void *pBuf, uint32_t Len) override
	{
		if (pBuf == NULL || !RangeValid(Off, Len))
		{
			return -EINVAL;
		}
		memcpy(pBuf, (const void *)(s_RegionBase + (uint32_t)Off), Len);
		return (int)Len;
	}

	// [HARDWARE] HAL_FLASH_Program in quad-word mode. Confirm the program type
	// constant name for WBA HAL (FLASH_TYPEPROGRAM_QUADWORD) and the source
	// alignment. Synchronous: committed on return.
	int Write(uint64_t Off, const void *pData, uint32_t Len) override
	{
		if (pData == NULL ||
			(Off % WBA_FLASH_WRITE_GRAN) != 0 ||
			(Len % WBA_FLASH_WRITE_GRAN) != 0 ||
			!RangeValid(Off, Len))
		{
			return -EINVAL;
		}

		int rc = (int)Len;
		HAL_FLASH_Unlock();

		const uint8_t *src = (const uint8_t *)pData;
		for (uint32_t i = 0; i < Len; i += WBA_FLASH_WRITE_GRAN)
		{
			// [HARDWARE][TODO] confirm the data argument is the ADDRESS of the
			// source quad-word on WBA HAL (it is, for QUADWORD).
			uint32_t dst = s_RegionBase + (uint32_t)Off + i;
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, dst,
								   (uint32_t)(uintptr_t)(src + i)) != HAL_OK)
			{
				rc = -EIO;
				break;
			}
		}

		HAL_FLASH_Lock();
		return rc;
	}

	// [HARDWARE] HAL_FLASHEx_Erase page mode. HAL wants a page index, not an
	// address; confirm FLASH_BASE and the bank layout for the target part.
	int Erase(uint64_t Off, uint32_t Len) override
	{
		if ((Off % WBA_FLASH_PAGE_SIZE) != 0 ||
			(Len % WBA_FLASH_PAGE_SIZE) != 0 ||
			!RangeValid(Off, Len))
		{
			return -EINVAL;
		}

		uint32_t off = (uint32_t)Off;
		uint32_t remain = Len;
		int rc = 0;
		HAL_FLASH_Unlock();
		while (remain > 0U)
		{
			// [HARDWARE][TODO] page index relative to flash base.
			uint32_t absAddr = s_RegionBase + off;
			uint32_t pageIdx = (absAddr - FLASH_BASE) / WBA_FLASH_PAGE_SIZE;

			FLASH_EraseInitTypeDef ei;
			memset(&ei, 0, sizeof(ei));
			ei.TypeErase = FLASH_TYPEERASE_PAGES;
			ei.Page      = pageIdx;		// TODO confirm field name/semantics
			ei.NbPages   = 1;
			// ei.Banks  = FLASH_BANK_1;	// TODO set if part is multi-bank

			uint32_t pageError = 0;
			if (HAL_FLASHEx_Erase(&ei, &pageError) != HAL_OK)
			{
				rc = -EIO;
				break;
			}
			off += WBA_FLASH_PAGE_SIZE;
			remain -= WBA_FLASH_PAGE_SIZE;
		}
		HAL_FLASH_Lock();
		return rc;
	}
};

static BtPdsWbaNvm s_WbaPdsNvm;

// ---------------------------------------------------------------------------
// One-time init entry point. Called from the WBA bond glue (bt_smp_bond_wba)
// before the first bond access. Mirrors the Nordic BtPdsSdcNvmInit role.
// ---------------------------------------------------------------------------
extern "C" int BtPdsWbaInit(void)
{
	return BtPdsInit(&s_WbaPdsNvm);
}
