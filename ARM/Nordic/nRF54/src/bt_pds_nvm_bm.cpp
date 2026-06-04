/**-------------------------------------------------------------------------
@file	bt_pds_nvm_bm.c

@brief	BtPds NVM backend for the nRF54L bm (sdk-nrf-bm / S145) port.

		Implements the BtPdsNvm_t primitives over RRAM. The region is the
		peer_manager partition reserved in the linker script. Writes go through
		the SoftDevice (sd_flash_write) because S145 arbitrates flash access
		against radio activity; a direct nrfx RRAMC write while the SoftDevice
		is enabled is unsafe. sd_flash_write is asynchronous: completion arrives
		as a SoC event (NRF_EVT_FLASH_OPERATION_SUCCESS / _ERROR), so each write
		is pumped to completion here to present a synchronous interface to the
		store.

		RRAM is memory mapped and byte readable, so Read is a plain memcpy from
		the absolute address. RRAM is rewrite-in-place, so Erase writes the
		erased pattern (0xFF) over the sector via the same SoftDevice path.

		Region: must match the peer_manager_partition reserved in
		gcc_nrf54l15_xxaa_s145.ld. The target owns these constants; adjust here
		if the flash map changes.

@author	Hoang Nguyen Hoan
@date	Jun 03, 2026

@license

MIT License

Copyright (c) 2016, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#include <string.h>
#include <errno.h>

#include "nrf_soc.h"			// sd_flash_write
#include "nrf_error.h"
#include "nrf_sdh_soc.h"		// SoC event observer
#include "nrf.h"				// __WFE

#include "bluetooth/bt_pds.h"

// Reserved peer_manager region (see gcc_nrf54l15_xxaa_s145.ld:
// 0x158800 .. 0x1597FF, 4 KB, modeled as 4 x 1 KB sectors to match the bm SDK
// PM_BM_ZMS_SECTOR_SIZE of 1024).
#ifndef BT_PDS_BM_REGION_ADDR
#define BT_PDS_BM_REGION_ADDR		0x00158800UL
#endif
#ifndef BT_PDS_BM_REGION_SIZE
#define BT_PDS_BM_REGION_SIZE		0x00001000UL	// 4 KB
#endif
#ifndef BT_PDS_BM_SECTOR_SIZE
#define BT_PDS_BM_SECTOR_SIZE		0x00000400UL	// 1 KB
#endif

#define BT_PDS_BM_WRITE_GRAN		4				// sd_flash_write is word based

// sd_flash_write completion is signalled through the SoC event path. These
// flags are set by the SoC observer and polled by the write wait loop.
static volatile bool s_FlashOpDone;
static volatile bool s_FlashOpOk;

// SoC event observer: catch flash operation completion. Registered into the
// nrf_sdh_soc observer section so soc_evt_poll dispatches here.
static void BtPdsBmSocEvtHandler(uint32_t SysEvt, void *pCtx)
{
	(void)pCtx;

	switch (SysEvt)
	{
		case NRF_EVT_FLASH_OPERATION_SUCCESS:
			s_FlashOpOk = true;
			s_FlashOpDone = true;
			break;
		case NRF_EVT_FLASH_OPERATION_ERROR:
			s_FlashOpOk = false;
			s_FlashOpDone = true;
			break;
		default:
			break;
	}
}

NRF_SDH_SOC_OBSERVER(s_BtPdsBmSocObs, BtPdsBmSocEvtHandler, NULL, HIGH);

// Issue one sd_flash_write and pump SoC events until it completes. p_dst/p_src
// are word aligned, size is in 32-bit words. Returns 0 on success.
static int FlashWriteWords(uint32_t *p_dst, const uint32_t *p_src, uint32_t words)
{
	uint32_t err;

	do
	{
		s_FlashOpDone = false;
		s_FlashOpOk = false;

		err = sd_flash_write(p_dst, p_src, words);

		if (err == NRF_ERROR_BUSY)
		{
			// Controller owns flash this moment; let events run and retry.
			__WFE();
			continue;
		}
		if (err != NRF_SUCCESS)
		{
			return -EIO;
		}

		// Wait for the async completion event.
		while (!s_FlashOpDone)
		{
			__WFE();
		}

		if (!s_FlashOpOk)
		{
			return -EIO;
		}
		return 0;

	} while (true);
}

static int BtPdsBmRead(uint32_t Off, void *pBuf, uint32_t Len)
{
	// RRAM is memory mapped; read directly.
	memcpy(pBuf, (const void *)(BT_PDS_BM_REGION_ADDR + Off), Len);
	return 0;
}

static int BtPdsBmWrite(uint32_t Off, const void *pData, uint32_t Len)
{
	if ((Off & 3) || (Len & 3))
	{
		return -EINVAL;		// store always passes word aligned offset/len
	}

	uint32_t *dst = (uint32_t *)(BT_PDS_BM_REGION_ADDR + Off);
	const uint32_t *src = (const uint32_t *)pData;

	return FlashWriteWords(dst, src, Len / 4);
}

static int BtPdsBmErase(uint32_t Off)
{
	// RRAM rewrite-in-place: write the erased pattern over the sector in
	// word-sized chunks from a small stack block to cut sd_flash_write calls.
	uint32_t blk[32];
	memset(blk, 0xFF, sizeof(blk));

	uint32_t *dst = (uint32_t *)(BT_PDS_BM_REGION_ADDR + Off);
	uint32_t words = BT_PDS_BM_SECTOR_SIZE / 4;
	uint32_t i = 0;

	while (i < words)
	{
		uint32_t n = words - i;
		if (n > (uint32_t)(sizeof(blk) / sizeof(blk[0])))
		{
			n = sizeof(blk) / sizeof(blk[0]);
		}
		int r = FlashWriteWords(&dst[i], blk, n);
		if (r != 0)
		{
			return r;
		}
		i += n;
	}
	return 0;
}

static const BtPdsNvm_t s_BtPdsBmNvm = {
	.RegionOffset = BT_PDS_BM_REGION_ADDR,
	.RegionSize   = BT_PDS_BM_REGION_SIZE,
	.SectorSize   = BT_PDS_BM_SECTOR_SIZE,
	.WriteGran    = BT_PDS_BM_WRITE_GRAN,
	.Read         = BtPdsBmRead,
	.Write        = BtPdsBmWrite,
	.Erase        = BtPdsBmErase,
};

// Convenience init the platform calls instead of building the backend itself.
int BtPdsBmInit(void)
{
	return BtPdsInit(&s_BtPdsBmNvm);
}
