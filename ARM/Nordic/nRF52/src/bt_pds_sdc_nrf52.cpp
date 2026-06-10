/**-------------------------------------------------------------------------
@file	bt_pds_sdc_nrf52.cpp

@brief	BtPdsNvm_t backend over nRF52 internal flash (NVMC) for SDC builds.

		Provides the radio safe NVM primitives the bt_pds store needs, with no
		dependency on nRF5_SDK. The SoftDevice Controller exposes no flash
		service, so the application owns flash and serializes every NVMC
		operation against the radio through an MPSL timeslot (bt_pds_nvm_mpsl).

		Region: the same flash pages an FDS based product reserves (top of
		application flash, below the bootloader / MBR params). Page size is the
		nRF52 erase unit (4 KB), so SectorSize is one page. The region is at
		least two pages: usable plus one spare for the bt_pds compaction
		rebuild.

		Write: NVMC word writes inside one timeslot. A bond record is a handful
		of words, well within a timeslot, so a single Step completes it.

		Erase: an nRF52 page erase takes ~85 ms, far longer than a radio safe
		timeslot. The driver call (nrfx_nvmc_page_erase) busy waits to
		completion, and the hardware partial erase exists only on nRF52840, not
		nRF52832. So this backend drives the erase at the HAL layer: it starts
		the page erase (nrf_nvmc_page_erase_start), yields the timeslot, and
		polls nrf_nvmc_ready_check on later timeslots until done. The NVMC
		controller runs the erase autonomously, so the radio runs between polls.
		This works on nRF52832 and nRF52840 with one path (no partial erase
		dependency).

		Note: while the page erase is in progress the flash bus is busy. The
		erase Step and the MPSL timeslot handling must execute from RAM during
		that window, otherwise instruction fetches from the erasing flash stall.
		The MPSL timeslot callback path is the place this runs.

		No unnecessary erase: bt_pds calls Erase only during compaction (region
		full), which is rare. A page that already reads as the erased pattern is
		skipped, so a clean spare page costs nothing.

		Init entry BtPdsSdcNvmInit matches the name the bond glue
		(bt_smp_bond_sdc.cpp) calls, so the same glue serves this backend and
		the nRF54L RRAMC backend; only one backend is linked per build.

@author	Hoang Nguyen Hoan
@date	Jun 09, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include "hal/nrf_nvmc.h"

#include "bluetooth/bt_pds.h"
#include "bt_pds_sdc.h"

// ---- Region geometry -------------------------------------------------------
// nRF52 flash page (erase unit) is 4 KB.
#ifndef BT_PDS_NVMC_PAGE_SIZE
#define BT_PDS_NVMC_PAGE_SIZE		0x1000UL		// 4 KB
#endif

// Bond region size: two pages (one usable, one compaction spare). Increase to
// spread wear and reduce compaction frequency.
#ifndef BT_PDS_NVMC_REGION_PAGES
#define BT_PDS_NVMC_REGION_PAGES	2
#endif

#define BT_PDS_NVMC_REGION_SIZE		(BT_PDS_NVMC_PAGE_SIZE * BT_PDS_NVMC_REGION_PAGES)

// Region base address. Default reserves the pages an FDS product would use:
// the top of application flash. Override from the linker layout so the region
// does not overlap the application, SDC, or bootloader. MUST be page aligned.
// 0xFE000 suits an nRF52840 (1 MB) reserving the last pages below the MBR
// params; set the correct value for the target part and map.
#ifndef BT_PDS_NVMC_REGION_ADDR
#define BT_PDS_NVMC_REGION_ADDR		0x000FE000UL
#endif

// NVMC writes are 32-bit word based.
#define BT_PDS_NVMC_WRITE_GRAN		4

// Erase slice length per timeslot, milliseconds. Short enough to keep the
// timeslot radio safe; the MPSL core re-requests until the page is done.
#ifndef BT_PDS_NVMC_ERASE_SLICE_MS
#define BT_PDS_NVMC_ERASE_SLICE_MS	2
#endif

// Worst-case microseconds for one Step inside a timeslot. The write Step writes
// a small record; the erase Step runs one ERASE_SLICE_MS slice. Both fit a
// short timeslot; size with margin.
#ifndef BT_PDS_NVMC_STEP_BUDGET_US
#define BT_PDS_NVMC_STEP_BUDGET_US	((BT_PDS_NVMC_ERASE_SLICE_MS + 1) * 1000UL)
#endif

// ---- Write -----------------------------------------------------------------
typedef struct {
	uint32_t		Addr;			// absolute flash address (word aligned)
	const uint32_t *pSrc;			// source words
	uint32_t		Words;			// word count
} NvmcWriteCtx_t;

static uint32_t NvmcWriteStep(void *pv)
{
	NvmcWriteCtx_t *c = (NvmcWriteCtx_t *)pv;

	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_WRITE);
	volatile uint32_t *pDst = (volatile uint32_t *)c->Addr;
	for (uint32_t i = 0; i < c->Words; i++)
	{
		pDst[i] = c->pSrc[i];
		while (!nrf_nvmc_ready_check(NRF_NVMC))
		{
			// Word write ~338 us; a record's words complete in the timeslot.
		}
	}
	nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);
	return 0;					// done in one Step
}

// ---- Erase (HAL async: start, then poll ready across timeslots) ------------
typedef struct {
	uint32_t	Addr;			// page base address
	bool		Started;		// erase has been kicked off
} NvmcEraseCtx_t;

// True if the page already reads as the erased pattern (all 0xFF), so it can be
// skipped (no unnecessary erase, saves wear).
static bool PageIsErased(uint32_t Addr)
{
	const uint32_t *p = (const uint32_t *)Addr;
	uint32_t words = BT_PDS_NVMC_PAGE_SIZE / 4;
	for (uint32_t i = 0; i < words; i++)
	{
		if (p[i] != 0xFFFFFFFFUL)
		{
			return false;
		}
	}
	return true;
}

static uint32_t NvmcEraseStep(void *pv)
{
	NvmcEraseCtx_t *c = (NvmcEraseCtx_t *)pv;

	if (!c->Started)
	{
		if (PageIsErased(c->Addr))
		{
			return 0;			// already erased, nothing to do
		}
		// Kick off the page erase and return; the NVMC controller runs it while
		// the radio resumes. Do not busy wait here.
		nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_ERASE);
		nrf_nvmc_page_erase_start(NRF_NVMC, c->Addr);
		c->Started = true;
		return BT_PDS_NVMC_ERASE_SLICE_MS * 1000UL;		// poll again next slot
	}

	// Erase running in hardware. Poll for completion.
	if (nrf_nvmc_ready_check(NRF_NVMC))
	{
		nrf_nvmc_mode_set(NRF_NVMC, NRF_NVMC_MODE_READONLY);
		return 0;				// page erase complete
	}

	return BT_PDS_NVMC_ERASE_SLICE_MS * 1000UL;			// not done, keep polling
}

// ---- BtPdsNvm_t primitives -------------------------------------------------
static int NvmcNvmRead(uint32_t Off, void *pBuf, uint32_t Len)
{
	// Flash is memory mapped; read directly.
	memcpy(pBuf, (const void *)(BT_PDS_NVMC_REGION_ADDR + Off), Len);
	return 0;
}

static int NvmcNvmWrite(uint32_t Off, const void *pData, uint32_t Len)
{
	// Len is a multiple of WriteGran (4) per the BtPdsNvm_t contract. The store
	// only writes into previously erased (0xFF) space, so no erase is implied.
	NvmcWriteCtx_t ctx;
	ctx.Addr  = BT_PDS_NVMC_REGION_ADDR + Off;
	ctx.pSrc  = (const uint32_t *)pData;
	ctx.Words = Len / BT_PDS_NVMC_WRITE_GRAN;

	BtPdsMpslOp_t op;
	op.Step         = NvmcWriteStep;
	op.StepBudgetUs = BT_PDS_NVMC_STEP_BUDGET_US;
	op.pCtx         = &ctx;

	return BtPdsMpslRun(&op);
}

static int NvmcNvmErase(uint32_t Off)
{
	NvmcEraseCtx_t ctx;
	ctx.Addr    = BT_PDS_NVMC_REGION_ADDR + Off;
	ctx.Started = false;

	BtPdsMpslOp_t op;
	op.Step         = NvmcEraseStep;
	op.StepBudgetUs = BT_PDS_NVMC_STEP_BUDGET_US;
	op.pCtx         = &ctx;

	return BtPdsMpslRun(&op);
}

static const BtPdsNvm_t s_BtPdsNvmcSdcNvm = {
	.RegionOffset = BT_PDS_NVMC_REGION_ADDR,
	.RegionSize   = BT_PDS_NVMC_REGION_SIZE,
	.SectorSize   = BT_PDS_NVMC_PAGE_SIZE,
	.WriteGran    = BT_PDS_NVMC_WRITE_GRAN,
	.Read         = NvmcNvmRead,
	.Write        = NvmcNvmWrite,
	.Erase        = NvmcNvmErase,
};

// Init entry: bring up the MPSL timeslot session, then mount the store. Name
// matches the bond glue (bt_smp_bond_sdc.cpp) so the same glue serves this and
// the nRF54L RRAMC backend.
extern "C" int BtPdsSdcNvmInit(void)
{
	int r = BtPdsMpslInit();
	if (r != 0)
	{
		return r;
	}
	return BtPdsInit(&s_BtPdsNvmcSdcNvm);
}
