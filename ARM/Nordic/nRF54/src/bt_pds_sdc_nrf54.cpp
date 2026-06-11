/**-------------------------------------------------------------------------
@file	bt_pds_nvm_rramc_sdc.cpp

@brief	BtPdsNvm_t backend over nRF54L RRAM for SDC (SoftDevice Controller)
		builds.

		Same medium as the bm port (RRAM), but the arbitration differs: there is
		no SoftDevice, so writes go through an MPSL timeslot instead of
		sd_flash_write. RRAM writes are short (microseconds), so a write fits in
		a single timeslot, no work splitting is needed (unlike NVMC erase on
		nRF52). Read is a direct memcpy from the memory-mapped region.

		RRAM is rewrite-in-place: there is no separate erase. The store needs a
		reclaimed sector to read back as the erased pattern (0xFF) to find its
		write head, so Erase writes 0xFF over the sector (still arbitrated).

		Pair this with the SDC bt_smp_bond persistence hooks (bt_smp_bond over
		BtPds). BtPdsInit(&g_BtPdsRramcSdcNvm) wires the store to this backend.

@author	Hoang Nguyen Hoan
@date	Jun 09, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <errno.h>
#include <string.h>

#include "nrf.h"
#include "hal/nrf_rramc.h"

#include "bluetooth/bt_pds.h"
#include "bt_pds_sdc.h"

// RRAM region reserved for the persistent store on the SDC build. Keep this in
// sync with the SDC linker script's reserved storage partition. Defaults match
// the bm port's region so a board can carry one bond region regardless of
// backend; override per board/linker as needed.
#ifndef BT_PDS_RRAMC_REGION_ADDR
#define BT_PDS_RRAMC_REGION_ADDR	0x00158800UL
#endif
#ifndef BT_PDS_RRAMC_REGION_SIZE
#define BT_PDS_RRAMC_REGION_SIZE	0x00001000UL	// 4 KB
#endif
#ifndef BT_PDS_RRAMC_SECTOR_SIZE
#define BT_PDS_RRAMC_SECTOR_SIZE	0x00000400UL	// 1 KB
#endif

#define BT_PDS_RRAMC_WRITE_GRAN		4				// RRAMC word based

// Worst-case microseconds for one write/erase Step inside a timeslot. RRAM
// word writes are a few microseconds; a 1 KB sector 0xFF-fill is well under a
// millisecond. Budget generously but within MPSL bounds.
#define BT_PDS_RRAMC_STEP_BUDGET_US	2000

// ---- RRAMC low-level write -------------------------------------------------
// Enable RRAMC write mode, write words to the mapped address, wait for the
// write buffer to drain, then restore. Runs inside the MPSL timeslot callback,
// so it must be short and non-blocking on radio; RRAM commit is fast.

static void RramcWriteEnable(bool bEnable)
{
	nrf_rramc_config_t cfg;
	nrf_rramc_config_get(NRF_RRAMC, &cfg);
	cfg.mode_write = bEnable;
	nrf_rramc_config_set(NRF_RRAMC, &cfg);
}

static void RramcWriteWords(uint32_t *pDst, const uint32_t *pSrc, uint32_t words)
{
	for (uint32_t i = 0; i < words; i++)
	{
		pDst[i] = pSrc[i];
		// Wait for the write buffer to accept/commit before issuing the next.
		while (!nrf_rramc_write_ready_check(NRF_RRAMC))
		{
		}
	}
	// Ensure the buffer is fully drained before returning.
	while (!nrf_rramc_empty_buffer_check(NRF_RRAMC))
	{
	}
}

// ---- Operation contexts run inside the timeslot ----------------------------

typedef struct {
	uint32_t	DstAddr;		// absolute RRAM address
	const uint32_t *pSrc;		// source words
	uint32_t	Words;			// word count
} RramcWriteCtx_t;

static uint32_t RramcWriteStep(void *pv)
{
	RramcWriteCtx_t *c = (RramcWriteCtx_t *)pv;

	RramcWriteEnable(true);
	RramcWriteWords((uint32_t *)c->DstAddr, c->pSrc, c->Words);
	RramcWriteEnable(false);

	return 0;	// done in one timeslot
}

// 0xFF fill for Erase. RRAM is rewrite-in-place; writing the erased pattern
// makes the store see the sector as free. A 1 KB sector fits one timeslot.
typedef struct {
	uint32_t	Addr;			// absolute RRAM address
	uint32_t	Words;			// words to fill with 0xFFFFFFFF
} RramcFillCtx_t;

static uint32_t RramcFillStep(void *pv)
{
	RramcFillCtx_t *c = (RramcFillCtx_t *)pv;
	static const uint32_t ones = 0xFFFFFFFFUL;

	RramcWriteEnable(true);
	uint32_t *dst = (uint32_t *)c->Addr;
	for (uint32_t i = 0; i < c->Words; i++)
	{
		dst[i] = ones;
		while (!nrf_rramc_write_ready_check(NRF_RRAMC))
		{
		}
	}
	while (!nrf_rramc_empty_buffer_check(NRF_RRAMC))
	{
	}
	RramcWriteEnable(false);

	return 0;	// done in one timeslot
}

// ---- BtPdsNvm_t primitives -------------------------------------------------

static int RramcNvmRead(uint32_t Off, void *pBuf, uint32_t Len)
{
	// RRAM is memory mapped; read directly.
	memcpy(pBuf, (const void *)(BT_PDS_RRAMC_REGION_ADDR + Off), Len);
	return 0;
}

static int RramcNvmWrite(uint32_t Off, const void *pData, uint32_t Len)
{
	// Len is a multiple of WriteGran (4) per the BtPdsNvm_t contract.
	RramcWriteCtx_t ctx;
	ctx.DstAddr = BT_PDS_RRAMC_REGION_ADDR + Off;
	ctx.pSrc    = (const uint32_t *)pData;
	ctx.Words   = Len / BT_PDS_RRAMC_WRITE_GRAN;

	BtPdsMpslOp_t op;
	op.Step         = RramcWriteStep;
	op.StepBudgetUs = BT_PDS_RRAMC_STEP_BUDGET_US;
	op.pCtx         = &ctx;

	return BtPdsMpslRun(&op);
}

static int RramcNvmErase(uint32_t Off)
{
	RramcFillCtx_t ctx;
	ctx.Addr  = BT_PDS_RRAMC_REGION_ADDR + Off;
	ctx.Words = BT_PDS_RRAMC_SECTOR_SIZE / BT_PDS_RRAMC_WRITE_GRAN;

	BtPdsMpslOp_t op;
	op.Step         = RramcFillStep;
	op.StepBudgetUs = BT_PDS_RRAMC_STEP_BUDGET_US;
	op.pCtx         = &ctx;

	return BtPdsMpslRun(&op);
}

static const BtPdsNvm_t s_BtPdsRramcSdcNvm = {
	.RegionOffset = BT_PDS_RRAMC_REGION_ADDR,
	.RegionSize   = BT_PDS_RRAMC_REGION_SIZE,
	.SectorSize   = BT_PDS_RRAMC_SECTOR_SIZE,
	.WriteGran    = BT_PDS_RRAMC_WRITE_GRAN,
	.Read         = RramcNvmRead,
	.Write        = RramcNvmWrite,
	.Erase        = RramcNvmErase,
};

// Init entry: bring up the MPSL timeslot session, then mount the store.
int BtPdsSdcNvmInit(void)
{
	int r = BtPdsMpslInit();
	if (r != 0)
	{
		return r;
	}
	return BtPdsInit(&s_BtPdsRramcSdcNvm);
}
