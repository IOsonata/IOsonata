/**-------------------------------------------------------------------------
@file	bt_smp_bond_nvm.cpp

@brief	Persistent bond storage glue over an Nvm.

		The generic SMP bond table keeps bonds in RAM and calls the weak
		BtSmpBondSave / BtSmpBondLoad / BtSmpBondErase hooks. This file
		provides strong overrides backed by the generic transactional PDS.

		Each bond slot maps to BOND_KEY_BASE + Slot. The record is opaque here;
		BtSmpBondSerialize builds the exact blob when the application-context
		save handler is ready to write it.

@author	Hoang Nguyen Hoan
@date	Jun 09, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <atomic>

#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_pds.h"
#include "crypto/icrypto.h"
#include "storage/nvm.h"
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"
#include "app_evt_handler.h"

// The generic bond layer only makes a signed-write counter range usable after
// the persistence backend confirms that the exact serialized record reached
// the medium. This is internal to the bond-store implementation.
extern "C" void BtSmpBondPersistComplete(int Slot, const void *pBond,
											 size_t Len, bool Success);

#define BT_SMP_BOND_TRACE

#ifdef BT_SMP_BOND_TRACE
#include "syslog.h"
#define BOND_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define BOND_PRINTF(...)
#endif

#define BT_SMP_BOND_KEY_BASE	0x42440000u		// 'B''D' 00 00
#define BT_SMP_LOCALID_KEY		(BT_SMP_BOND_KEY_BASE + 0x100u)

static bool s_PdsReady;
static bool s_PdsArmed;

#ifndef BT_SMP_BOND_REGION_NO
#define BT_SMP_BOND_REGION_NO		0
#endif

static NvmIntrf s_BondIntrf;
static Nvm s_BondMem;

static int PdsEnsureReady(void)
{
	if (s_PdsArmed == false)
	{
		return -EPERM;
	}
	if (s_PdsReady)
	{
		return 0;
	}

	BOND_PRINTF("PDS: mount, store on Nvm (not fstorage/fds)\r\n");

	if (s_BondIntrf.Init() == false)
	{
		BOND_PRINTF("PDS: memory interface init failed\r\n");
		return -EIO;
	}

	uintptr_t base = NvmRegionAddr(BT_SMP_BOND_REGION_NO);
	size_t size = NvmRegionSize(BT_SMP_BOND_REGION_NO);

	BOND_PRINTF("PDS: linker NVM%d at %08lX size %08lX\r\n",
				BT_SMP_BOND_REGION_NO, (unsigned long)base,
				(unsigned long)size);

	if (base == 0 || size == 0)
	{
		BOND_PRINTF("PDS: no NVM%d region in the linker script, "
					"bonds will not persist\r\n", BT_SMP_BOND_REGION_NO);
		return -ENODEV;
	}

	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	NvmMcuCfg(cfg);
	cfg.BaseAddr = base;
	cfg.TotalSize = size;

	BOND_PRINTF("PDS: dev %lu bytes, erase %lu, sect %lu, page %lu, wr %lu, addr %lu\r\n",
				(unsigned long)cfg.TotalSize, (unsigned long)cfg.EraseSize,
				(unsigned long)cfg.SectorSize, (unsigned long)cfg.PageSize,
				(unsigned long)cfg.WriteGran, (unsigned long)cfg.AddrSize);

	if (s_BondMem.Init(cfg, &s_BondIntrf, 0, 0) == false)
	{
		BOND_PRINTF("PDS: Nvm init failed\r\n");
		return -EIO;
	}

	BOND_PRINTF("PDS: Nvm ok, size %lu, sector %lu, wr %lu\r\n",
				(unsigned long)s_BondMem.Size(),
				(unsigned long)s_BondMem.LogicalSectorSize(),
				(unsigned long)s_BondMem.WriteGran());

	uint32_t first = 0;
	int rd = s_BondMem.Read(0, &first, sizeof(first));
	BOND_PRINTF("PDS: raw [0] = %08lX (read %d)\r\n",
				(unsigned long)first, rd);

	int r = BtPdsInit(&s_BondMem);
	if (r != 0)
	{
		BOND_PRINTF("PDS: store mount failed %d\r\n", r);
		return r;
	}

	size_t rsz = BtSmpBondRecordSize();
	if (rsz > BT_PDS_RECORD_DATA_MAX)
	{
		BOND_PRINTF("PDS: bond record %u exceeds BT_PDS_RECORD_DATA_MAX %u, "
					"nothing will persist\r\n",
					(unsigned)rsz, (unsigned)BT_PDS_RECORD_DATA_MAX);
		return -EINVAL;
	}

	BOND_PRINTF("PDS: store mounted, bond record %u of max %u\r\n",
				(unsigned)rsz, (unsigned)BT_PDS_RECORD_DATA_MAX);

	s_PdsReady = true;
	return 0;
}

// A save can originate inside Bluetooth event dispatch. NVM access is deferred
// to the application handler because Nordic radio arbitration cannot complete
// while the higher-priority stack callback is still running.
#define BT_SMP_BOND_PEND_MAX		32

static std::atomic<uint32_t> s_PendMask;
static uint32_t s_PendDropCnt;
static uint32_t s_PendFailCnt;
static std::atomic<bool> s_LocalIdPend;

static void BondSaveHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	int r = PdsEnsureReady();
	if (r != 0)
	{
		uint32_t left = s_PendMask.load();
		if (left != 0)
		{
			BOND_PRINTF("PDS: save mask %08lX held, store not ready %d\r\n",
						(unsigned long)left, r);
		}
		return;
	}

	uint8_t rec[BT_PDS_RECORD_DATA_MAX];

	if (s_LocalIdPend.exchange(false))
	{
		size_t len = BtSmpLocalIdSerialize(rec, sizeof(rec));
		if (len != 0)
		{
			ssize_t w = BtPdsWrite(BT_SMP_LOCALID_KEY, rec, len);
			if (w != (ssize_t)len)
			{
				s_LocalIdPend.store(true);
				BOND_PRINTF("PDS: local id save len %u failed %d, still marked\r\n",
							(unsigned)len, (int)w);
			}
		}
	}

	for (int slot = 0; slot < BT_SMP_BOND_PEND_MAX; slot++)
	{
		if (s_PendMask.load() == 0)
		{
			break;
		}

		uint32_t bit = 1UL << slot;

		// Clear before serialization. A change arriving afterward marks the slot
		// again, so it is written a second time instead of being lost.
		if ((s_PendMask.fetch_and(~bit) & bit) == 0)
		{
			continue;
		}

		size_t len = BtSmpBondSerialize(slot, rec, sizeof(rec));
		if (len == 0)
		{
			BOND_PRINTF("PDS: save slot %d refused by the bond table\r\n", slot);
			continue;
		}

		ssize_t w = BtPdsWrite(BT_SMP_BOND_KEY_BASE + (uint32_t)slot,
							 rec, len);
		bool committed = w == (ssize_t)len;

		// A signed-write counter window is unusable until this acknowledgment.
		// Pass the exact record written so a later reservation cannot be
		// mistaken for the one that just reached the medium.
		BtSmpBondPersistComplete(slot, rec, len, committed);

		if (!committed)
		{
			s_PendMask.fetch_or(bit);
			s_PendFailCnt++;
			BOND_PRINTF("PDS: save slot %d len %u failed %d, still marked, "
						"%lu so far\r\n", slot, (unsigned)len, (int)w,
						(unsigned long)s_PendFailCnt);
			break;
		}

		BOND_PRINTF("PDS: save slot %d len %u -> %d\r\n", slot,
					(unsigned)len, (int)w);
	}

	CryptoSecureWipe(rec, sizeof(rec));
}

void BtSmpBondSave(int Slot, const void *pBond, size_t Len)
{
	(void)pBond;
	(void)Len;

	if (Slot < 0 || Slot >= BT_SMP_BOND_PEND_MAX)
	{
		s_PendDropCnt++;
		BOND_PRINTF("PDS: save slot %d dropped, past the %d that can be "
					"marked, %lu so far\r\n", Slot, BT_SMP_BOND_PEND_MAX,
					(unsigned long)s_PendDropCnt);
		return;
	}

	s_PendMask.fetch_or(1UL << Slot);

	if (AppEvtHandlerQue(0, NULL, BondSaveHandler) == false)
	{
		BOND_PRINTF("PDS: save slot %d not queued yet, event queue full\r\n",
					Slot);
	}
}

void BtSmpLocalIdSave(void)
{
	s_LocalIdPend.store(true);

	if (AppEvtHandlerQue(0, NULL, BondSaveHandler) == false)
	{
		BOND_PRINTF("PDS: local id save not queued yet, event queue full\r\n");
	}
}

void BtSmpBondLoad(void)
{
	int r = PdsEnsureReady();
	if (r != 0)
	{
		BOND_PRINTF("PDS: load skipped, store not ready %d\r\n", r);
		return;
	}

	int slots = BtSmpBondSlotCount();
	size_t rsz = BtSmpBondRecordSize();
	if (rsz == 0 || rsz > 256)
	{
		BOND_PRINTF("PDS: load refused, record size %u\r\n", (unsigned)rsz);
		return;
	}

	uint8_t blob[256];
	int found = 0;
	for (int s = 0; s < slots; s++)
	{
		ssize_t n = BtPdsRead(BT_SMP_BOND_KEY_BASE + (uint32_t)s, blob, rsz);
		if (n == (ssize_t)rsz)
		{
			BtSmpBondRestore(s, blob, rsz);
			found++;
		}
		else if (n != -ENOENT)
		{
			BOND_PRINTF("PDS: load slot %d -> %d (want %u)\r\n", s,
						(int)n, (unsigned)rsz);
		}
	}

	size_t isz = BtSmpLocalIdRecordSize();
	ssize_t n = BtPdsRead(BT_SMP_LOCALID_KEY, blob, isz);
	if (n == (ssize_t)isz)
	{
		if (BtSmpLocalIdRestore(blob, isz))
		{
			BOND_PRINTF("PDS: local identity restored\r\n");
		}
		else
		{
			BOND_PRINTF("PDS: local identity record rejected\r\n");
		}
	}
	else if (n != -ENOENT)
	{
		BOND_PRINTF("PDS: local identity load -> %d (want %u)\r\n",
					(int)n, (unsigned)isz);
	}

	BOND_PRINTF("PDS: load done, %d of %d slots restored\r\n", found, slots);
}

void BtSmpBondErase(void)
{
	if (PdsEnsureReady() != 0)
	{
		return;
	}

	int r = BtPdsClear();
	if (r != 0)
	{
		BOND_PRINTF("PDS: clear failed %d, the stored bonds may or may not "
					"be gone\r\n", r);
	}
}

int BtSmpBondNvmInit(void)
{
	BOND_PRINTF("PDS: BtSmpBondNvmInit, arming the store\r\n");

	if (BtSmpBondSlotCount() > BT_SMP_BOND_PEND_MAX)
	{
		BOND_PRINTF("PDS: %d bond slots, only %d can be marked, refusing\r\n",
					BtSmpBondSlotCount(), BT_SMP_BOND_PEND_MAX);
		return -ENOTSUP;
	}

	s_PdsArmed = true;

	int r = PdsEnsureReady();
	if (r != 0)
	{
		BOND_PRINTF("PDS: init failed %d, bonds will not survive a reset\r\n", r);
		return r;
	}

	BtSmpBondLoad();

	// Restore reserves a fresh counter window above every persisted high-water
	// mark. Initialization already runs in application context, so commit those
	// reservations now and return only after they are durable. A queued copy of
	// the same handler is harmless: the pending mask is empty afterward.
	BondSaveHandler(0, nullptr);

	return 0;
}
