/**-------------------------------------------------------------------------
@file	bt_smp_bond_sdc.cpp

@brief	Persistent bond storage glue for the SoftDevice Controller (SDC) builds.

		The generic SMP bond table (src/bluetooth/bt_smp_bond.cpp) keeps bonds
		in RAM and calls the weak hooks BtSmpBondSave / BtSmpBondLoad /
		BtSmpBondErase when a slot changes, at init, and on clear. This file
		provides strong overrides that persist the bond records through the
		generic bt_pds key value store, so bonds survive a reset.

		Layering:
		    bt_smp_bond.cpp (RAM table, weak hooks)
		      -> these strong hooks (slot <-> bt_pds key)
		        -> bt_pds.cpp (log structured KV over a region)
		          -> Nvm (nvm_nrfx.cpp: NVMC on nRF52, RRAMC on nRF54L, and
		             the radio arbitration that goes with either)

		The memory is brought up here because this is where the store is
		mounted. Nothing target specific is left to do beyond saying where the
		region is: the page size, the write unit and the sector size all come
		off the memory.

		Each bond slot maps to a fixed bt_pds key (BOND_KEY_BASE + Slot). The
		record is an opaque blob of BtSmpBondRecordSize() bytes; this file does
		not need the BtSmpBond_t layout.

@author	Hoang Nguyen Hoan
@date	Jun 09, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "bluetooth/bt_smp.h"
#include "bluetooth/bt_pds.h"
#include "crypto/icrypto.h"
#include "storage/nvm.h"
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"
#include "app_evt_handler.h"
#include "bt_pds_sdc.h"				// declares BtSmpBondSdcInit with C linkage

// Comment out to silence. Deliberately not guarded on NDEBUG: the release
// build is where the store has to be watched, and a silent mount failure is
// indistinguishable from a bond that was saved and not found again.
#define BT_SMP_BOND_TRACE

#ifdef BT_SMP_BOND_TRACE
#include "syslog.h"
#define BOND_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define BOND_PRINTF(...)
#endif

// bt_pds key for bond slot N. Kept in a dedicated range so other persisted
// items (if added later) do not collide with bond slots.
#define BT_SMP_BOND_KEY_BASE	0x42440000u		// 'B''D' 00 00

static bool s_PdsReady;

// Set by BtSmpBondSdcInit, which the application layer calls only when the
// device is secure. Linking this file overrides the weak RAM-only bond hooks
// for every build, so without this arming a broadcaster or an unsecured
// application would still mount the store, and format a blank region, the
// first time BtSmpInit loads bonds.
static bool s_PdsArmed;

// Which linker declared region the bonds live in. A project that wants them
// somewhere else moves the region in its linker script, not here.
#ifndef BT_SMP_BOND_REGION_NO
#define BT_SMP_BOND_REGION_NO		0
#endif

static NvmIntrf s_BondIntrf;
static Nvm s_BondMem;

// Bring up the memory and mount the store on it once. Returns 0 on success.
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

	// Nothing is set up here beyond the memory itself. Who may touch it and
	// when is registered with the interface by whoever owns the radio, and how
	// long a program takes is the memory's own timing.
	if (s_BondIntrf.Init() == false)
	{
		BOND_PRINTF("PDS: memory interface init failed\r\n");
		return -EIO;
	}

	// The region the linker set aside is the device: base and size come from
	// it, and the geometry from the part. Nothing here states an address, so
	// there is nothing to keep in sync with the linker script.
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
	BOND_PRINTF("PDS: region NVM%d at %08lX size %08lX\r\n",
				BT_SMP_BOND_REGION_NO, (unsigned long)base,
				(unsigned long)size);

	// The region is the whole of this device, so no window inside it.
	if (s_BondMem.Init(cfg, &s_BondIntrf, 0, 0) == false)
	{
		BOND_PRINTF("PDS: Nvm init failed\r\n");
		return -EIO;
	}

	BOND_PRINTF("PDS: Nvm ok, size %lu, sector %lu, wr %lu\r\n",
				(unsigned long)s_BondMem.Size(),
				(unsigned long)s_BondMem.LogicalSectorSize(),
				(unsigned long)s_BondMem.WriteGran());

	// What the region holds before the store touches it. All ones is a blank
	// region; anything else should be a sector header the scan recognises.
	uint32_t first = 0;
	int rd = s_BondMem.Read(0, &first, sizeof(first));
	BOND_PRINTF("PDS: raw [0] = %08lX (read %d)\r\n", (unsigned long)first, rd);

	int r = BtPdsInit(&s_BondMem);
	if (r != 0)
	{
		BOND_PRINTF("PDS: store mount failed %d\r\n", r);
		return r;
	}

	// A bond bigger than the store's record cap is rejected on every save,
	// which otherwise shows up only as an errno per pairing.
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

// Persist one bond slot. The generic layer calls this whenever slot Slot
// changes. pBond points at Len bytes (BtSmpBondRecordSize()).
// A save arrives from inside the stack event dispatch, at the priority the
// SMP state machine runs at. The memory is arbitrated with an MPSL timeslot,
// and BtPdsMpslRun waits for a callback that cannot preempt that context, so
// writing there ends the slot abnormally. The record is copied and the write
// runs from the application event handler instead, which is what nvm_demo
// does with its own writes for the same reason.
//
// pBond points at a caller stack buffer that is wiped on return, so the copy
// is not optional.
// One record is staged at a time. A second save for the same slot is the
// common case, and it supersedes the first, so one buffer holds it. A
// second save for a different slot is the case one buffer cannot hold;
// it is counted so a log says how often it happens rather than the bond
// simply not being there later.
static uint8_t s_PendRec[BT_PDS_RECORD_DATA_MAX];
static size_t s_PendLen;
static int s_PendSlot;
static volatile bool s_PendBusy;
static uint32_t s_PendDropCnt;

static void BondSaveHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	if (s_PendBusy == false)
	{
		return;
	}

	// Take the record before the write, not after. BtSmpBondSave runs at
	// the priority the stack dispatches at, so it can arrive part way
	// through the write below. Releasing the staging buffer first means
	// such a save stages a fresh record instead of overwriting the one
	// being written.
	uint8_t rec[BT_PDS_RECORD_DATA_MAX];
	size_t len = s_PendLen;
	int slot = s_PendSlot;

	if (len > sizeof(rec))
	{
		len = sizeof(rec);
	}
	memcpy(rec, s_PendRec, len);
	s_PendBusy = false;

	int r = PdsEnsureReady();
	if (r != 0)
	{
		BOND_PRINTF("PDS: save slot %d dropped, store not ready %d\r\n",
					slot, r);
		CryptoSecureWipe(rec, sizeof(rec));
		return;
	}

	ssize_t w = BtPdsWrite(BT_SMP_BOND_KEY_BASE + (uint32_t)slot,
						   rec, len);

	BOND_PRINTF("PDS: save slot %d len %u -> %d\r\n", slot,
				(unsigned)len, (int)w);

	CryptoSecureWipe(rec, sizeof(rec));
}

void BtSmpBondSave(int Slot, const void *pBond, size_t Len)
{
	if (pBond == NULL || Len == 0)
	{
		BOND_PRINTF("PDS: save slot %d ignored, no data\r\n", Slot);
		return;
	}
	if (Len > sizeof(s_PendRec))
	{
		BOND_PRINTF("PDS: save slot %d len %u exceeds %u\r\n", Slot,
					(unsigned)Len, (unsigned)sizeof(s_PendRec));
		return;
	}
	if (s_PendBusy && Slot != s_PendSlot)
	{
		// A record for another slot is already staged and one buffer holds
		// one. Keeping the staged one is the safer of the two, since it has
		// already been queued.
		s_PendDropCnt++;
		BOND_PRINTF("PDS: save slot %d dropped, slot %d staged, %lu so far\r\n",
					Slot, s_PendSlot, (unsigned long)s_PendDropCnt);
		return;
	}

	// Nothing staged, or the same slot again. The later record is the
	// current state of that bond, so it replaces the earlier one. Refusing
	// it kept the stale record and wrote that instead.
	memcpy(s_PendRec, pBond, Len);
	s_PendLen = Len;
	s_PendSlot = Slot;
	s_PendBusy = true;

	if (AppEvtHandlerQue(0, NULL, BondSaveHandler) == false)
	{
		// The record stays staged. The next save queues the handler again,
		// so a queue that is full for a moment does not lose the bond.
		BOND_PRINTF("PDS: save slot %d not queued yet, event queue full\r\n",
					Slot);
	}
}

// Load all persisted bonds into the RAM table at init. Reads each slot key and
// hands the blob to BtSmpBondRestore. Missing slots are skipped.
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

	BOND_PRINTF("PDS: load done, %d of %d slots restored\r\n", found, slots);
}

// Wipe all persisted bonds. Called from BtSmpBondClearAll so stale records do
// not reappear on the next BtSmpBondLoad.
void BtSmpBondErase(void)
{
	if (PdsEnsureReady() != 0)
	{
		return;
	}
	(void)BtPdsClear();
}

// Explicit init entry. The application MUST call this once during setup (after
// BtSmpInit). Its purpose is twofold:
//   1. Force the linker to pull this object from the static library. The bond
//      hooks above are strong overrides of the weak BtSmpBondSave/Load/Erase in
//      bt_smp_bond.cpp; in an archive, the linker only extracts this object if
//      something references a symbol it defines. Without this referenced entry,
//      the already-linked weak no-ops win and bonds are never persisted.
//   2. Bring up the NVM implementation and load any stored bonds into the RAM table
//      now, so a reconnect right after boot finds its LTK.
// Returns 0 on success, negative errno on implementation init failure.
int BtSmpBondSdcInit(void)
{
	BOND_PRINTF("PDS: BtSmpBondSdcInit, arming the store\r\n");

	s_PdsArmed = true;

	int r = PdsEnsureReady();
	if (r != 0)
	{
		BOND_PRINTF("PDS: init failed %d, bonds will not survive a reset\r\n", r);
		return r;
	}

	BtSmpBondLoad();

	return 0;
}
