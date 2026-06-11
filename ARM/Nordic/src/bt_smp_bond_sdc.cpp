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
		          -> BtPdsNvm_t backend, radio safe per target:
		             nRF54L SDC : bt_pds_nvm_rramc_sdc.cpp (RRAMC via MPSL)
		             nRF52  SDC : bt_pds_nvm_nvmc_sdc.cpp  (NVMC via MPSL)

		The backend init is provided by the linked backend translation unit
		(BtPdsSdcNvmInit). Only one backend is linked per build, so this glue
		stays target neutral: it never touches RRAMC, NVMC, or MPSL directly.

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
#include "bt_pds_sdc.h"

// bt_pds key for bond slot N. Kept in a dedicated range so other persisted
// items (if added later) do not collide with bond slots.
#define BT_SMP_BOND_KEY_BASE	0x42440000u		// 'B''D' 00 00

static bool s_PdsReady;

// Bring up the KV store and its backend once. Returns 0 on success.
static int PdsEnsureReady(void)
{
	if (s_PdsReady)
	{
		return 0;
	}
	int r = BtPdsSdcNvmInit();
	if (r != 0)
	{
		return r;
	}
	s_PdsReady = true;
	return 0;
}

// Persist one bond slot. The generic layer calls this whenever slot Slot
// changes. pBond points at Len bytes (BtSmpBondRecordSize()).
void BtSmpBondSave(int Slot, const void *pBond, size_t Len)
{
	if (pBond == NULL || Len == 0)
	{
		return;
	}
	if (PdsEnsureReady() != 0)
	{
		return;
	}
	(void)BtPdsWrite(BT_SMP_BOND_KEY_BASE + (uint32_t)Slot, pBond, Len);
}

// Load all persisted bonds into the RAM table at init. Reads each slot key and
// hands the blob to BtSmpBondRestore. Missing slots are skipped.
void BtSmpBondLoad(void)
{
	if (PdsEnsureReady() != 0)
	{
		return;
	}

	int slots = BtSmpBondSlotCount();
	size_t rsz = BtSmpBondRecordSize();
	if (rsz == 0 || rsz > 256)
	{
		return;
	}

	uint8_t blob[256];
	for (int s = 0; s < slots; s++)
	{
		ssize_t n = BtPdsRead(BT_SMP_BOND_KEY_BASE + (uint32_t)s, blob, rsz);
		if (n == (ssize_t)rsz)
		{
			BtSmpBondRestore(s, blob, rsz);
		}
	}
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
//   2. Bring up the NVM backend and load any stored bonds into the RAM table
//      now, so a reconnect right after boot finds its LTK.
// Returns 0 on success, negative errno on backend init failure.
int BtSmpBondSdcInit(void)
{
	int r = PdsEnsureReady();
	if (r != 0)
	{
		return r;
	}
	BtSmpBondLoad();
	return 0;
}
