/**-------------------------------------------------------------------------
@file	bt_smp_bond_stm32wba.cpp

@brief	Persistent bond storage glue for STM32WBA (ST host) builds.

	WBA differs from the SDC path in WHERE the bond data comes from:

	  SDC : IOsonata's SMP produces each bond record; the generic
	        bt_smp_bond.cpp RAM table holds it and the weak hooks persist
	        individual slots.
	  WBA : ST's host owns the bonding table. The stack keeps it in an NVM
	        cache (RAM mirror) and asks the platform to flush it via
	        BLEPLAT_NvmStore. The bond is an OPAQUE BLOB to IOsonata.

	Because the bt_smp_bond / bt_pds seam was built to store opaque blobs
	through a swappable BtPdsNvm_t implementation (see bt_smp_bond_sdc.cpp comment:
	"the record is an opaque blob ... this file does not need the layout"),
	the WBA cache image drops straight into the same store. Only the medium
	implementation differs (bt_pds_stm32wba.cpp).

	Layering:
	    ST stack NVM cache  --BLEPLAT_NvmStore-->  THIS FILE
	      -> bt_pds.cpp (KV over a region)
	        -> bt_pds_stm32wba.cpp (WBA flash medium)

	STATUS:
	  [VERIFIED] BLEPLAT_NvmStore signature: stm32-mw-wpan bleplat.h
	             ( void BLEPLAT_NvmStore(const uint64_t *ptr, uint16_t size) ;
	               size is in 64-bit words ).
	  [VERIFIED] BtPdsWrite/BtPdsRead signatures: IOsonata bt_pds.h.
	  [HARDWARE] the restore-at-init path: ST expects the cache to be
	             preloaded before BleStack_Init. The exact preload hook and
	             cache buffer pointer come from the ST stack init params -
	             confirm on the SDK App reference before trusting on board.

@author	Hoang Nguyen Hoan (port skeleton)
@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "bluetooth/bt_pds.h"

// Forward decl of the WBA flash implementation init (bt_pds_stm32wba.cpp).
extern "C" int BtPdsWbaInit(void);

// bt_pds key holding the ST NVM-cache blob. The WBA bond table is a single
// opaque image (not per-slot like SDC), so one key suffices. Kept in a
// dedicated range to avoid collision with anything added later.
#define BT_WBA_NVM_KEY		0x57424E56u		// 'W''B''N''V'  [arbitrary, stable]

static bool s_PdsReady;

static int PdsEnsureReady(void)
{
	if (s_PdsReady)
	{
		return 0;
	}
	int r = BtPdsWbaInit();			// [VERIFIED] bt_pds.h implementation init
	if (r != 0)
	{
		return r;
	}
	s_PdsReady = true;
	return 0;
}

// ---------------------------------------------------------------------------
// BLEPLAT_NvmStore - called by the ST stack to flush (part of) its NVM cache.
// [VERIFIED] exact signature from stm32-mw-wpan bleplat.h. size is in 64-bit
// words; valid data always starts at the beginning of the cache buffer.
//
// Simplest correct policy: persist the whole reported prefix as one bt_pds
// record. [HARDWARE] If the cache is large, a smarter diff/segmented scheme
// reduces flash wear; not needed for first bring-up.
// ---------------------------------------------------------------------------
extern "C" void BLEPLAT_NvmStore(const uint64_t *ptr, uint16_t size)
{
	if (ptr == NULL || size == 0)
	{
		return;
	}
	if (PdsEnsureReady() != 0)
	{
		return;
	}
	size_t bytes = (size_t)size * sizeof(uint64_t);	// words -> bytes [VERIFIED]
	(void)BtPdsWrite(BT_WBA_NVM_KEY, ptr, bytes);
}

// ---------------------------------------------------------------------------
// Restore the cache image into the buffer the ST stack will use, BEFORE
// BleStack_Init. Returns bytes loaded, 0 if none (first boot).
// [HARDWARE] caller must pass the stack's NVM cache buffer + capacity; wire
// this to the ST init param that points at the cache. Confirm the param name
// in the SDK App example (host_event_fifo / nvm cache buffer in init struct).
// ---------------------------------------------------------------------------
extern "C" size_t BtSmpBondWbaRestore(void *pCacheBuf, size_t Capacity)
{
	if (pCacheBuf == NULL || Capacity == 0)
	{
		return 0;
	}
	if (PdsEnsureReady() != 0)
	{
		return 0;
	}
	ssize_t n = BtPdsRead(BT_WBA_NVM_KEY, pCacheBuf, Capacity);	// [VERIFIED]
	return (n > 0) ? (size_t)n : 0;
}

// ---------------------------------------------------------------------------
// Commit trigger from the pairing-complete handler (bt_app_stm32wba.cpp).
// On WBA the stack flushes through BLEPLAT_NvmStore on its own schedule, so
// this is usually a no-op marker. Kept as the named entry the patch calls so
// the integration point is explicit and greppable.
// [HARDWARE] if the stack does NOT auto-flush on pairing complete for the
// chosen config, force a flush here via the ST API that triggers NvmStore.
// ---------------------------------------------------------------------------
extern "C" void BtSmpBondWbaCommit(uint16_t ConnHdl)
{
	(void)ConnHdl;
	// Intentionally empty for first bring-up: ST stack drives BLEPLAT_NvmStore.
}

// ---------------------------------------------------------------------------
// Init entry point, called internally from bt_app_stm32wba.cpp gated on
// bSecure (mirrors the BtSmpBondSdcInit placement rule). Ensures the store +
// implementation are up so the first BLEPLAT_NvmStore/Restore succeeds. Having a
// referenced entry point also guarantees this TU is pulled from the archive
// (the weak/strong archive-extraction lesson).
// ---------------------------------------------------------------------------
extern "C" int BtSmpBondWbaInit(void)
{
	return PdsEnsureReady();
}
