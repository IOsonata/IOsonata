/**-------------------------------------------------------------------------
@file	bt_pds.h

@brief	Persistent Data Store for the Bluetooth peer manager.

		A small, id-keyed, variable length record store over a single
		non-volatile region. It replaces the sdk-nrf-bm ZMS filesystem
		(bm_zms / bm_storage) that peer_data_storage.c depended on, removing
		the Zephyr dependency while keeping peer_manager unchanged above it.

		The store is generic and platform independent. It performs all record
		management (lookup, append, tombstone, compaction) and calls a thin
		BtPdsNvm implementation for the actual medium access. Each target provides a
		implementation: RRAM through the SoftDevice on the bm port, flash on SDC, etc.

		Keying and semantics match what peer_data_storage.c expects from the
		old bm_zms API:
		  - Id is a 32-bit key (peer manager packs peer_id<<16 | data_id).
		  - Read returns the byte count of the stored value, or -ENOENT.
		  - Write upserts; Delete removes.
		Operations are synchronous: when a call returns, the medium reflects
		the result. peer_data_storage.c emits its peer-manager completion
		events inline after each call rather than via an async callback.

@author	Hoang Nguyen Hoan
@date	Jun 03, 2026

@license

MIT License

Copyright (c) 2016, I-SYST inc., all rights reserved

----------------------------------------------------------------------------*/
#ifndef __BT_PDS_H__
#define __BT_PDS_H__

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>		// ssize_t

// Largest value the store must hold. Matches PM_PEER_DATA_MAX_SIZE
// (PM_PEER_DATA_LOCAL_GATT_DB_MAX_SIZE = 128) in the peer manager.
#define BT_PDS_RECORD_DATA_MAX		128

/**
 * @brief	NVM implementation interface.
 *
 * The store accesses the medium only through these calls. A platform supplies
 * one instance describing its region and the read/write/erase primitives.
 *
 * Offsets are relative to the start of the region (0 .. RegionSize-1), not
 * absolute addresses, so the store logic is medium agnostic.
 *
 * Write must be synchronous from the caller's view: on return the data is
 * committed (the bm implementation pumps sd_flash_write to completion). Len passed to
 * Write is always a multiple of the implementation WriteGran. Erase clears a sector
 * to the erased value (0xFF bytes) if the medium needs it; on rewrite-in-place
 * media such as RRAM it may simply overwrite, in which case Erase can write
 * the sector with the erased pattern or be a documented no-op the store
 * accounts for.
 */
typedef struct __Bt_Pds_Nvm {
	uint32_t	RegionOffset;	//!< Absolute medium offset of the region start
	uint32_t	RegionSize;		//!< Region size in bytes
	uint32_t	SectorSize;		//!< Erase/compaction unit in bytes
	uint32_t	WriteGran;		//!< Write granularity in bytes (4 for word media)

	// Read Len bytes from RegionOffset+Off into pBuf. Returns 0 on success.
	int (*Read)(uint32_t Off, void *pBuf, uint32_t Len);

	// Write Len bytes (multiple of WriteGran) from pData to RegionOffset+Off.
	// Synchronous: committed on return. Returns 0 on success.
	int (*Write)(uint32_t Off, const void *pData, uint32_t Len);

	// Erase the sector at RegionOffset+Off (SectorSize bytes). Returns 0 on
	// success. May overwrite with the erased pattern on RRAM.
	int (*Erase)(uint32_t Off);
} BtPdsNvm_t;

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief	Initialize the store on the given implementation.
 *
 * Scans the region, rebuilds the in-RAM index of live records, and runs
 * recovery if a prior compaction was interrupted. Synchronous.
 *
 * @param	pNvm	Implementation describing the region and medium primitives.
 *
 * @return	0 on success, negative errno on failure.
 */
int BtPdsInit(const BtPdsNvm_t *pNvm);

/**
 * @brief	Read the value stored under Id.
 *
 * @param	Id		32-bit record key.
 * @param	pBuf	Destination buffer.
 * @param	Len		Size of pBuf.
 *
 * @return	Number of bytes copied (the stored value length) on success,
 *			-ENOENT if no live record exists for Id,
 *			negative errno on error.
 */
ssize_t BtPdsRead(uint32_t Id, void *pBuf, size_t Len);

/**
 * @brief	Store (upsert) a value under Id.
 *
 * Appends a new record version. The previous version for Id becomes stale and
 * is reclaimed at the next compaction. Triggers compaction if the region is
 * too full to hold the new record.
 *
 * @param	Id		32-bit record key.
 * @param	pData	Value bytes.
 * @param	Len		Value length, 0 .. BT_PDS_RECORD_DATA_MAX.
 *
 * @return	Number of bytes written (Len) on success, negative errno on error
 *			(-ENOMEM if the region is full even after compaction).
 */
ssize_t BtPdsWrite(uint32_t Id, const void *pData, size_t Len);

/**
 * @brief	Delete the value stored under Id.
 *
 * Appends a tombstone. Returns 0 even if Id was absent (idempotent).
 *
 * @param	Id		32-bit record key.
 *
 * @return	0 on success, negative errno on error.
 */
int BtPdsDelete(uint32_t Id);

/**
 * @brief	Erase the entire store (all records).
 *
 * @return	0 on success, negative errno on error.
 */
int BtPdsClear(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_PDS_H__
