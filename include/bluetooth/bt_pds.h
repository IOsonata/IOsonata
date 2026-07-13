/**-------------------------------------------------------------------------
@file	bt_pds.h

@brief	Persistent Data Store for Bluetooth peer data.

		A small, id-keyed, variable-length record store over a fixed NVM region.
		The region is divided into erase sectors. Records are appended inside
		sectors, and one erased sector is retained for garbage collection.

		Garbage collection is transactional at sector level. Live records are
		copied to the spare sector one at a time, a completion marker is committed,
		and only then is the source sector erased. BtPdsInit resumes an interrupted
		copy or erase. No heap or live-record staging array is used.

		The store is platform independent. Medium access is supplied by a
		BtPdsNvm_t implementation. The current on-medium format requires 4-byte
		write granularity.

		Operations are synchronous: when a call returns successfully, the medium
		contains the committed result.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2016-2026 I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef __BT_PDS_H__
#define __BT_PDS_H__

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

// Largest value required by the current Peer Manager data types.
#define BT_PDS_RECORD_DATA_MAX		128

/**
 * @brief NVM implementation used by BtPds.
 *
 * Offsets passed to Read, Write and Erase are relative to RegionOffset.
 * RegionSize must be an integral number of SectorSize units and contain at
 * least two sectors. WriteGran is currently required to be 4 bytes.
 *
 * Write and Erase must be synchronous from the caller's view. Write receives
 * aligned offsets, aligned source buffers and lengths that are multiples of
 * WriteGran. Erase operates on one SectorSize unit.
 */
typedef struct __Bt_Pds_Nvm {
	uint32_t	RegionOffset;	//!< Absolute medium address of the region
	uint32_t	RegionSize;		//!< Total region size in bytes
	uint32_t	SectorSize;		//!< Erase and garbage-collection unit
	uint32_t	WriteGran;		//!< Write granularity, currently 4 bytes

	int (*Read)(uint32_t Off, void *pBuf, uint32_t Len);
	int (*Write)(uint32_t Off, const void *pData, uint32_t Len);
	int (*Erase)(uint32_t Off);
} BtPdsNvm_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mount the store on an NVM implementation.
 *
 * Scans every sector, recovers an interrupted garbage collection, and selects
 * the newest writable sector. The earlier development format without sector
 * headers is cleared once because it cannot be upgraded in place while
 * preserving the power-loss guarantee.
 *
 * @return 0 on success, negative errno on failure.
 */
int BtPdsInit(const BtPdsNvm_t *pNvm);

/**
 * @brief Read the latest live value stored under Id.
 *
 * Up to Len bytes are copied. The return value is the full stored length, so a
 * caller can detect that its destination was smaller than the record.
 *
 * @return Stored length, -ENOENT when absent/deleted, or negative errno.
 */
ssize_t BtPdsRead(uint32_t Id, void *pBuf, size_t Len);

/**
 * @brief Append a new value for Id.
 *
 * Triggers sector garbage collection when the active sector cannot hold the
 * record. Id 0xFFFFFFFE is reserved internally.
 *
 * @return Len on success or negative errno. -ENOMEM means the live set plus
 * the requested record cannot fit while retaining a spare sector.
 */
ssize_t BtPdsWrite(uint32_t Id, const void *pData, size_t Len);

/**
 * @brief Delete Id by appending a tombstone.
 *
 * Deleting an absent value succeeds without writing.
 *
 * @return 0 on success or negative errno.
 */
int BtPdsDelete(uint32_t Id);

/**
 * @brief Erase all records and create a new empty active sector.
 *
 * @return 0 on success or negative errno.
 */
int BtPdsClear(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_PDS_H__
