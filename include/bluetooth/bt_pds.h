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
		NvmIO implementation. The current on-medium format requires 4-byte
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

#ifdef __cplusplus
#include "storage/nvmio.h"
#endif

// Largest value required by the current Peer Manager data types.
#define BT_PDS_RECORD_DATA_MAX		128

/**
 * @brief Mount the store on a persistent memory device.
 *
 * The device is an NvmIO whose region is the store area. Its EraseSize is the
 * sector and garbage-collection unit; the region must be an integral number of
 * sectors and contain at least two. WriteGran must be 4. Read, Write and Erase
 * must be synchronous from the caller's view: Write programs previously erased
 * cells at aligned offsets with lengths that are multiples of WriteGran, and
 * Erase clears whole sectors.
 *
 * Scans every sector, recovers or safely discards an interrupted garbage
 * collection, and selects the newest writable sector. Earlier development
 * formats are cleared once because they cannot be upgraded in place while
 * preserving the power-loss guarantee.
 *
 * @param pNvm : Persistent memory device holding the store region.
 *
 * @return 0 on success, negative errno on failure.
 */
#ifdef __cplusplus
int BtPdsInit(NvmIO * const pNvm);

extern "C" {
#endif

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
 * The clear commits when one sector header under a new clear epoch reaches
 * the medium. A power loss after that point cannot bring cleared records
 * back; a power loss before it leaves the store unchanged.
 *
 * @return 0 on success or negative errno.
 */
int BtPdsClear(void);

/**
 * @brief Erase all records with a non-atomic sequential format.
 *
 * Explicitly destructive and not power-loss atomic. BtPdsClear is the atomic
 * clear; it returns -ENOSPC when no erased spare is available to commit onto.
 * This entry performs the wipe anyway for callers that accept that risk.
 *
 * @return 0 on success, negative errno on failure.
 */
int BtPdsForceFormat(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_PDS_H__
