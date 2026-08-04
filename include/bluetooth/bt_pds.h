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

		The store is platform independent. It runs on an Nvm instance, so the
		same store serves MCU internal memory, serial flash, EEPROM and FRAM,
		and the arbitration a live radio needs stays inside the memory driver.
		The current on-medium format requires 4-byte write granularity.

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

// Largest value a record may hold. It has to cover the largest thing any
// consumer stores: the SMP bond record (BtSmpBondRecordSize(), 136 bytes at
// the time of writing) is the biggest one today, and it grows whenever a field
// is added to BtSmpBond_t. A value below that makes every save fail with
// -EINVAL and nothing is persisted.
//
// It sizes six stack buffers in the implementation, so raising it costs stack
// in the scan and garbage collection paths rather than medium space.
#ifndef BT_PDS_RECORD_DATA_MAX
#define BT_PDS_RECORD_DATA_MAX		192
#endif

#ifdef __cplusplus

class Nvm;

/**
 * @brief Mount the store on a memory.
 *
 * Scans every sector, recovers or safely discards an interrupted garbage
 * collection, and selects the newest writable sector. Earlier development
 * formats are cleared once because they cannot be upgraded in place while
 * preserving the power-loss guarantee.
 *
 * The memory supplies the geometry, so the region is set once where it is
 * initialised. Its write unit must be a power of two from 4 up to
 * BT_PDS_MAX_WRITE_GRAN and must divide the sector, its region must hold at
 * least two whole sectors, and the sector size is the erase and garbage
 * collection unit. Either completion mode works; each write and erase is
 * drained before the store moves on.
 *
 * Only declared for C++ because Nvm is a C++ class. The rest of the store is
 * a flat C API and stays reachable from C.
 *
 * @return 0 on success, negative errno on failure.
 */
int BtPdsInit(Nvm *pMem);

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
 * @brief Persist SMP bonds through this store.
 *
 * Mounts the store on the Nvm covering linker region
 * BT_SMP_BOND_REGION_NO, loads any stored bonds into the RAM bond table so a
 * reconnect right after boot finds its LTK, and arms the strong
 * BtSmpBondSave / BtSmpBondLoad / BtSmpBondErase overrides in
 * bt_smp_bond_nvm.cpp. Called by the port app init when a secure SecType is
 * configured, not by the application.
 *
 * Linking bt_smp_bond_nvm.cpp replaces the weak RAM-only bond hooks for the
 * whole build, so a build that does not want bonds on the memory should not
 * link it rather than not call this.
 *
 * @return 0 on success, negative errno on failure.
 */
int BtSmpBondNvmInit(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_PDS_H__
