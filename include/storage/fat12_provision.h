/**-------------------------------------------------------------------------
@file	fat12_provision.h

@brief	Optional first-use FAT12 provisioning over any DiskIO backend.

@license
MIT License
Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/
#ifndef __FAT12_PROVISION_H__
#define __FAT12_PROVISION_H__

#include "storage/diskio.h"

#ifdef __cplusplus
/** Initialize a small FAT12 superfloppy only when sector zero is unsigned.
 *
 * A signed existing volume is never overwritten. Signed non-FAT12 layouts or
 * incompatible geometry are rejected. No USB or NVM dependency is involved.
 * pWork must hold at least Disk.GetSectSize() bytes and remain valid for
 * the duration of the call. This function does not mount the filesystem.
 */
bool Fat12ProvisionIfBlank(DiskIO &Disk, uint8_t *pWork, uint32_t WorkSize);
#endif

#endif // __FAT12_PROVISION_H__
