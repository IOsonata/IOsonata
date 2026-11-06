/**-------------------------------------------------------------------------
@file	nvm_mcu_disk.h

@brief	Bind a linker-reserved MCU NVM region to a generic block disk.

This storage helper has no USB or filesystem dependency. The linker owns the
bounds. Objects are caller-owned and must outlive any disk operations.

@author	Hoang Nguyen Hoan
@date	Nov. 6, 2026

@license
MIT License
Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/
#ifndef __NVM_MCU_DISK_H__
#define __NVM_MCU_DISK_H__

#include "storage/diskio_nvm.h"
#include "storage/nvm_mcu_intrf.h"

#ifdef __cplusplus

/**
 * @brief	Initialize a block disk from a linker-reserved MCU NVM region.
 *
 * Does not erase, format, mount or inspect the volume.
 * @param	Disk	Caller-owned block adapter.
 * @param	Mem	Caller-owned NVM driver.
 * @param	Intrf	Caller-owned MCU memory interface.
 * @param	RegionNo	Existing linker-reserved NVM region number.
 * @return	true when the block disk is initialized.
 */
inline bool NvmMcuDiskInit(NvmDiskIO &Disk, Nvm &Mem,
					NvmMcuIntrf &Intrf, int RegionNo,
					uint32_t SectSize = 0, uint8_t *pEraseWork = nullptr,
					uint32_t WorkSize = 0)
{
	NvmCfg_t cfg = {};

	if (!NvmMcuCfgInit(cfg, RegionNo))
	{
		return false;
	}

	if (!Intrf.Init())
	{
		return false;
	}

	if (!Mem.Init(cfg, &Intrf))
	{
		return false;
	}

	return Disk.Init(Mem, SectSize, pEraseWork, WorkSize);
}

#endif // __cplusplus
#endif // __NVM_MCU_DISK_H__
