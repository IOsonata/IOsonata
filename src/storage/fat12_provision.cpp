/**-------------------------------------------------------------------------
@file	fat12_provision.cpp

@brief	Small FAT12 superfloppy provisioning on generic DiskIO.

@license
MIT License
Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/
#include <string.h>

#include "storage/fat12_provision.h"
#include "storage/fatfs.h"

bool Fat12ProvisionIfBlank(DiskIO &Disk, uint8_t *pWork, uint32_t WorkSize)
{
	const uint16_t sectSize = Disk.GetSectSize();
	const uint32_t nbSect = Disk.GetNbSect();

	if (pWork == nullptr || sectSize != 512U || WorkSize < sectSize ||
		nbSect == 0U || nbSect > 4084U)
	{
		return false;
	}

	if (!Disk.SectRead(0U, pWork))
	{
		return false;
	}

	if (pWork[510] == 0x55U && pWork[511] == 0xAAU)
	{
		// A partition table or an unrelated signed boot sector is not an
		// invitation to overwrite the medium. Preserve valid FAT12 only.
		const FATFS_BSBPB *pBoot =
			reinterpret_cast<const FATFS_BSBPB *>(pWork);
		return pBoot->BytsPerSec == sectSize &&
			pBoot->TotSec16 == nbSect &&
			pBoot->NumFATs == FATFS_NBFAT &&
			memcmp(pBoot->BPB.Bpb16.FilSysType, "FAT12   ", 8U) == 0;
	}

	// FAT12 has at most 4084 data clusters. The count below is a
	// conservative upper bound before subtracting reserved/FAT/root sectors.
	const uint16_t fatSect = (uint16_t)((((nbSect + 2U) * 3U + 1U) / 2U +
		sectSize - 1U) / sectSize);
	const uint32_t rootSect = FATFS_RSVDSECCNT_FAT16 +
		(uint32_t)FATFS_NBFAT * fatSect;
	if (nbSect <= rootSect + 1U)
	{
		return false;
	}

	for (uint32_t i = 0; i < (uint32_t)FATFS_NBFAT * fatSect; i++)
	{
		memset(pWork, 0, sectSize);
		if (i % fatSect == 0U)
		{
			pWork[0] = FATFS_MEDIA_FIXED;
			pWork[1] = 0xFFU;
			pWork[2] = 0xFFU;
		}
		if (!Disk.SectWrite(FATFS_RSVDSECCNT_FAT16 + i, pWork))
		{
			return false;
		}
	}

	memset(pWork, 0, sectSize);
	memcpy(pWork, "IOSONATA   ", 11U);
	pWork[11] = FATFS_DIRATTR_VOLUME_ID;
	if (!Disk.SectWrite(rootSect, pWork))
	{
		return false;
	}

	FATFS_BSBPB *pBoot = reinterpret_cast<FATFS_BSBPB *>(pWork);
	memset(pWork, 0, sectSize);
	pBoot->JmpBoot[0] = 0xEBU;
	pBoot->JmpBoot[1] = 0x3CU;
	pBoot->JmpBoot[2] = 0x90U;
	memcpy(pBoot->OEMName, "IOsonata", sizeof(pBoot->OEMName));
	pBoot->BytsPerSec = sectSize;
	pBoot->SecPerClus = 1U;
	pBoot->RsvdSecCnt = FATFS_RSVDSECCNT_FAT16;
	pBoot->NumFATs = FATFS_NBFAT;
	pBoot->RootEntCnt = sectSize / 32U;
	pBoot->TotSec16 = (uint16_t)nbSect;
	pBoot->Media = FATFS_MEDIA_FIXED;
	pBoot->FATSz16 = fatSect;
	pBoot->SecPerTrk = 32U;
	pBoot->NumHeads = 1U;
	pBoot->BPB.Bpb16.DrvNum = 0x80U;
	pBoot->BPB.Bpb16.BootSig = 0x29U;
	pBoot->BPB.Bpb16.VolID = 0x4E534F49U;
	memcpy(pBoot->BPB.Bpb16.VolLab, "IOSONATA   ", 11U);
	memcpy(pBoot->BPB.Bpb16.FilSysType, "FAT12   ", 8U);
	pBoot->Signature_word[0] = 0x55U;
	pBoot->Signature_word[1] = 0xAAU;

	// Boot sector last: failed provisioning leaves an unsigned sector zero
	// and can be retried on the next boot.
	if (!Disk.SectWrite(0U, pWork) || !Disk.Commit())
	{
		return false;
	}
	return true;
}
