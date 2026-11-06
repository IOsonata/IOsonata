/**-------------------------------------------------------------------------
@file	fat12_provision_test.cpp
@brief	Generic FAT12 first-use provisioning regression.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "storage/fat12_provision.h"
#include "storage/fatfs.h"

class MockDisk : public DiskIO {
public:
	uint8_t Data[512U * 1664U];
	uint32_t Writes;
	uint32_t Commits;
	bool FailWrites;
	bool FailCommit;

	MockDisk() : Writes(0), Commits(0), FailWrites(false), FailCommit(false)
	{
		memset(Data, 0xFF, sizeof(Data));
	}

	uint16_t GetSectSize(void) override { return 512; }
	uint32_t GetNbSect(void) override { return sizeof(Data) / 512U; }
	uint32_t GetSize(void) override { return sizeof(Data) / 1024U; }
	bool SectRead(uint32_t n, uint8_t *p) override
	{
		if (n >= GetNbSect()) { return false; }
		memcpy(p, Data + n * 512U, 512U);
		return true;
	}
	bool SectWrite(uint32_t n, uint8_t *p) override
	{
		if (n >= GetNbSect() || FailWrites) { return false; }
		memcpy(Data + n * 512U, p, 512U);
		Writes++;
		return true;
	}
	bool Commit(void) override
	{
		Commits++;
		return !FailCommit;
	}
};

int main(void)
{
	static MockDisk disk;
	uint8_t work[512];

	assert(Fat12ProvisionIfBlank(disk, work, sizeof(work)));
	FATFS_BSBPB *boot = reinterpret_cast<FATFS_BSBPB *>(disk.Data);
	assert(boot->BytsPerSec == 512U);
	assert(boot->TotSec16 == disk.GetNbSect());
	assert(boot->Signature_word[0] == 0x55 && boot->Signature_word[1] == 0xAA);
	assert(boot->SecPerClus == 1U);
	assert(boot->NumFATs == 2U);
	assert(memcmp(boot->BPB.Bpb16.FilSysType, "FAT12   ", 8U) == 0);
	uint32_t root = 1U + 2U * boot->FATSz16;
	assert(disk.Data[512U] == 0xF8U && disk.Data[513U] == 0xFFU);
	assert(disk.Data[(1U + boot->FATSz16) * 512U] == 0xF8U);
	assert(memcmp(disk.Data + root * 512U, "IOSONATA   ", 11U) == 0);
	assert(disk.Commits == 1U);
	uint32_t writes = disk.Writes;
	assert(Fat12ProvisionIfBlank(disk, work, sizeof(work)));
	assert(disk.Writes == writes && disk.Commits == 1U);

	// A signed but incompatible volume must remain untouched.
	boot->BytsPerSec = 4096;
	assert(!Fat12ProvisionIfBlank(disk, work, sizeof(work)));
	assert(disk.Writes == writes);
	boot->BytsPerSec = 512;

	// Failed write leaves sector zero unsigned, so the next call retries.
	memset(disk.Data, 0xFF, sizeof(disk.Data));
	disk.FailWrites = true;
	assert(!Fat12ProvisionIfBlank(disk, work, sizeof(work)));
	assert(disk.Data[510] == 0xFF);
	disk.FailWrites = false;
	assert(Fat12ProvisionIfBlank(disk, work, sizeof(work)));

	// Too small a workspace cannot overwrite anything.
	writes = disk.Writes;
	assert(!Fat12ProvisionIfBlank(disk, work, sizeof(work) - 1U));
	assert(disk.Writes == writes);

	printf("PASS: generic FAT12 provisioning, signed preservation and retry\n");
	return 0;
}
