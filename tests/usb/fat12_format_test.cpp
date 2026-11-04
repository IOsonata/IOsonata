/**-------------------------------------------------------------------------
@file	fat12_format_test.cpp

@brief	Host tests for blank volume formatting and interrupted initialization.
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "storage/diskio.h"
#include "storage/fatfs_format.h"

// Large enough for the biggest test case, retained in static host test RAM.
#define TEST_STORAGE_SIZE (512U * 4200U)
static uint8_t s_Storage[TEST_STORAGE_SIZE];
static uint8_t s_Sector[4096];
static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; \
} } while (0)

class TestDisk : public DiskIO {
public:
	TestDisk(uint16_t SectorSize, uint32_t Count)
	{
		vSectorSize = SectorSize;
		vCount = Count;
		vWrites = 0;
		vFailSector = -1;
		vbPartialBoot = false;
		memset(s_Storage, 0xFF, (size_t)SectorSize * Count);
	}
	uint16_t GetSectSize(void) override { return vSectorSize; }
	uint32_t GetNbSect(void) override { return vCount; }
	uint32_t GetSize(void) override { return (vSectorSize * vCount) / 1024U; }
	bool SectRead(uint32_t SectNo, uint8_t *pBuff) override
	{
		if (SectNo >= vCount || pBuff == nullptr)
		{
			return false;
		}
		memcpy(pBuff, &s_Storage[SectNo * vSectorSize], vSectorSize);
		return true;
	}
	bool SectWrite(uint32_t SectNo, uint8_t *pData) override
	{
		if (SectNo >= vCount || pData == nullptr)
		{
			return false;
		}
		vWrites++;
		if ((int)SectNo == vFailSector)
		{
			if (vbPartialBoot && SectNo == 0U)
			{
				memcpy(s_Storage, pData, 64U);
			}
			return false;
		}
		memcpy(&s_Storage[SectNo * vSectorSize], pData, vSectorSize);
		return true;
	}

	uint16_t vSectorSize;
	uint32_t vCount;
	int vWrites;
	int vFailSector;
	bool vbPartialBoot;
};

static uint16_t Le16(const uint8_t *pData)
{
	return (uint16_t)pData[0] | ((uint16_t)pData[1] << 8);
}

static void TestBlankFAT12(uint16_t SectorSize, uint32_t Count)
{
	TestDisk disk(SectorSize, Count);
	uint32_t fatSect;
	uint32_t clusterCount;
	int writes;

	CHECK(FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(s_Storage[510] == 0x55U && s_Storage[511] == 0xAAU);
	CHECK(Le16(&s_Storage[11]) == SectorSize);
	CHECK(s_Storage[13] == 1U);
	CHECK(s_Storage[16] == 2U);
	CHECK(Le16(&s_Storage[19]) == Count);
	CHECK(memcmp(&s_Storage[54], "FAT12   ", 8U) == 0);
	fatSect = Le16(&s_Storage[22]);
	clusterCount = Count - 2U - 2U * fatSect;
	CHECK(clusterCount > 0U && clusterCount < 4085U);
	CHECK(fatSect * SectorSize >= ((clusterCount + 2U) * 3U + 1U) / 2U);
	CHECK(s_Storage[SectorSize] == 0xF8U);
	CHECK(s_Storage[(1U + fatSect) * SectorSize] == 0xF8U);
	CHECK(memcmp(&s_Storage[(1U + 2U * fatSect) * SectorSize],
		"IOSONATA   ", 11U) == 0);
	CHECK(s_Storage[(1U + 2U * fatSect) * SectorSize + 11U] == 8U);
	writes = disk.vWrites;
	CHECK(FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(disk.vWrites == writes);
}

static void TestPowerLoss(void)
{
	TestDisk disk(4096U, 208U);

	// Formatting interrupted during one FAT sector: boot stays erased.
	disk.vFailSector = 2;
	CHECK(!FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(s_Storage[0] == 0xFFU);
	disk.vFailSector = -1;
	CHECK(FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(s_Storage[510] == 0x55U && s_Storage[511] == 0xAAU);
}

static void TestPartialBootRepair(void)
{
	TestDisk disk(4096U, 64U);
	uint32_t rootSector;
	uint8_t previous;

	disk.vFailSector = 0;
	disk.vbPartialBoot = true;
	CHECK(!FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(s_Storage[510] == 0xFFU);
	rootSector = 1U + 2U * 1U;
	previous = s_Storage[rootSector * 4096U + 20U];
	disk.vFailSector = -1;
	CHECK(FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(s_Storage[510] == 0x55U && s_Storage[511] == 0xAAU);
	CHECK(s_Storage[rootSector * 4096U + 20U] == previous);
}

static void TestPreservation(void)
{
	TestDisk disk(4096U, 64U);

	// Preserve a host-created boot sector with another layout, and an MBR.
	s_Storage[510] = 0x55U;
	s_Storage[511] = 0xAAU;
	s_Storage[54] = 'X';
	CHECK(FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(disk.vWrites == 0);
	CHECK(s_Storage[54] == 'X');

	// Nonblank unknown contents with no signature must not be reformatted.
	s_Storage[510] = 0xFFU;
	s_Storage[511] = 0xFFU;
	s_Storage[3] = 'X';
	CHECK(!FatFsFormatIfBlank(disk, s_Sector, sizeof(s_Sector)));
	CHECK(disk.vWrites == 0);
}

static void TestGeometry(void)
{
	TestDisk small(4096U, 4U);
	CHECK(!FatFsFormatIfBlank(small, s_Sector, sizeof(s_Sector)));
	CHECK(small.vWrites == 0);

	TestDisk unsupported(256U, 64U);
	CHECK(!FatFsFormatIfBlank(unsupported, s_Sector, sizeof(s_Sector)));
	CHECK(unsupported.vWrites == 0);

	TestDisk large(512U, 4200U);
	CHECK(!FatFsFormatIfBlank(large, s_Sector, sizeof(s_Sector)));
	CHECK(large.vWrites == 0);

	TestDisk valid(4096U, 64U);
	CHECK(!FatFsFormatIfBlank(valid, s_Sector, 512U));
	CHECK(!FatFsFormatIfBlank(valid, nullptr, 4096U));
	CHECK(valid.vWrites == 0);
}

int main(void)
{
	TestBlankFAT12(512U, 5U);
	TestBlankFAT12(512U, 344U);
	TestBlankFAT12(512U, 345U);
	TestBlankFAT12(512U, 4090U);
	TestBlankFAT12(1024U, 64U);
	TestBlankFAT12(2048U, 64U);
	TestBlankFAT12(4096U, 64U);
	TestBlankFAT12(4096U, 208U);
	TestPowerLoss();
	TestPartialBootRepair();
	TestPreservation();
	TestGeometry();

	if (s_Fail != 0)
	{
		printf("%d failed\n", s_Fail);
		return 1;
	}
	printf("fat12 format tests pass\n");
	return 0;
}
