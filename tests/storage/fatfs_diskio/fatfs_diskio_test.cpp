// FatFs on generic DiskIO. Host tests, independent of USB MSC.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "ff_diskio.h"
#include "storage/diskio.h"

#define MEDIA_SIZE	(4U * 1024U * 1024U)

static unsigned s_Checks;
static unsigned s_Fails;

#define CHECK(cond) do { \
	s_Checks++; \
	if (!(cond)) { \
		s_Fails++; \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
	} \
} while (0)

class TestDisk : public DiskIO {
public:
	explicit TestDisk(uint16_t Size, uint32_t VolumeSize = MEDIA_SIZE)
		: vSectorSize(Size), vVolumeSize(VolumeSize),
		  vReadFail(false), vWriteFail(false), vWrites(0)
	{
		Blank();
	}

	uint16_t GetSectSize(void) override { return vSectorSize; }
	uint32_t GetNbSect(void) override { return vVolumeSize / vSectorSize; }
	uint32_t GetSize(void) override { return vVolumeSize / 1024U; }
	uint32_t GetMinEraseSize(void) override { return vSectorSize; }

	bool SectRead(uint32_t SectNo, uint8_t *pData) override
	{
		if (vReadFail || pData == nullptr || SectNo >= GetNbSect())
		{
			return false;
		}
		memcpy(pData, vMedia + SectNo * vSectorSize, vSectorSize);
		return true;
	}

	bool SectWrite(uint32_t SectNo, uint8_t *pData) override
	{
		if (vWriteFail || pData == nullptr || SectNo >= GetNbSect())
		{
			return false;
		}
		// A whole physical sector replacement. For NVM this includes
		// erase/program and completion inside NvmDiskIO::SectWrite.
		memcpy(vMedia + SectNo * vSectorSize, pData, vSectorSize);
		vWrites++;
		return true;
	}

	void Blank(void)
	{
		memset(vMedia, 0xFF, sizeof(vMedia));
		vReadFail = false;
		vWriteFail = false;
		vWrites = 0;
	}

	alignas(4) uint8_t vMedia[MEDIA_SIZE];
	uint16_t vSectorSize;
	uint32_t vVolumeSize;
	bool vReadFail;
	bool vWriteFail;
	uint32_t vWrites;
};

static TestDisk s_Flash(4096);
static TestDisk s_Direct(512);
static TestDisk s_Small(512, 256U * 1024U);
static TestDisk s_Expanded(512, 832U * 1024U);
static FATFS s_Fs;
alignas(4) static uint8_t s_Work[4096];
alignas(4) static uint8_t s_Data[4096];
alignas(4) static uint8_t s_Readback[4096];

static void TestMedium(TestDisk &Disk)
{
	FatFsMountCfg_t cfg = {};
	cfg.pDisk = &Disk;
	cfg.pFatFs = &s_Fs;
	cfg.pPath = "0:";

	CHECK(FatFsMount(cfg) == FR_NO_FILESYSTEM);
	CHECK(Disk.vWrites == 0U);

	LBA_t count = 0;
	WORD sectorSize = 0;
	DWORD eraseBlock = 0;

	CHECK(disk_status(0) == 0U);
	CHECK(disk_status(1) == STA_NOINIT);
	CHECK(disk_ioctl(0, GET_SECTOR_COUNT, &count) == RES_OK);
	CHECK(count == Disk.GetNbSect());
	CHECK(disk_ioctl(0, GET_SECTOR_SIZE, &sectorSize) == RES_OK);
	CHECK(sectorSize == Disk.GetSectSize());
	CHECK(disk_ioctl(0, GET_BLOCK_SIZE, &eraseBlock) == RES_OK);
	CHECK(eraseBlock == 1U);
	CHECK(disk_ioctl(0, CTRL_TRIM, nullptr) == RES_PARERR);

	cfg.bFormatIfBlank = true;
	cfg.pWork = s_Work;
	cfg.WorkSize = sizeof(s_Work);

	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(Disk.vWrites != 0U);

	// Normal firmware file operations, with no USB initialization.
	const char data[] = "IOsonata FatFs over DiskIO";
	FIL file = {};
	UINT nb = 0;
	char back[sizeof(data)] = {};

	CHECK(f_open(&file, "0:/logger.txt", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK);
	CHECK(f_write(&file, data, sizeof(data), &nb) == FR_OK);
	CHECK(nb == sizeof(data));
	CHECK(f_close(&file) == FR_OK);
	CHECK(f_open(&file, "0:/logger.txt", FA_READ) == FR_OK);
	nb = 0;
	CHECK(f_read(&file, back, sizeof(back), &nb) == FR_OK);
	CHECK(nb == sizeof(back));
	CHECK(memcmp(data, back, sizeof(data)) == 0);
	CHECK(f_close(&file) == FR_OK);

	CHECK(FatFsUnmount() == FR_OK);
	cfg.bFormatIfBlank = false;
	uint32_t writes = Disk.vWrites;
	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(Disk.vWrites == writes);
	CHECK(FatFsUnmount() == FR_OK);

	// The original blank volume had no directory label, so computers
	// displayed "NO NAME". A later mount can name it without formatting.
	cfg.pLabelIfUnnamed = "0:IOSONATA";
	TCHAR label[FF_MAX_LFN + 1] = {};
	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(f_getlabel("0:", label, nullptr) == FR_OK);
	CHECK(strcmp(label, "IOSONATA") == 0);
	CHECK(f_open(&file, "0:/logger.txt", FA_READ) == FR_OK);
	CHECK(f_close(&file) == FR_OK);
	CHECK(FatFsUnmount() == FR_OK);

	// An existing label is never replaced, even if the configuration
	// subsequently requests another label.
	writes = Disk.vWrites;
	cfg.pLabelIfUnnamed = "0:OTHER";
	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(f_getlabel("0:", label, nullptr) == FR_OK);
	CHECK(strcmp(label, "IOSONATA") == 0);
	CHECK(Disk.vWrites == writes);
	CHECK(f_setlabel("0:CUSTOM") == FR_OK);
	CHECK(FatFsUnmount() == FR_OK);

	writes = Disk.vWrites;
	cfg.pLabelIfUnnamed = "0:IOSONATA";
	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(f_getlabel("0:", label, nullptr) == FR_OK);
	CHECK(strcmp(label, "CUSTOM") == 0);
	CHECK(Disk.vWrites == writes);
	CHECK(FatFsUnmount() == FR_OK);

	// Errors from the physical sector driver must not be reported as OK.
	FFDiskSetDevice(&Disk);
	for (unsigned i = 0; i < sizeof(s_Data); i++)
	{
		s_Data[i] = (uint8_t)(i * 3U + 11U);
	}
	Disk.vReadFail = true;
	CHECK(disk_read(0, s_Readback, 0, 1) == RES_ERROR);
	Disk.vReadFail = false;
	Disk.vWriteFail = true;
	CHECK(disk_write(0, s_Data, 0, 1) == RES_ERROR);
	Disk.vWriteFail = false;
	CHECK(disk_read(0, s_Readback, Disk.GetNbSect(), 1) == RES_PARERR);
	CHECK(disk_write(0, s_Data, Disk.GetNbSect() - 1, 2) == RES_PARERR);
	CHECK(disk_read(0, s_Readback, 0, 0) == RES_PARERR);
	FFDiskSetDevice(nullptr);
	CHECK(disk_status(0) == STA_NOINIT);
}

static void TestExpandedDiskWithLegacyData(void)
{
	// New Debug NVM1 begins at 0x30000; the old 256 KiB NVM1 began at
	// 0xC0000. The old filesystem is 0x90000 bytes into the new disk.
	// Reading sector zero cannot identify the old volume, but the full
	// blank scan must detect it and refuse to overwrite existing files.
	s_Expanded.Blank();
	s_Expanded.vMedia[0x90000U + 510U] = 0x55U;
	s_Expanded.vMedia[0x90000U + 511U] = 0xAAU;
	s_Expanded.vMedia[0x90000U + 54U] = 'F';

	FatFsMountCfg_t cfg = {};
	cfg.pDisk = &s_Expanded;
	cfg.pFatFs = &s_Fs;
	cfg.pPath = "0:";
	cfg.bFormatIfBlank = true;
	cfg.pWork = s_Work;
	cfg.WorkSize = sizeof(s_Work);
	cfg.pLabelIfUnnamed = "0:IOSONATA";
	MKFS_PARM format = { FM_FAT | FM_SFD, 2, 8, 0, 0 };
	cfg.pFormatOptions = &format;

	CHECK(FatFsMount(cfg) == FR_NO_FILESYSTEM);
	CHECK(s_Expanded.vWrites == 0U);
	CHECK(s_Expanded.vMedia[0x90000U + 510U] == 0x55U);
	CHECK(s_Expanded.vMedia[0x90000U + 511U] == 0xAAU);
	CHECK(FatFsUnmount() == FR_OK);

	// The reusable mount is non-destructive. This USB test application
	// explicitly erases its disposable NVM1 region after this result.
	// Simulate that erase, then use the same FatFsMount again.
	s_Expanded.Blank();
	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(s_Expanded.vWrites > 0U);
	CHECK(s_Expanded.vMedia[11] == 0x00U); // 512-byte sector in BPB
	CHECK(s_Expanded.vMedia[12] == 0x02U);
	CHECK(s_Expanded.vMedia[16] == 2U);    // Two FAT copies
	CHECK(s_Expanded.vMedia[510] == 0x55U);
	CHECK(s_Expanded.vMedia[511] == 0xAAU);

	TCHAR label[FF_MAX_LFN + 1] = {};
	CHECK(f_getlabel("0:", label, nullptr) == FR_OK);
	CHECK(strcmp(label, "IOSONATA") == 0);

	const char contents[] = "test flash volume persists";
	char readback[sizeof(contents)] = {};
	UINT transferred = 0;
	FIL file = {};

	CHECK(f_open(&file, "0:/record.txt", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK);
	CHECK(f_write(&file, contents, sizeof(contents), &transferred) == FR_OK);
	CHECK(transferred == sizeof(contents));
	CHECK(f_close(&file) == FR_OK);
	CHECK(FatFsUnmount() == FR_OK);

	// A second boot must mount the same volume, without another format.
	uint32_t writes = s_Expanded.vWrites;
	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(s_Expanded.vWrites == writes);
	CHECK(f_getlabel("0:", label, nullptr) == FR_OK);
	CHECK(strcmp(label, "IOSONATA") == 0);
	CHECK(f_open(&file, "0:/record.txt", FA_READ) == FR_OK);
	transferred = 0;
	CHECK(f_read(&file, readback, sizeof(readback), &transferred) == FR_OK);
	CHECK(transferred == sizeof(readback));
	CHECK(memcmp(contents, readback, sizeof(contents)) == 0);
	CHECK(f_close(&file) == FR_OK);
	CHECK(FatFsUnmount() == FR_OK);
}

static void TestBlankOnly(void)
{
	s_Flash.Blank();
	s_Flash.vMedia[2U * 4096U] = 0x12U;

	FatFsMountCfg_t cfg = {};
	cfg.pDisk = &s_Flash;
	cfg.pFatFs = &s_Fs;
	cfg.bFormatIfBlank = true;
	cfg.pWork = s_Work;
	cfg.WorkSize = sizeof(s_Work);

	// Sector zero is erased but another sector contains data.
	CHECK(FatFsMount(cfg) == FR_NO_FILESYSTEM);
	CHECK(s_Flash.vWrites == 0U);
	CHECK(FatFsUnmount() == FR_OK);

	s_Flash.Blank();
	s_Flash.vReadFail = true;
	CHECK(FatFsMount(cfg) == FR_DISK_ERR);
	CHECK(s_Flash.vWrites == 0U);
	CHECK(FatFsUnmount() == FR_OK);

	s_Flash.vReadFail = false;
	cfg.WorkSize = 512U;
	CHECK(FatFsMount(cfg) == FR_INVALID_PARAMETER);
	CHECK(s_Flash.vWrites == 0U);
}

int main(void)
{
	TestMedium(s_Flash);
	TestMedium(s_Direct);
	// Regression: 256 KiB must have 512 logical sectors to meet the
	// FatFs mkfs minimum of 128 sectors.
	TestMedium(s_Small);
	TestBlankOnly();
	TestExpandedDiskWithLegacyData();
	printf("FatFs DiskIO: %u checks, %u failures\n", s_Checks, s_Fails);
	return s_Fails == 0U ? 0 : 1;
}
