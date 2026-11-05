// Host-side FatFs and DiskIO regression tests.
// Verify 512-byte SD-style media and 4 KiB erase-before-write flash media.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"
#include "ff_diskio.h"
#include "storage/diskio.h"

#define DISK_BYTES		(4U * 1024U * 1024U)
#define FAT_SECTOR		512U
#define FLASH_SECTOR	4096U

static int s_Checks = 0;
static int s_Fails = 0;

#define CHECK(expr) do { \
	s_Checks++; \
	if (!(expr)) { \
		s_Fails++; \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
	} \
} while (0)

class MemoryDisk : public DiskIO {
public:
	explicit MemoryDisk(uint16_t SectorSize)
		: vSectorSize(SectorSize), vReadFail(false), vWriteFail(false), vWrites(0)
	{
		Blank();
	}

	uint16_t GetSectSize(void) override { return vSectorSize; }
	uint32_t GetSize(void) override { return sizeof(vData) / 1024U; }
	uint32_t GetMinEraseSize(void) override { return vSectorSize; }

	bool SectRead(uint32_t SectNo, uint8_t *pData) override
	{
		if (vReadFail || pData == nullptr || SectNo >= sizeof(vData) / vSectorSize)
		{
			return false;
		}
		memcpy(pData, vData + SectNo * vSectorSize, vSectorSize);
		return true;
	}

	bool SectWrite(uint32_t SectNo, uint8_t *pData) override
	{
		if (vWriteFail || pData == nullptr || SectNo >= sizeof(vData) / vSectorSize)
		{
			return false;
		}
		// Model a physical sector replacement; on NOR the disk driver
		// erases before programming, on SD a sector is overwritten.
		memset(vData + SectNo * vSectorSize, 0xFF, vSectorSize);
		memcpy(vData + SectNo * vSectorSize, pData, vSectorSize);
		vWrites++;
		return true;
	}

	void Blank(void)
	{
		memset(vData, 0xFF, sizeof(vData));
		vWrites = 0;
		vReadFail = false;
		vWriteFail = false;
	}

	uint8_t vData[DISK_BYTES];
	uint16_t vSectorSize;
	bool vReadFail;
	bool vWriteFail;
	uint32_t vWrites;
};

static MemoryDisk s_Flash(FLASH_SECTOR);
static MemoryDisk s_Corrupt(FLASH_SECTOR);
static MemoryDisk s_Direct(FAT_SECTOR);
static FATFS s_Fs;
alignas(4) static uint8_t s_Physical[FLASH_SECTOR];
alignas(4) static uint8_t s_Format[FLASH_SECTOR];
static BYTE s_In[FAT_SECTOR * 2];
static BYTE s_Out[FAT_SECTOR * 2];

static FatFsMountCfg_t FlashCfg(MemoryDisk &Disk)
{
	FatFsMountCfg_t cfg = {};
	cfg.pDisk = &Disk;
	cfg.pFatFs = &s_Fs;
	cfg.pPath = "0:";
	cfg.pSectorBuffer = s_Physical;
	cfg.SectorBufferSize = sizeof(s_Physical);
	return cfg;
}

static void TestAdapter(void)
{
	FatFsMountCfg_t cfg = FlashCfg(s_Flash);

	CHECK(FatFsMount(cfg) == FR_NO_FILESYSTEM);
	CHECK(s_Flash.vWrites == 0);

	WORD sectorSize = 0;
	LBA_t sectorCount = 0;
	DWORD eraseSectors = 0;

	CHECK(disk_status(0) == 0);
	CHECK(disk_status(1) == STA_NOINIT);
	CHECK(disk_ioctl(0, GET_SECTOR_SIZE, &sectorSize) == RES_OK);
	CHECK(sectorSize == FAT_SECTOR);
	CHECK(disk_ioctl(0, GET_SECTOR_COUNT, &sectorCount) == RES_OK);
	CHECK(sectorCount == DISK_BYTES / FAT_SECTOR);
	CHECK(disk_ioctl(0, GET_BLOCK_SIZE, &eraseSectors) == RES_OK);
	CHECK(eraseSectors == FLASH_SECTOR / FAT_SECTOR);
	CHECK(disk_ioctl(0, CTRL_TRIM, &sectorSize) == RES_PARERR);

	for (unsigned i = 0; i < sizeof(s_In); i++)
	{
		s_In[i] = (uint8_t)(i * 7U + 3U);
	}

	CHECK(disk_write(0, s_In, 6, 2) == RES_OK);
	CHECK(disk_read(0, s_Out, 6, 2) == RES_OK);
	CHECK(memcmp(s_In, s_Out, sizeof(s_In)) == 0);

	// Two logical sectors share physical sector 0. A change to sector 7
	// must not erase sector 6 or the next physical sector.
	memset(s_In, 0xA5, FAT_SECTOR);
	CHECK(disk_write(0, s_In, 7, 1) == RES_OK);
	CHECK(disk_read(0, s_Out, 6, 1) == RES_OK);
	for (unsigned i = 0; i < FAT_SECTOR; i++)
	{
		CHECK(s_Out[i] == (uint8_t)(i * 7U + 3U));
	}
	CHECK(disk_read(0, s_Out, 8, 1) == RES_OK);
	for (unsigned i = 0; i < FAT_SECTOR; i++)
	{
		CHECK(s_Out[i] == 0xFF);
	}

	// A single FatFs write crossing the physical erase-sector boundary
	// must preserve the untouched 512-byte sectors on either side.
	for (unsigned i = 0; i < sizeof(s_In); i++)
	{
		s_In[i] = (uint8_t)(i * 11U + 5U);
	}
	CHECK(disk_write(0, s_In, 7, 2) == RES_OK);
	CHECK(disk_read(0, s_Out, 7, 2) == RES_OK);
	CHECK(memcmp(s_In, s_Out, sizeof(s_In)) == 0);
	CHECK(disk_read(0, s_Out, 6, 1) == RES_OK);
	for (unsigned i = 0; i < FAT_SECTOR; i++)
	{
		CHECK(s_Out[i] == (uint8_t)(i * 7U + 3U));
	}
	CHECK(disk_read(0, s_Out, 9, 1) == RES_OK);
	for (unsigned i = 0; i < FAT_SECTOR; i++)
	{
		CHECK(s_Out[i] == 0xFF);
	}

	s_Flash.vReadFail = true;
	CHECK(disk_read(0, s_Out, 3, 1) == RES_ERROR);
	CHECK(disk_write(0, s_In, 3, 1) == RES_ERROR);
	s_Flash.vReadFail = false;
	s_Flash.vWriteFail = true;
	CHECK(disk_write(0, s_In, 3, 1) == RES_ERROR);
	s_Flash.vWriteFail = false;

	CHECK(disk_read(0, s_Out, DISK_BYTES / FAT_SECTOR, 1) == RES_PARERR);
	CHECK(disk_write(0, s_In, DISK_BYTES / FAT_SECTOR - 1, 2) == RES_PARERR);
	CHECK(disk_read(0, s_Out, 0, 0) == RES_PARERR);

	cfg.pSectorBuffer = nullptr;
	cfg.SectorBufferSize = 0;
	CHECK(FatFsMount(cfg) == FR_INVALID_PARAMETER);
}

static void TestFilesystem(void)
{
	s_Flash.Blank();
	FatFsMountCfg_t cfg = FlashCfg(s_Flash);
	cfg.bFormatIfBlank = true;
	cfg.pFormatWork = s_Format;
	cfg.FormatWorkSize = sizeof(s_Format);

	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(s_Flash.vWrites > 0);

	FIL file = {};
	UINT transferred = 0;
	const char data[] = "Firmware logging through generic DiskIO";
	char back[sizeof(data)] = {};

	CHECK(f_open(&file, "0:/record.txt", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK);
	CHECK(f_write(&file, data, sizeof(data), &transferred) == FR_OK);
	CHECK(transferred == sizeof(data));
	CHECK(f_close(&file) == FR_OK);

	CHECK(f_open(&file, "0:/record.txt", FA_READ) == FR_OK);
	transferred = 0;
	CHECK(f_read(&file, back, sizeof(back), &transferred) == FR_OK);
	CHECK(transferred == sizeof(back));
	CHECK(memcmp(data, back, sizeof(data)) == 0);
	CHECK(f_close(&file) == FR_OK);

	CHECK(FatFsUnmount() == FR_OK);
	cfg.bFormatIfBlank = false;
	uint32_t writes = s_Flash.vWrites;
	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(s_Flash.vWrites == writes);
	CHECK(FatFsUnmount() == FR_OK);
}

static void TestNoDestructiveFormat(void)
{
	FatFsMountCfg_t cfg = FlashCfg(s_Corrupt);
	cfg.bFormatIfBlank = true;
	cfg.pFormatWork = s_Format;
	cfg.FormatWorkSize = sizeof(s_Format);

	// The boot sector is erased but data elsewhere is not. This is not
	// an empty volume and must never be formatted automatically.
	s_Corrupt.Blank();
	s_Corrupt.vData[FLASH_SECTOR * 2] = 0x32;
	CHECK(FatFsMount(cfg) == FR_NO_FILESYSTEM);
	CHECK(s_Corrupt.vWrites == 0);

	s_Corrupt.Blank();
	s_Corrupt.vReadFail = true;
	CHECK(FatFsMount(cfg) == FR_DISK_ERR);
	CHECK(s_Corrupt.vWrites == 0);

	cfg.pFormatWork = s_Physical;
	CHECK(FatFsMount(cfg) == FR_INVALID_PARAMETER);
	CHECK(FatFsUnmount() == FR_OK);
}

static void TestDirectMedia(void)
{
	FatFsMountCfg_t cfg = {};
	cfg.pDisk = &s_Direct;
	cfg.pFatFs = &s_Fs;
	cfg.pPath = "0:";
	cfg.bFormatIfBlank = true;
	cfg.pFormatWork = s_Format;
	cfg.FormatWorkSize = sizeof(s_Format);

	CHECK(FatFsMount(cfg) == FR_OK);
	CHECK(s_Direct.vWrites > 0);
	DWORD blockSize = 0;
	CHECK(disk_ioctl(0, GET_BLOCK_SIZE, &blockSize) == RES_OK);
	CHECK(blockSize == 1);
	CHECK(FatFsUnmount() == FR_OK);

	CHECK(disk_ioctl(0, SET_DEVICE, &s_Direct) == RES_OK);
	CHECK(disk_initialize(0) == 0);
	CHECK(disk_ioctl(0, SET_DEVICE, &s_Flash) == RES_PARERR);
	FFDiskSetDevice(nullptr);
	CHECK(disk_status(0) == STA_NOINIT);
}

int main(void)
{
	TestAdapter();
	TestFilesystem();
	TestNoDestructiveFormat();
	TestDirectMedia();

	printf("FatFs DiskIO: %d checks, %d failures\n", s_Checks, s_Fails);
	return s_Fails == 0 ? 0 : 1;
}
