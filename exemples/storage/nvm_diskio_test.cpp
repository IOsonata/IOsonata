/**-------------------------------------------------------------------------
@example	nvm_diskio_test.cpp

@brief	Host test for NvmDiskIO, the block device over any Nvm.

		The point of the adapter is that a filesystem cannot tell what it is
		sitting on. This test runs the same block sequence over two media that
		behave nothing alike underneath: one with an erase step that programs by
		clearing bits, and one that overwrites directly and has no erase at all.
		Both must answer identically through the block API, including the
		buffered Read and Write that a filesystem actually uses.

Build and run on the host:

  g++ -std=gnu++23 -O1 -I include -I include/storage -I Linux/include \
	  -I tests/dfu/hostport \
	  exemples/storage/nvm_diskio_test.cpp src/storage/diskio_nvm.cpp \
	  src/storage/diskio_impl.cpp src/storage/nvm.cpp \
	  src/device.cpp src/device_intrf.cpp \
	  -o nvm_diskio_test
  ./nvm_diskio_test

@author	Hoang Nguyen Hoan
@date	July 23, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "storage/diskio_nvm.h"
#include "coredev/spi.h"

#define DISK_SIZE		(64u * 1024u)
#define DISK_SECT		4096u

static int s_Fail = 0;
static int s_Checks = 0;

#define CHECK(cond, ...) do { \
	s_Checks++; \
	if (!(cond)) { \
		s_Fail++; \
		printf("FAIL %s:%d: ", __func__, __LINE__); \
		printf(__VA_ARGS__); \
		printf("\n"); \
	} \
} while (0)

// ---------------------------------------------------------------------------
// A medium with an erase step, programming by clearing bits, like NOR flash.
// ---------------------------------------------------------------------------
class EraseNvm : public Nvm {
public:
	uint8_t vBuf[DISK_SIZE];
	uint32_t vEraseCalls = 0;
	uint32_t vWriteCalls = 0;
	bool vbFailErase = false;

	EraseNvm() { memset(vBuf, 0xFF, sizeof(vBuf)); Region(0, sizeof(vBuf)); }

	uint32_t EraseSize(void) const override { return DISK_SECT; }
	uint32_t WriteGran(void) const override { return 1; }

	int Read(uint64_t Off, void *pBuf, uint32_t Len) override
	{
		if (!RangeValid(Off, Len)) { return -EINVAL; }
		memcpy(pBuf, vBuf + Off, Len);
		return (int)Len;
	}

	int Write(uint64_t Off, const void *pData, uint32_t Len) override
	{
		if (!RangeValid(Off, Len)) { return -EINVAL; }
		vWriteCalls++;
		const uint8_t *p = (const uint8_t *)pData;
		for (uint32_t i = 0; i < Len; i++)
		{
			vBuf[Off + i] &= p[i];		// clear only, like flash
		}
		return (int)Len;
	}

	int Erase(uint64_t Off, uint32_t Len) override
	{
		if (!RangeValid(Off, Len)) { return -EINVAL; }
		if ((Off % DISK_SECT) != 0 || (Len % DISK_SECT) != 0) { return -EINVAL; }
		if (vbFailErase) { return -EIO; }
		vEraseCalls++;
		memset(vBuf + Off, 0xFF, Len);
		return 0;
	}
};

// ---------------------------------------------------------------------------
// A medium that overwrites directly and has no erase, like an EEPROM.
// ---------------------------------------------------------------------------
class DirectNvm : public Nvm {
public:
	uint8_t vBuf[DISK_SIZE];

	DirectNvm() { memset(vBuf, 0xFF, sizeof(vBuf)); Region(0, sizeof(vBuf)); }

	uint32_t EraseSize(void) const override { return 0; }	// no erase step
	uint32_t WriteGran(void) const override { return 1; }

	int Read(uint64_t Off, void *pBuf, uint32_t Len) override
	{
		if (!RangeValid(Off, Len)) { return -EINVAL; }
		memcpy(pBuf, vBuf + Off, Len);
		return (int)Len;
	}

	int Write(uint64_t Off, const void *pData, uint32_t Len) override
	{
		if (!RangeValid(Off, Len)) { return -EINVAL; }
		memcpy(vBuf + Off, pData, Len);		// straight overwrite
		return (int)Len;
	}
};

static EraseNvm s_Erase;
static DirectNvm s_Direct;

static uint8_t s_CacheMem1[DISK_SECT];
static uint8_t s_CacheMem2[DISK_SECT];
static DiskIOCache_t s_Cache1 = { 0, 0xFFFFFFFF, s_CacheMem1 };
static DiskIOCache_t s_Cache2 = { 0, 0xFFFFFFFF, s_CacheMem2 };

// The pin driver is per architecture; the driver only toggles a protect pin.
extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
extern "C" void IOPinSet(int, int) {}
extern "C" void IOPinClear(int, int) {}

// The in-memory media below never use QSPI. Satisfy the generic Nvm
// driver's target hooks and reject any accidental hardware command.
extern "C" void QuadSPISetMemSize(SPIDev_t * const, uint32_t) {}
extern "C" bool QuadSPISendCmd(SPIDev_t * const, uint8_t, uint32_t,
				   uint8_t, uint32_t, uint8_t) { return false; }

// ---------------------------------------------------------------------------
// The same sequence, run against whatever disk it is handed.
// ---------------------------------------------------------------------------
static void RunDisk(DiskIO &Disk, const char *pWhat)
{
	printf("--- %s\n", pWhat);

	CHECK(Disk.GetSectSize() == DISK_SECT, "%s: sector size", pWhat);
	CHECK(Disk.GetSize() == DISK_SIZE / 1024, "%s: size in KBytes", pWhat);
	CHECK(Disk.GetNbSect() == DISK_SIZE / DISK_SECT, "%s: sector count", pWhat);

	uint8_t out[DISK_SECT], in[DISK_SECT];

	// Write a sector and read it back.
	for (uint32_t i = 0; i < DISK_SECT; i++) { out[i] = (uint8_t)(i * 7 + 1); }
	memset(in, 0, sizeof(in));

	CHECK(Disk.SectWrite(1, out), "%s: write sector 1", pWhat);
	CHECK(Disk.SectRead(1, in), "%s: read sector 1", pWhat);
	CHECK(memcmp(out, in, DISK_SECT) == 0, "%s: sector round trip", pWhat);

	// Replace it with different data. On an erase-write medium this only works
	// because the adapter erases first; on a direct medium it just overwrites.
	for (uint32_t i = 0; i < DISK_SECT; i++) { out[i] = (uint8_t)(i * 3 + 200); }
	memset(in, 0, sizeof(in));

	CHECK(Disk.SectWrite(1, out), "%s: rewrite sector 1", pWhat);
	CHECK(Disk.SectRead(1, in), "%s: read it back", pWhat);
	CHECK(memcmp(out, in, DISK_SECT) == 0, "%s: rewrite replaced the sector", pWhat);

	// A neighbouring sector must be untouched.
	memset(in, 0, sizeof(in));
	CHECK(Disk.SectRead(2, in), "%s: read sector 2", pWhat);
	bool untouched = true;
	for (uint32_t i = 0; i < DISK_SECT; i++)
	{
		if (in[i] != 0xFF) { untouched = false; break; }
	}
	CHECK(untouched, "%s: neighbouring sector untouched", pWhat);

	// The buffered access a filesystem actually uses, crossing a sector edge.
	uint8_t small[64], back[64];
	for (int i = 0; i < 64; i++) { small[i] = (uint8_t)(i + 11); }
	memset(back, 0, sizeof(back));

	uint64_t cross = (uint64_t)DISK_SECT * 2 - 32;
	CHECK(Disk.Write(cross, small, sizeof(small)) == (int)sizeof(small),
		  "%s: buffered write across a sector edge", pWhat);
	Disk.Flush();
	CHECK(Disk.Read(cross, back, sizeof(back)) == (int)sizeof(back),
		  "%s: buffered read", pWhat);
	CHECK(memcmp(small, back, sizeof(small)) == 0,
		  "%s: buffered round trip across the edge", pWhat);
}

static void TestSubEraseLogicalSectors(void)
{
	static uint8_t s_EraseWork[DISK_SECT];
	NvmDiskIO disk;
	uint8_t data[512];
	uint8_t back[512];

	CHECK(!disk.Init(s_Erase, 512), "sub-erase without scratch rejected");
	CHECK(!disk.Init(s_Erase, 512, s_EraseWork, 512),
		  "insufficient scratch rejected");
	CHECK(disk.Init(s_Erase, 512, s_EraseWork, sizeof(s_EraseWork)),
		  "512-byte sectors over 4K erase memory");
	CHECK(disk.GetSectSize() == 512, "512-byte logical sectors");
	CHECK(disk.GetMinEraseSize() == DISK_SECT, "physical 4K erase");
	CHECK(disk.GetNbSect() == DISK_SIZE / 512, "logical sector count");

	// Discard earlier test data, then write two logical sectors in the
	// same erase page; the second write must preserve the first.
	CHECK(s_Erase.Erase(0, DISK_SIZE) == 0, "erase mock flash");
	for (unsigned i = 0; i < sizeof(data); i++)
	{
		data[i] = (uint8_t)(i + 17);
	}

	CHECK(!disk.SectWrite(6, s_EraseWork),
		  "scratch cannot also be the input buffer");
	CHECK(disk.SectWrite(6, data), "write logical sector 6");
	memset(data, 0xA5, sizeof(data));
	CHECK(disk.SectWrite(7, data), "write logical sector 7");
	uint32_t erases = s_Erase.vEraseCalls;
	uint32_t writes = s_Erase.vWriteCalls;
	CHECK(disk.SectWrite(7, data), "repeat identical logical sector");
	CHECK(s_Erase.vEraseCalls == erases, "unchanged sector does not erase");
	CHECK(s_Erase.vWriteCalls == writes, "unchanged sector does not program");
	memset(back, 0, sizeof(back));
	CHECK(disk.SectRead(6, back), "read previous logical sector");
	for (unsigned i = 0; i < sizeof(back); i++)
	{
		CHECK(back[i] == (uint8_t)(i + 17), "sector 6 preserved");
	}
	CHECK(disk.SectRead(7, back), "read current logical sector");
	CHECK(memcmp(back, data, sizeof(back)) == 0, "sector 7 updated");

	// A 512-byte TRIM cannot erase a whole page containing other sectors.
	disk.EraseSector(7, 1);
	CHECK(disk.SectRead(7, back), "read after partial erase request");
	CHECK(memcmp(back, data, sizeof(back)) == 0, "partial erase refused");

	// Crossing from physical erase page 0 to 1 is safe.
	memset(data, 0x5A, sizeof(data));
	CHECK(disk.SectWrite(8, data), "write next physical erase page");
	CHECK(disk.SectRead(7, back), "read previous physical erase page");
	CHECK(back[0] == 0xA5, "previous physical page preserved");
}

static void TestWriteBackErasePage(void)
{
	static uint8_t page[DISK_SECT];
	NvmDiskIO disk;
	uint8_t data[512];
	uint8_t back[512];
	uint64_t pageOff = 2U * DISK_SECT;

	CHECK(s_Erase.Erase(0, DISK_SIZE) == 0, "clear mock for cached test");
	CHECK(disk.Init(s_Erase, 512, page, sizeof(page), true),
		"initialize opt-in one-page cache");
	uint32_t erases = s_Erase.vEraseCalls;
	uint32_t writes = s_Erase.vWriteCalls;

	for (int i = 0; i < 8; i++)
	{
		memset(data, (uint8_t)(i + 21), sizeof(data));
		CHECK(disk.SectWrite(16U + i, data), "cache logical sector");
	}
	CHECK(s_Erase.vEraseCalls == erases, "no erase before commit");
	CHECK(s_Erase.vWriteCalls == writes, "no flash write before commit");
	CHECK(disk.SectRead(17U, back), "read dirty sector from RAM cache");
	CHECK(back[0] == 22, "cached read sees pending write");
	CHECK(s_Erase.vBuf[pageOff + 512U] == 0xFF,
		"underlying flash unchanged before commit");

	CHECK(disk.Commit(), "commit cached erase page");
	CHECK(s_Erase.vEraseCalls == erases + 1U,
		"eight logical sectors require one physical erase");
	CHECK(s_Erase.vWriteCalls == writes + 1U,
		"eight logical sectors require one physical write");
	CHECK(s_Erase.vBuf[pageOff + 512U] == 22,
		"committed contents persisted to backing store");
	CHECK(disk.Commit(), "repeated commit succeeds");
	CHECK(s_Erase.vEraseCalls == erases + 1U,
		"clean commit does not erase");

	memset(data, 0x99, sizeof(data));
	CHECK(disk.SectWrite(16U, data), "cache updated sector");
	CHECK(disk.SectWrite(24U, data), "switch to next erase page");
	CHECK(s_Erase.vEraseCalls == erases + 2U,
		"page eviction commits dirty previous page");
	CHECK(s_Erase.vBuf[pageOff] == 0x99,
		"eviction retains changed sector");
	CHECK(disk.Commit(), "commit second erase page");
	CHECK(s_Erase.vEraseCalls == erases + 3U,
		"commit newly loaded dirty page");

	memset(data, 0xB6, sizeof(data));
	CHECK(disk.SectWrite(24U, data), "cache write for failure test");
	s_Erase.vbFailErase = true;
	CHECK(!disk.Commit(), "failed erase is reported by Commit");
	CHECK(!disk.SectWrite(32U, data),
		"failed eviction does not switch cache pages");
	CHECK(disk.SectRead(24U, back), "failed commit retains dirty data");
	CHECK(back[0] == 0xB6, "failed commit retains modified sector");
	s_Erase.vbFailErase = false;
	CHECK(disk.Commit(), "retry failed commit");
	CHECK(s_Erase.vBuf[3U * DISK_SECT] == 0xB6,
		"retried commit persists modified data");
}

static void TestCheckedGenericCommit(void)
{
	static uint8_t cacheMem[DISK_SECT];
	static DiskIOCache_t cache = { 0, 0xFFFFFFFF, cacheMem };
	NvmDiskIO disk;
	uint8_t data[32];
	uint8_t back[32];

	CHECK(s_Erase.Erase(0, DISK_SIZE) == 0, "clear mock for checked commit");
	CHECK(disk.Init(s_Erase), "initialize uncached NVM sector device");
	disk.SetCache(&cache, 1);
	memset(data, 0x42, sizeof(data));
	CHECK(disk.Write((uint64_t)16U, data, sizeof(data)) ==
		(int)sizeof(data), "populate generic RAM sector cache");

	s_Erase.vbFailErase = true;
	CHECK(!disk.Commit(), "generic cached write failure reported");
	s_Erase.vbFailErase = false;
	CHECK(disk.Commit(), "generic failed cached write can be retried");
	CHECK(disk.SectRead(0U, cacheMem), "read committed physical sector");
	memcpy(back, cacheMem + 16U, sizeof(back));
	CHECK(memcmp(data, back, sizeof(data)) == 0,
		"generic committed data is persistent");

	memset(data, 0x5C, sizeof(data));
	CHECK(disk.Write((uint64_t)16U, data, sizeof(data)) ==
		(int)sizeof(data), "update generic cached sector");
	s_Erase.vbFailErase = true;
	CHECK(disk.Write((uint64_t)DISK_SECT + 16U, data,
		sizeof(data)) == -1, "failed eviction propagates to caller");
	CHECK(disk.Read((uint64_t)16U, back, sizeof(back)) ==
		(int)sizeof(back), "failed eviction retains cached sector");
	CHECK(memcmp(data, back, sizeof(back)) == 0,
		"failed eviction keeps modified data");
	s_Erase.vbFailErase = false;
	CHECK(disk.Commit(), "recover after failed generic eviction");
	CHECK(disk.Read((uint64_t)16U, back, sizeof(back)) ==
		(int)sizeof(back), "read recovered data");
	CHECK(memcmp(data, back, sizeof(back)) == 0,
		"recovered data intact");
}

int main(void)
{
	NvmDiskIO diskErase;
	NvmDiskIO diskDirect;

	// The erase-write medium reports its own logical sector.
	CHECK(diskErase.Init(s_Erase), "init on an erase-write medium");
	// The direct medium has no erase unit, so it is given a sector size.
	CHECK(diskDirect.Init(s_Direct, DISK_SECT), "init on a direct write medium");

	CHECK(diskErase.GetMinEraseSize() == DISK_SECT, "erase medium min erase");
	CHECK(diskDirect.GetMinEraseSize() == DISK_SECT, "direct medium min erase");

	// Sub-erase sectors require a scratch buffer spanning an erase unit.
	NvmDiskIO bad;
	CHECK(bad.Init(s_Erase, DISK_SECT / 2) == false,
		  "sub-erase without scratch refused");

	diskErase.SetCache(&s_Cache1, 1);
	diskDirect.SetCache(&s_Cache2, 1);

	RunDisk(diskErase, "erase-write medium");
	RunDisk(diskDirect, "direct write medium");
	TestSubEraseLogicalSectors();
	TestWriteBackErasePage();
	TestCheckedGenericCommit();

	printf("\nChecks run: %d\n", s_Checks);
	if (s_Fail == 0)
	{
		printf("RESULT: ALL PASS\n");
		return 0;
	}
	printf("RESULT: %d FAILURES\n", s_Fail);
	return 1;
}
