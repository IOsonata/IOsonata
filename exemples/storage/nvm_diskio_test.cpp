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
	  exemples/storage/nvm_diskio_test.cpp src/storage/diskio_nvm.cpp \
	  src/storage/diskio_impl.cpp src/device.cpp src/device_intrf.cpp \
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


// The pin driver is per architecture; the driver only toggles a protect pin.
extern "C" {
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinSet(int, int) {}
void IOPinClear(int, int) {}
}

#define DISK_SIZE		(64u * 1024u)
#define DISK_SECT		4096u

static int g_Fail = 0;
static int g_Checks = 0;

#define CHECK(cond, ...) do { \
	g_Checks++; \
	if (!(cond)) { \
		g_Fail++; \
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

static EraseNvm s_Erase;
static DirectNvm s_Direct;

static uint8_t s_CacheMem1[DISK_SECT];
static uint8_t s_CacheMem2[DISK_SECT];
static DiskIOCache_t s_Cache1 = { 0, 0xFFFFFFFF, s_CacheMem1 };
static DiskIOCache_t s_Cache2 = { 0, 0xFFFFFFFF, s_CacheMem2 };

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

	// A sector that is not a whole number of erase units is refused.
	NvmDiskIO bad;
	CHECK(bad.Init(s_Erase, DISK_SECT / 2) == false, "part of an erase unit refused");

	diskErase.SetCache(&s_Cache1, 1);
	diskDirect.SetCache(&s_Cache2, 1);

	RunDisk(diskErase, "erase-write medium");
	RunDisk(diskDirect, "direct write medium");

	printf("\nChecks run: %d\n", g_Checks);
	if (g_Fail == 0)
	{
		printf("RESULT: ALL PASS\n");
		return 0;
	}
	printf("RESULT: %d FAILURES\n", g_Fail);
	return 1;
}
