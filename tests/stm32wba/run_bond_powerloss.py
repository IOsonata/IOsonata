#!/usr/bin/env python3
"""Power-cut tests of the WBA bond adapter with the production PDS.

The Nvm model uses two 8 KB sectors and 16-byte programming units, as on
WBA65. Cut every program unit and erase operation, remount, and check that
the committed image survives. This does not exercise the flash controller.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
NVM = r'''
#pragma once
#include <cstdint>
class Nvm {
public:
	uint64_t Size() const { return 16384; }
	uint32_t LogicalSectorSize() const { return 8192; }
	uint32_t EraseSize() const { return 8192; }
	uint32_t WriteGran() const { return 16; }
	int Read(uint64_t Off, void *pData, uint32_t Len);
	int Write(uint64_t Off, const void *pData, uint32_t Len);
	int Erase(uint64_t Off, uint32_t Len);
	int Sync() { return 0; }
};
'''
TEST = r'''
#include <array>
#include <vector>
#include <cassert>
#include <cstring>
#include <cstdio>
#include "storage/nvm.h"
#include "src/bluetooth/bt_pds.cpp"
#include "ARM/ST/STM32WBAxx/src/bt_smp_bond_stm32wba.cpp"

struct PowerCut {};
static std::array<uint8_t, 16384> flash;
static Nvm medium;
static int budget = -1;
static unsigned operations, erases, resets;
static bool partial;
static uint64_t cache[512];

int Nvm::Read(uint64_t Off, void *pData, uint32_t Len)
{
	assert(Off + Len <= flash.size());
	memcpy(pData, flash.data() + Off, Len);
	return Len;
}

int Nvm::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	assert(Off % 16 == 0 && Len % 16 == 0 && Off + Len <= flash.size());
	const uint8_t *p = (const uint8_t *)pData;
	for (unsigned i = 0; i < Len; i += 16)
	{
		unsigned count = budget == 0 ? (partial ? 8 : 0) : 16;
		for (unsigned j = 0; j < count; j++)
		{
			assert((flash[Off + i + j] & p[i + j]) == p[i + j]);
			flash[Off + i + j] &= p[i + j];
		}
		if (budget == 0)
		{
			throw PowerCut();
		}
		operations++;
		if (budget > 0)
		{
			budget--;
		}
	}
	return Len;
}

int Nvm::Erase(uint64_t Off, uint32_t Len)
{
	assert(Off % 8192 == 0 && Len == 8192 && Off + Len <= flash.size());
	if (budget == 0)
	{
		if (partial)
		{
			memset(flash.data() + Off, 0xFF, Len / 2);
		}
		throw PowerCut();
	}
	memset(flash.data() + Off, 0xFF, Len);
	operations++;
	erases++;
	if (budget > 0)
	{
		budget--;
	}
	return 0;
}

extern "C" int BtPdsWbaInit(void)
{
	return BtPdsInit(&medium);
}

static size_t Boot()
{
	budget = -1;
	s_PdsReady = false;
	s_bStorePending = false;
	s_RetryWait = 0;
	s_RetryInterval = 0;
	s_StoreWords = 0;
	return (size_t)BtSmpBondWbaCacheInit(cache, 512) * 8;
}

static std::vector<uint8_t> Image(unsigned Len, uint8_t Seed)
{
	std::vector<uint8_t> image(Len);
	for (unsigned i = 0; i < Len; i++)
	{
		image[i] = (uint8_t)(Seed + i * 17U);
	}
	return image;
}

static void Save(const std::vector<uint8_t> &Image)
{
	if (!Image.empty())
	{
		memcpy(cache, Image.data(), Image.size());
	}
	BLEPLAT_NvmStore(cache, (uint16_t)(Image.size() / 8));
	BtSmpBondWbaFlush();
}

static bool Matches(size_t Len, const std::vector<uint8_t> &Image)
{
	return Len == Image.size() &&
		(Len == 0 || memcmp(cache, Image.data(), Len) == 0);
}

static void CutEveryWrite(const std::vector<uint8_t> &Old,
		const std::vector<uint8_t> &New, bool RequireGc)
{
	const auto before = flash;
	assert(Matches(Boot(), Old));
	operations = erases = 0;
	Save(New);
	assert(!s_bStorePending);
	unsigned steps = operations;
	assert(!RequireGc || erases != 0);
	assert(Matches(Boot(), New));
	const auto after = flash;
	for (unsigned cut = 0; cut <= steps; cut++)
	{
		for (bool tear : {false, true})
		{
			flash = before;
			assert(Matches(Boot(), Old));
			budget = (int)cut;
			partial = tear;
			try
			{
				Save(New);
			}
			catch (PowerCut &)
			{
			}
			size_t restored = Boot();
			assert(Matches(restored, Old) || Matches(restored, New));
			// Recovery must also leave the store writable.
			Save(New);
			assert(!s_bStorePending);
			assert(Matches(Boot(), New));
			resets++;
		}
	}
	flash = after;
	assert(Matches(Boot(), New));
}

int main()
{
	flash.fill(0xFF);
	assert(Boot() == 0);
	auto first = Image(312, 0x31);
	auto second = Image(512, 0x72);
	Save(first);
	assert(Matches(Boot(), first));
	CutEveryWrite(first, second, false);
	CutEveryWrite(second, first, false);
	puts("PASS both bank directions, every program boundary and torn unit");

	// Force the next save through the production sector garbage collector.
	uint8_t filler[BT_PDS_RECORD_DATA_MAX] = {};
	while (s_Sectors[s_ActiveSector].Head + 208 <= SectorEnd(s_ActiveSector))
	{
		assert(BtPdsWrite(0x1234, filler, sizeof(filler)) == sizeof(filler));
	}
	CutEveryWrite(first, second, true);
	puts("PASS power loss during PDS garbage collection and source erase");
	CutEveryWrite(second, {}, false);
	CutEveryWrite({}, first, false);
	puts("PASS empty-image commit does not resurrect deleted bonds");

	// The original eight-byte manifest and bank-0 records remain readable.
	flash.fill(0xFF);
	assert(Boot() == 0);
	for (unsigned off = 0; off < first.size(); off += BT_WBA_CHUNK_SIZE)
	{
		unsigned len = (unsigned)first.size() - off;
		if (len > BT_WBA_CHUNK_SIZE)
		{
			len = BT_WBA_CHUNK_SIZE;
		}
		assert(BtPdsWrite(BT_WBA_NVM_KEY_BASE + 1 + off / BT_WBA_CHUNK_SIZE,
			first.data() + off, len) == len);
	}
	uint32_t legacy[2] = {(uint32_t)first.size(),
		crc32_ieee(first.data(), (int)first.size())};
	assert(BtPdsWrite(BT_WBA_NVM_KEY_BASE, legacy, sizeof(legacy)) == sizeof(legacy));
	CutEveryWrite(first, second, false);
	puts("PASS legacy migration retains the old image until commit");

	// Two full 4 KB banks do not fit the live set of a two-sector PDS.
	// Exhaustion must preserve the last committed bond, including after retry.
	flash.fill(0xFF);
	assert(Boot() == 0);
	auto large = Image(4096, 0x11);
	Save(large);
	assert(!s_bStorePending && Matches(Boot(), large));
	Save(Image(4096, 0x22));
	assert(s_bStorePending);
	for (unsigned i = 0; i < 300; i++)
	{
		BtSmpBondWbaPoll();
	}
	assert(!s_bStorePending && Matches(Boot(), large));
	puts("PASS store exhaustion retains committed image after bounded retries");
	printf("%u simulated power cuts passed\n", resets);
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix='wba-bond-powerloss-') as td:
        tmp = Path(td)
        (tmp / 'storage').mkdir()
        (tmp / 'storage/nvm.h').write_text(NVM)
        (tmp / 'syslog.h').write_text(
            '#pragma once\ninline void *SysLogGet() { return nullptr; }\n'
            'inline void SysLogPrintf(void *, const char *, ...) {}\n')
        (tmp / 'test.cpp').write_text(TEST)
        for opt in ('-O0', '-Os'):
            san = ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
            crc = tmp / 'crc.o'
            subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu17', opt,
                            *san, '-I' + str(ROOT / 'include'), '-c',
                            str(ROOT / 'src/crc.c'), '-o', str(crc)], check=True)
            exe = tmp / 'test'
            subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++2b', opt,
                            '-Wall', '-Wextra', '-Werror', *san,
                            '-I' + str(tmp), '-I' + str(ROOT),
                            '-I' + str(ROOT / 'include'), str(tmp / 'test.cpp'),
                            str(crc), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(opt + ': WBA bond power-loss checks passed', flush=True)


if __name__ == '__main__':
    main()
