#!/usr/bin/env python3
"""WBA bond adapter failure tests with an in-memory PDS and the real CRC.

Compiles the complete port adapter. PDS failures are injected at mount,
chunk and manifest writes, including the retry back-off, the retry limit and
a missing NVM region. This does not test flash or radio scheduling.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST = r'''
#include <algorithm>
#include <cassert>
#include <cstring>
#include <cstdio>
#include <map>
#include <vector>
#include "ARM/ST/STM32WBAxx/src/bt_smp_bond_stm32wba.cpp"

static std::map<uint32_t, std::vector<uint8_t>> records;
static int mountResult;
static int mountCalls;
static int writes;
static int failWrite;
static bool requestDuringWrite;
static uint64_t cache[64];
static uint64_t restored[64];

extern "C" int BtPdsWbaInit(void)
{
	mountCalls++;
	return mountResult;
}

extern "C" ssize_t BtPdsWrite(uint32_t Id, const void *pData, size_t Len)
{
	writes++;
	if (writes == failWrite)
	{
		return -EIO;
	}
	const uint8_t *p = (const uint8_t *)pData;
	records[Id] = std::vector<uint8_t>(p, p + Len);
	if (requestDuringWrite)
	{
		requestDuringWrite = false;
		BLEPLAT_NvmStore(cache, 32);
	}
	return Len;
}

extern "C" ssize_t BtPdsRead(uint32_t Id, void *pData, size_t Len)
{
	auto it = records.find(Id);
	if (it == records.end())
	{
		return -ENOENT;
	}
	memcpy(pData, it->second.data(), std::min(Len, it->second.size()));
	return it->second.size();
}

// The bond deletion erases the store; tests/stm32wba/run_bond_flash.py covers
// it over the real PDS and flash driver.
extern "C" int BtPdsClear(void)
{
	records.clear();
	return 0;
}

static void Reset()
{
	records.clear();
	mountResult = 0;
	writes = failWrite = 0;
	requestDuringWrite = false;
	s_PdsReady = false;
	s_bStorePending = false;
	s_bErasePending = false;
	s_RetryWait = 0;
	s_RetryInterval = 0;
	s_pCache = nullptr;
	assert(BtSmpBondWbaCacheInit(cache, 64) == 0);
	for (unsigned i = 0; i < 64; i++)
	{
		cache[i] = 0x1122334455667700ULL + i;
	}
}

static void CheckRestored(size_t Len, const void *pExpected = cache)
{
	memset(restored, 0xA5, sizeof(restored));
	assert(BtSmpBondWbaRestore(restored, sizeof(restored)) == Len);
	assert(memcmp(restored, pExpected, Len) == 0);
	const uint8_t *p = (const uint8_t *)restored;
	for (size_t i = Len; i < sizeof(restored); i++)
	{
		assert(p[i] == 0);
	}
}

int main()
{
	Reset();
	s_PdsReady = false;
	mountResult = -EIO;
	BLEPLAT_NvmStore(cache + 2, 39);
	BtSmpBondWbaFlush();
	assert(writes == 0 && s_bStorePending);
	mountResult = 0;
	BtSmpBondWbaFlush();
	assert(writes == 0);
	BtSmpBondWbaPoll();
	assert(!s_bStorePending);
	CheckRestored(312);
	puts("PASS mount failure retained and retried, portion pointer uses cache base");

	// A 312-byte image has two chunks and one manifest.
	for (int fail = 1; fail <= 3; fail++)
	{
		Reset();
		BLEPLAT_NvmStore(cache, 39);
		failWrite = fail;
		BtSmpBondWbaFlush();
		assert(s_bStorePending);
		int failedAt = writes;
		BtSmpBondWbaFlush();
		assert(writes == failedAt);
		failWrite = 0;
		BtSmpBondWbaPoll();
		assert(!s_bStorePending);
		CheckRestored(312);
		int before = writes;
		BtSmpBondWbaFlush();
		assert(writes == before);
	}
	puts("PASS each chunk and manifest failure retained, timed retry commits once");

	Reset();
	BLEPLAT_NvmStore(cache, 39);
	requestDuringWrite = true;
	BtSmpBondWbaFlush();
	assert(s_bStorePending);
	BtSmpBondWbaFlush();
	assert(!s_bStorePending);
	CheckRestored(256);
	puts("PASS new request during flush retained");

	Reset();
	BLEPLAT_NvmStore(cache, 39);
	BtSmpBondWbaFlush();
	uint64_t previous[64];
	memcpy(previous, cache, sizeof(previous));
	cache[0] ^= 1;
	BLEPLAT_NvmStore(cache, 39);
	failWrite = writes + 2;
	BtSmpBondWbaFlush();
	CheckRestored(312, previous);
	failWrite = 0;
	BtSmpBondWbaPoll();
	CheckRestored(312);
	puts("PASS interrupted bank update preserves committed image, retry commits replacement");

	BtWbaNvmManifest_t active;
	assert(BondManifestRead(&active) == 0);
	records.erase(BondChunkKey(active.Bank, 1));
	CheckRestored(0);
	puts("PASS missing chunk clears entire destination");

	Reset();
	BLEPLAT_NvmStore(cache, 39);
	BtSmpBondWbaFlush();
	records[BT_WBA_NVM_KEY_BASE + 1][0] ^= 1;
	CheckRestored(0);
	puts("PASS bad CRC clears entire destination");

	Reset();
	BLEPLAT_NvmStore(cache, 39);
	BtSmpBondWbaFlush();
	BLEPLAT_NvmStore(cache, 0);
	BtSmpBondWbaFlush();
	CheckRestored(0);
	puts("PASS empty image replaces stored bond");

	Reset();
	BLEPLAT_NvmStore(nullptr, 39);
	BLEPLAT_NvmStore(cache + 64, 39);
	BLEPLAT_NvmStore(cache, 65);
	BtSmpBondWbaFlush();
	assert(writes == 0);
	puts("PASS invalid store requests refused");

	BtWbaNvmManifest_t man = { 7, 0, 0 };
	BtPdsWrite(BT_WBA_NVM_KEY_BASE, &man, sizeof(man));
	CheckRestored(0);
	puts("PASS non-word image length refused");

	// A save that keeps failing is tried after 1, 2, 4 .. 64 polls, then
	// dropped. Flush calls from the idle loop between polls do not try it.
	Reset();
	s_PdsReady = false;
	mountResult = -EIO;
	mountCalls = 0;
	BLEPLAT_NvmStore(cache, 39);
	BtSmpBondWbaFlush();
	assert(mountCalls == 1 && s_bStorePending);
	for (unsigned gap = 1; gap <= BT_WBA_STORE_RETRY_MAX; gap *= 2)
	{
		int before = mountCalls;
		for (unsigned i = 1; i < gap; i++)
		{
			BtSmpBondWbaPoll();
			BtSmpBondWbaFlush();
			assert(mountCalls == before);
		}
		BtSmpBondWbaPoll();
		assert(mountCalls == before + 1);
	}
	assert(!s_bStorePending && mountCalls == 8);
	for (int i = 0; i < 300; i++)
	{
		BtSmpBondWbaPoll();
		BtSmpBondWbaFlush();
	}
	assert(mountCalls == 8 && writes == 0);
	puts("PASS failed save waits 1 to 64 polls, then dropped");

	mountResult = 0;
	BLEPLAT_NvmStore(cache, 39);
	BtSmpBondWbaFlush();
	assert(!s_bStorePending);
	CheckRestored(312);
	puts("PASS new store request after a drop saves at once");

	// No NVM region: dropped at once, never retried.
	Reset();
	s_PdsReady = false;
	mountResult = -ENODEV;
	mountCalls = 0;
	BLEPLAT_NvmStore(cache, 39);
	BtSmpBondWbaFlush();
	assert(mountCalls == 1 && !s_bStorePending && s_RetryWait == 0);
	for (int i = 0; i < 10; i++)
	{
		BtSmpBondWbaPoll();
	}
	assert(mountCalls == 1 && writes == 0);
	puts("PASS missing NVM region not retried");

	// A save that succeeds after a retry starts the next failure at 1 s.
	Reset();
	BLEPLAT_NvmStore(cache, 39);
	failWrite = 1;
	BtSmpBondWbaFlush();
	assert(s_bStorePending && s_RetryInterval == 1);
	failWrite = writes + 1;
	BtSmpBondWbaPoll();
	assert(s_bStorePending && s_RetryInterval == 2 && s_RetryWait == 2);
	failWrite = 0;
	BtSmpBondWbaPoll();
	assert(s_bStorePending);
	BtSmpBondWbaPoll();
	assert(!s_bStorePending && s_RetryInterval == 0);
	CheckRestored(312);
	cache[1] ^= 1;
	BLEPLAT_NvmStore(cache, 39);
	failWrite = writes + 1;
	BtSmpBondWbaFlush();
	assert(s_bStorePending && s_RetryInterval == 1 && s_RetryWait == 1);
	failWrite = 0;
	BtSmpBondWbaPoll();
	assert(!s_bStorePending);
	CheckRestored(312);
	puts("PASS successful save resets the retry wait");
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix='wba-bond-') as td:
        tmp = Path(td)
        (tmp / 'syslog.h').write_text(
            '#pragma once\ninline void *SysLogGet() { return nullptr; }\n'
            'inline void SysLogPrintf(void *, const char *, ...) {}\n')
        (tmp / 'test.cpp').write_text(TEST)
        san = ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        for opt in ('-O0', '-Os'):
            exe = tmp / 'test'
            # crc.c is C, as in the firmware; crc.h declares C linkage. A C++
            # driver given a .c file treats it as C++, which clang refuses.
            crc = tmp / 'crc.o'
            subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu17', opt, *san,
                            '-I' + str(ROOT / 'include'), '-c',
                            str(ROOT / 'src/crc.c'), '-o', str(crc)], check=True)
            subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++2b', opt,
                            '-Wall', '-Wextra', '-Werror', *san, '-I' + str(tmp),
                            '-I' + str(ROOT), '-I' + str(ROOT / 'include'),
                            str(tmp / 'test.cpp'), str(crc),
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(opt + ': WBA bond store checks passed', flush=True)


if __name__ == '__main__':
    main()

