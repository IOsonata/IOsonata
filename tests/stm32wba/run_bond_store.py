#!/usr/bin/env python3
"""WBA bond adapter failure tests with an in-memory PDS and the real CRC.

Compiles the complete port adapter. PDS failures are injected at mount,
chunk and manifest writes. This does not test flash or radio scheduling.
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
static int writes;
static int failWrite;
static bool requestDuringWrite;
static uint64_t cache[64];
static uint64_t restored[64];

extern "C" int BtPdsWbaInit(void)
{
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

static void Reset()
{
	records.clear();
	mountResult = 0;
	writes = failWrite = 0;
	requestDuringWrite = false;
	s_PdsReady = false;
	s_bStorePending = false;
	s_bStoreRetry = false;
	s_pCache = nullptr;
	assert(BtSmpBondWbaCacheInit(cache, 64) == 0);
	for (unsigned i = 0; i < 64; i++)
	{
		cache[i] = 0x1122334455667700ULL + i;
	}
}

static void CheckRestored(size_t Len)
{
	memset(restored, 0xA5, sizeof(restored));
	assert(BtSmpBondWbaRestore(restored, sizeof(restored)) == Len);
	assert(memcmp(restored, cache, Len) == 0);
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
	cache[0] ^= 1;
	BLEPLAT_NvmStore(cache, 39);
	failWrite = writes + 2;
	BtSmpBondWbaFlush();
	CheckRestored(0);
	failWrite = 0;
	BtSmpBondWbaPoll();
	CheckRestored(312);
	puts("PASS mixed old/new chunks rejected and cache cleared, retry repairs image");

	records.erase(BT_WBA_NVM_KEY_BASE + 2);
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

	BtWbaNvmManifest_t man = { 7, 0 };
	BtPdsWrite(BT_WBA_NVM_KEY_BASE, &man, sizeof(man));
	CheckRestored(0);
	puts("PASS non-word image length refused");
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix='wba-bond-') as td:
        tmp = Path(td)
        (tmp / 'syslog.h').write_text(
            '#pragma once\ninline void *SysLogGet() { return nullptr; }\n'
            'inline void SysLogPrintf(void *, const char *, ...) {}\n')
        (tmp / 'test.cpp').write_text(TEST)
        for opt in ('-O0', '-Os'):
            exe = tmp / 'test'
            subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++23', opt,
                            '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', '-I' + str(tmp),
                            '-I' + str(ROOT), '-I' + str(ROOT / 'include'),
                            str(tmp / 'test.cpp'), str(ROOT / 'src/crc.c'),
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
            print(opt + ': WBA bond store checks passed', flush=True)


if __name__ == '__main__':
    main()
