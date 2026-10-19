/**-------------------------------------------------------------------------
@file	bond_flash_fw.cpp

@brief	STM32WBA bond store test firmware over the flash controller model

Runs on the instruction set emulator of run_bond_flash.py, which models the
STM32WBA flash controller with write protection. The firmware links the real
bond store (bt_smp_bond_stm32wba.cpp), the real PDS store (bt_pds.cpp,
bt_pds_stm32wba.cpp) and the real flash driver (nvm_stm32.cpp), and plays
the part of the ST stack: it registers a cache, asks for saves through
BLEPLAT_NvmStore and calls the one-second poll.

The emulator sets g_BondFwBoot before each boot and keeps the flash between
boots, so a boot is a reset. BondFwProtect asks the emulator to refuse writes
to the bond store region: -1 off, 0 from now on, n from the n-th next
BtPdsWrite call. Printed lines go to the emulator through BondFwConsole, and
the emulator checks the retry times against the poll count. Boots 6 to 8
are the power cut sweep of run_bond_flash.py --powercut: prepare the old
image, the save the emulator cuts, and the boot after the cut.

Not hardware validation.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "syslog.h"
#include "coredev/spi.h"
#include "coredev/iopincfg.h"
#include "bluetooth/bt_pds.h"

// Image of the stack's NVM cache: 39 words, 312 bytes, two PDS chunks
#define BOND_FW_WORDS		39U
#define BOND_FW_CACHE_WORDS	80U

extern "C" uint16_t BtSmpBondWbaCacheInit(uint64_t *pCache, uint16_t MaxWords);
extern "C" size_t BtSmpBondWbaRestore(void *pCacheBuf, size_t Capacity);
extern "C" void BLEPLAT_NvmStore(const uint64_t *ptr, uint16_t size);
extern "C" void BtSmpBondWbaFlush(void);
extern "C" void BtSmpBondWbaPoll(void);

extern "C" void BondFwConsole(const char *pStr, int Len) __attribute__((noinline));
extern "C" void BondFwProtect(int Mode) __attribute__((noinline));
extern "C" void BondFwDone(void) __attribute__((noinline));

extern "C" volatile uint32_t g_BondFwBoot;
extern "C" volatile int32_t g_BondFwResult;

// Power cut sweep of run_bond_flash.py --powercut: old seed, old words, new
// seed, new words, filler records written after the old image, and 1 when
// the old image is saved twice so that it sits in the second bank.
extern "C" volatile uint32_t g_BondFwArg[6];

// Written by the emulator before the start code runs, which leaves them alone
volatile uint32_t g_BondFwBoot __attribute__((section(".noinit")));
volatile uint32_t g_BondFwArg[6] __attribute__((section(".noinit")));
volatile int32_t g_BondFwResult = -1000;

static uint64_t s_Cache[BOND_FW_CACHE_WORDS];
static uint64_t s_Check[BOND_FW_CACHE_WORDS];
static int s_Fail;

extern "C" void BondFwConsole(const char *pStr, int Len)
{
	__asm volatile ("" : : "r" (pStr), "r" (Len) : "memory");
}

extern "C" void BondFwProtect(int Mode)
{
	__asm volatile ("" : : "r" (Mode) : "memory");
}

extern "C" void BondFwDone(void)
{
	__asm volatile ("" ::: "memory");
}

// The store prints through SysLog; here every line goes to the emulator.
SysLog_t *SysLogGet(void)
{
	return nullptr;
}

int SysLogPrintf(SysLog_t * const pLog, const char *pFormat, ...)
{
	char buf[160];
	va_list args;

	(void)pLog;
	va_start(args, pFormat);
	int n = vsnprintf(buf, sizeof(buf), pFormat, args);
	va_end(args);
	if (n > (int)sizeof(buf) - 1)
	{
		n = sizeof(buf) - 1;
	}
	if (n > 0)
	{
		BondFwConsole(buf, n);
	}

	return n;
}

static void Print(const char *pFormat, ...)
{
	char buf[160];
	va_list args;

	va_start(args, pFormat);
	int n = vsnprintf(buf, sizeof(buf), pFormat, args);
	va_end(args);
	if (n > (int)sizeof(buf) - 1)
	{
		n = sizeof(buf) - 1;
	}
	if (n > 0)
	{
		BondFwConsole(buf, n);
	}
}

static void Check(bool Ok, const char *pName)
{
	Print("[%s] %s\r\n", Ok ? "PASS" : "FAIL", pName);
	if (!Ok)
	{
		s_Fail++;
	}
}

// Nvm references these for QSPI parts and for a write protect pin. The
// internal flash takes neither path, so a call is a test failure.
void QuadSPISetMemSize(SPIDev_t * const pDev, uint32_t Size)
{
	(void)pDev;
	(void)Size;
	Check(false, "QuadSPISetMemSize called");
}

bool QuadSPISendCmd(SPIDev_t * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen,
					uint32_t DataLen, uint8_t DummyCycle)
{
	(void)pDev;
	(void)Cmd;
	(void)Addr;
	(void)AddrLen;
	(void)DataLen;
	(void)DummyCycle;
	Check(false, "QuadSPISendCmd called");

	return false;
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	(void)PortNo;
	(void)PinNo;
	(void)PinOp;
	(void)Dir;
	(void)Resistor;
	(void)Type;
	Check(false, "IOPinConfig called");
}

static uint64_t Word(uint32_t Seed, uint32_t Idx)
{
	return (0x9E3779B97F4A7C15ULL * (Seed + 1U)) ^ ((uint64_t)Idx << 8) ^ Idx;
}

static void Fill(uint32_t Seed, uint32_t Words = BOND_FW_WORDS)
{
	for (uint32_t i = 0; i < Words; i++)
	{
		s_Cache[i] = Word(Seed, i);
	}
}

static bool CacheIs(uint32_t Seed, uint32_t Words = BOND_FW_WORDS)
{
	for (uint32_t i = 0; i < Words; i++)
	{
		if (s_Cache[i] != Word(Seed, i))
		{
			return false;
		}
	}

	return true;
}

// What a reset would hand the stack now, without touching the stack's cache
static bool StoredIs(uint32_t Seed, uint32_t Words = BOND_FW_WORDS)
{
	memset(s_Check, 0xA5, sizeof(s_Check));
	if (BtSmpBondWbaRestore(s_Check, sizeof(s_Check)) != Words * 8U)
	{
		return false;
	}
	for (uint32_t i = 0; i < Words; i++)
	{
		if (s_Check[i] != Word(Seed, i))
		{
			return false;
		}
	}

	return true;
}

static bool NothingStored(void)
{
	memset(s_Check, 0xA5, sizeof(s_Check));
	if (BtSmpBondWbaRestore(s_Check, sizeof(s_Check)) != 0)
	{
		return false;
	}
	for (uint32_t i = 0; i < BOND_FW_CACHE_WORDS; i++)
	{
		if (s_Check[i] != 0)
		{
			return false;
		}
	}

	return true;
}

// The stack changed its cache and asks for a save; the application loop
// then runs its status check, which flushes.
static void Save(uint32_t Seed, uint32_t Words = BOND_FW_WORDS)
{
	Fill(Seed, Words);
	BLEPLAT_NvmStore(s_Cache, (uint16_t)Words);
	BtSmpBondWbaFlush();
}

// One second of the application: the timer tick, then idle loop passes
static void Seconds(int Count)
{
	for (int i = 0; i < Count; i++)
	{
		BtSmpBondWbaPoll();
		BtSmpBondWbaFlush();
		BtSmpBondWbaFlush();
	}
}

static void Boot0(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == 0,
		  "erased store restores nothing");
	Save(1);
	Check(StoredIs(1), "first save stored");
}

static void Boot1(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == BOND_FW_WORDS &&
		  CacheIs(1), "image restored after reset");

	// The medium refuses every write until the save is dropped
	BondFwProtect(0);
	Save(2);
	Seconds(140);
	BondFwProtect(-1);
	Seconds(10);
	Check(StoredIs(1), "old image kept while every write was refused");

	BLEPLAT_NvmStore(s_Cache, BOND_FW_WORDS);
	BtSmpBondWbaFlush();
	Check(StoredIs(2), "new save request after the drop is stored");
}

static void Boot2(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == BOND_FW_WORDS &&
		  CacheIs(2), "image restored after reset, after a refused medium");

	// Writes come back during the wait after the second try
	BondFwProtect(0);
	Save(3);
	Seconds(2);
	BondFwProtect(-1);
	Seconds(1);
	Check(StoredIs(3), "retry stores once the medium takes writes again");
}

static void Boot3(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == BOND_FW_WORDS &&
		  CacheIs(3), "image restored after reset, after a retry");

	// The first chunk goes down, the second is refused. Only the bank the
	// manifest does not select has changed, so the committed image stays.
	BondFwProtect(2);
	Save(4);
	Check(StoredIs(3), "partly written save keeps the committed image");
	BondFwProtect(-1);
	Seconds(1);
	Check(StoredIs(4), "retry completes the partly written save");
}

static void Boot4(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == BOND_FW_WORDS &&
		  CacheIs(4), "completed save restored after reset");
}

// Image linked without the bond store region
static void BootNoRegion(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == 0,
		  "no region restores nothing");
	Save(5);
	Seconds(10);
	Check(NothingStored(), "no region stores nothing");
}

// What the start files and the system file give a firmware build: the handle
// the static destructors register with, and the delay loop count (one loop
// per microsecond is enough here, the flash model is never busy).
extern "C" void *__dso_handle;
extern "C" uint32_t SystemMicroSecLoopCnt;

void *__dso_handle = nullptr;
uint32_t SystemMicroSecLoopCnt = 1;

// Power cut sweep. Prepare: the old image, and filler records so that the
// next save runs the PDS garbage collection when asked for.
static void BootCutPrepare(void)
{
	static const uint8_t filler[BT_PDS_RECORD_DATA_MAX] = {};

	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == 0, "cut sweep: erased store");
	if (g_BondFwArg[5])
	{
		Save(g_BondFwArg[0] + 100U, g_BondFwArg[1]);
	}
	Save(g_BondFwArg[0], g_BondFwArg[1]);
	Check(StoredIs(g_BondFwArg[0], g_BondFwArg[1]), "cut sweep: old image stored");
	for (uint32_t i = 0; i < g_BondFwArg[4]; i++)
	{
		if (BtPdsWrite(0x1234U, filler, sizeof(filler)) != (ssize_t)sizeof(filler))
		{
			Check(false, "cut sweep: filler record");
			break;
		}
	}
}

// The save the emulator cuts at a chosen flash operation
static void BootCutSave(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == g_BondFwArg[1] &&
		  CacheIs(g_BondFwArg[0], g_BondFwArg[1]), "cut sweep: old image restored before the save");
	Save(g_BondFwArg[2], g_BondFwArg[3]);
	Check(StoredIs(g_BondFwArg[2], g_BondFwArg[3]), "cut sweep: save completed");
}

// The boot after the cut: the old or the new image, never anything else,
// and the store takes the next save.
static void BootCutCheck(void)
{
	uint16_t n = BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS);
	bool isOld = n == g_BondFwArg[1] && CacheIs(g_BondFwArg[0], g_BondFwArg[1]);
	bool isNew = n == g_BondFwArg[3] && CacheIs(g_BondFwArg[2], g_BondFwArg[3]);

	Print("cut sweep: restored %s\r\n", isOld ? "old" : (isNew ? "new" : "neither"));
	Check(isOld || isNew, "after a power cut the restore is the old or the new image");
	Save(g_BondFwArg[2], g_BondFwArg[3]);
	Check(StoredIs(g_BondFwArg[2], g_BondFwArg[3]), "save after the power cut stored");
}

extern "C" void __cxa_pure_virtual(void)
{
	while (1)
	{
	}
}

void operator delete(void *) noexcept {}
void operator delete(void *, size_t) noexcept {}

extern "C" int main(void)
{
	static void (* const s_Boot[])(void) = {
		Boot0, Boot1, Boot2, Boot3, Boot4, BootNoRegion,
		BootCutPrepare, BootCutSave, BootCutCheck
	};
	uint32_t boot = g_BondFwBoot;

	Print("bond flash test, boot %lu\r\n", (unsigned long)boot);
	if (boot < sizeof(s_Boot) / sizeof(s_Boot[0]))
	{
		s_Boot[boot]();
	}
	else
	{
		Check(false, "boot number");
	}
	g_BondFwResult = s_Fail;
	BondFwDone();

	return 0;
}

// Minimal start: stack, data and bss, the static constructors of the store
// objects, then main
extern "C" uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern "C" void (*__init_array_start[])(void);
extern "C" void (*__init_array_end[])(void);

extern "C" void ResetHandler(void)
{
	uint32_t *src = &_sidata;

	for (uint32_t *dst = &_sdata; dst < &_edata; )
	{
		*dst++ = *src++;
	}
	for (uint32_t *dst = &_sbss; dst < &_ebss; )
	{
		*dst++ = 0;
	}
	for (void (**fn)(void) = __init_array_start; fn < __init_array_end; fn++)
	{
		(*fn)();
	}
	main();
	while (1)
	{
	}
}

extern "C" __attribute__((section(".isr_vector"), used))
void (* const __Vectors[16 + 128])(void) = {
	(void (*)(void))&_estack,
	ResetHandler,
};
