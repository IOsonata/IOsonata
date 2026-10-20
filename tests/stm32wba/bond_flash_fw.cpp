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
the emulator checks the retry times against the poll count. The vector table
is the WBA65 one (vectors_stm32wba65.c), so an NMI the emulator raises for a
flash double ECC error runs the port's NMI_Handler; BondFwNmiTrap is where
the emulator returns from it. Boots 6 to 8
are the power cut sweep of run_bond_flash.py --powercut: prepare the old
image, the save the emulator cuts, and the boot after the cut. Boots 9 to 12
are the bond deletion of BtSmpBondErase: the firmware plays the ST stack
clearing its table between BtSmpBondWbaEraseBegin and BtSmpBondWbaEraseEnd.
Boots 13 and 14 are the power cut sweep of a deletion. Boot 15 runs the flash
driver alone through an Nvm over the region: programs the controller refuses
must fail the write.

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
#include "storage/nvm.h"
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"

// Image of the stack's NVM cache: 39 words, 312 bytes, two PDS chunks
#define BOND_FW_WORDS		39U
#define BOND_FW_CACHE_WORDS	80U

extern "C" uint16_t BtSmpBondWbaCacheInit(uint64_t *pCache, uint16_t MaxWords);
extern "C" size_t BtSmpBondWbaRestore(void *pCacheBuf, size_t Capacity);
extern "C" void BLEPLAT_NvmStore(const uint64_t *ptr, uint16_t size);
extern "C" void BtSmpBondWbaFlush(void);
extern "C" void BtSmpBondWbaPoll(void);
extern "C" void BtSmpBondWbaEraseBegin(void);
extern "C" void BtSmpBondWbaEraseEnd(bool bCleared);

extern "C" void BondFwConsole(const char *pStr, int Len) __attribute__((noinline));
extern "C" void BondFwProtect(int Mode) __attribute__((noinline));
extern "C" void BondFwDone(void) __attribute__((noinline));
extern "C" void BondFwNmiTrap(void) __attribute__((noinline));

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

// Return address the emulator gives the NMI it raises. Nothing calls it, so
// the pointer below keeps it in the link.
extern "C" void BondFwNmiTrap(void)
{
	__asm volatile ("" ::: "memory");
}

extern "C" void (* volatile g_pBondFwNmiTrap)(void);
void (* volatile g_pBondFwNmiTrap)(void) = BondFwNmiTrap;

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

// Bond deletion as BtSmpBondErase does it. Words is the size of the save
// request the stack makes while it clears its table, 0 for none. bCleared is
// what aci_gap_clear_security_db reports.
static void Erase(bool bCleared, uint32_t Seed = 0, uint32_t Words = 0)
{
	BtSmpBondWbaEraseBegin();
	if (bCleared)
	{
		memset(s_Cache, 0, sizeof(s_Cache));
		if (Words != 0)
		{
			Fill(Seed, Words);
			BLEPLAT_NvmStore(s_Cache, (uint16_t)Words);
		}
	}
	BtSmpBondWbaEraseEnd(bCleared);
}

// Deletion with no save request from the stack
static void BootErase0(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == 0, "delete: erased store");
	Save(31);
	Erase(true);
	Check(StoredIs(31), "delete: the image stays until the application loop runs");
	BtSmpBondWbaFlush();
	Check(NothingStored(), "delete: the stored image is erased");
}

// The deletion holds across a reset. A save request still waiting when the
// table is cleared names the old cache and is dropped; one the stack makes
// while clearing is saved after the erase.
static void BootErase1(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == 0,
		  "delete: nothing restored after reset");

	Save(32);
	Fill(33);
	BLEPLAT_NvmStore(s_Cache, BOND_FW_WORDS);
	Erase(true);
	BtSmpBondWbaFlush();
	Check(NothingStored(), "delete: a save request made before the clear is dropped");

	Save(34);
	Erase(true, 35, 20);
	BtSmpBondWbaFlush();
	Check(StoredIs(35, 20), "delete: the save request of the clear is stored after the erase");
}

// A refused clear keeps everything. A medium refusing the erase is retried
// until it takes it, then the store takes the next save.
static void BootErase2(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == 20 && CacheIs(35, 20),
		  "delete: the save after the erase restored after reset");

	Save(36);
	Fill(37);
	BLEPLAT_NvmStore(s_Cache, BOND_FW_WORDS);
	Erase(false);
	BtSmpBondWbaFlush();
	Check(StoredIs(37), "delete: a refused clear keeps the image and the waiting save");

	BondFwProtect(0);
	Erase(true);
	BtSmpBondWbaFlush();
	Seconds(3);
	Check(StoredIs(37), "delete: the image stays while the medium refuses the erase");
	BondFwProtect(-1);
	Seconds(5);
	Check(NothingStored(), "delete: the erase is retried until the medium takes it");

	Save(38);
	Check(StoredIs(38), "delete: a save after the deletion is stored");
}

static void BootErase3(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == BOND_FW_WORDS && CacheIs(38),
		  "delete: the save after the retried erase restored after reset");
}

// Power cut sweep of a deletion, after BootCutPrepare: the deletion the
// emulator cuts, then the boot after the cut.
static void BootCutErase(void)
{
	Check(BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS) == g_BondFwArg[1] &&
		  CacheIs(g_BondFwArg[0], g_BondFwArg[1]), "erase cut: old image restored before the deletion");
	Erase(true);
	BtSmpBondWbaFlush();
	Check(NothingStored(), "erase cut: deletion completed");
}

// The old image or nothing, never anything else, and the store takes the
// next save
static void BootCutEraseCheck(void)
{
	uint16_t n = BtSmpBondWbaCacheInit(s_Cache, BOND_FW_CACHE_WORDS);
	bool isOld = n == g_BondFwArg[1] && CacheIs(g_BondFwArg[0], g_BondFwArg[1]);
	bool isNone = n == 0 && s_Cache[0] == 0;

	Print("erase cut: restored %s\r\n", isOld ? "old" : (isNone ? "none" : "neither"));
	Check(isOld || isNone, "after a power cut in a deletion the restore is the old image or nothing");
	Save(g_BondFwArg[2], g_BondFwArg[3]);
	Check(StoredIs(g_BondFwArg[2], g_BondFwArg[3]), "save after the power cut in a deletion stored");
}

// The flash driver alone, through an Nvm over the store region: a program
// the controller refuses (PROGERR, the quad word is not erased) must fail
// the write, for a full 64 byte block as for a shorter one.
static NvmIntrf s_TestIntrf;
static Nvm s_TestMem;

static bool TestMemInit(void)
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	NvmMcuCfg(cfg);
	cfg.BaseAddr = NvmRegionAddr(0);
	cfg.TotalSize = NvmRegionSize(0);

	return s_TestIntrf.Init() && s_TestMem.Init(cfg, &s_TestIntrf, 0, 0);
}

static void BootProgramError(void)
{
	uint8_t a[64];
	uint8_t b[64];
	uint8_t c[64];

	for (int i = 0; i < 64; i++)
	{
		a[i] = (uint8_t)(i + 1);
		b[i] = (uint8_t)(0x80 + i);
	}

	Check(TestMemInit(), "program error: Nvm over the region");
	Check(s_TestMem.Erase(0, 0x2000) == 0, "program error: page erased");
	Check(s_TestMem.Write(0x40, a, 64) == 64, "program error: 64 bytes written");
	Check(s_TestMem.Write(0x40, b, 64) < 0, "program error: 64 bytes over data fail");
	Check(s_TestMem.Write(0x80, a, 48) == 48, "program error: 48 bytes written");
	Check(s_TestMem.Write(0x80, b, 48) < 0, "program error: 48 bytes over data fail");

	// Only the last quad word of the block holds data: the first three go
	// in, the fourth is refused, and the write fails.
	Check(s_TestMem.Write(0xF0, a, 16) == 16, "program error: one quad word written");
	Check(s_TestMem.Write(0xC0, b, 64) < 0, "program error: block ending on data fails");

	// A write after a failed one lands where it is asked to
	Check(s_TestMem.Write(0x100, b, 64) == 64 && s_TestMem.Read(0x100, c, 64) == 64 &&
		  memcmp(c, b, 64) == 0, "program error: next write lands at its own address");
	Check(s_TestMem.Read(0x40, c, 64) == 64 && memcmp(c, a, 64) == 0,
		  "program error: the refused write left the data");
}

// Raw reads must reject a double error even when another correction masks
// both attempts. A genuine corrected word must remain readable. Argument 0:
// the read is to be refused; argument 1: where in the region it starts.
static void BootEccRetry(void)
{
	uint8_t data[64];
	uint32_t off = g_BondFwArg[1];
	Check(TestMemInit(), "ECC retry: Nvm over the region");
	int count = s_TestMem.Read(off, data, sizeof(data));
	if (g_BondFwArg[0])
	{
		Check(count < 0, "ECC retry: uncertain read refused");
	}
	else
	{
		Check(count == (int)sizeof(data), "ECC retry: valid corrected data read");
		bool same = true;
		for (unsigned i = 0; i < sizeof(data); i++)
		{
			same = same && data[i] == 0xA5;
		}
		Check(same, "ECC retry: data preserved");
	}
	Check(s_TestMem.Read(off + 64, data, sizeof(data)) == (int)sizeof(data),
		  "ECC retry: next read remains usable");
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
		BootCutPrepare, BootCutSave, BootCutCheck,
		BootErase0, BootErase1, BootErase2, BootErase3,
		BootCutErase, BootCutEraseCheck, BootProgramError, BootEccRetry
	};
	uint32_t boot = g_BondFwBoot;

	(void)g_pBondFwNmiTrap;

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
extern "C" uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern "C" void (*__init_array_start[])(void);
extern "C" void (*__init_array_end[])(void);

extern "C" void ResetEntry(void)
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
