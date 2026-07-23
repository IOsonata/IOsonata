/**-------------------------------------------------------------------------
@example	nvm_mcu_test.cpp

@brief	Host test for the NvmMcu driver on the NvmIO base.

A mock target memory controller backs the driver with a RAM array placed at a
real address, so the memory mapped read path is exercised as it is on the
device. The mock enforces the internal memory rules: word aligned programming,
clear only bit semantics, a page erase that restores all ones, and the word
count limit a stack may place on one program request. A protect flag lets the
write protect path be checked.

Build and run on the host:

  g++ -std=gnu++23 -O1 -I include -I include/storage -I Linux/include \
	  exemples/storage/nvm_mcu_test.cpp src/storage/nvm_mcu.cpp \
	  src/device.cpp src/device_intrf.cpp -o nvm_mcu_test
  ./nvm_mcu_test

@author	Hoang Nguyen Hoan
@date	Jul. 22, 2026

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

#include "storage/nvm_mcu.h"

// ---------------------------------------------------------------------------
// Test bookkeeping
// ---------------------------------------------------------------------------
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
// Mock internal memory and its controller operations
// ---------------------------------------------------------------------------
#define MEM_SIZE	(16u * 1024u)	// 16 KB
#define MEM_PAGE	4096u			// 4 KB erase page
#define MEM_GRAN	4u				// word program
#define MEM_MAXW	64u				// word limit of one program request

// Page aligned so the mock sees the same alignment a device region has.
alignas(MEM_PAGE) static uint32_t s_Mem[MEM_SIZE / 4];
static int s_ProgCalls;
static int s_EraseCalls;
static int s_VioOverLimit;
static int s_VioUnaligned;
static bool s_Protected;

static void MockPowerOn(void)
{
	memset(s_Mem, 0xFF, sizeof(s_Mem));
	s_ProgCalls = 0;
	s_EraseCalls = 0;
	s_VioOverLimit = 0;
	s_VioUnaligned = 0;
	s_Protected = false;
}

static uintptr_t MemBase(void) { return (uintptr_t)s_Mem; }

static int MockProgram(uintptr_t Addr, const uint32_t *pSrc, uint32_t WordCnt,
					   void *pCtx)
{
	(void)pCtx;
	s_ProgCalls++;

	if ((Addr % MEM_GRAN) != 0) { s_VioUnaligned++; return -EINVAL; }
	if (WordCnt > MEM_MAXW) { s_VioOverLimit++; return -EINVAL; }
	if (s_Protected) { return -EACCES; }

	// Programming clears bits only; a set bit needs an erase first.
	uint32_t *p = (uint32_t *)(uintptr_t)Addr;
	for (uint32_t i = 0; i < WordCnt; i++)
	{
		p[i] &= pSrc[i];
	}

	return 0;
}

static int MockErasePage(uintptr_t Addr, void *pCtx)
{
	(void)pCtx;
	s_EraseCalls++;

	if ((Addr % MEM_PAGE) != 0) { s_VioUnaligned++; return -EINVAL; }
	if (s_Protected) { return -EACCES; }

	memset((void *)(uintptr_t)Addr, 0xFF, MEM_PAGE);

	return 0;
}

static int MockSetProtect(uintptr_t Addr, uint32_t Len, bool bEnable, void *pCtx)
{
	(void)Addr; (void)Len; (void)pCtx;
	s_Protected = bEnable;
	return 0;
}

static NvmCfg_t MakeCfg(void)
{
	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.BaseAddr = MemBase();
	cfg.TotalSize = MEM_SIZE;
	cfg.EraseSize = MEM_PAGE;
	cfg.WriteGran = MEM_GRAN;
	return cfg;
}

static NvmMcuOp_t MakeOp(bool bWithProtect)
{
	NvmMcuOp_t op;
	memset(&op, 0, sizeof(op));
	op.Program = MockProgram;
	op.ErasePage = MockErasePage;
	op.SetProtect = bWithProtect ? MockSetProtect : nullptr;
	op.MaxProgWords = MEM_MAXW;
	op.pCtx = nullptr;
	return op;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
static void TestInit(void)
{
	MockPowerOn();
	NvmMcu mem;
	CHECK(mem.Init(MakeCfg(), MakeOp(false)), "init");
	CHECK(mem.Size() == MEM_SIZE, "size");
	CHECK(mem.EraseSize() == MEM_PAGE, "erase size is the page");
	CHECK(mem.WriteGran() == MEM_GRAN, "write gran");
	CHECK(mem.PageSize() == 0, "mapped memory has no auto increment window");
	CHECK(mem.LogicalSectorSize() == MEM_PAGE, "logical sector follows erase");

	// The program and erase operations are required.
	NvmMcuOp_t bad = MakeOp(false);
	bad.Program = nullptr;
	NvmMcu m2;
	CHECK(m2.Init(MakeCfg(), bad) == false, "missing program rejected");

	// The page must be a whole number of write units.
	NvmCfg_t badcfg = MakeCfg();
	badcfg.WriteGran = 3;
	NvmMcu m3;
	CHECK(m3.Init(badcfg, MakeOp(false)) == false, "page not a multiple of gran");
}

static void TestWriteRead(void)
{
	MockPowerOn();
	NvmMcu mem;
	mem.Init(MakeCfg(), MakeOp(false));

	// A request larger than the program word limit must be split.
	uint32_t len = (MEM_MAXW * MEM_GRAN) + (8 * MEM_GRAN);
	uint8_t out[(MEM_MAXW + 8) * MEM_GRAN], in[(MEM_MAXW + 8) * MEM_GRAN];
	for (uint32_t i = 0; i < len; i++) { out[i] = (uint8_t)(i * 5 + 1); }
	memset(in, 0, sizeof(in));

	CHECK(mem.Write(0, out, len) == (int)len, "write larger than the limit");
	CHECK(s_ProgCalls == 2, "split into two program calls, got %d", s_ProgCalls);
	CHECK(s_VioOverLimit == 0, "no request exceeded the word limit");
	CHECK(mem.Read(0, in, len) == (int)len, "read back");
	CHECK(memcmp(out, in, len) == 0, "round trip");
}

static void TestProgramSemantics(void)
{
	MockPowerOn();
	NvmMcu mem;
	mem.Init(MakeCfg(), MakeOp(false));

	uint32_t all = 0xFFFFFFFFu;
	uint32_t val = 0x0F0F0F0Fu;
	uint32_t rd = 0;

	CHECK(mem.Write(0, &val, 4) == 4, "program a word");
	mem.Read(0, &rd, 4);
	CHECK(rd == val, "word programmed");

	// Programming cannot set a cleared bit back.
	CHECK(mem.Write(0, &all, 4) == 4, "program all ones over it");
	mem.Read(0, &rd, 4);
	CHECK(rd == val, "clear only, bits do not come back without erase");

	// After an erase the word reads all ones again.
	CHECK(mem.Erase(0, MEM_PAGE) == 0, "erase page");
	mem.Read(0, &rd, 4);
	CHECK(rd == 0xFFFFFFFFu, "erased word reads all ones");
	CHECK(s_EraseCalls == 1, "one page erased");
}

static void TestAlignment(void)
{
	MockPowerOn();
	NvmMcu mem;
	mem.Init(MakeCfg(), MakeOp(false));

	uint8_t buf[8];
	memset(buf, 0, sizeof(buf));

	CHECK(mem.Write(1, buf, 4) == -EINVAL, "unaligned write offset");
	CHECK(mem.Write(0, buf, 3) == -EINVAL, "unaligned write length");
	CHECK(mem.Erase(1, MEM_PAGE) == -EINVAL, "unaligned erase offset");
	CHECK(mem.Erase(0, 100) == -EINVAL, "unaligned erase length");
	CHECK(mem.Erase(0, 0) == -EINVAL, "zero length erase");
	CHECK(mem.Read(MEM_SIZE, buf, 4) == -EINVAL, "read past end");
	CHECK(mem.Write(MEM_SIZE, buf, 4) == -EINVAL, "write past end");
	CHECK(s_VioUnaligned == 0, "no unaligned request reached the controller");
}

static void TestWriteProtect(void)
{
	MockPowerOn();
	NvmMcu mem;
	mem.Init(MakeCfg(), MakeOp(true));

	uint32_t val = 0x12345678u;
	CHECK(mem.SetWriteProtect(0, MEM_PAGE, true) == 0, "protect");
	CHECK(mem.Write(0, &val, 4) == -EACCES, "write refused while protected");
	CHECK(mem.SetWriteProtect(0, MEM_PAGE, false) == 0, "unprotect");
	CHECK(mem.Write(0, &val, 4) == 4, "write allowed after unprotect");

	// A target with no region protect reports the operation unsupported.
	MockPowerOn();
	NvmMcu noProt;
	noProt.Init(MakeCfg(), MakeOp(false));
	CHECK(noProt.SetWriteProtect(0, MEM_PAGE, true) == -ENOTSUP,
		  "no protect operation, not supported");
}

static void TestRegionWindow(void)
{
	MockPowerOn();
	NvmMcu win;
	// A window covering the second page only.
	CHECK(win.Init(MakeCfg(), MakeOp(false), MEM_PAGE, MEM_PAGE), "init window");
	CHECK(win.Size() == MEM_PAGE, "window size");
	CHECK(win.RegionOffset() == MEM_PAGE, "window offset");

	uint32_t val = 0x0BADF00Du;
	uint32_t rd = 0;
	CHECK(win.Write(0, &val, 4) == 4, "write at window start");
	win.Read(0, &rd, 4);
	CHECK(rd == val, "read back through the window");

	// The write landed in the second page, not the first.
	CHECK(s_Mem[MEM_PAGE / 4] == val, "data placed at the window offset");
	CHECK(s_Mem[0] == 0xFFFFFFFFu, "first page untouched");

	// The window bounds the range.
	CHECK(win.Write(MEM_PAGE, &val, 4) == -EINVAL, "write past the window");
}

int main(void)
{
	TestInit();
	TestWriteRead();
	TestProgramSemantics();
	TestAlignment();
	TestWriteProtect();
	TestRegionWindow();

	printf("\nChecks run: %d\n", g_Checks);
	if (g_Fail == 0)
	{
		printf("RESULT: ALL PASS\n");
		return 0;
	}
	printf("RESULT: %d FAILURES\n", g_Fail);
	return 1;
}
