/**-------------------------------------------------------------------------
@example	nvm_flash_test.cpp

@brief	Host test harness for the NvmFlash device and FlashDiskIO adapter.

Builds and runs on a PC, not on a target board. A mock SPI NOR Flash is
implemented behind a C style DevIntrf_t: it parses the command protocol
(JEDEC id, status, write enable latch, page program, read, sector erase,
chip erase, reset, deep power down) over a RAM array with NOR semantics
(programming only clears bits). The mock records every program operation
and counts protocol violations, so the test verifies the driver observes
the NvmIO charter, not only that data round trips:

  - page program chunks never cross a page boundary and are issued in
    ascending address order
  - the write enable latch is set before every program and erase
  - no operation is issued while write is in progress; the driver polls
    status until ready
  - region windowing maps region relative offsets to the configured
    device area and rejects out of range and misaligned accesses
  - MassErase is refused on a windowed instance
  - Enable/Disable issue release/enter deep power down

The DeviceIntrf wrapper class implements only the conversion operator and
Rate, relying on the DeviceIntrf default transaction methods, so the test
also exercises those defaults over the C helper busy protocol.

Build and run on the host:

  g++ -std=gnu++23 -O1 -I include -I include/storage -I Linux/include \
      exemples/storage/nvm_flash_test.cpp src/storage/nvm_flash.cpp \
      src/storage/diskio_flash.cpp src/storage/diskio_impl.cpp \
      src/device.cpp src/device_intrf.cpp -o nvm_flash_test
  ./nvm_flash_test

The harness prints RESULT: ALL PASS on success and a FAIL line per
violated check otherwise. This file is host-only and is not referenced
by any board project.

@author	Hoang Nguyen Hoan
@date	Jul 19, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cerrno>
#include <vector>

#include "device_intrf.h"
#include "coredev/spi.h"
#include "storage/nvm_flash.h"
#include "storage/diskio_flash.h"

#define NOR_SIZE		(1024U * 1024U)
#define NOR_PAGE		256U
#define NOR_SECT		4096U
#define NOR_ID			0x1628C2U	// MX25R style, 3 bytes
#define NOR_ID_LEN		3

// *** Mock NOR state ***
static uint8_t s_Mem[NOR_SIZE];
static bool s_Wel;
static int s_WipPolls;			// remaining status reads reporting WIP
static int s_BusyPolls = 0;		// WIP duration per program/erase
static bool s_Dpd;
static bool s_RejectWren = false;
static int s_EnableCnt, s_DisableCnt;

// Protocol violation counters
static int s_VioNoWel;			// program/erase without write enable latch
static int s_VioWhileBusy;		// command while write in progress
static int s_VioPageCross;		// program chunk crossing a page boundary

// Program operation log
struct ProgOp { uint32_t Addr; uint32_t Len; };
static std::vector<ProgOp> s_ProgLog;
static std::vector<uint8_t> s_CmdLog;

// Per transaction state
static uint8_t s_TxBuf[16];
static int s_TxLen;
static bool s_InRx;
static uint32_t s_RdAddr;
static int s_RxServed;
static std::vector<uint8_t> s_PayBuf;
static int s_MaxChunk = 65536;	// mock transfer chunk bound

static uint32_t MockAddr(const uint8_t *p, int AddrLen)
{
	uint32_t a = 0;
	for (int i = 0; i < AddrLen; i++)
	{
		a = (a << 8) | p[i];
	}
	return a;
}

static bool MockBusy(void)
{
	return s_WipPolls > 0;
}

static void MockExecTx(const uint8_t *pCmd, int CmdLen,
					   const uint8_t *pData, int DataLen)
{
	uint8_t cmd = pCmd[0];
	s_CmdLog.push_back(cmd);

	switch (cmd)
	{
		case FLASH_CMD_WRENABLE:
			if (MockBusy()) { s_VioWhileBusy++; break; }
			if (!s_RejectWren) s_Wel = true;
			break;
		case FLASH_CMD_WRDISABLE:
			s_Wel = false;
			break;
		case FLASH_CMD_WRITE:
		{
			if (MockBusy()) { s_VioWhileBusy++; break; }
			if (!s_Wel) { s_VioNoWel++; break; }
			uint32_t a = MockAddr(pCmd + 1, CmdLen - 1);
			if ((a / NOR_PAGE) != ((a + DataLen - 1) / NOR_PAGE))
			{
				s_VioPageCross++;
			}
			for (int i = 0; i < DataLen; i++)
			{
				s_Mem[(a + i) % NOR_SIZE] &= pData[i];
			}
			s_ProgLog.push_back({a, (uint32_t)DataLen});
			s_Wel = false;
			s_WipPolls = s_BusyPolls;
			break;
		}
		case FLASH_CMD_SECTOR_ERASE:
		{
			if (MockBusy()) { s_VioWhileBusy++; break; }
			if (!s_Wel) { s_VioNoWel++; break; }
			uint32_t a = MockAddr(pCmd + 1, CmdLen - 1) & ~(NOR_SECT - 1);
			memset(&s_Mem[a % NOR_SIZE], 0xFF, NOR_SECT);
			s_Wel = false;
			s_WipPolls = s_BusyPolls;
			break;
		}
		case FLASH_CMD_BULK_ERASE:
			if (MockBusy()) { s_VioWhileBusy++; break; }
			if (!s_Wel) { s_VioNoWel++; break; }
			memset(s_Mem, 0xFF, NOR_SIZE);
			s_Wel = false;
			s_WipPolls = s_BusyPolls;
			break;
		case FLASH_CMD_DPD:
			s_Dpd = true;
			break;
		case FLASH_CMD_RDPD:
			s_Dpd = false;
			break;
		case FLASH_CMD_RESET_ENABLE:
		case FLASH_CMD_RESET_DEVICE:
		case FLASH_CMD_EN4B:
			break;
		default:
			break;
	}
}

// *** DevIntrf_t function pointers ***

static void MockDisable(DevIntrf_t * const pDev) { s_DisableCnt++; }
static void MockEnable(DevIntrf_t * const pDev) { s_EnableCnt++; }
static uint32_t MockGetRate(DevIntrf_t * const pDev) { return 8000000; }
static uint32_t MockSetRate(DevIntrf_t * const pDev, uint32_t Rate) { return Rate; }
static void MockPowerOff(DevIntrf_t * const pDev) {}
static void *MockGetHandle(DevIntrf_t * const pDev) { return nullptr; }

static bool MockStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	// Restart condition: a command sent in a preceding no-stop Tx phase
	// (DeviceIntrfRead composite) stays valid for the read phase.
	s_InRx = true;
	s_RxServed = 0;
	return true;
}

static bool MockStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	s_TxLen = 0;
	s_InRx = false;
	s_PayBuf.clear();
	return true;
}

static int MockTxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	int l = DataLen < s_MaxChunk ? DataLen : s_MaxChunk;

	if (s_InRx)
	{
		// Command and address phase of a read transaction
		for (int i = 0; i < l && s_TxLen < (int)sizeof(s_TxBuf); i++)
		{
			s_TxBuf[s_TxLen++] = pData[i];
		}
		return l;
	}

	if (s_TxLen == 0)
	{
		// First TxData of a write transaction: command and address
		for (int i = 0; i < DataLen && s_TxLen < (int)sizeof(s_TxBuf); i++)
		{
			s_TxBuf[s_TxLen++] = pData[i];
		}
		// Commands without payload execute at StopTx
		return DataLen;
	}

	// Payload phase: page program data accumulates until the transaction
	// ends; the device programs the whole stream at once.
	s_PayBuf.insert(s_PayBuf.end(), pData, pData + l);
	return l;
}

static void MockStopTx(DevIntrf_t * const pDev)
{
	if (s_TxLen > 0)
	{
		uint8_t cmd = s_TxBuf[0];
		if (cmd == FLASH_CMD_WRITE)
		{
			MockExecTx(s_TxBuf, s_TxLen, s_PayBuf.data(),
					   (int)s_PayBuf.size());
		}
		else
		{
			MockExecTx(s_TxBuf, s_TxLen, nullptr, 0);
		}
	}
	s_TxLen = 0;
	s_PayBuf.clear();
}

static int MockRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	int l = BuffLen < s_MaxChunk ? BuffLen : s_MaxChunk;
	uint8_t cmd = s_TxLen > 0 ? s_TxBuf[0] : 0;

	if (s_RxServed == 0)
	{
		s_CmdLog.push_back(cmd);
	}

	switch (cmd)
	{
		case FLASH_CMD_READSTATUS:
		{
			uint8_t st = 0;
			if (s_WipPolls > 0)
			{
				st |= FLASH_STATUS_WIP;
				s_WipPolls--;
			}
			if (s_Wel)
			{
				st |= FLASH_STATUS_WEL;
			}
			pBuff[0] = st;
			return 1;
		}
		case FLASH_CMD_READID:
		{
			uint32_t id = NOR_ID;
			for (int i = 0; i < l && i < 4; i++)
			{
				pBuff[i] = (id >> (8 * i)) & 0xFF;
			}
			return l < 4 ? l : 4;
		}
		case FLASH_CMD_READ:
		{
			if (s_RxServed == 0)
			{
				s_RdAddr = MockAddr(&s_TxBuf[1], s_TxLen - 1);
				if (MockBusy())
				{
					s_VioWhileBusy++;
				}
			}
			for (int i = 0; i < l; i++)
			{
				pBuff[i] = s_Mem[(s_RdAddr + i) % NOR_SIZE];
			}
			s_RdAddr += l;
			s_RxServed += l;
			return l;
		}
		default:
			memset(pBuff, 0, l);
			return l;
	}
}

static void MockStopRx(DevIntrf_t * const pDev)
{
	s_TxLen = 0;
	s_InRx = false;
}

static int MockTxSrData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	// Command and address phase of a composite read (no stop follows)
	s_TxLen = 0;
	for (int i = 0; i < DataLen && s_TxLen < (int)sizeof(s_TxBuf); i++)
	{
		s_TxBuf[s_TxLen++] = pData[i];
	}
	return DataLen;
}

static void MockReset(DevIntrf_t * const pDev) {}

/// DeviceIntrf wrapper implementing only the conversion operator and Rate:
/// the six transaction methods come from the DeviceIntrf defaults.
class MockSpi : public DeviceIntrf {
public:
	MockSpi()
	{
		memset(&vDev, 0, sizeof(vDev));
		vDev.pDevData = this;
		vDev.Type = DEVINTRF_TYPE_SPI;
		vDev.Disable = MockDisable;
		vDev.Enable = MockEnable;
		vDev.GetRate = MockGetRate;
		vDev.SetRate = MockSetRate;
		vDev.StartRx = MockStartRx;
		vDev.RxData = MockRxData;
		vDev.StopRx = MockStopRx;
		vDev.StartTx = MockStartTx;
		vDev.TxData = MockTxData;
		vDev.TxSrData = MockTxSrData;
		vDev.StopTx = MockStopTx;
		vDev.PowerOff = MockPowerOff;
		vDev.GetHandle = MockGetHandle;
		vDev.MaxRetry = 5;
		vDev.MaxTrxLen = 65536;
		vDev.EnCnt = 1;
		atomic_flag_clear(&vDev.bBusy);
	}
	operator DevIntrf_t * const () { return &vDev; }
	uint32_t Rate(uint32_t RateHz) { return RateHz; }
	uint32_t Rate(void) { return 8000000; }

	DevIntrf_t vDev;
};

static void MockPowerOn(void)
{
	memset(s_Mem, 0xFF, sizeof(s_Mem));
	s_Wel = false;
	s_WipPolls = 0;
	s_BusyPolls = 0;
	s_Dpd = false;
	s_RejectWren = false;
	s_VioNoWel = s_VioWhileBusy = s_VioPageCross = 0;
	s_EnableCnt = s_DisableCnt = 0;
	s_ProgLog.clear();
	s_CmdLog.clear();
	s_TxLen = 0;
	s_InRx = false;
	s_PayBuf.clear();
	s_MaxChunk = 65536;
}

// The Quad SPI command helpers are implemented per target. The mock is a
// plain SPI interface, so the QSPI branches of the driver never run on the
// host; these stubs only satisfy the linker.
extern "C" bool QuadSPISendCmd(SPIDev_t * const pDev, uint8_t Cmd,
							   uint32_t Addr, uint8_t AddrLen,
							   uint32_t DataLen, uint8_t DummyCycle)
{
	return false;
}

extern "C" void QuadSPISetMemSize(SPIDev_t * const pDev, uint32_t Size)
{
}

static int g_Fail;

#define CHECK(cond, ...) do { if (!(cond)) { g_Fail++; \
	printf("FAIL %s:%d: ", __func__, __LINE__); printf(__VA_ARGS__); \
	printf("\n"); } } while (0)

static const FlashCfg_t s_Cfg = {
	.DevNo = 0,
	.TotalSize = NOR_SIZE / 1024,
	.BlkSize = 32 * 1024,
	.SectSize = NOR_SECT,
	.PageSize = NOR_PAGE,
	.AddrSize = 3,
	.DevId = NOR_ID,
	.DevIdSize = NOR_ID_LEN,
	.pInitCB = nullptr,
	.pWaitCB = nullptr,
};

static void Fill(uint8_t *p, uint32_t Len, uint32_t Seed)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		p[i] = (uint8_t)(Seed * 31U + i * 7U);
	}
}

static void TestInitProbe(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(s_Cfg, &Spi), "init");
	CHECK(flash.Size() == NOR_SIZE, "size %u", (unsigned)flash.Size());
	CHECK(flash.EraseSize() == NOR_SECT, "erase size");
	CHECK(flash.PageSize() == NOR_PAGE, "page size");
	CHECK(flash.WriteGran() == 1, "write gran");

	FlashCfg_t bad = s_Cfg;
	bad.DevId = 0x123456;
	NvmFlash f2;
	CHECK(f2.Init(bad, &Spi) == false, "wrong id must fail");
}

static void TestWriteRead(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(s_Cfg, &Spi), "init");

	// 700 bytes from an odd offset spanning multiple pages
	uint8_t wr[700], rd[700];
	Fill(wr, sizeof(wr), 3);
	s_ProgLog.clear();
	CHECK(flash.Write(101, wr, sizeof(wr)) == (int)sizeof(wr), "write");
	CHECK(s_VioNoWel == 0 && s_VioPageCross == 0 && s_VioWhileBusy == 0,
		  "violations %d %d %d", s_VioNoWel, s_VioPageCross, s_VioWhileBusy);
	// Ascending page chunks within page bounds
	uint32_t expect = 101;
	for (const auto &op : s_ProgLog)
	{
		CHECK(op.Addr == expect, "prog addr %u expect %u", op.Addr, expect);
		CHECK(op.Len <= NOR_PAGE, "prog len %u", op.Len);
		CHECK((op.Addr / NOR_PAGE) == ((op.Addr + op.Len - 1) / NOR_PAGE),
			  "prog chunk crosses page");
		expect += op.Len;
	}
	CHECK(expect == 101 + sizeof(wr), "prog coverage %u", expect);

	CHECK(flash.Read(101, rd, sizeof(rd)) == (int)sizeof(rd), "read");
	CHECK(memcmp(wr, rd, sizeof(wr)) == 0, "readback");

	// Single odd byte
	uint8_t b = 0x5A;
	CHECK(flash.Write(4093, &b, 1) == 1, "odd write");
	uint8_t rb = 0;
	CHECK(flash.Read(4093, &rb, 1) == 1 && rb == 0x5A, "odd readback");

	// Bounds
	CHECK(flash.Read(NOR_SIZE, rd, 1) == -EINVAL, "read past end");
	CHECK(flash.Write(NOR_SIZE - 1, wr, 2) == -EINVAL, "write crossing end");
	CHECK(flash.Read(0, nullptr, 1) == -EINVAL, "null buffer");
}

static void TestErase(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(s_Cfg, &Spi), "init");

	uint8_t wr[256], rd[256];
	Fill(wr, sizeof(wr), 9);
	CHECK(flash.Write(NOR_SECT, wr, sizeof(wr)) == (int)sizeof(wr), "write");
	CHECK(flash.Erase(NOR_SECT, NOR_SECT) == 0, "erase");
	CHECK(flash.Read(NOR_SECT, rd, sizeof(rd)) == (int)sizeof(rd), "read");
	for (unsigned i = 0; i < sizeof(rd); i++)
	{
		if (rd[i] != 0xFF) { CHECK(false, "not erased at %u", i); break; }
	}
	CHECK(s_VioNoWel == 0, "erase without wel");

	CHECK(flash.Erase(100, NOR_SECT) == -EINVAL, "misaligned offset");
	CHECK(flash.Erase(0, 100) == -EINVAL, "misaligned length");
	CHECK(flash.Erase(NOR_SIZE - NOR_SECT, 2 * NOR_SECT) == -EINVAL,
		  "erase past end");
	CHECK(flash.Erase(0, 0) == -EINVAL, "zero length");
}

static void TestRegion(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	// 64KB window at 64KB
	CHECK(flash.Init(s_Cfg, &Spi, 0x10000, 0x10000), "init region");
	CHECK(flash.Size() == 0x10000, "region size");
	CHECK(flash.RegionOffset() == 0x10000, "region offset");

	uint8_t wr[64], rd[64];
	Fill(wr, sizeof(wr), 5);
	CHECK(flash.Write(0, wr, sizeof(wr)) == (int)sizeof(wr), "write");
	CHECK(memcmp(&s_Mem[0x10000], wr, sizeof(wr)) == 0,
		  "physical placement");
	CHECK(flash.Read(0, rd, sizeof(rd)) == (int)sizeof(rd) &&
		  memcmp(wr, rd, sizeof(rd)) == 0, "readback");

	CHECK(flash.Read(0x10000, rd, 1) == -EINVAL, "read past region");
	CHECK(flash.Write(0x10000 - 1, wr, 2) == -EINVAL, "write past region");
	CHECK(flash.Erase(0, 0x10000) == 0, "erase full region");
	CHECK(flash.MassErase() == -EPERM, "mass erase on window");

	// Whole device instance may mass erase
	NvmFlash whole;
	CHECK(whole.Init(s_Cfg, &Spi), "init whole");
	uint8_t b = 0x11;
	CHECK(whole.Write(0, &b, 1) == 1, "seed");
	CHECK(whole.MassErase() == 0, "mass erase");
	CHECK(s_Mem[0] == 0xFF, "chip erased");
	bool bulkSeen = false;
	for (uint8_t c : s_CmdLog) if (c == FLASH_CMD_BULK_ERASE) bulkSeen = true;
	CHECK(bulkSeen, "bulk erase command");

	// Misaligned region rejected
	NvmFlash f3;
	CHECK(f3.Init(s_Cfg, &Spi, 100, NOR_SECT) == false, "misaligned region");
}

static void TestBusyAndWel(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(s_Cfg, &Spi), "init");

	// Device stays busy for several status polls after each operation
	s_BusyPolls = 5;
	uint8_t wr[300], rd[300];
	Fill(wr, sizeof(wr), 7);
	CHECK(flash.Write(0, wr, sizeof(wr)) == (int)sizeof(wr), "write busy");
	CHECK(flash.Write(NOR_SECT * 2, wr, sizeof(wr)) == (int)sizeof(wr),
		  "write busy 2");
	CHECK(flash.Erase(0, NOR_SECT) == 0, "erase busy");
	CHECK(s_VioWhileBusy == 0, "%d ops while busy", s_VioWhileBusy);
	CHECK(s_VioNoWel == 0, "%d ops without wel", s_VioNoWel);
	CHECK(flash.Read(NOR_SECT * 2, rd, sizeof(rd)) == (int)sizeof(rd) &&
		  memcmp(wr, rd, sizeof(rd)) == 0, "readback after busy");

	// Write enable rejected: the driver must report failure, not program
	s_BusyPolls = 0;
	s_RejectWren = true;
	s_ProgLog.clear();
	CHECK(flash.Write(0, wr, 16) == -EBUSY, "wren rejected reports error");
	CHECK(s_ProgLog.empty(), "no program without wel");
	s_RejectWren = false;
}

static void TestChunkedTransfer(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(s_Cfg, &Spi), "init");

	// Interface serves at most 13 bytes per data call
	s_MaxChunk = 13;
	uint8_t wr[500], rd[500];
	Fill(wr, sizeof(wr), 11);
	CHECK(flash.Write(50, wr, sizeof(wr)) == (int)sizeof(wr), "write chunked");
	CHECK(flash.Read(50, rd, sizeof(rd)) == (int)sizeof(rd), "read chunked");
	CHECK(memcmp(wr, rd, sizeof(wr)) == 0, "chunked readback");
	CHECK(s_VioPageCross == 0 && s_VioNoWel == 0, "chunked violations");
}

static void TestLifecycle(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(s_Cfg, &Spi), "init");

	flash.Disable();
	CHECK(s_Dpd, "deep power down entered");
	CHECK(s_DisableCnt == 1, "interface disable refcount");
	flash.Enable();
	CHECK(!s_Dpd, "deep power down released");
	CHECK(s_EnableCnt == 1, "interface enable refcount");
}

static void TestFlashDiskIO(MockSpi &Spi)
{
	MockPowerOn();
	FlashDiskIO disk;
	alignas(4) static uint8_t cacheMem[NOR_SECT];
	DiskIOCache_t cache = { -1, 0xFFFFFFFF, cacheMem };

	CHECK(disk.Init(s_Cfg, &Spi, &cache, 1), "init");
	CHECK(disk.GetSize() == NOR_SIZE / 1024, "size KB");
	CHECK(disk.GetSectSize() == NOR_SECT, "sect size");

	uint8_t wr[NOR_SECT], rd[NOR_SECT];
	Fill(wr, sizeof(wr), 13);
	CHECK(disk.SectWrite(3, wr), "sect write");
	CHECK(disk.SectRead(3, rd), "sect read");
	CHECK(memcmp(wr, rd, sizeof(wr)) == 0, "sector roundtrip");

	// Byte access through the cache path
	uint8_t d[32];
	Fill(d, sizeof(d), 17);
	CHECK(disk.Write(3 * NOR_SECT + 100, d, sizeof(d)) == (int)sizeof(d),
		  "cached write");
	disk.Flush();
	memset(rd, 0, sizeof(rd));
	CHECK(disk.SectRead(3, rd), "sect read back");
	CHECK(memcmp(&rd[100], d, sizeof(d)) == 0, "cached data");
	CHECK(memcmp(rd, wr, 100) == 0, "sector head preserved");

	disk.EraseSector(3, 1);
	CHECK(s_Mem[3 * NOR_SECT] == 0xFF, "erase sector");
}

// The synchronous NvmIO contract requires Write to return only after the
// medium holds the committed result. With the mock modelling several status
// polls of write in progress after each page program, the device must no
// longer be busy by the time Write returns.
static void TestWriteIsSynchronous(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(s_Cfg, &Spi), "init");

	// Every program keeps write in progress asserted for several polls.
	s_BusyPolls = 8;

	// A multi page write: the final page must also be waited for.
	uint8_t wr[700];
	Fill(wr, sizeof(wr), 21);
	CHECK(flash.Write(0, wr, sizeof(wr)) == (int)sizeof(wr), "write");
	CHECK(!MockBusy(), "device still busy after Write returned");

	// A single page write ending exactly on a page boundary.
	uint8_t wr2[NOR_PAGE];
	Fill(wr2, sizeof(wr2), 22);
	CHECK(flash.Write(NOR_PAGE * 3, wr2, sizeof(wr2)) == (int)sizeof(wr2),
		  "aligned write");
	CHECK(!MockBusy(), "device busy after aligned write");

	// Erase must be synchronous as well.
	CHECK(flash.Erase(0, NOR_SECT) == 0, "erase");
	CHECK(!MockBusy(), "device busy after Erase returned");

	s_BusyPolls = 0;
}

// Region arithmetic must reject an offset past the device and any region
// whose end overflows, not wrap into an accepted range. Config with an out of
// range address or id size must be rejected before any access.
static void TestInitValidation(MockSpi &Spi)
{
	MockPowerOn();

	// Offset past the device.
	{
		NvmFlash f;
		CHECK(f.Init(s_Cfg, &Spi, NOR_SIZE + NOR_SECT, 0) == false,
			  "offset past device");
	}
	// Region end past the device (would overflow the old check).
	{
		NvmFlash f;
		CHECK(f.Init(s_Cfg, &Spi, NOR_SIZE - NOR_SECT, NOR_SECT * 4) == false,
			  "region end past device");
	}
	// A huge size that wraps when added to the offset.
	{
		NvmFlash f;
		uint64_t huge = (uint64_t)0 - NOR_SECT;	// wraps if added to offset
		CHECK(f.Init(s_Cfg, &Spi, NOR_SECT, huge) == false,
			  "overflow region size");
	}
	// Address size out of range.
	{
		FlashCfg_t bad = s_Cfg;
		bad.AddrSize = 5;
		NvmFlash f;
		CHECK(f.Init(bad, &Spi) == false, "addr size too large");
		bad.AddrSize = 0;
		NvmFlash f2;
		CHECK(f2.Init(bad, &Spi) == false, "addr size zero");
	}
	// Id size out of range.
	{
		FlashCfg_t bad = s_Cfg;
		bad.DevIdSize = 5;
		NvmFlash f;
		CHECK(f.Init(bad, &Spi) == false, "id size too large");
	}
	// A valid whole-device init still works.
	{
		NvmFlash f;
		CHECK(f.Init(s_Cfg, &Spi), "valid whole device");
		CHECK(f.Size() == NOR_SIZE, "size");
	}
}

int main(void)
{
	MockSpi spi;

	TestInitProbe(spi);
	TestInitValidation(spi);
	TestWriteRead(spi);
	TestWriteIsSynchronous(spi);
	TestErase(spi);
	TestRegion(spi);
	TestBusyAndWel(spi);
	TestChunkedTransfer(spi);
	TestLifecycle(spi);
	TestFlashDiskIO(spi);

	printf(g_Fail ? "RESULT: %d FAILURES\n" : "RESULT: ALL PASS\n", g_Fail);
	return g_Fail ? 1 : 0;
}
