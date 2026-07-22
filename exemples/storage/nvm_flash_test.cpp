/**--------------------------------------------------------------------------
@file	nvm_flash_test.cpp

@brief	Host test for the new NvmFlash driver on the NvmIO base.

A SPI NOR mock DeviceIntrf models the chip: the write enable latch, the write
in progress flag, program with clear only bit semantics, sector erase to all
ones, a page boundary that a single program must not cross, and a status
register carrying the block protect bits. The real NvmFlash class runs against
it through the NvmIO verbs.

Build and run on the host:

  g++ -std=gnu++23 -O1 -I include -I include/storage -I Linux/include \
	  exemples/storage/nvm_flash_test.cpp src/storage/nvm_flash.cpp \
	  src/device.cpp src/device_intrf.cpp -o nvm_flash_test
  ./nvm_flash_test

@author	Hoang Nguyen Hoan
@date	Jul. 20, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <vector>

#include "device_intrf.h"
#include "coredev/spi.h"
#include "storage/nvm_flash.h"

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
// SPI NOR mock. One command frame is captured in s_Tx during a transaction,
// acted on at StopTx, or served during RxData.
// ---------------------------------------------------------------------------
#define NOR_SIZE	(64u * 1024u)
#define NOR_SECT	4096u
#define NOR_PAGE	256u
#define NOR_ADDR	3

static uint8_t s_Mem[NOR_SIZE];
static bool s_Wel;
static uint8_t s_StatusBP;
static int s_VioNoWel;
static int s_VioPageCross;
static uint32_t s_DevId = 0x1234C2;

static std::vector<uint8_t> s_Tx;

static void MockPowerOn(void)
{
	memset(s_Mem, 0xFF, sizeof(s_Mem));
	s_Wel = false;
	s_StatusBP = 0;
	s_VioNoWel = 0;
	s_VioPageCross = 0;
	s_Tx.clear();
}

static uint32_t MockAddr(const uint8_t *p)
{
	uint32_t a = 0;
	for (int i = 0; i < NOR_ADDR; i++) { a = (a << 8) | p[i]; }
	return a;
}

extern "C" {

// StartRx must not clear the command frame; in DeviceIntrfRead it is a restart
// after the command and address were already sent through TxData.
static bool MockStartRx(DevIntrf_t *, uint32_t) { return true; }
static bool MockStartTx(DevIntrf_t *, uint32_t) { s_Tx.clear(); return true; }

static int MockTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_Tx.push_back(pData[i]); }
	return Len;
}
static int MockTxSrData(DevIntrf_t *d, const uint8_t *pData, int Len)
{
	return MockTxData(d, pData, Len);
}

static int MockRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	uint8_t cmd = s_Tx.empty() ? 0 : s_Tx[0];
	if (cmd == FLASH_CMD_READID)
	{
		for (int i = 0; i < Len; i++)
		{
			pBuff[i] = (uint8_t)(s_DevId >> (8 * i));
		}
		return Len;
	}
	if (cmd == FLASH_CMD_READSTATUS)
	{
		uint8_t st = s_StatusBP;
		if (s_Wel) { st |= 0x02u; }
		for (int i = 0; i < Len; i++) { pBuff[i] = st; }
		return Len;
	}
	// Data read: command byte then address.
	if ((int)s_Tx.size() < 1 + NOR_ADDR)
	{
		for (int i = 0; i < Len; i++) { pBuff[i] = 0xFF; }
		return Len;
	}
	uint32_t addr = MockAddr(&s_Tx[1]);
	for (int i = 0; i < Len; i++) { pBuff[i] = s_Mem[(addr + i) % NOR_SIZE]; }
	return Len;
}

static void MockStopRx(DevIntrf_t *) {}

static void MockStopTx(DevIntrf_t *)
{
	if (s_Tx.empty()) { return; }
	uint8_t cmd = s_Tx[0];
	switch (cmd)
	{
		case FLASH_CMD_WRENABLE:  s_Wel = true;  break;
		case FLASH_CMD_WRDISABLE: s_Wel = false; break;
		case FLASH_CMD_WRSR:
			if (!s_Wel) { s_VioNoWel++; break; }
			if (s_Tx.size() >= 2) { s_StatusBP = s_Tx[1] & 0x3Cu; }
			s_Wel = false;
			break;
		case FLASH_CMD_SECTOR_ERASE:
		{
			if (!s_Wel) { s_VioNoWel++; break; }
			uint32_t a = MockAddr(&s_Tx[1]) & ~(NOR_SECT - 1);
			memset(&s_Mem[a % NOR_SIZE], 0xFF, NOR_SECT);
			s_Wel = false;
			break;
		}
		case FLASH_CMD_BULK_ERASE:
			if (!s_Wel) { s_VioNoWel++; break; }
			memset(s_Mem, 0xFF, NOR_SIZE);
			s_Wel = false;
			break;
		case FLASH_CMD_WRITE:
		{
			if (!s_Wel) { s_VioNoWel++; break; }
			uint32_t a = MockAddr(&s_Tx[1]);
			uint32_t hdr = 1 + NOR_ADDR;
			if (s_Tx.size() > hdr)
			{
				uint32_t n = (uint32_t)s_Tx.size() - hdr;
				if ((a / NOR_PAGE) != ((a + n - 1) / NOR_PAGE))
				{
					s_VioPageCross++;
				}
				for (uint32_t i = 0; i < n; i++)
				{
					s_Mem[(a + i) % NOR_SIZE] &= s_Tx[hdr + i];
				}
			}
			s_Wel = false;
			break;
		}
		default: break;
	}
	s_Tx.clear();
}

static void MockDisable(DevIntrf_t *) {}
static void MockEnable(DevIntrf_t *) {}
static uint32_t MockGetRate(DevIntrf_t *) { return 8000000; }
static uint32_t MockSetRate(DevIntrf_t *, uint32_t r) { return r; }
static void MockPowerOff(DevIntrf_t *) {}
static void *MockGetHandle(DevIntrf_t *pDev) { return pDev->pDevData; }

}	// extern "C"

// Host stub. The SPI path of NvmFlash never calls this; it exists only so the
// QSPI branch links on the host.
extern "C" bool QuadSPISendCmd(SPIDev_t * const, uint8_t, uint32_t, uint8_t,
							   uint32_t, uint8_t) { return false; }

class MockSpi : public DeviceIntrf {
public:
	DevIntrf_t vDev;
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
	operator DevIntrf_t * const () override { return &vDev; }
	uint32_t Rate(uint32_t r) override { return r; }
	uint32_t Rate(void) override { return 8000000; }
};

static NvmCfg_t MakeCfg(void)
{
	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = 0;
	cfg.TotalSize = NOR_SIZE;
	cfg.EraseSize = NOR_SECT;
	cfg.PageSize = NOR_PAGE;
	cfg.AddrSize = NOR_ADDR;
	cfg.DevId = s_DevId;
	cfg.DevIdSize = 3;
	cfg.RdCmd = { FLASH_CMD_READ, 0 };
	cfg.WrCmd = { FLASH_CMD_WRITE, 0 };
	return cfg;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
static void TestInit(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	CHECK(flash.Init(MakeCfg(), &Spi), "init");
	CHECK(flash.Size() == NOR_SIZE, "size");
	CHECK(flash.EraseSize() == NOR_SECT, "erase size");
	CHECK(flash.PageSize() == NOR_PAGE, "page size");
	CHECK(flash.WriteGran() == 1, "write gran");
	CHECK(flash.ReadId(3) == s_DevId, "read id");

	// A wrong id fails init.
	NvmCfg_t bad = MakeCfg();
	bad.DevId = 0xDEAD00;
	NvmFlash f2;
	CHECK(f2.Init(bad, &Spi) == false, "wrong id rejected");
}

static void TestWriteRead(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	flash.Init(MakeCfg(), &Spi);

	CHECK(flash.Erase(0, NOR_SECT) == 0, "erase sector");

	// Write across a page boundary; the driver must split it.
	uint32_t len = NOR_PAGE + 40;
	std::vector<uint8_t> out(len), in(len, 0);
	for (uint32_t i = 0; i < len; i++) { out[i] = (uint8_t)(i * 3 + 7); }

	CHECK(flash.Write(0, out.data(), len) == (int)len, "write across page");
	CHECK(flash.Read(0, in.data(), len) == (int)len, "read back");
	CHECK(memcmp(out.data(), in.data(), len) == 0, "round trip");
	CHECK(s_VioNoWel == 0, "no program without wel");
	CHECK(s_VioPageCross == 0, "no single write crossed a page");
}

static void TestErase(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	flash.Init(MakeCfg(), &Spi);

	uint8_t wr[256], rd[256];
	memset(wr, 0x5A, sizeof(wr));
	flash.Write(NOR_SECT, wr, sizeof(wr));
	CHECK(flash.Erase(NOR_SECT, NOR_SECT) == 0, "erase");
	flash.Read(NOR_SECT, rd, sizeof(rd));
	bool ones = true;
	for (unsigned i = 0; i < sizeof(rd); i++) { if (rd[i] != 0xFF) { ones = false; break; } }
	CHECK(ones, "erased reads all ones");

	// Alignment and bounds.
	CHECK(flash.Erase(100, NOR_SECT) == -EINVAL, "misaligned offset");
	CHECK(flash.Erase(0, 100) == -EINVAL, "misaligned length");
	CHECK(flash.Erase(0, 0) == -EINVAL, "zero length");
	CHECK(flash.Read(NOR_SIZE, rd, 1) == -EINVAL, "read past end");
}

static void TestWriteProtect(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	flash.Init(MakeCfg(), &Spi);

	CHECK(flash.SetWriteProtect(0, (uint32_t)flash.Size(), true) == 0,
		  "protect enable");
	CHECK((flash.ReadStatus() & 0x3Cu) == 0x3Cu, "bp bits set");
	CHECK(flash.SetWriteProtect(0, (uint32_t)flash.Size(), false) == 0,
		  "protect disable");
	CHECK((flash.ReadStatus() & 0x3Cu) == 0x00u, "bp bits clear");
	CHECK(s_VioNoWel == 0, "wrsr issued with wel");
}

static void TestMassErase(MockSpi &Spi)
{
	MockPowerOn();
	NvmFlash flash;
	flash.Init(MakeCfg(), &Spi);
	uint8_t d = 0x00;
	flash.Write(10, &d, 1);
	CHECK(flash.MassErase() == 0, "mass erase whole device");
	uint8_t r = 0;
	flash.Read(10, &r, 1);
	CHECK(r == 0xFF, "device erased");

	// A windowed instance may not mass erase.
	NvmFlash win;
	win.Init(MakeCfg(), &Spi, NOR_SECT, NOR_SECT);
	CHECK(win.MassErase() == -EPERM, "mass erase rejected on window");
}

int main(void)
{
	MockSpi spi;

	TestInit(spi);
	TestWriteRead(spi);
	TestErase(spi);
	TestWriteProtect(spi);
	TestMassErase(spi);

	printf("\nChecks run: %d\n", g_Checks);
	if (g_Fail == 0)
	{
		printf("RESULT: ALL PASS\n");
		return 0;
	}
	printf("RESULT: %d FAILURES\n", g_Fail);
	return 1;
}
