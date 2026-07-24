/**-------------------------------------------------------------------------
@example	nvm_test.cpp

@brief	Host test for the Nvm serial memory driver.

		One driver serves every addressed serial memory, so one test exercises
		it against two devices that behave nothing alike underneath: a SPI NOR
		flash that needs a command byte, a write enable latch, a status poll
		and an erase, and an I2C EEPROM that needs none of those and overwrites
		directly after a fixed write time. Both are described by a config and
		driven by the same code.

		The mocks enforce what the real parts enforce, so a driver mistake
		shows up as a violation rather than as wrong data: programming without
		the write enable latch, a transfer crossing a page boundary, and
		programming that sets a bit without an erase.

Build and run on the host:

  g++ -std=gnu++23 -O1 -I include -I include/storage -I Linux/include \
	  exemples/storage/nvm_test.cpp src/storage/nvm.cpp \
	  src/device.cpp src/device_intrf.cpp -o nvm_test
  ./nvm_test

@author	Hoang Nguyen Hoan
@date	July 24, 2026

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
#include <atomic>
#include <vector>

#include "device_intrf.h"
#include "storage/nvm.h"
#include "storage/flash.h"		// for the FLASH_CMD_* opcodes

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

// The pin driver is per architecture; the driver only toggles a protect pin.
extern "C" {
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinSet(int, int) {}
void IOPinClear(int, int) {}
}

// ---------------------------------------------------------------------------
// SPI NOR flash. Command byte, write enable latch, status poll, sector erase.
// ---------------------------------------------------------------------------
#define NOR_SIZE	(64u * 1024u)
#define NOR_SECT	4096u
#define NOR_PAGE	256u
#define NOR_ADDR	3
#define NOR_ID		0x1234C2

static uint8_t s_Nor[NOR_SIZE];
static bool s_NorWel;
static uint8_t s_NorBp;
static int s_NorVioNoWel;
static int s_NorVioPageCross;
static std::vector<uint8_t> s_NorTx;

static void NorPowerOn(void)
{
	memset(s_Nor, 0xFF, sizeof(s_Nor));
	s_NorWel = false;
	s_NorBp = 0;
	s_NorVioNoWel = 0;
	s_NorVioPageCross = 0;
	s_NorTx.clear();
}

static uint32_t NorAddr(const uint8_t *p)
{
	uint32_t a = 0;
	for (int i = 0; i < NOR_ADDR; i++) { a = (a << 8) | p[i]; }
	return a;
}

extern "C" {

// A restart inside a read: the command and address are already sent, so the
// frame must survive.
static bool NorStartRx(DevIntrf_t *, uint32_t) { return true; }
static bool NorStartTx(DevIntrf_t *, uint32_t) { s_NorTx.clear(); return true; }

static int NorTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_NorTx.push_back(pData[i]); }
	return Len;
}
static int NorTxSrData(DevIntrf_t *d, const uint8_t *p, int n)
{
	return NorTxData(d, p, n);
}

static int NorRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	uint8_t cmd = s_NorTx.empty() ? 0 : s_NorTx[0];

	if (cmd == FLASH_CMD_READID)
	{
		for (int i = 0; i < Len; i++) { pBuff[i] = (uint8_t)(NOR_ID >> (8 * i)); }
		return Len;
	}
	if (cmd == FLASH_CMD_READSTATUS)
	{
		uint8_t st = s_NorBp;
		if (s_NorWel) { st |= 0x02u; }
		for (int i = 0; i < Len; i++) { pBuff[i] = st; }
		return Len;
	}
	if ((int)s_NorTx.size() < 1 + NOR_ADDR)
	{
		for (int i = 0; i < Len; i++) { pBuff[i] = 0xFF; }
		return Len;
	}

	uint32_t addr = NorAddr(&s_NorTx[1]);
	for (int i = 0; i < Len; i++) { pBuff[i] = s_Nor[(addr + i) % NOR_SIZE]; }
	return Len;
}

static void NorStopRx(DevIntrf_t *) {}

static void NorStopTx(DevIntrf_t *)
{
	if (s_NorTx.empty()) { return; }

	uint8_t cmd = s_NorTx[0];

	switch (cmd)
	{
		case FLASH_CMD_WRENABLE:  s_NorWel = true;  break;
		case FLASH_CMD_WRDISABLE: s_NorWel = false; break;

		case FLASH_CMD_WRSR:
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			if (s_NorTx.size() >= 2) { s_NorBp = s_NorTx[1] & 0x3Cu; }
			s_NorWel = false;
			break;

		case FLASH_CMD_SECTOR_ERASE:
		{
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			uint32_t a = NorAddr(&s_NorTx[1]) & ~(NOR_SECT - 1);
			memset(&s_Nor[a % NOR_SIZE], 0xFF, NOR_SECT);
			s_NorWel = false;
			break;
		}

		case FLASH_CMD_BULK_ERASE:
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			memset(s_Nor, 0xFF, NOR_SIZE);
			s_NorWel = false;
			break;

		case FLASH_CMD_WRITE:
		{
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			uint32_t a = NorAddr(&s_NorTx[1]);
			uint32_t hdr = 1 + NOR_ADDR;
			if (s_NorTx.size() > hdr)
			{
				uint32_t n = (uint32_t)s_NorTx.size() - hdr;
				if ((a / NOR_PAGE) != ((a + n - 1) / NOR_PAGE))
				{
					s_NorVioPageCross++;
				}
				// Programming clears bits only.
				for (uint32_t i = 0; i < n; i++)
				{
					s_Nor[(a + i) % NOR_SIZE] &= s_NorTx[hdr + i];
				}
			}
			s_NorWel = false;
			break;
		}

		default: break;
	}

	s_NorTx.clear();
}

}	// extern "C"

// ---------------------------------------------------------------------------
// I2C EEPROM. No command byte, no latch, no erase, a fixed write time.
// ---------------------------------------------------------------------------
#define EEP_SIZE	(8u * 1024u)
#define EEP_PAGE	32u
#define EEP_ADDR	2
#define EEP_DEVNO	0x50

static uint8_t s_Eep[EEP_SIZE];
static int s_EepVioPageCross;
static uint32_t s_EepDevAddr;
static bool s_EepRx;
static std::vector<uint8_t> s_EepBuf;

static void EepPowerOn(void)
{
	memset(s_Eep, 0xFF, sizeof(s_Eep));
	s_EepVioPageCross = 0;
	s_EepBuf.clear();
}

static uint32_t EepAddr(void)
{
	uint32_t a = ((uint32_t)(s_EepDevAddr & 7)) << (EEP_ADDR * 8);

	for (int i = 0; i < EEP_ADDR; i++)
	{
		a |= (uint32_t)s_EepBuf[i] << (8 * (EEP_ADDR - 1 - i));
	}

	return a;
}

extern "C" {

static bool EepStartRx(DevIntrf_t *, uint32_t DevAddr)
{
	s_EepDevAddr = DevAddr; s_EepRx = true; return true;
}
static bool EepStartTx(DevIntrf_t *, uint32_t DevAddr)
{
	s_EepDevAddr = DevAddr; s_EepRx = false; s_EepBuf.clear(); return true;
}

static int EepTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_EepBuf.push_back(pData[i]); }
	return Len;
}
static int EepTxSrData(DevIntrf_t *d, const uint8_t *p, int n)
{
	return EepTxData(d, p, n);
}

static int EepRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (s_EepBuf.size() < EEP_ADDR) { return 0; }

	uint32_t addr = EepAddr();
	for (int i = 0; i < Len; i++) { pBuff[i] = s_Eep[(addr + i) % EEP_SIZE]; }
	return Len;
}

static void EepStopRx(DevIntrf_t *) {}

static void EepStopTx(DevIntrf_t *)
{
	if (s_EepRx || s_EepBuf.size() < EEP_ADDR) { return; }

	uint32_t addr = EepAddr();
	uint32_t n = (uint32_t)s_EepBuf.size() - EEP_ADDR;

	if (n == 0) { return; }			// address only, a read setup

	if ((addr / EEP_PAGE) != ((addr + n - 1) / EEP_PAGE))
	{
		s_EepVioPageCross++;
	}
	// Overwrites directly, no erase.
	for (uint32_t i = 0; i < n; i++)
	{
		s_Eep[(addr + i) % EEP_SIZE] = s_EepBuf[EEP_ADDR + i];
	}
}

}	// extern "C"

// ---------------------------------------------------------------------------
// A mock bus taking its behaviour from a table of handlers.
// ---------------------------------------------------------------------------
extern "C" {
static void MockDisable(DevIntrf_t *) {}
static void MockEnable(DevIntrf_t *) {}
static uint32_t MockGetRate(DevIntrf_t *) { return 1000000; }
static uint32_t MockSetRate(DevIntrf_t *, uint32_t r) { return r; }
static void MockPowerOff(DevIntrf_t *) {}
static void *MockGetHandle(DevIntrf_t *pDev) { return pDev->pDevData; }
}

class MockIntrf : public DeviceIntrf {
public:
	DevIntrf_t vDev;

	MockIntrf(DEVINTRF_TYPE Type,
			  bool (*StartRx)(DevIntrf_t*, uint32_t),
			  int (*RxData)(DevIntrf_t*, uint8_t*, int),
			  void (*StopRx)(DevIntrf_t*),
			  bool (*StartTx)(DevIntrf_t*, uint32_t),
			  int (*TxData)(DevIntrf_t*, const uint8_t*, int),
			  int (*TxSrData)(DevIntrf_t*, const uint8_t*, int),
			  void (*StopTx)(DevIntrf_t*))
	{
		memset(&vDev, 0, sizeof(vDev));
		vDev.pDevData = this;
		vDev.Type = Type;
		vDev.Disable = MockDisable;
		vDev.Enable = MockEnable;
		vDev.GetRate = MockGetRate;
		vDev.SetRate = MockSetRate;
		vDev.StartRx = StartRx;
		vDev.RxData = RxData;
		vDev.StopRx = StopRx;
		vDev.StartTx = StartTx;
		vDev.TxData = TxData;
		vDev.TxSrData = TxSrData;
		vDev.StopTx = StopTx;
		vDev.PowerOff = MockPowerOff;
		vDev.GetHandle = MockGetHandle;
		vDev.MaxRetry = 5;
		vDev.MaxTrxLen = 65536;
		vDev.EnCnt = 1;
		atomic_flag_clear(&vDev.bBusy);
	}

	operator DevIntrf_t * const () override { return &vDev; }
	uint32_t Rate(uint32_t r) override { return r; }
	uint32_t Rate(void) override { return 1000000; }
};

// ---------------------------------------------------------------------------
// The configs. This is the whole difference between the two devices.
// ---------------------------------------------------------------------------
static NvmCfg_t NorCfg(void)
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = 0;
	cfg.TotalSize = NOR_SIZE;
	cfg.EraseSize = NOR_SECT;
	cfg.PageSize = NOR_PAGE;
	cfg.AddrSize = NOR_ADDR;
	cfg.DevId = NOR_ID;
	cfg.DevIdSize = 3;
	cfg.RdCmd = { FLASH_CMD_READ, 0 };
	cfg.WrCmd = { FLASH_CMD_WRITE, 0 };
	cfg.WrEnCmd = { FLASH_CMD_WRENABLE, 0 };
	cfg.WrDisCmd = { FLASH_CMD_WRDISABLE, 0 };
	cfg.EraseCmd = { FLASH_CMD_SECTOR_ERASE, 0 };
	cfg.MassEraseCmd = { FLASH_CMD_BULK_ERASE, 0 };
	cfg.RdStatusCmd = { FLASH_CMD_READSTATUS, 0 };
	cfg.WrStatusCmd = { FLASH_CMD_WRSR, 0 };
	cfg.WrProtMask = 0x3C;
	cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };

	return cfg;
}

static NvmCfg_t EepCfg(void)
{
	NvmCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = EEP_DEVNO;
	cfg.TotalSize = EEP_SIZE;
	cfg.EraseSize = 0;			// overwrites directly
	cfg.PageSize = EEP_PAGE;
	cfg.AddrSize = EEP_ADDR;
	cfg.WriteDelayUs = 0;		// no real wait on the host
	// Port 0 pin 0 is a real pin, so an unused one has to say so.
	cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };

	return cfg;
}

// ---------------------------------------------------------------------------
// The sequence every addressed memory must answer, whatever it is.
// ---------------------------------------------------------------------------
static void RunCommon(Nvm &Mem, const char *pWhat, uint32_t Total, uint32_t Page)
{
	printf("--- %s\n", pWhat);

	CHECK(Mem.Size() == Total, "%s: size", pWhat);
	CHECK(Mem.PageSize() == Page, "%s: page size", pWhat);
	CHECK(Mem.WriteGran() == 1, "%s: write granularity", pWhat);

	// A write spanning several pages; the driver has to split it.
	uint32_t len = Page * 3 + 11;
	std::vector<uint8_t> out(len), in(len, 0);
	for (uint32_t i = 0; i < len; i++) { out[i] = (uint8_t)(i * 7 + 3); }

	if (Mem.EraseSize() != 0)
	{
		CHECK(Mem.Erase(0, Mem.EraseSize()) == 0, "%s: erase first unit", pWhat);
	}

	CHECK(Mem.Write(5, out.data(), len) == (int)len, "%s: write across pages", pWhat);
	CHECK(Mem.Read(5, in.data(), len) == (int)len, "%s: read back", pWhat);
	CHECK(memcmp(out.data(), in.data(), len) == 0, "%s: round trip", pWhat);

	// Bounds.
	uint8_t b[4];
	CHECK(Mem.Read(Total, b, 4) == -EINVAL, "%s: read past the end", pWhat);
	CHECK(Mem.Write(Total, b, 4) == -EINVAL, "%s: write past the end", pWhat);
}

static void TestNorFlash(MockIntrf &Bus)
{
	NorPowerOn();

	Nvm mem;
	CHECK(mem.Init(NorCfg(), &Bus), "flash init");
	CHECK(mem.ReadId(3) == NOR_ID, "flash id");
	CHECK(mem.EraseSize() == NOR_SECT, "flash erase unit");

	RunCommon(mem, "NOR flash", NOR_SIZE, NOR_PAGE);

	CHECK(s_NorVioNoWel == 0, "flash: nothing programmed without the latch");
	CHECK(s_NorVioPageCross == 0, "flash: no transfer crossed a page");

	// A wrong id is refused.
	NvmCfg_t bad = NorCfg();
	bad.DevId = 0xDEAD00;
	Nvm f2;
	CHECK(f2.Init(bad, &Bus) == false, "flash: wrong id refused");

	// Erase restores all ones and alignment is enforced.
	NorPowerOn();
	Nvm m2;
	m2.Init(NorCfg(), &Bus);
	uint8_t wr[64], rd[64];
	memset(wr, 0x5A, sizeof(wr));
	m2.Write(NOR_SECT, wr, sizeof(wr));
	CHECK(m2.Erase(NOR_SECT, NOR_SECT) == 0, "flash: erase a sector");
	m2.Read(NOR_SECT, rd, sizeof(rd));
	bool ones = true;
	for (unsigned i = 0; i < sizeof(rd); i++) { if (rd[i] != 0xFF) { ones = false; break; } }
	CHECK(ones, "flash: erased reads all ones");
	CHECK(m2.Erase(100, NOR_SECT) == -EINVAL, "flash: misaligned erase offset");
	CHECK(m2.Erase(0, 100) == -EINVAL, "flash: misaligned erase length");

	// Programming clears bits and cannot set them back.
	uint8_t half = 0x0F, all = 0xFF, one = 0;
	m2.Erase(0, NOR_SECT);
	m2.Write(0, &half, 1);
	m2.Write(0, &all, 1);
	m2.Read(0, &one, 1);
	CHECK(one == half, "flash: bits do not come back without an erase");

	// Write protect through the status block protect bits.
	CHECK(m2.SetWriteProtect(0, (uint32_t)m2.Size(), true) == 0, "flash: protect");
	CHECK((m2.ReadStatus() & 0x3Cu) == 0x3Cu, "flash: protect bits set");
	CHECK(m2.SetWriteProtect(0, (uint32_t)m2.Size(), false) == 0, "flash: unprotect");
	CHECK((m2.ReadStatus() & 0x3Cu) == 0x00u, "flash: protect bits clear");

	// Chip erase, and only on an instance covering the whole device.
	NorPowerOn();
	Nvm whole, win;
	whole.Init(NorCfg(), &Bus);
	uint8_t d = 0x00;
	whole.Write(10, &d, 1);
	CHECK(whole.MassErase() == 0, "flash: mass erase");
	uint8_t r = 0;
	whole.Read(10, &r, 1);
	CHECK(r == 0xFF, "flash: device erased");
	win.Init(NorCfg(), &Bus, NOR_SECT, NOR_SECT);
	CHECK(win.MassErase() == -EPERM, "flash: mass erase refused on a window");
}

static void TestEeprom(MockIntrf &Bus)
{
	EepPowerOn();

	Nvm mem;
	CHECK(mem.Init(EepCfg(), &Bus), "eeprom init");
	CHECK(mem.EraseSize() == 0, "eeprom has no erase unit");

	RunCommon(mem, "I2C EEPROM", EEP_SIZE, EEP_PAGE);

	CHECK(s_EepVioPageCross == 0, "eeprom: no transfer crossed a page");

	// It overwrites in place, so a second write replaces the first and erase
	// is a no-op success.
	uint8_t a[8], b[8], rd[8];
	memset(a, 0xAA, sizeof(a));
	memset(b, 0x55, sizeof(b));
	mem.Write(100, a, sizeof(a));
	mem.Write(100, b, sizeof(b));
	mem.Read(100, rd, sizeof(rd));
	CHECK(memcmp(b, rd, sizeof(b)) == 0, "eeprom: overwrites without an erase");
	CHECK(mem.Erase(0, 64) == 0, "eeprom: erase is a no-op success");

	// No status register and no protect bits, so write protect needs a pin.
	CHECK(mem.SetWriteProtect(0, 1, true) == -ENOTSUP, "eeprom: no pin, unsupported");

	NvmCfg_t pincfg = EepCfg();
	pincfg.WrProtPin = { 0, 12, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
	Nvm pinned;
	pinned.Init(pincfg, &Bus);
	CHECK(pinned.SetWriteProtect(0, 1, true) == 0, "eeprom: protect through the pin");
	CHECK(pinned.SetWriteProtect(0, 1, false) == 0, "eeprom: unprotect through the pin");
}

int main(void)
{
	MockIntrf nor(DEVINTRF_TYPE_SPI, NorStartRx, NorRxData, NorStopRx,
				  NorStartTx, NorTxData, NorTxSrData, NorStopTx);
	MockIntrf eep(DEVINTRF_TYPE_I2C, EepStartRx, EepRxData, EepStopRx,
				  EepStartTx, EepTxData, EepTxSrData, EepStopTx);

	TestNorFlash(nor);
	TestEeprom(eep);

	printf("\nChecks run: %d\n", g_Checks);
	if (g_Fail == 0)
	{
		printf("RESULT: ALL PASS\n");
		return 0;
	}
	printf("RESULT: %d FAILURES\n", g_Fail);
	return 1;
}
