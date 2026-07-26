/**-------------------------------------------------------------------------
@example	nvm_test.cpp

@brief	Host tests for the generic Nvm driver.

The same Nvm class is exercised against SPI NOR and I2C EEPROM models. The
tests cover page splitting, erase behavior, lifecycle ownership, whole-device
write protection and failed initialization cleanup.

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
----------------------------------------------------------------------------*/
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "device_intrf.h"
#include "storage/nvm.h"
#include "storage/flash.h"

static int g_Fail;
static int g_Checks;

#define CHECK(cond, ...) do { \
	g_Checks++; \
	if (!(cond)) { \
		g_Fail++; \
		printf("FAIL %s:%d: ", __func__, __LINE__); \
		printf(__VA_ARGS__); \
		printf("\n"); \
	} \
} while (0)

static int s_PinSet;
static int s_PinClear;

extern "C" {
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinSet(int, int) { s_PinSet++; }
void IOPinClear(int, int) { s_PinClear++; }
}

#define NOR_SIZE	(64u * 1024u)
#define NOR_SECT	4096u
#define NOR_PAGE	256u
#define NOR_ADDR	3
#define NOR_ID		0x1234C2u

static uint8_t s_Nor[NOR_SIZE];
static bool s_NorWel;
static uint8_t s_NorBp;
static uint32_t s_NorResetCnt;
static int s_NorVioNoWel;
static int s_NorVioPageCross;
static std::vector<uint8_t> s_NorTx;

static uint32_t NorAddr(const uint8_t *p)
{
	return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

static void NorPowerOn(void)
{
	memset(s_Nor, 0xFF, sizeof(s_Nor));
	s_NorWel = false;
	s_NorBp = 0;
	s_NorResetCnt = 0;
	s_NorVioNoWel = 0;
	s_NorVioPageCross = 0;
	s_NorTx.clear();
}

extern "C" {
static bool NorStartRx(DevIntrf_t *, uint32_t) { return true; }
static bool NorStartTx(DevIntrf_t *, uint32_t) { s_NorTx.clear(); return true; }
static int NorTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_NorTx.push_back(pData[i]); }
	return Len;
}
static int NorTxSrData(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	return NorTxData(pDev, pData, Len);
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
		uint8_t status = s_NorBp | (s_NorWel ? 0x02u : 0u);
		for (int i = 0; i < Len; i++) { pBuff[i] = status; }
		return Len;
	}
	if ((int)s_NorTx.size() < 1 + NOR_ADDR)
	{
		return 0;
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
		case FLASH_CMD_WRENABLE:
			s_NorWel = true;
			break;
		case FLASH_CMD_WRDISABLE:
			s_NorWel = false;
			break;
		case FLASH_CMD_RESET_DEVICE:
			s_NorResetCnt++;
			break;
		case FLASH_CMD_WRSR:
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			if (s_NorTx.size() >= 2) { s_NorBp = s_NorTx[1] & 0x3Cu; }
			s_NorWel = false;
			break;
		case FLASH_CMD_SECTOR_ERASE:
		{
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			uint32_t addr = NorAddr(&s_NorTx[1]) & ~(NOR_SECT - 1u);
			memset(&s_Nor[addr % NOR_SIZE], 0xFF, NOR_SECT);
			s_NorWel = false;
			break;
		}
		case FLASH_CMD_BULK_ERASE:
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			memset(s_Nor, 0xFF, sizeof(s_Nor));
			s_NorWel = false;
			break;
		case FLASH_CMD_WRITE:
		{
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			uint32_t addr = NorAddr(&s_NorTx[1]);
			uint32_t hdr = 1 + NOR_ADDR;
			uint32_t len = s_NorTx.size() > hdr ? (uint32_t)s_NorTx.size() - hdr : 0;
			if (len > 0 && addr / NOR_PAGE != (addr + len - 1) / NOR_PAGE)
			{
				s_NorVioPageCross++;
			}
			for (uint32_t i = 0; i < len; i++) { s_Nor[(addr + i) % NOR_SIZE] &= s_NorTx[hdr + i]; }
			s_NorWel = false;
			break;
		}
		default:
			break;
	}
	s_NorTx.clear();
}
}

#define EEP_SIZE	(8u * 1024u)
#define EEP_PAGE	32u
#define EEP_ADDR	2
#define EEP_DEVNO	0x50u

static uint8_t s_Eep[EEP_SIZE];
static uint32_t s_EepDevAddr;
static bool s_EepRx;
static int s_EepVioPageCross;
static std::vector<uint8_t> s_EepBuf;

static uint32_t EepAddr(void)
{
	uint32_t addr = ((uint32_t)(s_EepDevAddr & 7u)) << (EEP_ADDR * 8);
	for (int i = 0; i < EEP_ADDR; i++) { addr |= (uint32_t)s_EepBuf[i] << (8 * (EEP_ADDR - 1 - i)); }
	return addr;
}

static void EepPowerOn(void)
{
	memset(s_Eep, 0xFF, sizeof(s_Eep));
	s_EepDevAddr = EEP_DEVNO;
	s_EepRx = false;
	s_EepVioPageCross = 0;
	s_EepBuf.clear();
}

extern "C" {
static bool EepStartRx(DevIntrf_t *, uint32_t DevAddr) { s_EepDevAddr = DevAddr; s_EepRx = true; return true; }
static bool EepStartTx(DevIntrf_t *, uint32_t DevAddr) { s_EepDevAddr = DevAddr; s_EepRx = false; s_EepBuf.clear(); return true; }
static int EepTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_EepBuf.push_back(pData[i]); }
	return Len;
}
static int EepTxSrData(DevIntrf_t *pDev, const uint8_t *pData, int Len) { return EepTxData(pDev, pData, Len); }
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
	uint32_t len = (uint32_t)s_EepBuf.size() - EEP_ADDR;
	if (len == 0) { return; }
	if (addr / EEP_PAGE != (addr + len - 1) / EEP_PAGE) { s_EepVioPageCross++; }
	for (uint32_t i = 0; i < len; i++) { s_Eep[(addr + i) % EEP_SIZE] = s_EepBuf[EEP_ADDR + i]; }
}
}

extern "C" {
static void MockDisable(DevIntrf_t *) {}
static void MockEnable(DevIntrf_t *) {}
static uint32_t MockGetRate(DevIntrf_t *) { return 1000000; }
static uint32_t MockSetRate(DevIntrf_t *, uint32_t Rate) { return Rate; }
static void MockReset(DevIntrf_t *) {}
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
		vDev.Reset = MockReset;
		vDev.PowerOff = MockPowerOff;
		vDev.GetHandle = MockGetHandle;
		vDev.MaxRetry = 5;
		atomic_store(&vDev.EnCnt, 1);
		atomic_flag_clear(&vDev.bBusy);
	}

	operator DevIntrf_t * const () override { return &vDev; }
	uint32_t Rate(uint32_t Rate) override { return Rate; }
	uint32_t Rate(void) override { return 1000000; }
};

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
	cfg.EraseSize = 0;
	cfg.PageSize = EEP_PAGE;
	cfg.AddrSize = EEP_ADDR;
	cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
	return cfg;
}

static void CheckRoundTrip(Nvm &Mem, uint32_t Size, uint32_t Page)
{
	uint32_t len = Page * 3 + 11;
	std::vector<uint8_t> out(len), in(len, 0);
	for (uint32_t i = 0; i < len; i++) { out[i] = (uint8_t)(i * 7 + 3); }

	if (Mem.EraseSize() != 0) { CHECK(Mem.Erase(0, Mem.EraseSize()) == 0, "erase first unit"); }
	CHECK(Mem.Write(5, out.data(), len) == (int)len, "write across pages");
	CHECK(Mem.Read(5, in.data(), len) == (int)len, "read back");
	CHECK(memcmp(out.data(), in.data(), len) == 0, "round trip");
	CHECK(Mem.Read(Size, in.data(), 1) == -EINVAL, "read past end");
}

static void TestNor(MockIntrf &Bus)
{
	NorPowerOn();
	Nvm Mem;

	CHECK(atomic_load(&Bus.vDev.EnCnt) == 1, "initial interface reference");
	CHECK(Mem.Init(NorCfg(), &Bus), "NOR init");
	CHECK(atomic_load(&Bus.vDev.EnCnt) == 2, "Nvm owns one interface reference");
	CHECK(s_NorResetCnt == 1, "reset sent at init");
	CheckRoundTrip(Mem, NOR_SIZE, NOR_PAGE);
	CHECK(s_NorVioNoWel == 0, "all writes held WEL");
	CHECK(s_NorVioPageCross == 0, "no program crossed page");

	CHECK(Mem.SetWriteProtect(0, 1, true) == -ENOTSUP, "partial protect rejected");
	CHECK(Mem.SetWriteProtect(0, (uint32_t)Mem.Size(), true) == 0, "whole device protected");
	CHECK((Mem.ReadStatus() & 0x3Cu) == 0x3Cu, "BP bits set");
	CHECK(Mem.SetWriteProtect(0, (uint32_t)Mem.Size(), false) == 0, "whole device unprotected");

	Nvm Window;
	CHECK(Window.Init(NorCfg(), &Bus, NOR_SECT, NOR_SECT), "window init");
	CHECK(Window.SetWriteProtect(0, (uint32_t)Window.Size(), true) == -EPERM,
		  "window cannot change whole-device protection");
	Window.Disable();

	uint32_t resetBefore = s_NorResetCnt;
	Mem.Reset();
	CHECK(Mem.Valid(), "reset leaves device valid");
	CHECK(s_NorResetCnt == resetBefore + 1, "reset reissues device reset");

	Mem.Disable();
	CHECK(atomic_load(&Bus.vDev.EnCnt) == 1, "disable releases owned reference");
	uint8_t b = 0;
	CHECK(Mem.Read(0, &b, 1) == -EACCES, "disabled device rejects access");
	CHECK(Mem.Enable(), "enable succeeds");
	CHECK(atomic_load(&Bus.vDev.EnCnt) == 2, "enable reacquires reference");
	CHECK(Mem.Read(0, &b, 1) == 1, "enabled device reads");
	Mem.Disable();

	NvmCfg_t bad = NorCfg();
	bad.DevId = 0xDEAD00;
	Nvm Bad;
	CHECK(Bad.Init(bad, &Bus) == false, "wrong id rejected");
	CHECK(atomic_load(&Bus.vDev.EnCnt) == 1, "failed init releases reference");
}

static void TestEeprom(MockIntrf &Bus)
{
	EepPowerOn();
	Nvm Mem;
	CHECK(Mem.Init(EepCfg(), &Bus), "EEPROM init");
	CheckRoundTrip(Mem, EEP_SIZE, EEP_PAGE);
	CHECK(s_EepVioPageCross == 0, "EEPROM writes split at page");
	CHECK(Mem.SetWriteProtect(0, (uint32_t)Mem.Size(), true) == -ENOTSUP,
		  "EEPROM without WP mechanism rejected");

	NvmCfg_t cfg = EepCfg();
	cfg.WrProtPin = { 0, 12, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
	Nvm Pinned;
	CHECK(Pinned.Init(cfg, &Bus), "pinned EEPROM init");
	int setBefore = s_PinSet;
	int clearBefore = s_PinClear;
	CHECK(Pinned.SetWriteProtect(0, 1, true) == -ENOTSUP, "partial pin protect rejected");
	CHECK(Pinned.SetWriteProtect(0, (uint32_t)Pinned.Size(), true) == 0, "WP pin set");
	CHECK(Pinned.SetWriteProtect(0, (uint32_t)Pinned.Size(), false) == 0, "WP pin cleared");
	CHECK(s_PinSet == setBefore + 1, "pin set once");
	CHECK(s_PinClear == clearBefore + 1, "pin cleared once");
	Pinned.Disable();
	Mem.Disable();
}

int main(void)
{
	MockIntrf Nor(DEVINTRF_TYPE_SPI, NorStartRx, NorRxData, NorStopRx,
				  NorStartTx, NorTxData, NorTxSrData, NorStopTx);
	MockIntrf Eep(DEVINTRF_TYPE_I2C, EepStartRx, EepRxData, EepStopRx,
				  EepStartTx, EepTxData, EepTxSrData, EepStopTx);

	TestNor(Nor);
	TestEeprom(Eep);

	printf("\nChecks run: %d\n", g_Checks);
	if (g_Fail == 0)
	{
		printf("RESULT: ALL PASS\n");
		return 0;
	}
	printf("RESULT: %d FAILURES\n", g_Fail);
	return 1;
}
