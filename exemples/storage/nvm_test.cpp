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
#include "coredev/spi.h"		// for the phased QSPI path

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
static uint32_t s_NorBusyCnt;	// status polls left before WIP clears
static bool s_NorIntrfAsync;	// interface returns -1 and reports by event
// Start the transfer and report nothing, so the driver is left with a
// chunk in flight and an event can be delivered by hand.
static bool s_NorHoldEvt;

// True between StartTx and the transfer that sends the payload.
// A status read reaches TxSrData with the write frame still in s_NorTx,
// because StartRx keeps the frame on purpose, and without this the read
// takes the program path a second time and answers -1 for a transfer
// nobody started.
static bool s_NorTxOpen;

// A transfer handed over and not yet reported, and how many times the
// driver opened a receive while one was outstanding. A driver that has
// taken something for the end of its transfer goes to the status register
// next, so this counts the mistake directly rather than inferring it from
// a state that looks the same either way.
static bool s_NorHeld;
static uint32_t s_NorRxWhileHeld;
static uint32_t s_NorRstCnt;
static uint8_t s_NorBp;
static uint32_t s_NorPgmOk;		// programs accepted before the part refuses
static int s_NorVioNoWel;
static int s_NorVioPageCross;
static std::vector<uint8_t> s_NorTx;

static void NorPowerOn(void)
{
	memset(s_Nor, 0xFF, sizeof(s_Nor));
	s_NorWel = false;
	s_NorBusyCnt = 0;
	s_NorIntrfAsync = false;
	s_NorHoldEvt = false;
	s_NorTxOpen = false;
	s_NorHeld = false;
	s_NorRxWhileHeld = 0;
	s_NorBp = 0;
	s_NorPgmOk = 0xFFFFFFFFUL;
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
static bool NorStartRx(DevIntrf_t *, uint32_t)
{
	if (s_NorHeld)
	{
		s_NorRxWhileHeld++;
	}

	return true;
}
static bool NorStartTx(DevIntrf_t *, uint32_t)
{
	s_NorTx.clear();
	s_NorTxOpen = true;

	return true;
}

static void NorApply(void);

static int NorTxData(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_NorTx.push_back(pData[i]); }

	// The part stops accepting programs after so many, so a chunk part way
	// through a multi page operation can be made to fail.
	if (!s_NorTx.empty() && s_NorTx[0] == FLASH_CMD_WRITE
		&& s_NorTx.size() > (size_t)(1 + NOR_ADDR))
	{
		if (s_NorPgmOk == 0)
		{
			return 0;
		}
		s_NorPgmOk--;
	}

	// Interrupt driven interface: a data bearing program transfer starts,
	// reports -1, applies, and completes through the event, the way the
	// real ports do. Command frames stay synchronous, as the real ports
	// wait for a no stop command phase.
	if (s_NorIntrfAsync && s_NorTxOpen && !s_NorTx.empty()
		&& s_NorTx[0] == FLASH_CMD_WRITE
		&& s_NorTx.size() > (size_t)(1 + NOR_ADDR))
	{
		NorApply();
		s_NorTxOpen = false;

		// Held: the transfer is started and nothing is reported, so the test
		// can deliver events itself and see which of them the driver takes.
		if (s_NorHoldEvt)
		{
			s_NorHeld = true;
		}
		else if (pDev->EvtCB)
		{
			pDev->EvtCB(pDev, DEVINTRF_EVT_COMPLETED, nullptr, 0);
		}
		// The real ports release the interface busy latch from the interrupt
		// at completion, since the framework skips Stop on this path.
		atomic_flag_clear(&pDev->bBusy);
		return -1;
	}

	return Len;
}
static int NorTxSrData(DevIntrf_t *d, const uint8_t *p, int n)
{
	return NorTxData(d, p, n);
}

static int NorRxData(DevIntrf_t *pDev, uint8_t *pBuff, int Len)
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
		if (s_NorBusyCnt > 0)
		{
			// The medium is still programming or erasing.
			st |= 0x01u;
			s_NorBusyCnt--;
		}
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

	if (s_NorIntrfAsync && cmd == FLASH_CMD_READ)
	{
		// The data landed in the caller's buffer; completion comes through
		// the event, the way an interrupt driven receive works.
		if (pDev->EvtCB)
		{
			pDev->EvtCB(pDev, DEVINTRF_EVT_COMPLETED, nullptr, 0);
		}
		atomic_flag_clear(&pDev->bBusy);
		return -1;
	}

	return Len;
}


// ---------------------------------------------------------------------------
// Simulated SPI FRAM: a direct read write medium on a serial bus. It
// overwrites in place and has no erase, but speaks the serial protocol: it
// latches writes behind WREN, answers the status poll, and holds block
// protect bits. This is the branch the command derivation exists for.
// ---------------------------------------------------------------------------

#define FRAM_SIZE		(32 * 1024)
#define FRAM_PAGE		256			// transfer chunk, FRAM has no real page
#define FRAM_ADDR		2
#define FRAM_ID			0x017F04

static uint8_t s_Fram[FRAM_SIZE];
static std::vector<uint8_t> s_FramTx;
static bool s_FramWel;
static uint8_t s_FramBp;
static uint32_t s_FramWrsrCnt;
static uint32_t s_FramRstCnt;
static uint32_t s_FramVioNoWel;

static void FramPowerOn(void)
{
	memset(s_Fram, 0, sizeof(s_Fram));
	s_FramTx.clear();
	s_FramWel = false;
	s_FramBp = 0;
	s_FramWrsrCnt = 0;
	s_FramRstCnt = 0;
	s_FramVioNoWel = 0;
}

static uint32_t FramAddr(const uint8_t *p)
{
	return ((uint32_t)p[0] << 8) | p[1];
}

static bool FramStartRx(DevIntrf_t *, uint32_t) { return true; }
static bool FramStartTx(DevIntrf_t *, uint32_t) { s_FramTx.clear(); return true; }

static int FramTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_FramTx.push_back(pData[i]); }
	return Len;
}
static int FramTxSrData(DevIntrf_t *d, const uint8_t *p, int n)
{
	return FramTxData(d, p, n);
}

static int FramRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	uint8_t cmd = s_FramTx.empty() ? 0 : s_FramTx[0];

	if (cmd == FLASH_CMD_READID)
	{
		for (int i = 0; i < Len; i++) { pBuff[i] = (uint8_t)(FRAM_ID >> (8 * i)); }
		return Len;
	}
	if (cmd == FLASH_CMD_READSTATUS)
	{
		uint8_t st = s_FramBp;
		if (s_FramWel) { st |= 0x02u; }
		for (int i = 0; i < Len; i++) { pBuff[i] = st; }
		return Len;
	}
	if ((int)s_FramTx.size() < 1 + FRAM_ADDR)
	{
		for (int i = 0; i < Len; i++) { pBuff[i] = 0xFF; }
		return Len;
	}

	uint32_t addr = FramAddr(&s_FramTx[1]);
	for (int i = 0; i < Len; i++) { pBuff[i] = s_Fram[(addr + i) % FRAM_SIZE]; }
	return Len;
}

static void FramStopRx(DevIntrf_t *) {}

static void FramStopTx(DevIntrf_t *)
{
	if (s_FramTx.empty()) { return; }

	uint8_t cmd = s_FramTx[0];

	switch (cmd)
	{
		case FLASH_CMD_WRENABLE:  s_FramWel = true;  break;
		case FLASH_CMD_WRDISABLE: s_FramWel = false; break;

		case FLASH_CMD_RESET_DEVICE: s_FramRstCnt++; break;

		case FLASH_CMD_WRSR:
			if (s_FramTx.size() >= 2) { s_FramBp = s_FramTx[1]; }
			s_FramWrsrCnt++;
			s_FramWel = false;
			break;

		case FLASH_CMD_WRITE:
			if ((int)s_FramTx.size() > 1 + FRAM_ADDR)
			{
				if (!s_FramWel)
				{
					s_FramVioNoWel++;
					break;
				}
				uint32_t addr = FramAddr(&s_FramTx[1]);
				for (size_t i = 1 + FRAM_ADDR; i < s_FramTx.size(); i++)
				{
					// Overwrites in place: no erase, no bit clearing rule.
					s_Fram[(addr + i - 1 - FRAM_ADDR) % FRAM_SIZE] = s_FramTx[i];
				}
				s_FramWel = false;
			}
			break;

		default: break;
	}
}

static void NorStopRx(DevIntrf_t *) {}

static void NorStopTx(DevIntrf_t *)
{
	NorApply();
}

static void NorApply(void)
{
	if (s_NorTx.empty()) { return; }

	uint8_t cmd = s_NorTx[0];

	switch (cmd)
	{
		case FLASH_CMD_WRENABLE:  s_NorWel = true;  break;
		case FLASH_CMD_WRDISABLE: s_NorWel = false; break;

		case FLASH_CMD_RESET_DEVICE: s_NorRstCnt++; break;

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
			s_NorBusyCnt = 3;
			break;
		}

		case FLASH_CMD_BULK_ERASE:
			if (!s_NorWel) { s_NorVioNoWel++; break; }
			memset(s_Nor, 0xFF, NOR_SIZE);
			s_NorWel = false;
			s_NorBusyCnt = 3;
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
				s_NorBusyCnt = 3;
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

// ---------------------------------------------------------------------------
// Simulated quad SPI NOR behind a phased interface. Commands, addresses and
// dummy cycles arrive through QuadSPISendCmd as phases, never as frame
// bytes; only data moves through the Rx/Tx handlers. The mock records what
// reached the interface, so the checks can prove the part's own quad
// command and dummy cycles were passed through.
// ---------------------------------------------------------------------------

#define QSPI_SIZE		(256u * 1024u)
#define QSPI_PAGE		256u
#define QSPI_ADDR		3
#define QSPI_ID			0x1628C2

static SPIDev_t s_QDev;
static uint8_t s_QMem[QSPI_SIZE];
static bool s_QWel;
static uint8_t s_QBp;
static uint8_t s_QCmd;
static uint32_t s_QAddr;
static uint8_t s_QLastRdDummy;
static uint8_t s_QLastWrCmd;
static uint32_t s_QSetMemSize;
static uint32_t s_QRstCnt;
static uint32_t s_QEraseCnt;
static uint32_t s_QWrsrCnt;
static uint32_t s_QVioNoWel;
static uint32_t s_QVioEraseNoWel;

static void QspiPowerOn(void)
{
	memset(s_QMem, 0xFF, sizeof(s_QMem));
	s_QWel = false;
	s_QBp = 0;
	s_QCmd = 0;
	s_QAddr = 0;
	s_QLastRdDummy = 0xFF;
	s_QLastWrCmd = 0;
	s_QSetMemSize = 0;
	s_QRstCnt = 0;
	s_QEraseCnt = 0;
	s_QWrsrCnt = 0;
	s_QVioNoWel = 0;
	s_QVioEraseNoWel = 0;
}

// The two functions a real target defines in its QSPI implementation.
void QuadSPISetMemSize(SPIDev_t * const, uint32_t Size)
{
	s_QSetMemSize = Size;
}

bool QuadSPISendCmd(SPIDev_t * const, uint8_t Cmd, uint32_t Addr,
					uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	(void)AddrLen;
	(void)DataLen;

	s_QCmd = Cmd;
	s_QAddr = Addr;

	switch (Cmd)
	{
		case FLASH_CMD_WRENABLE:  s_QWel = true;  break;
		case FLASH_CMD_WRDISABLE: s_QWel = false; break;

		case FLASH_CMD_RESET_DEVICE: s_QRstCnt++; break;

		case FLASH_CMD_SECTOR_ERASE:
			if (!s_QWel) { s_QVioEraseNoWel++; break; }
			for (uint32_t i = 0; i < 4096u; i++)
			{
				s_QMem[(Addr + i) % QSPI_SIZE] = 0xFF;
			}
			s_QEraseCnt++;
			s_QWel = false;
			break;

		case FLASH_CMD_BULK_ERASE:
			if (!s_QWel) { s_QVioEraseNoWel++; break; }
			memset(s_QMem, 0xFF, sizeof(s_QMem));
			s_QWel = false;
			break;

		default: break;
	}

	if (Cmd == 0x6B)
	{
		s_QLastRdDummy = DummyCycle;	// the part's dummies, if they made it
	}

	return true;
}

static bool QspiStartRx(DevIntrf_t *, uint32_t) { return true; }
static void QspiStopRx(DevIntrf_t *) {}
static bool QspiStartTx(DevIntrf_t *, uint32_t) { return true; }

static int QspiRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (s_QCmd == FLASH_CMD_READID)
	{
		for (int i = 0; i < Len; i++)
		{
			pBuff[i] = (uint8_t)(QSPI_ID >> (8 * i));
		}
		return Len;
	}
	if (s_QCmd == FLASH_CMD_READSTATUS)
	{
		uint8_t st = s_QBp;
		if (s_QWel) { st |= 0x02u; }
		for (int i = 0; i < Len; i++) { pBuff[i] = st; }
		return Len;
	}

	// A data read: the address arrived as a phase.
	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = s_QMem[(s_QAddr + i) % QSPI_SIZE];
	}
	return Len;
}

static int QspiTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	if (s_QCmd == FLASH_CMD_WRSR)
	{
		if (Len >= 1) { s_QBp = pData[0]; }
		s_QWrsrCnt++;
		s_QWel = false;
		return Len;
	}

	// A program: the command and address arrived as phases.
	s_QLastWrCmd = s_QCmd;
	if (!s_QWel)
	{
		s_QVioNoWel++;
		return Len;
	}
	for (int i = 0; i < Len; i++)
	{
		// Programming clears bits only.
		s_QMem[(s_QAddr + i) % QSPI_SIZE] &= pData[i];
	}
	s_QWel = false;
	return Len;
}
static int QspiTxSrData(DevIntrf_t *d, const uint8_t *p, int n)
{
	return QspiTxData(d, p, n);
}
static void QspiStopTx(DevIntrf_t *) {}

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
	CHECK(s_NorRstCnt == 1, "reset sent once at init, before the id probe");
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
	CHECK(m2.SetWriteProtect(true) == 0, "flash: protect");
	CHECK((m2.ReadStatus() & 0x3Cu) == 0x3Cu, "flash: protect bits set");
	CHECK(m2.SetWriteProtect(false) == 0, "flash: unprotect");
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
	CHECK(win.SetWriteProtect(true) == -EPERM,
		  "flash: write protect refused on a window");
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
	CHECK(mem.SetWriteProtect(true) == -ENOTSUP,
		  "eeprom: no pin, unsupported");

	NvmCfg_t pincfg = EepCfg();
	pincfg.WrProtPin = { 0, 12, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
	Nvm pinned;
	pinned.Init(pincfg, &Bus);
	CHECK(pinned.SetWriteProtect(true) == 0,
		  "eeprom: protect through the pin");
	CHECK(pinned.SetWriteProtect(false) == 0,
		  "eeprom: unprotect through the pin");
}

static void TestFram(MockIntrf &Bus)
{
	FramPowerOn();

	Nvm mem;
	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = 0;
	cfg.TotalSize = FRAM_SIZE;
	cfg.EraseSize = 0;					// direct read write
	cfg.PageSize = FRAM_PAGE;
	cfg.AddrSize = FRAM_ADDR;
	cfg.DevId = FRAM_ID;
	cfg.DevIdSize = 3;
	cfg.WrProtMask = 0x0C;				// BP0..BP1
	cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };

	CHECK(mem.Init(cfg, &Bus), "fram init");
	CHECK(s_FramRstCnt == 0, "fram: no reset for a direct medium");
	CHECK(mem.ReadId(3) == FRAM_ID, "fram id");
	CHECK(mem.EraseSize() == 0, "fram has no erase unit");

	RunCommon(mem, "SPI FRAM", FRAM_SIZE, FRAM_PAGE);

	// Every program went through the derived write latch.
	CHECK(s_FramVioNoWel == 0, "fram: every program held the write latch");

	// Overwrites in place, no erase between.
	uint8_t a[8], b[8], rd[8];
	memset(a, 0xAA, sizeof(a));
	memset(b, 0x55, sizeof(b));
	mem.Write(100, a, sizeof(a));
	mem.Write(100, b, sizeof(b));
	mem.Read(100, rd, sizeof(rd));
	CHECK(memcmp(b, rd, sizeof(b)) == 0, "fram: overwrites without an erase");
	CHECK(mem.Erase(0, 64) == 0, "fram: erase is a no-op success");

	// Status register block protect on a direct serial part.
	CHECK(mem.SetWriteProtect(true) == 0,
		  "fram: protect through the status bits");
	CHECK(s_FramWrsrCnt >= 1, "fram: the write status command was used");
	CHECK(mem.SetWriteProtect(false) == 0,
		  "fram: unprotect through the status bits");
}

static void TestQspi(MockIntrf &Bus)
{
	QspiPowerOn();

	Nvm mem;
	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.TotalSize = QSPI_SIZE;
	cfg.EraseSize = 4096;
	cfg.PageSize = QSPI_PAGE;
	cfg.AddrSize = QSPI_ADDR;
	cfg.DevId = QSPI_ID;
	cfg.DevIdSize = 3;
	cfg.WrProtMask = 0x3C;
	cfg.RdCmd = { 0x6B, 8 };			// the part's quad read, 8 dummies
	cfg.WrCmd = { 0x32, 0 };			// the part's quad page program
	cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };

	CHECK(mem.Init(cfg, &Bus), "qspi init");
	CHECK(s_QSetMemSize == QSPI_SIZE, "qspi: device size handed to the interface");
	CHECK(s_QRstCnt == 1, "qspi: reset went as a phase, before the id probe");
	CHECK(mem.ReadId(3) == QSPI_ID, "qspi id");

	RunCommon(mem, "Quad SPI NOR", QSPI_SIZE, QSPI_PAGE);

	// The design's central claim: the part's own quad command and its dummy
	// cycles reach the interface as phases.
	CHECK(s_QLastRdDummy == 8, "qspi: the part's dummy cycles reach the interface");
	CHECK(s_QLastWrCmd == 0x32, "qspi: the part's program command is the one used");
	CHECK(s_QVioNoWel == 0, "qspi: every program held the latch");
	CHECK(s_QVioEraseNoWel == 0, "qspi: every erase held the latch");

	// Erase and chip erase as address and command phases.
	uint8_t rd[16];
	CHECK(mem.Erase(0, 4096) == 0, "qspi: erase one unit");
	mem.Read(0, rd, sizeof(rd));
	bool ff = true;
	for (size_t i = 0; i < sizeof(rd); i++) { if (rd[i] != 0xFF) { ff = false; } }
	CHECK(ff, "qspi: erased unit reads all ones");
	CHECK(mem.MassErase() == 0, "qspi: chip erase as a phase");

	CHECK(mem.SetWriteProtect(true) == 0,
		  "qspi: protect through the status bits");
	CHECK(s_QWrsrCnt >= 1, "qspi: the write status command was used");
	CHECK(mem.SetWriteProtect(false) == 0,
		  "qspi: unprotect through the status bits");
}

// ---------------------------------------------------------------------------
// Asynchronous mode. Write and Erase return once the transfer is done; the
// medium keeps working. One completion event per call, reported by
// Complete() or by the next operation.
// ---------------------------------------------------------------------------

static uint32_t s_EvtCnt;
static NVM_EVT s_LastEvt;
static uint64_t s_LastOff;
static uint32_t s_LastLen;
static int s_LastRes;

static void AsyncEvtHandler(Nvm * const, NVM_EVT Evt, uint64_t Off,
							uint32_t Len, int Res)
{
	s_EvtCnt++;
	s_LastEvt = Evt;
	s_LastOff = Off;
	s_LastLen = Len;
	s_LastRes = Res;
}

static Nvm *s_pAppNvm;

// The application owned interface callback: forwards events to the driver,
// the path the header documents for an application that keeps EvtCB.
static int AppIntrfCB(DevIntrf_t * const, DEVINTRF_EVT EvtId, uint8_t *, int Len)
{
	if (s_pAppNvm != nullptr)
	{
		(void)Len;
		s_pAppNvm->IntrfEvent(EvtId);
	}

	return 0;
}

static void TestNorAsync(MockIntrf &Bus)
{
	NorPowerOn();
	s_EvtCnt = 0;
	s_LastEvt = NVM_EVT_NONE;
	s_LastOff = 0;
	s_LastLen = 0;
	s_LastRes = -1;

	printf("--- NOR flash, interrupt driven\n");

	Nvm mem;
	NvmCfg_t cfg = NorCfg();

	cfg.bIntEn = true;
	cfg.EvtHandler = AsyncEvtHandler;

	// The interface's callback belongs to whoever built the interface, so the
	// application points it at this memory. Nvm::Init does not take it.
	Bus.vDev.EvtCB = AppIntrfCB;
	s_pAppNvm = &mem;
	CHECK(mem.Init(cfg, &Bus), "intdrv: init");
	CHECK(Bus.vDev.EvtCB == AppIntrfCB,
		  "intdrv: init left the application's callback alone");

	// Start and return: a page crossing write returns after issuing only
	// the first chunk; the second page has not been touched yet.
	uint8_t big[NOR_PAGE + 4];
	uint8_t rd[NOR_PAGE + 8];

	for (size_t i = 0; i < sizeof(big); i++) { big[i] = (uint8_t)(i * 7 + 1); }

	CHECK(mem.Write(NOR_PAGE - 4, big, NOR_PAGE + 4) == (int)(NOR_PAGE + 4),
		  "intdrv: write returns after the first chunk");
	CHECK(s_Nor[NOR_PAGE] == 0xFF, "intdrv: later chunks not issued yet");
	CHECK(s_EvtCnt == 0, "intdrv: no event before completion");
	CHECK(mem.IsBusy(), "intdrv: IsBusy while the operation runs");
	CHECK(mem.Sync() == 0, "intdrv: Sync drains the operation");
	CHECK(s_EvtCnt == 1 && s_LastEvt == NVM_EVT_WRITE_DONE
		  && s_LastOff == NOR_PAGE - 4 && s_LastLen == NOR_PAGE + 4
		  && s_LastRes == 0,
		  "intdrv: one event with the call's range");
	CHECK(mem.IsBusy() == false, "intdrv: idle after completion");
	CHECK(mem.Read(NOR_PAGE - 4, rd, NOR_PAGE + 4) == (int)(NOR_PAGE + 4)
		  && memcmp(rd, big, NOR_PAGE + 4) == 0,
		  "intdrv: every chunk landed");

	// The polling service alone advances the operation: IsBusy clears by
	// itself as the medium finishes, and the completing step reports.
	CHECK(mem.Write(0, big, 16) == 16, "intdrv: single chunk write");

	int spins = 0;

	while (mem.IsBusy() && spins < 20) { spins++; }
	CHECK(spins > 0 && spins < 20, "intdrv: IsBusy clears as the medium finishes");
	CHECK(s_EvtCnt == 2 && s_LastLen == 16, "intdrv: the completing step reported");

	// The next operation drains the previous one first.
	CHECK(mem.Write(100, big, 8) == 8, "intdrv: chained write");
	CHECK(mem.Read(100, rd, 8) == 8, "intdrv: read runs after the write");
	CHECK(s_EvtCnt == 3 && s_LastOff == 100 && s_LastLen == 8,
		  "intdrv: the read drained and reported it");
	CHECK(memcmp(rd, big, 8) == 0, "intdrv: chained data landed");

	// Erase returns after the first unit; the second is untouched until the
	// service reaches it.
	CHECK(mem.Write(NOR_SECT, big, 4) == 4, "intdrv: marker in the second unit");
	CHECK(mem.Sync() == 0, "intdrv: marker done");
	CHECK(mem.Erase(0, 2 * NOR_SECT) == 0, "intdrv: erase returns after one unit");
	CHECK(s_Nor[NOR_SECT] != 0xFF, "intdrv: second unit not erased yet");
	CHECK(mem.Sync() == 0 && s_LastEvt == NVM_EVT_ERASE_DONE
		  && s_LastOff == 0 && s_LastLen == 2 * NOR_SECT,
		  "intdrv: one erase event with the call's range");
	CHECK(s_Nor[NOR_SECT] == 0xFF, "intdrv: both units erased");

	// Timeout: the medium never finishes. The next call drains, reports the
	// failure as an error event, and does not start.
	CHECK(mem.Write(64, big, 4) == 4, "intdrv: write that will not finish");
	s_NorBusyCnt = 0x7FFFFFFF;

	uint32_t evts = s_EvtCnt;

	CHECK(mem.Write(80, big, 4) == -ETIMEDOUT,
		  "intdrv: a failed previous operation stops the next call");
	CHECK(s_EvtCnt == evts + 1 && s_LastEvt == NVM_EVT_ERROR
		  && s_LastRes == -ETIMEDOUT,
		  "intdrv: the failure was reported as an error event");
	s_NorBusyCnt = 0;
	CHECK(mem.Write(80, big, 4) == 4 && mem.Sync() == 0,
		  "intdrv: the retried call runs");
}


static void TestNorAsyncIntrf(MockIntrf &Bus)
{
	NorPowerOn();
	s_EvtCnt = 0;

	printf("--- NOR flash, interrupt driven interface\n");

	// The interface starts transfers and reports -1; data and completion
	// arrive through the event, the way the real ports behave.
	s_NorIntrfAsync = true;
	Bus.vDev.bIntEn = true;

	// Polling driver over the interrupt driven interface: the application
	// owns the interface callback and forwards events.
	Nvm mem;
	NvmCfg_t cfg = NorCfg();

	Bus.vDev.EvtCB = AppIntrfCB;
	s_pAppNvm = &mem;
	CHECK(mem.Init(cfg, &Bus), "aintrf: polling driver init");

	uint8_t wr[16];
	uint8_t rd[16];

	for (size_t i = 0; i < sizeof(wr); i++) { wr[i] = (uint8_t)(0xC0 + i); }
	CHECK(mem.Write(0, wr, 16) == 16,
		  "aintrf: polling write over a started transfer");
	CHECK(mem.Read(0, rd, 16) == 16 && memcmp(rd, wr, 16) == 0,
		  "aintrf: polling read, the data landed with the event");

	// Interrupt driven driver over the same interface. The application moves
	// its callback to the new memory; nothing installs itself.
	Nvm amem;

	s_pAppNvm = &amem;

	cfg.bIntEn = true;
	cfg.EvtHandler = AsyncEvtHandler;
	s_EvtCnt = 0;
	CHECK(amem.Init(cfg, &Bus), "aintrf: interrupt driven init");
	CHECK(amem.Write(64, wr, 16) == 16, "aintrf: write starts and returns");
	CHECK(amem.Sync() == 0 && s_EvtCnt == 1 && s_LastEvt == NVM_EVT_WRITE_DONE,
		  "aintrf: drained with one event");
	CHECK(amem.Read(64, rd, 16) == 16 && memcmp(rd, wr, 16) == 0,
		  "aintrf: readback over the started receive");

	s_NorIntrfAsync = false;
	Bus.vDev.bIntEn = false;
	// mem and amem unhook in their destructors at return.
}

// A memory that is not mapped at 0. The frame has to hold the address the
// part answers at, so an offset of 0 in the region has to land at the base.
static void TestBaseAddr(MockIntrf &Bus)
{
	NorPowerOn();

	printf("--- base address\n");

	NvmCfg_t cfg = NorCfg();
	cfg.BaseAddr = 0x1000;

	Nvm mem;
	CHECK(mem.Init(cfg, &Bus), "base: init");
	CHECK(mem.BaseAddress() == 0x1000, "base: reported back");

	uint8_t wr[4] = { 0x11, 0x22, 0x33, 0x44 };
	uint8_t rd[4] = { 0, 0, 0, 0 };

	CHECK(mem.Write(0, wr, sizeof(wr)) == (int)sizeof(wr), "base: write at offset 0");
	CHECK(memcmp(&s_Nor[0x1000], wr, sizeof(wr)) == 0,
		  "base: the data landed at the base, not at 0");
	CHECK(s_Nor[0] == 0xFF, "base: nothing was written at 0");
	CHECK(mem.Read(0, rd, sizeof(rd)) == (int)sizeof(rd) &&
		  memcmp(rd, wr, sizeof(wr)) == 0, "base: read comes back from the base");

	// A base that is not a whole write unit would break every program.
	NvmCfg_t bad = NorCfg();
	bad.BaseAddr = 0x1000;
	bad.WriteGran = 4;
	Nvm ok4;
	CHECK(ok4.Init(bad, &Bus), "base: aligned base with a write unit");
	bad.BaseAddr = 0x1002;
	Nvm bad4;
	CHECK(bad4.Init(bad, &Bus) == false, "base: unaligned base refused");
}

// A program that fails while IsBusy is doing the polling. The result has to
// survive that call, so Sync or the next operation still reports it.
static void TestHeldResult(MockIntrf &Bus)
{
	NorPowerOn();
	s_EvtCnt = 0;

	printf("--- held result\n");

	NvmCfg_t cfg = NorCfg();
	cfg.bIntEn = true;
	cfg.EvtHandler = AsyncEvtHandler;

	Nvm mem;
	CHECK(mem.Init(cfg, &Bus), "held: init");

	// A write over three pages, with the part refusing the second chunk. The
	// first chunk goes out from Write, so the failure happens on a later step,
	// and that step is driven by IsBusy rather than by the call that asked.
	static uint8_t wr[NOR_PAGE * 3];
	memset(wr, 0x5A, sizeof(wr));

	s_NorPgmOk = 1;
	CHECK(mem.Write(0, wr, sizeof(wr)) == (int)sizeof(wr), "held: write starts");

	// Step until the medium is done. IsBusy is what sees the refusal.
	int guard = 64;
	while (mem.IsBusy() && guard-- > 0) {}
	CHECK(guard > 0, "held: the operation ended");
	CHECK(s_EvtCnt == 1 && s_LastEvt == NVM_EVT_ERROR,
		  "held: the failure was reported as an event");

	// The call that observed the failure was IsBusy, which has no way to
	// report it. Sync still has to.
	CHECK(mem.Sync() == -EIO, "held: Sync reports the failure IsBusy saw");
	CHECK(mem.Sync() == 0, "held: reported once, then clean");

	s_NorPgmOk = 0xFFFFFFFFUL;
}

// A memory reached through its own controller. It answers to an address and
// nothing else, so the driver must send no command byte and none of the
// command traffic a chip on a bus needs.
static uint8_t s_Mem[16384];
static uint32_t s_MemEraseCnt;

// The erase a target port supplies. Nvm calls this rather than framing a
// command, because a memory reached through a controller has no bus to put
// one on. Overrides the weak answer in nvm.cpp, the way a real port does.
extern "C" int NvmMcuErase(uintptr_t Addr)
{
	if (Addr + 4096 > sizeof(s_Mem))
	{
		return -EINVAL;
	}

	s_MemEraseCnt++;
	memset(&s_Mem[Addr], 0xFF, 4096);

	return 0;
}
static std::vector<uint8_t> s_MemTx;
static uint32_t s_MemAddr;
static bool s_MemAddrSet;
static int s_MemCmdBytes;

static bool MemStartTx(DevIntrf_t *, uint32_t)
{
	s_MemTx.clear();
	s_MemAddrSet = false;

	return true;
}

static void MemStopTx(DevIntrf_t *) {}

// The address phase already latched where to read. Starting the read half of
// the transaction must not throw it away, the same way CRACEN keeps the offset
// latched by TxSrData across into RxData.
static bool MemStartRx(DevIntrf_t *, uint32_t)
{
	return true;
}

static void MemStopRx(DevIntrf_t *) {}

// The frame is the address, four bytes, nothing in front of it. Anything
// longer means the driver put a command byte on the wire.
static int MemTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++) { s_MemTx.push_back(pData[i]); }

	if (s_MemAddrSet == false)
	{
		if (s_MemTx.size() < 4)
		{
			return Len;
		}
		if (s_MemTx.size() > 4)
		{
			s_MemCmdBytes += (int)s_MemTx.size() - 4;
		}
		s_MemAddr = ((uint32_t)s_MemTx[0] << 24) | ((uint32_t)s_MemTx[1] << 16) |
					((uint32_t)s_MemTx[2] << 8) | s_MemTx[3];
		s_MemAddrSet = true;
		s_MemTx.clear();

		return Len;
	}

	for (int i = 0; i < Len; i++)
	{
		if (s_MemAddr < sizeof(s_Mem))
		{
			s_Mem[s_MemAddr++] &= pData[i];
		}
	}
	s_MemTx.clear();

	return Len;
}

static int MemRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (s_MemAddrSet == false)
	{
		// A status poll would land here. The driver must not make one.
		s_MemCmdBytes++;

		return 0;
	}

	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = s_MemAddr < sizeof(s_Mem) ? s_Mem[s_MemAddr++] : 0xFF;
	}

	return Len;
}

static void TestMemCtrl(void)
{
	printf("--- memory controller\n");

	memset(s_Mem, 0xFF, sizeof(s_Mem));
	s_MemCmdBytes = 0;

	// TxSrData is the address phase; this interface takes it the same way.
	MockIntrf bus(DEVINTRF_TYPE_MEMCTRL, MemStartRx, MemRxData, MemStopRx,
				  MemStartTx, MemTxData, MemTxData, MemStopTx);

	NvmCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.TotalSize = sizeof(s_Mem);
	cfg.EraseSize = 4096;
	cfg.PageSize = 64;
	cfg.WriteGran = 4;
	cfg.AddrSize = 4;
	cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };

	Nvm mem;
	CHECK(mem.Init(cfg, &bus), "memctrl: init");
	CHECK(s_MemCmdBytes == 0, "memctrl: no command traffic at init");

	uint8_t wr[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	uint8_t rd[8] = { 0 };

	CHECK(mem.Write(0x40, wr, sizeof(wr)) == (int)sizeof(wr), "memctrl: write");
	CHECK(s_MemCmdBytes == 0, "memctrl: no command byte on a program");
	CHECK(memcmp(&s_Mem[0x40], wr, sizeof(wr)) == 0, "memctrl: the data landed");

	CHECK(mem.Read(0x40, rd, sizeof(rd)) == (int)sizeof(rd) &&
		  memcmp(rd, wr, sizeof(wr)) == 0, "memctrl: read back");
	CHECK(s_MemCmdBytes == 0, "memctrl: no status poll and no command on a read");

	// A write bigger than one page, so the address is reframed per chunk.
	static uint8_t big[256];
	for (uint32_t i = 0; i < sizeof(big); i++) { big[i] = (uint8_t)(i ^ 0x5A); }
	CHECK(mem.Write(0x400, big, sizeof(big)) == (int)sizeof(big),
		  "memctrl: write across pages");
	CHECK(memcmp(&s_Mem[0x400], big, sizeof(big)) == 0,
		  "memctrl: every chunk landed at its own address");
	CHECK(s_MemCmdBytes == 0, "memctrl: still no command traffic");

	// A mass erase is a command and this medium has no command protocol, so
	// there is no way to ask for one. The chip erase opcode went out as a
	// lone byte, the interface held it as the first byte of a frame that was
	// never completed, and this answered 0 for an erase that never happened.
	// Erase reaches the port directly and never becomes a transfer, so no
	// command byte goes anywhere. Before, the interface read an opcode off
	// the frame to find the erase, which is the transport reading a command
	// set it has no business knowing.
	s_MemEraseCnt = 0;
	CHECK(mem.Erase(0x1000, 4096) == 0, "memctrl: erase through the port");
	CHECK(s_MemEraseCnt == 1, "memctrl: the port was asked exactly once");
	CHECK(s_MemCmdBytes == 0, "memctrl: erase sent no command");

	uint8_t er[4] = { 0 };
	CHECK(mem.Read(0x1000, er, sizeof(er)) == (int)sizeof(er) &&
		  er[0] == 0xFF && er[3] == 0xFF, "memctrl: the unit reads erased");

	CHECK(mem.MassErase() == -ENOTSUP, "memctrl: mass erase refused");
	CHECK(memcmp(&s_Mem[0x40], wr, sizeof(wr)) == 0,
		  "memctrl: mass erase left the data alone");
	CHECK(s_MemCmdBytes == 0, "memctrl: mass erase sent no command");

	// Kept and read from long after the call returns when the memory is
	// interrupt driven, so a null one has to be refused at the door.
	CHECK(mem.Write(0x40, nullptr, 4) == -EINVAL,
		  "memctrl: write refuses a null buffer");
	CHECK(mem.Read(0x40, nullptr, 4) == -EINVAL,
		  "memctrl: read refuses a null buffer");
	CHECK(mem.Write(0x40, nullptr, 0) == 0 && mem.Read(0x40, nullptr, 0) == 0,
		  "memctrl: nothing asked for is not an error");

	// The frame address is 32 bit the whole way down. A part mapped past
	// that would be programmed at a wrapped address.
	NvmCfg_t far = cfg;

	far.BaseAddr = 0xFFFFFF00ULL;

	Nvm farmem;
	CHECK(farmem.Init(far, &bus) == false,
		  "memctrl: a region past 32 bits is refused");
}

// A completion arrives while a read is open on the same interface. Nothing
// was handed to the interface at that moment, so it belongs to nothing and
// has to be discarded rather than ending whatever the driver is doing.
//
// Note the read cannot overlap an operation on purpose: Nvm::Read drains
// anything outstanding before it starts. So what is delivered here is a
// stray event, which is the case the transfer state has to survive.
static Nvm *s_pLateNvm;
static uint32_t s_LateAt;

static int NorRxDataLate(DevIntrf_t *pDev, uint8_t *pBuff, int Len)
{
	// Deliver the outstanding write completion in the middle of this read.
	if (s_pLateNvm != nullptr && s_LateAt > 0)
	{
		s_LateAt--;

		if (s_LateAt == 0)
		{
			// A completion arriving while a read is open. Nothing was handed
			// over, so it belongs to nothing and must be discarded.
			s_pLateNvm->IntrfEvent(DEVINTRF_EVT_COMPLETED);
		}
	}

	return NorRxData(pDev, pBuff, Len);
}

static void TestLateCompletion(MockIntrf &Bus)
{
	NorPowerOn();
	s_EvtCnt = 0;

	printf("--- completion during another transfer\n");

	NvmCfg_t cfg = NorCfg();
	cfg.bIntEn = true;
	cfg.EvtHandler = AsyncEvtHandler;

	Nvm mem;
	CHECK(mem.Init(cfg, &Bus), "late: init");

	uint8_t wr[8];
	memset(wr, 0x3C, sizeof(wr));

	// Leave a write running, then read while it is still outstanding. The
	// read arms its own transfer flags; the completion must not land on them.
	s_NorBusyCnt = 4;
	CHECK(mem.Write(0, wr, sizeof(wr)) == (int)sizeof(wr), "late: write starts");

	s_pLateNvm = &mem;
	s_LateAt = 1;
	Bus.vDev.RxData = NorRxDataLate;

	uint8_t rd[8] = { 0 };
	(void)mem.Read(0x200, rd, sizeof(rd));

	Bus.vDev.RxData = NorRxData;
	s_pLateNvm = nullptr;
	s_LateAt = 0;
	s_NorBusyCnt = 0;

	// The write must still finish. Before the chunk had its own completion
	// flags the read swallowed the event and this never returned.
	int guard = 1000;
	while (mem.IsBusy() && guard-- > 0) {}

	CHECK(guard > 0, "late: the write still finished");
	CHECK(mem.Sync() == 0, "late: nothing left to drain");
	CHECK(s_EvtCnt >= 1 && s_LastEvt == NVM_EVT_WRITE_DONE,
		  "late: the write reported done");

}

// TX_READY asks the handler for more data and TX_FIFO_EMPTY says a software
// FIFO drained. Neither says the transfer finished, and the nRF I2C master
// raises TX_READY on LASTTX, before the stop condition. Taking either as the
// end returns a chunk to the driver while the bus is still on it.
static void TestSpuriousComplete(MockIntrf &Bus)
{
	NorPowerOn();
	s_EvtCnt = 0;

	printf("--- only COMPLETED ends a transfer\n");

	s_NorIntrfAsync = true;
	s_NorHoldEvt = true;		// start it and report nothing
	Bus.vDev.bIntEn = true;
	Bus.vDev.EvtCB = nullptr;

	Nvm mem;
	NvmCfg_t cfg = NorCfg();

	cfg.bIntEn = true;
	cfg.EvtHandler = AsyncEvtHandler;
	CHECK(mem.Init(cfg, &Bus), "spurious: init");

	uint8_t wr[8];
	memset(wr, 0x5A, sizeof(wr));

	CHECK(mem.Write(0, wr, sizeof(wr)) == (int)sizeof(wr),
		  "spurious: the write started");
	CHECK(mem.IsBusy() && s_NorHeld,
		  "spurious: the chunk is with the interface");

	// A driver that took the event for the end of its transfer moves on to
	// the medium and reads the status register. Both states look busy from
	// outside, so the bus is what tells them apart.
	s_NorRxWhileHeld = 0;

	mem.IntrfEvent(DEVINTRF_EVT_TX_READY);
	(void)mem.IsBusy();
	CHECK(s_NorRxWhileHeld == 0 && s_EvtCnt == 0,
		  "spurious: TX_READY did not end the transfer");

	mem.IntrfEvent(DEVINTRF_EVT_TX_FIFO_EMPTY);
	(void)mem.IsBusy();
	CHECK(s_NorRxWhileHeld == 0 && s_EvtCnt == 0,
		  "spurious: TX_FIFO_EMPTY did not end the transfer");

	s_NorHeld = false;
	mem.IntrfEvent(DEVINTRF_EVT_COMPLETED);
	CHECK(mem.Sync() == 0, "spurious: COMPLETED ended it");
	CHECK(s_EvtCnt == 1 && s_LastEvt == NVM_EVT_WRITE_DONE,
		  "spurious: one event, the write");

	s_NorHoldEvt = false;
	s_NorIntrfAsync = false;
	Bus.vDev.bIntEn = false;

	uint8_t rd[8] = { 0 };
	CHECK(mem.Read(0, rd, sizeof(rd)) == (int)sizeof(rd) &&
		  memcmp(rd, wr, sizeof(wr)) == 0,
		  "spurious: the data landed");
}



int main(void)
{
	MockIntrf nor(DEVINTRF_TYPE_SPI, NorStartRx, NorRxData, NorStopRx,
				  NorStartTx, NorTxData, NorTxSrData, NorStopTx);
	MockIntrf fram(DEVINTRF_TYPE_SPI, FramStartRx, FramRxData, FramStopRx,
				   FramStartTx, FramTxData, FramTxSrData, FramStopTx);
	MockIntrf eep(DEVINTRF_TYPE_I2C, EepStartRx, EepRxData, EepStopRx,
				  EepStartTx, EepTxData, EepTxSrData, EepStopTx);

	TestNorFlash(nor);
	TestEeprom(eep);
	TestFram(fram);

	MockIntrf qspi(DEVINTRF_TYPE_QSPI, QspiStartRx, QspiRxData, QspiStopRx,
				   QspiStartTx, QspiTxData, QspiTxSrData, QspiStopTx);
	// The phased path reaches the interface through the SPI handle; wire the
	// handle to the same handlers.
	memcpy(&s_QDev.DevIntrf, &qspi.vDev, sizeof(DevIntrf_t));
	s_QDev.DevIntrf.pDevData = &s_QDev;
	atomic_flag_clear(&s_QDev.DevIntrf.bBusy);
	qspi.vDev.pDevData = &s_QDev;
	TestQspi(qspi);
	TestNorAsync(nor);
	TestBaseAddr(nor);
	TestNorAsyncIntrf(nor);
	TestHeldResult(nor);
	TestMemCtrl();
	TestLateCompletion(nor);
	TestSpuriousComplete(nor);

	printf("\nChecks run: %d\n", g_Checks);
	if (g_Fail == 0)
	{
		printf("RESULT: ALL PASS\n");
		return 0;
	}
	printf("RESULT: %d FAILURES\n", g_Fail);
	return 1;
}
