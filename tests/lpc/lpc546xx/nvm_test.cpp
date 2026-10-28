// Host regression of the LPC546xx internal flash port (nvm_lpc546xx.cpp)
// under the generic Nvm driver, with a model of the flash and the IAP ROM.
//
// Model: the flash is a host array, 0xFF erased. The IAP calls run with
// interrupts masked. Erase page and copy RAM to flash need the sector of the
// page prepared by the call before, a prepare serves one operation. Copy RAM
// to flash takes 256 bytes, a destination on a 256 byte boundary, a word
// aligned source outside the flash, and a page erased since its last
// program: a page is programmed once between erases. A test can make the
// next erase or program fail, or leave a page with a wrong byte.
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "LPC546xx.h"
#include "lpc_iap.h"

uint32_t g_IrqMasked = 0;
uint32_t SystemCoreClock = 220000000;

alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];

#define FLASH_SIZE		FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES
#define SECT_SIZE		FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES
#define PAGE_SIZE		FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES
#define NB_PAGE			(FLASH_SIZE / PAGE_SIZE)

alignas(4) static uint8_t s_Flash[FLASH_SIZE];
static bool s_Programmed[NB_PAGE];
static int s_Prep = -1;
static int s_Erases = 0, s_Programs = 0, s_Preps = 0;
static IAPSTATUS s_FailErase = IAPSTATUS_SUCCESS, s_FailProg = IAPSTATUS_SUCCESS;
static bool s_Corrupt = false;
static int s_FailPrep = 0;		// The prepare that fails, 1 the next one
static int s_EraseLeave = -1;	// Byte the next erase leaves at 0

extern "C" IAPSTATUS IAPPrepSectWrite(int StartSect, int EndSect)
{
	assert(g_IrqMasked);
	assert(StartSect == EndSect && StartSect >= 0 && StartSect < FLASH_SIZE / SECT_SIZE);
	s_Preps++;
	if (s_FailPrep > 0 && --s_FailPrep == 0)
	{
		return IAPSTATUS_INVALID_SECT;
	}
	s_Prep = StartSect;
	return IAPSTATUS_SUCCESS;
}

extern "C" IAPSTATUS IAPErasePage(int StartPage, int EndPage)
{
	assert(g_IrqMasked);
	assert(StartPage == EndPage && StartPage >= 0 && StartPage < (int)NB_PAGE);
	assert(s_Prep == StartPage * PAGE_SIZE / SECT_SIZE);
	s_Prep = -1;
	if (s_FailErase != IAPSTATUS_SUCCESS)
	{
		IAPSTATUS r = s_FailErase;

		s_FailErase = IAPSTATUS_SUCCESS;
		return r;
	}
	memset(&s_Flash[StartPage * PAGE_SIZE], 0xFF, PAGE_SIZE);
	if (s_EraseLeave >= 0)
	{
		s_Flash[StartPage * PAGE_SIZE + s_EraseLeave] = 0;
		s_EraseLeave = -1;
	}
	s_Programmed[StartPage] = false;
	s_Erases++;
	return IAPSTATUS_SUCCESS;
}

extern "C" IAPSTATUS IAPCopyRamToFlash(uint8_t *pDst, uint8_t *pSrc, int Size)
{
	uintptr_t dst = (uintptr_t)pDst;

	assert(g_IrqMasked);
	assert(Size == PAGE_SIZE && (dst % PAGE_SIZE) == 0 && dst < FLASH_SIZE);
	assert(((uintptr_t)pSrc & 3) == 0);
	assert(pSrc + Size <= s_Flash || pSrc >= s_Flash + FLASH_SIZE);
	assert(s_Prep == (int)(dst / SECT_SIZE));
	s_Prep = -1;
	assert(!s_Programmed[dst / PAGE_SIZE]);
	for (int i = 0; i < Size; i++)
	{
		s_Flash[dst + i] &= pSrc[i];
	}
	s_Programmed[dst / PAGE_SIZE] = true;
	// An error status, the data written or not
	if (s_FailProg != IAPSTATUS_SUCCESS)
	{
		IAPSTATUS r = s_FailProg;

		s_FailProg = IAPSTATUS_SUCCESS;
		return r;
	}
	if (s_Corrupt)
	{
		s_Flash[dst + 7] ^= 0x10;
		s_Corrupt = false;
	}
	s_Programs++;
	return IAPSTATUS_SUCCESS;
}

#undef GPIO
#define GPIO ((GPIO_Type *)s_GpioMem)
#define LPC546XX_NVM_MEM(Addr)		(&s_Flash[(uintptr_t)(Addr)])

// The generic Nvm (src/storage/nvm.cpp), device.cpp and device_intrf.cpp are
// built as their own objects: nvm.cpp defines the weak NvmMcuErase the port
// replaces.
#include "../../../ARM/NXP/LPC546xx/src/nvm_lpc546xx.cpp"
#include "coredev/spi.h"

// Used by nvm.cpp for QSPI parts and the write protect pin, neither here
void QuadSPISetMemSize(SPIDev_t * const, uint32_t) {}
bool QuadSPISendCmd(SPIDev_t * const, uint8_t, uint32_t, uint8_t, uint32_t, uint8_t) { assert(false); return false; }
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) { assert(false); }
void ModelDelayUs(uint32_t) {}

static int Arbiter(NvmIntrfOp_t *) { return -EBUSY; }

static void Pattern(uint8_t *p, uint32_t Len, uint8_t Seed)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		p[i] = (uint8_t)(Seed + i * 7);
	}
}

int main()
{
	memset(s_Flash, 0xFF, sizeof(s_Flash));

	NvmCfg_t cfg = {};

	NvmMcuCfg(cfg);
	assert(cfg.BaseAddr == 0 && cfg.TotalSize == FLASH_SIZE && cfg.EraseSize == PAGE_SIZE);
	assert(cfg.PageSize == PAGE_SIZE && cfg.WriteGran == 1 && cfg.AddrSize == 4);
	assert(cfg.SectorSize == SECT_SIZE && NvmMcuCeiling() == FLASH_SIZE);

	NvmIntrf intrf;
	Nvm mem;

	assert(!intrf.Init(nullptr, true));
	assert(intrf.Init());
	assert(intrf.Type() == DEVINTRF_TYPE_MEMCTRL);

	// Region: the last 4 pages of sector 14 and 4 pages of sector 15
	const uint32_t region = 15 * SECT_SIZE - 4 * PAGE_SIZE;

	assert(mem.Init(cfg, &intrf, region, 8 * PAGE_SIZE));

	// Writes into erased pages, across a page and a sector boundary
	uint8_t data[1100], back[1100];

	Pattern(data, sizeof(data), 3);
	int erases = s_Erases;

	assert(mem.Write(31, data, sizeof(data)) == (int)sizeof(data));
	assert(memcmp(&s_Flash[region + 31], data, sizeof(data)) == 0);
	// Pages 0 to 4 of the region, each erased then programmed once
	assert(s_Programs == 5 && s_Erases == erases + 5);
	for (uint32_t i = 0; i < 31; i++)
	{
		assert(s_Flash[region + i] == 0xFF);
	}
	assert(mem.Read(31, back, sizeof(back)) == (int)sizeof(back));
	assert(memcmp(back, data, sizeof(data)) == 0);
	assert(!g_IrqMasked);

	// A byte in the middle: its page keeps the other bytes
	uint8_t b = 0x5A;

	assert(mem.Write(300, &b, 1) == 1);
	assert(s_Flash[region + 300] == 0x5A && s_Programs == 6);
	data[300 - 31] = 0x5A;
	assert(memcmp(&s_Flash[region + 31], data, sizeof(data)) == 0);
	// Bits set back to one: the page rewrites in place
	b = 0xFF;
	assert(mem.Write(300, &b, 1) == 1 && s_Flash[region + 300] == 0xFF);
	data[300 - 31] = 0xFF;
	// The same content again programs nothing
	int progs = s_Programs;

	assert(mem.Write(31, data, sizeof(data)) == (int)sizeof(data) && s_Programs == progs);

	// The source in the flash itself
	assert(mem.Write(1780, &s_Flash[region + 31], 64) == 64);
	assert(memcmp(&s_Flash[region + 1780], data, 64) == 0);

	// Erase of pages, checked blank
	assert(mem.Erase(0, 2 * PAGE_SIZE) == 0);
	for (uint32_t i = 0; i < 2 * PAGE_SIZE; i++)
	{
		assert(s_Flash[region + i] == 0xFF);
	}
	assert(s_Flash[region + 2 * PAGE_SIZE] == data[2 * PAGE_SIZE - 31]);
	assert(mem.Erase(1, PAGE_SIZE) == -EINVAL);
	assert(NvmMcuErase(region + 1) == -EINVAL && NvmMcuErase(FLASH_SIZE) == -EINVAL);

	// Failures are reported, not hidden. Each write has new content.
	s_FailProg = IAPSTATUS_DST_ADDR_ERROR;
	assert(mem.Write(0, data, 16) < 0);
	data[0]++;
	s_FailErase = IAPSTATUS_BUSY;
	assert(mem.Write(0, data, 16) < 0);
	data[0]++;
	s_FailPrep = 1;
	assert(mem.Write(0, data, 16) < 0);
	data[0]++;
	// The prepare of the program, after the erase
	s_FailPrep = 2;
	assert(mem.Write(0, data, 16) < 0);
	data[0]++;
	s_Corrupt = true;
	assert(mem.Write(0, data, 16) < 0);
	data[0]++;
	s_FailErase = IAPSTATUS_BUSY;
	assert(NvmMcuErase(region) == -EIO);
	// The status counts, also on a page that reads blank
	assert(NvmMcuErase(region + 6 * PAGE_SIZE) == 0);
	s_FailErase = IAPSTATUS_BUSY;
	assert(NvmMcuErase(region + 6 * PAGE_SIZE) == -EIO);
	s_FailPrep = 1;
	assert(NvmMcuErase(region) == -EIO);
	// An erase that leaves a byte: the first, then one inside the page
	s_EraseLeave = 0;
	assert(NvmMcuErase(region) == -EIO);
	s_EraseLeave = 100;
	assert(NvmMcuErase(region) == -EIO);
	s_EraseLeave = 255;
	assert(mem.Write(0, data, 16) < 0);
	assert(!g_IrqMasked);
	// Works again after them
	assert(mem.Write(0, data, 16) == 16 && memcmp(&s_Flash[region], data, 16) == 0);

	// Out of the flash
	assert(mem.Read(8 * PAGE_SIZE, back, 4) < 0);
	assert(mem.Write(8 * PAGE_SIZE - 2, data, 4) < 0);

	// A registered arbiter: refused, the port has none to call
	NvmIntrfStat_t st;

	NvmIntrfGetStat(&st);
	uint32_t busy = st.Busy;

	NvmIntrfSetArbiter(Arbiter);
	assert(mem.Write(0, data, 4) < 0);
	assert(mem.Read(0, back, 4) < 0);
	assert(NvmMcuErase(region) == -EBUSY);
	NvmIntrfSetArbiter(nullptr);
	NvmIntrfGetStat(&st);
	assert(st.Busy > busy && st.Ops > 0 && st.Direct == st.Ops);

	// One operation for each page program and each erase
	uint32_t ops = st.Ops;

	data[0] ^= 0xFF;
	assert(mem.Write(0, data, 4) == 4);
	NvmIntrfGetStat(&st);
	assert(st.Ops == ops + 1 && st.Direct == st.Ops);
	assert(NvmMcuErase(region) == 0);
	NvmIntrfGetStat(&st);
	assert(st.Ops == ops + 2 && st.Direct == st.Ops);

	// The port alone: frames as Nvm sends them, opcode and range checked
	DevIntrf_t *dev = NvmMcuDevIntrf();
	uint8_t fr[5];
	auto frame = [&fr](uint8_t Op, uint32_t Addr) {
		fr[0] = Op;
		fr[1] = (uint8_t)(Addr >> 24);
		fr[2] = (uint8_t)(Addr >> 16);
		fr[3] = (uint8_t)(Addr >> 8);
		fr[4] = (uint8_t)Addr;
	};

	// Read: the last two bytes of the flash, not three
	frame(0x03, FLASH_SIZE - 2);
	assert(NvmStartTx(dev, 0) && NvmTxData(dev, fr, 5) == 5);
	assert(NvmRxData(dev, back, 3) == 0 && NvmRxData(dev, back, 2) == 2);
	assert(back[0] == s_Flash[FLASH_SIZE - 2] && back[1] == s_Flash[FLASH_SIZE - 1]);
	// Two reads in one transfer follow each other
	frame(0x03, region + 0x40);
	memset(&s_Flash[region + 0x40], 0x11, 2);
	s_Flash[region + 0x42] = 0x22;
	assert(NvmStartTx(dev, 0) && NvmTxData(dev, fr, 5) == 5);
	assert(NvmRxData(dev, back, 2) == 2 && NvmRxData(dev, back, 1) == 1 && back[0] == 0x22);
	// Not a read opcode
	frame(0x02, region);
	assert(NvmStartTx(dev, 0) && NvmTxData(dev, fr, 5) == 5 && NvmRxData(dev, back, 1) == 0);

	// Write: a read opcode or past the end takes the frame only
	uint8_t w[300];

	Pattern(w, sizeof(w), 9);
	frame(0x03, region);
	assert(NvmStartTx(dev, 0) && NvmTxData(dev, fr, 5) == 5 && NvmTxData(dev, w, 4) == 0);
	frame(0x02, FLASH_SIZE - 2);
	assert(NvmStartTx(dev, 0) && NvmTxData(dev, fr, 5) == 5 && NvmTxData(dev, w, 3) == 0);
	// Frame and data in one call, across a page boundary
	uint8_t fw[5 + 300];

	frame(0x02, region + 100);
	memcpy(fw, fr, 5);
	memcpy(fw + 5, w, 300);
	assert(NvmStartTx(dev, 0) && NvmTxData(dev, fw, sizeof(fw)) == (int)sizeof(fw));
	assert(memcmp(&s_Flash[region + 100], w, 300) == 0);

	assert(mem.Write(0, data, 4) == 4);

	// The whole flash as one Nvm: the last page
	Nvm all;

	assert(all.Init(cfg, &intrf));
	assert(all.Write(FLASH_SIZE - 3, data, 3) == 3 && memcmp(&s_Flash[FLASH_SIZE - 3], data, 3) == 0);
	assert(all.Read(FLASH_SIZE - 3, back, 3) == 3 && memcmp(back, data, 3) == 0);
	assert(all.Write(FLASH_SIZE - 2, data, 3) < 0);

	printf("nvm: PASS\n");
	return 0;
}
