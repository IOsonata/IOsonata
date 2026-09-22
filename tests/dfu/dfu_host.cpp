// Host support for the DFU tests. See dfu_host.h.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#include "dfu_host.h"
#include "coredev/iopincfg.h"
#include "coredev/spi.h"

static uint8_t *s_Flash = nullptr;
static HostMem s_Kind = HOST_MEM_NOR;

int g_HostNorViolations = 0;
int g_HostFailAfter = -1;
int g_HostMemOps = 0;
jmp_buf g_HostPowerJmp;
jmp_buf g_HostStartJmp;
uintptr_t g_HostStartAddr = 0;
jmp_buf g_HostResetJmp;

// Units programmed since their last erase, ECC16 and BIG256.
static std::vector<bool> s_Programmed;

const char *HostMemName(HostMem Kind)
{
	switch (Kind)
	{
		case HOST_MEM_NOR:		return "NOR";
		case HOST_MEM_RRAM:		return "RRAM";
		case HOST_MEM_ECC16:	return "ECC16";
		case HOST_MEM_BIG256:	return "BIG256";
		case HOST_MEM_SECT:		return "SECT";
	}
	return "?";
}

uint32_t HostUnit(void)
{
	switch (s_Kind)
	{
		case HOST_MEM_ECC16:	return 16;
		case HOST_MEM_BIG256:	return 256;
		default:				return 4;
	}
}

static bool HostOnce(void)
{
	return s_Kind == HOST_MEM_ECC16 || s_Kind == HOST_MEM_BIG256;
}

void HostFlashInit(HostMem Kind)
{
	s_Kind = Kind;
	if (s_Flash == nullptr)
	{
		void *p = mmap((void *)HOST_FLASH_BASE, HOST_FLASH_SIZE,
					   PROT_READ | PROT_WRITE,
					   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
		if (p != (void *)HOST_FLASH_BASE)
		{
			fprintf(stderr, "cannot map test memory at 0x%lx\n",
					(unsigned long)HOST_FLASH_BASE);
			exit(2);
		}
		s_Flash = (uint8_t *)p;
	}
	HostFlashErase();
}

void HostFlashErase(void)
{
	memset(s_Flash, 0xFF, HOST_FLASH_SIZE);
	s_Programmed.assign(HOST_FLASH_SIZE / 4, false);
	g_HostNorViolations = 0;
}

uint8_t *HostFlash(uintptr_t Addr)
{
	if (Addr < HOST_FLASH_BASE || Addr >= HOST_FLASH_BASE + HOST_FLASH_SIZE)
	{
		fprintf(stderr, "address 0x%lx outside test memory\n",
				(unsigned long)Addr);
		abort();
	}
	return (uint8_t *)Addr;
}

HostMem HostFlashKind(void)
{
	return s_Kind;
}

// A write the way the memory kind does it.
static void HostProgram(uintptr_t Addr, const uint8_t *pSrc, uint32_t Len)
{
	uint8_t *d = HostFlash(Addr);
	(void)HostFlash(Addr + Len - 1);

	if (HostOnce())
	{
		uint32_t u = HostUnit();

		if ((Addr % u) != 0 || (Len % u) != 0)
		{
			g_HostNorViolations++;
		}
		for (uint32_t o = 0; o < Len; o += u)
		{
			size_t i = (Addr + o - HOST_FLASH_BASE) / u;
			if (s_Programmed[i])
			{
				g_HostNorViolations++;
			}
			s_Programmed[i] = true;
		}
	}

	for (uint32_t i = 0; i < Len; i++)
	{
		if (s_Kind == HOST_MEM_NOR)
		{
			if ((d[i] & pSrc[i]) != pSrc[i])
			{
				g_HostNorViolations++;
			}
			d[i] &= pSrc[i];
		}
		else
		{
			d[i] = pSrc[i];
		}
	}
}

std::vector<uint8_t> HostReadFile(const std::string &Path)
{
	std::vector<uint8_t> v;
	FILE *f = fopen(Path.c_str(), "rb");

	if (f == nullptr)
	{
		fprintf(stderr, "cannot open %s\n", Path.c_str());
		exit(2);
	}
	uint8_t b[4096];
	size_t n;
	while ((n = fread(b, 1, sizeof(b), f)) > 0)
	{
		v.insert(v.end(), b, b + n);
	}
	fclose(f);

	return v;
}

DfuLayout_t HostLayout(void)
{
	DfuLayout_t lay;

	if (DfuLayoutGet(&lay) == false)
	{
		fprintf(stderr, "no DFU layout\n");
		exit(2);
	}

	return lay;
}

uint32_t HostPending(void)
{
	DfuLayout_t lay = HostLayout();

	return DfuTrailerPending(lay);
}

uint32_t HostDone(void)
{
	DfuLayout_t lay = HostLayout();

	return DfuTrailerDone(lay);
}

// A program unit's worth of erased memory, marked erased for the once rule.
static void HostEraseRange(uintptr_t Addr, uint32_t Len)
{
	memset(HostFlash(Addr), 0xFF, Len);
	for (uint32_t o = 0; o < Len; o += 4)
	{
		size_t i = (Addr + o - HOST_FLASH_BASE) / HostUnit();
		s_Programmed[i] = false;
	}
}

void HostPutSlot1(const std::vector<uint8_t> &Img, bool bPending)
{
	DfuLayout_t lay = HostLayout();

	HostEraseRange(HOST_SLOT1, HOST_SLOT1_SIZE);
	memcpy(HostFlash(HOST_SLOT1), Img.data(), Img.size());
	if (bPending)
	{
		uint32_t v = DFU_TRAILER_PENDING;
		memcpy(HostFlash(HOST_SLOT1 + DfuTrailerOff(lay)), &v, sizeof(v));
	}
}

// ---------------------------------------------------------------------------
// Stage 0 target port
// ---------------------------------------------------------------------------

static bool HostPowerCheck(uintptr_t Addr, const uint8_t *pSrc, uint32_t Len)
{
	g_HostMemOps++;
	if (g_HostFailAfter < 0)
	{
		return false;
	}
	if (g_HostFailAfter > 0)
	{
		g_HostFailAfter--;
		return false;
	}

	// Power goes during this one: half of it lands, in whole units.
	g_HostFailAfter = -1;
	uint32_t half = Len / 2 / HostUnit() * HostUnit();
	if (pSrc != nullptr)
	{
		if (half != 0)
		{
			HostProgram(Addr, pSrc, half);
		}
	}
	else
	{
		HostEraseRange(Addr, Len / 2);
	}
	longjmp(g_HostPowerJmp, 1);
}

uint32_t DfuTgtWriteUnit(void)
{
	return HostUnit();
}

uint32_t DfuTgtEraseUnit(uintptr_t Addr)
{
	if (Addr < HOST_FLASH_BASE || Addr >= HOST_FLASH_BASE + HOST_FLASH_SIZE)
	{
		return 0;
	}
	if (s_Kind == HOST_MEM_SECT && Addr < HOST_SLOT0 + HOST_SLOT0_SIZE)
	{
		return Addr < HOST_SLOT0 + 0x8000 ? 0x4000 : 0x8000;
	}
	return HOST_UNIT;
}

bool DfuTgtErase(uintptr_t Addr)
{
	uint32_t u = DfuTgtEraseUnit(Addr);

	if (u == 0 || (Addr % u) != 0)
	{
		return false;
	}

	HostPowerCheck(Addr, nullptr, u);
	HostEraseRange(Addr, u);

	return true;
}

bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len)
{
	uint32_t u = HostUnit();

	if ((Addr % u) != 0 || (Len % u) != 0)
	{
		return false;
	}

	HostPowerCheck(Addr, (const uint8_t *)pData, Len);
	HostProgram(Addr, (const uint8_t *)pData, Len);

	return true;
}

bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize)
{
	return DfuImgVectorValid((const uint32_t *)pImg, RunAddr, ImgSize,
							 0x20000000UL, 0x40000000UL);
}

void DfuTgtStart(uintptr_t RunAddr)
{
	g_HostStartAddr = RunAddr;
	longjmp(g_HostStartJmp, 1);
}

void DfuTgtReset(void)
{
	longjmp(g_HostResetJmp, 1);
}

// ---------------------------------------------------------------------------
// Nvm over the simulated memory
// ---------------------------------------------------------------------------

HostNvm::HostNvm(uintptr_t Addr, uint32_t Size)
{
	vAddr = Addr;
	vSize = Size;
	vWrites = 0;
	vErases = 0;
	vBusyLeft = 0;
}

uint32_t HostNvm::EraseSize(void) const
{
	// The nRF54L RRAM Nvm reports no erase, as nvm_nrfx does.
	return s_Kind == HOST_MEM_RRAM ? 0 : HOST_UNIT;
}

int HostNvm::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (vBusyLeft > 0)
	{
		vBusyLeft--;
		return -EBUSY;
	}
	if (Off > vSize || Len > vSize - Off || pBuf == nullptr)
	{
		return -EINVAL;
	}
	memcpy(pBuf, HostFlash(vAddr + Off), Len);

	return (int)Len;
}

int HostNvm::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	if (vBusyLeft > 0)
	{
		vBusyLeft--;
		return -EBUSY;
	}
	if (Off > vSize || Len > vSize - Off || pData == nullptr ||
		(Off % HostUnit()) != 0 || (Len % HostUnit()) != 0)
	{
		return -EINVAL;
	}
	HostProgram(vAddr + Off, (const uint8_t *)pData, Len);
	vWrites++;

	return (int)Len;
}

int HostNvm::Erase(uint64_t Off, uint32_t Len)
{
	uint32_t e = EraseSize();

	if (e == 0)
	{
		return 0;
	}
	if (Off > vSize || Len > vSize - Off || (Off % e) != 0 || (Len % e) != 0 ||
		Len == 0)
	{
		return -EINVAL;
	}
	HostEraseRange(vAddr + Off, Len);
	vErases++;

	return 0;
}

// ---------------------------------------------------------------------------
// What nvm.cpp links against on a target and the tests never reach.
// ---------------------------------------------------------------------------

extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}

extern "C" bool QuadSPISendCmd(SPIDev_t * const, uint8_t, uint32_t, uint8_t,
							   uint32_t, uint8_t)
{
	return false;
}

extern "C" void QuadSPISetMemSize(SPIDev_t * const, uint32_t)
{
}

// ---------------------------------------------------------------------------
// Byte stream interface
// ---------------------------------------------------------------------------

static HostByteIntrf *HostByteOf(DevIntrf_t * const pDev)
{
	return (HostByteIntrf *)pDev->pDevData;
}

static void HostByteNop(DevIntrf_t * const) {}
static uint32_t HostByteGetRate(DevIntrf_t * const) { return 0; }
static uint32_t HostByteSetRate(DevIntrf_t * const, uint32_t) { return 0; }
static bool HostByteStart(DevIntrf_t * const, uint32_t) { return true; }

static int HostByteRxData(DevIntrf_t * const pDev, uint8_t *pBuf, int Len)
{
	return HostByteOf(pDev)->RxBytes(pBuf, Len);
}

static int HostByteTxData(DevIntrf_t * const pDev, const uint8_t *pData, int Len)
{
	return HostByteOf(pDev)->TxBytes(pData, Len);
}

HostByteIntrf::HostByteIntrf()
{
	memset((void *)&vDevIntrf, 0, sizeof(vDevIntrf));
	vDevIntrf.pDevData = this;
	vDevIntrf.Type = DEVINTRF_TYPE_UART;
	vDevIntrf.Enable = HostByteNop;
	vDevIntrf.Disable = HostByteNop;
	vDevIntrf.GetRate = HostByteGetRate;
	vDevIntrf.SetRate = HostByteSetRate;
	vDevIntrf.StartRx = HostByteStart;
	vDevIntrf.RxData = HostByteRxData;
	vDevIntrf.StopRx = HostByteNop;
	vDevIntrf.StartTx = HostByteStart;
	vDevIntrf.TxData = HostByteTxData;
	vDevIntrf.StopTx = HostByteNop;
	vDevIntrf.Reset = HostByteNop;
	vDevIntrf.PowerOff = HostByteNop;
	atomic_flag_clear(&vDevIntrf.bBusy);
}
