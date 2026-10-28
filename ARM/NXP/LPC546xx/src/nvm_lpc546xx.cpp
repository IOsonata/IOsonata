/**-------------------------------------------------------------------------
@file	nvm_lpc546xx.cpp

@brief	LPC546xx internal flash as the Nvm memory controller interface

The flash is mapped at 0, sectors of 32 KB made of 256 byte pages, erased
and programmed by the IAP ROM (lpc_iap.c): prepare the sector, then erase a
page or copy a 256 byte page from RAM. The ROM programs whole pages only.

Nvm geometry (NvmMcuCfg): base 0, the flash size of the part, erase unit
and transfer page 256 bytes, write granularity 1 byte. A write updates its
page: the page is read into RAM, the new bytes merged, the page erased and
programmed with the merged content, then compared. A write of the content
the page already holds programs nothing. A page is never programmed twice
without an erase. A reset or a power loss during a write can lose the whole
page, not only the bytes written. NvmMcuErase erases one page and checks
that it reads all ones.

Each ROM call runs with interrupts masked, prepare and erase or program
together: nothing may read the flash while the ROM works on it. A page erase
or program keeps interrupts off for the time the flash needs. Reads are
memory mapped. Every operation finishes inside its call: NvmIntrf::Init
with bIntEn fails, and an operation is refused while an arbiter is
registered, the port has none to call.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <string.h>
#include <errno.h>

#include "LPC546xx.h"

#include "coredev/interrupt.h"
#include "lpc_iap.h"
#include "storage/nvm_intrf.h"

#define LPC546XX_NVM_SIZE			FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES
#define LPC546XX_NVM_SECT			FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES
#define LPC546XX_NVM_PAGE			FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES
#define LPC546XX_NVM_CMD_READ		0x03		// Frame opcodes, as the other ports
#define LPC546XX_NVM_CMD_WRITE		0x02

// Flash content at a flash address, memory mapped from 0
#ifndef LPC546XX_NVM_MEM
#define LPC546XX_NVM_MEM(Addr)		((const uint8_t *)(uintptr_t)(Addr))
#endif

static NvmDevIntrf_t s_NvmDev;
static NvmIntrfStat_t s_Stat;
static NvmIntrfArb_t s_Arbiter;

// The page the ROM copies from, in RAM, word aligned
static uint32_t s_Lpc546xxNvmPage[LPC546XX_NVM_PAGE / 4];

DevIntrf_t *NvmMcuDevIntrf(void)
{
	return &s_NvmDev.DevIntrf;
}

static bool InRange(uintptr_t Addr, uint32_t Len)
{
	return Addr <= LPC546XX_NVM_SIZE && Len <= LPC546XX_NVM_SIZE - Addr;
}

// No arbiter on this port: an operation is refused while one is registered,
// never done behind its back
static bool Available(void)
{
	if (s_Arbiter)
	{
		s_Stat.Busy++;

		return false;
	}

	return true;
}

static bool Blank(uintptr_t Page)
{
	const uint8_t *p = LPC546XX_NVM_MEM(Page);

	for (uint32_t i = 0; i < LPC546XX_NVM_PAGE; i++)
	{
		if (p[i] != 0xFF)
		{
			return false;
		}
	}

	return true;
}

static int ErasePage(uintptr_t Page)
{
	int sect = (int)(Page / LPC546XX_NVM_SECT);
	int pg = (int)(Page / LPC546XX_NVM_PAGE);
	uint32_t state = DisableInterrupt();
	IAPSTATUS res = IAPPrepSectWrite(sect, sect);

	if (res == IAPSTATUS_SUCCESS)
	{
		res = IAPErasePage(pg, pg);
	}
	EnableInterrupt(state);

	return res == IAPSTATUS_SUCCESS && Blank(Page) ? 0 : -EIO;
}

// Page erased, then programmed from s_Lpc546xxNvmPage
static int ProgramPage(uintptr_t Page)
{
	int res = ErasePage(Page);

	if (res != 0)
	{
		return res;
	}

	int sect = (int)(Page / LPC546XX_NVM_SECT);
	uint32_t state = DisableInterrupt();
	IAPSTATUS st = IAPPrepSectWrite(sect, sect);

	if (st == IAPSTATUS_SUCCESS)
	{
		st = IAPCopyRamToFlash((uint8_t *)Page, (uint8_t *)s_Lpc546xxNvmPage, LPC546XX_NVM_PAGE);
	}
	EnableInterrupt(state);

	if (st != IAPSTATUS_SUCCESS ||
		memcmp(LPC546XX_NVM_MEM(Page), s_Lpc546xxNvmPage, LPC546XX_NVM_PAGE) != 0)
	{
		return -EIO;
	}

	s_Stat.Ops++;
	s_Stat.Direct++;

	return 0;
}

// Bytes within one page: merged into the content of the page
static int Write(uintptr_t Addr, const uint8_t *pData, uint32_t Len)
{
	uintptr_t page = Addr - Addr % LPC546XX_NVM_PAGE;
	uint8_t *buf = (uint8_t *)s_Lpc546xxNvmPage;

	// The source may be in the flash, copied while the flash reads
	memcpy(buf, LPC546XX_NVM_MEM(page), LPC546XX_NVM_PAGE);
	memcpy(buf + (Addr - page), pData, Len);
	if (memcmp(buf, LPC546XX_NVM_MEM(page), LPC546XX_NVM_PAGE) == 0)
	{
		return 0;
	}

	return ProgramPage(page);
}

int NvmMcuErase(uintptr_t Addr)
{
	if ((Addr % LPC546XX_NVM_PAGE) != 0 || !InRange(Addr, LPC546XX_NVM_PAGE))
	{
		return -EINVAL;
	}
	if (!Available())
	{
		return -EBUSY;
	}

	int res = ErasePage(Addr);

	if (res == 0)
	{
		s_Stat.Ops++;
		s_Stat.Direct++;
	}

	return res;
}

void NvmMcuCfg(NvmCfg_t &Cfg)
{
	Cfg.DevNo = 0;
	Cfg.BaseAddr = 0;
	Cfg.TotalSize = LPC546XX_NVM_SIZE;
	Cfg.EraseSize = LPC546XX_NVM_PAGE;
	Cfg.SectorSize = LPC546XX_NVM_SECT;
	Cfg.PageSize = LPC546XX_NVM_PAGE;
	Cfg.WriteGran = 1;
	Cfg.AddrSize = 4;
	Cfg.RdCmd = { LPC546XX_NVM_CMD_READ, 0 };
	Cfg.WrCmd = { LPC546XX_NVM_CMD_WRITE, 0 };
	Cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

// The whole flash belongs to the application. Weak, a build with a reserved
// top answers with its boundary.
__attribute__((weak)) uint64_t NvmMcuCeiling(void)
{
	return LPC546XX_NVM_SIZE;
}

static uintptr_t FrameAddr(void)
{
	uintptr_t addr = 0;

	for (int i = 1; i < NVM_INTRF_FRAME_SIZE; i++)
	{
		addr = (addr << 8) | s_NvmDev.Hdr[i];
	}

	return addr;
}

static void FrameAdvance(uint32_t Len)
{
	uintptr_t addr = FrameAddr() + Len;

	for (int i = NVM_INTRF_FRAME_SIZE - 1; i > 0; i--)
	{
		s_NvmDev.Hdr[i] = (uint8_t)addr;
		addr >>= 8;
	}
}

static bool NvmStartTx(DevIntrf_t *, uint32_t)
{
	s_NvmDev.HdrLen = 0;

	return true;
}

static bool NvmStartRx(DevIntrf_t *, uint32_t)
{
	// The command and address stay across the repeated start of a read

	return true;
}

static int NvmTxData(DevIntrf_t *, const uint8_t *pData, int Len)
{
	int n = 0;

	if (!pData || Len <= 0)
	{
		return 0;
	}
	while (n < Len && s_NvmDev.HdrLen < NVM_INTRF_FRAME_SIZE)
	{
		s_NvmDev.Hdr[s_NvmDev.HdrLen++] = pData[n++];
	}

	if (s_NvmDev.HdrLen < NVM_INTRF_FRAME_SIZE || s_NvmDev.Hdr[0] != LPC546XX_NVM_CMD_WRITE ||
		!InRange(FrameAddr(), (uint32_t)(Len - n)))
	{
		return n;
	}

	// Page by page
	while (n < Len)
	{
		uintptr_t addr = FrameAddr();
		uint32_t room = LPC546XX_NVM_PAGE - addr % LPC546XX_NVM_PAGE;
		uint32_t count = (uint32_t)(Len - n) < room ? (uint32_t)(Len - n) : room;

		if (!Available() || Write(addr, pData + n, count) != 0)
		{
			return n;
		}
		FrameAdvance(count);
		n += (int)count;
	}

	return n;
}

static int NvmRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (!pBuff || Len <= 0 || s_NvmDev.HdrLen != NVM_INTRF_FRAME_SIZE ||
		s_NvmDev.Hdr[0] != LPC546XX_NVM_CMD_READ || !InRange(FrameAddr(), (uint32_t)Len) ||
		!Available())
	{
		return 0;
	}
	memcpy(pBuff, LPC546XX_NVM_MEM(FrameAddr()), Len);
	FrameAdvance(Len);

	return Len;
}

static void NvmNoOp(DevIntrf_t *)
{
}

static uint32_t NvmGetRate(DevIntrf_t *)
{
	return 0;
}

static uint32_t NvmSetRate(DevIntrf_t *, uint32_t)
{
	return 0;
}

static void *NvmGetHandle(DevIntrf_t *pDev)
{
	return pDev->pDevData;
}

bool NvmIntrf::Init(DevIntrfEvtHandler_t EvtCB, bool bIntEn)
{
	DevIntrf_t &dev = s_NvmDev.DevIntrf;

	// Every operation finishes inside its call, no completion event
	if (bIntEn)
	{
		return false;
	}

	// Not while another object has a transfer
	if (atomic_flag_test_and_set(&dev.bBusy))
	{
		return false;
	}

	s_NvmDev.HdrLen = 0;
	dev.pDevData = &s_NvmDev;
	dev.Type = DEVINTRF_TYPE_MEMCTRL;
	dev.Enable = NvmNoOp;
	dev.Disable = NvmNoOp;
	dev.GetRate = NvmGetRate;
	dev.SetRate = NvmSetRate;
	dev.StartRx = NvmStartRx;
	dev.RxData = NvmRxData;
	dev.StopRx = NvmNoOp;
	dev.StartTx = NvmStartTx;
	dev.TxData = NvmTxData;
	dev.TxSrData = NvmTxData;
	dev.StopTx = NvmNoOp;
	dev.PowerOff = NvmNoOp;
	dev.GetHandle = NvmGetHandle;
	dev.MaxRetry = 0;
	dev.EnCnt = 1;
	dev.EvtCB = EvtCB;
	dev.bIntEn = false;
	atomic_flag_clear(&dev.bBusy);

	return true;
}

void NvmIntrfSetArbiter(NvmIntrfArb_t pArb)
{
	s_Arbiter = pArb;
}

void NvmIntrfGetStat(NvmIntrfStat_t *pStat)
{
	if (pStat)
	{
		*pStat = s_Stat;
	}
}
