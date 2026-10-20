/**-------------------------------------------------------------------------
@file	nvm_ra4m1.cpp

@brief	RA4M1 data flash. Polling, byte program and explicit 1 KB erase.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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
#include "ra4m1xxx.h"
#include "idelay.h"
#include "storage/nvm_intrf.h"

#define NVM_BASE 0x40100000UL
#define NVM_SIZE 0x2000UL
#define NVM_ERASE_SIZE 0x400UL
#define NVM_PE_BASE 0xFE000000UL
#define NVM_ERRORS (R_FACI_LP_FSTATR2_ILGLERR_Msk | \
	R_FACI_LP_FSTATR2_PRGERR1_Msk | R_FACI_LP_FSTATR2_ERERR_Msk)


static NvmDevIntrf_t s_NvmDev;
static NvmIntrfStat_t s_Stat;
static NvmIntrfArb_t s_Arbiter;

DevIntrf_t *NvmMcuDevIntrf(void)
{
	return &s_NvmDev.DevIntrf;
}

static bool InRange(uintptr_t Addr, uint32_t Len)
{
	return Addr >= NVM_BASE && Addr - NVM_BASE <= NVM_SIZE &&
		Len <= NVM_SIZE - (Addr - NVM_BASE);
}

static bool Available(void)
{
	// No timeslot implementation on this polling port. Refuse an operation
	// if an owner registers an arbiter; never bypass that owner.
	if (s_Arbiter || !NvmMcuIsReady())
	{
		s_Stat.Busy++;

		return false;
	}

	return true;
}

bool NvmMcuIsReady(void)
{
	return R_FACI_LP->FENTRYR == 0 && R_FACI_LP->FCR == 0;
}

static bool ControllerInit(void)
{
	if (!NvmMcuIsReady())
	{
		return false;
	}

	if (R_FACI_LP->DFLCTL != 1)
	{
		R_FACI_LP->DFLCTL = 1;
		usDelay(6);
	}

	return true;
}

static bool SetMode(uint8_t Mode)
{
	uint32_t mask = __get_PRIMASK();
	bool ok;

	__disable_irq();
	R_FACI_LP->FPR = 0xA5;
	R_FACI_LP->FPMCR = Mode;
	R_FACI_LP->FPMCR = (uint8_t)~Mode;
	R_FACI_LP->FPMCR = Mode;
	ok = R_FACI_LP->FPMCR == Mode;
	__set_PRIMASK(mask);

	return ok;
}

static int LeaveMode(int Res)
{
	if (Res != 0)
	{
		R_FACI_LP->FRESETR = 1;
		R_FACI_LP->FRESETR = 0;
	}

	if (!SetMode(0x08))
	{
		return -EIO;
	}
	usDelay(6);
	R_FACI_LP->FENTRYR = 0xAA00;
	for (uint32_t n = 0; n < 1000000UL; n++)
	{
		if (R_FACI_LP->FENTRYR == 0)
		{
			return Res;
		}
	}

	return -ETIMEDOUT;
}

static int EnterMode(void)
{
	uint32_t clock;

	if (!Available())
	{
		return -EBUSY;
	}
	clock = SystemFlashClockGet();
	// MF3 PCKA uses five bits. The application must keep FCLK stable for
	// the entire operation; no writes at an unsupported flash clock.
	if (clock < 4000000UL || clock > 32000000UL)
	{
		return -EINVAL;
	}

	if (!ControllerInit())
	{
		return -EBUSY;
	}
	R_FACI_LP->FENTRYR = 0xAA80;
	usDelay(6);
	if (R_FACI_LP->FENTRYR != 0x0080)
	{
		return LeaveMode(-EIO);
	}

	if (!SetMode(R_SYSTEM->OPCCR_b.OPCM == 0 ? 0x10 : 0x50))
	{
		return LeaveMode(-EIO);
	}
	R_FACI_LP->FISR_b.PCKA = (clock + 999999UL) / 1000000UL - 1;
	R_FACI_LP->FASR = 0;

	return 0;
}

static int FinishCommand(uint32_t TimeoutUs)
{
	// Data-flash erase can take 504 ms at the slow supported FCLK. Use
	// operation-specific budgets, independent of compiler optimization.
	uint32_t n = TimeoutUs;
	while (!(R_FACI_LP->FSTATR1 & R_FACI_LP_FSTATR1_FRDY_Msk) && --n)
	{
		usDelay(1);
	}

	if (!n)
	{
		return -ETIMEDOUT;
	}
	R_FACI_LP->FCR = 0;
	n = 5000;
	while ((R_FACI_LP->FSTATR1 & R_FACI_LP_FSTATR1_FRDY_Msk) && --n)
	{
		usDelay(1);
	}

	if (!n)
	{
		return -ETIMEDOUT;
	}

	return (R_FACI_LP->FSTATR2 & NVM_ERRORS) ? -EIO : 0;
}

static void Address(uintptr_t Addr, uint32_t Len)
{
	uint32_t start = NVM_PE_BASE + (Addr - NVM_BASE);
	uint32_t end = start + Len - 1;
	R_FACI_LP->FSARH = (uint16_t)(start >> 16);
	R_FACI_LP->FSARL = (uint16_t)start;
	R_FACI_LP->FEARH = (uint16_t)(end >> 16);
	R_FACI_LP->FEARL = (uint16_t)end;
}

static int BlankCheck(uintptr_t Addr, uint32_t Len)
{
	int res;

	Address(Addr, Len);
	R_FACI_LP->FCR = 0x83;
	res = FinishCommand(1000 + 200 * Len);
	if (res == 0 && R_FACI_LP->FSTATR00_b.BCERR0)
	{
		res = -EACCES;
	}

	return res;
}

static int Program(uintptr_t Addr, const uint8_t *pData, int Len)
{
	int res = 0;

	if (!InRange(Addr, Len) || Len <= 0)
	{
		return -EINVAL;
	}
	res = EnterMode();
	if (res != 0)
	{
		return res;
	}

	// Use the sequencer blank check, not an erased-byte assumption. Never
	// erase implicitly or program the same data-flash cell twice.
	res = BlankCheck(Addr, Len);
	for (int i = 0; i < Len && res == 0; i++)
	{
		Address(Addr + i, 1);
		R_FACI_LP->FWBL0 = pData[i];
		R_FACI_LP->FCR = 0x81;
		res = FinishCommand(5000);
	}
	res = LeaveMode(res);
	if (res == 0 && memcmp((const void *)Addr, pData, Len) != 0)
	{
		res = -EIO;
	}

	return res;
}

int NvmMcuErase(uintptr_t Addr)
{
	int res;

	if ((Addr % NVM_ERASE_SIZE) || !InRange(Addr, NVM_ERASE_SIZE))
	{
		return -EINVAL;
	}
	res = EnterMode();
	if (res != 0)
	{
		return res;
	}
	Address(Addr, NVM_ERASE_SIZE);
	R_FACI_LP->FCR = 0x84;
	res = FinishCommand(1000000);
	if (res == 0)
	{
		res = BlankCheck(Addr, NVM_ERASE_SIZE);
	}
	res = LeaveMode(res);
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
	Cfg.BaseAddr = NVM_BASE;
	Cfg.TotalSize = NVM_SIZE;
	Cfg.EraseSize = NVM_ERASE_SIZE;
	Cfg.SectorSize = NVM_ERASE_SIZE;
	Cfg.PageSize = NVM_INTRF_XFER_SIZE;
	Cfg.WriteGran = 1;
	Cfg.AddrSize = 4;
	Cfg.RdCmd = { 0x03, 0 };
	Cfg.WrCmd = { 0x02, 0 };
	Cfg.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

__attribute__((weak)) uint64_t NvmMcuCeiling(void)
{
	return (uint64_t)NVM_BASE + NVM_SIZE;
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
	// Keep the command/address across the repeated start in DeviceIntrfRead.

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

	if (s_NvmDev.HdrLen < NVM_INTRF_FRAME_SIZE)
	{
		return n;
	}

	if (s_NvmDev.Hdr[0] != 0x02)
	{
		return n;
	}

	if (!InRange(FrameAddr(), (uint32_t)(Len - n)))
	{
		return n;
	}

	while (n < Len)
	{
		int count = Len - n;
		if (count > (int)sizeof(s_NvmDev.Data))
		{
			count = sizeof(s_NvmDev.Data);
		}
		// Stage before entering P/E mode, including a source in data flash.
		if (!Available())
		{
			return n;
		}
		memcpy(s_NvmDev.Data, pData + n, count);
		if (Program(FrameAddr(), s_NvmDev.Data, count) != 0)
		{
			return n;
		}
		s_Stat.Ops++;
		s_Stat.Direct++;
		FrameAdvance(count);
		n += count;
	}

	return n;
}

static int NvmRxData(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	if (!pBuff || Len <= 0 || s_NvmDev.HdrLen != NVM_INTRF_FRAME_SIZE ||
		s_NvmDev.Hdr[0] != 0x03 || !InRange(FrameAddr(), (uint32_t)Len) ||
		!Available())
	{
		return 0;
	}
	memcpy(pBuff, (const void *)FrameAddr(), Len);
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

	// This port completes inside the call; it cannot promise an IRQ event.
	if (bIntEn)
	{
		return false;
	}

	// Do not reinitialize a controller while another object has a transfer.
	if (atomic_flag_test_and_set(&s_NvmDev.DevIntrf.bBusy))
	{
		return false;
	}

	if (!ControllerInit())
	{
		atomic_flag_clear(&s_NvmDev.DevIntrf.bBusy);

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
