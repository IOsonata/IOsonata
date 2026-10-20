/**-------------------------------------------------------------------------
@file	nvm_stm32l1xx.cpp

@brief	STM32L152xE true data EEPROM. Polling, byte overwrite.

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
#include "stm32l1xx.h"
#include "storage/nvm_intrf.h"

#if !defined(STM32L152xE)
#error "This EEPROM geometry is for STM32L152xE"
#endif
#define NVM_BASE FLASH_EEPROM_BASE
#define NVM_SIZE 0x4000UL
#define NVM_ERASE_SIZE 0U
#define NVM_ERRORS (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR)


static NvmDevIntrf_t s_NvmDev;
static NvmIntrfStat_t s_Stat;
static NvmIntrfArb_t s_Arbiter;

DevIntrf_t *NvmMcuDevIntrf(void) { return &s_NvmDev.DevIntrf; }

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

static bool s_RelockPending;

bool NvmMcuIsReady(void)
{
	if (FLASH->SR & FLASH_SR_BSY) return false;
	if (s_RelockPending)
	{
		FLASH->PECR |= FLASH_PECR_PELOCK;
		s_RelockPending = false;
	}
	return true;
}

static bool ControllerInit(void) { return NvmMcuIsReady(); }

static int WaitReady(void)
{
	// Bounded even when SysTick is stopped or interrupts are masked.
	for (uint32_t n = 0; n < 1000000UL; n++)
		if (NvmMcuIsReady()) return (FLASH->SR & NVM_ERRORS) ? -EIO : 0;
	return -ETIMEDOUT;
}

static int Program(uintptr_t Addr, const uint8_t *pData, int Len)
{
	if (!InRange(Addr, Len)) return -EINVAL;
	if (!Available()) return -EBUSY;
	// Only unlock the data EEPROM; program flash and option bytes stay locked.
	FLASH->SR = NVM_ERRORS | FLASH_SR_EOP;
	uint32_t mask = __get_PRIMASK();
	__disable_irq();
	if (FLASH->PECR & FLASH_PECR_PELOCK)
	{
		FLASH->PEKEYR = 0x89ABCDEFUL;
		FLASH->PEKEYR = 0x02030405UL;
	}
	__set_PRIMASK(mask);
	if (FLASH->PECR & FLASH_PECR_PELOCK) return -EACCES;
	FLASH->PECR &= ~(FLASH_PECR_PROG | FLASH_PECR_DATA | FLASH_PECR_ERASE |
		FLASH_PECR_FPRG | FLASH_PECR_FTDW | FLASH_PECR_EOPIE | FLASH_PECR_ERRIE);
	int res = 0;
	for (int i = 0; i < Len; i++)
	{
		volatile uint8_t *dst = (volatile uint8_t *)(Addr + i);
		if (*dst == pData[i]) continue;
		*dst = pData[i];
		res = WaitReady();
		if (res != 0) break;
		if (*dst != pData[i]) { res = -EIO; break; }
	}
	// PECR must not be changed while BSY is set. A timeout leaves the port
	// unavailable until the controller finishes; the next readiness check relocks.
	if (NvmMcuIsReady())
	{
		FLASH->PECR |= FLASH_PECR_PELOCK;
		FLASH->SR = NVM_ERRORS | FLASH_SR_EOP;
	}
	else s_RelockPending = true;
	return res;
}

int NvmMcuErase(uintptr_t)
{
	// True EEPROM overwrites bytes. Nvm::Erase is a no-op with EraseSize=0.
	return -ENOTSUP;
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
		addr = (addr << 8) | s_NvmDev.Hdr[i];
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
	if (!pData || Len <= 0) return 0;
	int n = 0;
	while (n < Len && s_NvmDev.HdrLen < NVM_INTRF_FRAME_SIZE)
		s_NvmDev.Hdr[s_NvmDev.HdrLen++] = pData[n++];
	if (s_NvmDev.HdrLen < NVM_INTRF_FRAME_SIZE) return n;
	if (s_NvmDev.Hdr[0] != 0x02) return n;
	if (!InRange(FrameAddr(), (uint32_t)(Len - n))) return n;

	while (n < Len)
	{
		int count = Len - n;
		if (count > (int)sizeof(s_NvmDev.Data)) count = sizeof(s_NvmDev.Data);
		// Stage before entering P/E mode, including a source in data flash.
		if (!Available()) return n;
		memcpy(s_NvmDev.Data, pData + n, count);
		if (Program(FrameAddr(), s_NvmDev.Data, count) != 0) return n;
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
		!Available()) return 0;
	memcpy(pBuff, (const void *)FrameAddr(), Len);
	FrameAdvance(Len);
	return Len;
}

static void NvmNoOp(DevIntrf_t *) {}
static uint32_t NvmGetRate(DevIntrf_t *) { return 0; }
static uint32_t NvmSetRate(DevIntrf_t *, uint32_t) { return 0; }
static void *NvmGetHandle(DevIntrf_t *pDev) { return pDev->pDevData; }

bool NvmIntrf::Init(DevIntrfEvtHandler_t EvtCB, bool bIntEn)
{
	// This port completes inside the call; it cannot promise an IRQ event.
	if (bIntEn) return false;
	// Do not reinitialize a controller while another object has a transfer.
	if (atomic_flag_test_and_set(&s_NvmDev.DevIntrf.bBusy)) return false;
	if (!ControllerInit())
	{
		atomic_flag_clear(&s_NvmDev.DevIntrf.bBusy);
		return false;
	}
	s_NvmDev.HdrLen = 0;
	DevIntrf_t &dev = s_NvmDev.DevIntrf;
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

void NvmIntrfSetArbiter(NvmIntrfArb_t pArb) { s_Arbiter = pArb; }
void NvmIntrfGetStat(NvmIntrfStat_t *pStat)
{
	if (pStat) *pStat = s_Stat;
}
