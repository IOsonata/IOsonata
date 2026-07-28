/**-------------------------------------------------------------------------
@file	nvm.cpp

@brief	Generic addressed non-volatile memory driver.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

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

#include "idelay.h"
#include "iopinctrl.h"
#include "coredev/spi.h"
#include "storage/nvm.h"

/******** For DEBUG Trace ************/
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#define NVM_SR_WIP			0x01U
#define NVM_SR_WEL			0x02U

#define NVM_CMD_READID		0x9FU
#define NVM_CMD_READ		0x03U
#define NVM_CMD_WRITE		0x02U
#define NVM_CMD_WRENABLE	0x06U
#define NVM_CMD_WRDISABLE	0x04U
#define NVM_CMD_RDSR		0x05U
#define NVM_CMD_WRSR		0x01U
#define NVM_CMD_SECT_ERASE	0x20U
#define NVM_CMD_BLK32_ERASE	0x52U
#define NVM_CMD_BLK64_ERASE	0xD8U
#define NVM_CMD_CHIP_ERASE	0xC7U
#define NVM_CMD_RESET_EN	0x66U
#define NVM_CMD_RESET		0x99U
#define NVM_CMD_EN4B		0xB7U

#define NVM_XFER_TMOUT		100000U
#define NVM_OP_TMOUT		100000U

int Nvm::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	return Interface()->Read(DeviceAddress(), pCmdAddr, CmdAddrLen,
							 pBuff, BuffLen);
}

Nvm::Nvm()
{
	Valid(false);
	InterruptEnabled(false);
	vRegionOffset = 0;
	vRegionSize = 0;
	vpWaitCB = nullptr;
	vEvtHandler = nullptr;
	vOpEvt = NVM_EVT_NONE;
	vOpRes = 0;
	vOpOff = 0;
	vOpLen = 0;
	vOpAddr = 0;
	vpOpData = nullptr;
	vOpRemain = 0;
	atomic_flag_clear(&vOpLock);
	vOpState = NVM_OPSTATE_IDLE;
	vXferState = NVM_XFER_NONE;
	vbXferWaiting = false;
	vbXferDone = false;
	vbXferFail = false;
	vbInitialized = false;
	vbEnabled = false;
	vBaseAddr = 0;
	vDevSize = 0;
	vEraseSize = 0;
	vSectSize = 0;
	vPageSize = 0;
	vWrGran = 1;
	vAddrSize = 3;
	vAddrSpan = 0;
	vWrDelayUs = 0;
	vWrProtMask = 0;
	memset(&vRdCmd, 0, sizeof(vRdCmd));
	memset(&vWrCmd, 0, sizeof(vWrCmd));
	vbBare = true;
	vbPhased = false;
	vBaseDevAddr = 0;
	vWrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE,
				   IOPINTYPE_NORMAL };
}

Nvm::~Nvm()
{
	while (atomic_flag_test_and_set(&vOpLock)) {}
	vOpEvt = NVM_EVT_NONE;
	vOpRemain = 0;
	vpOpData = nullptr;
	vOpState = NVM_OPSTATE_IDLE;
	vXferState = NVM_XFER_NONE;
	vbXferWaiting = false;
	atomic_flag_clear(&vOpLock);
	Unhook();
}

int Nvm::FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr,
				   uint32_t *pDevAddr)
{
	int len = 0;
	uint32_t devaddr = vBaseDevAddr;

	if (Cmd != 0)
	{
		pFrame[len++] = Cmd;
	}

	for (int i = 0; i < vAddrSize; i++)
	{
		pFrame[len++] = (uint8_t)(Addr >> (8 * (vAddrSize - 1 - i)));
	}

	if (vAddrSpan != 0 && Addr >= vAddrSpan)
	{
		devaddr |= (Addr / vAddrSpan) & 7U;
	}

	if (pDevAddr != nullptr)
	{
		*pDevAddr = devaddr;
	}

	return len;
}

int Nvm::SendCmd(const NvmCmd_t &Cmd)
{
	if (Cmd.Cmd == 0)
	{
		return 0;
	}

	if (vbPhased)
	{
		SPIDev_t *dev = SPIGetHandle(*Interface());

		SPIStartTx(dev, (int)vBaseDevAddr);
		bool ok = QuadSPISendCmd(dev, Cmd.Cmd, (uint32_t)-1, 0, 0,
								 Cmd.DummyCycle);
		SPIStopTx(dev);

		return ok ? 0 : -EIO;
	}

	uint8_t c = Cmd.Cmd;

	XferBegin();
	int n = Interface()->Tx(DeviceAddress(), &c, 1);

	return XferShort(n, 1) ? 0 : -EIO;
}

int Nvm::ReadStatus(uint8_t &Status)
{
	Status = 0;

	if (vbBare)
	{
		return 0;
	}

	if (vbPhased)
	{
		SPIDev_t *dev = SPIGetHandle(*Interface());

		SPIStartRx(dev, (int)vBaseDevAddr);
		QuadSPISendCmd(dev, NVM_CMD_RDSR, (uint32_t)-1, 0, 1, 0);
		int rd = SPIRxData(dev, &Status, 1);
		SPIStopRx(dev);

		return rd == 1 ? 0 : -EIO;
	}

	uint8_t cmd = NVM_CMD_RDSR;

	XferBegin();
	int n = Read(&cmd, 1, &Status, 1);

	return XferShort(n, 1) ? 0 : -EIO;
}

uint8_t Nvm::ReadStatus(void)
{
	if (vbEnabled == false)
	{
		return 0;
	}

	uint8_t sr = 0;
	(void)ReadStatus(sr);

	return sr;
}

uint32_t Nvm::ReadId(int Len)
{
	if (vbEnabled == false || Len < 1 || Len > 4)
	{
		return 0;
	}

	uint32_t id = 0;
	uint8_t cmd = NVM_CMD_READID;

	if (vbPhased)
	{
		SPIDev_t *dev = SPIGetHandle(*Interface());

		SPIStartRx(dev, (int)vBaseDevAddr);
		QuadSPISendCmd(dev, cmd, (uint32_t)-1, 0, Len, 0);
		int rd = SPIRxData(dev, (uint8_t*)&id, Len);
		SPIStopRx(dev);

		return rd == Len ? id : 0;
	}

	XferBegin();
	int n = Read(&cmd, 1, (uint8_t*)&id, Len);

	return XferShort(n, Len) ? id : 0;
}

bool Nvm::WaitReady(uint32_t Timeout)
{
	if (vbBare)
	{
		if (WaitPoll() == false)
		{
			return false;
		}
		if (vWrDelayUs > 0)
		{
			usDelay(vWrDelayUs);
		}
		return true;
	}

	while (Timeout-- > 0)
	{
		uint8_t sr;

		if (ReadStatus(sr) != 0)
		{
			return false;
		}
		if ((sr & NVM_SR_WIP) == 0)
		{
			return true;
		}
		if (WaitPoll() == false)
		{
			return false;
		}
	}

	return false;
}

bool Nvm::WriteEnable(uint32_t Timeout)
{
	if (vbBare)
	{
		return true;
	}

	if (WaitReady(Timeout) == false ||
		SendCmd({ NVM_CMD_WRENABLE, 0 }) != 0)
	{
		return false;
	}

	while (Timeout-- > 0)
	{
		uint8_t sr;

		if (ReadStatus(sr) != 0)
		{
			return false;
		}
		if ((sr & NVM_SR_WEL) != 0)
		{
			return true;
		}
		if (WaitPoll() == false)
		{
			return false;
		}
	}

	return false;
}

void Nvm::WriteDisable(void)
{
	if (Interface() != nullptr && vbBare == false)
	{
		(void)SendCmd({ NVM_CMD_WRDISABLE, 0 });
	}
}

#define NVM_HOOK_MAX		4

typedef struct {
	DevIntrf_t	*pIntrf;
	Nvm			*pNvm;
} NvmHook_t;

static NvmHook_t s_NvmHook[NVM_HOOK_MAX];

static int NvmIntrfEvtCB(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *, int)
{
	for (int i = 0; i < NVM_HOOK_MAX; i++)
	{
		if (s_NvmHook[i].pIntrf == pDev && s_NvmHook[i].pNvm != nullptr)
		{
			s_NvmHook[i].pNvm->IntrfEvent(EvtId);
		}
	}

	return 0;
}

void Nvm::Unhook(void)
{
	for (int i = 0; i < NVM_HOOK_MAX; i++)
	{
		if (s_NvmHook[i].pNvm != this)
		{
			continue;
		}

		DevIntrf_t *ip = s_NvmHook[i].pIntrf;

		s_NvmHook[i].pNvm = nullptr;
		s_NvmHook[i].pIntrf = nullptr;

		bool used = false;
		for (int j = 0; j < NVM_HOOK_MAX; j++)
		{
			if (s_NvmHook[j].pIntrf == ip && s_NvmHook[j].pNvm != nullptr)
			{
				used = true;
				break;
			}
		}

		if (used == false && ip != nullptr && ip->EvtCB == NvmIntrfEvtCB)
		{
			ip->EvtCB = nullptr;
		}
	}
}

bool Nvm::Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			   uint64_t RegionOff, uint64_t RegionSize)
{
	vbEnabled = false;
	Valid(false);
	CancelOp(-ECANCELED);
	Unhook();
	vbInitialized = false;
	Region(0, 0);

	if (pIntrf == nullptr)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(Cfg.DevNo);
	InterruptEnabled(Cfg.bIntEn);

	// Tell the interface what the application asked for. An interface that can
	// report completion by interrupt uses this to submit and return instead of
	// submitting and waiting; one that cannot simply finishes in the call and
	// reports the full length, which the driver handles either way. Only set,
	// never cleared: an interface that runs interrupt driven for its own
	// reasons keeps doing so.
	if (Cfg.bIntEn)
	{
		DevIntrf_t *dip = *pIntrf;

		dip->bIntEn = true;
	}
	vpWaitCB = Cfg.pWaitCB;
	vEvtHandler = Cfg.EvtHandler;

	auto fail = [this]() -> bool {
		Unhook();
		vbEnabled = false;
		vbInitialized = false;
		Valid(false);
		Region(0, 0);
		return false;
	};

	vBaseAddr = Cfg.BaseAddr;
	vDevSize = Cfg.TotalSize;
	vEraseSize = Cfg.EraseSize;
	vSectSize = Cfg.SectorSize;
	vPageSize = Cfg.PageSize;
	vWrGran = Cfg.WriteGran != 0 ? Cfg.WriteGran : 1;
	vAddrSize = Cfg.AddrSize != 0 ? Cfg.AddrSize : 3;
	vBaseDevAddr = (uint32_t)Cfg.DevNo;
	vWrDelayUs = Cfg.WriteDelayUs;
	vWrProtMask = Cfg.WrProtMask;
	vWrProtPin = Cfg.WrProtPin;

	switch (pIntrf->Type())
	{
		case DEVINTRF_TYPE_I2C:
		case DEVINTRF_TYPE_SPI:
		case DEVINTRF_TYPE_UNKOWN:
		case DEVINTRF_TYPE_QSPI:
		case DEVINTRF_TYPE_OSPI:
		case DEVINTRF_TYPE_MEMCTRL:
			break;

		default:
			return fail();
	}

	// Media that answer to an address and nothing else: no command protocol,
	// so no write enable, no status register, no reset pair and no device id.
	// An I2C part is one. MCU internal memory reached through its controller
	// is another: write enable and ready are registers the interface touches,
	// not commands sent to a chip.
	vbBare = pIntrf->Type() == DEVINTRF_TYPE_I2C ||
			 pIntrf->Type() == DEVINTRF_TYPE_MEMCTRL;
	vbPhased = pIntrf->Type() == DEVINTRF_TYPE_QSPI ||
				pIntrf->Type() == DEVINTRF_TYPE_OSPI;

	if (vbPhased)
	{
		QuadSPISetMemSize(SPIGetHandle(*pIntrf), (uint32_t)vDevSize);
	}

	if (vbBare)
	{
		vRdCmd = Cfg.RdCmd;
		vWrCmd = Cfg.WrCmd;
	}
	else
	{
		vRdCmd = Cfg.RdCmd.Cmd != 0 ? Cfg.RdCmd
									: NvmCmd_t{ NVM_CMD_READ, 0 };
		vWrCmd = Cfg.WrCmd.Cmd != 0 ? Cfg.WrCmd
									: NvmCmd_t{ NVM_CMD_WRITE, 0 };
	}

	if (vDevSize == 0 || vPageSize == 0 || vAddrSize < 1 || vAddrSize > 4 ||
		Cfg.DevIdSize > 4)
	{
		DEBUG_PRINTF("Nvm geometry invalid\r\n");
		return fail();
	}

	if (vEraseSize != 0 && vEraseSize != 4U * 1024U &&
		vEraseSize != 32U * 1024U && vEraseSize != 64U * 1024U)
	{
		DEBUG_PRINTF("Nvm erase size unsupported\r\n");
		return fail();
	}

	vAddrSpan = vAddrSize >= 4 ? 0 : (1UL << (8 * vAddrSize));
	if (vAddrSpan != 0 && vDevSize <= vAddrSpan)
	{
		vAddrSpan = 0;
	}

	vOpEvt = NVM_EVT_NONE;
	vOpRes = 0;
	vOpRemain = 0;
	vpOpData = nullptr;
	vOpState = NVM_OPSTATE_IDLE;
	vXferState = NVM_XFER_NONE;
	vbXferWaiting = false;
	vbXferDone = false;
	vbXferFail = false;

	if (vWrProtPin.PortNo >= 0 && vWrProtPin.PinNo >= 0)
	{
		IOPinCfg(&vWrProtPin, 1);
		IOPinClear(vWrProtPin.PortNo, vWrProtPin.PinNo);
	}

	vbEnabled = true;

	if (Cfg.pInitCB != nullptr && Cfg.pInitCB(this, pIntrf) == false)
	{
		return fail();
	}

	if (vbBare == false && vEraseSize != 0)
	{
		if (SendCmd({ NVM_CMD_RESET_EN, 0 }) != 0 ||
			SendCmd({ NVM_CMD_RESET, 0 }) != 0)
		{
			return fail();
		}
	}

	if (vbBare == false && Cfg.DevId != 0 && Cfg.DevIdSize > 0)
	{
		bool found = false;
		for (int rtry = 0; rtry < 6; rtry++)
		{
			if (ReadId(Cfg.DevIdSize) == Cfg.DevId)
			{
				found = true;
				break;
			}
		}
		if (found == false)
		{
			return fail();
		}
	}

	if (vbBare == false && vEraseSize != 0 && vAddrSize > 3 &&
		SendCmd({ NVM_CMD_EN4B, 0 }) != 0)
	{
		return fail();
	}

	if (RegionOff > vDevSize)
	{
		return fail();
	}

	uint64_t avail = vDevSize - RegionOff;
	if (RegionSize > avail)
	{
		return fail();
	}

	uint64_t rsize = RegionSize != 0 ? RegionSize : avail;
	if ((vBaseAddr % vWrGran) != 0 || (RegionOff % vWrGran) != 0 ||
		(vPageSize % vWrGran) != 0)
	{
		return fail();
	}
	if (vEraseSize != 0 && (RegionOff % vEraseSize) != 0)
	{
		return fail();
	}

	Region(RegionOff, rsize);

	DevIntrf_t *ip = *pIntrf;
	bool needHook = InterruptEnabled() || ip->bIntEn;

	if (needHook && (ip->EvtCB == nullptr || ip->EvtCB == NvmIntrfEvtCB))
	{
		int slot = -1;
		for (int i = 0; i < NVM_HOOK_MAX; i++)
		{
			if (s_NvmHook[i].pNvm == nullptr)
			{
				slot = i;
				break;
			}
		}
		if (slot < 0)
		{
			return fail();
		}

		s_NvmHook[slot].pIntrf = ip;
		s_NvmHook[slot].pNvm = this;
		if (ip->EvtCB == nullptr)
		{
			ip->EvtCB = NvmIntrfEvtCB;
		}
	}

	vbInitialized = true;
	vbEnabled = true;
	Valid(true);

	return true;
}

int Nvm::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (vbEnabled == false)
	{
		return -ENODEV;
	}
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	int sr = ServiceRun(NVM_OP_TMOUT);
	if (sr < 0)
	{
		return sr;
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	uint32_t addr = (uint32_t)(vBaseAddr + RegionOffset() + Off);
	uint8_t *pb = (uint8_t*)pBuf;
	uint32_t cnt = Len;

	while (cnt > 0)
	{
		uint8_t frame[8];
		uint32_t devaddr;
		int flen = FrameAddr(frame, vRdCmd.Cmd, addr, &devaddr);
		uint32_t l = cnt;

		if (vAddrSpan != 0)
		{
			uint32_t room = vAddrSpan - (addr % vAddrSpan);
			if (l > room) { l = room; }
		}

		int rd;
		if (vbPhased)
		{
			SPIDev_t *dev = SPIGetHandle(*Interface());
			SPIStartRx(dev, (int)vBaseDevAddr);
			QuadSPISendCmd(dev, vRdCmd.Cmd, addr, vAddrSize, l,
						   vRdCmd.DummyCycle);
			rd = SPIRxData(dev, pb, (int)l);
			SPIStopRx(dev);
		}
		else
		{
			DeviceAddress(devaddr);
			XferBegin();
			rd = Read(frame, flen, pb, (int)l);
			if (XferShort(rd, (int)l))
			{
				rd = (int)l;
			}
		}

		if (rd != (int)l)
		{
			return -EIO;
		}

		addr += (uint32_t)rd;
		pb += rd;
		cnt -= (uint32_t)rd;
	}

	return (int)Len;
}

void Nvm::IntrfEvent(DEVINTRF_EVT EvtId)
{
	bool done;

	// Only COMPLETED ends a transfer. TX_READY asks the handler for more
	// data and TX_FIFO_EMPTY says a software FIFO drained, and neither
	// means the transfer finished: the nRF I2C master raises TX_READY on
	// LASTTX, before the stop condition, and taking that as the end put a
	// chunk back in the driver's hands while the bus was still on it. An
	// interface that has its own idea of finished says so with COMPLETED.
	switch (EvtId)
	{
		case DEVINTRF_EVT_COMPLETED:
			done = true;
			break;

		case DEVINTRF_EVT_TX_TIMEOUT:
		case DEVINTRF_EVT_RX_TIMEOUT:
			done = false;
			break;

		default:
			return;
	}

	// Only a chunk actually handed over takes it. A read or a command frame
	// never puts the transfer in RUNNING, and ServiceStep returns it to NONE
	// the moment it consumes one, so an event arriving at any other time
	// belongs to nothing here and is discarded.
	if (vXferState == NVM_XFER_RUNNING)
	{
		vXferState = done ? NVM_XFER_COMPLETE : NVM_XFER_FAILED;

		return;
	}

	// Otherwise it is a transfer this call is waiting on itself.
	if (vbXferWaiting)
	{
		if (done) { vbXferDone = true; }
		else { vbXferFail = true; }
	}
}

void Nvm::XferBegin(void)
{
	vbXferDone = false;
	vbXferFail = false;
	vbXferWaiting = true;
}

bool Nvm::XferWait(uint32_t Timeout)
{
	while (Timeout-- > 0)
	{
		if (vbXferFail)
		{
			vbXferWaiting = false;
			return false;
		}
		if (vbXferDone)
		{
			vbXferWaiting = false;
			return true;
		}
		if (WaitPoll() == false)
		{
			break;
		}
	}

	vbXferWaiting = false;
	return false;
}

bool Nvm::XferShort(int Count, int Expect)
{
	if (Count == Expect)
	{
		vbXferWaiting = false;
		return true;
	}

	DevIntrf_t *ip = *Interface();
	if (Count == -1 && ip->bIntEn)
	{
		return XferWait(NVM_XFER_TMOUT);
	}

	vbXferWaiting = false;
	return false;
}

bool Nvm::FinishOpLocked(int Res, NVM_EVT &Evt, uint64_t &Off, uint32_t &Len)
{
	if (vOpEvt == NVM_EVT_NONE)
	{
		return false;
	}

	Evt = Res == 0 ? vOpEvt : NVM_EVT_ERROR;
	Off = vOpOff;
	Len = vOpLen;
	vOpRes = Res;
	vOpEvt = NVM_EVT_NONE;
	vOpRemain = 0;
	vpOpData = nullptr;

	return true;
}

void Nvm::Notify(NVM_EVT Evt, uint64_t Off, uint32_t Len, int Res)
{
	if (vEvtHandler != nullptr)
	{
		vEvtHandler(this, Evt, Off, Len, Res);
	}
}

void Nvm::CancelOp(int Res)
{
	NVM_EVT evt = NVM_EVT_NONE;
	uint64_t off = 0;
	uint32_t len = 0;

	while (atomic_flag_test_and_set(&vOpLock)) {}
	bool notify = FinishOpLocked(Res, evt, off, len);
	vOpRes = 0;
	vOpState = NVM_OPSTATE_IDLE;
	vXferState = NVM_XFER_NONE;
	vbXferWaiting = false;
	vbXferDone = false;
	vbXferFail = false;
	atomic_flag_clear(&vOpLock);

	if (notify)
	{
		Notify(evt, off, len, Res);
	}
}

int Nvm::IssueNext(void)
{
	if (vOpEvt == NVM_EVT_ERASE_DONE)
	{
		int r = EraseUnit(vOpAddr, true);
		if (r < 0)
		{
			return r;
		}
		vOpAddr += vEraseSize;
		vOpRemain -= vEraseSize;
		return 0;
	}

	uint32_t room = vPageSize - (vOpAddr % vPageSize);
	uint32_t l = vOpRemain < room ? vOpRemain : room;
	int r = Program(vOpAddr, vpOpData, l, true);

	if (r < 0)
	{
		return r;
	}

	vOpAddr += l;
	vpOpData += l;
	vOpRemain -= l;

	return 0;
}

// One step of the operation. Every stage change happens here and nowhere
// else, so what an operation can do next is the switch below and not a
// combination of flags that has to be kept consistent.
//
// Returns 1 while there is still work, 0 once the operation has finished.
// A finished operation stays in DONE or FAILED until TakeResult reports it.
int Nvm::ServiceStep(void)
{
	if (vOpState == NVM_OPSTATE_IDLE)
	{
		return 0;
	}

	NVM_EVT evt = NVM_EVT_NONE;
	uint64_t off = 0;
	uint32_t len = 0;
	int notifyRes = 0;
	bool notify = false;

	while (atomic_flag_test_and_set(&vOpLock)) {}

	switch (vOpState)
	{
		case NVM_OPSTATE_IDLE:
		case NVM_OPSTATE_DONE:
		case NVM_OPSTATE_FAILED:
			// Finished. The result waits here for whoever asks.
			atomic_flag_clear(&vOpLock);

			return 0;

		case NVM_OPSTATE_INFLIGHT:
		{
			// A chunk is with the interface. Asking the medium for status
			// now is wrong, and on a mapped memory meaningless.
			NVM_XFERSTATE x = vXferState;

			if (x == NVM_XFER_RUNNING)
			{
				atomic_flag_clear(&vOpLock);

				return 1;
			}

			vXferState = NVM_XFER_NONE;

			if (x == NVM_XFER_FAILED)
			{
				vOpState = NVM_OPSTATE_FAILED;
				notify = FinishOpLocked(-EIO, evt, off, len);
				atomic_flag_clear(&vOpLock);
				if (notify) { Notify(evt, off, len, -EIO); }

				return 0;
			}

			// The interface is done with it; the medium may not be.
			vOpState = NVM_OPSTATE_SETTLING;
			break;
		}

		case NVM_OPSTATE_ISSUING:
		case NVM_OPSTATE_SETTLING:
			break;
	}

	if (vOpState == NVM_OPSTATE_SETTLING)
	{
		bool ready;

		if (vbBare)
		{
			if (vWrDelayUs > 0)
			{
				usDelay(vWrDelayUs);
			}
			ready = true;
		}
		else
		{
			uint8_t sr;
			ready = ReadStatus(sr) == 0 && (sr & NVM_SR_WIP) == 0;
		}

		if (ready == false)
		{
			atomic_flag_clear(&vOpLock);

			return 1;
		}

		if (vOpRemain == 0)
		{
			vOpState = NVM_OPSTATE_DONE;
			notify = FinishOpLocked(0, evt, off, len);
			atomic_flag_clear(&vOpLock);
			if (notify) { Notify(evt, off, len, 0); }

			return 0;
		}

		vOpState = NVM_OPSTATE_ISSUING;
	}

	// ISSUING: hand the next chunk over. IssueNext leaves the transfer in
	// RUNNING when the interface kept it, and in NONE when it finished in
	// the call.
	int r = IssueNext();

	if (r < 0)
	{
		vOpState = NVM_OPSTATE_FAILED;
		notifyRes = r;
		notify = FinishOpLocked(r, evt, off, len);
		atomic_flag_clear(&vOpLock);
		if (notify) { Notify(evt, off, len, notifyRes); }

		return 0;
	}

	vOpState = vXferState == NVM_XFER_NONE ? NVM_OPSTATE_SETTLING
										   : NVM_OPSTATE_INFLIGHT;

	atomic_flag_clear(&vOpLock);

	return 1;
}

// Report the held result once. A caller that asks has been told, so the next
// caller starts clean.
int Nvm::TakeResult(void)
{
	while (atomic_flag_test_and_set(&vOpLock)) {}

	int r = vOpRes;

	vOpRes = 0;

	// Reported, so the operation is over as far as anyone is concerned.
	if (vOpState == NVM_OPSTATE_DONE || vOpState == NVM_OPSTATE_FAILED)
	{
		vOpState = NVM_OPSTATE_IDLE;
	}

	atomic_flag_clear(&vOpLock);

	return r;
}

int Nvm::ServiceRun(uint32_t Timeout)
{
	for (;;)
	{
		if (ServiceStep() <= 0)
		{
			return TakeResult();
		}

		if (Timeout-- == 0 || WaitPoll() == false)
		{
			NVM_EVT evt = NVM_EVT_NONE;
			uint64_t off = 0;
			uint32_t len = 0;

			while (atomic_flag_test_and_set(&vOpLock)) {}

			// Gave up on it. The chunk may still be with the interface, so
			// drop it: a completion arriving later finds nothing running and
			// is discarded.
			vOpState = NVM_OPSTATE_FAILED;
			vXferState = NVM_XFER_NONE;

			bool notify = FinishOpLocked(-ETIMEDOUT, evt, off, len);
			atomic_flag_clear(&vOpLock);

			if (notify)
			{
				Notify(evt, off, len, -ETIMEDOUT);
			}
			return TakeResult();
		}
	}
}

bool Nvm::IsBusy(void) const
{
	return const_cast<Nvm *>(this)->ServiceStep() > 0;
}

int Nvm::Sync(void)
{
	return ServiceRun(NVM_OP_TMOUT);
}

int Nvm::Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData,
			   int DataLen)
{
	DevIntrf_t *ip = *Interface();

	XferBegin();

	if (DeviceIntrfStartTx(ip, DeviceAddress()) == false)
	{
		vbXferWaiting = false;
		return -EBUSY;
	}

	// The frame and the payload are one transaction, so no stop between them.
	ip->bNoStop = true;

	// An erase is command and address with no payload, so the interface can
	// hand it over from this transfer. Armed first, same reason as above.
	if (InterruptEnabled() && (pData == nullptr || DataLen == 0))
	{
		vXferState = NVM_XFER_RUNNING;
	}

	int n = DeviceIntrfTxData(ip, pCmdAddr, CmdAddrLen);

	if (n < 0 && ip->bIntEn)
	{
		// The frame alone started something, which is what an erase looks
		// like: the command and address are the whole of it.
		if (InterruptEnabled() && (pData == nullptr || DataLen == 0))
		{
			// An erase is command and address with no payload, so the
			// interface starts it from this transfer. Handed over the same
			// way as a payload chunk.
			ip->bNoStop = false;

			vbXferWaiting = false;

			return 0;
		}

		// The frame transfer was started and reports through the event.
		if (XferWait(NVM_XFER_TMOUT) == false)
		{
			ip->bNoStop = false;
			DeviceIntrfStopTx(ip);
			return -EIO;
		}
		n = CmdAddrLen;
		XferBegin();
	}

	if (n != CmdAddrLen)
	{
		ip->bNoStop = false;
		DeviceIntrfStopTx(ip);
		vbXferWaiting = false;
		vXferState = NVM_XFER_NONE;

		return -EIO;
	}

	ip->bNoStop = false;

	if (pData == nullptr || DataLen == 0)
	{
		// Finished inside the call, so nothing was handed over.
		if (vXferState == NVM_XFER_RUNNING)
		{
			vXferState = NVM_XFER_NONE;
		}

		DeviceIntrfStopTx(ip);
		vbXferWaiting = false;

		return 0;
	}

	// Armed before the transfer, because the completion can arrive inside it.
	if (InterruptEnabled())
	{
		vXferState = NVM_XFER_RUNNING;
	}

	int wr = DeviceIntrfTxData(ip, pData, DataLen);

	if (wr < 0 && ip->bIntEn)
	{
		// Started and in flight. The interface releases its own busy latch
		// from the completion interrupt, so there is no stop to send here.
		if (InterruptEnabled())
		{
			// Handed over: the interface kept it and will report. Waiting
			// here would put the wait back in the caller, which is the whole
			// point of the mode. No stop is sent either, because on such an
			// interface the completion is what ends the transfer.
			vbXferWaiting = false;

			return DataLen;
		}

		return XferWait(NVM_XFER_TMOUT) ? DataLen : -EIO;
	}

	// Finished inside the call after all, so nothing was handed over.
	if (vXferState == NVM_XFER_RUNNING)
	{
		vXferState = NVM_XFER_NONE;
	}

	DeviceIntrfStopTx(ip);
	vbXferWaiting = false;

	return wr == DataLen ? DataLen : -EIO;
}

int Nvm::Program(uint32_t Addr, const uint8_t *pData, uint32_t Len,
				 bool bDefer)
{
	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	int wr;
	if (vbPhased)
	{
		SPIDev_t *dev = SPIGetHandle(*Interface());
		SPIStartTx(dev, (int)vBaseDevAddr);
		QuadSPISendCmd(dev, vWrCmd.Cmd, Addr, vAddrSize, Len,
						   vWrCmd.DummyCycle);
		wr = SPITxData(dev, pData, (int)Len);
		SPIStopTx(dev);
	}
	else
	{
		uint8_t frame[8];
		uint32_t devaddr;
		int flen = FrameAddr(frame, vWrCmd.Cmd, Addr, &devaddr);

		DeviceAddress(devaddr);
		wr = Write(frame, flen, pData, (int)Len);
	}

	if (wr != (int)Len)
	{
		return wr < 0 ? wr : -EIO;
	}

	if (bDefer)
	{
		return (int)Len;
	}

	return WaitReady() ? (int)Len : -EIO;
}

int Nvm::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	if (vbEnabled == false)
	{
		return -ENODEV;
	}
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}
	if (vWrGran > 1 && ((Off % vWrGran) != 0 || (Len % vWrGran) != 0))
	{
		return -EINVAL;
	}

	int sr = ServiceRun(NVM_OP_TMOUT);
	if (sr < 0)
	{
		return sr;
	}

	uint32_t addr = (uint32_t)(vBaseAddr + RegionOffset() + Off);
	const uint8_t *p = (const uint8_t*)pData;

	if (InterruptEnabled())
	{
		while (atomic_flag_test_and_set(&vOpLock)) {}
		vOpEvt = NVM_EVT_WRITE_DONE;
		vOpState = NVM_OPSTATE_ISSUING;
		vXferState = NVM_XFER_NONE;
		vOpOff = Off;
		vOpLen = Len;
		vOpAddr = addr;
		vpOpData = p;
		vOpRemain = Len;

		int r = IssueNext();
		if (r < 0)
		{
			vOpState = NVM_OPSTATE_IDLE;
			vXferState = NVM_XFER_NONE;
			vOpEvt = NVM_EVT_NONE;
			vOpRemain = 0;
			vpOpData = nullptr;
			atomic_flag_clear(&vOpLock);

			return r;
		}

		// The chunk is either with the interface or already back with the
		// medium, the same reading ServiceStep makes.
		vOpState = vXferState == NVM_XFER_NONE ? NVM_OPSTATE_SETTLING
											   : NVM_OPSTATE_INFLIGHT;

		atomic_flag_clear(&vOpLock);
		return (int)Len;
	}

	uint32_t cnt = Len;
	while (cnt > 0)
	{
		uint32_t room = vPageSize - (addr % vPageSize);
		uint32_t l = cnt < room ? cnt : room;
		int res = Program(addr, p, l, false);

		if (res < 0)
		{
			return res;
		}

		cnt -= (uint32_t)res;
		addr += (uint32_t)res;
		p += res;
	}

	return (int)Len;
}

int Nvm::EraseUnit(uint32_t Addr, bool bDefer)
{
	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	uint8_t op = NVM_CMD_SECT_ERASE;
	if (vEraseSize == 32U * 1024U) { op = NVM_CMD_BLK32_ERASE; }
	else if (vEraseSize == 64U * 1024U) { op = NVM_CMD_BLK64_ERASE; }

	if (vbPhased)
	{
		SPIDev_t *dev = SPIGetHandle(*Interface());
		SPIStartTx(dev, (int)vBaseDevAddr);
		bool ok = QuadSPISendCmd(dev, op, Addr, vAddrSize, 0, 0);
		SPIStopTx(dev);
		if (ok == false)
		{
			return -EIO;
		}
	}
	else
	{
		uint8_t frame[8];
		uint32_t devaddr;
		int flen = FrameAddr(frame, op, Addr, &devaddr);

		DeviceAddress(devaddr);

		int r = Write(frame, flen, nullptr, 0);
		if (r < 0)
		{
			return r;
		}
	}

	if (bDefer)
	{
		return 0;
	}

	return WaitReady() ? 0 : -EIO;
}

int Nvm::Erase(uint64_t Off, uint32_t Len)
{
	if (vbEnabled == false)
	{
		return -ENODEV;
	}
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (vEraseSize == 0)
	{
		return 0;
	}
	if ((Off % vEraseSize) != 0 || (Len % vEraseSize) != 0 || Len == 0)
	{
		return -EINVAL;
	}

	int sr = ServiceRun(NVM_OP_TMOUT);
	if (sr < 0)
	{
		return sr;
	}

	uint32_t addr = (uint32_t)(vBaseAddr + RegionOffset() + Off);

	if (InterruptEnabled())
	{
		while (atomic_flag_test_and_set(&vOpLock)) {}
		vOpEvt = NVM_EVT_ERASE_DONE;
		vOpState = NVM_OPSTATE_ISSUING;
		vXferState = NVM_XFER_NONE;
		vOpOff = Off;
		vOpLen = Len;
		vOpAddr = addr;
		vpOpData = nullptr;
		vOpRemain = Len;

		int r = IssueNext();
		if (r < 0)
		{
			vOpState = NVM_OPSTATE_IDLE;
			vXferState = NVM_XFER_NONE;
			vOpEvt = NVM_EVT_NONE;
			vOpRemain = 0;
			atomic_flag_clear(&vOpLock);

			return r;
		}

		// The unit is either with the interface or already back with the
		// medium, the same reading ServiceStep makes.
		vOpState = vXferState == NVM_XFER_NONE ? NVM_OPSTATE_SETTLING
											   : NVM_OPSTATE_INFLIGHT;

		atomic_flag_clear(&vOpLock);

		return 0;
	}

	uint32_t cnt = Len;
	while (cnt > 0)
	{
		int res = EraseUnit(addr, false);
		if (res < 0)
		{
			return res;
		}
		addr += vEraseSize;
		cnt -= vEraseSize;
	}

	return 0;
}

int Nvm::MassErase(void)
{
	if (vbEnabled == false)
	{
		return -ENODEV;
	}
	if (vEraseSize == 0 || Interface()->Type() == DEVINTRF_TYPE_UNKOWN)
	{
		return -ENOTSUP;
	}

	int sr = ServiceRun(NVM_OP_TMOUT);
	if (sr < 0)
	{
		return sr;
	}

	if (RegionOffset() != 0 || Size() != vDevSize)
	{
		return -EPERM;
	}
	if (WriteEnable() == false)
	{
		return -EBUSY;
	}
	if (SendCmd({ NVM_CMD_CHIP_ERASE, 0 }) != 0)
	{
		return -EIO;
	}
	if (WaitReady((uint32_t)-1) == false)
	{
		return -EBUSY;
	}

	WriteDisable();
	return 0;
}

int Nvm::SetWriteProtect(bool bEnable)
{
	if (vbEnabled == false)
	{
		return -ENODEV;
	}

	// The mechanisms cover the whole part, so a window has no say over it.
	if (RegionOffset() != 0 || Size() != vDevSize)
	{
		return -EPERM;
	}

	int srvc = ServiceRun(NVM_OP_TMOUT);
	if (srvc < 0)
	{
		return srvc;
	}

	if (vbBare || vWrProtMask == 0)
	{
		if (vWrProtPin.PortNo >= 0 && vWrProtPin.PinNo >= 0)
		{
			if (bEnable)
			{
				IOPinSet(vWrProtPin.PortNo, vWrProtPin.PinNo);
			}
			else
			{
				IOPinClear(vWrProtPin.PortNo, vWrProtPin.PinNo);
			}
			return 0;
		}
		return -ENOTSUP;
	}

	uint8_t sr;
	if (ReadStatus(sr) != 0)
	{
		return -EIO;
	}

	if (bEnable) { sr |= vWrProtMask; }
	else { sr &= (uint8_t)~vWrProtMask; }

	if (WriteEnable() == false)
	{
		return -EBUSY;
	}

	if (vbPhased)
	{
		SPIDev_t *dev = SPIGetHandle(*Interface());
		SPIStartTx(dev, (int)vBaseDevAddr);
		QuadSPISendCmd(dev, NVM_CMD_WRSR, (uint32_t)-1, 0, 1, 0);
		int wr = SPITxData(dev, &sr, 1);
		SPIStopTx(dev);
		if (wr != 1)
		{
			WriteDisable();
			return -EIO;
		}
	}
	else
	{
		uint8_t wrsr = NVM_CMD_WRSR;

		if (Write(&wrsr, 1, &sr, 1) != 1)
		{
			WriteDisable();
			return -EIO;
		}
	}

	if (WaitReady() == false)
	{
		WriteDisable();
		return -EBUSY;
	}

	WriteDisable();
	return 0;
}

bool Nvm::Enable(void)
{
	if (vbInitialized == false)
	{
		return false;
	}

	vbEnabled = true;
	Valid(true);
	return true;
}

void Nvm::Disable(void)
{
	if (vbInitialized == false)
	{
		return;
	}

	vbEnabled = false;
	Valid(false);
	CancelOp(-ECANCELED);
	WriteDisable();
}

void Nvm::Reset(void)
{
	if (vbInitialized == false || Interface() == nullptr)
	{
		Valid(false);
		return;
	}

	vbEnabled = false;
	Valid(false);
	CancelOp(-ECANCELED);

	bool ok = true;
	if (vEraseSize != 0)
	{
		ok = SendCmd({ NVM_CMD_RESET_EN, 0 }) == 0 &&
			 SendCmd({ NVM_CMD_RESET, 0 }) == 0;
		if (ok && vAddrSize > 3)
		{
			ok = SendCmd({ NVM_CMD_EN4B, 0 }) == 0;
		}
	}
	else
	{
		WriteDisable();
	}

	vbEnabled = ok;
	Valid(ok);
}
