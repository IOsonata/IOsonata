/**-------------------------------------------------------------------------
@file	nvm.cpp

@brief	Serial non volatile memory driver.

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
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the app configured (UART, USB, RTT, BLE, or any other
// DeviceIntrf); the trace does not assume a transport. A release build
// defines NDEBUG, which strips all trace regardless of DEBUG_ENABLE.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// Status register bits common to serial flash.
#define NVM_SR_WIP			0x01U		// write in progress
#define NVM_SR_WEL			0x02U		// write enable latch

// Read the device id where the medium has the usual JEDEC command.
#define NVM_CMD_READID		0x9FU

// The serial NOR protocol the JEDEC standard fixed. An erase medium speaks
// all of it; none of it is configuration, because none of it varies.
#define NVM_CMD_READ		0x03U
#define NVM_CMD_WRITE		0x02U
#define NVM_CMD_WRENABLE	0x06U
#define NVM_CMD_WRDISABLE	0x04U
#define NVM_CMD_RDSR		0x05U
#define NVM_CMD_WRSR		0x01U
#define NVM_CMD_SECT_ERASE	0x20U		// 4K granule
#define NVM_CMD_BLK32_ERASE	0x52U		// 32K granule
#define NVM_CMD_BLK64_ERASE	0xD8U		// 64K granule
#define NVM_CMD_CHIP_ERASE	0xC7U
#define NVM_CMD_RESET_EN	0x66U
#define NVM_CMD_RESET		0x99U

// Bounds for an in-call interface transfer wait and for draining a running
// operation, in cooperative wait iterations.
#define NVM_XFER_TMOUT		100000
#define NVM_OP_TMOUT		100000
#define NVM_CMD_EN4B		0xB7U

int Nvm::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	// Straight through; a memory command is not a sensor register address.
	return Interface()->Read(DeviceAddress(), pCmdAddr, CmdAddrLen,
							 pBuff, BuffLen);
}

Nvm::Nvm()
{
	Valid(false);
	vRegionOffset = 0;
	vRegionSize = 0;
	vpWaitCB = nullptr;
	vbIntEn = false;
	vEvtHandler = nullptr;
	vOpEvt = NVM_EVT_NONE;
	vOpRes = 0;
	vOpOff = 0;
	vOpLen = 0;
	vOpAddr = 0;
	vpOpData = nullptr;
	vOpRemain = 0;
	vbXferWaiting = false;
	vbXferDone = false;
	vbXferFail = false;
	vbMediumDone = false;
	atomic_flag_clear(&vOpLock);
	vDevSize = 0;
	vSectSize = 0;
	vEraseSize = 0;
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
	vWrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL };
}

Nvm::~Nvm()
{
	Unhook();
}

int Nvm::FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr,
				   uint32_t *pDevAddr)
{
	int len = 0;
	uint32_t devaddr = vBaseDevAddr;

	// A command byte where the medium has one. An EEPROM has none and the
	// frame is the address alone.
	if (Cmd != 0)
	{
		pFrame[len++] = Cmd;
	}

	// Address MSB first on the wire.
	for (int i = 0; i < vAddrSize; i++)
	{
		pFrame[len++] = (uint8_t)(Addr >> (8 * (vAddrSize - 1 - i)));
	}

	// Bits the address bytes cannot hold go into the device selection, the way
	// a small EEPROM selects a memory block.
	if (vAddrSpan != 0 && Addr >= vAddrSpan)
	{
		devaddr |= (Addr / vAddrSpan) & 7;
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

	// A command only transaction has no payload, so Device::Write's payload
	// count is zero whether it worked or not. Only the full transfer count
	// distinguishes the two.
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

		return (rd == 1) ? 0 : -EIO;
	}

	uint8_t cmd = NVM_CMD_RDSR;

	XferBegin();
	int n = Read(&cmd, 1, &Status, 1);

	return XferShort(n, 1) ? 0 : -EIO;
}

uint8_t Nvm::ReadStatus(void)
{
	uint8_t sr = 0;

	(void)ReadStatus(sr);

	return sr;
}

uint32_t Nvm::ReadId(int Len)
{
	uint32_t id = 0;
	uint8_t cmd = NVM_CMD_READID;

	if (Len < 1 || Len > 4)
	{
		return 0;
	}

	if (vbPhased)
	{
		SPIDev_t *dev = SPIGetHandle(*Interface());

		SPIStartRx(dev, (int)vBaseDevAddr);
		QuadSPISendCmd(dev, cmd, (uint32_t)-1, 0, Len, 0);
		int rd = SPIRxData(dev, (uint8_t*)&id, Len);
		SPIStopRx(dev);

		return (rd == Len) ? id : 0;
	}

	XferBegin();
	int n = Read(&cmd, 1, (uint8_t*)&id, Len);

	if (XferShort(n, Len) == false)
	{
		return 0;
	}

	return id;
}

bool Nvm::WaitReady(uint32_t Timeout)
{
	// A medium with a status register says when it is done. One without takes
	// a known time, which the config gives.
	if (vbBare)
	{
		// No status register. An EEPROM in its write cycle does not
		// acknowledge its address, so a one byte current address read
		// answers only when the cycle is over. This is the standard
		// acknowledge poll; it also spends no time when the medium is
		// already idle.
		do {
			uint8_t b;

			XferBegin();
			if (XferShort(Interface()->Rx((uint32_t)vBaseDevAddr, &b, 1), 1))
			{
				return true;
			}
			if (WaitPoll() == false)
			{
				return false;
			}
		} while (Timeout-- > 0);

		return false;
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
	// A bare bus has no latch: always writable.
	if (vbBare)
	{
		return true;
	}

	if (WaitReady(Timeout) == false)
	{
		return false;
	}
	if (SendCmd({ NVM_CMD_WRENABLE, 0 }) != 0)
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
	if (vbBare == false)
	{
		SendCmd({ NVM_CMD_WRDISABLE, 0 });
	}
}

// The interface event callback has no user context slot, so the instances
// running interrupt driven register here. The trampoline dispatches to every
// instance on the reporting interface; each checks its own deferred state.
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
	// A failed reinitialization must not leave a previously working instance
	// marked ready.
	Valid(false);
	Region(0, 0);

	if (pIntrf == nullptr)
	{
		return false;
	}

	Interface(pIntrf);
	DeviceAddress(Cfg.DevNo);

	vDevSize = Cfg.TotalSize;
	vEraseSize = Cfg.EraseSize;
	vSectSize = Cfg.SectorSize;
	vPageSize = Cfg.PageSize;
	vWrGran = (Cfg.WriteGran != 0) ? Cfg.WriteGran : 1;
	vAddrSize = (Cfg.AddrSize != 0) ? Cfg.AddrSize : 3;
	vBaseDevAddr = (uint32_t)Cfg.DevNo;
	vWrDelayUs = Cfg.WriteDelayUs;
	vWrProtMask = Cfg.WrProtMask;

	// The current implementation is the polling SPI/I2C path plus the internal
	// command-emulating adapter. QSPI/OSPI need their command, phase-width and
	// dummy-cycle transfer path before they can be accepted here.
	switch (pIntrf->Type())
	{
		case DEVINTRF_TYPE_I2C:
		case DEVINTRF_TYPE_SPI:
		case DEVINTRF_TYPE_UNKOWN:
		// The phased transaction path serves these two.
		case DEVINTRF_TYPE_QSPI:
		case DEVINTRF_TYPE_OSPI:
			break;

		default:
			return false;
	}

	// The bus is one of the two facts every command follows from; the kind,
	// from EraseSize, is the other. Nothing else is stored: each operation
	// decides its command where it runs.
	vbBare = (pIntrf->Type() == DEVINTRF_TYPE_I2C);
	vbPhased = (pIntrf->Type() == DEVINTRF_TYPE_QSPI
				|| pIntrf->Type() == DEVINTRF_TYPE_OSPI);

	if (vbPhased)
	{
		// The phased interface maps the device; it needs the size.
		QuadSPISetMemSize(SPIGetHandle(*pIntrf), (uint32_t)vDevSize);
	}

	if (vbBare)
	{
		vRdCmd = Cfg.RdCmd;
		vWrCmd = Cfg.WrCmd;
	}
	else
	{
		vRdCmd = (Cfg.RdCmd.Cmd != 0) ? Cfg.RdCmd
									  : NvmCmd_t{ NVM_CMD_READ, 0 };
		vWrCmd = (Cfg.WrCmd.Cmd != 0) ? Cfg.WrCmd
									  : NvmCmd_t{ NVM_CMD_WRITE, 0 };
	}
	vWrProtPin = Cfg.WrProtPin;

	if (vDevSize == 0 || vPageSize == 0 || vAddrSize < 1 || vAddrSize > 4 ||
		Cfg.DevIdSize > 4)
	{
		DEBUG_PRINTF("Nvm geometry invalid: size=%lu page=%lu addr=%d id=%u\r\n",
					 (unsigned long)vDevSize, (unsigned long)vPageSize,
					 vAddrSize, Cfg.DevIdSize);
		return false;
	}

	// Only these standard erase commands are implemented. Accepting another
	// granule would issue a 4K command while advancing by the configured size.
	if (vEraseSize != 0 && vEraseSize != 4 * 1024 &&
		vEraseSize != 32 * 1024 && vEraseSize != 64 * 1024)
	{
		DEBUG_PRINTF("Nvm erase size unsupported: %lu\r\n",
					 (unsigned long)vEraseSize);
		return false;
	}

	// What the address bytes can reach. Anything beyond has to go into the
	// device selection.
	vAddrSpan = (vAddrSize >= 4) ? 0 : (1UL << (8 * vAddrSize));
	if (vAddrSpan != 0 && vDevSize <= vAddrSpan)
	{
		vAddrSpan = 0;			// the address bytes cover the whole device
	}

	vbIntEn = Cfg.bIntEn;
	vEvtHandler = Cfg.EvtHandler;
	vOpEvt = NVM_EVT_NONE;
	vOpRes = 0;
	vOpRemain = 0;
	vbXferWaiting = false;
	vbXferDone = false;
	vbXferFail = false;
	vbMediumDone = false;
	vpWaitCB = Cfg.pWaitCB;

	// Configure the write protect pin and start unprotected.
	if (vWrProtPin.PortNo >= 0 && vWrProtPin.PinNo >= 0)
	{
		IOPinCfg(&vWrProtPin, 1);
		IOPinClear(vWrProtPin.PortNo, vWrProtPin.PinNo);
	}

	// Device-specific preparation comes first, matching the legacy Flash
	// driver. Some parts must be released or put into a command mode before
	// the generic reset and ID probe can be understood.
	if (Cfg.pInitCB != nullptr)
	{
		if (Cfg.pInitCB(this, pIntrf) == false)
		{
			return false;
		}
	}

	// Reset an erase medium before the normal status and ID transactions. The
	// chip does not reset when the MCU does and can be left in power down,
	// continuous read, quad mode, 4 byte address mode, or mid operation.
	if (vEraseSize != 0)
	{
		if (SendCmd({ NVM_CMD_RESET_EN, 0 }) != 0 ||
			SendCmd({ NVM_CMD_RESET, 0 }) != 0)
		{
			return false;
		}
	}

	// Probe the device id where the config asked for it. The retries cover the
	// device's recovery time after the reset above.
	if (Cfg.DevId != 0 && Cfg.DevIdSize > 0)
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
			DEBUG_PRINTF("Nvm id mismatch\r\n");
			return false;
		}
	}

	// A part larger than 16 MBytes takes its addresses in 4 bytes; the
	// reset above cleared the mode, so it is entered last.
	if (vEraseSize != 0 && vAddrSize > 3)
	{
		if (SendCmd({ NVM_CMD_EN4B, 0 }) != 0)
		{
			return false;
		}
	}

	if (RegionOff > vDevSize)
	{
		return false;
	}

	uint64_t avail = vDevSize - RegionOff;

	if (RegionSize > avail)
	{
		return false;
	}

	uint64_t rsize = (RegionSize != 0) ? RegionSize : avail;

	// The physical placement must respect the write granularity, or a frame
	// starting mid word would silently lose its tail; and an erase region
	// must start on an erase unit.
	if ((RegionOff % vWrGran) != 0 || (vPageSize % vWrGran) != 0)
	{
		return false;
	}
	if (vEraseSize != 0 && (RegionOff % vEraseSize) != 0)
	{
		return false;
	}
	Region(RegionOff, rsize);

	// Any registration from a previous initialization, possibly on another
	// interface, is dropped first.
	Unhook();

	if (vbIntEn)
	{
		// Interrupt driven: completion arrives through the interface event.
		// Take the interface callback when it is free; an application that
		// owns it calls IntrfEvent from its own handler instead.
		DevIntrf_t *ip = *pIntrf;
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
			// A full table would run with no completion path: fail here
			// rather than silently.
			return false;
		}
		s_NvmHook[slot].pIntrf = ip;
		s_NvmHook[slot].pNvm = this;
		if (ip->EvtCB == nullptr)
		{
			ip->EvtCB = NvmIntrfEvtCB;
		}
	}

	Valid(true);

	return true;
}

int Nvm::Read(uint64_t Off, void *pBuf, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}

	{
		int sr = ServiceRun(NVM_OP_TMOUT);

		if (sr < 0)
		{
			// The previous operation's completion failed and was reported;
			// this call does not start on top of it.
			return sr;
		}
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	uint8_t *pb = (uint8_t*)pBuf;
	uint32_t cnt = Len;

	while (cnt > 0)
	{
		uint8_t frame[8];
		uint32_t devaddr;
		int flen = FrameAddr(frame, vRdCmd.Cmd, addr, &devaddr);

		// Split only where the device selection holds part of the address,
		// so a read does not run past what this selection reaches.
		uint32_t l = cnt;
		if (vAddrSpan != 0)
		{
			uint32_t r = vAddrSpan - (addr % vAddrSpan);
			if (l > r) { l = r; }
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
				// On an interrupt driven interface the data landed in the
				// buffer by the time the event ended the wait.
				rd = (int)l;
			}
		}
		if (rd != (int)l)
		{
			return -EIO;
		}
		addr += rd;
		pb += rd;
		cnt -= rd;
	}

	return (int)Len;
}

void Nvm::IntrfEvent(DEVINTRF_EVT EvtId)
{
	switch (EvtId)
	{
		case DEVINTRF_EVT_COMPLETED:
		case DEVINTRF_EVT_TX_READY:
		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			if (vbXferWaiting)
			{
				// A call waits on its transfer: this event ends the wait.
				vbXferDone = true;
				return;
			}
			if (vOpEvt != NVM_EVT_NONE)
			{
				// No transfer is waited on: an adapter whose machinery knows
				// the medium reports the operation itself done. A serial
				// transfer end never lands here. Finish from this context
				// only when nothing remains to transfer; otherwise the flag
				// lets the service advance without a status transaction.
				vbMediumDone = true;
				if (vOpRemain == 0
					&& atomic_flag_test_and_set(&vOpLock) == false)
				{
					if (vOpEvt != NVM_EVT_NONE && vOpRemain == 0)
					{
						OpComplete(vOpRes);
					}
					atomic_flag_clear(&vOpLock);
				}
			}
			return;

		case DEVINTRF_EVT_TX_TIMEOUT:
		case DEVINTRF_EVT_RX_TIMEOUT:
			if (vbXferWaiting)
			{
				vbXferFail = true;
			}
			return;

		default:
			return;
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
	if (((DevIntrf_t*)*Interface())->bIntEn == false)
	{
		vbXferWaiting = false;
		return false;
	}

	// The interface started the transfer: the ports report -1 or a short
	// count in interrupt mode, and the event ends the wait.
	return XferWait(NVM_XFER_TMOUT);
}

void Nvm::OpComplete(int Res)
{
	NVM_EVT evt = (Res == 0) ? vOpEvt : NVM_EVT_ERROR;
	uint64_t off = vOpOff;
	uint32_t len = vOpLen;

	vOpEvt = NVM_EVT_NONE;
	vbMediumDone = false;

	if (vEvtHandler != nullptr)
	{
		vEvtHandler(this, evt, off, len, Res);
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

int Nvm::ServiceStep(void)
{
	if (vOpEvt == NVM_EVT_NONE)
	{
		return 0;
	}

	while (atomic_flag_test_and_set(&vOpLock)) {}

	if (vOpEvt == NVM_EVT_NONE)
	{
		atomic_flag_clear(&vOpLock);
		return 0;
	}

	bool ready;

	if (vbMediumDone)
	{
		ready = true;
		vbMediumDone = false;
	}
	else if (vbBare)
	{
		// Acknowledge poll, one attempt: an EEPROM in its write cycle does
		// not acknowledge, so a one byte current address read answers only
		// when the cycle is over.
		uint8_t b;

		XferBegin();
		ready = XferShort(Interface()->Rx((uint32_t)vBaseDevAddr, &b, 1), 1);
	}
	else
	{
		uint8_t sr;

		ready = (ReadStatus(sr) == 0 && (sr & NVM_SR_WIP) == 0);
	}

	if (ready == false)
	{
		atomic_flag_clear(&vOpLock);
		return 1;
	}

	if (vOpRemain == 0)
	{
		int res = vOpRes;

		OpComplete(res);
		atomic_flag_clear(&vOpLock);
		return res;
	}

	int r = IssueNext();

	if (r < 0)
	{
		OpComplete(r);
		atomic_flag_clear(&vOpLock);
		return r;
	}

	atomic_flag_clear(&vOpLock);

	return 1;
}

int Nvm::ServiceRun(uint32_t Timeout)
{
	for (;;)
	{
		int r = ServiceStep();

		if (r <= 0)
		{
			return r;
		}
		if (Timeout-- == 0 || WaitPoll() == false)
		{
			while (atomic_flag_test_and_set(&vOpLock)) {}
			if (vOpEvt != NVM_EVT_NONE)
			{
				OpComplete(-ETIMEDOUT);
			}
			atomic_flag_clear(&vOpLock);

			return -ETIMEDOUT;
		}
	}
}

bool Nvm::IsBusy(void) const
{
	// The polling service: one step. Const to match the storage verb; the
	// step is the service this call provides.
	return const_cast<Nvm *>(this)->ServiceStep() > 0;
}

int Nvm::Sync(void)
{
	return ServiceRun(NVM_OP_TMOUT);
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
		// Device::Write returns payload bytes; the command/address frame is
		// not included in this count.
		XferBegin();
		wr = Device::Write(frame, flen, pData, (int)Len);
		if (XferShort(wr, (int)Len))
		{
			wr = (int)Len;
		}
	}
	if (wr != (int)Len)
	{
		return -EIO;
	}

	if (bDefer)
	{
		// The transfer is done; the medium programs in the background.
		return (int)Len;
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	return (int)Len;
}

int Nvm::Write(uint64_t Off, const void *pData, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}
	if (Len == 0)
	{
		return 0;
	}
	// A memory that programs whole words cannot take a part of one.
	if (vWrGran > 1 && ((Off % vWrGran) != 0 || (Len % vWrGran) != 0))
	{
		return -EINVAL;
	}

	{
		int sr = ServiceRun(NVM_OP_TMOUT);

		if (sr < 0)
		{
			// The previous operation's completion failed and was reported;
			// this call does not start on top of it.
			return sr;
		}
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);
	const uint8_t *p = (const uint8_t*)pData;

	if (vbIntEn)
	{
		// The whole call is one operation: its state is established before
		// the first transfer starts, then one chunk goes out and the call
		// returns. The service and the interface event advance the rest and
		// report one event for the call.
		while (atomic_flag_test_and_set(&vOpLock)) {}
		vOpEvt = NVM_EVT_WRITE_DONE;
		vOpRes = 0;
		vOpOff = Off;
		vOpLen = Len;
		vOpAddr = addr;
		vpOpData = p;
		vOpRemain = Len;

		int r = IssueNext();

		if (r < 0)
		{
			vOpEvt = NVM_EVT_NONE;
			atomic_flag_clear(&vOpLock);
			return r;
		}
		atomic_flag_clear(&vOpLock);

		return (int)Len;
	}

	uint32_t cnt = Len;

	// Split at page boundaries. The address counter auto increments only
	// within a page and wraps at the boundary, so one transfer must stay
	// inside a page. Each chunk reissues the address for its page.
	while (cnt > 0)
	{
		uint32_t r = vPageSize - (addr % vPageSize);
		uint32_t l = cnt < r ? cnt : r;

		int res = Program(addr, p, l, false);
		if (res < 0)
		{
			return res;
		}
		cnt -= res;
		addr += res;
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
	if (vEraseSize == 32 * 1024) { op = NVM_CMD_BLK32_ERASE; }
	else if (vEraseSize == 64 * 1024) { op = NVM_CMD_BLK64_ERASE; }

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
		XferBegin();
		if (XferShort(Interface()->Tx(devaddr, frame, flen), flen) == false)
		{
			return -EIO;
		}
	}

	if (bDefer)
	{
		// The command is out; the medium erases in the background.
		return 0;
	}

	if (WaitReady() == false)
	{
		return -EIO;
	}

	return 0;
}

int Nvm::Erase(uint64_t Off, uint32_t Len)
{
	if (!RangeValid(Off, Len))
	{
		return -EINVAL;
	}

	// A medium that overwrites directly has nothing to erase.
	if (vEraseSize == 0)
	{
		return 0;
	}

	if ((Off % vEraseSize) != 0 || (Len % vEraseSize) != 0 || Len == 0)
	{
		return -EINVAL;
	}

	{
		int sr = ServiceRun(NVM_OP_TMOUT);

		if (sr < 0)
		{
			// The previous operation's completion failed and was reported;
			// this call does not start on top of it.
			return sr;
		}
	}

	uint32_t addr = (uint32_t)(RegionOffset() + Off);

	if (vbIntEn)
	{
		// One operation for the call, state first, one unit out, return.
		while (atomic_flag_test_and_set(&vOpLock)) {}
		vOpEvt = NVM_EVT_ERASE_DONE;
		vOpRes = 0;
		vOpOff = Off;
		vOpLen = Len;
		vOpAddr = addr;
		vpOpData = nullptr;
		vOpRemain = Len;

		int r = IssueNext();

		if (r < 0)
		{
			vOpEvt = NVM_EVT_NONE;
			atomic_flag_clear(&vOpLock);
			return r;
		}
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
	// The internal memory adapter reports UNKNOWN because it emulates the
	// serial command frame. It supports unit erase, but never chip erase of the
	// device executing this code.
	if (vEraseSize == 0 || Interface()->Type() == DEVINTRF_TYPE_UNKOWN)
	{
		return -ENOTSUP;
	}

	{
		int sr = ServiceRun(NVM_OP_TMOUT);

		if (sr < 0)
		{
			// The previous operation's completion failed and was reported;
			// this call does not start on top of it.
			return sr;
		}
	}
	if (RegionOffset() != 0 || Size() != vDevSize)
	{
		// The command wipes the whole device, so only an instance covering all
		// of it may issue one.
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

int Nvm::SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable)
{
	(void)Off;
	(void)Len;

	{
		int sr = ServiceRun(NVM_OP_TMOUT);

		if (sr < 0)
		{
			// The previous operation's completion failed and was reported;
			// this call does not start on top of it.
			return sr;
		}
	}

	// Block protect bits where the medium has them. Checked first because a
	// config left at zero would otherwise look like it had a pin on port 0.
	if (vbBare || vWrProtMask == 0)
	{
		// No status bits, so a pin if one is configured.
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

	if (bEnable)
	{
		sr |= vWrProtMask;
	}
	else
	{
		sr &= (uint8_t)~vWrProtMask;
	}

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

		XferBegin();
		if (XferShort(Device::Write(&wrsr, 1, &sr, 1), 1) == false)
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

void Nvm::Disable(void)
{
	WriteDisable();
}

void Nvm::Reset(void)
{
	WriteDisable();
}
