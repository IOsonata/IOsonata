/**-------------------------------------------------------------------------
@file	nvm.h

@brief	Generic addressed non-volatile memory driver.

One configuration-driven driver supports serial NOR flash, I2C EEPROM, FRAM,
MRAM, RRAM and internal-memory adapters. The transport is supplied through a
DeviceIntrf; the memory behaviour is selected from the configured geometry and
interface type.

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
#ifndef __NVM_H__
#define __NVM_H__

#include <stdint.h>
#include <errno.h>

#include "device.h"
#include "device_intrf.h"
#include "coredev/iopincfg.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

class Nvm;

typedef enum __Nvm_Evt {
	NVM_EVT_NONE,
	NVM_EVT_WRITE_DONE,
	NVM_EVT_ERASE_DONE,
	NVM_EVT_ERROR
} NVM_EVT;

/// Res is 0 on success or a negative errno on failure. Off and Len are
/// relative to the configured region.
typedef void (*NvmEvtHandler_t)(Nvm * const pDev, NVM_EVT Evt,
								uint64_t Off, uint32_t Len, int Res);

/// Called while polling a long operation. Returning false aborts the wait.
typedef bool (*NvmWaitCb_t)(Nvm * const pDev);

/// Device-specific preparation before the generic reset and ID probe.
typedef bool (*NvmInitCb_t)(Nvm * const pDev, DeviceIntrf * const pIntrf);

typedef struct __Nvm_Cmd {
	uint8_t	Cmd;
	uint8_t	DummyCycle;
} NvmCmd_t;

typedef struct __Nvm_Cfg {
	int			DevNo;

	/// Where the memory is mapped. An internal memory adapter hands the frame
	/// address to the controller, so the frame address has to be the address
	/// the part answers at. A chip on a bus addresses from 0 and leaves this 0.
	uint64_t	BaseAddr;

	uint64_t	TotalSize;
	uint32_t	EraseSize;
	uint32_t	SectorSize;
	uint32_t	PageSize;
	uint32_t	WriteGran;
	uint8_t		AddrSize;
	uint32_t	DevId;
	uint8_t		DevIdSize;
	NvmCmd_t	RdCmd;
	NvmCmd_t	WrCmd;
	uint8_t		WrProtMask;
	IOPinCfg_t	WrProtPin;
	uint32_t	WriteDelayUs;

	/// false: Write and Erase wait for the medium.
	/// true: Write and Erase issue one chunk and return; IsBusy(), Sync(), or
	/// the next operation advances the medium state and reports one event.
	/// DeviceIntrf events are transport completion only and never mean that the
	/// memory program or erase operation itself has completed.
	bool			bIntEn;
	NvmEvtHandler_t	EvtHandler;
	NvmWaitCb_t		pWaitCB;
	NvmInitCb_t		pInitCB;
} NvmCfg_t;

class Nvm : virtual public Device {
public:
	Nvm();
	virtual ~Nvm();
	Nvm(Nvm&) = delete;

	using Device::Read;
	using Device::Write;

	bool Init(const NvmCfg_t &Cfg, DeviceIntrf * const pIntrf,
			  uint64_t RegionOff = 0, uint64_t RegionSize = 0);

	virtual uint64_t Size(void) const { return vRegionSize; }
	virtual uint32_t EraseSize(void) const { return vEraseSize; }
	virtual uint32_t WriteGran(void) const { return vWrGran; }
	virtual uint32_t LogicalSectorSize(void) const {
		return vSectSize != 0 ? vSectSize : EraseSize();
	}
	virtual uint32_t PageSize(void) const { return vPageSize; }

	virtual int Read(uint64_t Off, void *pBuf, uint32_t Len);

	/// In interrupt-driven NVM mode, pData must remain valid and unchanged until
	/// the matching completion event has been delivered or Sync() has returned.
	virtual int Write(uint64_t Off, const void *pData, uint32_t Len);

	virtual int Erase(uint64_t Off, uint32_t Len);

	/// Entry called from the DeviceIntrf callback. It records transport
	/// completion or timeout only; it never completes an NVM program or erase.
	void IntrfEvent(DEVINTRF_EVT EvtId);

	/// Protect or release the whole device.
	///
	/// The parts themselves can usually do more than this. Serial NOR block
	/// protect bits cover a range anchored at one end of the array in coarse
	/// power of two steps, and some parts carry a per sector lock bitmap
	/// behind its own commands. Representing that would need the bit layout,
	/// the top or bottom bit and the step size in the config, for a mechanism
	/// almost nothing asks for. The whole device is what this driver offers.
	///
	/// Uses the status register bits when WrProtMask is set, otherwise the
	/// write protect pin. Note that on a serial NOR the pin guards the status
	/// register rather than the array, so it is an EEPROM mechanism; do not
	/// configure it as an array protect on a flash.
	///
	/// Returns -EPERM for a windowed Nvm instance, and -ENOTSUP for a device
	/// with no protection mechanism configured.
	virtual int SetWriteProtect(bool bEnable);

	int MassErase(void);

	/// True while the medium is still working. A failure seen here is held
	/// and reported by Sync() or by the next operation.
	virtual bool IsBusy(void) const;

	/// Drain any operation in flight. Returns 0, or the error of the last
	/// operation to finish if it failed and nothing has reported it yet.
	virtual int Sync(void);

	uint64_t RegionOffset(void) const { return vRegionOffset; }
	uint64_t BaseAddress(void) const { return vBaseAddr; }
	uint32_t ReadId(int Len);
	uint8_t ReadStatus(void);

	int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff,
			 int BuffLen) override;

	/// Sends the command and address frame and the payload as one transaction.
	/// The base implementation gathers both into a local buffer and flattens a
	/// negative count to 0, so it can neither keep the data alive nor report an
	/// interface that started the transfer and completes through the event.
	/// Returns payload bytes written, or a negative errno.
	int Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData,
			  int DataLen) override;

	bool Enable(void) override;
	void Disable(void) override;
	void Reset(void) override;

protected:
	void Region(uint64_t Offset, uint64_t Size) {
		vRegionOffset = Offset;
		vRegionSize = Size;
	}

	bool WaitPoll(void) {
		return vpWaitCB != nullptr ? vpWaitCB(this) : true;
	}

	bool RangeValid(uint64_t Off, uint32_t Len) const {
		return Off <= vRegionSize && Len <= vRegionSize - Off;
	}

	/// Microseconds, not loop iterations. A page erase on internal flash is
	/// tens of milliseconds and can be spread over several radio timeslots.
	bool WaitReady(uint32_t Timeout = 1000000);
	bool WriteEnable(uint32_t Timeout = 1000000);
	void WriteDisable(void);

private:
	int FrameAddr(uint8_t *pFrame, uint8_t Cmd, uint32_t Addr,
				  uint32_t *pDevAddr);
	int Program(uint32_t Addr, const uint8_t *pData, uint32_t Len,
				bool bDefer);
	int EraseUnit(uint32_t Addr, bool bDefer);
	int SendCmd(const NvmCmd_t &Cmd);
	int ReadStatus(uint8_t &Status);

	/// Arm a transfer before the call that can complete inside it.
	void XferBegin(void);

	/// Stand a transfer down when it turns out nothing was handed over.
	void XferEnd(void);
	bool XferShort(int Count, int Expect);
	bool XferWait(uint32_t Timeout);

	/// 1 while the medium is still working, 0 otherwise. A failure ends the
	/// operation and leaves its result in vOpRes for TakeResult to report.
	int ServiceStep(void);
	int ServiceRun(uint32_t Timeout);
	int IssueNext(void);
	int TakeResult(void);

	bool FinishOpLocked(int Res, NVM_EVT &Evt, uint64_t &Off, uint32_t &Len);
	void Notify(NVM_EVT Evt, uint64_t Off, uint32_t Len, int Res);
	void CancelOp(int Res);
	void Unhook(void);

	uint64_t		vRegionOffset;
	uint64_t		vRegionSize;
	NvmWaitCb_t	vpWaitCB;
	NvmEvtHandler_t	vEvtHandler;

	/// Where the operation has got to. One value at a time, so the stages
	/// cannot contradict each other the way a set of flags can.
	///
	///	IDLE      nothing running
	///	ISSUING   the next chunk is to be handed to the interface
	///	INFLIGHT  a chunk is with the interface and its completion is owed
	///	SETTLING  the medium is finishing the chunk it was given
	///	DONE      finished, result not yet reported
	///	FAILED    finished badly, result not yet reported
	///
	/// DONE and FAILED persist until a call reports them, which is how a
	/// failure seen by IsBusy survives to the next Sync or operation.
	typedef enum {
		NVM_OPSTATE_IDLE = 0,
		NVM_OPSTATE_ISSUING,
		NVM_OPSTATE_INFLIGHT,
		NVM_OPSTATE_SETTLING,
		NVM_OPSTATE_DONE,
		NVM_OPSTATE_FAILED
	} NVM_OPSTATE;

	/// One transfer handed to the interface. A completion is only accepted
	/// while one is RUNNING, which is what keeps it from ending a transfer
	/// that has already been consumed, or one that never started.
	///
	/// There is one of these and not two. Whether the transfer is waited out
	/// inside the call or left for ServiceStep to pick up is a question of
	/// who consumes it, not of what state it is in, and the caller already
	/// knows which it is doing. Keeping a second set of flags for the waited
	/// case meant one interface completion had two places it could land and
	/// the arriving event had to choose between them.
	typedef enum {
		NVM_XFER_NONE = 0,		//!< nothing outstanding
		NVM_XFER_RUNNING,		//!< handed over, completion owed
		NVM_XFER_COMPLETE,		//!< its completion arrived
		NVM_XFER_FAILED			//!< its completion said it failed
	} NVM_XFERSTATE;

	NVM_OPSTATE		vOpState;
	volatile NVM_XFERSTATE	vXferState;

	NVM_EVT			vOpEvt;

	/// Result of the last operation to finish, held until a call reports it.
	/// Without this a failure observed by IsBusy would be gone by the time
	/// Sync or the next operation asks.
	int				vOpRes;
	uint64_t		vOpOff;
	uint32_t		vOpLen;
	uint32_t		vOpAddr;
	const uint8_t	*vpOpData;
	uint32_t		vOpRemain;
	atomic_flag		vOpLock;

	bool		vbInitialized;
	bool		vbEnabled;
	uint64_t	vBaseAddr;
	uint64_t	vDevSize;
	uint32_t	vEraseSize;
	uint32_t	vSectSize;
	uint32_t	vPageSize;
	uint32_t	vWrGran;
	int			vAddrSize;
	uint32_t	vAddrSpan;
	uint32_t	vWrDelayUs;
	uint8_t		vWrProtMask;
	NvmCmd_t	vRdCmd;
	NvmCmd_t	vWrCmd;
	bool		vbBare;
	bool		vbPhased;
	uint32_t	vBaseDevAddr;
	IOPinCfg_t	vWrProtPin;
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_H__
