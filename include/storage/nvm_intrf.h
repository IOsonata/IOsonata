/**-------------------------------------------------------------------------
@file	nvm_intrf.h

@brief	The MCU internal memory as a DeviceIntrf.

Internal memory is a device like any other: it takes a command, an address and
data. It simply has no wire, so this interface stands in the position a SPI or
an I2C occupies for a flash or an EEPROM chip, and the same Nvm driver serves
it with no code of its own.

There is one class, here, and one implementation per target in its own file
(nvm_nrfx.cpp on the nRF). An application declares the interface the way it
declares a UART or a SPI, and names nothing target specific. The target
implementation owns the controller access and the arbitration with whatever
stack runs.

Usage :

	NvmIntrf g_MemIntrf;
	NvmCfg_t cfg;

	g_MemIntrf.Init();

	memset(&cfg, 0, sizeof(cfg));
	NvmIntrfCfg(cfg);

	Nvm g_Nvm;
	g_Nvm.Init(cfg, &g_MemIntrf, RegionAddr, RegionSize);

The region placement is the application's decision. NvmIntrfCeiling() reports
where the memory the application owns ends, which is not always the device
size: a stack image or a reserved partition can sit at the top, and writing
there destroys it.

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
#ifndef __NVM_INTRF_H__
#define __NVM_INTRF_H__

#include "device_intrf.h"
#include "storage/nvm.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// One unit of a memory operation, run where it is safe to touch the memory.
/// Returns 0 when the operation is finished, or the microseconds it still
/// expects to need so the arbiter can size the next window.
///
/// It runs wherever the arbiter puts it, which for a radio timeslot is a high
/// priority interrupt: short, deterministic, no blocking calls.
typedef struct __Nvm_Intrf_Op {
	uint32_t (*Step)(void *pCtx);	//!< One unit. 0 when done, else usec left
	uint32_t	StepBudgetUs;		//!< Worst case for one Step, sizes the window
	void		*pCtx;				//!< Passed back to Step

	/// Where to report the result, or NULL to have the arbiter wait for it
	/// and return it instead. Set it and the arbiter submits and returns, so
	/// the caller is not held while the memory works. It is called from
	/// wherever the arbiter finishes, which for a radio timeslot is a high
	/// priority interrupt: do nothing there but hand the result over.
	///
	/// The operation and its context outlive the call in that case, so
	/// neither may be a local.
	void (*Done)(void *pCtx, int Res);
} NvmIntrfOp_t;

/// The arbiter answers this when the operation is under way and Done will
/// report it. Positive, so a check for a negative errno is unchanged.
#define NVM_INTRF_OP_STARTED		1

/// Runs an operation where the memory is safe to touch. Returns 0 when it
/// completed in the call, NVM_INTRF_OP_STARTED when Done will report it, or
/// a negative errno when it could not be started.
typedef int (*NvmIntrfArb_t)(NvmIntrfOp_t *pOp);

/// Counts of what the interface did, for a test to show which path ran.
typedef struct __Nvm_Intrf_Stat {
	uint32_t	Ops;		//!< Operations completed
	uint32_t	Busy;		//!< Refused while a stack held the memory
	uint32_t	Evt;		//!< Results delivered as a stack event
	uint32_t	Skipped;	//!< Erases skipped, the unit already erased

	// Which of the three arbitration paths carried the operation. Ops counts
	// them all together, so on its own it cannot tell an operation that went
	// through a stack from one that drove the controller with the radio up,
	// and those two are the pair worth telling apart.
	uint32_t	Sd;			//!< Submitted to the SoftDevice
	uint32_t	Slot;		//!< Run inside an MPSL timeslot
	uint32_t	Direct;		//!< Drove the controller, the memory was ours

	// Completions the interface saw and what became of them. Passed on plus
	// dropped should equal Evt; a dropped one leaves the driver waiting for a
	// completion that has already been and gone.
	uint32_t	RepDone;	//!< Passed on to the driver
	uint32_t	RepNoWant;	//!< Dropped, nothing asked to be told
	uint32_t	RepNoPend;	//!< Dropped, nothing was outstanding
} NvmIntrfStat_t;

/// Command byte plus a 32 bit address. The frame the driver puts in front of
/// a transfer, the way FlashEraseSector does.
#define NVM_INTRF_FRAME_SIZE		5

/// Largest payload one transfer stages. The driver splits at the page size it
/// was configured with, which no target sets above this.
#ifndef NVM_INTRF_XFER_SIZE
#define NVM_INTRF_XFER_SIZE			64
#endif

/// One transaction's worth of state. It belongs to the instance, not to the
/// file, the same way nRFSpiDev_t holds a transfer's buffer and index, and it
/// is what pDevData points at.
/// A word write, wherever it is carried out. The address is an integer so one
/// context serves a controller write and a stack submit that wants a pointer;
/// there is one write in flight either way.
typedef struct __Nvm_Intrf_Wr_Ctx {
	uintptr_t		Addr;			//!< Where the words go
	const uint32_t	*pSrc;			//!< The words
	uint32_t		Words;			//!< How many
} NvmIntrfWrCtx_t;

/// An erase of one unit. Addr is the address, or the page number where the
/// stack asks for one.
typedef struct __Nvm_Intrf_Er_Ctx {
	uintptr_t		Addr;			//!< The unit
	bool			bStarted;		//!< The erase has been kicked off
} NvmIntrfErCtx_t;

/// The interface and everything one transaction needs, in one object.
///
/// DevIntrf_t is the first member, so pDevData is a handle that casts to
/// either: a transfer callback gets the state, and a C caller can hold the
/// whole thing without knowing about the class that owns it.
///
/// The state is per transaction, so it belongs here and not at file scope,
/// the same way nRFSpiDev_t holds a transfer's buffer and index. That takes
/// in the arbitrated operation and its context: the stack and the arbiter
/// both report after the call has returned, so both outlive it.
///
/// Packed to 4 so the staged payload keeps the alignment word wise transfers
/// need, without alignas, which C does not have, so the header stays
/// includable from C.
#pragma pack(push, 4)
typedef struct __Nvm_Dev_Intrf {
	DevIntrf_t		DevIntrf;		//!< First, so the handle casts both ways
	uint8_t			Hdr[NVM_INTRF_FRAME_SIZE];	//!< Command and address frame
	int				HdrLen;			//!< Frame bytes taken so far
	int				DataLen;		//!< Payload bytes staged
	bool volatile	bAsyncPending;	//!< A submit is owed a completion

	NvmIntrfOp_t	Op;				//!< The arbitrated operation
	union {
		NvmIntrfWrCtx_t	Wr;
		NvmIntrfErCtx_t	Er;
	} OpCtx;						//!< One operation is in flight at a time

	uint8_t			Data[NVM_INTRF_XFER_SIZE];	//!< Staged payload
} NvmDevIntrf_t;
#pragma pack(pop)

/// @brief	The MCU memory controller as a device interface.
class NvmIntrf : public DeviceIntrf {
public:
	bool Init(void);

	operator DevIntrf_t * const () override { return &vDevIntrf.DevIntrf; }

	uint32_t Rate(uint32_t RateHz) override { (void)RateHz; return 0; }
	uint32_t Rate(void) override { return 0; }

	bool StartRx(uint32_t DevAddr) override {
		return DeviceIntrfStartRx(&vDevIntrf.DevIntrf, DevAddr);
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vDevIntrf.DevIntrf, pBuff, BuffLen);
	}
	void StopRx(void) override { DeviceIntrfStopRx(&vDevIntrf.DevIntrf); }

	bool StartTx(uint32_t DevAddr) override {
		return DeviceIntrfStartTx(&vDevIntrf.DevIntrf, DevAddr);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vDevIntrf.DevIntrf, pData, DataLen);
	}
	void StopTx(void) override { DeviceIntrfStopTx(&vDevIntrf.DevIntrf); }

protected:
	NvmDevIntrf_t	vDevIntrf;
};

/**
 * @brief	Register who decides when the memory may be touched.
 *
 * On a part with a radio, writing the memory has to be arbitrated against it,
 * and only the stack knows when a window is free. The stack registers its
 * arbiter here; the interface asks it to run each operation and never learns
 * what a timeslot is.
 *
 * With nothing registered the memory is the application's alone and the
 * interface drives the controller directly.
 *
 * @param	pArb	: The arbiter, or NULL to go back to driving directly.
 */
void NvmIntrfSetArbiter(NvmIntrfArb_t pArb);

/**
 * @brief	Read the operation counts.
 *
 * @param	pStat : Filled with the counts since reset
 */
void NvmIntrfGetStat(NvmIntrfStat_t *pStat);

/**
 * @brief	Fill a config for the internal memory.
 *
 * Sets the geometry and the command set the interface understands. The
 * remaining fields are left untouched.
 *
 * @param	Cfg	: Config to fill
 */
void NvmIntrfCfg(NvmCfg_t &Cfg);

/**
 * @brief	Where the memory the application owns ends.
 *
 * On most targets this is the device size. Where a stack image or a reserved
 * partition sits at the top, it is that boundary instead: on the nRF54L the
 * S145 image occupies the top of the RRAM and the application slot ends at
 * the storage partition. C linkage so a target can override the weak default
 * from a C file.
 *
 * @return	First byte past the application owned memory.
 */
extern "C" uint64_t NvmIntrfCeiling(void);

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_INTRF_H__
