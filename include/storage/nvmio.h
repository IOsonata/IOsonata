/**--------------------------------------------------------------------------
@file	nvmio.h

@brief	Non volatile memory input output device base class.

One API for any non volatile memory, independent of medium and transport.
Serial NOR and QSPI flash, I2C EEPROM, MCU internal flash and NVM, and a
remote medium reached over a link all present the same NvmIO verbs. A
different device is a different configuration, not a new subclass hierarchy;
a concrete driver implements the verbs for its family.

Design points:

  1. One uniform API. The caller uses Read, Write, Erase, SetWriteProtect and
	 the geometry queries. It never deals with the medium type or the bus.

  2. Transport is injected and transport agnostic. A driver holds a
	 DeviceIntrf, which only moves bytes. It may be a local bus (SPI, QSPI,
	 I2C) or a remote link (UART, USB, BLE). A filesystem on an NvmIO whose
	 transport is a bridge to a PC cannot tell it from local flash.

  3. Geometry comes from the device or the config, never the transport. The
	 user sets geometry in the config because the medium is known, or the
	 driver queries the device (a JEDEC id, or a remote emulation report). The
	 transport does not know or supply geometry.

  4. Uniform verbs, per silicon implementations. Read, Write, Erase, Sync and
	 write protect look the same to the caller. Write protect exists on all
	 families but works differently underneath: an EEPROM drives a pin, serial
	 flash sets status register block protect bits, an MCU sets the controller
	 region protect.

  5. Sync or async is transparent, selected by the config bIntEn flag as
	 elsewhere in IOsonata. bIntEn false: polling, the call blocks until the
	 operation is committed and may call the wait callback during a long wait.
	 bIntEn true: interrupt driven, the call starts the operation and returns,
	 completion arrives through the event handler. The return value is the
	 same in both cases and the caller uses the same verbs. There are no
	 separate async methods.

  6. Contention is contained in the driver. MCU internal program and erase
	 contend with the running code and, on a wireless part, the radio
	 timeslot. The driver handles that and does not leak an event model to the
	 caller.

@author	Hoang Nguyen Hoan
@date	Jul. 20, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#ifndef __NVMIO_H__
#define __NVMIO_H__

#include <stdint.h>
#include <errno.h>

#include "device.h"
#include "device_intrf.h"
#include "coredev/iopincfg.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

class NvmIO;

/// Event notification for interrupt driven operation.
typedef enum __Nvm_Evt {
	NVMIO_EVT_UNKNOWN,
	NVMIO_EVT_WRITE_DONE,		//!< An interrupt driven Write completed
	NVMIO_EVT_ERASE_DONE,		//!< An interrupt driven Erase completed
	NVMIO_EVT_READ_DONE,		//!< An interrupt driven Read completed
	NVMIO_EVT_ERROR				//!< The pending operation failed
} NVMIO_EVT;

/// Completion handler for interrupt driven operation. Res is 0 on success or a
/// negative errno on failure. Off and Len identify the completed request.
typedef void (*NvmEvtHandler_t)(NvmIO * const pDev, NVMIO_EVT Evt,
								uint64_t Off, uint32_t Len, int Res);

/// Cooperative wait callback, called during a long blocking operation when
/// polling so the caller can do other work. Returning false requests the
/// driver to abort the wait where the medium allows it.
typedef bool (*NvmWaitCb_t)(NvmIO * const pDev);

/// Custom device init callback for chip or medium specific setup such as
/// enabling quad mode or writing configuration registers. Returns true on
/// success. Runs at the end of Init with the transport available.
typedef bool (*NvmInitCb_t)(NvmIO * const pDev, DeviceIntrf * const pIntrf);

/// A command plus dummy cycle pair for media that need a command table (serial
/// QSPI read and write). Ignored by media that do not.
typedef struct __Nvm_Cmd {
	uint8_t	Cmd;			//!< Command byte
	uint8_t	DummyCycle;		//!< Dummy cycles after the address phase
} NvmCmd_t;

/// Consolidated configuration. A driver uses the fields that apply to its
/// family and ignores the rest.
typedef struct __Nvm_Cfg {
	// Identity and placement on the transport.
	int			DevNo;			//!< Device index, or I2C address for EEPROM
	uint64_t	BaseAddr;		//!< Region base. Mapped or physical base for
								//!< MCU memory, region start for a serial part.
	// Geometry. Set by the user, or filled by the driver from a device query
	// (a JEDEC id, or a remote emulation report). Never from the transport.
	uint64_t	TotalSize;		//!< Total usable region size in bytes
	uint32_t	EraseSize;		//!< Physical erase unit. 0 : no erase.
	uint32_t	SectorSize;		//!< Logical sector. 0 : equals EraseSize.
	uint32_t	PageSize;		//!< Address auto increment window. 0 : none.
	uint32_t	WriteGran;		//!< Minimum aligned write unit. 0 : treat as 1.
	uint8_t		AddrSize;		//!< Address bytes on the wire (1..4). 0 : mapped
	// Device probe (serial flash).
	uint32_t	DevId;			//!< Expected JEDEC id, 0 to skip the check
	uint8_t		DevIdSize;		//!< Id length in bytes
	// Command table (serial QSPI). Unused by other media.
	NvmCmd_t	RdCmd;			//!< Read command and dummy cycles
	NvmCmd_t	WrCmd;			//!< Program command and dummy cycles
	// Write protect. Meaning is per family: EEPROM drives WrProtPin, serial
	// flash uses status register block protect, MCU uses controller region
	// protect.
	IOPinCfg_t	WrProtPin;		//!< Write protect pin, if used
	// Timing.
	uint32_t	WriteDelayUs;	//!< Post write settle time (EEPROM). 0 : unused
	// Operation mode and callbacks.
	bool			bIntEn;			//!< true : interrupt driven, completion via
									//!< EvtHandler. false : polling, calls block.
	NvmEvtHandler_t	EvtHandler;		//!< Completion handler, used when bIntEn
	NvmWaitCb_t		pWaitCB;		//!< Cooperative wait, used when polling
	NvmInitCb_t		pInitCB;		//!< Custom device init, may be NULL
} NvmCfg_t;

/// @brief	Non volatile memory device base class.
///
/// Abstract base for persistent memory devices. A subclass implements the
/// medium and transport specific access and fills the region window with
/// Region() during its Init. The verbs below are the same to every caller.
class NvmIO : virtual public Device {
public:
	// Keep the Device register access overloads visible alongside the byte
	// addressed overloads declared here.
	using Device::Read;
	using Device::Write;

	/**
	 * @brief	Get the region size in bytes.
	 *
	 * @return	Region size.
	 */
	virtual uint64_t Size(void) const { return vRegionSize; }

	/**
	 * @brief	Get the physical erase unit in bytes.
	 *
	 * @return	Erase size. 0 : the medium is directly rewritable and has no
	 *			erase step (EEPROM, RAM, FRAM).
	 */
	virtual uint32_t EraseSize(void) const = 0;

	/**
	 * @brief	Get the minimum aligned write unit in bytes.
	 *
	 * 1 for a byte writable medium. Larger for an ECC protected line that
	 * cannot be reprogrammed without an erase (some MCU flash).
	 *
	 * @return	Write granularity in bytes.
	 */
	virtual uint32_t WriteGran(void) const = 0;

	/**
	 * @brief	Get the logical sector size in bytes.
	 *
	 * A log structured consumer partitions the region into sectors. On erase
	 * write media the logical sector is the physical erase unit, so this
	 * defaults to EraseSize(). A rewritable medium whose physical unit is
	 * smaller than the sector a consumer wants reports the larger sector here
	 * while EraseSize() stays the physical unit. The logical sector must be a
	 * whole multiple of EraseSize().
	 *
	 * @return	Logical sector size in bytes.
	 */
	virtual uint32_t LogicalSectorSize(void) const { return EraseSize(); }

	/**
	 * @brief	Get the page size in bytes.
	 *
	 * The page is the address auto increment window of the medium. During a
	 * write burst the address counter advances only within the current page
	 * and wraps at the page boundary rather than moving to the next page, so a
	 * single write must not cross a page boundary. Reads and writes may be any
	 * size; the driver splits a burst at page boundaries and reissues the
	 * address per page. EEPROM and NOR flash have a page; byte addressable and
	 * memory mapped media report 0.
	 *
	 * @return	Page size in bytes. 0 : no page boundary.
	 */
	virtual uint32_t PageSize(void) const { return 0; }

	/**
	 * @brief	Read data from the region.
	 *
	 * @param	Off		: Region relative offset
	 * @param	pBuf	: Buffer to receive the data
	 * @param	Len		: Number of bytes to read
	 *
	 * @return	Len on success, negative errno on failure. When an EvtHandler
	 *			is configured the driver also notifies completion through it.
	 *			Whether the call blocks or completes on interrupt is internal.
	 */
	virtual int Read(uint64_t Off, void *pBuf, uint32_t Len) = 0;

	/**
	 * @brief	Write data to the region.
	 *
	 * On an erase write medium the destination must be erased and Off and Len
	 * multiples of WriteGran. On a direct write medium data is overwritten.
	 *
	 * @param	Off		: Region relative offset
	 * @param	pData	: Data to write
	 * @param	Len		: Number of bytes to write
	 *
	 * @return	Len on success, negative errno on failure. When an EvtHandler
	 *			is configured the driver also notifies completion through it.
	 *			Whether the call blocks or completes on interrupt is internal.
	 */
	virtual int Write(uint64_t Off, const void *pData, uint32_t Len) = 0;

	/**
	 * @brief	Erase a range. Off and Len are multiples of EraseSize. A no-op
	 *			success on a direct write medium.
	 *
	 * @param	Off		: Region relative offset, a multiple of EraseSize
	 * @param	Len		: Number of bytes, a multiple of EraseSize
	 *
	 * @return	0 on success, negative errno on failure. When an EvtHandler is
	 *			configured the driver also notifies completion through it.
	 *			Whether the call blocks or completes on interrupt is internal.
	 */
	virtual int Erase(uint64_t Off, uint32_t Len) {
		if (!RangeValid(Off, Len)) { return -EINVAL; }
		return EraseSize() == 0 ? 0 : -ENOTSUP;
	}

	/**
	 * @brief	Set or clear write protection over a range.
	 *
	 * Uniform verb across all families. The mechanism is per medium: EEPROM
	 * drives the write protect pin, serial flash sets status register block
	 * protect bits, MCU sets the controller region protect. A driver that
	 * only protects the whole device applies the request device wide and may
	 * ignore Off and Len.
	 *
	 * @param	Off		: Region relative offset of the protected range
	 * @param	Len		: Length of the range, or the whole region
	 * @param	bEnable	: true to protect, false to unprotect
	 *
	 * @return	0 on success, negative errno on failure, -ENOTSUP if the medium
	 *			has no write protect.
	 */
	virtual int SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable) {
		(void)Off; (void)Len; (void)bEnable;
		return -ENOTSUP;
	}

	/**
	 * @brief	Report whether an interrupt driven operation is in progress.
	 *
	 * @return	true when a pending operation has not completed. A medium that
	 *			is never busy inherits false.
	 */
	virtual bool IsBusy(void) const { return false; }

	/**
	 * @brief	Flush buffered state, or drain a pending operation.
	 *
	 * @return	0 on success, negative errno on failure.
	 */
	virtual int Sync(void) { return 0; }

	/**
	 * @brief	Get the region offset on the physical medium.
	 *
	 * @return	Absolute offset of the region in bytes.
	 */
	uint64_t RegionOffset(void) const { return vRegionOffset; }

protected:
	NvmIO() : vRegionOffset(0), vRegionSize(0), vbIntEn(false),
			  vEvtHandler(nullptr), vpWaitCB(nullptr) {}

	/**
	 * @brief	Set the region window. A subclass Init calls this once geometry
	 *			is resolved.
	 */
	void Region(uint64_t Offset, uint64_t Size) {
		vRegionOffset = Offset;
		vRegionSize = Size;
	}

	/**
	 * @brief	Store the operation mode and callbacks. A subclass Init calls
	 *			this so the base holds the shared lifecycle state.
	 */
	void SetMode(bool bIntEn, NvmEvtHandler_t EvtHandler, NvmWaitCb_t pWaitCB) {
		vbIntEn = bIntEn;
		vEvtHandler = EvtHandler;
		vpWaitCB = pWaitCB;
	}

	/**
	 * @brief	Report completion of an interrupt driven operation.
	 */
	void NotifyDone(NVMIO_EVT Evt, uint64_t Off, uint32_t Len, int Res) {
		if (vEvtHandler != nullptr) {
			vEvtHandler(this, Evt, Off, Len, Res);
		}
	}

	/**
	 * @brief	Invoke the cooperative wait during a long polling operation.
	 *
	 * @return	false when the caller asked to abort.
	 */
	bool WaitPoll(void) {
		return vpWaitCB != nullptr ? vpWaitCB(this) : true;
	}

	/**
	 * @brief	True when the device is configured interrupt driven.
	 */
	bool IntEn(void) const { return vbIntEn; }

	/**
	 * @brief	Validate a region relative range.
	 */
	bool RangeValid(uint64_t Off, uint32_t Len) const {
		return Off <= vRegionSize && Len <= vRegionSize - Off;
	}

private:
	uint64_t		vRegionOffset;	//!< Absolute region offset on the medium
	uint64_t		vRegionSize;	//!< Region size in bytes
	bool			vbIntEn;		//!< Interrupt driven when true, else polling
	NvmEvtHandler_t	vEvtHandler;	//!< Completion handler for interrupt mode
	NvmWaitCb_t		vpWaitCB;		//!< Cooperative wait for polling mode
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVMIO_H__
