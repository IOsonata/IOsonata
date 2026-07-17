/**-------------------------------------------------------------------------
@file	cc3xx_intrf.h

@brief	DeviceIntrf over the Arm CryptoCell CC3xx register file.

		The CC3xx family (CC310, CC312) is Arm IP licensed across vendors; the
		register file layout is Arm's. The vendor port implements the three
		required functions below (base address and wrapper power operations)
		in its own source file, cc3xx_intrf_nrfx.cpp on Nordic targets. A
		function seam instead of a macro lets the vendor decide the base at
		run time, for example secure versus non-secure aliasing on CC312
		parts. This interface gives crypto engines the same
		transfer model as any bus: StartTx / StartRx receive and save the
		DevAddr selector, the address phase latches a register offset, and
		the data phase moves 32-bit words at that offset. The PKA SRAM is a
		port inside the register file (address register plus data registers),
		so engines drive it through ordinary register transfers.

		OpHold / OpRelease give a device-level exclusion for an operation
		spanning several transfers (a whole point multiply). It is distinct
		from the per transfer busy flag the framework manages.

		Enable / Disable follow the interface reference count: the CryptoCell
		wrapper is powered on first use and off on last release.

@author	Hoang Nguyen Hoan
@date	Jul. 17, 2026

@license MIT, (c) 2026 I-SYST. See crypto.h for full text.
----------------------------------------------------------------------------*/
#ifndef __CC3XX_INTRF_H__
#define __CC3XX_INTRF_H__

#include <stdint.h>

#include "device_intrf.h"

/// DevAddr base selector. The CC3xx has a single register file; the selector
/// exists for the transfer protocol and future sub-block splits.
#define CC3XX_ADDR_REG		0U		//!< Register file base

/// PKA SRAM bytes. The CC310 has 4 KB; override for family members that differ.
#ifndef CC3XX_PKA_SRAM_SIZE
#define CC3XX_PKA_SRAM_SIZE	0x1000UL
#endif

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Required vendor surface, implemented per target (cc3xx_intrf_nrfx.cpp on
// Nordic). The interface calls these; nothing else in the CC3xx code touches
// vendor registers.
//-----------------------------------------------------------------------------

/// Base address of the CC3xx register file for this target.
uintptr_t Cc3xxBase(void);

/// Power the CC3xx wrapper on. Returns false when the block does not come up.
bool Cc3xxEnable(void);

/// Power the CC3xx wrapper off.
void Cc3xxDisable(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Cc3xxIntrf : public DeviceIntrf {
public:
	bool Init(void);

	/// Hold the CC3xx for an operation spanning several transfers. Fails when
	/// another engine holds it; the caller retries or reports busy. Interrupt
	/// context gets a single attempt.
	bool OpHold(void);
	void OpRelease(void);

	operator DevIntrf_t * const () override { return &vDevIntrf; }
	uint32_t Rate(uint32_t DataRate) override { (void)DataRate; return 0; }
	uint32_t Rate(void) override { return 0; }
	bool StartRx(uint32_t DevAddr) override {
		return DeviceIntrfStartRx(&vDevIntrf, DevAddr);
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vDevIntrf, pBuff, BuffLen);
	}
	void StopRx(void) override { DeviceIntrfStopRx(&vDevIntrf); }
	bool StartTx(uint32_t DevAddr) override {
		return DeviceIntrfStartTx(&vDevIntrf, DevAddr);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vDevIntrf, pData, DataLen);
	}
	void StopTx(void) override { DeviceIntrfStopTx(&vDevIntrf); }

private:
	DevIntrf_t vDevIntrf;
};

/// Interface singleton for the standard construction path.
Cc3xxIntrf *Cc3xxIntrfInstance(void);

#endif // __cplusplus

#endif // __CC3XX_INTRF_H__
