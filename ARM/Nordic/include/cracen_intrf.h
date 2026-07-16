/**-------------------------------------------------------------------------
@file	cracen_intrf.h

@brief	Device interface to the Nordic on-die crypto core (CRACEN).

		CRACEN is not a crypto engine. It is the access path that the Silex
		engines sit on: the symmetric engine (CryptoMaster), the public-key
		engine (BA414EP / PKE+IKG), and the random generator all reach the die
		through it, and it owns the shared enable, power, clock and microcode
		state. It is therefore modeled as a DeviceIntrf, the same way an SPI or
		I2C controller is the interface a sensor sits on.

		An engine acquires the core by starting a transaction addressed to its
		module and releases it by stopping. The base DeviceIntrf supplies the
		busy flag (mutual exclusion between the engines) and the enable reference
		count (the core is not powered down under an engine still using it), so
		no separate lock is needed.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __CRACEN_INTRF_H__
#define __CRACEN_INTRF_H__

#include <stdint.h>

#include "device_intrf.h"

/// Module address passed as the transaction DevAddr. Selects which CRACEN
/// module is enabled for the duration of the acquired transaction.
typedef enum __Cracen_Module {
	CRACEN_MODULE_CRYPTOMASTER = 0,	//!< Symmetric engine (AES, digest)
	CRACEN_MODULE_PKEIKG,			//!< Public-key engine and key generator
	CRACEN_MODULE_RNG				//!< Random generator
} CRACEN_MODULE;

/// DevAddr base selector, used the way SPI uses a chip-select index. Picks which
/// base a Device Read / Write transfer lands in; the transfer address is the
/// byte offset within it.
#define CRACEN_ADDR_REG		0U		//!< Engine register base
#define CRACEN_ADDR_MEM		1U		//!< Operand memory base

#ifdef __cplusplus

/// @brief	Device interface to the CRACEN crypto core.
///
/// One instance per die. The three crypto engines hold a pointer to it and go
/// through StartTx/StopTx (or StartRx/StopRx) to acquire and release the core.
class CracenIntrf : public DeviceIntrf {
public:
	bool Init(void);

	operator DevIntrf_t * const () { return &vDevIntrf; }

	uint32_t Rate(uint32_t RateHz) override { (void)RateHz; return 0; }
	uint32_t Rate(void) override { return 0; }

	// A crypto transaction has no separate address/data stream: the module is
	// the DevAddr, and acquire/release is the whole point. Rx and Tx are the
	// same acquire/release on this interface.
	bool StartRx(uint32_t Module) override {
		return DeviceIntrfStartRx(&vDevIntrf, Module);
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		(void)pBuff; (void)BuffLen; return 0;
	}
	void StopRx(void) override { DeviceIntrfStopRx(&vDevIntrf); }

	bool StartTx(uint32_t Module) override {
		return DeviceIntrfStartTx(&vDevIntrf, Module);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		(void)pData; (void)DataLen; return 0;
	}
	void StopTx(void) override { DeviceIntrfStopTx(&vDevIntrf); }

	// Direct register and operand access, used inside a held operation
	// (between ModuleHold and ModuleRelease). These do not take the busy flag,
	// because the operation already holds it; they are plain word accesses at
	// the sub-block base plus the Silex IP offset. Registers and operand memory
	// are two sub-blocks selected by the accessor, so the engine passes offsets
	// only and never a base.
	uint32_t RegRead(uint32_t Offset) const;
	void RegWrite(uint32_t Offset, uint32_t Value);
	void MemRead(uint32_t Offset, uint8_t *pDst, size_t Len) const;
	void MemWrite(uint32_t Offset, const uint8_t *pSrc, size_t Len);

	// Hold a crypto module enabled for the duration of an operation and take the
	// busy flag so the engines serialize against one another. The engine calls
	// ModuleHold once before its work and ModuleRelease once after. An engine
	// that reaches registers and operands (the public-key engine) then uses the
	// Device Read / Write path within the held operation; an engine that drives
	// its own hardware or a vendor driver (RNG, symmetric) just holds and
	// releases. Returns false if another engine holds the core.
	bool ModuleHold(uint32_t Module);
	void ModuleRelease(void);

protected:
	DevIntrf_t vDevIntrf;
	volatile uint8_t *vpRegBase;	//!< Engine register sub-block base (per MCU)
	volatile uint8_t *vpMemBase;	//!< Operand memory sub-block base (per MCU)
};

/// @brief	Return the single per-die CRACEN interface instance.
///
/// Constructed and initialized on first use. The crypto engines take this
/// pointer; there is one interface for the whole die.
CracenIntrf *CracenIntrfInstance(void);

#endif // __cplusplus

#endif // __CRACEN_INTRF_H__
