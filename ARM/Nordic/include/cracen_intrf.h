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
/// sub-block a Read / Write transfer lands in; the transfer address (pAdCmd) is
/// the byte offset within it.
#define CRACEN_ADDR_REG		0U		//!< Held module register base
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

	// Transfer follows the SPI and I2C pattern: the inherited Device Read /
	// Write drive StartTx / TxSrData / TxData / StartRx / RxData / StopTx below.
	// StartTx and StartRx save the DevAddr (which sub-block base the access lands
	// in, the way SPI saves the chip-select); the address phase latches the byte
	// offset; TxData and RxData transfer at the saved base plus the offset.
	// Registers are word accessed and operand memory byte accessed. The engine
	// reaches all of this through the inherited Device Read / Write, exactly as a
	// sensor reaches its bus.
	bool StartRx(uint32_t DevAddr) override {
		return DeviceIntrfStartRx(&vDevIntrf, DevAddr);
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return vDevIntrf.RxData(&vDevIntrf, pBuff, BuffLen);
	}
	void StopRx(void) override { DeviceIntrfStopRx(&vDevIntrf); }
	bool StartTx(uint32_t DevAddr) override {
		return DeviceIntrfStartTx(&vDevIntrf, DevAddr);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		return vDevIntrf.TxData(&vDevIntrf, pData, DataLen);
	}
	void StopTx(void) override { DeviceIntrfStopTx(&vDevIntrf); }

	// Hold a crypto module enabled for the duration of an operation and take the
	// busy flag so the engines serialize against one another. The engine calls
	// ModuleHold once before its work and ModuleRelease once after; register
	// access between them lands in the held module's sub-block. Returns false if
	// another engine holds the core.
	bool ModuleHold(uint32_t Module);
	void ModuleRelease(void);

protected:
	DevIntrf_t vDevIntrf;
};

/// @brief	Return the single per-die CRACEN interface instance.
///
/// Constructed and initialized on first use. The crypto engines take this
/// pointer; there is one interface for the whole die.
CracenIntrf *CracenIntrfInstance(void);

#endif // __cplusplus

#endif // __CRACEN_INTRF_H__
