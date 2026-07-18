/**-------------------------------------------------------------------------
@file	cracen_intrf.h

@brief	Device interface to the Nordic on-die crypto core (CRACEN).

		CRACEN is not a crypto engine. It is the access path that the Silex
		engines sit on: the symmetric engine (CryptoMaster), the public-key
		engine (BA414EP / PKE+IKG), and the random generator all reach the die
		through it, and it owns the shared enable, power, clock and microcode
		state. It is therefore modeled as a DeviceIntrf, the same way an SPI or
		I2C controller is the interface a sensor sits on.

		Every transfer is self-addressing through the DevAddr module selector,
		so the interface keeps no cross-operation state. The base DeviceIntrf
		supplies the per-transfer busy flag and the enable reference count
		(the core is not powered down under an engine still using it); the
		per-operation lock belongs to each engine device.

@author	Hoang Nguyen Hoan
@date	Jul 2026

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
#ifndef __CRACEN_INTRF_H__
#define __CRACEN_INTRF_H__

#include <stdint.h>

#include "device_intrf.h"

/// DevAddr base selectors are the engines' own (BA414EP 0 and 1,
/// CryptoMaster 2). CRACEN_ADDR_RNG is a Nordic addition: an Rx transfer on
/// it fills the buffer with entropy (no address phase).
#define CRACEN_ADDR_RNG		3U				//!< Random generator entropy read

#ifdef __cplusplus

/// @brief	Device interface to the CRACEN crypto core.
///
/// One instance per die. The three crypto engines hold a pointer to it and go
/// through StartTx/StopTx (or StartRx/StopRx) to acquire and release the core.
class CracenIntrf : public DeviceIntrf {
public:
	bool Init(void);

	operator DevIntrf_t * const () override { return &vDevIntrf; }

	uint32_t Rate(uint32_t RateHz) override { (void)RateHz; return 0; }
	uint32_t Rate(void) override { return 0; }

	// The transfer methods are the DeviceIntrf defaults. StartTx and StartRx
	// save the DevAddr (which sub-block base the access lands in, the way SPI
	// saves the chip-select); the address phase latches the byte offset;
	// TxData and RxData transfer at the saved base plus the offset. Registers
	// are word accessed and operand memory byte accessed. The engine reaches
	// all of this through the inherited Device Read / Write, exactly as a
	// sensor reaches its bus.

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
