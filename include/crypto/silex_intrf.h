/**-------------------------------------------------------------------------
@file	silex_intrf.h

@brief	Generic access path to the Silex crypto engines.

		The Silex engines (the CryptoMaster symmetric engine, the BA414EP
		public-key engine, the random generator) are licensed IP blocks that
		appear in several vendors' silicon behind different wrappers. This is
		the vendor-neutral interface the generic engine drivers talk to: a
		DeviceIntrf for register and operand-memory transfers, plus the
		operation hold that serializes the engines sharing one core.

		The implementation owns everything vendor specific behind this
		surface: power, clocking, exclusion primitives, and any preparation
		the wrapper requires before an engine can run. On the Nordic CRACEN
		wrapper that preparation includes loading the BA414EP microcode; a
		standalone memory-mapped Silex core on another MCU may need none of
		it. The engine drivers never see the difference.

		CoreAcquire takes the module for the coming operation and returns
		with the module powered and ready. CoreRelease returns the core.
		CoreReset forces the held module through a hard reset for timeout
		recovery while the caller keeps the hold.

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
#ifndef __SILEX_INTRF_H__
#define __SILEX_INTRF_H__

#include <stdint.h>

#include "device_intrf.h"

/// Module id passed to CoreAcquire. Selects which Silex engine the coming
/// operation runs on.
#define SILEX_MODULE_CRYPTOMASTER	0U	//!< Symmetric engine (AES, digest)
#define SILEX_MODULE_PKE			1U	//!< BA414EP public-key engine
#define SILEX_MODULE_RNG			2U	//!< Random generator

/// DevAddr base selector, used the way SPI uses a chip-select index. Picks
/// which sub-block of the held module a Read / Write transfer lands in; the
/// transfer address (pAdCmd) is the byte offset within it. Registers are word
/// accessed and operand memory byte accessed.
#define SILEX_ADDR_REG		0U		//!< Held module register base
#define SILEX_ADDR_MEM		1U		//!< Operand memory base

#ifdef __cplusplus

/// @brief	Vendor-neutral interface to a shared Silex crypto core.
///
/// The engine drivers hold a pointer to this and go through the inherited
/// Device Read / Write for every register and operand access, bracketed by
/// CoreAcquire / CoreRelease around each multi-transfer operation.
class SilexIntrf : public DeviceIntrf {
public:
	/// Acquire the core for an operation on the given module. Returns with
	/// the module powered and ready to accept commands, or false when
	/// another owner holds the core. One owner at a time; not recursive.
	virtual bool CoreAcquire(uint32_t Module, const void *pOwner) = 0;

	/// Release the hold owned by pOwner. A stray release with the wrong
	/// owner fails without touching the hold.
	virtual bool CoreRelease(const void *pOwner) = 0;

	/// Force the held module through a hard reset while pOwner keeps the
	/// hold. Timeout recovery: no other engine can enter in between.
	virtual bool CoreReset(const void *pOwner) = 0;
};

#endif // __cplusplus

#endif // __SILEX_INTRF_H__
