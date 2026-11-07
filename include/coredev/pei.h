/**-------------------------------------------------------------------------
@file	pei.h

@brief	Peripheral Event Interconnect hardware connection matrix

PEI connects hardware event sources to hardware task destinations.
No data moves through PEI. No CPU interrupt is used to forward events.
The target implementation allocates the required hardware routing resources.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef __PEI_H__
#define __PEI_H__

#include <stdbool.h>
#include <stdint.h>

// Encoded by the MCU port: register address, generator/user number, or another
// target-specific endpoint identifier. Source and destination are distinct roles.
typedef uintptr_t PeiEndpoint_t;

typedef struct {
	int DevNo;				//!< Hardware interconnect instance, zero on single-matrix MCUs
	PeiEndpoint_t Event;	//!< Event source identifier
	PeiEndpoint_t Task;	//!< Task destination identifier
	uint32_t Options;		//!< Zero: target default; other values are target-defined
} PeiCfg_t;

// Caller-owned connection state. Zero-initialize before first use.
// Handle is opaque and is not a physical PPI/DPPI/EVSYS channel number.
typedef struct {
	uint32_t Handle;
} PeiConn_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Connect an event producer to a task consumer.
 *
 * A successful connection is initially disabled. An existing connection
 * must first be disconnected. Invalid or unsupported routing, conflicting
 * resources, and exhausted hardware resources all return false.
 *
 * @param	pConn	: Caller-owned, zero-initialized connection state
 * @param	pCfg	: Event source, task destination and matrix instance
 * @return	true on success
 */
bool PeiConnect(PeiConn_t * const pConn, const PeiCfg_t * const pCfg);

/** @brief Enable a configured hardware connection. Repeated calls are safe. */
bool PeiEnable(PeiConn_t * const pConn);

/** @brief Disable a configured hardware connection. Repeated calls are safe. */
bool PeiDisable(PeiConn_t * const pConn);

/** @brief Disable, disconnect and release a connection. Clears pConn->Handle. */
void PeiDisconnect(PeiConn_t * const pConn);

#ifdef __cplusplus
}

// A lightweight owner of one logical connection in the interconnect matrix.
class Pei {
public:
	Pei() : vConn{0} {}
	~Pei() { Disconnect(); }

	Pei(const Pei &) = delete;
	Pei &operator=(const Pei &) = delete;

	bool Connect(const PeiCfg_t &Cfg) { return PeiConnect(&vConn, &Cfg); }
	bool Enable() { return PeiEnable(&vConn); }
	bool Disable() { return PeiDisable(&vConn); }
	void Disconnect() { PeiDisconnect(&vConn); }

private:
	PeiConn_t vConn;
};

#endif // __cplusplus

#endif // __PEI_H__
