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
typedef uint32_t PeiEvent_t;	//!< Logical event identity within a Pei matrix

typedef struct {
	int DevNo;				//!< Hardware interconnect instance, zero on single-matrix MCUs
	PeiEndpoint_t Event;	//!< Event source identifier
	PeiEndpoint_t Task;	//!< Task destination identifier
	uint32_t Options;		//!< Zero: target default; other values are target-defined
} PeiCfg_t;

// Lower-level hardware connection primitive, used by the C matrix.

// Caller-owned connection state. Zero-initialize before first use.
// Keep its address stable until disconnect; do not copy active connections.
// Calls operating on one connection must be serialized by the caller.
// Handle is opaque and is not a physical PPI/DPPI/EVSYS channel number.
typedef struct {
	uint32_t Handle;
} PeiConn_t;

// Fixed-size, caller-owned PEI matrix storage. C and C++ clients use the
// same matrix implementation. No heap allocation or MCU-specific routing
// registers are involved here.
#ifndef PEI_MAX_EVENTS
#define PEI_MAX_EVENTS 16
#endif
#ifndef PEI_MAX_CONNECTIONS
#define PEI_MAX_CONNECTIONS 32
#endif
#if PEI_MAX_EVENTS < 1 || PEI_MAX_EVENTS > 63 || PEI_MAX_CONNECTIONS < 1
#error Invalid PEI matrix capacity
#endif

typedef struct {
	PeiEvent_t Token;
	PeiEndpoint_t Endpoint;
	bool Enabled;
} PeiEventState_t;

typedef struct {
	PeiEvent_t Token;
	PeiEndpoint_t Task;
	PeiConn_t Conn;
} PeiRouteState_t;

typedef struct __Pei_Device {
	int DevNo;
	uint32_t Generation;
	PeiEventState_t Events[PEI_MAX_EVENTS];
	PeiRouteState_t Routes[PEI_MAX_CONNECTIONS];
} PeiDev_t;

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

// C matrix API. Initialize before first use; never copy or move a live
// device because hardware connection ownership depends on stable addresses.
bool PeiInit(PeiDev_t *pPei, int DevNo);
bool PeiDeinit(PeiDev_t *pPei);
bool PeiBindEvent(PeiDev_t *pPei, PeiEvent_t *pEvt, PeiEndpoint_t Endpoint);
bool PeiUnbindEvent(PeiDev_t *pPei, PeiEvent_t Evt, PeiEndpoint_t Endpoint);
bool PeiConnectTask(PeiDev_t *pPei, PeiEvent_t Evt, PeiEndpoint_t Task);
bool PeiDisconnectTask(PeiDev_t *pPei, PeiEvent_t Evt, PeiEndpoint_t Task);
bool PeiEnableEvent(PeiDev_t *pPei, PeiEvent_t Evt);
bool PeiDisableEvent(PeiDev_t *pPei, PeiEvent_t Evt);
bool PeiReleaseEvent(PeiDev_t *pPei, PeiEvent_t *pEvt);
PeiEndpoint_t PeiGetEventEndpoint(const PeiDev_t *pPei, PeiEvent_t Evt);

#ifdef __cplusplus
}

// Convenience wrapper for the same C matrix device.
#include "pei_matrix.h"

#endif // __cplusplus

#endif // __PEI_H__
