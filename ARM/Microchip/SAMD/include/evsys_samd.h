/**-------------------------------------------------------------------------
@file	evsys_samd.h

@brief	SAM D11/D21 event system channels, private definitions

A channel connects one event generator to at most one user here. The
synchronous and resynchronized paths run on the channel generic clock, on
generator 0, and have the event detected interrupt. The asynchronous path
has no clock and no interrupt.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#ifndef __EVSYS_SAMD_H__
#define __EVSYS_SAMD_H__

#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"
#include "coredev/iopincfg.h"

// CHANNEL.PATH
#define SAMD_EVSYS_PATH_SYNC		0
#define SAMD_EVSYS_PATH_RESYNC		1
#define SAMD_EVSYS_PATH_ASYNC		2

// CHANNEL.EDGSEL, synchronous and resynchronized paths
#define SAMD_EVSYS_EDGE_NONE		0
#define SAMD_EVSYS_EDGE_RISE		1
#define SAMD_EVSYS_EDGE_FALL		2
#define SAMD_EVSYS_EDGE_BOTH		3

// Generators and users, from the device description
#define SAMD_EVSYS_GEN_RTC_PER(n)	(4 + (n))
#define SAMD_EVSYS_GEN_EIC_EXTINT(n)	(12 + (n))

#if defined(TC3_REGS)
#define SAMD_EVSYS_USER_TCC0_MC0	6
#define SAMD_EVSYS_USER_TCC1_MC0	12
#define SAMD_EVSYS_USER_TCC2_MC0	16
#define SAMD_EVSYS_USER_TC3_EVU		18		// TC4 to TC7 follow
#else
#define SAMD_EVSYS_USER_TCC0_MC0	6
#define SAMD_EVSYS_USER_TC1_EVU		10		// TC2 follows
#endif

#define SAMD_EVSYS_USER_NONE		(-1)

/**
 * @brief	Event detected callback, from the EVSYS interrupt.
 *
 * @param	Chan	: Channel
 * @param	pCtx	: Context given to SamdEvsysOpen
 */
typedef void (*SamdEvsysHandler_t)(int Chan, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Channel from a generator to a user.
 *
 * @param	Gen		: Generator
 * @param	Path	: SAMD_EVSYS_PATH_xxx
 * @param	Edge	: SAMD_EVSYS_EDGE_xxx, synchronous and resynchronized paths
 * @param	User	: User, SAMD_EVSYS_USER_NONE for none
 * @param	Handler	: Event detected callback, synchronous and resynchronized paths, NULL for none
 * @param	pCtx	: Context of the callback
 * @param	IntPrio	: Priority of the EVSYS interrupt
 *
 * @return	Channel, -1 if no channel is free or the settings are not valid
 */
int SamdEvsysOpen(int Gen, int Path, int Edge, int User, SamdEvsysHandler_t Handler, void *pCtx, int IntPrio);

/**
 * @brief	Channel and user released.
 */
void SamdEvsysClose(int Chan, int User);

/**
 * @brief	EIC event output of an EXTINT line, for an event channel.
 *
 * The application sets the pin of the line to the EIC, function A
 * (IOPINOP_FUNC0), as an input. A line with a pin interrupt is refused, and a
 * pin interrupt is refused on a line with the event output on.
 *
 * @param	ExtInt	: EXTINT line
 * @param	Sense	: Edge of the event, IOPINSENSE_DISABLE turns it off
 *
 * @return	true on success
 */
bool SamdEicEvent(int ExtInt, IOPINSENSE Sense);

#ifdef __cplusplus
}
#endif

#endif // __EVSYS_SAMD_H__
