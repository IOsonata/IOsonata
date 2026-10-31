/**-------------------------------------------------------------------------
@file	dmac_samd.h

@brief	SAM D11/D21 DMAC channels, private definitions

A channel moves bytes between memory and a peripheral register, one beat per
peripheral trigger (TRIGACT BEAT), one block per start, then disables
itself. The UART, SPI and I2C drivers take their channels at init. The
descriptors and the write-back section are in this module, one per channel:
12 channels on SAM D21, 6 on SAM D11. One DMAC interrupt serves all the
channels, at the most urgent priority asked by a channel with a handler.

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
#ifndef __DMAC_SAMD_H__
#define __DMAC_SAMD_H__

#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"

// Trigger sources of the SERCOM, CHCTRLB.TRIGSRC
#define SAMD_DMAC_TRIG_SERCOM_RX(n)		(SERCOM0_DMAC_ID_RX + 2 * (n))
#define SAMD_DMAC_TRIG_SERCOM_TX(n)		(SERCOM0_DMAC_ID_TX + 2 * (n))

#define SAMD_DMAC_COUNT_MAX				65535	//!< Beats of a block

/**
 * @brief	Channel interrupt callback, from the DMAC interrupt.
 *
 * @param	Chan	: Channel
 * @param	Flags	: DMAC_CHINTFLAG_TCMPL_Msk (block done), DMAC_CHINTFLAG_TERR_Msk
 *					  (bus error, channel disabled)
 * @param	pCtx	: Context given to SamdDmacOpen
 */
typedef void (*SamdDmacHandler_t)(int Chan, uint32_t Flags, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Take a free channel for a peripheral trigger.
 *
 * @param	Trig	: Trigger source, SAMD_DMAC_TRIG_xxx
 * @param	Handler	: Block done and error callback, NULL for none (the user
 *					  checks SamdDmacBusy)
 * @param	pCtx	: Context of the callback
 * @param	IntPrio	: Priority of the DMAC interrupt, with a handler
 *
 * @return	Channel, -1 if no channel is free or the settings are not valid
 */
int SamdDmacOpen(int Trig, SamdDmacHandler_t Handler, void *pCtx, int IntPrio);

/**
 * @brief	Channel stopped and released.
 */
void SamdDmacClose(int Chan);

/**
 * @brief	Trigger source of a stopped channel, for a channel serving both
 *			directions of a peripheral.
 *
 * @param	Chan	: Channel, stopped
 * @param	Trig	: Trigger source, SAMD_DMAC_TRIG_xxx
 */
void SamdDmacTrigger(int Chan, int Trig);

/**
 * @brief	Start a block of byte beats.
 *
 * The channel takes a beat at each trigger of its peripheral, the first at
 * once when the trigger is already set.
 *
 * @param	Chan	: Channel, stopped
 * @param	pSrc	: Source, the first byte
 * @param	bSrcInc	: Source address incremented (memory)
 * @param	pDst	: Destination, the first byte
 * @param	bDstInc	: Destination address incremented (memory)
 * @param	Count	: Bytes, 1 to SAMD_DMAC_COUNT_MAX
 */
void SamdDmacStart(int Chan, const volatile void *pSrc, bool bSrcInc, volatile void *pDst, bool bDstInc,
				   uint32_t Count);

/**
 * @brief	Block running.
 *
 * @return	true until the last beat of the block is done or the channel is
 *			stopped
 */
bool SamdDmacBusy(int Chan);

/**
 * @brief	Stop the channel, the beat in progress ends first.
 *
 * @return	Bytes of the block not moved, 0 for a block done
 */
uint32_t SamdDmacStop(int Chan);

#ifdef __cplusplus
}
#endif

#endif // __DMAC_SAMD_H__
