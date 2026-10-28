/**-------------------------------------------------------------------------
@file	dma_lpc546xx.h

@brief	LPC546xx DMA controller channels

		The controller has 30 channels. Each channel serves one fixed
		peripheral request, the channel number is the request number:
		Flexcomm 0 to 7 RX and TX on channels 0 to 15, DMIC 16 and 17,
		SPIFI 18, SHA 19, Flexcomm 8 and 9 RX and TX on channels 20 to 23.

		A driver claims the channels of its peripheral, sets them up once
		and starts one transfer at a time. A transfer moves at most
		LPC546XX_DMA_XFER_MAX values of the channel width.

		All channels share the DMA0 interrupt. The interrupt takes the flags
		of a channel and clears them in the controller before it calls the
		channel handler, so that one channel cannot hold the interrupt.

@author	Hoang Nguyen Hoan
@date	Oct. 26, 2026

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
#ifndef __DMA_LPC546XX_H__
#define __DMA_LPC546XX_H__

#include <stdint.h>
#include <stdbool.h>

#include "LPC546xx.h"

/// Number of channels
#define LPC546XX_DMA_NBCH			FSL_FEATURE_DMA_NUMBER_OF_CHANNELS

/// Most values in one transfer, XFERCOUNT + 1
#define LPC546XX_DMA_XFER_MAX		1024U

/// Channel of the RX request of Flexcomm FcNo, RX FIFO not empty or I2C slave
#define LPC546XX_DMA_CH_FC_RX(FcNo)	((FcNo) < 8 ? (FcNo) * 2 : 20 + ((FcNo) - 8) * 2)

/// Channel of the TX request of Flexcomm FcNo, TX FIFO not full or I2C master
#define LPC546XX_DMA_CH_FC_TX(FcNo)	(LPC546XX_DMA_CH_FC_RX(FcNo) + 1)

#define LPC546XX_DMA_CH_DMIC0		16
#define LPC546XX_DMA_CH_DMIC1		17
#define LPC546XX_DMA_CH_SPIFI		18
#define LPC546XX_DMA_CH_SHA			19

/// Channel flags returned by Lpc546xxDmaChFlags
#define LPC546XX_DMA_FLAG_DONE		1U		//!< Transfer complete
#define LPC546XX_DMA_FLAG_ERR		2U		//!< Transfer error, or a transfer that could not start

/**
 * @brief	Function called from the DMA interrupt.
 *
 * The flags of the channel are already taken, Lpc546xxDmaChFlags returns
 * them until the channel is cleared or started again.
 *
 * @param	Ch	: Channel number
 * @param	pCtx: Context given when the channel was taken
 */
typedef void (*Lpc546xxDmaChHandler_t)(int Ch, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

// Start, Stop, StopRemain and Clear mask the interrupts while they change a
// channel, they can be called from thread and interrupt code.

/**
 * @brief	Take a channel.
 *
 * Succeeds when the channel is free, or taken before with the same Handler
 * and pCtx. The controller is turned on with the first channel taken.
 *
 * The DMA0 interrupt priority is the most urgent priority asked by the
 * channels that use the interrupt.
 *
 * @param	Ch		: Channel number
 * @param	Handler	: Function the interrupt calls. Can be NULL
 * @param	pCtx	: Context passed to Handler
 * @param	IntPrio	: Interrupt priority
 * @param	bIrq	: true - interrupt at the end of each transfer and on error.
 * 					  false - the flags are read with Lpc546xxDmaChFlags. An
 * 					  error of such a channel can still enter the DMA0
 * 					  interrupt of the other channels, which takes it into
 * 					  the channel flags without calling Handler.
 *
 * @return	true - The channel is yours
 */
bool Lpc546xxDmaChClaim(int Ch, Lpc546xxDmaChHandler_t Handler, void *pCtx, int IntPrio, bool bIrq);

/**
 * @brief	Stop a channel and give it back. The controller is turned off
 * 			with the last channel.
 */
void Lpc546xxDmaChFree(int Ch);

/**
 * @brief	Set up a channel for its peripheral request.
 *
 * The channel is stopped. The peripheral address does not increment, the
 * memory address does, see Lpc546xxDmaChMemInc.
 *
 * @param	Ch			: Channel number
 * @param	bToPeriph	: true - memory to peripheral, false - peripheral to memory
 * @param	pPeriph		: Peripheral data register
 * @param	WidthLog2	: 0 byte, 1 half word, 2 word
 */
void Lpc546xxDmaChSetup(int Ch, bool bToPeriph, volatile void *pPeriph, int WidthLog2);

/**
 * @brief	Set whether the memory address of a channel increments.
 *
 * Setup sets it. A transfer of one value repeated, such as the dummy data of
 * a receive, switches it off before Start.
 *
 * @param	Ch		: Channel
 * @param	bInc	: true to increment the memory address, false to keep it
 */
void Lpc546xxDmaChMemInc(int Ch, bool bInc);

/**
 * @brief	Start a transfer of Count bytes with the memory at pMem.
 *
 * The channel was set up with Lpc546xxDmaChSetup. Count is a multiple of the
 * width, at most LPC546XX_DMA_XFER_MAX values, and pMem is aligned to the
 * width. A transfer still running on the channel is stopped first.
 *
 * @param	Ch		: Channel
 * @param	pMem	: Memory address, RAM or, from memory to peripheral, flash
 * @param	Count	: Number of bytes
 *
 * @return	false - Count or pMem not valid, LPC546XX_DMA_FLAG_ERR is set
 */
bool Lpc546xxDmaChStart(int Ch, const void *pMem, uint32_t Count);

/**
 * @brief	Stop the channel and clear its flags.
 */
void Lpc546xxDmaChStop(int Ch);

/**
 * @brief	Stop the channel, take the bytes it has still to move, then
 * 			clear its flags.
 *
 * @return	Bytes not moved.
 */
uint32_t Lpc546xxDmaChStopRemain(int Ch);

/**
 * @brief	Bytes the channel has still to move.
 */
uint32_t Lpc546xxDmaChRemain(int Ch);

/**
 * @brief	Channel flags LPC546XX_DMA_FLAG_xxx, those in the controller and
 * 			those taken by the interrupt since the transfer started.
 */
uint32_t Lpc546xxDmaChFlags(int Ch);

/**
 * @brief	Clear the channel flags.
 */
void Lpc546xxDmaChClear(int Ch);

#ifdef __cplusplus
}
#endif

#endif // __DMA_LPC546XX_H__
