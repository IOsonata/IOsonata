/**-------------------------------------------------------------------------
@file	gpdma_stm32wba.h

@brief	STM32WBA GPDMA1 channel manager

		GPDMA1 has 8 channels, each with its own interrupt vector. A driver
		that uses DMA asks this manager for a channel and gives the function
		the channel interrupt calls. The manager owns the 8 vectors, so the
		UART, I2C and SPI drivers share the channels: the first to ask for a
		channel has it, until it frees it.

		A channel request is the GPDMA1 request number of the peripheral, from
		the reference manual. The functions below set up single byte, half
		word or word transfers between memory and one peripheral data register,
		the transfers the I2C and SPI drivers need.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2026

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
#ifndef __GPDMA_STM32WBA_H__
#define __GPDMA_STM32WBA_H__

#include <stdint.h>
#include <stdbool.h>

#define WBA_GPDMA_NBCH				8

/// GPDMA1 request numbers of the I2C and SPI instances
#define WBA_GPDMA_REQ_SPI1_RX		1
#define WBA_GPDMA_REQ_SPI1_TX		2
#define WBA_GPDMA_REQ_SPI3_RX		3
#define WBA_GPDMA_REQ_SPI3_TX		4
#define WBA_GPDMA_REQ_I2C1_RX		5
#define WBA_GPDMA_REQ_I2C1_TX		6
#define WBA_GPDMA_REQ_I2C3_RX		8
#define WBA_GPDMA_REQ_I2C3_TX		9
#define WBA_GPDMA_REQ_SPI2_RX		52
#define WBA_GPDMA_REQ_SPI2_TX		53
#define WBA_GPDMA_REQ_I2C2_RX		54
#define WBA_GPDMA_REQ_I2C2_TX		55
#define WBA_GPDMA_REQ_I2C4_RX		57
#define WBA_GPDMA_REQ_I2C4_TX		58

/// Channel flags returned by WbaGpdmaFlags
#define WBA_GPDMA_FLAG_TC			1U		//!< Transfer complete
#define WBA_GPDMA_FLAG_ERR			2U		//!< Data transfer, link or user setting error

/**
 * @brief	Function called from the channel interrupt.
 *
 * @param	Ch	: Channel number
 * @param	pCtx: Context given when the channel was taken
 */
typedef void (*WbaGpdmaHandler_t)(int Ch, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Take any free channel.
 *
 * @param	Handler	: Function the channel interrupt calls. Can be NULL
 * @param	pCtx	: Context passed to Handler
 * @param	IntPrio	: Interrupt priority
 * @param	bIrq	: true - Enable the channel interrupt
 *
 * @return	Channel number, -1 when all channels are taken
 */
int WbaGpdmaAlloc(WbaGpdmaHandler_t Handler, void *pCtx, int IntPrio, bool bIrq);

/**
 * @brief	Take a given channel.
 *
 * Succeeds when the channel is free, or taken before with the same Handler
 * and pCtx.
 *
 * @return	true - The channel is yours
 */
bool WbaGpdmaClaim(int Ch, WbaGpdmaHandler_t Handler, void *pCtx, int IntPrio, bool bIrq);

/**
 * @brief	Stop a channel and give it back.
 */
void WbaGpdmaFree(int Ch);

/**
 * @brief	Set up a channel. The channel is stopped, the peripheral drives it.
 *
 * @param	Ch			: Channel number
 * @param	Req			: Request number WBA_GPDMA_REQ_xxx
 * @param	bToPeriph	: true - memory to peripheral, false - peripheral to memory
 * @param	pPeriph		: Peripheral data register
 * @param	WidthLog2	: 0 byte, 1 half word, 2 word
 * @param	bIrq		: true - transfer complete and error interrupts
 */
void WbaGpdmaSetup(int Ch, uint32_t Req, bool bToPeriph, volatile void *pPeriph, int WidthLog2, bool bIrq);

/**
 * @brief	Set whether the memory address of a channel increments.
 *
 * Setup sets it. A transfer of one value repeated, such as the dummy data of
 * a receive, switches it off before Start.
 *
 * @param	Ch		: Channel
 * @param	bInc	: true to increment the memory address, false to keep it
 */
void WbaGpdmaMemInc(int Ch, bool bInc);

/**
 * @brief	Start a transfer of Count bytes with the memory at pMem.
 *
 * The channel was set up with WbaGpdmaSetup. Count is a multiple of the width.
 */
void WbaGpdmaStart(int Ch, const void *pMem, uint32_t Count);

/**
 * @brief	Stop the channel, clear its flags and end the transfer in progress.
 */
void WbaGpdmaStop(int Ch);

/**
 * @brief	Suspend the channel, take the bytes it has still to move, then
 * 			stop it and clear its flags.
 *
 * @return	Bytes not moved.
 */
uint32_t WbaGpdmaStopRemain(int Ch);

/**
 * @brief	Bytes the channel has still to read from its source.
 */
uint32_t WbaGpdmaRemain(int Ch);

/**
 * @brief	Channel flags WBA_GPDMA_FLAG_xxx, those seen by the hardware and
 * 			those taken by WbaGpdmaAck() since the transfer started.
 */
uint32_t WbaGpdmaFlags(int Ch);

/**
 * @brief	Take the channel flags into the latch and clear them in the
 * 			hardware. A handler that does not clear the flags itself calls
 * 			this first, so that the channel interrupt does not fire again.
 * 			WbaGpdmaFlags() keeps reporting what was taken.
 */
void WbaGpdmaAck(int Ch);

/**
 * @brief	Clear the channel flags.
 */
void WbaGpdmaClear(int Ch);

#ifdef __cplusplus
}
#endif

#endif	// __GPDMA_STM32WBA_H__
