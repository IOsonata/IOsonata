/**-------------------------------------------------------------------------
@file	dma_stm32.h

@brief	STM32 DMA channel manager, DMA controllers made of channels

		For the DMA controllers made of channels with a group of 4 flags
		each in ISR and IFCR. The request of a channel is selected in CSELR
		(L4) or routed by DMAMUX1 (L4+). Only the L4 and L4+ families use the
		manager for now; the F0 and G4 UART driver keeps its own channel
		vectors, the stream DMA of the F2, F4 and F7 and the GPDMA of the WBA
		(gpdma_stm32wba.h) are separate.

		A driver that uses DMA asks this manager for the channel the
		reference manual gives its request and passes the function the
		channel interrupt calls. The manager owns the channel interrupt
		vectors, so the UART, I2C and SPI drivers share the channels: the
		first to ask for a channel has it, until it frees it.

		Channel numbers are 0 based: STM32_DMA_CH(1, 1) is DMA1 channel 1,
		STM32_DMA_CH(2, 1) is DMA2 channel 1. The functions below set up
		single byte, half word or word transfers between memory and one
		peripheral data register, as the WBA GPDMA manager does.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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
#ifndef __DMA_STM32_H__
#define __DMA_STM32_H__

#include <stdint.h>
#include <stdbool.h>

#include "stm32.h"

#if defined(IOSONATA_STM32_L4)
// DMA1 and DMA2, 7 channels each
#define STM32_DMA_CTRL_NBCH			7
#define STM32_DMA_NBCTRL			2
#else
#error "dma_stm32.h: the DMA channel manager is for the L4 family"
#endif

#define STM32_DMA_NBCH				(STM32_DMA_CTRL_NBCH * STM32_DMA_NBCTRL)

/// Channel number of channel ChNo (1 based) of controller Ctrl (1 or 2)
#define STM32_DMA_CH(Ctrl, ChNo)	(((Ctrl) - 1) * STM32_DMA_CTRL_NBCH + (ChNo) - 1)

/// Request of a channel: the CSELR value on the L4, the DMAMUX1 request on
/// the L4+ (the L4R and L4S numbers; the L4P and L4Q number the UART, SPI
/// and I2C requests one higher)
#if defined(DMAMUX1) && (defined(STM32L4P5xx) || defined(STM32L4Q5xx))
#define STM32_DMA_REQ(Csel, Mux)	((Mux) + 1)
#elif defined(DMAMUX1)
#define STM32_DMA_REQ(Csel, Mux)	(Mux)
#else
#define STM32_DMA_REQ(Csel, Mux)	(Csel)
#endif

/// Channel flags returned by Stm32DmaChFlags
#define STM32_DMA_FLAG_TC			1U		//!< Transfer complete
#define STM32_DMA_FLAG_ERR			2U		//!< Transfer error

/**
 * @brief	Function called from the channel interrupt.
 *
 * @param	Ch	: Channel number
 * @param	pCtx: Context given when the channel was taken
 */
typedef void (*Stm32DmaChHandler_t)(int Ch, void *pCtx);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Take a given channel.
 *
 * Succeeds when the channel is free, or taken before with the same Handler
 * and pCtx. The controller clock is turned on.
 *
 * @param	Ch		: Channel number, STM32_DMA_CH
 * @param	Handler	: Function the channel interrupt calls. Can be NULL
 * @param	pCtx	: Context passed to Handler
 * @param	IntPrio	: Interrupt priority
 * @param	bIrq	: true - Enable the channel interrupt
 *
 * @return	true - The channel is yours
 */
bool Stm32DmaChClaim(int Ch, Stm32DmaChHandler_t Handler, void *pCtx, int IntPrio, bool bIrq);

/**
 * @brief	Stop a channel and give it back.
 */
void Stm32DmaChFree(int Ch);

/**
 * @brief	Set up a channel. The channel is stopped, the peripheral drives it.
 *
 * The request is routed to the channel: CSELR or DMAMUX1.
 *
 * @param	Ch			: Channel number
 * @param	Req			: Request, STM32_DMA_REQ
 * @param	bToPeriph	: true - memory to peripheral, false - peripheral to memory
 * @param	pPeriph		: Peripheral data register
 * @param	WidthLog2	: 0 byte, 1 half word, 2 word
 * @param	bIrq		: true - transfer complete and error interrupts
 */
void Stm32DmaChSetup(int Ch, uint32_t Req, bool bToPeriph, volatile void *pPeriph, int WidthLog2, bool bIrq);

/**
 * @brief	Set whether the memory address of a channel increments.
 *
 * Setup sets it. A transfer of one value repeated, such as the dummy data of
 * a receive, switches it off before Start.
 *
 * @param	Ch		: Channel
 * @param	bInc	: true to increment the memory address, false to keep it
 */
void Stm32DmaChMemInc(int Ch, bool bInc);

/**
 * @brief	Start a transfer of Count bytes with the memory at pMem.
 *
 * The channel was set up with Stm32DmaChSetup. Count is a multiple of the
 * width, at most 65535 transfers.
 */
void Stm32DmaChStart(int Ch, const void *pMem, uint32_t Count);

/**
 * @brief	Stop the channel and clear its flags.
 */
void Stm32DmaChStop(int Ch);

/**
 * @brief	Stop the channel, take the bytes it has still to move, then
 * 			clear its flags.
 *
 * @return	Bytes not moved.
 */
uint32_t Stm32DmaChStopRemain(int Ch);

/**
 * @brief	Bytes the channel has still to move.
 */
uint32_t Stm32DmaChRemain(int Ch);

/**
 * @brief	Channel flags STM32_DMA_FLAG_xxx, those seen by the hardware and
 * 			those taken by Stm32DmaChAck() since the transfer started.
 */
uint32_t Stm32DmaChFlags(int Ch);

/**
 * @brief	Take the channel flags into the latch and clear them in the
 * 			hardware. A handler that does not clear the flags itself calls
 * 			this first, so that the channel interrupt does not fire again.
 * 			Stm32DmaChFlags() keeps reporting what was taken.
 */
void Stm32DmaChAck(int Ch);

/**
 * @brief	Clear the channel flags.
 */
void Stm32DmaChClear(int Ch);

#ifdef __cplusplus
}
#endif

#endif // __DMA_STM32_H__
