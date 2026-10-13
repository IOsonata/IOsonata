/**-------------------------------------------------------------------------
@file	sau_r9a02.h

@brief	R9A02G021 serial array unit channel sharing

The SAU channels are used by the UART driver (uart_r9a02.cpp) and the
simplified SPI driver (spi_r9a02.cpp). A driver claims its channels before it
configures them, so a channel cannot be set up by two drivers at once, and it
gets its transfer clock from R9a02SauSetRate: the operation clocks CKm0 and
CKm1 and their prescaler register SPSm are common to all channels of a unit
(R9A02G021 User's Manual R01UH1036EJ0110, 23.3.1).

Channels: unit 0 has channels 0 to 3, unit 1 channels 0 and 1 (tables 23.1
to 23.3).

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef __SAU_R9A02_H__
#define __SAU_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#include "r9a02g021.h"

#define R9A02_SAU_UNIT_CNT			2U
#define R9A02_SAU_CHAN_CNT			4U			//!< Channels of unit 0
#define R9A02_SAU_FMCK_MAX			32000000UL	//!< Table 23.5, PCLKB 48 MHz needs PRS >= 1
#define R9A02_SAU_PRS_MAX			15U
#define R9A02_SAU_DIV_MAX			128U		//!< STCLK + 1

/// Transfer clock of a channel group, set by R9a02SauSetRate
typedef struct __R9a02_Sau_Clk {
	uint16_t Cks;					//!< SMRmn.CKS value, 0 or R9A02_SAU_SMR_CKS
	uint16_t Sdr;					//!< SDRmn value with STCLK set and DAT 0
	uint32_t FmckHz;				//!< Operation clock fMCK
} R9a02SauClk_t;

#ifdef __cplusplus
extern "C" {
#endif

/// Register block of a unit
static inline R9A02_SAU_Type *R9a02SauReg(uint32_t Unit)
{
	return Unit ? R9A02_SAU1 : R9A02_SAU0;
}

/// @brief	Claim channels of a unit for a driver instance.
///
/// The first claim of a unit releases its module stop.
///
/// @param	Unit	: 0 or 1
/// @param	ChMask	: Channel bits
/// @param	pOwner	: Driver instance. A channel it already has is claimed again.
///
/// @return	false if a channel does not exist or another owner has it
bool R9a02SauClaim(uint32_t Unit, uint16_t ChMask, const void *pOwner);

/// @brief	Release channels of a driver instance.
///
/// The unit is put in module stop when none of its channels is claimed.
///
/// @param	Unit	: 0 or 1
/// @param	ChMask	: Channel bits, those of another owner are left
/// @param	pOwner	: Driver instance
void R9a02SauRelease(uint32_t Unit, uint16_t ChMask, const void *pOwner);

/// @brief	Choose the operation clock and divider of a transfer clock.
///
/// Transfer clock = fMCK / (2 x Div), Div = STCLK + 1 from MinDiv to 128,
/// fMCK = PCLKB / 2^PRS, at most 32 MHz. CKm0 or CKm1 is taken. The prescaler
/// of a clock is changed only when no other channel of the unit runs and no
/// other claimed channel uses that clock, SPSm is then written here.
///
/// The channels of ChMask must be claimed and stopped. The caller writes
/// pClk->Cks to their SMRmn and pClk->Sdr to their SDRmn.
///
/// @param	Unit	: 0 or 1
/// @param	ChMask	: Channels that use the clock
/// @param	Rate	: Requested transfer clock in Hz
/// @param	MinDiv	: Smallest STCLK + 1 the mode allows
/// @param	MaxRate	: Highest transfer clock the mode allows
/// @param	pClk	: Settings found
///
/// @return	Transfer clock in Hz, the closest to Rate, 0 when none fits
uint32_t R9a02SauSetRate(uint32_t Unit, uint16_t ChMask, uint32_t Rate, uint32_t MinDiv,
						 uint32_t MaxRate, R9a02SauClk_t *pClk);

#ifdef __cplusplus
}
#endif

#endif
