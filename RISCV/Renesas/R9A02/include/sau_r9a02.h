/**-------------------------------------------------------------------------
@file	sau_r9a02.h

@brief	R9A02G021 serial array unit channel sharing and Snooze reception

The SAU channels are used by the UART driver (uart_r9a02.cpp) and the
simplified SPI driver (spi_r9a02.cpp). A driver claims its channels before it
configures them, so a channel cannot be set up by two drivers at once, and it
gets its transfer clock from R9a02SauSetRate: the operation clocks CKm0 and
CKm1 and their prescaler register SPSm are common to all channels of a unit
(R9A02G021 User's Manual R01UH1036EJ0110, 23.3.1).

Channels: unit 0 has channels 0 to 3, unit 1 channels 0 and 1 (tables 23.1
to 23.3).

The application calls R9a02UartSnooze and R9a02SpiSnooze, at the end of this
header, for the reception in Snooze of UART0, UART2, SPI00 and SPI20 (23.5.7,
23.6.3). They are in uart_r9a02.cpp and spi_r9a02.cpp.

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
#include "device_intrf.h"

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

/// @brief	Choose the operation clock PCLKB / 2^Prs.
///
/// CKm0 or CKm1 is taken: one already at Prs, or one whose prescaler can
/// change, as for R9a02SauSetRate. The channels of ChMask must be claimed
/// and stopped. The caller writes pClk->Cks to their SMRmn and their STCLK,
/// pClk->Sdr is 0.
///
/// @param	Unit	: 0 or 1
/// @param	ChMask	: Channels that use the clock
/// @param	Prs		: Prescaler 0 to 15, fMCK at most 32 MHz
/// @param	pClk	: Settings found
///
/// @return	false when no clock can have this prescaler
bool R9a02SauSetPrs(uint32_t Unit, uint16_t ChMask, uint32_t Prs, R9a02SauClk_t *pClk);

/// @brief	Clock source for SAU reception in Snooze (23.3.25).
///
/// The CPU and PCLKB must run from the HOCO or the MOCO, the main oscillator
/// (EXTAL) stopped.
///
/// @return	Source frequency in Hz (HOCO 24, 32 or 48 MHz, MOCO 8 MHz), 0 when
/// 			the clocks do not allow Snooze reception
uint32_t R9a02SauSnoozeSrcHz(void);

/// @brief	UART reception in Snooze, UART0 and UART2 (23.6.3).
///
/// In Software Standby a start bit on RxD starts Snooze, the UART receives
/// the character without the CPU. A good character wakes the CPU: the
/// Snooze end interrupt puts it in the RX CFIFO and the UART back in normal
/// operation, then UART_EVT_RXDATA. A character with an error sends the MCU
/// back to Software Standby (SCRm1.EOC = 1, SSCm.SSEC = 1, figure 23.41).
///
/// Enabling sets up Snooze with R9a02SnoozeSet (request RxD, wake-up on the
/// reception end, back to Software Standby on an error), replacing any
/// other Snooze set up, and adds a Software Standby hook. Requirements:
/// UARTInit with bIntMode, rate 4800 or 9600, PCLKB the HOCO at 24 or 32 MHz
/// without divider (table 23.95), EXTAL stopped. The UART then runs on the
/// operation clock of table 23.95, the closest STCLK gives the rate in
/// normal operation; in Snooze STCLK is the one of the table. The TX CFIFO
/// is sent before the channels stop, when enabling and at each Software
/// Standby. Set the clocks before, and call R9a02LowPowerEnter with
/// interrupts enabled. Once another Snooze set up replaces this one
/// (R9a02SnoozeSet, R9a02SpiSnooze), the UART stays in normal operation;
/// disabling then gives it back the other rates.
///
/// @param	pDev	: UART device interface (UARTDev_t DevIntrf, or the UART object)
/// @param	bEnable	: true to enable, false to remove the Snooze set up
///
/// @return	false when the UART or the clocks do not allow it
bool R9a02UartSnooze(DevIntrf_t * const pDev, bool bEnable);

/// @brief	SPI slave reception in Snooze, SPI00 and SPI20 (23.5.7).
///
/// In Software Standby an SCK edge starts Snooze, the slave receives the
/// first byte of the frame without the CPU (MISO stays at its idle level),
/// its end wakes the CPU, which then takes over the frame from byte 1. The
/// master leaves the slave time to wake up after byte 0 (Software Standby
/// to Normal, table 37.21, plus the interrupt) before it clocks byte 1.
///
/// Enabling sets up Snooze with R9a02SnoozeSet (request SCK, wake-up on the
/// reception end), replacing any other Snooze set up, and adds a Software
/// Standby hook. Requirements: SPIMODE_SLAVE on SPI00 (DevNo 0) or SPI20
/// (DevNo 4), rate at most 500 kHz (SetRate above it fails while enabled),
/// PCLKB the HOCO or the MOCO, EXTAL stopped. A frame already started at the
/// standby entry is not received in Snooze. Call R9a02LowPowerEnter with
/// interrupts enabled. Once another Snooze set up replaces this one, the
/// slave stays in normal operation.
///
/// @param	pDev	: SPI device interface (SPIDev_t DevIntrf, or the SPI object)
/// @param	bEnable	: true to enable, false to remove the Snooze set up
///
/// @return	false when the SPI or the clocks do not allow it
bool R9a02SpiSnooze(DevIntrf_t * const pDev, bool bEnable);

#ifdef __cplusplus
}
#endif

#endif
