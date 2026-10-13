/**-------------------------------------------------------------------------
@file	dtc_r9a02.h

@brief	R9A02G021 data transfer controller (DTC)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 14. The DTC moves data when
an ICU event occurs, without the CPU. A driver routes its event to an IELSRn
(interrupt_r9a02.h), writes the transfer information records of that slot
with R9a02DtcSet and turns the slot over to the DTC with R9a02DtcEnable.
Each event then runs one transfer of the records (14.4).

Vector number: the IELSR number n of the slot, 0 to 31 (14.3.1). The vector
table holds the address of the first record of each slot, it is 128 bytes
in the section .bss.dtc_vector, aligned on 1 KB (DTCVBR). The R9A02G021
linker scripts put it at the start of the SRAM0 ECC area (IRAM), where it
uses no more than its size. With another script the section goes into .bss
and the alignment can leave a gap of up to 1 KB.

Records (R9a02DtcInfo_t, 16 bytes) are in SRAM, 4-byte aligned, and stay in
place while the slot is enabled. Chained records follow each other (MRB.CHNE
= 1 on all but the last one). After each transfer the DTC writes SAR, DAR
and the counters back into them.

End of a transfer (table 12.8, 14.4):
	- normal or block mode, the counter of the last record goes to 0: the
	  DTC clears IELSRn.DTCE and leaves IR set, the CPU interrupt of the slot
	  follows, its handler sees the end of the whole transfer
	- otherwise IR is cleared, no CPU interrupt
	- repeat mode never ends, no CPU interrupt
While IR is 1 the slot takes no new event, an event then is lost. MRB.DISEL
= 1, a CPU interrupt after each transfer with DTCE still 1, is not supported:
the dispatcher of interrupt_r9a02.cpp leaves a slot to the DTC while DTCE is
1.

The DTC runs in Sleep. Before Software Standby DTCST must be 0 unless the
DTC is used in Snooze (14.9), R9a02LowPowerEnter takes care of it.

Events that can start the DTC: R9a02DtcSet refuses the others. Those that
table 12.7 gives as CPU only are not accepted, TAU, RTC, COMP, error and DTC
end events among them.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#ifndef __DTC_R9A02_H__
#define __DTC_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#include "r9a02g021.h"

// MRA, mode register A (14.2.1)
#define R9A02_DTC_MRA_MD_NORMAL		(0U << 6)
#define R9A02_DTC_MRA_MD_REPEAT		(1U << 6)
#define R9A02_DTC_MRA_MD_BLOCK		(2U << 6)
#define R9A02_DTC_MRA_SZ_BYTE		(0U << 4)
#define R9A02_DTC_MRA_SZ_HALF		(1U << 4)
#define R9A02_DTC_MRA_SZ_WORD		(2U << 4)
#define R9A02_DTC_MRA_SM_FIXED		(0U << 2)		//!< Source address fixed
#define R9A02_DTC_MRA_SM_INC		(2U << 2)		//!< Source address incremented
#define R9A02_DTC_MRA_SM_DEC		(3U << 2)		//!< Source address decremented

// MRB, mode register B (14.2.2)
#define R9A02_DTC_MRB_CHNE			(1U << 7)		//!< Next record follows
#define R9A02_DTC_MRB_CHNS			(1U << 6)		//!< Chain only when the counter ends
#define R9A02_DTC_MRB_DISEL			(1U << 5)		//!< CPU interrupt after every transfer, not supported
#define R9A02_DTC_MRB_DTS			(1U << 4)		//!< Repeat or block area is the source
#define R9A02_DTC_MRB_DM_FIXED		(0U << 2)		//!< Destination address fixed
#define R9A02_DTC_MRB_DM_INC		(2U << 2)		//!< Destination address incremented
#define R9A02_DTC_MRB_DM_DEC		(3U << 2)		//!< Destination address decremented

#define R9A02_DTC_VECTOR_CNT		32U				//!< One per IELSR

#pragma pack(push, 4)

/// Transfer information record, manual figure 14.3
typedef struct __R9a02_Dtc_Info {
	uint16_t Rsv;			//!< Reserved, 0
	uint8_t Mrb;			//!< Mode register B
	uint8_t Mra;			//!< Mode register A
	uint32_t Sar;			//!< Source address
	uint32_t Dar;			//!< Destination address
	uint16_t Crb;			//!< Block count, block mode only
	uint16_t Cra;			//!< Transfer count. Repeat and block mode: size in [15:8], count in [7:0]
} R9a02DtcInfo_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// @brief	Set the records of a slot.
///
/// The first call starts the DTC (vector table, DTCST = 1). The slot must
/// not be enabled.
///
/// @param	IntNo	: IELSR number of a routed event that can start the DTC
/// @param	pInfo	: First record, in SRAM, 4-byte aligned, kept in place
/// 				  while the slot is enabled
///
/// @return	false if the slot is not routed, its event cannot start the DTC
/// 			(an IRQ pin in low level mode included), it is enabled, or pInfo
/// 			is not valid
bool R9a02DtcSet(int IntNo, R9a02DtcInfo_t *pInfo);

/// @brief	Give the events of a slot to the DTC.
///
/// Sets IELSRn.DTCE. The IR flag is cleared, an event already pending is
/// dropped: enable before the peripheral is started.
///
/// @param	IntNo	: IELSR number set by R9a02DtcSet
///
/// @return	false if the slot has no records or is enabled
bool R9a02DtcEnable(int IntNo);

/// @brief	Give the events of a slot back to the CPU.
///
/// Waits for a transfer of the slot in progress to end, then clears DTCE
/// and IR, a pending event is dropped. The records keep the state of the
/// last transfer.
///
/// @param	IntNo	: IELSR number
void R9a02DtcDisable(int IntNo);

/// @brief	Check whether a slot has the DTC.
///
/// @param	IntNo	: IELSR number
///
/// @return	true while DTCE is 1: the slot is enabled and its last transfer
/// 			has not been done
bool R9a02DtcEnabled(int IntNo);

/// @brief	Remove the records of a slot.
///
/// Disables the slot first. Call it before the route is removed.
///
/// @param	IntNo	: IELSR number
void R9a02DtcClear(int IntNo);

#ifdef __cplusplus
}
#endif

#endif
