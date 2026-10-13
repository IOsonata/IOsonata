/**-------------------------------------------------------------------------
@file	elc_r9a02.h

@brief	R9A02G021 event link controller (ELC)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 15. An event link starts a
peripheral action from an event of another peripheral, without the CPU:

	R9A02_ELC_DEST_ADC		: start of an A/D conversion (ADM1.ADTRS = 100b)
	R9A02_ELC_DEST_PORT1/2	: output set, reset or input latch of a port group
	R9A02_ELC_DEST_DAC0/1	: start of a D/A conversion of a channel
	R9A02_ELC_DEST_TML32	: TML32 count clock or capture trigger

The event is the number of table 15.3, the same as the ICU event numbers of
interrupt_r9a02.h (R9A02_EVT_xxx) except the DTC end event, which is
R9A02_ELC_EVT_DTC_END here. SYSTEM_SNZREQ links to the ADC, the ports and
the TML32 only. Source and destination modules must be running.

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
#ifndef __ELC_R9A02_H__
#define __ELC_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#include "r9a02g021.h"

#define R9A02_ELC_EVT_DTC_END		0x0AU		//!< DTC0_DTCEND, table 15.3

/// Event link destinations, ELSRn number, table 15.2
typedef enum __R9a02_Elc_Dest {
	R9A02_ELC_DEST_ADC = 8,
	R9A02_ELC_DEST_PORT1 = 14,
	R9A02_ELC_DEST_PORT2 = 15,
	R9A02_ELC_DEST_DAC1 = 19,
	R9A02_ELC_DEST_DAC0 = 20,
	R9A02_ELC_DEST_TML32 = 23,
} R9A02_ELC_DEST;

#ifdef __cplusplus
extern "C" {
#endif

/// @brief	Link an event to a destination, or remove the link.
///
/// The first link starts the ELC, it is stopped again when no link is left
/// (15.3.3). Configure the destination before the link and start the
/// source after it.
///
/// @param	Dest	: Destination
/// @param	EvtId	: Event number, 0 removes the link
///
/// @return	false if the event cannot be linked to the destination
bool R9a02ElcLink(R9A02_ELC_DEST Dest, uint8_t EvtId);

/// @brief	Event linked to a destination.
///
/// @param	Dest	: Destination
///
/// @return	Event number, 0 when none
uint8_t R9a02ElcEvent(R9A02_ELC_DEST Dest);

#ifdef __cplusplus
}
#endif

#endif
