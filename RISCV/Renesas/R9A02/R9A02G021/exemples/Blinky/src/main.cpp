/*--------------------------------------------------------------------------
File   : main.cpp

Author : Hoang Nguyen Hoan          May 2026

Desc   : Blinky example for FPB-R9A02G021 (Renesas R9A02G021 RISC-V).

         Toggles both on-board LEDs at ~2 Hz using the IOsonata GPIO
         abstraction.  This is the minimal smoke test for:
           - ResetEntry / SystemInit (clock to 24 MHz HOCO by default)
           - PFS unlock + write through IOPinConfig
           - Atomic POSR/PORR pin set/clear through IOPinToggle
           - usDelay / msDelay calibrated against SystemCoreClockPeriodus

         The LED pin map lives in board.h so the same source compiles
         for other R9A02G021 boards by swapping the board header.

Copyright (c) 2026, I-SYST INC. All rights reserved.

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or promote
products derived from this software without specific prior written
permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "miscdev/led.h"

#include "board.h"

static const IOPinCfg_t s_LedPins[] = LED_PINS_MAP;
static const int s_NbLedPins = sizeof(s_LedPins) / sizeof(s_LedPins[0]);

int main(void)
{
	// Configure both on-board LEDs as plain GPIO outputs.  No pulls, no
	// open-drain -- the FPB-R9A02G021 wires the LEDs through series
	// resistors with active-high polarity (see board.h: LED_LOGIC_HIGH).
	IOPinCfg(s_LedPins, s_NbLedPins);

	// Make sure both LEDs start in the OFF (driven low) state regardless
	// of whatever the option-setting memory left them as after reset.
	for (int i = 0; i < s_NbLedPins; i++)
	{
		IOPinClear(s_LedPins[i].PortNo, s_LedPins[i].PinNo);
	}

	for (;;)
	{
		// Toggle every LED on the board, then sleep ~250 ms -> 2 Hz visible
		// blink rate with the default 24 MHz HOCO from SystemInit().
		for (int i = 0; i < s_NbLedPins; i++)
		{
			IOPinToggle(s_LedPins[i].PortNo, s_LedPins[i].PinNo);
		}
		msDelay(250);
	}

	// Unreachable -- silence "non-void function returns no value" if the
	// build settings ever turn that into an error.
	return 0;
}
