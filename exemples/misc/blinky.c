/**-------------------------------------------------------------------------
@example	blinky.c

@brief	Blinky example

This example shows how to use GPIO to
- Blink LED
- Detect button
- Generate a pulse train. Pulse moving from 1 GPIO to the next

@author	Hoang Nguyen Hoan
@date	Aug. 31, 2014

@license

Copyright (c) 2014, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

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
#include <stdio.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "pulse_train.h"
//#include "bsdlib_os_bare.h"
#include "board.h"

/// Buttons & LED pins map
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;
static const int s_NbButtons = sizeof(s_Buttons) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

/// Pulse train config
static const PulseTrainPin_t s_PulseTrainPins[] = PULSE_TRAIN_PINS_MAP;

PulseTrainCfg_t g_PulseTrainCfg = {
	.pPins = (PulseTrainPin_t *)s_PulseTrainPins,
	.NbPins = sizeof(s_PulseTrainPins) / sizeof(PulseTrainPin_t),
	.Period = 1,
	.Pol = PULSE_TRAIN_POL_HIGH
};

volatile bool g_bBut1Pressed = false;
volatile bool g_bBut2Pressed = false;

void But1Handler(int IntNo)
{
	if (IntNo == BUT1_SENSE_INT)
	{
		g_bBut1Pressed = true;
		printf("But 1 Int\r\n");
	}
}

#ifdef BUT2_SENSE_INT
void But2Handler(int IntNo)
{
	if (IntNo == BUT2_SENSE_INT)
	{
		g_bBut2Pressed = true;
		printf("But 2 Int\r\n");
	}
}
#endif

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	// Configure
	//IOPinCfg(s_Buttons, s_NbButtons);
	//IOPinEnableInterrupt(BUT1_SENSE_INT, BUT1_INT_PRIO, BUT1_PORT, BUT1_PIN, BUT1_SENSE, But1Handler);
	//IOPinEnableInterrupt(BUT2_SENSE_INT, BUT2_INT_PRIO, BUT2_PORT, BUT2_PIN, BUT2_SENSE, But2Handler);
	IOPinCfg(s_Leds, s_NbLeds);

//	uint64_t id = (uint64_t)NRF_FICR_S->INFO.DEVICEID[0] | ((uint64_t)NRF_FICR_S->INFO.DEVICEID[1] << 32);

	//IOPinConfig(0, 28, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	//IOPinConfig(0, 29, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	//IOPinConfig(0, 30, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	//IOPinConfig(0, 31, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
//	IOPinConfig(6, 8, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
#if 0
	while(1)
	{
		//IOPinClear(LED1_PORT, LED1_PIN);
		//msDelay(250);
		//IOPinSet(LED1_PORT, LED1_PIN);
		IOPinClear(0, 28);
		msDelay(1000);
		IOPinSet(0, 28);
		IOPinClear(0, 29);
		//IOPinToggle(3, 28);
		msDelay(1000);
		IOPinSet(0, 29);
		IOPinClear(0, 30);
		//IOPinToggle(3, 28);
		msDelay(1000);
		IOPinSet(0, 30);
		IOPinClear(0, 31);
		//IOPinToggle(3, 28);
		msDelay(1000);
		IOPinSet(0, 31);
	}

#endif
	//while(1) __WFE();
#if 0
	NRF_REGULATORS_S->DCDCEN = REGULATORS_DCDCEN_DCDCEN_Enabled;
	NRF_CLOCK_S->LFCLKSRC = CLOCK_LFCLKSRCCOPY_SRC_LFXO;

	NRF_CLOCK_S->TASKS_LFCLKSTART = 1;

	do {

	} while (NRF_CLOCK_S->EVENTS_LFCLKSTARTED == 0);
	NRF_CLOCK_S->EVENTS_LFCLKSTARTED = 0;

	int res = bsdlid_init(NULL, true);
	if (res == 0)
	{
		while (1)
		{
			IOPinClear(s_Leds[0].PortNo, s_Leds[0].PinNo);
			msDelay(250);
			IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
			msDelay(250);
		}
	}
#endif
#if 0
	for (int j = 0; j < 5; j++)
	{
		for (int i = 0; i < s_NbLeds; i++)
		{
			IOPinClear(s_Leds[i].PortNo, s_Leds[i].PinNo);
		}
		msDelay(250);
		for (int i = 0; i < s_NbLeds; i++)
		{
			if (res & (1<<i))
			{
				IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
			}
			else
			{
				IOPinClear(s_Leds[i].PortNo, s_Leds[i].PinNo);
			}
		}
		msDelay(250);
	}
#endif
	PulseTrain(&g_PulseTrainCfg, 0);

	return 0;
}


