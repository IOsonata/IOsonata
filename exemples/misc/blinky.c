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

MIT License

Copyright (c) 2014 I-SYST inc. All rights reserved.

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
#include <stdio.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "pulse_train.h"

#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

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
volatile bool g_bBut3Pressed = false;

void But1Handler(int IntNo, void *pCtx)
{
	if (IntNo == BUT1_SENSE_INT)
	{
		g_bBut1Pressed = true;
		printf("But 1 Int\r\n");
	}
}

#ifdef BUT2_SENSE_INT
void But2Handler(int IntNo, void *pCtx)
{
	if (IntNo == BUT2_SENSE_INT)
	{
		g_bBut2Pressed = true;
		printf("But 2 Int\r\n");
	}
}
#endif

#ifdef BUT3_SENSE_INT
void But3Handler(int IntNo, void *pCtx)
{
	if (IntNo == BUT3_SENSE_INT)
	{
		g_bBut3Pressed = true;
		printf("But 3 Int\r\n");
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
	// Configure Leds
	IOPinCfg(s_Leds, s_NbLeds);

	// Clear all leds
	for (int i = 0; i < s_NbLeds; i++)
	{
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	// Configure buttons
	IOPinCfg(s_Buttons, s_NbButtons);
	//IOPinEnableInterrupt(BUT1_SENSE_INT, BUT1_INT_PRIO, BUT1_PORT, BUT1_PIN, BUT1_SENSE, But1Handler, NULL);
	//IOPinEnableInterrupt(BUT2_SENSE_INT, BUT2_INT_PRIO, BUT2_PORT, BUT2_PIN, BUT2_SENSE, But2Handler, NULL);
	//IOPinEnableInterrupt(BUT3_SENSE_INT, BUT3_INT_PRIO, BUT3_PORT, BUT3_PIN, BUT3_SENSE, But3Handler, NULL);


	int i = 0;

	// Loop until button pressed
	while (g_bBut1Pressed == false)
	{
		if (IOPinRead(s_Buttons[0].PortNo, s_Buttons[0].PinNo) == 0)
		{
			//printf("%d: %x\r\n", i, x);
			g_bBut1Pressed = true;
		}
		IOPinToggle(s_Leds[i].PortNo, s_Leds[i].PinNo);
		msDelay(250);
	//	IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	//	msDelay(5000);
		i = (i + 1) % 3;
	}

	PulseTrain(&g_PulseTrainCfg, 0);

	return 0;
}


