/**-------------------------------------------------------------------------
@file	countdown_timer_test.cpp

@brief	Host test of the countdown timer example.

Model the board's SPI OLED and joystick while using hardware timer stamps.

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

#define COUNTDOWN_TIMER_TEST
#include "joystick_display_fixture.h"

static void Advance(uint32_t Ms)
{
	ticks += uint64_t(Ms) * 1000;
	PollJoystick();
}

static void ExpectClock(const char *Text)
{
	uint8_t actual[8][128];

	memcpy(actual, ram, sizeof(ram));
	g_Lcd.Clear();
	g_Lcd.SetCursor(0, 16);
	g_Lcd.printf("%s", Text);
	for (unsigned y = 16; y < 23; y++)
	{
		for (unsigned x = 0; x < 128; x++)
		{
			assert((actual[y / 8][x] & (1U << (y % 8))) == (ram[y / 8][x] & (1U << (y % 8))));
		}
	}
}

int main()
{
	assert(HardwareInit() && displayOn && ledHigh && !s_Done);
	assert(!s_Running && s_RemainingMs == 60000);
	ExpectClock("01:00.0");
	Sample(0, 20);
	assert(!s_Running); // Held at boot must be released.
	Press(2.64f);
	assert(s_DurationMs == 70000);
	Sample(2.64f, 20);
	assert(s_DurationMs == 70000); // A held key changes the preset once.
	Press(2.01f);
	assert(s_DurationMs == 130000);
	Press(1.32f);
	assert(s_DurationMs == 70000);
	Press(0);
	assert(s_Running);
	Advance(12345);
	assert(s_RemainingMs == 57655);
	ExpectClock("00:57.7");
	Press(2.64f);
	Press(0.66f);
	assert(s_DurationMs == 70000 && s_RemainingMs == 57655); // Editing/reset ignored while running.
	Press(0);
	assert(!s_Running);
	Advance(90000);
	assert(s_RemainingMs == 57655); // Pause excludes time spent paused.
	Press(0);
	Advance(1000);
	assert(s_Running && s_RemainingMs == 56655);
	readOk = false;
	Advance(56654);
	assert(s_InputError && s_Running && s_RemainingMs == 1 && ledHigh);
	ExpectClock("00:00.1"); // Do not show zero before the deadline.
	Advance(20);
	assert(!s_Running && s_Done && s_RemainingMs == 0 && !ledHigh);
	ExpectClock("00:00.0"); // Expiry still runs while the joystick fails.
	readOk = true;
	Sample(0);
	assert(!s_InputError && s_Done && !s_Running); // Recovery requires release.
	Press(0);
	assert(s_Running && !s_Done && s_RemainingMs == 70000 && ledHigh);
	Advance(1000);
	Press(0);
	Press(0.66f);
	assert(!s_Running && s_RemainingMs == s_DurationMs && ledHigh);
	for (unsigned i = 0; i < 3; i++)
	{
		Press(1.32f);
	}
	assert(s_DurationMs == s_MinDurationMs && s_RemainingMs == 10000);
	for (unsigned i = 0; i < 105; i++)
	{
		Press(2.01f);
	}
	Press(2.64f);
	assert(s_DurationMs == s_MaxDurationMs && s_RemainingMs == 5999000);
	ExpectClock("99:59.0");
	ticks = uint64_t(0xFFFFFFF0U) * 1000;
	PollJoystick();
	Press(0);
	Advance(32);
	assert(s_Running && s_RemainingMs == s_MaxDurationMs - 32);
	Advance(s_MaxDurationMs); // A delayed iteration crossing expiry must clamp at zero.
	assert(s_Done && !s_Running && s_RemainingMs == 0 && !ledHigh && !selected);
	std::puts("Countdown: OLED digits, preset bounds, debounce, pause/resume, reset, expiry LED, ADC failure and timer wrap PASS");
}
