/**-------------------------------------------------------------------------
@file	joystick_menu_test.cpp

@brief	Host test of the joystick menu example.

With JOYSTICK_STOPWATCH defined it tests the stopwatch example.

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

#include "joystick_display_fixture.h"

#ifdef JOYSTICK_STOPWATCH
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

static void StopwatchChecks()
{
	assert(s_Page == 3 && s_Item == 3 && !s_Running);
	ExpectClock("00:00:00.00");
	Sample(0, 20);
	assert(!s_Running); // Held at boot ignored
	Press(0);
	assert(s_Running);
	// A single delayed iteration must account for all elapsed hardware time.
	Advance(61234);
	assert(s_ElapsedMs == 61234);
	ExpectClock("00:01:01.23");
	Press(2.64f);
	assert(s_HaveLap && s_LapMs == 61234 && s_Running);
	Advance(4321);
	assert(s_ElapsedMs == 65555 && s_LapMs == 61234);
	Press(0.66f);
	assert(s_ElapsedMs == 65555 && s_Running); // Reset ignored while running
	Press(0);
	assert(!s_Running);
	Advance(99999);
	assert(s_ElapsedMs == 65555);
	Press(0);
	assert(s_Running);
	Advance(1000);
	assert(s_ElapsedMs == 66555);
	Press(2.01f);
	assert(s_Page == 0 && s_Running);
	Advance(2000);
	assert(s_ElapsedMs == 68555); // Continues away from its page
	Press(1.32f);
	assert(s_Item == 0); // Four-item menu wraps
	Press(2.01f);
	assert(s_Item == 3);
	Press(0);
	assert(s_Page == 3 && s_Running);
	Press(0);
	assert(!s_Running);
	Press(0.66f);
	assert(s_ElapsedMs == 0 && !s_HaveLap);
	// Place the free-running timestamp near its uint32 millisecond wrap.
	ticks = uint64_t(0xFFFFFFF0U) * 1000;
	PollJoystick();
	Press(0);
	Advance(32);
	assert(s_ElapsedMs == 32);
	// ADC errors must neither pause time nor cause phantom key actions.
	readOk = false;
	Advance(1000);
	assert(s_InputError && s_ElapsedMs == 1032 && s_Running);
	readOk = true;
	Sample(0);
	assert(!s_InputError && s_Running);
	Press(0);
	assert(!s_Running);
	Press(0.66f);
	Press(0);
	Advance(359999980);
	assert(s_Running);
	Advance(100);
	assert(!s_Running && s_ElapsedMs == s_MaxElapsedMs);
	ExpectClock("99:59:59.99");
	Press(0);
	assert(!s_Running); // Full display requires reset
	Press(0.66f);
	assert(s_ElapsedMs == 0);
	Press(0);
	assert(s_Running);
	assert(!selected);
	std::puts("Stopwatch: hardware timestamps, OLED digits, pause/resume, lap, menu, ADC failure, wrap and limit PASS");
}
#endif

int main()
{
	assert(HardwareInit() && displayOn && ledHigh && !s_LedOn);
	assert(resetLow == 1 && resetHigh >= 1);
#ifdef JOYSTICK_STOPWATCH
	StopwatchChecks();
#else
	assert(DecodeKey(0) == KEY_SELECT && DecodeKey(0.66f) == KEY_LEFT);
	assert(DecodeKey(1.32f) == KEY_DOWN && DecodeKey(2.01f) == KEY_UP);
	assert(DecodeKey(2.64f) == KEY_RIGHT && DecodeKey(3.3f) == KEY_NONE);
	assert(DecodeKey(-1) == KEY_INVALID && DecodeKey(NAN) == KEY_INVALID);
	// A key held during boot is ignored until a stable release.
	Sample(0, 20);
	assert(!s_LedOn);
	Press(0);
	assert(s_LedOn && !ledHigh);
	Sample(0, 20);
	assert(s_LedOn);
	Sample(3.3f);
	Sample(1.32f, 2);
	Sample(3.3f);
	assert(s_Item == 0); // Short bounce ignored
	Press(1.32f);
	assert(s_Item == 1);
	uint8_t menu[sizeof(ram)];
	memcpy(menu, ram, sizeof(ram));
	Press(0);
	assert(s_Page == 1 && memcmp(menu, ram, sizeof(ram)) != 0);
	for (int i = 0; i < 20; i++)
	{
		Press(2.64f);
	}
	assert(s_Level == 10);
	for (int i = 0; i < 20; i++)
	{
		Press(0.66f);
	}
	assert(s_Level == 1);
	Press(0);
	assert(s_Page == 0);
	Press(1.32f);
	assert(s_Item == 2);
	Press(0);
	assert(s_Page == 2);
	Press(0.66f);
	assert(s_Page == 0);
	Press(1.32f);
	assert(s_Item == 0);
	Press(2.01f);
	assert(s_Item == 2); // Wrap both ways
	readOk = false;
	PollJoystick();
	assert(s_InputError);
	readOk = true;
	Sample(0);
	assert(!s_InputError && s_Page == 0);
	// Recovery requires a release; a stale held selection cannot activate.
	Press(0);
	assert(s_Page == 2);
	assert(!selected);
	std::puts("Joystick menu: SPI/RAM, ladder, debounce, held keys, navigation, bounds and read failure PASS");
#endif
}
