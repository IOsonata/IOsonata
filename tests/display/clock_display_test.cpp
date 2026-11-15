/**-------------------------------------------------------------------------
@file	clock_display_test.cpp

@brief	Clock display host tests

Time adjustment, rollover and bitmap bounds.

@author	Hoang Nguyen Hoan
@date	Nov. 15, 2026

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

#include <cassert>
#include <cstdio>
#include <cstring>
#include "clock_display_digits.h"
#include "clock_display_time.h"

int main(int argc, char **argv)
{
	ClockTime_t time = {43200};
	uint8_t frame[CLOCK_FACE_BYTES + 2];
	uint8_t midnight[CLOCK_FACE_BYTES];

	assert(ClockTimeUpdate(&time, 0, 48000000, false) == 43200);
	assert(ClockTimeUpdate(&time, 48000000ULL * 43200, 48000000, false) == 0);
	assert(ClockTimeUpdate(&time, 48000000ULL * 86400, 48000000, false) == 43200);
	// One sample of bounce does not count as a press.
	time = {};
	ClockTimeUpdate(&time, 1, 20, true);
	ClockTimeUpdate(&time, 2, 20, false);
	assert(time.Offset == 0 && !time.Pressed);
	// A short press increments once, after the second release sample.
	ClockTimeUpdate(&time, 3, 20, true);
	ClockTimeUpdate(&time, 4, 20, true);
	ClockTimeUpdate(&time, 5, 20, false);
	assert(time.Offset == 0);
	ClockTimeUpdate(&time, 6, 20, false);
	assert(time.Offset == 60 && !time.Pressed);
	// A hold repeats hours, and does not add a minute on release.
	ClockTimeUpdate(&time, 7, 20, true);
	ClockTimeUpdate(&time, 8, 20, true);
	ClockTimeUpdate(&time, 28, 20, true);
	assert(time.Offset == 3660 && time.Held);
	ClockTimeUpdate(&time, 48, 20, true);
	assert(time.Offset == 7260);
	ClockTimeUpdate(&time, 49, 20, false);
	ClockTimeUpdate(&time, 50, 20, false);
	assert(time.Offset == 7260 && !time.Pressed);
	// Delayed foreground work derives elapsed time from the counter.
	time = {};
	assert(ClockTimeUpdate(&time, 32768ULL * 12345, 32768, false) == 12345);
	// All times of day remain within the fixed bitmap.
	frame[0] = 0xa5;
	frame[sizeof(frame) - 1] = 0x5a;
	ClockRender(midnight, 0);
	for (unsigned seconds = 0; seconds <= 86400; seconds++)
	{
		ClockRender(frame + 1, seconds);
		assert(frame[0] == 0xa5 && frame[sizeof(frame) - 1] == 0x5a);
		assert((frame[1 + 9 * 16 + 39 / 8] & (0x80U >> (39 & 7))) != 0);
	}
	assert(memcmp(frame + 1, midnight, CLOCK_FACE_BYTES) == 0);
	// Known digit positions: 1 has no top segment; 2 has one.
	ClockRender(frame + 1, 12 * 3600 + 34 * 60 + 56);
	assert((frame[1 + 5 / 8] & (0x80U >> (5 & 7))) == 0);
	assert((frame[1 + 23 / 8] & (0x80U >> (23 & 7))) != 0);
	if (argc == 2)
	{
		FILE *out = fopen(argv[1], "wb");
		assert(out != nullptr);
		fputs("P4\n128 32\n", out);
		assert(fwrite(frame + 1, 1, CLOCK_FACE_BYTES, out) == CLOCK_FACE_BYTES);
		fclose(out);
	}
	puts("Clock time, rollover, button adjustment and bitmap bounds: PASS");
}
