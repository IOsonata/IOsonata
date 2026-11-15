/**-------------------------------------------------------------------------
@file	clock_display_digits.h

@brief	128x32 monochrome seven-segment clock face

The bitmap is row-major with the most significant bit first in each byte.

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

#ifndef __CLOCK_DISPLAY_DIGITS_H__
#define __CLOCK_DISPLAY_DIGITS_H__

#include <stdint.h>
#include <string.h>

#define CLOCK_FACE_WIDTH 128
#define CLOCK_FACE_HEIGHT 32
#define CLOCK_FACE_BYTES (CLOCK_FACE_WIDTH * CLOCK_FACE_HEIGHT / 8)

// Segments: top, upper right, lower right, bottom, lower left, upper left, middle.
static const uint8_t s_ClockSegments[10] = {
	0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f
};
static const uint8_t s_ClockDigitX[6] = {2, 20, 45, 63, 88, 106};
static const uint8_t s_ClockBars[7][4] = {
	{3, 0, 10, 3}, {13, 3, 3, 11}, {13, 17, 3, 11},
	{3, 28, 10, 3}, {0, 17, 3, 11}, {0, 3, 3, 11}, {3, 14, 10, 3}
};

static void ClockRect(uint8_t *pFrame, unsigned X, unsigned Y, unsigned Width, unsigned Height)
{
	for (unsigned y = Y; y < Y + Height; y++)
	{
		for (unsigned x = X; x < X + Width; x++)
		{
			pFrame[y * (CLOCK_FACE_WIDTH / 8) + x / 8] |= 0x80U >> (x & 7);
		}
	}
}

static void ClockRender(uint8_t *pFrame, uint32_t Seconds)
{
	uint32_t time = Seconds % 86400U;
	uint8_t digits[6];

	digits[0] = time / 36000U;
	digits[1] = (time / 3600U) % 10U;
	digits[2] = (time / 600U) % 6U;
	digits[3] = (time / 60U) % 10U;
	digits[4] = (time / 10U) % 6U;
	digits[5] = time % 10U;
	memset(pFrame, 0, CLOCK_FACE_BYTES);
	for (unsigned i = 0; i < 6; i++)
	{
		uint8_t segments = s_ClockSegments[digits[i]];
		for (unsigned b = 0; b < 7; b++)
		{
			if (segments & (1U << b))
			{
				ClockRect(pFrame, s_ClockDigitX[i] + s_ClockBars[b][0], s_ClockBars[b][1],
						  s_ClockBars[b][2], s_ClockBars[b][3]);
			}
		}
	}
	ClockRect(pFrame, 39, 9, 3, 3);
	ClockRect(pFrame, 39, 21, 3, 3);
	ClockRect(pFrame, 82, 9, 3, 3);
	ClockRect(pFrame, 82, 21, 3, 3);
}

#endif
