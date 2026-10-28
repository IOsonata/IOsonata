/**-------------------------------------------------------------------------
@file	clock_lpc546xx.h

@brief	LPC546xx audio PLL

The audio PLL clocks the I2S Flexcomms and the MCLK pin. Its input is clk_in
when the core oscillator is external (g_McuOsc), the 12 MHz FRO otherwise.
Fout = Fin x 2M / (N x 2P), with the CCO (Fin x 2M / N) from 275 to 550 MHz
and the PLL reference (Fin / N) at 10 kHz or more, as the NXP SDK. The
fractional divider is not used: the audio rates of 48 kHz and 44.1 kHz and
their multiples are exact from 12 MHz, 24.576 MHz with N 125, M 1536, P 6 for
example.

Implementation in system_LPC546xx.c.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __CLOCK_LPC546XX_H__
#define __CLOCK_LPC546XX_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Start the audio PLL at the output closest to Freq.
 *
 * The PLL is powered down while it changes. Clocks taken from it stop during
 * the change.
 *
 * @param	Freq	: Output frequency to approach, Hz
 *
 * @return	Output frequency in Hz, the lowest or the highest the PLL makes
 * 			for a Freq out of its range.
 * 			0 - Freq 0, or the PLL did not lock. The PLL is then off.
 */
uint32_t Lpc546xxAudioPllStart(uint32_t Freq);

/**
 * @brief	Power the audio PLL down.
 */
void Lpc546xxAudioPllStop(void);

/**
 * @brief	Output frequency of the audio PLL, 0 when it is off.
 */
uint32_t Lpc546xxAudioPllFreq(void);

#ifdef __cplusplus
}
#endif

#endif // __CLOCK_LPC546XX_H__
