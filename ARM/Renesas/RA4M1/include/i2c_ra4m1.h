/**-------------------------------------------------------------------------
@file	i2c_ra4m1.h

@brief	RA4M1 IIC diagnostic helper

Fills a caller buffer with the clock and IIC register state of one IIC
instance. Nothing is printed by the library.

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
#ifndef __I2C_RA4M1_H__
#define __I2C_RA4M1_H__

#include "device_intrf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Write a one line text dump of the IIC state into Buf.
 *
 * Fields: PCLKB in Hz, MSTPCRB, SCKDIVCR, then ICCR1 ICCR2 ICMR1 ICMR2 ICMR3
 * ICFER ICSER ICIER ICSR1 ICSR2 ICBRL ICBRH in hex, then the SCL and SDA
 * input levels from ICCR1.
 *
 * @param	pIntrf	: Interface of an initialized RA4M1 I2C instance
 * @param	Buf		: Output buffer
 * @param	Len		: Size of Buf
 *
 * @return	Number of characters written, 0 on invalid handle
 */
int Ra4m1I2cDump(DevIntrf_t * const pIntrf, char *Buf, int Len);

#ifdef __cplusplus
}
#endif

#endif // __I2C_RA4M1_H__
