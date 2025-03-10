/**-------------------------------------------------------------------------
@file	crc.c

@brief	CRC calculations
		 reference : http://en.wikipedia.org/wiki/Computation_of_CRC

@author	Nguyen Hoan Hoang
@date	Dec. 27, 2011

@license

MIT License

Copyright (c) 2011 I-SYST inc. All rights reserved.

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
#include "crc.h"

/**
 * @brief   Calculate 8 bits CRC value
 *          Polynomial : (x7 + x3 + 1) × x (left-shifted CRC-7-CCITT)
 *          0x12 = (0x09 << 1) (MSBF/normal)
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 * 			SeedVal : Initial CRC seed value
 *
 * @return	8 bits CRC value
 */
uint8_t crc8_ccitt(uint8_t *pData, int Len, uint8_t SeedVal)
{
	uint8_t e, f, crc;

	crc = SeedVal;
	for (int i = 0; i < Len; i++)
	{
		e = crc ^ pData[i];
		f = e ^ (e >> 4) ^ (e >> 7);
		crc = (f << 1) ^ (f << 4);
	}
	return crc;
}

/**
 * @brief   Calculate 8 bits CRC value
 *
 * Brute force general crc calculation where polynomial value is passed
 * as parameter
 *
 *	ex. : ATM Polynomial : x8 + x2 + x + 1 => 0x107
 *
 * @param	Poly	: Polynomial value
 * 			pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 * 			SeedVal : Initial CRC seed value
 *
 * @return	8 bits CRC value
 */
uint8_t crc8(uint16_t Poly, uint8_t *pData, int Len, uint8_t SeedVal)
{
	uint16_t crc = SeedVal;
	Poly <<= 7;

	for (int i = 0; i < Len; i++)
	{
		crc ^= pData[i] << 8;

		for (int j = 0; j < 8; j++)
		{
			if (crc & 0x8000)
			{
				crc ^= Poly;
			}
			crc <<= 1;
		}
	}

	return crc >> 8;
}

/**
 * @brief   Calculate 16 bits CRC value
 *          Polynomial : x16 + x15 + x2 + 1 (CRC-16-ANSI)
 *          0x8005 (MSBF/normal)
 *
 * @param   pData   : Pointer to data buffer to calculate
 *          Len     : Data length in bytes
 *          SeedVal : Initial CRC seed value
 *
 * @return 16 bits CRC value
 */
uint16_t crc16_ansi(uint8_t *pData, int Len, uint16_t SeedVal)
{
    uint8_t  s;
    uint16_t t, crc;

    crc = SeedVal;
    for (int i = 0; i < Len; i++)
    {
        s = pData[i] ^ (crc >> 8);
        t = s ^ (s >> 4);
        t ^= (t >> 2);
        t ^= (t >> 1);
        t &= 1;
        t |= (s << 1);
        crc = (crc << 8) ^ t ^ (t << 1) ^ (t << 15);
    }

    return crc;
}

/**
 * @brief   Calculate 16 bits CRC value
 *          Polynomial : x16 + x12 + x5 + 1 (CRC-16-CCITT)
 *          0x1021 (MSBF/normal)
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 *          SeedVal : Initial CRC seed value
 *
 * @return	16 bits CRC value
 */
uint16_t crc16_ccitt(uint8_t *pData, int Len, uint16_t SeedVal)
{
	uint8_t  s, t;
	uint16_t crc;

	crc = SeedVal;
	for (int i = 0; i < Len; i++)
	{
		s = pData[i] ^ (crc >> 8);
		t = s ^ (s >> 4);
		crc = (crc << 8) ^ t ^ (t << 5) ^ (t << 12);
	}

	return crc;
}

/**
 * @brief	Calculate 32 bits CRC value
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * 			Len		: Data length in bytes
 *
 * @return	32 bits CRC value
 */
uint32_t crc32(uint8_t *pData, int Len)
{
	char byte; 	// current byte
	uint32_t crc; 	// CRC result
	int q0, q1, q2, q3; // temporary variables
	crc = 0xFFFFFFFF;

	for (int i = 0; i < Len; i++)
	{
		byte = *pData++;

		for (int j = 0; j < 2; j++)
		{
			if (((crc >> 28) ^ (byte >> 3)) & 0x00000001)
			{
				q3 = 0x04C11DB7;
			}
			else
			{
				q3 = 0x00000000;
			}

			if (((crc >> 29) ^ (byte >> 2)) & 0x00000001)
			{
				q2 = 0x09823B6E;
			}
			else
			{
				q2 = 0x00000000;
			}

			if (((crc >> 30) ^ (byte >> 1)) & 0x00000001)
			{
				q1 = 0x130476DC;
			}
			else
			{
				q1 = 0x00000000;
			}

			if (((crc >> 31) ^ (byte >> 0)) & 0x00000001)
			{
				q0 = 0x2608EDB8;
			}
			else
			{
				q0 = 0x00000000;
			}

			crc = (crc << 4) ^ q3 ^ q2 ^ q1 ^ q0;
			byte >>= 4;
		}
	}

	return crc;
}
