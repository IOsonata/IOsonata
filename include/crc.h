/**-------------------------------------------------------------------------
@file	crc.h

@brief	CRC calculations.

reference : http://en.wikipedia.org/wiki/Computation_of_CRC

@author Hoang Nguyen Hoan
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
#ifndef __CRC_H__
#define __CRC_H__

#include <stdint.h>

/** @addtogroup Utilities
  * @{
  */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Calculate 8 bits CRC value.
 *
 * Polynomial : (x7 + x3 + 1) × x (left-shifted CRC-7-CCITT)\n
 *              0x12 = (0x09 << 1) (MSBF/normal)
 *
 * @param   pData   : Pointer to data buffer to calculate
 * @param   Len     : Data length in bytes
 * @param   SeedVal : Initial CRC seed value
 *
 * @return	8 bits CRC value
 */
uint8_t crc8_ccitt(uint8_t *pData, int Len, uint8_t SeedVal);

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
uint8_t crc8(uint16_t Poly, uint8_t *pData, int Len, uint8_t SeedVal);

/**
 * @brief	Calculate 16 bits CRC value.
 *
 * Polynomial : x16 + x15 + x2 + 1 (CRC-16-ANSI/ARC)\n
 *          	0xA001 (reflected), RefIn=true, RefOut=true.\n
 * Nibble table-driven (16-entry, 32 bytes), 2 lookups per byte.\n
 * With SeedVal=0x0000: "123456789" → 0xBB3D (CRC-16/ARC).\n
 * With SeedVal=0xFFFF: "123456789" → 0x4B37 (CRC-16/MODBUS).
 *
 * @param   pData   : Pointer to data buffer to calculate
 * @param   Len     : Data length in bytes
 * @param   SeedVal : Initial CRC seed value
 *
 * @return	16 bits CRC value
 */
uint16_t crc16_ansi(uint8_t *pData, int Len, uint16_t SeedVal);

/**
 * @brief	Calculate 16 bits CRC value.
 *
 * Polynomial : x16 + x12 + x5 + 1 (CRC-16-CCITT)\n
 * 				0x1021 (MSBF/normal)
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * @param	Len		: Data length in bytes
 * @param	SeedVal : Initial CRC seed value
 *
 * @return	16 bits CRC value
 */
uint16_t crc16_ccitt(uint8_t *pData, int Len, uint16_t SeedVal);

/**
 * @brief	Calculate 32 bits CRC value.
 *
 * Polynomial : 0x04C11DB7 (MSBF/normal), init 0xFFFFFFFF, no final XOR.
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * @param	Len		: Data length in bytes
 *
 * @return	32 bits CRC value
 */
uint32_t crc32(uint8_t *pData, int Len);

/**
 * @brief	Calculate 32 bits CRC value, IEEE 802.3.
 *
 * Standard CRC-32 used in Ethernet, ZIP, PNG, gzip, etc.\n
 * Polynomial : 0xEDB88320 (reflected), init 0xFFFFFFFF, final XOR 0xFFFFFFFF.\n
 * Nibble table-driven (16-entry, 64 bytes), 2 lookups per byte.
 *
 * @param	pData 	: Pointer to data buffer to calculate
 * @param	Len		: Data length in bytes
 *
 * @return	32 bits CRC value
 */
uint32_t crc32_ieee(uint8_t *pData, int Len);

#ifdef __cplusplus
}
#endif

/** @} End of group Utilities */

#endif // __CRC_H__
