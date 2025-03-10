/**-------------------------------------------------------------------------
@file	intelhex.h

@brief	Intel Hex parser.

@author	Hoang Nguyen Hoan
@date	Feb. 8, 2015

@license

MIT License

Copyright (c) 2015 I-SYST inc. All rights reserved.

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

#ifndef __INTELHEX_H__
#define __INTELHEX_H__

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/** @addtogroup Utilities
  * @{
  */

#define IHEX_RECTYPE_DATA		0
#define IHEX_RECTYPE_EOF		1		// End of file
#define IHEX_RECTYPE_EXTSEG		2		// Extended segment address
#define IHEX_RECTYPE_STARTSEG	3		// Start segment address
#define IHEX_RECTYPE_EXTLADDR	4		// Extended linear address
#define IHEX_RECTYPE_STARTLADDR	5		// Start linear address


#define IHEX_MAX_RECSIZE	16

/// Structure containing results of parsed HEX record line
typedef struct {
	int Count;		//!< Data count
	int Offset;		//!< Address offset
	int Type;		//!< Record type
	int Checksum;	//!< Record checksum
	uint8_t Data[IHEX_MAX_RECSIZE];
} IHEXDATA;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Parse Intel Hex record (one line).
 *
 * @param	pRec : Pointer to text line of intel hex record
 * @param	pData : Pointer to place holder for parsed record
 *
 * @return	true - Success\n
 * 			false - Bad in record data
 */
bool IHexParseRecord(char *pRec, IHEXDATA *pData);

	
#ifdef __cplusplus
}
#endif

/** @} End of group Utilities */

#endif // __INTELHEX_H__
