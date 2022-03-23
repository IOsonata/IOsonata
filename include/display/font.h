/**-------------------------------------------------------------------------
@file	font.h

@brief	Generic display font definitions


@author	Hoang Nguyen Hoan
@date	Mar. 22, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#ifndef __FONT_H__
#define __FONT_H__

#include <stdint.h>

#define DISPL_FONT_ENCOD_VERTICAL		1	//!< Font encoding vertical
#define DISPL_FONT_ENCOD_FIXED			2	//!< Font type fixed

// NOTE: variable length font, first byte of character encoding is indicate the
// width in pixel of that character

#pragma pack(push,1)

typedef struct __Char_Desc {
	uint8_t Width;			//!< Char width in pixels
	uint8_t const *pBits;		//!< Offset in the font bitmap array
} CharDesc_t;

typedef struct __Display_Font {
	uint8_t Flag;			//!< Font map encoding horiz/vert, fix/var
	uint8_t Width;			//!< Width of biggest character in pixels
	uint8_t Height;			//!< Font height in pixels
	//uint8_t const *pBits; 	//!< Pointer to font bitmap of the variable length font, null for fixed font
	union {
		uint8_t Bits[1];	//!< Fixed font bitmap data
		CharDesc_t const *pCharDesc;	//!< Lockup table for variable length font bitmap
	};
} FontDesc_t;
#pragma pack(pop)



#endif // __FONT_H__
