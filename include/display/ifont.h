/**-------------------------------------------------------------------------
@file	ifont.h

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
#ifndef __IFONT_H__
#define __IFONT_H__

#include <stdint.h>

#define DISPL_FONT_ENCOD_VERTICAL		(1<<0)	//!< Font encoding vertical
#define DISPL_FONT_ENCOD_FIXED			(1<<1)	//!< Font type fixed
#define DISPL_FONT_EXTRN				(1<<2)	//!< Font stored in external storage such as SPI Flash

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
	union {
		uint8_t const *pBits;		//!< Fixed font bitmap data
		CharDesc_t const *pCharDesc;//!< Lockup table for variable length font bitmap
	};
} FontDesc_t;
#pragma pack(pop)

extern const FontDesc_t iFontFreeMono8pt;
extern const FontDesc_t iFontFreeMono10pt;
extern const FontDesc_t iFontFreeMono12pt;
extern const FontDesc_t iFontFreeMono16pt;
extern const FontDesc_t iFontFreeMono24pt;
extern const FontDesc_t iFontFreeMonoBold8pt;
extern const FontDesc_t iFontFreeMonoBold10pt;
extern const FontDesc_t iFontFreeMonoBold12pt;
extern const FontDesc_t iFontFreeMonoBold16pt;
extern const FontDesc_t iFontFreeMonoBold24pt;
extern const FontDesc_t iFontFreeMonoIta8pt;
extern const FontDesc_t iFontFreeMonoIta10pt;
extern const FontDesc_t iFontFreeMonoIta12pt;
extern const FontDesc_t iFontFreeMonoIta16pt;
extern const FontDesc_t iFontFreeMonoIta24pt;
extern const FontDesc_t iFontFreeMonoBoldIta8pt;
extern const FontDesc_t iFontFreeMonoBoldIta10pt;
extern const FontDesc_t iFontFreeMonoBoldIta12pt;
extern const FontDesc_t iFontFreeMonoBoldIta16pt;
extern const FontDesc_t iFontFreeMonoBoldIta24pt;

extern const FontDesc_t iFontFreeSans8pt;
extern const FontDesc_t iFontFreeSans10pt;
extern const FontDesc_t iFontFreeSans12pt;
extern const FontDesc_t iFontFreeSans16pt;
extern const FontDesc_t iFontFreeSans24pt;
extern const FontDesc_t iFontFreeSerif8pt;
extern const FontDesc_t iFontFreeSerif10pt;
extern const FontDesc_t iFontFreeSerif12pt;
extern const FontDesc_t iFontFreeSerif16pt;
extern const FontDesc_t iFontFreeSerif24pt;
extern const FontDesc_t iFontSystem5x7;

#endif // __IFONT_H__
