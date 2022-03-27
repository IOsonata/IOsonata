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

// Definition of font property
#define FONT_TYPE_MASK				(3<<0)
#define FONT_TYPE_VAR_WIDTH			(0<<0)	//!< Variable width bitmap
#define FONT_TYPE_FIXED_HOR			(1<<0)	//!< Fixed size bitmap horizontal coding
#define FONT_TYPE_FIXED_VERT		(2<<0)	//!< Fixed size bitmap vertical coding
#define FONT_TYPE_GLYTH				(3<<0)	//!< Glyth font
#define FONT_LOC_EXTERN				(1<<7)	//!< Font stored in external storage such as SPI Flash

#pragma pack(push,1)

typedef struct __Char_Desc {
	uint8_t Width;			//!< Char width in pixels
	uint8_t const *pBits;	//!< Offset in the font bitmap array
} CharDesc_t;

typedef struct __Font_Descriptor {
	uint8_t Prop;			//!< Font property see defines abouve
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
extern const FontDesc_t iFontFreeSansIta8pt;
extern const FontDesc_t iFontFreeSansIta10pt;
extern const FontDesc_t iFontFreeSansIta12pt;
extern const FontDesc_t iFontFreeSansIta16pt;
extern const FontDesc_t iFontFreeSansIta24pt;
extern const FontDesc_t iFontFreeSansBold8pt;
extern const FontDesc_t iFontFreeSansBold10pt;
extern const FontDesc_t iFontFreeSansBold12pt;
extern const FontDesc_t iFontFreeSansBold16pt;
extern const FontDesc_t iFontFreeSansBold24pt;
extern const FontDesc_t iFontFreeSansBoldIta8pt;
extern const FontDesc_t iFontFreeSansBoldIta10pt;
extern const FontDesc_t iFontFreeSansBoldIta12pt;
extern const FontDesc_t iFontFreeSansBoldIta16pt;
extern const FontDesc_t iFontFreeSansBoldIta24pt;

extern const FontDesc_t iFontFreeSerif8pt;
extern const FontDesc_t iFontFreeSerif10pt;
extern const FontDesc_t iFontFreeSerif12pt;
extern const FontDesc_t iFontFreeSerif16pt;
extern const FontDesc_t iFontFreeSerif24pt;
extern const FontDesc_t iFontFreeSerifIta8pt;
extern const FontDesc_t iFontFreeSerifIta10pt;
extern const FontDesc_t iFontFreeSerifIta12pt;
extern const FontDesc_t iFontFreeSerifIta16pt;
extern const FontDesc_t iFontFreeSerifIta24pt;
extern const FontDesc_t iFontFreeSerifBold8pt;
extern const FontDesc_t iFontFreeSerifBold10pt;
extern const FontDesc_t iFontFreeSerifBold12pt;
extern const FontDesc_t iFontFreeSerifBold16pt;
extern const FontDesc_t iFontFreeSerifBold24pt;
extern const FontDesc_t iFontFreeSerifBoldIta8pt;
extern const FontDesc_t iFontFreeSerifBoldIta10pt;
extern const FontDesc_t iFontFreeSerifBoldIta12pt;
extern const FontDesc_t iFontFreeSerifBoldIta16pt;
extern const FontDesc_t iFontFreeSerifBoldIta24pt;

extern const FontDesc_t iFontSystem5x7;

#endif // __IFONT_H__
