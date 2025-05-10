/**-------------------------------------------------------------------------
@file	igfx.h

@brief	Graphics primitives 


@author	Hoang Nguyen Hoan
@date	Mar. 25, 2022

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
#ifndef __IGFX_H__
#define __IGFX_H__

#include "display/display.h"

class iGfx {
public:

	virtual ~iGfx() {}

	bool Init(DisplayDotMatrix *pDev);

	/**
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset();

	/**
	 * @brief	Clear screen
	 *
	 * @return	None
	 */
	virtual void Clear();

	/**
	 * @brief	Set pixel color
	 *
	 * @param 	X		: Pixel X coordinate
	 * @param 	Y		: Pixel Y coordinate
	 * @param 	Color	: Color to set
	 *
	 * @return	None
	 */
	virtual void SetPixel(uint16_t X, uint16_t Y, uint32_t Color) { vpDev->SetPixel(X, Y, Color); }

	/**
	 * @brief	Draw line on the screen
	 *
	 * Option function to draw a line on matrix display
	 *
	 * @param 	StartX	: Start X coordinate
	 * @param 	StartY	: Start Y coordinate
	 * @param 	EndX	: End X coordinate
	 * @param 	EndY	: End Y coordinate
	 * @param	Color	: Pixel color
	 */
	virtual void Line(uint16_t StartX, uint16_t StartY, uint16_t EndX, uint16_t EndY, uint32_t Color);

	/**
	 * @brief	Display text string at location
	 *
	 * Print a zero terminated string to the screen using the current font
	 *
	 * @param 	Col		: X coordinate
	 * @param 	Row		: Y coordinate
	 * @param 	pStr	: Zero terminated string
	 */
	virtual void Text(uint16_t Col, uint16_t Row, char *pStr);

	virtual void SetFont(FontDesc_t const *pFont);

	virtual void Print(char *pStr, uint32_t Color);
//	virtual void printf(const char *pFormat, ...);

private:
	DisplayDotMatrix *vpDev;
	FontDesc_t const *vpFont;	//!< Current font
	uint16_t vLineHeight;
	uint16_t vCurLine;
	uint16_t vCurCol;
	uint16_t vCurScrollLine;	//!< Keep track of scrolling (rotation)
};

#endif // __IGFX_H__
