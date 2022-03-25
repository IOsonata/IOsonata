/**-------------------------------------------------------------------------
@file	display.h

@brief	Generic display controller definitions


@author	Hoang Nguyen Hoan
@date	Mar. 9, 2022

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
#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "device.h"
#include "iopinctrl.h"
#include "display/font.h"

#define DISPL_CTRL_DCX_PINIDX			0		// Cmd/Data mode pin index
#define DISPL_CTRL_BKLIGHT_PINIDX		1		// External back light pin index
#define DISPL_CTRL_RST_PINIDX			2		// Reset pin index

typedef enum __Display_Type {
	DISPL_TYPE_CHAR,				//!< Character based display type (non graphics)
	DISPL_TYPE_MATRIX,				//!< Dot matrix display (graphics)
} DISPL_TYPE;

typedef enum __Display_Orientation {
	DISPL_ORIENT_PORTRAIT,			//!< Portrait normal
	DISPL_ORIENT_LANDSCAPE,			//!< Landscape normal
	DISPL_ORIENT_PORTRAIT_INV,		//!< Portrait inverted
	DISPL_ORIENT_LANDSCAPE_INV		//!< Landscape inverted
} DISPL_ORIENT;

typedef enum __Display_Scroll_Direction {
	DISPL_SCROLL_DIR_UP,
	DISPL_SCROLL_DIR_DOWN,
	DISPL_SCROLL_DIR_LEFT,
	DISPL_SCROLL_DIR_RIGHT
} DISPL_SCROLL_DIR;

#define DISPL_FONT_ENCOD_VERTICAL		1	//!< Font encoding vertical
#define DISPL_FONT_ENCOD_FIXED			2	//!< Font type fixed

// NOTE: variable length font, first byte of character encoding is indicate the
// width in pixel of that character

#pragma pack(push,4)
typedef struct __Display_Cfg {
	uint32_t DevAddr;		//!< Device address (or SPI device CS index)
	IOPinCfg_t const *pPins;
	int NbPins;
	uint16_t Stride;		//!< Memory stride in bytes for one line
	uint16_t Width;			//!< Horizontal length in pixels
	uint16_t Height;		//!< Vertical length in pixels
	uint8_t PixelSize;		//!< Pixel size in bits/pixel
	DISPL_ORIENT Orient;	//!< Display orientation
} DisplayCfg_t;
#pragma pack(pop)

class Display : public Device {
public:
	virtual bool Init(DisplayCfg_t &, DeviceIntrf *pIntrf) = 0;

	/**
	 * @brief	Clear screen
	 *
	 * @return	None
	 */
	virtual void Clear() = 0;

	/**
	 * @brief	Set display orientation
	 *
	 * @param	Orient	: Orientation to set
	 *
	 * @return	None
	 */
	virtual void Orientation(DISPL_ORIENT Orient) = 0;

	/**
	 * @brief	Get current orientation
	 *
	 * @return	Current orientation
	 */
	virtual DISPL_ORIENT Orientation() { return vOrient; }

	/**
	 * @brief	Set current rendering location
	 *
	 * @param 	Col	:
	 * @param 	Row	:
	 */
	virtual void SetCurrent(uint16_t Col, uint16_t Row) { vCurCol = Col; vCurLine = Row; }

	/**
	 * @brief	Fill region with color
	 *
	 * @param	StartX 	: Region start X coordinate in pixel
	 * @param 	StartY 	: Region start Y coordinate in pixel
	 * @param 	Width	: Region width in pixel
	 * @param 	Height	: Region height in pixel
	 * @param 	Color	: Color to fill
	 *
	 * @return	None
	 */
	virtual void Fill(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint32_t Color) = 0;

	/**
	 * @brief	Set pixel color
	 *
	 * @param 	X		: Pixel X coordinate
	 * @param 	Y		: Pixel Y coordinate
	 * @param 	Color	: Color to set
	 *
	 * @return	None
	 */
	virtual void SetPixel(uint16_t X, uint16_t Y, uint32_t Color) = 0;

	/**
	 * @brief	Transfer local graphic memory to display region
	 *
	 * @param 	StartX	: Region start X in pixel
	 * @param 	StartY	: Region start Y in pixel
	 * @param 	Width	: Region width in pixel
	 * @param 	Height	: Region height in pixel
	 * @param 	pMem	: Pointer to local graphics memory
	 *
	 * @return	None
	 */
	virtual void BitBlt(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint8_t *pMem) = 0;

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
	virtual void Line(uint16_t StartX, uint16_t StartY, uint16_t EndX, uint16_t EndY, uint32_t Color) {}

	/**
	 * @brief	Display text string at location
	 *
	 * Print a zero terminated string to the screen using the current font
	 *
	 * @param 	Col		: X coordinate
	 * @param 	Row		: Y coordinate
	 * @param 	pStr	: Zero terminated string
	 */
	virtual void Text(uint16_t Col, uint16_t Row, char *pStr) = 0;

	/**
	 * @brief	Scroll display (optional)
	 *
	 * Scroll the screen in the specified direction by n steps.
	 * Step	:	1 char/line for character based display
	 * 		   	1 pixel for graphics based display
	 *
	 * @param 	Dir		: Direction of scroll
	 * @param 	Count	: Number of step to scroll
	 *
	 * @return	None
	 */
	virtual void Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count) {}
	virtual void printf(const char *pFormat, ...) = 0;

	virtual void SetFont(FontDesc_t const *pFont);
	/**
	 * @brief	Get physical width in pixel
	 *
	 * @return	Screen with in pixel
	 */
	virtual uint16_t Width() { return vWidth; }

	/**
	 * @brief	Get physical height in pixel
	 *
	 * @return	Screen height in pixel
	 */
	virtual uint16_t Height() { return vHeight; }

	/**
	 * @brief	Turn back light on or off
	 *
	 * @param 	bState : True - Turn back light on
	 * 					 False - Turn back light off
	 *
	 * @return	None
	 */
	virtual void Backlight(bool bState);

	/**
	 * @brief	Reset display
	 *
	 */
	virtual void Reset();

	/**
	 * @brief	Rotate display by 90 degree clockwise
	 */
	void Rotate90();

protected:

	/**
	 * @brief	Set device physical size
	 *
	 * This function sets the max resolutions supported by the controller. It
	 * may be bigger than the display panel resolutions
	 *
	 * @param 	Width 	: Screen width in pixels
	 * @param 	Height	: Screen height in pixels
	 *
	 * @return	None
	 */
	void SetDevRes(uint16_t Width, uint16_t Height) { vWidth = Width; vHeight = Height; }
	void SetDisplayType(DISPL_TYPE Type) { vType = Type; }

	DisplayCfg_t vCfg;		//!< Internal copy of config data
	uint16_t vWidth;		//!< Display physical width in pixels
	uint16_t vHeight;		//!< Display physical height in pixels
	DISPL_ORIENT vOrient;	//!< Current orientation
	DISPL_TYPE vType;		//!< Display type
	uint32_t vColor;		//!< Current color
	FontDesc_t const *vpFont;	//!< Current font
	uint16_t vCurLine;
	uint16_t vCurCol;
	uint16_t vLineHeight;
};


#endif // __DISPLAY_H__
