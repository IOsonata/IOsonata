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
#include "display/ifont.h"

#define DISPL_CTRL_DCX_PINIDX			0		// Cmd/Data mode pin index
#define DISPL_CTRL_BKLIGHT_PINIDX		1		// External back light pin index
#define DISPL_CTRL_RST_PINIDX			2		// Reset pin index

typedef enum __Display_Type {
	DISPL_TYPE_ALPHA,				//!< Character based display type (non graphics)
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

/// Generic display base objec
class Display : public Device {
public:
	virtual bool Init(const DisplayCfg_t &, DeviceIntrf * const pIntrf) = 0;

	/**
	 * @brief	Clear screen
	 *
	 * @return	None
	 */
	virtual void Clear() = 0;

	/**
	 * @brief	Set current rendering location
	 *
	 * @param 	Col	:
	 * @param 	Row	:
	 */
	virtual void SetCursor(uint16_t Col, uint16_t Row) { vCurCol = Col; vCurLine = Row; }


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
	virtual void Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count) { (void)Dir; (void)Count;}

	/**
	 * @brief	print to console
	 *
	 * printf formated console style line display. This function will call
	 * Print function to render to the screen.  Display driver must implement Print
	 * function. Graphic display shall implement a way to emulate console print
	 *
	 * @param 	pFormat	: Use standard printf format
	 */
	virtual void printf(const char *pFormat, ...);

	/**
	 * @brief	Get display type
	 *
	 * @return	Display type :
	 * 				- DISPL_TYPE_ALPHA : Character based display
	 * 				- DISPL_TYPE_MAXTRIX : Graphic display
	 */
	DISPL_TYPE Type() { return vType; }

	/**
	 * @brief	Get physical width in display units
	 *
	 * Get the display width in display unit. Number characters per line for
	 * alphanumeric display.  Number of pixels for graphics display
	 *
	 * @return	Screen with
	 */
	virtual uint16_t Width() { return vWidth; }

	/**
	 * @brief	Get physical height in display units
	 *
	 * Get the display height in display unit. Number of line for
	 * alphanumeric display.  Number of pixels for graphics display
	 *
	 * @return	Screen height
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
	virtual void Reset() = 0;

protected:

	/**
	 * @brief	Render a null terminated string to the screen.
	 *
	 * Display driver must implement this to emulate a console line print
	 *
	 * @param 	pStr	: Null terminated string to print
	 * @param 	Color	: Color to render
	 */
	virtual void Print(char const *pStr, uint32_t Color) = 0;

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
	void Type(DISPL_TYPE Type) { vType = Type; }

	DisplayCfg_t vCfg;		//!< Internal copy of config data
	uint16_t vWidth;		//!< Display physical width in pixels
	uint16_t vHeight;		//!< Display physical height in pixels
	DISPL_TYPE vType;		//!< Display type
	uint32_t vColor;		//!< Current color
	uint16_t vCurLine;
	uint16_t vCurCol;
};

/// Dot matrix display object definition
class DisplayDotMatrix : public Display {
public:
	/**
	 * @brief	Reset display
	 *
	 */
	virtual void Reset();

	/**
	 * @brief	Set display orientation
	 *
	 * @param	Orient	: Orientation to set
	 *
	 * @return	None
	 */
	virtual void Orientation(DISPL_ORIENT Orient) { vOrient = Orient; }

	/**
	 * @brief	Get current orientation
	 *
	 * @return	Current orientation
	 */
	virtual DISPL_ORIENT Orientation() { return vOrient; }

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

	//virtual void Print(char *pStr, uint32_t Color);
	//virtual void printf(const char *pFormat, ...);

	/**
	 * @brief	Rotate display by 90 degree clockwise
	 */
	void Rotate90();

	virtual bool SetFont(FontDesc_t const *pFont);

	int PixelLength() { return vPixelLen; }

protected:
	uint8_t *vpLineBuff;
	int vPixelLen;
	uint16_t vCurScrollLine;	//!< Keep track of scrolling (rotation)
	uint16_t vLineHeight;
	FontDesc_t const *vpFont;	//!< Current font

private:
	DISPL_ORIENT vOrient;	//!< Current orientation

	/**
	 * @brief	Render a null terminated string to the screen.
	 *
	 * Display driver must implement this to emulate a console line print
	 *
	 * @param 	pStr	: Null terminated string to print
	 * @param 	Color	: Color to render
	 */
	virtual void Print(char const *pStr, uint32_t Color);
};

/// Alpha numeric display object definition
class DisplayAlphNum : public Display {
public:
	virtual void Print(char *pStr, uint32_t Color) { (void)pStr; (void)Color; }

protected:
private:
};

#endif // __DISPLAY_H__
