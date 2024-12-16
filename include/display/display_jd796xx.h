/**-------------------------------------------------------------------------
@file	display_jd796xx.h

@brief	JD796XX EInk display controller implementation


@author	Hoang Nguyen Hoan
@date	Apr. 24, 2022

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
#ifndef __DISPLAY_JD796XX_H__
#define __DISPLAY_JD796XX_H__

#include "display/display_einkmtrx.h"

// JD79651 - 176x296, SPI, 1.8V32V
#define JD79651_WIDTH_MAX					176
#define JD79651_HEIGHT_MAX					296

// JD79656 - 128x250, SPI, 1.8V32V
#define JD79656_WIDTH_MAX					128
#define JD79656_HEIGHT_MAX					250

// JD79653 - 200x200, SPI, 1.8V32V
#define JD79653_WIDTH_MAX					200
#define JD79653_HEIGHT_MAX					200

// JD79657 - 200x384, SPI, 1.8V32V
#define JD79657_WIDTH_MAX					200
#define JD79657_HEIGHT_MAX					384

// JD79686 - 800x600, SPI, 1.8V32V
#define JD79686_WIDTH_MAX					800
#define JD79686_HEIGHT_MAX					600

// EK79652 - 320x300, SPI, 1.8V32V
#define EK79652_WIDTH_MAX					320
#define EK79652_HEIGHT_MAX					300


#define JD79651_LINE_BUFFLEN					(JD79651_HEIGHT_MAX)
#define JD79656_LINE_BUFFLEN					(JD79656_HEIGHT_MAX)
#define JD79653_LINE_BUFFLEN					(JD79653_HEIGHT_MAX)
#define JD79657_LINE_BUFFLEN					(JD79657_HEIGHT_MAX)
#define JD79686_LINE_BUFFLEN					(JD79686_WIDTH_MAX)
#define EK79652_LINE_BUFFLEN					(EK79652_WIDTH_MAX)

class EInkJD796xx : public EInkMatrix {
public:
	bool Init(DisplayCfg_t &, DeviceIntrf *pIntrf);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * This function is used to put the device in lowest power mode
	 * possible so that the Enable function can wake up without full
	 * initialization.
	 */
	virtual void Disable();

	/**
	 * @brief	Clear screen
	 *
	 * @return	None
	 */
	virtual void Clear();

	/**
	 * @brief	Render a null terminated string to the screen.
	 *
	 * Display driver must implement this to emulate a console line print
	 *
	 * @param 	pStr	: Null terminated string to print
	 * @param 	Color	: Color to render
	 */
	virtual void Print(char const *pStr, uint32_t Color);

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
	virtual void Fill(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint32_t Color);

	/**
	 * @brief	Set pixel color
	 *
	 * @param 	X		: Pixel X coordinate
	 * @param 	Y		: Pixel Y coordinate
	 * @param 	Color	: Color to set
	 *
	 * @return	None
	 */
	virtual void SetPixel(uint16_t X, uint16_t Y, uint32_t Color);

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
	virtual void BitBlt(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint8_t *pMem);

private:
	uint8_t vLineBuff[JD79686_WIDTH_MAX];
};


#endif // __DISPLAY_JD796XX_H__
