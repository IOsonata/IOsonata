/**-------------------------------------------------------------------------
@file	display_einkmtrx.cpp

@brief	Generic EInk matrix display controller implementation


@author	Hoang Nguyen Hoan
@date	Apr. 23, 2022

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
#include "display/display_einkmtrx.h"

bool EInkMatrix::Init(DisplayCfg_t &Cfg, DeviceIntrf *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	Interface(pIntrf);

	vCfg = Cfg;

	DeviceAddress(vCfg.DevAddr);
	Type(DISPL_TYPE_MATRIX);

	IOPinCfg(vCfg.pPins, vCfg.NbPins);

	Reset();

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool Enable()
{
	return false;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void Disable()
{

}

/**
 * @brief	Clear screen
 *
 * @return	None
 */
void Clear()
{

}

/**
 * @brief	Render a null terminated string to the screen.
 *
 * Display driver must implement this to emulate a console line print
 *
 * @param 	pStr	: Null terminated string to print
 * @param 	Color	: Color to render
 */
void Print(char const *pStr, uint32_t Color)
{

}

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
void Fill(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint32_t Color)
{

}

/**
 * @brief	Set pixel color
 *
 * @param 	X		: Pixel X coordinate
 * @param 	Y		: Pixel Y coordinate
 * @param 	Color	: Color to set
 *
 * @return	None
 */
void SetPixel(uint16_t X, uint16_t Y, uint32_t Color)
{

}

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
void BitBlt(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint8_t *pMem)
{

}

