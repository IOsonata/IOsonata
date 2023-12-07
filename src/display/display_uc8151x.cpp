/**-------------------------------------------------------------------------
@file	display_uc8151x.cpp

@brief	UC8151x e-paper display controller implementation


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
#include "display/display_uc8151x.h"

bool EpdUC8151x::Init(const DisplayCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	EpdMatrix::Init(Cfg, pIntrf);

	SetDevRes(UC8151X_WIDTH_MAX, UC8151X_HEIGHT_MAX);
	uint8_t d = 0x1f;
	uint32_t res = Cfg.Width * Cfg.Height;
	uint8_t buff[10];// = {0x80, 0x01, 0x28, };

	SetFrameBuffer(vFrameBuff, (res * (uint32_t)Cfg.PixelSize) >> 3U);

	// Power ON command
	Cmd(UC8151X_CMD_PON, 0, 0);
	WaitBusy(100000);


	if (res <= 96*230)
	{
		d |= UC8151X_CMD_PSR_RES_96_230;
		buff[0] = 96;
		buff[1] = 230 >> 8;
		buff[2] = 230 & 0xFF;
	}
	else if (res <= 96*252)
	{
		d |= UC8151X_CMD_PSR_RES_96_252;
		buff[0] = 96;
		buff[1] = 252 >> 8;
		buff[2] = 252 & 0xFF;
	}
	else if (res <= 128*296)
	{
		d |= UC8151X_CMD_PSR_RES_128_296;
		buff[0] = 128;
		buff[1] = 296 >> 8;
		buff[2] = 296 & 0xFF;
	}
	else
	{
		d |= UC8151X_CMD_PSR_RES_160_296;
		buff[0] = 160;
		buff[1] = 296 >> 8;
		buff[2] = 296 & 0xFF;
	}

	Cmd(UC8151X_CMD_PSR, &d, 1);
	Cmd(UC8151X_CMD_TRES, buff, 3);			//resolution setting

	d = 0x97;
	Cmd(UC8151X_CMD_CDI, &d, 1);			//VCOM AND DATA INTERVAL SETTING

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool EpdUC8151x::Enable()
{
	// Power ON command
	Cmd(UC8151X_CMD_PON, 0, 0);
	WaitBusy(100000);

	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void EpdUC8151x::Disable()
{

}

/**
 * @brief	Clear screen
 *
 * @return	None
 */
/*
void EpdUC8151x::Clear()
{
	memset(vFrameBuff, 0, vFrameBuffSize);
	//Cmd(UC8151X_CMD_DTM1, vFrameBuff, vFrameBuffSize);
	//Cmd(UC8151X_CMD_DTM2, vFrameBuff, vFrameBuffSize);
	//Cmd(UC8151X_CMD_DRF, 0, 0);
	//WaitBusy(100000);
	Refresh();
}
*/
/**
 * @brief	Render a null terminated string to the screen.
 *
 * Display driver must implement this to emulate a console line print
 *
 * @param 	pStr	: Null terminated string to print
 * @param 	Color	: Color to render
 */
void EpdUC8151x::Print(char const *pStr, uint32_t Color)
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
void EpdUC8151x::Fill(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint32_t Color)
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
void EpdUC8151x::SetPixel(uint16_t X, uint16_t Y, uint32_t Color)
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
void EpdUC8151x::BitBlt(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint8_t *pMem)
{

}

void EpdUC8151x::Refresh()
{
	Cmd(UC8151X_CMD_DTM1, vFrameBuff, vFrameBuffSize);
	Cmd(UC8151X_CMD_DTM2, vFrameBuff, vFrameBuffSize);
	Cmd(UC8151X_CMD_DRF, 0, 0);
	WaitBusy(100000);
}
