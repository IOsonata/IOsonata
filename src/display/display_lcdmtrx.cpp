/**-------------------------------------------------------------------------
@file	display_lcdmtrx.cpp

@brief	Generic LCD matrix display controller implementation


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
#include <math.h>
#include <stdarg.h>

#include "istddef.h"
#include "idelay.h"
#include "convutil.h"
#include "iopinctrl.h"
#include "display/display_lcdmtrx.h"

bool LCDMatrix::Init(const DisplayCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	Interface(pIntrf);

	vCfg = Cfg;

	DeviceAddress(vCfg.DevAddr);
	Type(DISPL_TYPE_MATRIX);

	IOPinCfg(vCfg.pPins, vCfg.NbPins);

	Reset();

	uint32_t d;
	uint8_t cmd = LCDMTRX_CMD_SLEEPOUT;
	Write(&cmd, 1, NULL, 0);

	vMadCtl = LCDMTRX_CMD_MADCTL_MX;

	cmd = LCDMTRX_CMD_COLMOD;
	switch (vCfg.PixelSize)
	{
		case 12:
#if 0
			d = ST77XX_CMD_COLMOD_COLOR_FMT_12 | ST77XX_CMD_COLMOD_RGB_INTRF_65K;
			vPixelLen = 2;
			ramctrl |= ST77XX_CMD_RAMCTRL_ENDIAN_LITTLE;
			madctl |= ST77XX_CMD_MADCTL_BGR;
			break;
#endif
			// 12 bit format is inefficient
			// Default it to 16bits instead.
			vCfg.PixelSize = 16;
		case 16:
			vMadCtl |= LCDMTRX_CMD_MADCTL_BGR;
			d = LCDMTRX_CMD_COLMOD_COLOR_FMT_16 | LCDMTRX_CMD_COLMOD_RGB_INTRF_65K;
			vPixelLen = 2;
			break;
		case 18:
			d = LCDMTRX_CMD_COLMOD_COLOR_FMT_18 | LCDMTRX_CMD_COLMOD_RGB_INTRF_262K;
			vPixelLen = 3;
			break;
		case 24:
			d = LCDMTRX_CMD_COLMOD_COLOR_FMT_24 | LCDMTRX_CMD_COLMOD_RGB_INTRF_16M;
			vPixelLen = 3;
			break;
	}
	Write(&cmd, 1, (uint8_t*)&d, 1);

	cmd = LCDMTRX_CMD_NORMALON;
	Write(&cmd, 1, NULL, 0);

	cmd = LCDMTRX_CMD_DISPON;
	Write(&cmd, 1, NULL, 0);

	cmd = LCDMTRX_CMD_VSCROLLDEF;
	uint16_t dat[3] = {
		0, EndianCvt16(vHeight), 0
	};
	Write(&cmd, 1, (uint8_t*)dat, 6);

	vCurScrollLine = Height();
	vCurLine = vLineHeight;
	vCurCol = 0;

	Orientation(vCfg.Orient);

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool LCDMatrix::Enable()
{
	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void LCDMatrix::Disable()
{

}

/**
 * @brief	Reset device to it initial default state
 */
void LCDMatrix::Reset()
{
	if (vCfg.NbPins > DISPL_CTRL_BKLIGHT_PINIDX)
	{
		// Reset controllable, try hard reset
		DisplayDotMatrix::Reset();
	}
	else
	{
		// Soft reset
		uint8_t cmd = LCDMTRX_CMD_SWRESET;
		Write(&cmd, 1, NULL, 0);
	}
	msDelay(200);
}

void LCDMatrix::Orientation(DISPL_ORIENT Orient)
{
	uint16_t w = Width();
	uint16_t h = Height();

	vMadCtl &= LCDMTRX_CMD_MADCTL_BGR;

	switch (Orient)
	{
		case DISPL_ORIENT_PORTRAIT:
			if (w > h)
			{
				SetDevRes(h, w);
			}

			vMadCtl |= LCDMTRX_CMD_MADCTL_MX;
			break;
		case DISPL_ORIENT_PORTRAIT_INV:
			if (w > h)
			{
				SetDevRes(h, w);
			}
			vMadCtl |= LCDMTRX_CMD_MADCTL_MH | LCDMTRX_CMD_MADCTL_MY;
			break;
		case DISPL_ORIENT_LANDSCAPE:
			if (w < h)
			{
				SetDevRes(h, w);
			}
			vMadCtl |= LCDMTRX_CMD_MADCTL_MV;
			break;
		case DISPL_ORIENT_LANDSCAPE_INV:
			if (w < h)
			{
				SetDevRes(h, w);
			}
			vMadCtl |= LCDMTRX_CMD_MADCTL_MV | LCDMTRX_CMD_MADCTL_MY | LCDMTRX_CMD_MADCTL_MX;
			break;
	}
	DisplayDotMatrix::Orientation(Orient);

	uint8_t cmd = LCDMTRX_CMD_MADCTL;
	Write(&cmd, 1, (uint8_t*)&vMadCtl, 1);
}

void LCDMatrix::Clear()
{
	uint16_t w = Width();
	uint16_t h = Height();
	int line = w * vPixelLen;

	memset(vpLineBuff, 0, line);
#if 0
	uint16_t *p = (uint16_t*)vpLineBuff;

	for (int i = 0; i < vCfg.HLen; i++)
	{
		p[i] = EndianCvt16(0xf800);
	}
#endif
#if 1
	SetRamWrRegion(0, 0, w, h);

	uint8_t cmd = LCDMTRX_CMD_RAMWR;
	Write(&cmd, 1, vpLineBuff, line);

	for (int i = 1; i < h; i++)
	{
		Write(0, 0, vpLineBuff, line);
	}
#else
	// test pixel/pixel write
	uint32_t d = (0xf800);
	uint8_t cmd = ST77XX_CMD_RAMWR;

	for (int j = 0; j < h; j++)
	{
		for (int i = 0; i < w; i++)
		{
			SetRamWrRegion(i, j, 1, 1);
			Write(&cmd, 1, (uint8_t*)&d, 2);
		}
	}
#endif

	vCurScrollLine = h;
	vCurLine = vLineHeight;
	vCurCol = 0;

	cmd = LCDMTRX_CMD_VSCROLL_START_ADDR;
	uint16_t d = EndianCvt16(0);
	Write(&cmd, 1, (uint8_t*)&d, 2);
}

void LCDMatrix::Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color)
{
	SetRamWrRegion(X, Y, Width, Height);
	uint8_t cmd = LCDMTRX_CMD_RAMWR;
	uint32_t l = Width * vPixelLen;

	switch (vPixelLen)
	{
		case 2:
			{
				uint16_t *p = (uint16_t*)vpLineBuff;
				for (int i = 0; i < Width; i++)
				{
					p[i] = (uint16_t)(Color & 0xFFFFU);
				}
			}
			break;
		case 3:
			for (int i = 0; i < Width * 3; i+=3)
			{
				vpLineBuff[i] = (uint8_t)(Color & 0xFFU);
				vpLineBuff[i + 1] = (uint8_t)((Color >> 8) & 0xFFU);
				vpLineBuff[i + 2] = (uint8_t)((Color >> 16) & 0xFFU);
			}
			break;
		default:
			return;
	}

	Write(&cmd, 1, vpLineBuff, Width * vPixelLen);
	for (int i = 1; i < Height; i++)
	{
		Write(0, 0, vpLineBuff, Width * vPixelLen);
	}
}

void LCDMatrix::SetPixel(uint16_t X, uint16_t Y, uint32_t Color)
{
	SetRamWrRegion(X, Y, 1, 1);
	uint8_t cmd = LCDMTRX_CMD_RAMWR;
	Write(&cmd, 1, (uint8_t*)&Color, vPixelLen);
}

void LCDMatrix::BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pBuffer)
{
	SetRamWrRegion(X, Y, Width, Height);
	uint8_t cmd = LCDMTRX_CMD_RAMWR;
	//uint32_t l = Width * Height * vPixelLen;
	Write(&cmd, 1, pBuffer, Width * Height * vPixelLen);
}

#if 0
/**
 * @brief	Draw line on the screen
 *
 * Function to draw a line on matrix display
 *
 * @param 	StartX	: Start X coordinate
 * @param 	StartY	: Start Y coordinate
 * @param 	EndX	: End X coordinate
 * @param 	EndY	: End Y coordinate
 * @param	Color	: Pixel color
 */
void LCDMatrix::Line(uint16_t StartX, uint16_t StartY, uint16_t EndX, uint16_t EndY, uint32_t Color)
{
	int dx = abs(EndX - StartX);
	int sx = StartX < EndX ? 1 : -1;
	int dy = -abs(EndY - StartY);
	int sy = StartY < EndY ? 1 : -1;
	int error = dx + dy;

	if (StartY == EndY)
	{
		Fill(StartX < EndX ? StartX : EndX, StartY, dx + 1, 1, Color);
		return;
	}

	SetPixel(StartX, StartY, Color);

	while (StartX != EndX || StartY != EndY)
	{
		int e2 = error << 1;
		if (e2 >= dy)
		{
			if (StartX == EndX)
				break;
			error = error + dy;
			StartX += sx;
		}
		if (e2 <= dx)
		{
			if (StartY == EndY)
				break;
			error = error + dx;
			StartY += sy;
		}

		SetPixel(StartX, StartY, Color);
	}
}


/**
 * @brief	Display text string at location
 *
 * Print a zero terminated string to the screen using the current font
 *
 * @param 	Col		: X coordinate
 * @param 	Row		: Y coordinate
 * @param 	pStr	: Zero terminated string
 */
void LCDMatrix::Text(uint16_t Col, uint16_t Row, char *pStr)
{

}
#endif

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
void LCDMatrix::Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count)
{
	switch (Dir)
	{
		case DISPL_SCROLL_DIR_UP:
			vCurScrollLine += Count;
			if (vCurScrollLine > vHeight)
			{
				vCurScrollLine = Count;
			}
			break;
		case DISPL_SCROLL_DIR_DOWN:
			vCurScrollLine -= Count;
			if (vCurScrollLine >= vHeight)
			{
				vCurScrollLine = vHeight - Count - 1;
			}
			break;
		default:
			;
	}

	uint8_t cmd = LCDMTRX_CMD_VSCROLL_START_ADDR;
	uint16_t d = EndianCvt16(vCurScrollLine);
	Write(&cmd, 1, (uint8_t*)&d, 2);
}

void LCDMatrix::Print(char const *pStr, uint32_t Color)
{
	uint8_t buff[vpFont->Height * vPixelLen * vpFont->Width];
	//uint8_t const *fp = vpFont->Bits;

	//uint16_t x = 0;
	//uint16_t y = vCurScrollLine + vpFont->Height + 10;

	while (*pStr != 0)
	{
		switch (*pStr)
		{
			case '\n':
				vCurLine += vLineHeight;
				break;
			case '\r':
				vCurCol = 0;
				break;
			case ' ':
				vCurCol += vpFont->Prop & FONT_TYPE_MASK ? vpFont->Width + 1 : vpFont->Width >> 1;
				break;
			default:
				switch (vpFont->Prop)
				{
					case FONT_TYPE_VAR_WIDTH:
						{
							// Variable length font
							uint16_t *p = (uint16_t*)buff;
							int l = *pStr - '!';
							uint8_t const *fp = vpFont->pCharDesc[l].pBits;
							int cw = vpFont->pCharDesc[l].Width;

							for (int i = 0; i < vpFont->Height; i++)
							{
								int w = cw;
								while (w > 0)
								{
									if (*fp == 0)
									{
										int n = min(w, 8);
										memset(p, 0, n * vPixelLen);
										p += n;
										w -= n;
									}
									else
									{
										uint32_t mask = 0x80UL;

										while (mask != 0 && w > 0)
										{
											*p = *fp & mask ? Color : 0;
											p++;
											w--;
											mask >>= 1UL;
										}

									}
									fp++;
								}
							}
							BitBlt(vCurCol, vCurLine - vpFont->Height, cw, vpFont->Height, buff);
							vCurCol += cw + 2;
						}
						break;
					case FONT_TYPE_FIXED_HOR:
						break;
					case FONT_TYPE_FIXED_VERT:
					{
						// Vertical encoded
						uint16_t *p = (uint16_t*)buff;
						int l = vpFont->Width * ((vpFont->Height>>3) + 1) * (*pStr - '!');
						uint8_t const *fp = &vpFont->pBits[l];
						uint32_t mask = 1;
						while (mask <= (1UL<<(vpFont->Height - 1UL)))
						{
							for (int i = 0; i < vpFont->Width; i++, p++)
							{
								*p = fp[i] & mask ? Color : 0;
							}
							mask <<= 1;
						}
						BitBlt(vCurCol, vCurLine - vpFont->Height, vpFont->Width, vpFont->Height, buff);
						vCurCol += vpFont->Width + 1;
					}
				}
		}

		pStr++;

		if (vCurCol >= vWidth)
		{
			vCurCol = 0;
			vCurLine += vLineHeight;
		}

		if (vCurLine > vCurScrollLine)
		{
			if (Orientation() == DISPL_ORIENT_PORTRAIT)
			{
				Scroll(DISPL_SCROLL_DIR_UP, vLineHeight);
				Fill(0, vCurScrollLine - vpFont->Height - 1, vWidth, vLineHeight, 0);
				vCurLine = vCurScrollLine;
			}
			else
			{
				Clear();
			}
		}
	}
}

void LCDMatrix::SetRamWrRegion(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height)
{
	uint8_t cmd = LCDMTRX_CMD_CASET;
	uint32_t d = (EndianCvt16(X) & 0xFFFF) | (EndianCvt16(X + Width - 1) << 16);
	Write(&cmd, 1, (uint8_t*)&d, 4);

	cmd = LCDMTRX_CMD_RASET;
	d = (EndianCvt16(Y) & 0xFFFF) | (EndianCvt16(Y + Height - 1) << 16);
	Write(&cmd, 1, (uint8_t*)&d, 4);
}

/**
 * @brief	Read device's register/memory block.
 *
 * This default implementation sets bit 7 of the Cmd/Addr byte for SPI read access as most
 * devices work this way on SPI interface. Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior reading data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pBuff		: Data buffer container
 * @param	BuffLen		: Data buffer size
 *
 * @return	Actual number of bytes read
 */
int LCDMatrix::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int cnt = 0;
	vpIntrf->StartTx(DeviceAddress());

	if (pCmdAddr && CmdAddrLen > 0)
	{
		IOPinClear(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);

		cnt += vpIntrf->TxData(pCmdAddr, CmdAddrLen);
	}
	if (pBuff != NULL && BuffLen > 0)
	{
		IOPinSet(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);
		cnt += vpIntrf->RxData(pBuff, BuffLen);
	}
	vpIntrf->StopTx();

	return cnt;
}

/**
 * @brief	Write to device's register/memory block
 *
 * This default implementation clears bit 7 of the Cmd/Addr byte for SPI write access as most
 * devices work this way on SPI interface.  Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior writing data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pData		: Data buffer to be written to the device
 * @param	DataLen		: Size of data
 *
 * @return	Actual number of bytes written
 */
int LCDMatrix::Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen)
{
	int cnt = 0;
	vpIntrf->StartTx(DeviceAddress());

	if (pCmdAddr && CmdAddrLen > 0)
	{
		IOPinClear(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);

		cnt += vpIntrf->TxData(pCmdAddr, CmdAddrLen);
	}
	if (pData != NULL && DataLen > 0)
	{
		IOPinSet(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);
		cnt += vpIntrf->TxData((uint8_t*)pData, DataLen);
	}
	vpIntrf->StopTx();

	return cnt;
}
