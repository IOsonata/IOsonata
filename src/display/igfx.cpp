/**-------------------------------------------------------------------------
@file	igfx.cpp

@brief	Graphics primitives implementation


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
#include <math.h>
#include <stdarg.h>

#include "istddef.h"
#include "display/igfx.h"

bool iGfx::Init(DisplayDotMatrix *pDev)
{
	if (pDev == NULL)
	{
		return false;
	}

	if (pDev->Type() != DISPL_TYPE_MATRIX)
	{
		return false;
	}

	vpDev = pDev;
	vCurScrollLine = vpDev->Height();

	return true;
}

/**
 * @brief	Reset device to it initial default state
 */
void iGfx::Reset()
{
	vpDev->Reset();
	SetFont(&iFontSystem5x7);
}

/**
 * @brief	Clear screen
 *
 * @return	None
 */
void iGfx::Clear()
{
	vpDev->Clear();
}

void iGfx::SetFont(FontDesc_t const *pFont)
{
	vpFont = pFont;
	uint32_t h = (uint32_t)vpDev->Height();

	uint32_t t = h / (uint32_t)(vpFont->Height + 1);
	vLineHeight = (h + (t>>1)) / t;
	vCurLine = ((vCurLine + (vLineHeight >> 1))/ vLineHeight) * vLineHeight;// + vLineHeight;
	if (vCurLine < vLineHeight)
	{
		vCurLine = vLineHeight;
	}
}

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
void iGfx::Line(uint16_t StartX, uint16_t StartY, uint16_t EndX, uint16_t EndY, uint32_t Color)
{
	int dx = abs(EndX - StartX);
	int sx = StartX < EndX ? 1 : -1;
	int dy = -abs(EndY - StartY);
	int sy = StartY < EndY ? 1 : -1;
	int error = dx + dy;

	if (StartY == EndY)
	{
		vpDev->Fill(StartX < EndX ? StartX : EndX, StartY, dx + 1, 1, Color);
		return;
	}

	vpDev->SetPixel(StartX, StartY, Color);

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

		vpDev->SetPixel(StartX, StartY, Color);
	}
}

void iGfx::Text(uint16_t Col, uint16_t Row, char *pStr)
{

}

void iGfx::Print(char *pStr, uint32_t Color)
{
	uint8_t buff[vpFont->Height * vpDev->PixelLength() * vpFont->Width];
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
										memset(p, 0, n * vpDev->PixelLength());
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
							vpDev->BitBlt(vCurCol, vCurLine - vpFont->Height, cw, vpFont->Height, buff);
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
						vpDev->BitBlt(vCurCol, vCurLine - vpFont->Height, vpFont->Width, vpFont->Height, buff);
						vCurCol += vpFont->Width + 1;
					}
				}
		}

		pStr++;

		if (vCurCol >= vpDev->Width())
		{
			vCurCol = 0;
			vCurLine += vLineHeight;
		}

		if (vCurLine > vCurScrollLine)
		{
			vpDev->Scroll(DISPL_SCROLL_DIR_UP, vLineHeight);
			vpDev->Fill(0, vCurScrollLine - vpFont->Height - 1, vpDev->Width(), vLineHeight, 0);
			vCurLine = vCurScrollLine;
		}
	}
}
/*
void iGfx::printf(const char *pFormat, ...)
{
#ifndef SPRT_BUFFER_SIZE
#define SPRT_BUFFER_SIZE	80
#endif

	char buff[SPRT_BUFFER_SIZE];

	va_list vl;
    va_start(vl, pFormat);
    vsnprintf(buff, sizeof(buff), pFormat, vl);
    buff[SPRT_BUFFER_SIZE - 1] = '\0';
    va_end(vl);

    Print(buff, -1);
}
*/
