/**-------------------------------------------------------------------------
@file	display.cpp

@brief	Generic display controller


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
#include "display/display.h"
#include "iopinctrl.h"
#include "display/ifont.h"

void Display::Backlight(bool bOn)
{
	if (vCfg.NbPins > DISPL_CTRL_DCX_PINIDX)
	{
		if (bOn)
		{
			IOPinSet(vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PinNo);
		}
		else
		{
			IOPinClear(vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PinNo);
		}
	}
}

void Display::printf(const char *pFormat, ...)
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

void DisplayDotMatrix::Reset()
{
	SetFont(&iFontSystem5x7);

	if (vCfg.NbPins > DISPL_CTRL_BKLIGHT_PINIDX)
	{
		IOPinClear(vCfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo);
		usDelay(100);
		IOPinSet(vCfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo);
	}
}

bool DisplayDotMatrix::SetFont(FontDesc_t const *pFont)
{
	if (pFont == nullptr || (pFont->Prop & FONT_TYPE_MASK) > 2 || (pFont->Prop & FONT_LOC_EXTERN))
	{
		return false;
	}

	vpFont = pFont;
	uint32_t h = (uint32_t)vHeight;

	uint32_t t = h / (uint32_t)(vpFont->Height + 1);
	vLineHeight = (h + (t>>1)) / t;
	vCurLine = ((vCurLine + (vLineHeight >> 1))/ vLineHeight) * vLineHeight;// + vLineHeight;
	if (vCurLine < vLineHeight)
	{
		vCurLine = vLineHeight;
	}

	return true;
}

void DisplayDotMatrix::Rotate90()
{
	int orient = vOrient == DISPL_ORIENT_LANDSCAPE_INV ? DISPL_ORIENT_PORTRAIT : vOrient + 1;

	Orientation((DISPL_ORIENT)orient);
}
#if 0
void DisplayDotMatrix::Print(char *pStr, uint32_t Color)
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
//				vCurCol += vpFont->Flag & DISPL_FONT_ENCOD_FIXED ? vpFont->Width + 1 : vpFont->Width >> 1;
				vCurCol += vpFont->Prop & FONT_TYPE_MASK ? vpFont->Width + 1 : vpFont->Width >> 1;
				break;
			default:
#if 1
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
#else
				if (vpFont->Flag & DISPL_FONT_ENCOD_FIXED)
				{
					// Fixed font
					if (vpFont->Flag & DISPL_FONT_ENCOD_VERTICAL)
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
					else
					{
						// Horizontal encode
					}
				}
				else
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
#endif
		}

		pStr++;

		if (vCurCol >= vWidth)
		{
			vCurCol = 0;
			vCurLine += vLineHeight;
		}

		if (vCurLine > vCurScrollLine)
		{
			Scroll(DISPL_SCROLL_DIR_UP, vLineHeight);
			Fill(0, vCurScrollLine - vpFont->Height - 1, vWidth, vLineHeight, 0);
			vCurLine = vCurScrollLine;
		}
	}
}

void DisplayDotMatrix::printf(const char *pFormat, ...)
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
#endif
