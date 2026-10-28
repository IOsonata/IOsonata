/**-------------------------------------------------------------------------
@file	lcd_lpc546xx.cpp

@brief	LPC546xx LCD controller, TFT panel on a frame buffer

See lcd_lpc546xx.h.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "LPC546xx.h"

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "display/ifont.h"
#include "lcd_lpc546xx.h"

#define LCD_LPC546XX_CLKSEL_MAIN	0U			// LCDCLKSEL main clock
#define LCD_LPC546XX_CLKSEL_NONE	3U
#define LCD_LPC546XX_BPP_RGB565		6U			// CTRL LCDBPP 16 bpp 5:6:5
#define LCD_LPC546XX_PCD_MAX		1023U		// POL PCD, 10 bits: divider 2 to 1025
#define LCD_LPC546XX_AHBCLK_IDX		2			// AHBCLKCTRL2 and PRESETCTRL2

LcdLpc546xx::LcdLpc546xx() : vpFb(NULL), vPanelW(0), vPanelH(0), vPixelClock(0)
{
	memset(&vCfg, 0, sizeof(vCfg));
	vpLineBuff = NULL;
	vPixelLen = 2;
	vpFont = NULL;
	vCurScrollLine = 0;
	vLineHeight = 0;
	vCurLine = 0;
	vCurCol = 0;
	vWidth = 0;
	vHeight = 0;
	vColor = 0xFFFF;
}

// A display control pin the board wires, by its index in the pin map
bool LcdLpc546xx::CtrlPin(int Idx) const
{
	return vCfg.NbPins > Idx && vCfg.pPins[Idx].PortNo >= 0 && vCfg.pPins[Idx].PinNo >= 0;
}

bool LcdLpc546xx::Init(const DisplayCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	(void)pIntrf;

	const DisplayTiming_t *t = Cfg.pTiming;

	if ((Cfg.Width & 15U) != 0U || Cfg.Width < 16U || Cfg.Width > 1024U || Cfg.Height < 1U || Cfg.Height > 1024U ||
		Cfg.PixelSize != 16U || Cfg.Orient > DISPL_ORIENT_LANDSCAPE_INV || t == NULL ||
		t->PixelClock == 0U || t->HSync < 1U || t->HSync > 256U || t->HFront < 1U || t->HFront > 256U ||
		t->HBack < 1U || t->HBack > 256U || t->VSync < 1U || t->VSync > 64U || t->VFront < 1U ||
		t->VFront > 256U || t->VBack < 1U || t->VBack > 256U || Cfg.pFrameBuffer == NULL ||
		((uintptr_t)Cfg.pFrameBuffer & 7U) != 0U || (Cfg.NbPins > 0 && Cfg.pPins == NULL) || Cfg.NbPins < 0)
	{
		return false;
	}

	if (vpFb != NULL)
	{
		PowerOff();
	}

	vCfg = Cfg;
	vpFb = (uint16_t *)Cfg.pFrameBuffer;
	vPanelW = Cfg.Width;
	vPanelH = Cfg.Height;
	Type(DISPL_TYPE_MATRIX);
	vPixelLen = 2;

	// The pins, the backlight off. Reset, at the end, pulses the panel reset.
	IOPinCfg(Cfg.pPins, Cfg.NbPins);
	if (CtrlPin(DISPL_CTRL_BKLIGHT_PINIDX))
	{
		IOPinClear(Cfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PortNo, Cfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PinNo);
	}

	// LCD clock: the main clock, not divided, as the NXP SDK examples
	SYSCON->LCDCLKSEL = SYSCON_LCDCLKSEL_SEL(LCD_LPC546XX_CLKSEL_MAIN);
	SYSCON->LCDCLKDIV = SYSCON_LCDCLKDIV_RESET_MASK;
	SYSCON->LCDCLKDIV = SYSCON_LCDCLKDIV_DIV(0);
	SYSCON->AHBCLKCTRLSET[LCD_LPC546XX_AHBCLK_IDX] = SYSCON_AHBCLKCTRL_LCD_MASK;
	SYSCON->PRESETCTRLSET[LCD_LPC546XX_AHBCLK_IDX] = SYSCON_PRESETCTRL_LCD_RST_MASK;
	SYSCON->PRESETCTRLCLR[LCD_LPC546XX_AHBCLK_IDX] = SYSCON_PRESETCTRL_LCD_RST_MASK;

	// The main clock is the core clock, AHBCLKDIV is 0 in system_LPC546xx.c
	uint32_t clk = SystemCoreClockGet() * ((SYSCON->AHBCLKDIV & 0xFFU) + 1U);
	uint32_t div = (clk + t->PixelClock / 2U) / t->PixelClock;
	uint32_t pol = LCD_POL_CPL(Cfg.Width - 1U);

	// The closest panel clock: not divided, or divided by 2 to 1025
	if (div <= 1U)
	{
		pol |= LCD_POL_BCD_MASK;
		div = 1;
	}
	else
	{
		if (div - 2U > LCD_LPC546XX_PCD_MAX)
		{
			div = LCD_LPC546XX_PCD_MAX + 2U;
		}
		pol |= LCD_POL_PCD_LO(div - 2U) | LCD_POL_PCD_HI((div - 2U) >> 5);
	}
	vPixelClock = clk / div;

	if (t->Flags & DISPL_TIMING_VSYNC_LOW)
	{
		pol |= LCD_POL_IVS_MASK;
	}
	if (t->Flags & DISPL_TIMING_HSYNC_LOW)
	{
		pol |= LCD_POL_IHS_MASK;
	}
	if (t->Flags & DISPL_TIMING_DE_LOW)
	{
		pol |= LCD_POL_IOE_MASK;
	}
	if (t->Flags & DISPL_TIMING_CLK_FALLING)
	{
		pol |= LCD_POL_IPC_MASK;
	}

	// TFT, 16 bpp 5:6:5, little endian. BGR puts red of bits 15:11 on the
	// red lines of the panel (NXP SDK palette description).
	LCD->CTRL = LCD_CTRL_LCDTFT_MASK | LCD_CTRL_LCDBPP(LCD_LPC546XX_BPP_RGB565) |
				((t->Flags & DISPL_TIMING_BGR) ? 0U : LCD_CTRL_BGR_MASK);
	LCD->INTMSK = 0;
	LCD->INTCLR = LCD_INTMSK_FUFIM_MASK | LCD_INTMSK_LNBUIM_MASK | LCD_INTMSK_VCOMPIM_MASK | LCD_INTMSK_BERIM_MASK;
	LCD->CRSR_CTRL = 0;
	LCD->CRSR_INTMSK = 0;
	LCD->TIMH = LCD_TIMH_PPL(Cfg.Width / 16U - 1U) | LCD_TIMH_HSW(t->HSync - 1U) | LCD_TIMH_HFP(t->HFront - 1U) |
				LCD_TIMH_HBP(t->HBack - 1U);
	LCD->TIMV = LCD_TIMV_LPP(Cfg.Height - 1U) | LCD_TIMV_VSW(t->VSync - 1U) | LCD_TIMV_VFP(t->VFront - 1U) |
				LCD_TIMV_VBP(t->VBack - 1U);
	LCD->POL = pol;
	LCD->LE = 0;
	LCD->UPBASE = (uint32_t)(uintptr_t)vpFb;
	LCD->LPBASE = 0;

	Orientation(Cfg.Orient);
	Reset();

	return Enable();
}

bool LcdLpc546xx::Enable()
{
	if (vpFb == NULL)
	{
		return false;
	}

	// Timing first, then the panel power and data
	LCD->CTRL |= LCD_CTRL_LCDEN_MASK;
	LCD->CTRL |= LCD_CTRL_LCDPWR_MASK;

	return true;
}

void LcdLpc546xx::Disable()
{
	if (vpFb == NULL)
	{
		return;
	}

	LCD->CTRL &= ~LCD_CTRL_LCDPWR_MASK;
	LCD->CTRL &= ~LCD_CTRL_LCDEN_MASK;
}

void LcdLpc546xx::PowerOff()
{
	if (vpFb == NULL)
	{
		return;
	}

	Backlight(false);
	Disable();
	SYSCON->AHBCLKCTRLCLR[LCD_LPC546XX_AHBCLK_IDX] = SYSCON_AHBCLKCTRL_LCD_MASK;
	SYSCON->LCDCLKSEL = SYSCON_LCDCLKSEL_SEL(LCD_LPC546XX_CLKSEL_NONE);
	vpFb = NULL;
}

void LcdLpc546xx::Reset()
{
	if (vpFb == NULL)
	{
		return;
	}

	// DisplayDotMatrix::Reset with the reset pin optional
	if (CtrlPin(DISPL_CTRL_RST_PINIDX))
	{
		IOPinClear(vCfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo);
		usDelay(100);
		IOPinSet(vCfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo);
		msDelay(2);
	}
	SetFont(&iFontSystem5x7);
	Clear();
}

void LcdLpc546xx::Backlight(bool bOn)
{
	if (vpFb == NULL || CtrlPin(DISPL_CTRL_BKLIGHT_PINIDX) == false)
	{
		return;
	}
	if (bOn)
	{
		IOPinSet(vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PinNo);
	}
	else
	{
		IOPinClear(vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PinNo);
	}
}

void LcdLpc546xx::Orientation(DISPL_ORIENT Orient)
{
	if (Orient == DISPL_ORIENT_PORTRAIT || Orient == DISPL_ORIENT_PORTRAIT_INV)
	{
		SetDevRes(vPanelH, vPanelW);
	}
	else
	{
		SetDevRes(vPanelW, vPanelH);
	}
	DisplayDotMatrix::Orientation(Orient);
	vCurScrollLine = vHeight;
	if (vpFont != NULL)
	{
		SetFont(vpFont);
	}
}

// Frame buffer address of a pixel in the orientation. Landscape is the panel.
// Portrait: the top left of the screen on the top right of the panel,
// portrait inverted on its bottom left.
uint16_t *LcdLpc546xx::Pixel(uint16_t X, uint16_t Y)
{
	uint32_t px = X;
	uint32_t py = Y;

	switch (Orientation())
	{
		case DISPL_ORIENT_LANDSCAPE_INV:
			px = vPanelW - 1U - X;
			py = vPanelH - 1U - Y;
			break;
		case DISPL_ORIENT_PORTRAIT:
			px = vPanelW - 1U - Y;
			py = X;
			break;
		case DISPL_ORIENT_PORTRAIT_INV:
			px = Y;
			py = vPanelH - 1U - X;
			break;
		default:
			break;
	}

	return vpFb + py * vPanelW + px;
}

// Frame buffer step of X + 1, in pixels
int LcdLpc546xx::PixelStep(void)
{
	switch (Orientation())
	{
		case DISPL_ORIENT_LANDSCAPE_INV:
			return -1;
		case DISPL_ORIENT_PORTRAIT:
			return vPanelW;
		case DISPL_ORIENT_PORTRAIT_INV:
			return -(int)vPanelW;
		default:
			return 1;
	}
}

void LcdLpc546xx::Clear()
{
	if (vpFb == NULL)
	{
		return;
	}

	memset(vpFb, 0, (size_t)vPanelW * vPanelH * 2U);
	vCurScrollLine = vHeight;
	vCurLine = vLineHeight;
	vCurCol = 0;
}

void LcdLpc546xx::Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color)
{
	if (vpFb == NULL || X >= vWidth || Y >= vHeight)
	{
		return;
	}

	uint32_t w = (uint32_t)X + Width > vWidth ? vWidth - X : Width;
	uint32_t h = (uint32_t)Y + Height > vHeight ? vHeight - Y : Height;
	int ps = PixelStep();
	uint16_t c = (uint16_t)Color;

	for (uint32_t j = 0; j < h; j++)
	{
		uint16_t *p = Pixel(X, (uint16_t)(Y + j));

		for (uint32_t i = 0; i < w; i++, p += ps)
		{
			*p = c;
		}
	}
}

void LcdLpc546xx::SetPixel(uint16_t X, uint16_t Y, uint32_t Color)
{
	if (vpFb != NULL && X < vWidth && Y < vHeight)
	{
		*Pixel(X, Y) = (uint16_t)Color;
	}
}

// pMem: Width x Height pixels of 16 bits, line after line
void LcdLpc546xx::BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pMem)
{
	if (vpFb == NULL || pMem == NULL || X >= vWidth || Y >= vHeight)
	{
		return;
	}

	uint32_t w = (uint32_t)X + Width > vWidth ? vWidth - X : Width;
	uint32_t h = (uint32_t)Y + Height > vHeight ? vHeight - Y : Height;
	int ps = PixelStep();

	for (uint32_t j = 0; j < h; j++)
	{
		uint16_t *p = Pixel(X, (uint16_t)(Y + j));
		const uint8_t *s = pMem + (size_t)j * Width * 2U;

		for (uint32_t i = 0; i < w; i++, p += ps, s += 2)
		{
			uint16_t v;

			memcpy(&v, s, 2);
			*p = v;
		}
	}
}

void LcdLpc546xx::Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count)
{
	if (vpFb == NULL || Count == 0U || (Dir != DISPL_SCROLL_DIR_UP && Dir != DISPL_SCROLL_DIR_DOWN))
	{
		return;
	}
	if (Count >= vHeight)
	{
		Fill(0, 0, vWidth, vHeight, 0);

		return;
	}

	int ps = PixelStep();
	uint32_t n = vHeight - Count;

	// A line of the screen is a line of the panel in landscape
	for (uint32_t k = 0; k < n; k++)
	{
		uint32_t dst = Dir == DISPL_SCROLL_DIR_UP ? k : vHeight - 1U - k;
		uint32_t src = Dir == DISPL_SCROLL_DIR_UP ? k + Count : vHeight - 1U - k - Count;
		uint16_t *d = Pixel(0, (uint16_t)dst);
		uint16_t *s = Pixel(0, (uint16_t)src);

		if (ps == 1)
		{
			memmove(d, s, (size_t)vWidth * 2U);
		}
		else if (ps == -1)
		{
			memmove(d - (vWidth - 1U), s - (vWidth - 1U), (size_t)vWidth * 2U);
		}
		else
		{
			for (uint32_t i = 0; i < vWidth; i++, d += ps, s += ps)
			{
				*d = *s;
			}
		}
	}
	Fill(0, Dir == DISPL_SCROLL_DIR_UP ? (uint16_t)n : 0, vWidth, Count, 0);
}
