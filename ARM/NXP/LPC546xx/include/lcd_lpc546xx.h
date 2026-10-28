/**-------------------------------------------------------------------------
@file	lcd_lpc546xx.h

@brief	LPC546xx LCD controller, TFT panel on a frame buffer

LcdLpc546xx is the display controller of a parallel RGB TFT panel: the LCD
controller of the LPC546xx scans a frame buffer in memory, the internal SRAM
or an SDRAM, and makes the panel timings. It is a DisplayDotMatrix: Clear,
Fill, SetPixel, BitBlt, text with printf and the fonts of IOsonata, Scroll,
the four orientations. Pixels are 16 bit RGB565, red in bits 15:11, as the
other IOsonata displays. The controller reads the frame buffer on its own: a
pixel written is on the panel at the next frame.

Init(const DisplayCfg_t &, DeviceIntrf *) as the other display controllers,
no interface (nullptr, not used):

	pPins		the display control pins, then the LCD pins in their LCD
				function (LCD_PWR, LCD_DCLK, LCD_FP, LCD_LP, LCD_AC, LCD_VDn):
				DISPL_CTRL_DCX_PINIDX none ({-1, -1}, an RGB panel has no
				command line), DISPL_CTRL_BKLIGHT_PINIDX the backlight GPIO,
				on high, DISPL_CTRL_RST_PINIDX the panel reset GPIO, active
				low; {-1, -1} for the ones the board does not have
	Width		pixels per line, a multiple of 16, 16 to 1024
	Height		lines, 1 to 1024
	PixelSize	16, RGB565
	Orient		orientation, the panel lines are horizontal in landscape
	pTiming		the panel timings: HSync, HFront, HBack 1 to 256 pixel
				clocks, VSync 1 to 64 lines, VFront and VBack 1 to 256 lines
	pFrameBuffer	Width x Height pixels, 8 byte aligned

DevAddr and Stride are not used: the lines of the frame buffer follow each
other, Width x 2 bytes.

The registers are set as the NXP SDK fsl_lcdc.c in TFT mode: the LCD clock
is the main clock, the panel clock this clock divided by 2 to 1025 (POL PCD),
or not divided (BCD) when the pixel clock asked is the LCD clock or more. A
field of the TIMH and TIMV registers holds the value - 1. The controller
turns the LCD_PWR pin on with the panel output (CTRL LCDPWR). The panel data
lines carry red on VD0 to VD7 and blue on VD16 to VD23; DISPL_TIMING_BGR
swaps them.

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
#ifndef __LCD_LPC546XX_H__
#define __LCD_LPC546XX_H__

#include <stdint.h>

#include "coredev/iopincfg.h"
#include "display/display.h"

#ifdef __cplusplus

class LcdLpc546xx : public DisplayDotMatrix {
public:
	LcdLpc546xx();
	virtual ~LcdLpc546xx() {}

	/**
	 * @brief	Set up the controller and the panel, clear the frame buffer,
	 * 			start the output. The backlight stays off.
	 *
	 * @param	Cfg		: Display configuration, see above
	 * @param	pIntrf	: Not used, nullptr
	 *
	 * @return	false - a configuration out of the ranges above
	 */
	virtual bool Init(const DisplayCfg_t &Cfg, DeviceIntrf * const pIntrf);

	/// Panel output on (LCDEN then LCDPWR)
	virtual bool Enable();

	/// Panel output off (LCDPWR then LCDEN), the frame buffer is kept
	virtual void Disable();

	/// Panel reset pulse when wired, font back to the system font, text at
	/// the top, the screen cleared
	virtual void Reset();

	/// Output off, the controller clock off, the backlight off
	virtual void PowerOff();

	virtual void Clear();
	virtual void Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color);
	virtual void SetPixel(uint16_t X, uint16_t Y, uint32_t Color);
	virtual void BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pMem);

	/// Up or down: the screen moves by Count pixels, the lines freed are cleared
	virtual void Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count);

	virtual void Backlight(bool bOn);

	/// Width and Height follow the orientation
	virtual void Orientation(DISPL_ORIENT Orient);
	virtual DISPL_ORIENT Orientation() { return DisplayDotMatrix::Orientation(); }

	/// Panel clock made, Hz
	uint32_t PixelClock() const { return vPixelClock; }

	/// RGB565 color from 8 bit components
	static uint16_t Rgb565(uint8_t R, uint8_t G, uint8_t B) {
		return (uint16_t)(((R & 0xF8U) << 8) | ((G & 0xFCU) << 3) | (B >> 3));
	}

private:
	uint16_t *Pixel(uint16_t X, uint16_t Y);
	int PixelStep(void);
	bool CtrlPin(int Idx) const;

	uint16_t *vpFb;
	uint16_t vPanelW;
	uint16_t vPanelH;
	uint32_t vPixelClock;
};

#endif // __cplusplus

#endif // __LCD_LPC546XX_H__
