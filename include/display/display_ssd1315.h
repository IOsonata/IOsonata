/**-------------------------------------------------------------------------
@file display_ssd1315.h
@brief SSD1315 monochrome OLED over a synchronous four-wire SPI interface.
@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef __DISPLAY_SSD1315_H__
#define __DISPLAY_SSD1315_H__

#include "display/display.h"

#define SSD1315_WIDTH_MAX 128
#define SSD1315_HEIGHT_MAX 64
#define SSD1315_FRAME_SIZE (SSD1315_WIDTH_MAX * SSD1315_HEIGHT_MAX / 8)

// Uses DisplayCfg_t: PixelSize = 1; Width <= 128; Height = 16..64 in whole
// pages; Stride = (Width + 7) / 8. Width and Height describe the native panel.
// pPins uses DISPL_CTRL_* indices. D/C is required; reset is optional. Set
// unused backlight/reset entries to PortNo = PinNo = -1. No backlight is used.
// The supplied SPI must be initialized, 8-bit, MSB first, mode 0, polling.
// All drawing methods update the display immediately. Colors: zero = black,
// nonzero = white. SetCursor uses the top-left pixel of the next character.
class LcdSSD1315 : public DisplayDotMatrix {
public:
	LcdSSD1315();
	bool Init(const DisplayCfg_t &Cfg, DeviceIntrf * const pIntrf) override;
	bool Enable() override;
	void Disable() override;
	void Reset() override;
	void Clear() override;
	void Backlight(bool bState) override { (void)bState; }
	void Orientation(DISPL_ORIENT Orient) override;
	DISPL_ORIENT Orientation() override { return DisplayDotMatrix::Orientation(); }
	bool SetFont(FontDesc_t const *pFont) override;
	void Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color) override;
	void SetPixel(uint16_t X, uint16_t Y, uint32_t Color) override;
	// Packed monochrome bitmap, MSB first, each row padded to a whole byte.
	void BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pMem) override;
	void Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count) override;
	// Retry sending the framebuffer after a transfer failure; no heap storage.
	bool Refresh();
	bool Contrast(uint8_t Value);

protected:
	void Print(char const *pStr, uint32_t Color) override;

private:
	bool Send(bool bData, const uint8_t *pData, int Len);
	bool Page(uint16_t X, uint16_t Y, uint16_t Width);
	bool Configure();
	bool Map(uint16_t &X, uint16_t &Y);
	void Pixel(uint16_t X, uint16_t Y, uint32_t Color);
	void NewLine();
	uint8_t vFrame[SSD1315_FRAME_SIZE];
	bool vbReady;
};

#endif
