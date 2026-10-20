/**-------------------------------------------------------------------------
@file display_ssd1315.cpp
@brief SSD1315 monochrome OLED display controller.
@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <string.h>
#include "idelay.h"
#include "display/display_ssd1315.h"

LcdSSD1315::LcdSSD1315() : vbReady(false)
{
	memset(&vCfg, 0, sizeof(vCfg));
	memset(vFrame, 0, sizeof(vFrame));
	vWidth = vHeight = vCurCol = vCurLine = 0;
	vpFont = nullptr;
	vpLineBuff = nullptr;
	vPixelLen = 0; // Packed pixels, rather than whole bytes per pixel.
	vLineHeight = 8;
	vCurScrollLine = 0;
	Type(DISPL_TYPE_MATRIX);
	DisplayDotMatrix::Orientation(DISPL_ORIENT_LANDSCAPE);
}

bool LcdSSD1315::Init(const DisplayCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	Valid(false);
	vbReady = false;
	if (pIntrf == nullptr || pIntrf->Type() != DEVINTRF_TYPE_SPI ||
		Cfg.pPins == nullptr || Cfg.NbPins <= DISPL_CTRL_DCX_PINIDX ||
		Cfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo < 0 ||
		Cfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo < 0 ||
		Cfg.Width == 0 || Cfg.Width > SSD1315_WIDTH_MAX ||
		Cfg.Height < 16 || Cfg.Height > SSD1315_HEIGHT_MAX ||
		(Cfg.Height & 7) != 0 || Cfg.PixelSize != 1 ||
		Cfg.Stride != (Cfg.Width + 7) / 8 ||
		(unsigned)Cfg.Orient > DISPL_ORIENT_LANDSCAPE_INV)
	{
		return false;
	}
	DevIntrf_t *intrf = *pIntrf;
	if (intrf == nullptr || intrf->bIntEn || intrf->bDma)
	{
		return false;
	}
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vCfg = Cfg;
	// Configure only the pins this driver owns, not a placeholder backlight.
	IOPinCfg(&Cfg.pPins[DISPL_CTRL_DCX_PINIDX], 1);
	if (Cfg.NbPins > DISPL_CTRL_RST_PINIDX &&
		Cfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo >= 0 &&
		Cfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo >= 0)
	{
		IOPinSet(Cfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo,
			Cfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo);
		IOPinCfg(&Cfg.pPins[DISPL_CTRL_RST_PINIDX], 1);
	}
	vbReady = true;
	Orientation(Cfg.Orient);
	SetFont(&iFontSystem5x7);
	Reset();
	return Valid();
}

bool LcdSSD1315::Send(bool bData, const uint8_t *pData, int Len)
{
	if (!vbReady || Len <= 0 || pData == nullptr || !vpIntrf->StartTx(DeviceAddress()))
	{
		return false;
	}
	const IOPinCfg_t &pin = vCfg.pPins[DISPL_CTRL_DCX_PINIDX];
	if (bData) { IOPinSet(pin.PortNo, pin.PinNo); }
	else { IOPinClear(pin.PortNo, pin.PinNo); }
	int count = vpIntrf->TxData(pData, Len);
	vpIntrf->StopTx();
	return count == Len;
}

bool LcdSSD1315::Configure()
{
	// Commands and their arguments all use D/C low. Page addressing permits
	// updating a single framebuffer byte without disturbing adjacent pixels.
	const uint8_t cmd[] = {
		0xAE,                   // Display off during configuration
		0xD5, 0x80,             // Clock divide and oscillator
		0xA8, (uint8_t)(vCfg.Height - 1), // Multiplex ratio
		0xD3, 0x00, 0x40,       // Offset zero, start line zero
		0x8D, 0x14,             // Internal charge pump
		0x20, 0x02,             // Page addressing
		0xA1, 0xC8,             // Segment remap and COM scan direction
		0xDA, (uint8_t)(vCfg.Height > 32 ? 0x12 : 0x02),
		0x81, 0x7F,             // Contrast
		0xD9, 0xF1, 0xDB, 0x20, // Precharge and VCOMH (0.77 x VCC)
		0xA4, 0xA6, 0x2E        // RAM display, normal pixels, scroll off
	};
	return Send(false, cmd, sizeof(cmd));
}

void LcdSSD1315::Reset()
{
	Valid(false);
	if (!vbReady) { return; }
	if (vCfg.NbPins > DISPL_CTRL_RST_PINIDX &&
		vCfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo >= 0 &&
		vCfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo >= 0)
	{
		const IOPinCfg_t &pin = vCfg.pPins[DISPL_CTRL_RST_PINIDX];
		IOPinClear(pin.PortNo, pin.PinNo);
		msDelay(1);
		IOPinSet(pin.PortNo, pin.PinNo);
	}
	msDelay(100);
	memset(vFrame, 0, sizeof(vFrame));
	vCurLine = vCurCol = 0;
	if (Configure() && Refresh() && Enable()) { Valid(true); }
}

bool LcdSSD1315::Enable()
{
	const uint8_t cmd[] = {0x8D, 0x14, 0xAF};
	return Send(false, cmd, sizeof(cmd));
}

void LcdSSD1315::Disable()
{
	const uint8_t cmd[] = {0xAE, 0x8D, 0x10};
	Send(false, cmd, sizeof(cmd));
}

bool LcdSSD1315::Contrast(uint8_t Value)
{
	const uint8_t cmd[] = {0x81, Value};
	return Send(false, cmd, sizeof(cmd));
}

bool LcdSSD1315::Page(uint16_t X, uint16_t Y, uint16_t Width)
{
	const uint8_t cmd[] = {(uint8_t)(0xB0 | (Y >> 3)),
		(uint8_t)(X & 0x0F), (uint8_t)(0x10 | (X >> 4))};
	return Send(false, cmd, sizeof(cmd)) &&
		Send(true, &vFrame[(Y >> 3) * vCfg.Width + X], Width);
}

bool LcdSSD1315::Refresh()
{
	if (!vbReady) { return false; }
	for (uint16_t y = 0; y < vCfg.Height; y += 8)
	{
		if (!Page(0, y, vCfg.Width)) { return false; }
	}
	return true;
}

void LcdSSD1315::Orientation(DISPL_ORIENT Orient)
{
	if ((unsigned)Orient > DISPL_ORIENT_LANDSCAPE_INV) { return; }
	DisplayDotMatrix::Orientation(Orient);
	bool portrait = Orient == DISPL_ORIENT_PORTRAIT || Orient == DISPL_ORIENT_PORTRAIT_INV;
	SetDevRes(portrait ? vCfg.Height : vCfg.Width, portrait ? vCfg.Width : vCfg.Height);
	vCurLine = vCurCol = 0;
	vCurScrollLine = vHeight;
}

bool LcdSSD1315::Map(uint16_t &X, uint16_t &Y)
{
	if (!vbReady || X >= vWidth || Y >= vHeight) { return false; }
	uint16_t x = X;
	switch (Orientation())
	{
		case DISPL_ORIENT_PORTRAIT: X = vCfg.Width - 1 - Y; Y = x; break;
		case DISPL_ORIENT_PORTRAIT_INV: X = Y; Y = vCfg.Height - 1 - x; break;
		case DISPL_ORIENT_LANDSCAPE_INV:
			X = vCfg.Width - 1 - X; Y = vCfg.Height - 1 - Y; break;
		default: break;
	}
	return true;
}

void LcdSSD1315::Pixel(uint16_t X, uint16_t Y, uint32_t Color)
{
	if (!Map(X, Y)) { return; }
	uint8_t &byte = vFrame[(Y >> 3) * vCfg.Width + X];
	uint8_t mask = 1U << (Y & 7);
	if (Color) { byte |= mask; }
	else { byte &= (uint8_t)~mask; }
}

void LcdSSD1315::SetPixel(uint16_t X, uint16_t Y, uint32_t Color)
{
	Pixel(X, Y, Color);
	if (Map(X, Y)) { Page(X, Y, 1); }
}

void LcdSSD1315::Clear()
{
	memset(vFrame, 0, sizeof(vFrame));
	vCurLine = vCurCol = 0;
	Refresh();
}

void LcdSSD1315::Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color)
{
	if (X >= vWidth || Y >= vHeight || !Width || !Height) { return; }
	uint16_t w = Width < vWidth - X ? Width : vWidth - X;
	uint16_t h = Height < vHeight - Y ? Height : vHeight - Y;
	for (uint16_t y = 0; y < h; y++)
	{
		for (uint16_t x = 0; x < w; x++) { Pixel(X + x, Y + y, Color); }
	}
	Refresh();
}

void LcdSSD1315::BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pMem)
{
	if (!pMem || X >= vWidth || Y >= vHeight || !Width || !Height) { return; }
	uint16_t w = Width < vWidth - X ? Width : vWidth - X;
	uint16_t h = Height < vHeight - Y ? Height : vHeight - Y;
	uint32_t stride = ((uint32_t)Width + 7) / 8;
	for (uint16_t y = 0; y < h; y++)
	{
		for (uint16_t x = 0; x < w; x++)
		{
			Pixel(X + x, Y + y, pMem[y * stride + (x >> 3)] & (0x80U >> (x & 7)));
		}
	}
	Refresh();
}

void LcdSSD1315::Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count)
{
	if (!vbReady || !Count) { return; }
	// Shift in logical coordinates, taking the source before overwriting it.
	// A single row/column is traversed in the direction opposite to motion.
	if ((unsigned)Dir > DISPL_SCROLL_DIR_RIGHT) { return; }
	for (uint16_t row = 0; row < vHeight; row++)
	{
		uint16_t y = Dir == DISPL_SCROLL_DIR_DOWN ? vHeight - 1 - row : row;
		for (uint16_t col = 0; col < vWidth; col++)
		{
			uint16_t x = Dir == DISPL_SCROLL_DIR_RIGHT ? vWidth - 1 - col : col;
			int sx = x, sy = y;
			if (Dir == DISPL_SCROLL_DIR_UP) { sy += Count; }
			if (Dir == DISPL_SCROLL_DIR_DOWN) { sy -= Count; }
			if (Dir == DISPL_SCROLL_DIR_LEFT) { sx += Count; }
			if (Dir == DISPL_SCROLL_DIR_RIGHT) { sx -= Count; }
			uint16_t px = (uint16_t)sx, py = (uint16_t)sy;
			bool on = sx >= 0 && sy >= 0 && Map(px, py) &&
				(vFrame[(py >> 3) * vCfg.Width + px] & (1U << (py & 7)));
			Pixel(x, y, on);
		}
	}
	Refresh();
}

bool LcdSSD1315::SetFont(FontDesc_t const *pFont)
{
	if (!pFont || !pFont->Width || !pFont->Height || pFont->Height > vHeight ||
		pFont->Width > vWidth || pFont->Prop & FONT_LOC_EXTERN ||
		(pFont->Prop & FONT_TYPE_MASK) == FONT_TYPE_GLYTH || !pFont->pBits)
	{
		return false;
	}
	vpFont = pFont;
	vLineHeight = pFont->Height + 1;
	return true;
}

void LcdSSD1315::NewLine()
{
	vCurCol = 0;
	vCurLine += vLineHeight;
	if (vCurLine + vpFont->Height > vHeight)
	{
		Scroll(DISPL_SCROLL_DIR_UP, vLineHeight);
		vCurLine = vHeight > vLineHeight ? vHeight - vLineHeight : 0;
	}
}

void LcdSSD1315::Print(char const *pStr, uint32_t Color)
{
	if (!vbReady || !pStr || !vpFont)
	{
		return;
	}
	uint16_t first[SSD1315_HEIGHT_MAX / 8];
	uint16_t last[SSD1315_HEIGHT_MAX / 8] = {};

	for (unsigned page = 0; page < SSD1315_HEIGHT_MAX / 8; page++)
	{
		first[page] = vCfg.Width;
	}
	while (*pStr)
	{
		unsigned ch = (uint8_t)*pStr++;
		if (ch == '\r')
		{
			vCurCol = 0;
			continue;
		}
		if (ch == '\n')
		{
			NewLine();
			continue;
		}
		if (ch < 32 || ch > 126)
		{
			ch = '?';
		}
		unsigned type = vpFont->Prop & FONT_TYPE_MASK;
		const uint8_t *bits = nullptr;
		uint16_t w = vpFont->Width;
		if (ch != ' ')
		{
			unsigned idx = ch - '!';
			if (type == FONT_TYPE_VAR_WIDTH)
			{
				w = vpFont->pCharDesc[idx].Width;
				bits = vpFont->pCharDesc[idx].pBits;
				if (w > vpFont->Width || !bits)
				{
					continue;
				}
			}
			else
			{
				unsigned size = type == FONT_TYPE_FIXED_VERT ? w * ((vpFont->Height + 7U) / 8U) : ((w + 7U) / 8U) * vpFont->Height;
				bits = vpFont->pBits + idx * size;
			}
		}
		if ((uint32_t)vCurCol + w > vWidth)
		{
			NewLine();
		}
		if ((uint32_t)vCurLine + vpFont->Height > vHeight)
		{
			vCurLine = 0;
		}
		for (uint16_t y = 0; y < vpFont->Height; y++)
		{
			for (uint16_t x = 0; x <= w; x++)
			{
				bool on = false;
				if (bits && x < w)
				{
					on = type == FONT_TYPE_FIXED_VERT ? (bits[(y >> 3) * w + x] & (1U << (y & 7))) : (bits[y * ((w + 7U) / 8U) + (x >> 3)] & (0x80U >> (x & 7)));
				}
				uint16_t px = vCurCol + x;
				uint16_t py = vCurLine + y;

				if (Map(px, py))
				{
					unsigned page = py >> 3;
					uint8_t &byte = vFrame[page * vCfg.Width + px];
					uint8_t before = byte;
					uint8_t mask = 1U << (py & 7);

					if (on && Color)
					{
						byte |= mask;
					}
					else
					{
						byte &= uint8_t(~mask);
					}
					if (byte != before)
					{
						if (px < first[page])
						{
							first[page] = px;
						}
						if (px > last[page])
						{
							last[page] = px;
						}
					}
				}
			}
		}
		vCurCol += w + 1;
	}
	// Text updates need only the changed spans, not all 1,024 screen bytes.
	for (unsigned page = 0; page < vCfg.Height / 8U; page++)
	{
		if (first[page] <= last[page])
		{
			Page(first[page], page * 8, last[page] - first[page] + 1);
		}
	}
}
