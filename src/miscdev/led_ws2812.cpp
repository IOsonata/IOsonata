/**-------------------------------------------------------------------------
@file	led_ws2812.cpp

@brief	Implementation of WS2812 / WS2812B / SK6812 RGB LED device class.

	Uses the IOsonata DeviceIntrf abstraction (typically an SPI
	master configured at ~3.2 MHz, MSB-first, mode 0) to clock out
	WS2812-encoded data.  Each WS2812 bit becomes 4 SPI bits:

	  WS2812 '0' -> 0b1000   (~312 ns high, ~938 ns low at 3.2 MHz)
	  WS2812 '1' -> 0b1110   (~938 ns high, ~312 ns low at 3.2 MHz)

	Two WS2812 bits pack into one SPI byte:

	  '00' -> 0x88   '01' -> 0x8E   '10' -> 0xE8   '11' -> 0xEE

	Per pixel: 24 WS2812 bits -> 12 SPI bytes.

	Before and after each synchronous transfer, the driver waits 300 us so the
	LED chain latches.  This is done with usDelay() from idelay.h
	which is calibrated per-MCU at runtime via SystemCoreClock.

@author	Hoang Nguyen Hoan (IOsonata project)
@date	May 2026

@license MIT
----------------------------------------------------------------------------*/

#include <stdint.h>

#include "idelay.h"
#include "device_intrf.h"
#include "miscdev/led_ws2812.h"

// Two WS2812 bits become one SPI byte, high bit first.
static const uint8_t s_Ws2812_Pair[4] = {0x88, 0x8E, 0xE8, 0xEE};

uint8_t *LedWs2812::EncodeByte(uint8_t in, uint8_t *pOut)
{
	pOut[0] = s_Ws2812_Pair[(in >> 6) & 3];
	pOut[1] = s_Ws2812_Pair[(in >> 4) & 3];
	pOut[2] = s_Ws2812_Pair[(in >> 2) & 3];
	pOut[3] = s_Ws2812_Pair[in & 3];
	return pOut + 4;
}

bool LedWs2812::Init(const WS2812_CFG &Cfg, DeviceIntrf &Intrf)
{
	int i;

	if (Cfg.NbLed <= 0 || Cfg.NbLed > WS2812_MAX_PIXELS || Cfg.Order > WS2812_ORDER_GBR)
	{
		return false;
	}
	vpIntrf = &Intrf;
	vNbLed = Cfg.NbLed;
	vOrder = Cfg.Order;
	vBrightness = Cfg.Brightness;
	vDevCs = Cfg.DevCs;
	for (i = 0; i < vNbLed * WS2812_BYTES_PER_PIXEL; i++)
	{
		vBuf[i] = s_Ws2812_Pair[0];
	}
	Type(LED_TYPE_STRIP);
	return true;
}

void LedWs2812::On()
{
	uint32_t value = WS2812_WHITE;

	Level(&value, 1);
}

void LedWs2812::Off()
{
	uint32_t off[WS2812_MAX_PIXELS];
	int n;
	int i;

	n = vNbLed;
	if (n > WS2812_MAX_PIXELS)
	{
		n = WS2812_MAX_PIXELS;
	}
	for (i = 0; i < n; i++)
	{
		off[i] = WS2812_OFF;
	}
	Level(off, n);
}

void LedWs2812::Level(uint32_t * const pVal, int NbLeds)
{
	uint8_t *p;
	uint32_t pixel;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t c0;
	uint8_t c1;
	uint8_t c2;
	int i;
	int length;

	if (vpIntrf == NULL || pVal == NULL || NbLeds <= 0)
	{
		return;
	}
	if (NbLeds > vNbLed)
	{
		NbLeds = vNbLed;
	}
	if (NbLeds > WS2812_MAX_PIXELS)
	{
		NbLeds = WS2812_MAX_PIXELS;
	}
	p = vBuf;
	for (i = 0; i < NbLeds; i++)
	{
		pixel = pVal[i];
		r = (uint8_t)(pixel >> 16);
		g = (uint8_t)(pixel >> 8);
		b = (uint8_t)pixel;
		if (vBrightness < 255)
		{
			r = (uint8_t)(((uint16_t)r * vBrightness) >> 8);
			g = (uint8_t)(((uint16_t)g * vBrightness) >> 8);
			b = (uint8_t)(((uint16_t)b * vBrightness) >> 8);
		}
		switch (vOrder)
		{
			default:
			case WS2812_ORDER_GRB:
				c0 = g;
				c1 = r;
				c2 = b;
				break;
			case WS2812_ORDER_RGB:
				c0 = r;
				c1 = g;
				c2 = b;
				break;
			case WS2812_ORDER_BGR:
				c0 = b;
				c1 = g;
				c2 = r;
				break;
			case WS2812_ORDER_BRG:
				c0 = b;
				c1 = r;
				c2 = g;
				break;
			case WS2812_ORDER_RBG:
				c0 = r;
				c1 = b;
				c2 = g;
				break;
			case WS2812_ORDER_GBR:
				c0 = g;
				c1 = b;
				c2 = r;
				break;
		}
		p = EncodeByte(c0, p);
		p = EncodeByte(c1, p);
		p = EncodeByte(c2, p);
	}
	length = NbLeds * WS2812_BYTES_PER_PIXEL;
	// The caller supplies synchronous SPI with MOSI idle low. This delay
	// separates frames; the final encoded bit also leaves MOSI low.
	usDelay(300);
	if (vpIntrf->StartTx(vDevCs))
	{
		vpIntrf->TxData(vBuf, length);
		vpIntrf->StopTx();
	}
	usDelay(300);
}
