/**-------------------------------------------------------------------------
@file	led_ws2812.cpp

@brief	Implementation of WS2812 / WS2812B / SK6812 RGB LED device class.

	Uses the IOsonata DeviceIntrf abstraction (typically an SPI
	master configured at ~3.2 MHz, MSB-first, mode 0) to clock out
	WS2812-encoded data.  Each WS2812 bit becomes 4 SPI bits:

	  WS2812 '0' → 0b1000   (~312 ns high, ~938 ns low at 3.2 MHz)
	  WS2812 '1' → 0b1110   (~938 ns high, ~312 ns low at 3.2 MHz)

	Two WS2812 bits pack into one SPI byte:

	  '00' → 0x88   '01' → 0x8E   '10' → 0xE8   '11' → 0xEE

	Per pixel: 24 WS2812 bits → 12 SPI bytes.

	After the bus tx completes, the driver delays >50 µs so the
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

/*-----------------------------------------------------------------------
 * Encoding lookup: each pair of WS2812 bits becomes one SPI byte.
 * Index = high-bit (sent first) << 1 | low-bit
 *---------------------------------------------------------------------*/
static const uint8_t s_Ws2812_Pair[4] = {
    0x88,   /* 00 → 1000 1000 */
    0x8E,   /* 01 → 1000 1110 */
    0xE8,   /* 10 → 1110 1000 */
    0xEE,   /* 11 → 1110 1110 */
};

/*
 * Encode one WS2812 source byte (8 bits, MSB first on the wire) into
 * 4 SPI bytes at *pOut.  Returns the new write pointer.
 */
uint8_t *LedWs2812::EncodeByte(uint8_t in, uint8_t *pOut)
{
    pOut[0] = s_Ws2812_Pair[(in >> 6) & 0x3];   /* bits 7..6 */
    pOut[1] = s_Ws2812_Pair[(in >> 4) & 0x3];   /* bits 5..4 */
    pOut[2] = s_Ws2812_Pair[(in >> 2) & 0x3];   /* bits 3..2 */
    pOut[3] = s_Ws2812_Pair[(in >> 0) & 0x3];   /* bits 1..0 */
    return pOut + 4;
}

bool LedWs2812::Init(const WS2812_CFG &Cfg, DeviceIntrf &Intrf)
{
    if (Cfg.NbLed <= 0 || Cfg.NbLed > WS2812_MAX_PIXELS)
    {
        return false;
    }

    vpIntrf     = &Intrf;
    vNbLed      = Cfg.NbLed;
    vOrder      = Cfg.Order;
    vBrightness = Cfg.Brightness;
    vDevCs      = Cfg.DevCs;

    /* Pre-fill buffer with all-zero-bit encoding so a stray Level(NULL,0)
     * leaves the LEDs dark rather than at random colours from prior RAM. */
    for (int i = 0; i < vNbLed * WS2812_BYTES_PER_PIXEL; i++)
    {
        vBuf[i] = s_Ws2812_Pair[0];
    }

    Type(LED_TYPE_STRIP);
    return true;
}

void LedWs2812::On()
{
    uint32_t v = WS2812_WHITE;
    Level(&v, 1);
}

void LedWs2812::Off()
{
    uint32_t off[WS2812_MAX_PIXELS];
    int n = vNbLed;
    if (n > WS2812_MAX_PIXELS) n = WS2812_MAX_PIXELS;
    for (int i = 0; i < n; i++) off[i] = WS2812_OFF;
    Level(off, n);
}

/*
 * Send a frame to the chain.
 *
 *   pVal    : array of 0xRRGGBB pixel values (driver re-orders for wire)
 *   NbLeds  : number of pixels (clamped to vNbLed and WS2812_MAX_PIXELS)
 */
void LedWs2812::Level(uint32_t * const pVal, int NbLeds)
{
    if (vpIntrf == nullptr || pVal == nullptr || NbLeds <= 0)
    {
        return;
    }

    if (NbLeds > vNbLed)            NbLeds = vNbLed;
    if (NbLeds > WS2812_MAX_PIXELS) NbLeds = WS2812_MAX_PIXELS;

    uint8_t *p = vBuf;

    for (int i = 0; i < NbLeds; i++)
    {
        uint32_t pix = pVal[i];

        uint8_t r = (uint8_t)((pix >> 16) & 0xFFU);
        uint8_t g = (uint8_t)((pix >>  8) & 0xFFU);
        uint8_t b = (uint8_t)((pix >>  0) & 0xFFU);

        /* Apply master brightness as a fixed-point scale. */
        if (vBrightness < 255)
        {
            r = (uint8_t)(((uint16_t)r * (uint16_t)vBrightness) >> 8);
            g = (uint8_t)(((uint16_t)g * (uint16_t)vBrightness) >> 8);
            b = (uint8_t)(((uint16_t)b * (uint16_t)vBrightness) >> 8);
        }

        /* Re-order channels to match the LED's wire format. */
        uint8_t c0, c1, c2;
        switch (vOrder)
        {
            default:
            case WS2812_ORDER_GRB:  c0 = g; c1 = r; c2 = b; break;
            case WS2812_ORDER_RGB:  c0 = r; c1 = g; c2 = b; break;
            case WS2812_ORDER_BGR:  c0 = b; c1 = g; c2 = r; break;
            case WS2812_ORDER_BRG:  c0 = b; c1 = r; c2 = g; break;
            case WS2812_ORDER_RBG:  c0 = r; c1 = b; c2 = g; break;
            case WS2812_ORDER_GBR:  c0 = g; c1 = b; c2 = r; break;
        }

        p = EncodeByte(c0, p);
        p = EncodeByte(c1, p);
        p = EncodeByte(c2, p);
    }

    /* Single-shot Tx through the bus.  For SPI masters the DevAddr is
     * the chip-select index. */
    int len = NbLeds * WS2812_BYTES_PER_PIXEL;
    if (vpIntrf->StartTx(vDevCs))
    {
        vpIntrf->TxData(vBuf, len);
        vpIntrf->StopTx();
    }

    /* Hold MOSI low for the latch period.  60 µs is comfortably above
     * the 50 µs minimum on classic WS2812; newer SK6812 / WS2812B
     * variants sometimes need 80-280 µs — bump if your strip needs it. */
    usDelay(60);
}
