/**-------------------------------------------------------------------------
@file	led_ws2812.h

@brief	Implementation of WS2812 / WS2812B / SK6812 (NeoPixel) RGB LED
	device class.

WS2812-family LEDs use a single-wire asynchronous protocol where each
bit is encoded by the ratio of high-time to low-time within a fixed
~1.25 µs bit period:

  '0' bit : 0.4 µs high, 0.85 µs low
  '1' bit : 0.8 µs high, 0.45 µs low
  Reset   : >50 µs low (latches the shifted-in data)

To make this work across every MCU IOsonata supports, this driver
serializes the WS2812 waveform out of an SPI peripheral's MOSI line
instead of bit-banging GPIO.  Each WS2812 bit is encoded as 4 SPI
bits at ~3.2 MHz SPI clock:

  WS2812 '0' = 0b1000  (~312 ns high, ~938 ns low)
  WS2812 '1' = 0b1110  (~938 ns high, ~312 ns low)

Two WS2812 bits pack into one SPI byte, so each 24-bit RGB pixel
takes 12 SPI bytes.  The SCK line is unused by the LED — only MOSI
is wired to the LED data input.

The SPI peripheral is configured by the caller and passed in as a
DeviceIntrf.  This is the same Init pattern used by the IOsonata
sensor classes — the driver does not care which physical SPI block
or which MCU underneath, only that the bus is configured for ~3.2 MHz,
MSB-first, SPI mode 0 (CPOL=0, CPHA=0).

Usage example:

    static const IOPinCfg_t s_SpiPins[] = {
        { SPI_SCK_PORT,  SPI_SCK_PIN,  0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
        { -1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },   // MISO unused
        { SPI_MOSI_PORT, SPI_MOSI_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
        { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },  // CS unused
    };

    static const SPICfg_t s_SpiCfg = {
        .DevNo     = 0,
        .Phy       = SPIPHY_NORMAL,
        .Mode      = SPIMODE_MASTER,
        .pIOPinMap = s_SpiPins,
        .NbIOPins  = 4,
        .Rate      = 3200000,
        .DataSize  = 8,
        .MaxRetry  = 5,
        .ChipSel   = SPICSEL_AUTO,
        .bDmaEn    = false,
        .bIntEn    = false,
        .IntPrio   = 6,
    };

    SPI g_Spi;
    LedWs2812 g_Led;

    g_Spi.Init(s_SpiCfg);

    WS2812_CFG cfg = {
        .NbLed      = 1,
        .Order      = WS2812_ORDER_GRB,
        .Brightness = 255,
        .DevCs      = 0,
    };
    g_Led.Init(cfg, *g_Spi);

    uint32_t pix = 0x00FF00;     // green (0xRRGGBB regardless of wire order)
    g_Led.Level(&pix, 1);

@author	Hoang Nguyen Hoan (IOsonata project)
@date	May 2026

@license

MIT

----------------------------------------------------------------------------*/

#ifndef __LED_WS2812_H__
#define __LED_WS2812_H__

#include <stdint.h>

#include "device_intrf.h"
#include "miscdev/led.h"

/* Convenience colour constants in 0xRRGGBB form (independent of wire order). */
#define WS2812_OFF      0x000000UL
#define WS2812_RED      0xFF0000UL
#define WS2812_GREEN    0x00FF00UL
#define WS2812_BLUE     0x0000FFUL
#define WS2812_WHITE    0xFFFFFFUL

/* Hardware byte order on the wire.  Most WS2812B and SK6812 expect GRB.
 * A few clones (older WS2811) want RGB.  The Level() function takes
 * 0xRRGGBB regardless and the driver re-orders before transmission. */
typedef enum __WS2812_Order {
    WS2812_ORDER_GRB = 0,   /* WS2812B, SK6812, most NeoPixels */
    WS2812_ORDER_RGB = 1,   /* WS2811-style chips */
    WS2812_ORDER_BGR = 2,
    WS2812_ORDER_BRG = 3,
    WS2812_ORDER_RBG = 4,
    WS2812_ORDER_GBR = 5,
} WS2812_Order;

#pragma pack(push, 4)

typedef struct __WS2812_Config {
    int      NbLed;         //!< Number of LEDs in the chain
    uint8_t  Order;         //!< Wire byte order (WS2812_ORDER_*)
    uint8_t  Brightness;    //!< Master brightness 0..255 applied to every channel
    uint8_t  DevCs;         //!< SPI chip-select index (most peripherals require one,
                            //!< even though the WS2812 doesn't use it; pass 0 if
                            //!< no CS is wired)
} WS2812_CFG;

#pragma pack(pop)

#ifdef __cplusplus

/* Maximum number of LEDs the driver will encode in one Level() call.
 * Each LED takes 12 bytes of SPI buffer.  Adjust if you need longer
 * strips (driver is happy to chunk transmission, but the static
 * encoding buffer must be large enough). */
#ifndef WS2812_MAX_PIXELS
#define WS2812_MAX_PIXELS   16
#endif

#define WS2812_BYTES_PER_PIXEL  12      /* 24 WS2812 bits × 4 SPI bits = 96 SPI bits */

/// WS2812 / WS2812B / SK6812 (NeoPixel) chained RGB LED driver.
class LedWs2812 : public LedDevice {
public:
    LedWs2812() : vpIntrf(nullptr), vNbLed(0), vOrder(WS2812_ORDER_GRB),
                  vBrightness(255), vDevCs(0) {}
    virtual ~LedWs2812() {}

    /**
     * @brief   Initialise the driver.
     *
     * @param   Cfg     : WS2812 configuration (chain length, brightness,
     *                    wire order, SPI CS index)
     * @param   Intrf   : Pre-initialised DeviceIntrf (typically an SPI
     *                    object configured at ~3.2 MHz, MSB-first,
     *                    SPI mode 0).  The driver keeps a pointer; the
     *                    caller must keep it alive for the lifetime of
     *                    this object.
     * @return  true on success
     */
    bool Init(const WS2812_CFG &Cfg, DeviceIntrf &Intrf);

    /**
     * Turn the first LED on at full white.  Other LEDs in the chain
     * are left at whatever they last received (LedDevice contract;
     * see APA102 driver for the same convention).
     */
    virtual void On();

    /**
     * Turn all LEDs off.
     */
    virtual void Off();

    /**
     * Toggle is not meaningful for chain LEDs.
     */
    virtual void Toggle() {}

    /**
     * Set brightness on a single LED.  The 32-bit value is interpreted
     * as 0xRRGGBB regardless of the WS2812_ORDER configured — the
     * driver swaps bytes for the wire as needed.
     */
    virtual void Level(uint32_t Val) { Level(&Val, 1); }

    /**
     * Send a buffer of NbLeds pixels (each pixel a 32-bit 0xRRGGBB
     * value) down the chain.  After transmission, MOSI is held low
     * via a short delay so the LEDs latch the new frame.
     *
     * @param   pVal    : Pointer to NbLeds × uint32_t pixel values
     * @param   NbLeds  : Number of pixels to send (must be ≤ WS2812_MAX_PIXELS)
     */
    virtual void Level(uint32_t * const pVal, int NbLeds);

private:
    /* Encode one source byte (8 WS2812 bits, MSB first) into 4 SPI bytes
     * placed at *pOut.  Returns advanced pOut. */
    static uint8_t *EncodeByte(uint8_t in, uint8_t *pOut);

    DeviceIntrf *vpIntrf;
    int          vNbLed;
    uint8_t      vOrder;
    uint8_t      vBrightness;
    uint8_t      vDevCs;

    /* Static encoding buffer — sized for the configured maximum.
     * Each pixel = WS2812_BYTES_PER_PIXEL bytes. */
    uint8_t      vBuf[WS2812_MAX_PIXELS * WS2812_BYTES_PER_PIXEL];
};

#endif  /* __cplusplus */

#endif  /* __LED_WS2812_H__ */
