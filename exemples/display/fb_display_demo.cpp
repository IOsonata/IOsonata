/**-------------------------------------------------------------------------
@example	fb_display_demo.cpp

@brief	Frame buffer display test pattern

For a display driven from a frame buffer, an RGB TFT panel on the LCD
controller of the MCU for example. Shows in turn, about 4 seconds each:

1. Eight vertical color bars, from left to right: red, green, blue, white,
yellow, cyan, magenta, black. A wrong order of the colors is a wrong RGB
order or data line.
2. A white frame on the edge pixels of the screen and a cross in its middle.
A frame cut or moved is a wrong panel timing.
3. Text lines from printf until the screen scrolls.
4. The four orientations, the name of each at the top left.
5. With a touch panel, in landscape then in portrait, about 10 seconds each:
a dot drawn under each contact, the contacts printed. A dot away from the
finger is a wrong axis setting of the touch panel.

Each step is printed. board.h gives:

	DISPLAY_DRIVER_HEADER	header of the display controller
	DISPLAY_DRIVER			its class, a DisplayDotMatrix set up from
							DisplayCfg_t, no interface
	TFT_PINS				pin map: DCX, backlight, reset (display/display.h
							indexes, {-1, -1} when not wired), then the panel
							pins of the controller
	DISPLAY_WIDTH			panel width, pixels, landscape
	DISPLAY_HEIGHT			panel height, pixels, landscape
	DISPLAY_TIMING			panel timings, a DisplayTiming_t initializer
	DISPLAY_PIXEL_SIZE		optional, bits per pixel, 16
	DISPLAY_ORIENTATION		optional, DISPL_ORIENT_LANDSCAPE

	SDRAM_PINS				optional, the frame buffer at the start of an
							SDRAM, set up first (coredev/sdram.h) with the
							SDRAM_ macros of exemples/misc/ext_ram_test.cpp.
							Else the frame buffer is in the internal RAM.

	TOUCH_I2C_PINS			optional, I2C pins of a touch panel controller:
							SDA, SCL. A GT911, then an FT5406 is tried.
	TOUCH_I2C_DEVNO			optional, I2C instance, 0
	TOUCH_I2C_RATE			optional, 100000
	TOUCH_RST_PIN			optional, reset pin of the touch controller
	TOUCH_INT_PIN			optional, interrupt pin of the touch controller
	TOUCH_GT911_FLAGS		optional, TOUCH_FLAG_ axis settings, GT911, 0
	TOUCH_FT5406_FLAGS		optional, TOUCH_FLAG_ axis settings, FT5406, 0

With UART_RETARGET and the UART pins, the Release build prints on the UART
at 1000000 baud 8N1, the Debug build uses semihosting.

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
#include <stdio.h>
#include <stdint.h>

#include "coredev/sdram.h"
#include "coredev/uart.h"
#include "display/display.h"
#include "idelay.h"
#include "stddev.h"

#include "board.h"

#include DISPLAY_DRIVER_HEADER

#ifdef TOUCH_I2C_PINS
#include "coredev/i2c.h"
#include "display/touch_ft5406.h"
#include "display/touch_gt911.h"
#endif

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#define DEMO_STEP_MS		4000
#define DEMO_TOUCH_MS		10000
#define DEMO_TOUCH_POLL_MS	20

#ifdef UART_RETARGET
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static UART s_Uart;
#endif

#ifdef SDRAM_PINS
#ifndef SDRAM_DEVNO
#define SDRAM_DEVNO			0
#endif

static const IOPinCfg_t s_SdramPins[] = SDRAM_PINS;

static const SdramCfg_t s_SdramCfg = {
	.DevNo = SDRAM_DEVNO,
	.pIOPinMap = s_SdramPins,
	.NbIOPins = sizeof(s_SdramPins) / sizeof(IOPinCfg_t),
	.Rate = SDRAM_RATE,
	.BusWidth = SDRAM_BUS_WIDTH,
	.DevWidth = SDRAM_DEV_WIDTH,
	.NbBanks = SDRAM_NB_BANKS,
	.RowBits = SDRAM_ROW_BITS,
	.ColBits = SDRAM_COL_BITS,
	.CasLatency = SDRAM_CAS_LATENCY,
	.TwrClk = SDRAM_TWR_CLK,
	.TmrdClk = SDRAM_TMRD_CLK,
	.RefreshNs = SDRAM_REFRESH_NS,
	.TrcdNs = SDRAM_TRCD_NS,
	.TrpNs = SDRAM_TRP_NS,
	.TrasNs = SDRAM_TRAS_NS,
	.TrcNs = SDRAM_TRC_NS,
	.TrfcNs = SDRAM_TRFC_NS,
	.TrrdNs = SDRAM_TRRD_NS,
	.TwrNs = SDRAM_TWR_NS,
	.TaprNs = SDRAM_TAPR_NS,
	.TsrexNs = SDRAM_TSREX_NS,
	.TxsrNs = SDRAM_TXSR_NS,
};

static Sdram s_Sdram;
#endif

#ifndef DISPLAY_PIXEL_SIZE
#define DISPLAY_PIXEL_SIZE	16
#endif
#ifndef DISPLAY_ORIENTATION
#define DISPLAY_ORIENTATION	DISPL_ORIENT_LANDSCAPE
#endif

#define DISPLAY_FB_SIZE		((uint32_t)DISPLAY_WIDTH * DISPLAY_HEIGHT * DISPLAY_PIXEL_SIZE / 8U)

#ifndef SDRAM_PINS
alignas(8) static uint8_t s_FrameBuffer[DISPLAY_FB_SIZE];
#endif

static const IOPinCfg_t s_DisplayPins[] = TFT_PINS;
static const DisplayTiming_t s_DisplayTiming = DISPLAY_TIMING;

static DisplayCfg_t s_DisplayCfg = {
	.DevAddr = 0,
	.pPins = s_DisplayPins,
	.NbPins = sizeof(s_DisplayPins) / sizeof(IOPinCfg_t),
	.Stride = (uint16_t)(DISPLAY_WIDTH * DISPLAY_PIXEL_SIZE / 8),
	.Width = DISPLAY_WIDTH,
	.Height = DISPLAY_HEIGHT,
	.PixelSize = DISPLAY_PIXEL_SIZE,
	.Orient = DISPLAY_ORIENTATION,
	.pTiming = &s_DisplayTiming,
#ifdef SDRAM_PINS
	.pFrameBuffer = NULL,			// the SDRAM, once set up
#else
	.pFrameBuffer = s_FrameBuffer,
#endif
};

static DISPLAY_DRIVER s_Display;

#ifdef TOUCH_I2C_PINS
#ifndef TOUCH_I2C_DEVNO
#define TOUCH_I2C_DEVNO		0
#endif
#ifndef TOUCH_I2C_RATE
#define TOUCH_I2C_RATE		100000
#endif
#ifndef TOUCH_RST_PIN
#define TOUCH_RST_PIN		{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}
#endif
#ifndef TOUCH_INT_PIN
#define TOUCH_INT_PIN		{-1, -1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}
#endif
#ifndef TOUCH_GT911_FLAGS
#define TOUCH_GT911_FLAGS	0
#endif
#ifndef TOUCH_FT5406_FLAGS
#define TOUCH_FT5406_FLAGS	0
#endif

static const IOPinCfg_t s_TouchI2cPins[] = TOUCH_I2C_PINS;

static const I2CCfg_t s_TouchI2cCfg = {
	.DevNo = TOUCH_I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_TouchI2cPins,
	.NbIOPins = sizeof(s_TouchI2cPins) / sizeof(IOPinCfg_t),
	.Rate = TOUCH_I2C_RATE,
	.MaxRetry = 0,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = { 0 },
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = NULL,
};

static const TouchCfg_t s_Gt911Cfg = {
	.DevAddr = 0,
	.Width = DISPLAY_WIDTH,
	.Height = DISPLAY_HEIGHT,
	.Flags = TOUCH_GT911_FLAGS,
	.RstPin = TOUCH_RST_PIN,
	.IntPin = TOUCH_INT_PIN,
};

static const TouchCfg_t s_Ft5406Cfg = {
	.DevAddr = 0,
	.Width = DISPLAY_WIDTH,
	.Height = DISPLAY_HEIGHT,
	.Flags = TOUCH_FT5406_FLAGS,
	.RstPin = TOUCH_RST_PIN,
	.IntPin = TOUCH_INT_PIN,
};

static I2C s_TouchI2c;
static TouchGt911 s_Gt911;
static TouchFt5406 s_Ft5406;

// The controller of the panel: a GT911, else an FT5406
static Touch *TouchInit(void)
{
	if (s_TouchI2c.Init(s_TouchI2cCfg) == false)
	{
		printf("Touch I2C init failed\r\n");

		return NULL;
	}
	if (s_Gt911.Init(s_Gt911Cfg, &s_TouchI2c))
	{
		printf("Touch: GT911\r\n");

		return &s_Gt911;
	}
	if (s_Ft5406.Init(s_Ft5406Cfg, &s_TouchI2c))
	{
		printf("Touch: FT5406, chip id 0x%02X\r\n", (unsigned)s_Ft5406.DeviceID());

		return &s_Ft5406;
	}
	printf("Touch: no GT911, no FT5406\r\n");

	return NULL;
}
#endif

// RGB565
#define COLOR_BLACK			0x0000U
#define COLOR_RED			0xF800U
#define COLOR_GREEN			0x07E0U
#define COLOR_BLUE			0x001FU
#define COLOR_WHITE			0xFFFFU
#define COLOR_YELLOW		0xFFE0U
#define COLOR_CYAN			0x07FFU
#define COLOR_MAGENTA		0xF81FU

static void ColorBars(DisplayDotMatrix *pDisp)
{
	static const uint16_t colors[] = {
		COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_YELLOW, COLOR_CYAN, COLOR_MAGENTA, COLOR_BLACK,
	};
	uint16_t w = pDisp->Width();
	uint16_t h = pDisp->Height();

	for (int i = 0; i < 8; i++)
	{
		uint16_t x0 = (uint16_t)(w * i / 8);
		uint16_t x1 = (uint16_t)(w * (i + 1) / 8);

		pDisp->Fill(x0, 0, (uint16_t)(x1 - x0), h, colors[i]);
	}
	printf("Color bars, left to right: red, green, blue, white, yellow, cyan, magenta, black\r\n");
}

static void Frame(DisplayDotMatrix *pDisp)
{
	uint16_t w = pDisp->Width();
	uint16_t h = pDisp->Height();

	pDisp->Clear();
	pDisp->Fill(0, 0, w, 1, COLOR_WHITE);
	pDisp->Fill(0, (uint16_t)(h - 1), w, 1, COLOR_WHITE);
	pDisp->Fill(0, 0, 1, h, COLOR_WHITE);
	pDisp->Fill((uint16_t)(w - 1), 0, 1, h, COLOR_WHITE);
	pDisp->Fill(0, (uint16_t)(h / 2), w, 1, COLOR_GREEN);
	pDisp->Fill((uint16_t)(w / 2), 0, 1, h, COLOR_GREEN);
	printf("White frame on the edge pixels, green cross in the middle\r\n");
}

static void Text(DisplayDotMatrix *pDisp)
{
	pDisp->Clear();
	printf("Text lines until the screen scrolls\r\n");
	for (int i = 0; i < pDisp->Height() / 8 + 10; i++)
	{
		pDisp->printf("Line %d of the IOsonata display test, %ux%u\r\n", i, pDisp->Width(), pDisp->Height());
		msDelay(50);
	}
}

static void Orientations(DisplayDotMatrix *pDisp)
{
	static const struct {
		DISPL_ORIENT Orient;
		const char *pName;
	} o[] = {
		{ DISPL_ORIENT_LANDSCAPE, "LANDSCAPE" },
		{ DISPL_ORIENT_PORTRAIT, "PORTRAIT" },
		{ DISPL_ORIENT_LANDSCAPE_INV, "LANDSCAPE INVERTED" },
		{ DISPL_ORIENT_PORTRAIT_INV, "PORTRAIT INVERTED" },
	};
	DISPL_ORIENT org = pDisp->Orientation();

	for (auto &e : o)
	{
		pDisp->Orientation(e.Orient);
		pDisp->Clear();
		pDisp->Fill(0, 0, 40, 4, COLOR_RED);
		pDisp->SetCursor(0, 16);
		pDisp->printf("%s %ux%u", e.pName, pDisp->Width(), pDisp->Height());
		printf("%s: red bar and the name at the top left\r\n", e.pName);
		msDelay(DEMO_STEP_MS / 2);
	}
	pDisp->Orientation(org);
}

#ifdef TOUCH_I2C_PINS
// A dot under each contact, one color per track id
static void TouchStep(DisplayDotMatrix *pDisp, Touch *pTouch, DISPL_ORIENT Orient, const char *pName)
{
	static const uint16_t colors[] = { COLOR_WHITE, COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW };
	DISPL_ORIENT org = pDisp->Orientation();
	TouchPoint_t pt[5];
	int last = 0;

	pDisp->Orientation(Orient);
	pTouch->Orientation(Orient);
	pDisp->Clear();
	pDisp->SetCursor(0, 0);
	pDisp->printf("%s: touch the screen", pName);
	printf("Touch, %s %ux%u: a dot under each contact\r\n", pName, pDisp->Width(), pDisp->Height());

	for (int t = 0; t < DEMO_TOUCH_MS; t += DEMO_TOUCH_POLL_MS)
	{
		int n = pTouch->Read(pt, 5);

		if (n < 0)
		{
			if (last >= 0)
			{
				printf("Touch read failed\r\n");
			}
		}
		else
		{
			for (int i = 0; i < n; i++)
			{
				uint16_t x = pt[i].X > 2 ? pt[i].X - 2 : 0;
				uint16_t y = pt[i].Y > 2 ? pt[i].Y - 2 : 0;

				pDisp->Fill(x, y, 5, 5, colors[pt[i].Id % 5]);
			}
			// The contacts when their number changes and every 0.5 s
			if (n > 0 && (n != last || t % 500 == 0))
			{
				printf("%d:", n);
				for (int i = 0; i < n; i++)
				{
					printf(" id %u (%u, %u)", pt[i].Id, pt[i].X, pt[i].Y);
				}
				printf("\r\n");
			}
			else if (n == 0 && last > 0)
			{
				printf("released\r\n");
			}
		}
		last = n;
		msDelay(DEMO_TOUCH_POLL_MS);
	}
	pDisp->Orientation(org);
	pTouch->Orientation(org);
}
#endif

int main()
{
#ifdef UART_RETARGET
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
#endif

	printf("\r\nFrame buffer display test\r\n");

#ifdef SDRAM_PINS
	if (s_Sdram.Init(s_SdramCfg) == false)
	{
		printf("SDRAM init failed\r\n");

		return 1;
	}
	printf("SDRAM %lu KB at 0x%08lX, clock %lu Hz\r\n", (unsigned long)(s_Sdram.Size() / 1024UL),
		   (unsigned long)(uintptr_t)s_Sdram.Base(), (unsigned long)s_Sdram.Rate());
	if (s_Sdram.Size() < DISPLAY_FB_SIZE)
	{
		printf("SDRAM too small for the frame buffer\r\n");

		return 1;
	}
	s_DisplayCfg.pFrameBuffer = s_Sdram.Base();
#endif

	DisplayDotMatrix *disp = &s_Display;

	if (s_Display.Init(s_DisplayCfg, NULL) == false)
	{
		printf("Display init failed\r\n");

		return 1;
	}
	printf("%u x %u pixels, frame buffer at 0x%08lX\r\n", disp->Width(), disp->Height(),
		   (unsigned long)(uintptr_t)s_DisplayCfg.pFrameBuffer);
#ifdef TOUCH_I2C_PINS
	Touch *touch = TouchInit();
#endif
	disp->Backlight(true);

	while (1)
	{
		ColorBars(disp);
		msDelay(DEMO_STEP_MS);
		Frame(disp);
		msDelay(DEMO_STEP_MS);
		Text(disp);
		msDelay(DEMO_STEP_MS);
		Orientations(disp);
#ifdef TOUCH_I2C_PINS
		if (touch != NULL)
		{
			TouchStep(disp, touch, DISPL_ORIENT_LANDSCAPE, "LANDSCAPE");
			TouchStep(disp, touch, DISPL_ORIENT_PORTRAIT, "PORTRAIT");
		}
#endif
	}
}
