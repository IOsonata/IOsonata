/**-------------------------------------------------------------------------
@example ssd1315_demo.cpp
@brief SSD1315 text, graphics and counter using the IOsonata display API.
@license MIT, (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include "board.h"
#include "coredev/spi.h"
#include "display/display_ssd1315.h"
#include "idelay.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_SpiPins[] = {
	{OLED_SCK_PORT, OLED_SCK_PIN, OLED_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{-1, -1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, // No MISO
	{OLED_MOSI_PORT, OLED_MOSI_PIN, OLED_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{OLED_CS_PORT, OLED_CS_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const SPICfg_t s_SpiCfg = {
	OLED_SPI_DEVNO, SPIPHY_NORMAL, SPIMODE_MASTER,
	s_SpiPins, sizeof(s_SpiPins) / sizeof(s_SpiPins[0]),
	4000000, 8, 5, SPIDATABIT_MSB, SPIDATAPHASE_FIRST_CLK,
	SPICLKPOL_LOW, SPICSEL_AUTO, false, false, 6, 0xFF, nullptr
};

static const IOPinCfg_t s_DisplayPins[] = {
	{OLED_DC_PORT, OLED_DC_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, // No backlight
	{OLED_RST_PORT, OLED_RST_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const DisplayCfg_t s_DisplayCfg = {
	0, s_DisplayPins, sizeof(s_DisplayPins) / sizeof(s_DisplayPins[0]),
	16, 128, 64, 1, DISPL_ORIENT_LANDSCAPE
};

static SPI s_Spi;
static LcdSSD1315 s_Display;

int main()
{
	if (!s_Spi.Init(s_SpiCfg) || !s_Display.Init(s_DisplayCfg, &s_Spi))
	{
		return 1;
	}
	s_Display.SetCursor(0, 0);
	s_Display.printf("IOsonata SSD1315\n128 x 64 OLED\nSPI display demo");
	// Border and checkerboard prove page boundaries and both pixel states.
	s_Display.Fill(0, 30, 128, 1, 1);
	s_Display.Fill(0, 63, 128, 1, 1);
	s_Display.Fill(0, 30, 1, 34, 1);
	s_Display.Fill(127, 30, 1, 34, 1);
	for (unsigned y = 0; y < 3; y++)
	{
		for (unsigned x = 0; x < 8; x++)
		{
			s_Display.Fill(4 + x * 4, 34 + y * 4, 4, 4, (x + y) & 1);
		}
	}
	for (unsigned count = 0;; count++)
	{
		s_Display.Fill(44, 34, 78, 20, 0);
		s_Display.SetCursor(44, 34);
		s_Display.printf("Count %u", count);
		msDelay(500);
	}
}
