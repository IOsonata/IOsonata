// SSD1315 SPI bus model: exercise the real driver, display/font base and
// DeviceIntrf locking. Check physical RAM rather than private driver state.
#include <cassert>
#include <cstdio>
#include <cstring>
#include "display/display_ssd1315.h"

static bool dataMode, selected, refuse, failData;
static unsigned starts, stops, sends, dcWrites, resetWrites;
static uint8_t ram[8][128];
static unsigned page, column;
static bool pageMode, displayOn;

extern "C" void IOPinConfig(int Port, int Pin, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert(Port == 0 && (Pin == 0 || Pin == 2));
}

extern "C" void IOPinSet(int Port, int Pin)
{
	assert(Port == 0 && (Pin == 0 || Pin == 2));
	if (Pin == 0)
	{
		dataMode = true;
		dcWrites++;
	}
	else
	{
		resetWrites++;
	}
}

extern "C" void IOPinClear(int Port, int Pin)
{
	assert(Port == 0 && (Pin == 0 || Pin == 2));
	if (Pin == 0)
	{
		dataMode = false;
		dcWrites++;
	}
	else
	{
		resetWrites++;
	}
}

static bool Start(DevIntrf_t *, uint32_t Address)
{
	assert(Address == 0 && !selected);
	if (refuse)
	{
		return false;
	}
	selected = true;
	starts++;
	return true;
}

static void Stop(DevIntrf_t *)
{
	assert(selected);
	selected = false;
	stops++;
}

static int Tx(DevIntrf_t *, const uint8_t *p, int n)
{
	assert(selected && n > 0);
	sends++;
	if (failData)
	{
		return n - 1;
	}
	if (dataMode)
	{
		assert(pageMode);
		for (int i = 0; i < n; i++)
		{
			assert(page < 8 && column < 128);
			ram[page][column++] = p[i];
		}
	}
	else
	{
		for (int i = 0; i < n; i++)
		{
			uint8_t cmd = p[i];
			if (cmd == 0x20 || cmd == 0xD5 || cmd == 0xA8 || cmd == 0xD3 ||
				cmd == 0x8D || cmd == 0xDA || cmd == 0x81 || cmd == 0xD9 || cmd == 0xDB)
			{
				assert(++i < n);
				if (cmd == 0x20)
				{
					assert(p[i] == 2);
					pageMode = true;
				}
				if (cmd == 0xA8)
				{
					assert(p[i] >= 15 && p[i] <= 63);
				}
				if (cmd == 0xDB)
				{
					assert((p[i] & ~0x30U) == 0);
				}
			}
			else if ((cmd & 0xF8) == 0xB0)
			{
				page = cmd & 7;
			}
			else if ((cmd & 0xF0) == 0x00)
			{
				column = (column & 0xF0) | (cmd & 15);
			}
			else if ((cmd & 0xF0) == 0x10)
			{
				column = (column & 15) | ((cmd & 15) << 4);
			}
			else if (cmd == 0xAF)
			{
				displayOn = true;
			}
			else if (cmd == 0xAE)
			{
				displayOn = false;
			}
		}
	}
	return n;
}

class Bus : public DeviceIntrf {
public:
	DevIntrf_t dev{};

	Bus()
	{
		dev.Type = DEVINTRF_TYPE_SPI;
		dev.StartTx = Start;
		dev.StopTx = Stop;
		dev.TxData = ::Tx;
		atomic_flag_clear(&dev.bBusy);
	}

	operator DevIntrf_t *() override
	{
		return &dev;
	}

	uint32_t Rate(uint32_t r) override
	{
		return r;
	}

	uint32_t Rate() override
	{
		return 4000000;
	}
};

static bool On(unsigned x, unsigned y)
{
	return ram[y / 8][x] & (1U << (y & 7));
}

int main()
{
	const IOPinCfg_t pins[] = {
		{ 0, 0, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
		{ -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
		{ 0, 2, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }
	};
	DisplayCfg_t cfg = { 0, pins, 3, 16, 128, 64, 1, DISPL_ORIENT_LANDSCAPE };
	Bus bus;
	LcdSSD1315 oled;
	assert(!oled.Init(cfg, nullptr));
	bus.dev.bIntEn = true;
	assert(!oled.Init(cfg, &bus));
	bus.dev.bIntEn = false;
	bus.dev.bDma = true;
	assert(!oled.Init(cfg, &bus));
	bus.dev.bDma = false;
	refuse = true;
	assert(!oled.Init(cfg, &bus));
	refuse = false;
	assert(oled.Init(cfg, &bus) && oled.Valid());
	assert(starts == stops && resetWrites && dcWrites && displayOn);
	for (const auto &p : ram)
	{
		for (uint8_t b : p)
		{
			assert(b == 0);
		}
	}

	// Updating a byte must preserve adjacent pixels, including page edges.
	oled.SetPixel(5, 7, 1);
	oled.SetPixel(5, 8, 1);
	oled.SetPixel(5, 9, 1);
	oled.SetPixel(5, 8, 0);
	assert(On(5, 7) && !On(5, 8) && On(5, 9));
	unsigned before = sends;
	oled.SetPixel(128, 0, 1);
	oled.SetPixel(0, 64, 1);
	assert(before == sends);
	oled.Fill(126, 62, 65535, 65535, 1);
	assert(On(127, 63) && On(126, 62));

	// Clipping retains the source bitmap's original stride, including padding.
	uint8_t bitmap[] = { 0x80, 0x80, 0x40, 0x00 }; // Two rows, nine pixels
	oled.Clear();
	oled.BitBlt(125, 7, 9, 2, bitmap);
	assert(On(125, 7) && !On(126, 7) && !On(125, 8) && On(126, 8));
	oled.Scroll(DISPL_SCROLL_DIR_LEFT, 1);
	assert(On(124, 7) && On(125, 8) && !On(126, 8));
	oled.Scroll(DISPL_SCROLL_DIR_DOWN, 1);
	assert(On(124, 8) && On(125, 9));
	oled.Scroll(DISPL_SCROLL_DIR_RIGHT, 1);
	oled.Scroll(DISPL_SCROLL_DIR_UP, 1);
	assert(On(125, 7) && On(126, 8));
	oled.Scroll(DISPL_SCROLL_DIR_UP, 65535);
	for (const auto &p : ram)
	{
		for (uint8_t b : p)
		{
			assert(b == 0);
		}
	}

	for (int o = 0; o < 4; o++)
	{
		oled.Clear();
		oled.Orientation((DISPL_ORIENT)o);
		oled.SetPixel(0, 0, 1);
		unsigned x = o == DISPL_ORIENT_PORTRAIT || o == DISPL_ORIENT_LANDSCAPE_INV ? 127 : 0;
		unsigned y = o == DISPL_ORIENT_PORTRAIT_INV || o == DISPL_ORIENT_LANDSCAPE_INV ? 63 : 0;
		assert(On(x, y));
		assert(oled.Width() == (o == 0 || o == 2 ? 64 : 128));
	}
	oled.Orientation(DISPL_ORIENT_LANDSCAPE);
	oled.Clear();
	assert(oled.SetFont(&iFontSystem5x7));
	oled.printf("!");
	assert(ram[0][2] == 0x5F); // Existing IOsonata system font
	// The shared console demo also uses proportional fonts. Render all of
	// its printable characters with actual small and large font tables.
	const FontDesc_t *fonts[] = { &iFontFreeMono8pt, &iFontFreeSansBold24pt };
	for (const FontDesc_t *font : fonts)
	{
		assert(oled.SetFont(font));
		oled.Clear();
		for (int ch = '!'; ch <= '~'; ch++)
		{
			oled.printf("%c", ch);
		}
	}
	assert(oled.SetFont(&iFontSystem5x7));
	assert(!oled.SetFont(nullptr));
	FontDesc_t huge = iFontSystem5x7;
	huge.Height = 255;
	assert(!oled.SetFont(&huge));
	oled.SetCursor(127, 63);
	oled.printf("Hello\nWorld\r!");
	assert(oled.Refresh());
	failData = true;
	assert(!oled.Refresh() && !selected && starts == stops);
	assert(!oled.Contrast(32));
	failData = false;
	assert(oled.Refresh() && oled.Contrast(32));
	refuse = true;
	assert(!oled.Refresh());
	refuse = false;
	assert(oled.Refresh());
	oled.Disable();
	assert(!displayOn);
	assert(oled.Enable() && displayOn);
	oled.Reset();
	assert(oled.Valid());
	cfg.NbPins = 1;
	assert(oled.Init(cfg, &bus)); // No reset/backlight entries
	cfg.Height = 32;
	assert(oled.Init(cfg, &bus));
	cfg.Height = 8;
	assert(!oled.Init(cfg, &bus));
	cfg.Height = 65;
	assert(!oled.Init(cfg, &bus));
	assert(!oled.Refresh());
	assert(starts == stops);
	std::puts("SSD1315 SPI RAM, pixels, clipping, rotation, scrolling, fonts and failure recovery: PASS");
}
