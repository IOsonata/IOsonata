// Host regression of the LPC546xx LCD controller driver, a display controller
// set up from DisplayCfg_t, with a register model: the configuration refused,
// the registers of the LPCXpresso54628 panel and of other panels (timings,
// closest panel clock, polarities, RGB order), the order of the output enable
// and of the panel power, the timing registers written with the output off,
// the clocks, pins, backlight and panel reset pin, then
// the frame buffer in the four orientations: pixels, Fill, BitBlt with
// clipping, Scroll up and down, text from the IOsonata font, Clear, Reset
// and PowerOff.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

#include "LPC546xx.h"

void NVIC_ClearPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPendingIRQ(IRQn_Type n) { (void)n; }
void NVIC_SetPriority(IRQn_Type n, uint32_t p) { (void)n; (void)p; }
void NVIC_EnableIRQ(IRQn_Type n) { (void)n; }
void NVIC_DisableIRQ(IRQn_Type n) { (void)n; }
void __DMB(void) {}
void __DSB(void) {}
void __NOP(void) {}
void ModelDelayUs(uint32_t Us) { (void)Us; }

static uint32_t s_CoreClk = 220000000;
extern "C" uint32_t SystemCoreClockGet(void) { return s_CoreClk; }

// ---- LCD controller model ----

static uint32_t s_Ctrl;
static int s_Enables, s_PowerOns;

static void CheckOff(void)
{
	// The timing and the frame base are written with the output off
	assert((s_Ctrl & LCD_CTRL_LCDEN_MASK) == 0U);
}

struct CfgR {
	uint32_t v = 0;
	operator uint32_t() const { return v; }
	CfgR &operator=(uint32_t x) { CheckOff(); v = x; return *this; }
};
struct CtrlR {
	operator uint32_t() const { return s_Ctrl; }
	CtrlR &operator=(uint32_t x)
	{
		bool en = (x & LCD_CTRL_LCDEN_MASK) != 0U, pwr = (x & LCD_CTRL_LCDPWR_MASK) != 0U;
		bool was_en = (s_Ctrl & LCD_CTRL_LCDEN_MASK) != 0U, was_pwr = (s_Ctrl & LCD_CTRL_LCDPWR_MASK) != 0U;

		// The panel power on after the timing, off before it
		assert(!pwr || was_en);
		assert(en || !was_pwr || !pwr);
		assert(en || !was_en || !was_pwr);
		// The format changes with the output off
		assert(!was_en || ((x ^ s_Ctrl) & ~(LCD_CTRL_LCDEN_MASK | LCD_CTRL_LCDPWR_MASK)) == 0U);
		s_Enables += en && !was_en;
		s_PowerOns += pwr && !was_pwr;
		s_Ctrl = x;
		return *this;
	}
	CtrlR &operator|=(uint32_t x) { return *this = s_Ctrl | x; }
	CtrlR &operator&=(uint32_t x) { return *this = s_Ctrl & x; }
};
struct LcdRegs {
	CfgR TIMH, TIMV, POL, LE, UPBASE, LPBASE;
	CtrlR CTRL;
	uint32_t INTMSK = 0xFFFFFFFFU, INTCLR = 0, CRSR_CTRL = 0xFFFFFFFFU, CRSR_INTMSK = 0xFFFFFFFFU;
};
static LcdRegs s_Lcd;

alignas(8) static uint8_t s_SysconMem[sizeof(SYSCON_Type)];
alignas(8) static uint8_t s_IoconMem[sizeof(IOCON_Type)];
alignas(8) static uint8_t s_GpioMem[sizeof(GPIO_Type)];
alignas(8) static uint8_t s_PintMem[sizeof(PINT_Type)];
alignas(8) static uint8_t s_InputMuxMem[sizeof(INPUTMUX_Type)];

#undef IOCON
#undef GPIO
#undef SYSCON
#undef PINT
#undef INPUTMUX
#undef LCD
#define IOCON ((IOCON_Type *)s_IoconMem)
#define GPIO ((GPIO_Type *)s_GpioMem)
#define SYSCON ((SYSCON_Type *)s_SysconMem)
#define PINT ((PINT_Type *)s_PintMem)
#define INPUTMUX ((INPUTMUX_Type *)s_InputMuxMem)
#define LCD (&s_Lcd)

#include "../../../ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp"
#include "../../../ARM/NXP/LPC546xx/src/lcd_lpc546xx.cpp"

static SYSCON_Type *Sys() { return (SYSCON_Type *)s_SysconMem; }
static GPIO_Type *Gpio() { return (GPIO_Type *)s_GpioMem; }
static IOCON_Type *Iocon() { return (IOCON_Type *)s_IoconMem; }

// ---- Tests ----

#define PW	480U
#define PH	272U

// The display control pins: no DCX, the backlight P3_31, no reset. Then the
// LCD pins of the LPCXpresso54628: P2_11 LCD_PWR, P2_13 DCLK, P2_14 FP, P2_15
// AC, P2_16 LP, VD3 to VD7, VD10 to VD15, VD19 to VD23 (NXP SDK pin_mux.c)
#define NB_CTRL_PINS	3
static const IOPinCfg_t s_Pins[] = {
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 31, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 11, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 13, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 14, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 15, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 16, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 21, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{3, 9, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(8) static uint16_t s_Fb[PW * PH + 8];

// The panel timings, set back to the board values by Cfg
static DisplayTiming_t s_T;

static DisplayCfg_t Cfg(void)
{
	DisplayCfg_t c;

	memset(&c, 0, sizeof(c));
	c.DevAddr = 0x5A;			// not used
	c.Stride = 7;				// not used
	c.pPins = s_Pins;
	c.NbPins = sizeof(s_Pins) / sizeof(s_Pins[0]);
	c.Width = PW;
	c.Height = PH;
	c.PixelSize = 16;
	c.Orient = DISPL_ORIENT_LANDSCAPE;
	s_T.PixelClock = 9000000;
	s_T.HSync = 2;
	s_T.HFront = 8;
	s_T.HBack = 43;
	s_T.VSync = 10;
	s_T.VFront = 4;
	s_T.VBack = 12;
	s_T.Flags = DISPL_TIMING_VSYNC_LOW | DISPL_TIMING_HSYNC_LOW;
	c.pTiming = &s_T;
	c.pFrameBuffer = s_Fb;
	return c;
}

static void Fresh(void)
{
	memset(s_SysconMem, 0, sizeof(s_SysconMem));
	memset(s_IoconMem, 0, sizeof(s_IoconMem));
	memset(s_GpioMem, 0, sizeof(s_GpioMem));
	s_Ctrl = 0;
	s_Enables = 0;
	s_PowerOns = 0;
	s_Lcd = LcdRegs();
	// Values the driver writes over
	s_Lcd.LE.v = 0xFFFFFFFFU;
	s_Lcd.LPBASE.v = 0x12345678U;
	Sys()->LCDCLKSEL = 3;
	Sys()->LCDCLKDIV = 0x55;
	s_CoreClk = 220000000;
	for (uint32_t i = 0; i < sizeof(s_Fb) / sizeof(s_Fb[0]); i++)
	{
		s_Fb[i] = 0xA5A5;
	}
}

static uint32_t Pcd(uint32_t Pol)
{
	return (Pol & LCD_POL_PCD_LO_MASK) | (((Pol & LCD_POL_PCD_HI_MASK) >> LCD_POL_PCD_HI_SHIFT) << 5);
}

// Frame buffer index of a pixel of the screen in an orientation
static uint32_t Idx(DISPL_ORIENT O, uint32_t X, uint32_t Y)
{
	switch (O)
	{
		case DISPL_ORIENT_LANDSCAPE_INV:
			return (PH - 1 - Y) * PW + (PW - 1 - X);
		case DISPL_ORIENT_PORTRAIT:
			return X * PW + (PW - 1 - Y);
		case DISPL_ORIENT_PORTRAIT_INV:
			return (PH - 1 - X) * PW + Y;
		default:
			return Y * PW + X;
	}
}

static void TestInitChecks()
{
	LcdLpc546xx lcd;
	DisplayCfg_t c;
	DisplayCfg_t dc;

	Fresh();
	memset(&dc, 0, sizeof(dc));
	assert(!lcd.Init(dc, NULL));
	static const struct { uint16_t DisplayCfg_t::*f; uint16_t v; } badCfg[] = {
		{ &DisplayCfg_t::Width, 0 }, { &DisplayCfg_t::Width, 488 }, { &DisplayCfg_t::Width, 1040 },
		{ &DisplayCfg_t::Height, 0 }, { &DisplayCfg_t::Height, 1025 },
	};
	for (auto &e : badCfg)
	{
		c = Cfg();
		c.*e.f = e.v;
		assert(!lcd.Init(c, NULL));
	}
	static const struct { uint16_t DisplayTiming_t::*f; uint16_t v; } badTiming[] = {
		{ &DisplayTiming_t::HSync, 0 }, { &DisplayTiming_t::HSync, 257 },
		{ &DisplayTiming_t::HFront, 0 }, { &DisplayTiming_t::HFront, 257 },
		{ &DisplayTiming_t::HBack, 0 }, { &DisplayTiming_t::HBack, 257 },
		{ &DisplayTiming_t::VSync, 0 }, { &DisplayTiming_t::VSync, 65 },
		{ &DisplayTiming_t::VFront, 0 }, { &DisplayTiming_t::VFront, 257 },
		{ &DisplayTiming_t::VBack, 0 }, { &DisplayTiming_t::VBack, 257 },
	};
	for (auto &e : badTiming)
	{
		c = Cfg();
		s_T.*e.f = e.v;
		assert(!lcd.Init(c, NULL));
	}
	static const uint8_t badSize[] = { 0, 1, 8, 18, 24, 32 };
	for (uint8_t sz : badSize)
	{
		c = Cfg();
		c.PixelSize = sz;
		assert(!lcd.Init(c, NULL));
	}
	c = Cfg();
	c.pTiming = NULL;
	assert(!lcd.Init(c, NULL));
	c = Cfg();
	s_T.PixelClock = 0;
	assert(!lcd.Init(c, NULL));
	c = Cfg();
	c.pFrameBuffer = NULL;
	assert(!lcd.Init(c, NULL));
	c = Cfg();
	c.pFrameBuffer = s_Fb + 1;
	assert(!lcd.Init(c, NULL));
	c = Cfg();
	c.pPins = NULL;
	assert(!lcd.Init(c, NULL));
	c = Cfg();
	c.NbPins = -1;
	assert(!lcd.Init(c, NULL));
	// Nothing touched
	assert(s_Ctrl == 0 && Sys()->AHBCLKCTRLSET[2] == 0 && s_Fb[0] == 0xA5A5);
	assert(!lcd.Enable());
	lcd.Disable();
	lcd.PowerOff();
	lcd.Clear();
	lcd.Fill(0, 0, 10, 10, 0xFFFF);
	lcd.SetPixel(0, 0, 0xFFFF);
	lcd.Scroll(DISPL_SCROLL_DIR_UP, 1);
	lcd.Backlight(true);
	lcd.Reset();
	assert(s_Fb[0] == 0xA5A5 && s_Ctrl == 0 && Gpio()->SET[3] == 0);
	// Limits taken, no pin
	c = Cfg();
	c.Width = 1024;
	c.Height = 1;
	s_T.HSync = s_T.HFront = s_T.HBack = 256;
	s_T.VSync = 64;
	s_T.VFront = s_T.VBack = 256;
	c.NbPins = 0;
	c.pPins = NULL;
	assert(lcd.Init(c, NULL));
	assert(s_Lcd.TIMH == (LCD_TIMH_PPL(63) | LCD_TIMH_HSW(255) | LCD_TIMH_HFP(255) | LCD_TIMH_HBP(255)));
	assert(s_Lcd.TIMV == (LCD_TIMV_LPP(0) | LCD_TIMV_VSW(63) | LCD_TIMV_VFP(255) | LCD_TIMV_VBP(255)));
	lcd.Backlight(true);
	lcd.Reset();
	lcd.PowerOff();
	// The control pins absent, or out of the pin count: the backlight and
	// the reset do nothing
	static const IOPinCfg_t none[3] = {
		{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{3, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{-1, 4, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	};
	static const IOPinCfg_t ctl[3] = {
		{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{3, 31, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{2, 27, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	};
	static const struct { const IOPinCfg_t *p; int n; uint32_t Port2, Port3; } t[] = {
		{ none, 1, 0, 0 }, { none, 2, 0, 0 }, { none, 3, 0, 0 },
		{ ctl, 1, 0, 0 }, { ctl, 2, 0, 1UL << 31 }, { ctl, 3, 1UL << 27, 1UL << 31 },
	};
	for (auto &e : t)
	{
		Fresh();
		c = Cfg();
		c.pPins = e.p;
		c.NbPins = e.n;
		assert(lcd.Init(c, NULL));
		lcd.Backlight(true);
		lcd.Reset();
		lcd.Backlight(false);
		assert(Gpio()->SET[2] == e.Port2 && Gpio()->CLR[2] == e.Port2);
		assert(Gpio()->SET[3] == e.Port3 && Gpio()->CLR[3] == e.Port3);
		assert(Gpio()->SET[0] == 0 && Gpio()->CLR[0] == 0 && Gpio()->SET[1] == 0 && Gpio()->CLR[1] == 0);
		lcd.PowerOff();
	}
	printf("init checks: PASS\n");
}

static void TestRegisters()
{
	LcdLpc546xx lcd;

	// LPCXpresso54628 panel, as the NXP SDK lcdc examples
	Fresh();
	DisplayCfg_t c = Cfg();
	assert(lcd.Init(c, NULL));
	assert(s_Lcd.TIMH == (LCD_TIMH_PPL(29) | LCD_TIMH_HSW(1) | LCD_TIMH_HFP(7) | LCD_TIMH_HBP(42)));
	assert(s_Lcd.TIMV == (LCD_TIMV_LPP(271) | LCD_TIMV_VSW(9) | LCD_TIMV_VFP(3) | LCD_TIMV_VBP(11)));
	// 220 MHz / 24: PCD 22
	assert(s_Lcd.POL == (LCD_POL_PCD_LO(22) | LCD_POL_CPL(479) | LCD_POL_IVS_MASK | LCD_POL_IHS_MASK));
	assert(lcd.PixelClock() == 220000000 / 24);
	assert(s_Lcd.LE == 0 && s_Lcd.UPBASE == (uint32_t)(uintptr_t)s_Fb && s_Lcd.LPBASE == 0);
	assert(s_Ctrl == (LCD_CTRL_LCDTFT_MASK | LCD_CTRL_LCDBPP(6) | LCD_CTRL_BGR_MASK | LCD_CTRL_LCDEN_MASK |
					  LCD_CTRL_LCDPWR_MASK));
	assert(s_Enables == 1 && s_PowerOns == 1);
	assert(s_Lcd.INTMSK == 0 && s_Lcd.CRSR_CTRL == 0 && s_Lcd.CRSR_INTMSK == 0);
	assert((s_Lcd.INTCLR & 0x1EU) == 0x1EU);
	// Clocks: main clock not divided, the controller clock on and out of reset
	assert(Sys()->LCDCLKSEL == 0 && (Sys()->LCDCLKDIV & 0xFFU) == 0 && (Sys()->LCDCLKDIV & SYSCON_LCDCLKDIV_HALT_MASK) == 0);
	assert(Sys()->AHBCLKCTRLSET[2] == SYSCON_AHBCLKCTRL_LCD_MASK);
	assert(Sys()->PRESETCTRLSET[2] == SYSCON_PRESETCTRL_LCD_RST_MASK);
	assert(Sys()->PRESETCTRLCLR[2] == SYSCON_PRESETCTRL_LCD_RST_MASK);
	// Pins in their function, the backlight a GPIO output, off
	for (uint32_t i = NB_CTRL_PINS; i < sizeof(s_Pins) / sizeof(s_Pins[0]); i++)
	{
		assert((Iocon()->PIO[s_Pins[i].PortNo][s_Pins[i].PinNo] & 7U) == 1U);
	}
	assert((Iocon()->PIO[3][31] & 7U) == 0U && (Gpio()->DIR[3] & (1UL << 31)) != 0U);
	assert(Gpio()->CLR[3] == (1UL << 31) && Gpio()->SET[3] == 0);
	lcd.Backlight(true);
	assert(Gpio()->SET[3] == (1UL << 31));
	Gpio()->CLR[3] = 0;
	lcd.Backlight(false);
	assert(Gpio()->CLR[3] == (1UL << 31));
	// The frame buffer cleared, text at the top
	for (uint32_t i = 0; i < PW * PH; i++)
	{
		assert(s_Fb[i] == 0);
	}
	assert(s_Fb[PW * PH] == 0xA5A5);
	assert(lcd.Width() == PW && lcd.Height() == PH);

	// Disable and Enable: output off then on in order, the frame kept
	s_Fb[5] = 0x1234;
	lcd.Disable();
	assert((s_Ctrl & (LCD_CTRL_LCDEN_MASK | LCD_CTRL_LCDPWR_MASK)) == 0);
	assert(lcd.Enable() && s_Enables == 2 && s_PowerOns == 2 && s_Fb[5] == 0x1234);

	// PowerOff: output and clocks off, backlight off; Init again
	lcd.Backlight(true);
	Gpio()->CLR[3] = 0;
	lcd.PowerOff();
	assert((s_Ctrl & (LCD_CTRL_LCDEN_MASK | LCD_CTRL_LCDPWR_MASK)) == 0);
	assert(Sys()->AHBCLKCTRLCLR[2] == SYSCON_AHBCLKCTRL_LCD_MASK && Sys()->LCDCLKSEL == 3);
	assert(Gpio()->CLR[3] == (1UL << 31));
	// Off until the next Init
	Gpio()->SET[3] = 0;
	lcd.Backlight(true);
	assert(Gpio()->SET[3] == 0);
	lcd.SetPixel(0, 0, 0xFFFF);
	assert(s_Fb[0] == 0);
	assert(lcd.Init(c, NULL) && s_Enables == 3);
	// Init again while running: stopped first
	assert(lcd.Init(c, NULL) && s_Enables == 4);

	// Panel clocks: the closest divider, no divider at the LCD clock or more,
	// the largest divider for a slow clock
	static const struct { uint32_t Core, Ask, Pcd, Bcd, Made; } k[] = {
		{ 220000000, 8900000, 23, 0, 8800000 }, { 220000000, 110000000, 0, 0, 110000000 },
		{ 220000000, 150000000, 0, 1, 220000000 }, { 220000000, 300000000, 0, 1, 220000000 },
		{ 220000000, 1, 1023, 0, 220000000 / 1025 }, { 12000000, 9000000, 0, 1, 12000000 },
		{ 180000000, 33000000, 3, 0, 36000000 },
	};
	for (auto &e : k)
	{
		Fresh();
		s_CoreClk = e.Core;
		c = Cfg();
		s_T.PixelClock = e.Ask;
		assert(lcd.Init(c, NULL));
		assert(Pcd(s_Lcd.POL) == e.Pcd && ((s_Lcd.POL & LCD_POL_BCD_MASK) != 0) == (e.Bcd != 0));
		assert(lcd.PixelClock() == e.Made);
	}

	// Polarities and the red lines
	Fresh();
	c = Cfg();
	s_T.Flags = DISPL_TIMING_DE_LOW | DISPL_TIMING_CLK_FALLING | DISPL_TIMING_BGR;
	assert(lcd.Init(c, NULL));
	assert((s_Lcd.POL & (LCD_POL_IVS_MASK | LCD_POL_IHS_MASK | LCD_POL_IOE_MASK | LCD_POL_IPC_MASK)) ==
		   (LCD_POL_IOE_MASK | LCD_POL_IPC_MASK));
	assert((s_Ctrl & LCD_CTRL_BGR_MASK) == 0);
	lcd.PowerOff();
	static const struct { uint32_t Flags, Pol; } pol[] = {
		{ DISPL_TIMING_VSYNC_LOW, LCD_POL_IVS_MASK }, { DISPL_TIMING_HSYNC_LOW, LCD_POL_IHS_MASK },
		{ DISPL_TIMING_DE_LOW, LCD_POL_IOE_MASK }, { DISPL_TIMING_CLK_FALLING, LCD_POL_IPC_MASK }, { 0, 0 },
	};
	for (auto &e : pol)
	{
		Fresh();
		c = Cfg();
		s_T.Flags = e.Flags;
		assert(lcd.Init(c, NULL));
		assert((s_Lcd.POL & (LCD_POL_IVS_MASK | LCD_POL_IHS_MASK | LCD_POL_IOE_MASK | LCD_POL_IPC_MASK)) == e.Pol);
		assert(s_Ctrl & LCD_CTRL_BGR_MASK);
		lcd.PowerOff();
	}

	// A panel reset pin: an output, pulsed low at Init and at Reset, high
	// after
	static IOPinCfg_t rst[sizeof(s_Pins) / sizeof(s_Pins[0])];
	memcpy(rst, s_Pins, sizeof(rst));
	rst[2] = {2, 27, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL};
	Fresh();
	c = Cfg();
	c.pPins = rst;
	assert(lcd.Init(c, NULL));
	assert((Iocon()->PIO[2][27] & 7U) == 0U && (Gpio()->DIR[2] & (1UL << 27)) != 0U);
	assert(Gpio()->CLR[2] == (1UL << 27) && Gpio()->SET[2] == (1UL << 27));
	Gpio()->CLR[2] = 0;
	Gpio()->SET[2] = 0;
	lcd.Reset();
	assert(Gpio()->CLR[2] == (1UL << 27) && Gpio()->SET[2] == (1UL << 27));
	lcd.PowerOff();
	printf("registers: PASS\n");
}

static void TestPixels()
{
	static const DISPL_ORIENT orients[] = {
		DISPL_ORIENT_LANDSCAPE, DISPL_ORIENT_LANDSCAPE_INV, DISPL_ORIENT_PORTRAIT, DISPL_ORIENT_PORTRAIT_INV,
	};
	static uint16_t blk[20 * 15];

	for (int i = 0; i < 20 * 15; i++)
	{
		blk[i] = (uint16_t)(0x100 + i);
	}

	for (DISPL_ORIENT o : orients)
	{
		LcdLpc546xx lcd;

		Fresh();
		DisplayCfg_t c = Cfg();
		c.Orient = o;
		assert(lcd.Init(c, NULL));
		bool port = o == DISPL_ORIENT_PORTRAIT || o == DISPL_ORIENT_PORTRAIT_INV;
		uint32_t w = port ? PH : PW, h = port ? PW : PH;
		assert(lcd.Width() == w && lcd.Height() == h && lcd.Orientation() == o);

		// Pixels at the corners and inside, out of the screen ignored
		lcd.SetPixel(0, 0, 0x1111);
		lcd.SetPixel((uint16_t)(w - 1), 0, 0x2222);
		lcd.SetPixel(0, (uint16_t)(h - 1), 0x3333);
		lcd.SetPixel((uint16_t)(w - 1), (uint16_t)(h - 1), 0x4444);
		lcd.SetPixel(17, 9, 0x5555);
		lcd.SetPixel((uint16_t)w, 0, 0x6666);
		lcd.SetPixel(0, (uint16_t)h, 0x6666);
		assert(s_Fb[Idx(o, 0, 0)] == 0x1111 && s_Fb[Idx(o, w - 1, 0)] == 0x2222);
		assert(s_Fb[Idx(o, 0, h - 1)] == 0x3333 && s_Fb[Idx(o, w - 1, h - 1)] == 0x4444);
		assert(s_Fb[Idx(o, 17, 9)] == 0x5555);
		int set = 0;
		for (uint32_t i = 0; i < PW * PH; i++)
		{
			set += s_Fb[i] != 0;
		}
		assert(set == 5 && s_Fb[PW * PH] == 0xA5A5);

		// Fill, clipped at the right and the bottom
		lcd.Clear();
		lcd.Fill(10, 20, 30, 40, 0xF800);
		lcd.Fill((uint16_t)(w - 5), (uint16_t)(h - 3), 100, 100, 0x07E0);
		lcd.Fill((uint16_t)w, 0, 10, 10, 0x001F);
		lcd.Fill((uint16_t)(w + 5), 0, 10, 10, 0x001F);
		lcd.Fill(0, (uint16_t)(h + 5), 10, 10, 0x001F);
		for (uint32_t y = 0; y < h; y++)
		{
			for (uint32_t x = 0; x < w; x++)
			{
				uint16_t want = 0;
				if (x >= 10 && x < 40 && y >= 20 && y < 60)
				{
					want = 0xF800;
				}
				if (x >= w - 5 && y >= h - 3)
				{
					want = 0x07E0;
				}
				assert(s_Fb[Idx(o, x, y)] == want);
			}
		}
		assert(s_Fb[PW * PH] == 0xA5A5);

		// BitBlt, the block lines of its own width when clipped
		lcd.Clear();
		lcd.BitBlt(3, 4, 20, 15, (uint8_t *)blk);
		lcd.BitBlt((uint16_t)(w - 6), (uint16_t)(h - 2), 20, 15, (uint8_t *)blk);
		lcd.BitBlt(0, (uint16_t)h, 20, 15, (uint8_t *)blk);
		lcd.BitBlt(0, (uint16_t)(h + 3), 20, 15, (uint8_t *)blk);
		lcd.BitBlt((uint16_t)(w + 3), 0, 20, 15, (uint8_t *)blk);
		lcd.BitBlt(0, 0, 20, 15, NULL);
		for (uint32_t y = 0; y < h; y++)
		{
			for (uint32_t x = 0; x < w; x++)
			{
				uint16_t want = 0;
				if (x >= 3 && x < 23 && y >= 4 && y < 19)
				{
					want = blk[(y - 4) * 20 + (x - 3)];
				}
				if (x >= w - 6 && y >= h - 2)
				{
					want = blk[(y - (h - 2)) * 20 + (x - (w - 6))];
				}
				assert(s_Fb[Idx(o, x, y)] == want);
			}
		}

		// Scroll: rows of the screen move, the rows freed are cleared
		for (uint32_t y = 0; y < h; y++)
		{
			for (uint32_t x = 0; x < w; x++)
			{
				s_Fb[Idx(o, x, y)] = (uint16_t)(y * 7 + x);
			}
		}
		lcd.Scroll(DISPL_SCROLL_DIR_UP, 10);
		for (uint32_t y = 0; y < h; y++)
		{
			for (uint32_t x = 0; x < w; x++)
			{
				assert(s_Fb[Idx(o, x, y)] == (y < h - 10 ? (uint16_t)((y + 10) * 7 + x) : 0));
			}
		}
		for (uint32_t y = 0; y < h; y++)
		{
			for (uint32_t x = 0; x < w; x++)
			{
				s_Fb[Idx(o, x, y)] = (uint16_t)(y * 7 + x);
			}
		}
		lcd.Scroll(DISPL_SCROLL_DIR_DOWN, 3);
		for (uint32_t y = 0; y < h; y++)
		{
			for (uint32_t x = 0; x < w; x++)
			{
				assert(s_Fb[Idx(o, x, y)] == (y >= 3 ? (uint16_t)((y - 3) * 7 + x) : 0));
			}
		}
		lcd.Scroll(DISPL_SCROLL_DIR_LEFT, 3);
		lcd.Scroll(DISPL_SCROLL_DIR_UP, 0);
		assert(s_Fb[Idx(o, 1, 5)] == (uint16_t)(2 * 7 + 1));
		lcd.Scroll(DISPL_SCROLL_DIR_UP, (uint16_t)h);
		for (uint32_t i = 0; i < PW * PH; i++)
		{
			assert(s_Fb[i] == 0);
		}
		lcd.Fill(0, 0, (uint16_t)w, (uint16_t)h, 0x1234);
		lcd.Scroll(DISPL_SCROLL_DIR_DOWN, (uint16_t)(h + 5));
		for (uint32_t i = 0; i < PW * PH; i++)
		{
			assert(s_Fb[i] == 0);
		}
		assert(s_Fb[PW * PH] == 0xA5A5);
		lcd.PowerOff();
	}
	printf("pixels: PASS\n");
}

// Pixels set in a box of the screen
static int Lit(DISPL_ORIENT O, uint32_t X0, uint32_t Y0, uint32_t W, uint32_t H)
{
	int n = 0;

	for (uint32_t y = Y0; y < Y0 + H; y++)
	{
		for (uint32_t x = X0; x < X0 + W; x++)
		{
			n += s_Fb[Idx(O, x, y)] != 0;
		}
	}
	return n;
}

static void TestText()
{
	static const DISPL_ORIENT orients[] = { DISPL_ORIENT_LANDSCAPE, DISPL_ORIENT_PORTRAIT };

	for (DISPL_ORIENT o : orients)
	{
		LcdLpc546xx lcd;

		Fresh();
		DisplayCfg_t c = Cfg();
		c.Orient = o;
		assert(lcd.Init(c, NULL));
		uint32_t w = lcd.Width(), h = lcd.Height();
		// The system font, 5 x 7: a character in the first line, in white
		lcd.printf("H");
		int lit = Lit(o, 0, 0, 6, 8);
		assert(lit > 5 && Lit(o, 0, 0, w, h) == lit);
		bool white = true;
		for (uint32_t i = 0; i < PW * PH; i++)
		{
			white = white && (s_Fb[i] == 0 || s_Fb[i] == 0xFFFF);
		}
		assert(white);
		// The next line, then a scroll when the screen is full: the first
		// line goes away
		lcd.printf("\r\nI");
		assert(Lit(o, 0, 8, 6, 8) > 0);
		int lines = (int)(h / 8) + 2;
		for (int i = 0; i < lines; i++)
		{
			lcd.printf("\r\nX");
		}
		assert(Lit(o, 0, 0, w, 8) > 0 && Lit(o, 0, 0, w, h) > 0);
		lcd.Clear();
		assert(Lit(o, 0, 0, w, h) == 0);
		lcd.printf("H");
		assert(Lit(o, 0, 0, 6, 8) == lit);
		// Reset: cleared, text at the top again
		lcd.printf("\r\nI\r\nJ");
		lcd.Reset();
		assert(Lit(o, 0, 0, w, h) == 0);
		lcd.printf("H");
		assert(Lit(o, 0, 0, 6, 8) == lit);
		// Fill and colors
		lcd.Fill(0, 0, 1, 1, LcdLpc546xx::Rgb565(255, 0, 0));
		assert(s_Fb[Idx(o, 0, 0)] == 0xF800);
		assert(LcdLpc546xx::Rgb565(0, 255, 0) == 0x07E0 && LcdLpc546xx::Rgb565(0, 0, 255) == 0x001F);
		// Orientation change: the screen size follows, the text goes down to
		// the new bottom before it scrolls
		DISPL_ORIENT o2 = o == DISPL_ORIENT_PORTRAIT ? DISPL_ORIENT_LANDSCAPE : DISPL_ORIENT_PORTRAIT;
		lcd.Orientation(o2);
		assert(lcd.Width() == h && lcd.Height() == w);
		uint32_t h2 = lcd.Height();
		for (uint32_t i = 0; i < h2 / 8 + 2; i++)
		{
			lcd.printf("\r\nX");
		}
		assert(Lit(o2, 0, h2 - 8, lcd.Width(), 8) > 0);
		lcd.PowerOff();
	}

	// A small panel: the line height follows the screen height of the
	// orientation, 10 on 20 lines, 8 on 32
	LcdLpc546xx lcd;
	Fresh();
	DisplayCfg_t c = Cfg();
	c.Width = 32;
	c.Height = 20;
	assert(lcd.Init(c, NULL));
	lcd.printf("\r\nI");
	assert(Lit(DISPL_ORIENT_LANDSCAPE, 0, 13, 6, 7) > 0);
	lcd.Orientation(DISPL_ORIENT_PORTRAIT);
	lcd.Clear();
	lcd.printf("\r\nI");
	// Index of the small panel in portrait: x * 32 + (31 - y)
	int low = 0, high = 0;
	for (uint32_t y = 0; y < 32; y++)
	{
		for (uint32_t x = 0; x < 20; x++)
		{
			(y < 16 ? low : high) += s_Fb[x * 32 + (31 - y)] != 0;
		}
	}
	assert(low > 0 && high == 0);
	lcd.PowerOff();
	printf("text: PASS\n");
}

int main()
{
	setvbuf(stdout, NULL, _IONBF, 0);
	TestInitChecks();
	TestRegisters();
	TestPixels();
	TestText();
	printf("lcd: PASS\n");

	return 0;
}
