// Exercise the shared menu with scripted ADC voltages and the real SSD1315 driver.
// Inspect OLED RAM at the SPI boundary using the WBA65 board configuration.
#include <cassert>
#include <cstdio>
#include <cstring>
#include "converters/adc_device.h"
#include "board.h"
static float voltage = 3.3f;
static bool readOk = true, ledHigh;

class FakeAdc {
public:
	bool Init(const AdcCfg_t &c)
	{
		assert(!c.bInterrupt && c.Mode == ADC_CONV_MODE_SINGLE);
		return true;
	}

	bool OpenChannel(const AdcChanCfg_t *c, int n)
	{
		assert(n == 1 && c->Chan == 0 && c->PinP.PinNo == 6 && c->Gain == 0x100);
		return true;
	}

	bool Read(int c, AdcData_t *p)
	{
		assert(c == 0);
		p->Data = voltage;
		return readOk;
	}
};

#undef ADC_DRIVER
#undef ADC_DRIVER_HEADER
#define ADC_DRIVER FakeAdc
#define ADC_DRIVER_HEADER "converters/adc_device.h"
#define main JoystickMenuMain
#include "../../exemples/display/joystick_menu_demo.cpp"
#undef main

#ifdef JOYSTICK_STOPWATCH
static uint64_t ticks;

static uint64_t TickCount(TimerDev_t *)
{
	return ticks;
}

extern "C" bool TimerInit(TimerDev_t *t, const TimerCfg_t *c)
{
	assert(c->DevNo == 2 && c->Freq == 1000000 && !c->bTickInt);
	t->Freq = c->Freq;
	t->GetTickCount = TickCount;
	return true;
}
#endif

static bool selected, dataMode;
static uint8_t ram[8][128];
static unsigned page, column;
static bool pageMode, displayOn;
static unsigned transfers, resetLow, resetHigh;

// Unused SPI virtual methods must never be called by a display master.
extern "C" void SPISetSlaveRxBuffer(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void SPISetSlaveTxData(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" SPIPHY SPISetPhy(SPIDev_t *, SPIPHY Phy)
{
	assert(false);
	return Phy;
}

extern "C" void IOPinConfig(int Port, int Pin, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert((Port == IOPORTE && (Pin == 0 || Pin == 3)) || (Port == IOPORTD && Pin == 8));
}

extern "C" void IOPinSet(int Port, int Pin)
{
	assert((Port == IOPORTE && (Pin == 0 || Pin == 3)) || (Port == IOPORTD && Pin == 8));
	if (Port == IOPORTD)
	{
		ledHigh = true;
		return;
	}
	if (Pin == 3)
	{
		resetHigh++;
	}
	else
	{
		dataMode = true;
	}
}

extern "C" void IOPinClear(int Port, int Pin)
{
	assert((Port == IOPORTE && (Pin == 0 || Pin == 3)) || (Port == IOPORTD && Pin == 8));
	if (Port == IOPORTD)
	{
		ledHigh = false;
		return;
	}
	if (Pin == 3)
	{
		resetLow++;
	}
	else
	{
		dataMode = false;
	}
}

static bool Start(DevIntrf_t *, uint32_t Address)
{
	assert(Address == 0 && !selected);
	selected = true;
	return true;
}

static void Stop(DevIntrf_t *)
{
	assert(selected);
	selected = false;
}

static int Tx(DevIntrf_t *, const uint8_t *p, int n)
{
	assert(selected && n > 0);
	transfers++;
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

extern "C" bool SPIInit(SPIDev_t *pDev, const SPICfg_t *pCfg)
{
	assert(pCfg->DevNo == 2 && pCfg->Phy == SPIPHY_NORMAL);
	assert(pCfg->DataSize == 8 && pCfg->BitOrder == SPIDATABIT_MSB);
	// Active-high clock polarity means CPOL=0, not CPOL=1.
	assert(pCfg->ClkPol == SPICLKPOL_HIGH);
	assert(pCfg->DataPhase == SPIDATAPHASE_FIRST_CLK);
	assert(!pCfg->bDmaEn && !pCfg->bIntEn);
	assert(pCfg->pIOPinMap[0].PortNo == IOPORTA && pCfg->pIOPinMap[0].PinNo == 0);
	assert(pCfg->pIOPinMap[0].PinOp == IOPINOP_FUNC6);
	assert(pCfg->pIOPinMap[2].PortNo == IOPORTB && pCfg->pIOPinMap[2].PinNo == 8);
	assert(pCfg->pIOPinMap[2].PinOp == IOPINOP_FUNC6);
	assert(pCfg->pIOPinMap[3].PortNo == IOPORTE && pCfg->pIOPinMap[3].PinNo == 1);
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.StartTx = Start;
	pDev->DevIntrf.StopTx = Stop;
	pDev->DevIntrf.TxData = Tx;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	return true;
}

static void Sample(float v, int count = 3)
{
	voltage = v;
	while (count-- > 0)
	{
		PollJoystick();
	}
}

static void Press(float v)
{
	Sample(3.3f);
	Sample(v);
}
#ifdef JOYSTICK_STOPWATCH
static void Advance(uint32_t Ms)
{
	ticks += uint64_t(Ms) * 1000;
	PollJoystick();
}

static void ExpectClock(const char *Text)
{
	uint8_t actual[8][128];
	memcpy(actual, ram, sizeof(ram));
	g_Lcd.Clear();
	g_Lcd.SetCursor(0, 16);
	g_Lcd.printf("%s", Text);
	for (unsigned y = 16; y < 23; y++)
	{
		for (unsigned x = 0; x < 128; x++)
		{
			assert((actual[y / 8][x] & (1U << (y % 8))) == (ram[y / 8][x] & (1U << (y % 8))));
		}
	}
}

static void StopwatchChecks()
{
	assert(s_Page == 3 && s_Item == 3 && !s_Running);
	ExpectClock("00:00:00.00");
	Sample(0, 20);
	assert(!s_Running); // Held at boot ignored
	Press(0);
	assert(s_Running);
	// A single delayed iteration must account for all elapsed hardware time.
	Advance(61234);
	assert(s_ElapsedMs == 61234);
	ExpectClock("00:01:01.23");
	Press(2.64f);
	assert(s_HaveLap && s_LapMs == 61234 && s_Running);
	Advance(4321);
	assert(s_ElapsedMs == 65555 && s_LapMs == 61234);
	Press(0.66f);
	assert(s_ElapsedMs == 65555 && s_Running); // Reset ignored while running
	Press(0);
	assert(!s_Running);
	Advance(99999);
	assert(s_ElapsedMs == 65555);
	Press(0);
	assert(s_Running);
	Advance(1000);
	assert(s_ElapsedMs == 66555);
	Press(2.01f);
	assert(s_Page == 0 && s_Running);
	Advance(2000);
	assert(s_ElapsedMs == 68555); // Continues away from its page
	Press(1.32f);
	assert(s_Item == 0); // Four-item menu wraps
	Press(2.01f);
	assert(s_Item == 3);
	Press(0);
	assert(s_Page == 3 && s_Running);
	Press(0);
	assert(!s_Running);
	Press(0.66f);
	assert(s_ElapsedMs == 0 && !s_HaveLap);
	// Place the free-running timestamp near its uint32 millisecond wrap.
	ticks = uint64_t(0xFFFFFFF0U) * 1000;
	PollJoystick();
	Press(0);
	Advance(32);
	assert(s_ElapsedMs == 32);
	// ADC errors must neither pause time nor cause phantom key actions.
	readOk = false;
	Advance(1000);
	assert(s_InputError && s_ElapsedMs == 1032 && s_Running);
	readOk = true;
	Sample(0);
	assert(!s_InputError && s_Running);
	Press(0);
	assert(!s_Running);
	Press(0.66f);
	Press(0);
	Advance(359999980);
	assert(s_Running);
	Advance(100);
	assert(!s_Running && s_ElapsedMs == s_MaxElapsedMs);
	ExpectClock("99:59:59.99");
	Press(0);
	assert(!s_Running); // Full display requires reset
	Press(0.66f);
	assert(s_ElapsedMs == 0);
	Press(0);
	assert(s_Running);
	assert(!selected);
	std::puts("Stopwatch: hardware timestamps, OLED digits, pause/resume, lap, menu, ADC failure, wrap and limit PASS");
}
#endif

int main()
{
	assert(HardwareInit() && displayOn && ledHigh && !s_LedOn);
	assert(resetLow == 1 && resetHigh >= 1);
#ifdef JOYSTICK_STOPWATCH
	StopwatchChecks();
#else
	assert(DecodeKey(0) == KEY_SELECT && DecodeKey(0.66f) == KEY_LEFT);
	assert(DecodeKey(1.32f) == KEY_DOWN && DecodeKey(2.01f) == KEY_UP);
	assert(DecodeKey(2.64f) == KEY_RIGHT && DecodeKey(3.3f) == KEY_NONE);
	assert(DecodeKey(-1) == KEY_INVALID && DecodeKey(NAN) == KEY_INVALID);
	// A key held during boot is ignored until a stable release.
	Sample(0, 20);
	assert(!s_LedOn);
	Press(0);
	assert(s_LedOn && !ledHigh);
	Sample(0, 20);
	assert(s_LedOn);
	Sample(3.3f);
	Sample(1.32f, 2);
	Sample(3.3f);
	assert(s_Item == 0); // Short bounce ignored
	Press(1.32f);
	assert(s_Item == 1);
	uint8_t menu[sizeof(ram)];
	memcpy(menu, ram, sizeof(ram));
	Press(0);
	assert(s_Page == 1 && memcmp(menu, ram, sizeof(ram)) != 0);
	for (int i = 0; i < 20; i++)
	{
		Press(2.64f);
	}
	assert(s_Level == 10);
	for (int i = 0; i < 20; i++)
	{
		Press(0.66f);
	}
	assert(s_Level == 1);
	Press(0);
	assert(s_Page == 0);
	Press(1.32f);
	assert(s_Item == 2);
	Press(0);
	assert(s_Page == 2);
	Press(0.66f);
	assert(s_Page == 0);
	Press(1.32f);
	assert(s_Item == 0);
	Press(2.01f);
	assert(s_Item == 2); // Wrap both ways
	readOk = false;
	PollJoystick();
	assert(s_InputError);
	readOk = true;
	Sample(0);
	assert(!s_InputError && s_Page == 0);
	// Recovery requires a release; a stale held selection cannot activate.
	Press(0);
	assert(s_Page == 2);
	assert(!selected);
	std::puts("Joystick menu: SPI/RAM, ladder, debounce, held keys, navigation, bounds and read failure PASS");
#endif
}
