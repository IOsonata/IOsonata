// Run the shared BLE scan example through the production advertising parser,
// AppEvt queue and SSD1315 driver; model only the timer, UART and SPI port.
// Inspect OLED RAM at the SPI boundary using the WBA65 board configuration.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#define main BleScanDisplayMain
#include "../../exemples/bluetooth/ble_central_scan.cpp"
#undef main

static uint64_t ticks;
static TimerTrigEvtHandler_t tickHandler;
static bool timerStopped;
static char uartLog[4096];
static size_t uartUsed;
extern "C" bool UARTInit(UARTDev_t *, const UARTCfg_t *) { return true; }
extern "C" void UARTSetCtrlLineState(UARTDev_t *, uint32_t) {}
extern "C" void UARTvprintf(UARTDev_t *, const char *fmt, va_list args)
{
	int n = vsnprintf(uartLog + uartUsed, sizeof(uartLog) - uartUsed, fmt, args);
	if (n > 0) { uartUsed += size_t(n); }
	assert(uartUsed < sizeof(uartLog));
}
static uint64_t Count(TimerDev_t *) { return ticks; }
static void DisableTimer(TimerDev_t *) { timerStopped = true; }
static uint64_t Trigger(TimerDev_t *, int i, uint64_t period, TIMER_TRIG_TYPE type,
		TimerTrigEvtHandler_t cb, void *)
{
	assert(i == 0 && period == 250000000 && type == TIMER_TRIG_TYPE_CONTINUOUS);
	tickHandler = cb;
	return period;
}
extern "C" bool TimerInit(TimerDev_t *t, const TimerCfg_t *c)
{
	assert(c->DevNo == 2 && c->Freq == 1000000 && !c->bTickInt);
	t->Freq = c->Freq; t->GetTickCount = Count; t->EnableTrigger = Trigger;
	t->Disable = DisableTimer;
	return true;
}
extern "C" bool AppCheckStatus() { return true; }

static bool selected, dataMode;
static uint8_t ram[8][128];
static unsigned page, column;
static bool pageMode, displayOn;
static unsigned transfers, resetLow, resetHigh;

// Unused SPI virtual methods must never be called by a display master.
extern "C" void SPISetSlaveRxBuffer(SPIDev_t *, int, uint8_t *, int) { assert(false); }
extern "C" void SPISetSlaveTxData(SPIDev_t *, int, uint8_t *, int) { assert(false); }
extern "C" SPIPHY SPISetPhy(SPIDev_t *, SPIPHY Phy) { assert(false); return Phy; }

extern "C" void IOPinConfig(int Port, int Pin, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert(Port == IOPORTE && (Pin == 0 || Pin == 3));
}
extern "C" void IOPinSet(int Port, int Pin)
{
	assert(Port == IOPORTE && (Pin == 0 || Pin == 3));
	if (Pin == 3) { resetHigh++; }
	else { dataMode = true; }
}
extern "C" void IOPinClear(int Port, int Pin)
{
	assert(Port == IOPORTE && (Pin == 0 || Pin == 3));
	if (Pin == 3) { resetLow++; }
	else { dataMode = false; }
}
static bool Start(DevIntrf_t *, uint32_t Address)
{
	assert(Address == 0 && !selected);
	selected = true;
	return true;
}
static void Stop(DevIntrf_t *) { assert(selected); selected = false; }
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
				if (cmd == 0x20) { assert(p[i] == 2); pageMode = true; }
				if (cmd == 0xA8) { assert(p[i] >= 15 && p[i] <= 63); }
				if (cmd == 0xDB) { assert((p[i] & ~0x30U) == 0); }
			}
			else if ((cmd & 0xF8) == 0xB0) { page = cmd & 7; }
			else if ((cmd & 0xF0) == 0x00) { column = (column & 0xF0) | (cmd & 15); }
			else if ((cmd & 0xF0) == 0x10) { column = (column & 15) | ((cmd & 15) << 4); }
			else if (cmd == 0xAF) { displayOn = true; }
			else if (cmd == 0xAE) { displayOn = false; }
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

static void Advance(uint32_t ms) { ticks = uint64_t(ms) * 1000; }
static void Report(uint8_t id, uint8_t type, int rssi, const char *name)
{
	uint8_t addr[6] = {id, 2, 3, 4, 5, 6};
	uint8_t adv[100] = {};
	size_t len = name ? strlen(name) : 0;
	assert(len <= 98);
	if (len) { adv[0] = len + 1; adv[1] = BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME; memcpy(adv + 2, name, len); }
	assert(BtAppScanReport(rssi, type, addr, len ? len + 2 : 0, adv));
}
static int Used()
{
	int n = 0;
	for (const auto &d : s_Devices) { n += d.Used; }
	return n;
}
static void NoOp(uint32_t, void *) {}
int main()
{
	assert(AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)));
	assert(ScanDisplayInit() && displayOn && tickHandler);
	assert(SignalWidth(-110) == 0 && SignalWidth(-100) == 0);
	assert(SignalWidth(-60) == 64 && SignalWidth(-20) == 128 && SignalWidth(0) == 128);
	assert(SignalWidth(127) == 0);
	Advance(100); Report(1, 0, -60, "Peripheral");
	assert(Used() == 1 && strcmp(s_Devices[0].Name, "Peripheral") == 0);
	assert(strstr(uartLog, "Peripheral\r\n")); // Last name character retained
	Advance(200); Report(1, 0, -40, nullptr);
	assert(Used() == 1 && strcmp(s_Devices[0].Name, "Peripheral") == 0 && s_Devices[0].Rssi == -40);
	// A report updates memory without transferring any display data.
	unsigned before = transfers;
	Report(1, 0, -60, "Peripheral"); assert(transfers == before);
	tickHandler(nullptr, 0, nullptr); tickHandler(nullptr, 0, nullptr);
	assert(transfers == before && AppEvtHandlerPending());
	AppEvtHandlerExec(); assert(transfers > before && !AppEvtHandlerPending());
	// The first device's bar has the expected 64-pixel width in OLED RAM.
	for (unsigned x = 0; x < 128; x++)
	{
		assert(bool(ram[20 / 8][x] & (1U << (20 % 8))) == (x < 64));
	}
	Report(1, 1, -50, "Other address type"); assert(Used() == 2);
	Advance(300); Report(2, 0, -80, "Control\nname");
	assert(strcmp(s_Devices[2].Name, "Control?name") == 0);
	Advance(400); Report(3, 0, -70, "0123456789abcdefghijkl");
	assert(strlen(s_Devices[3].Name) == 14 && s_Devices[3].Name[14] == 0);
	Advance(500); Report(4, 0, 127, nullptr);
	assert(Used() == 4 && s_Devices[0].Addr[0] == 4 && s_Devices[0].Name[0] == 0);
	// A full queue does not wedge refresh; the next tick retries.
	for (int i = 0; i < 16; i++) { assert(AppEvtHandlerQue(0, nullptr, NoOp)); }
	ScanDisplayTick(nullptr, 0, nullptr); assert(!s_DisplayQueued);
	AppEvtHandlerExec(); ScanDisplayTick(nullptr, 0, nullptr);
	assert(s_DisplayQueued); AppEvtHandlerExec(); assert(!s_DisplayQueued);
	Advance(10500); ScanDisplayRefresh(0, nullptr); assert(Used() == 0);
	Advance(0xFFFFFFF0U); Report(1, 0, -50, "Wrap");
	Advance(10); ScanDisplayRefresh(0, nullptr); assert(Used() == 1);
	Advance(10000); ScanDisplayRefresh(0, nullptr); assert(Used() == 0);
	uint8_t bad[] = {20, 9, 'A'}; uint8_t addr[6] = {5};
	ScanDisplayReport(-30, 0, addr, sizeof(bad), bad); assert(Used() == 1 && !s_Devices[0].Name[0]);
	ScanDisplayError("Scan init failed");
	assert(!s_DisplayReady && timerStopped && !selected);
	std::puts("BLE OLED scan: cache, names, RSSI pixels, deferred refresh, full queue, expiry and wrap PASS");
}
