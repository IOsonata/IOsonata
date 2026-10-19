// Exercise the real security example, peer pool, event queue and OLED driver.
// Model security queries and radio decisions, timer, UART and SPI ports.
// Inspect OLED RAM at the SPI boundary using the WBA65 board configuration.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#define IRQ_PRIO_LOW 5
#define __NOP() ((void)0)
#define main BleStatusDisplayMain
#include "../../exemples/bluetooth/ble_security_demo.cpp"
#undef main

static TimerTrigEvtHandler_t tickHandler;
static bool triggerDisabled;
static bool knownSecurity = true;
static bool inTimer;
static uint8_t levels[2] = {BT_GAP_SEC_LEVEL_NONE, BT_GAP_SEC_LEVEL_LESC_AUTH};
static int decisions;
static bool accepted;
static char uartInput;
static bool UartStart(DevIntrf_t *, uint32_t) { return true; }
static void UartStop(DevIntrf_t *) {}
static int Input(DevIntrf_t *, uint8_t *p, int n)
{
	assert(n > 0);
	if (!uartInput) { return 0; }
	*p = uartInput; uartInput = 0; return 1;
}
extern "C" bool UARTInit(UARTDev_t *p, const UARTCfg_t *)
{
	p->DevIntrf.RxData = Input;
	p->DevIntrf.Type = DEVINTRF_TYPE_UART;
	p->DevIntrf.StartRx = UartStart;
	p->DevIntrf.StopRx = UartStop;
	atomic_flag_clear(&p->DevIntrf.bBusy);
	return true;
}
extern "C" void UARTSetCtrlLineState(UARTDev_t *, uint32_t) {}
extern "C" void UARTvprintf(UARTDev_t *, const char *, va_list) {}
extern "C" void BtAppCheckStatus() {}
void BtAppPairDecision(uint16_t h, bool yes) { assert(h == 0 || h == 7); decisions++; accepted = yes; }
int BtAppNotify(BtGattChar_t *, uint8_t *, uint16_t) { assert(false); return 0; }
static SysLog_t logState;
extern "C" SysLog_t *SysLogGet() { return &logState; }
extern "C" int SysLogFlush(SysLog_t *) { return 0; }
extern "C" int SysLogPrintf(SysLog_t *, const char *, ...) { return 0; }
void BtGattCccdClear(uint16_t) {}
BtGattSrvc_t *BtGattGetSrvcList() { return nullptr; }
void BtGattSrvcDisconnected(BtGattSrvc_t *) {}
bool BtGapConnSecGet(uint16_t h, BtConnSec_t *p)
{
	assert(!inTimer);
	assert(h == 0 || h == 7);
	if (!knownSecurity) { return false; }
	p->Level = levels[h == 0 ? 0 : 1];
	p->KeySize = 0;
	return true;
}
static uint64_t Count(TimerDev_t *) { return 0; }
static void DisableTrigger(TimerDev_t *, int i) { assert(i == 0); triggerDisabled = true; }
static uint64_t Trigger(TimerDev_t *, int i, uint64_t period, TIMER_TRIG_TYPE type,
		TimerTrigEvtHandler_t cb, void *)
{
	assert(i == 0 && period == 250000000 && type == TIMER_TRIG_TYPE_CONTINUOUS);
	tickHandler = cb;
	return period;
}
extern "C" bool TimerInit(TimerDev_t *t, const TimerCfg_t *c)
{
	assert(c->DevNo == 2 && c->Freq == 1000000);
	t->Freq = c->Freq; t->GetTickCount = Count; t->EnableTrigger = Trigger;
	t->DisableTrigger = DisableTrigger;
	return true;
}

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

static void Refresh()
{
	unsigned before = transfers;
	inTimer = true;
	tickHandler(nullptr, 0, nullptr); tickHandler(nullptr, 0, nullptr);
	inTimer = false;
	assert(before == transfers && AppEvtHandlerPending());
	AppEvtHandlerExec(); assert(transfers > before && !s_DisplayQueued);
}
static void NoOp(uint32_t, void *) {}
static void ExpectTextRow(unsigned y, const char *text)
{
	uint8_t actual[8][128]; memcpy(actual, ram, sizeof(ram));
	s_Display.Clear(); s_Display.SetCursor(0, y); s_Display.printf("%s", text);
	for (unsigned x = 0; x < 128; x++)
	{
		for (unsigned row = y; row < y + 7; row++)
		{
			assert((actual[row / 8][x] & (1U << (row % 8))) == (ram[row / 8][x] & (1U << (row % 8))));
		}
	}
}
int main()
{
	assert(AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)));
	assert(BtPeerInit(nullptr, 0));
	assert(g_Timer.Init(s_TimerCfg) && StatusDisplayInit() && displayOn && tickHandler);
	s_BleReady = true; Refresh(); ExpectTextRow(24, "Waiting for central");
	uint8_t addr[6] = {1, 2, 3, 4, 5, 6};
	assert(BtPeerConnected(0, BT_CONN_ROLE_PERIPHERAL, 0, addr, 0, nullptr));
	Refresh(); ExpectTextRow(16, "H0000 open L1");
	levels[0] = 2; Refresh(); ExpectTextRow(16, "H0000 enc L2");
	levels[0] = 3; Refresh(); ExpectTextRow(16, "H0000 MITM L3");
	levels[0] = 4; Refresh(); ExpectTextRow(16, "H0000 LESC MITM L4");
	knownSecurity = false; Refresh(); ExpectTextRow(16, "H0000 unknown");
	knownSecurity = true;
	assert(BtPeerConnected(7, BT_CONN_ROLE_PERIPHERAL, 1, addr, 0, nullptr));
	Refresh(); ExpectTextRow(32, "H0007 LESC MITM L4");
	unsigned before = transfers;
	assert(BtAppPairConfirm(0, 123) == -1 && transfers == before);
	assert(BtAppPairConfirm(7, 456) == 0); // Existing pending confirmation preserved
	Refresh(); ExpectTextRow(48, "Compare 000123");
	assert(g_Uart.Init(g_UartCfg));
	uartInput = 'y'; UartRxChedHandler(0, nullptr); AppCheckStatus();
	assert(decisions == 1 && accepted && !s_PairAwaiting);
	Refresh(); ExpectTextRow(48, "UART: 1000000 baud");
	assert(BtAppPairConfirm(7, 654321) == -1);
	uartInput = 'n'; UartRxChedHandler(0, nullptr); AppCheckStatus();
	assert(decisions == 2 && !accepted && !s_PairAwaiting);
	assert(BtAppPairConfirm(7, 111111) == -1);
	BtAppEvtDisconnected(0); assert(s_PairAwaiting); // Other link must not clear it
	BtAppEvtDisconnected(7); BtPeerFreeByHdl(7); assert(!s_PairAwaiting);
	Refresh(); ExpectTextRow(8, "Links: 1");
	BtPeerFreeByHdl(0); Refresh(); ExpectTextRow(24, "Waiting for central");
	for (int i = 0; i < 16; i++) { assert(AppEvtHandlerQue(0, nullptr, NoOp)); }
	StatusDisplayTick(nullptr, 0, nullptr); assert(!s_DisplayQueued);
	AppEvtHandlerExec(); Refresh();
	StatusDisplayError("BLE init failed");
	assert(!s_DisplayReady && triggerDisabled && !selected);
	std::puts("BLE OLED status: real peer pool, live security, comparison, UART decisions, disconnect and queue recovery PASS");
}
