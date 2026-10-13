/**-------------------------------------------------------------------------
@example	self_test.cpp

@brief	R9A02G021 hardware self-test on the FPB-R9A02G021

Runs on the board what the manual leaves open or what the port assumed, and
prints PASS, FAIL or SKIP for each test with the values measured:

	1	ICU: the IR flag of a route sets with its CLIC input off, as the
		polling drivers expect
	2	TML32 interval on the LOCO, 16-bit mode, counted with the machine
		timer (also the LOCO): (ITLCMP + 1) x 2^div, read from figure 19.6
	3	TML32 interval on the HOCO, 16-bit mode with a divider, timed with
		mcycle
	4	TML32 interval on the HOCO, 32-bit mode, timed with mcycle
	5	DTC repeat transfer on TML32_OUTI clearing the TML32 flag (MODE_DTC),
		and the CLIC input of the slot while DTCE is 1
	6	SPI20 reception by the DTC, polled and with interrupt, at three rates:
		the last byte is read after it ended (no data left in SDR20)
	7	DAC8 outputs read by the A/D converter on their own pins (ANI16 on
		P000, ANI17 on P001)
	8	A/D scan mode ANI0 to ANI3 against select mode
	9	ANI18 and ANI19: which one is P106 (pin tables against table 29.12)
	10	Comparator 0 with its internal reference and with IVREF0 (P000,
		DAC8 channel 0)
	11	A/D compare window, hardware trigger: a result equal to ADLL (ANI2 at
		GND) with ADRCK 0 and 1 (29.3.8 text against figure)
	12	Comparator 0 wake-up from Software Standby (table 10.2 note 7 against
		tables 10.3 and 12.4), P106 touched to GND by hand

Wiring (board.h): P001 to P106, A2 (P400) to GND, A3 (P401) to 3.3 V.
Console on UART0, the J-Link OB virtual COM port, 115200 8N1. Test 12 enters
Software Standby, the debugger may lose the target there.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <string.h>

#include "idelay.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "converters/analog_comp.h"
#include "iopinctrl.h"
#include "stddev.h"
#include "clic.h"
#include "r9a02g021.h"
#include "system_r9a02.h"
#include "interrupt_r9a02.h"
#include "dtc_r9a02.h"
#include "tml32_r9a02.h"
#include "dac_r9a02.h"
#include "adc_r9a02.h"
#include "board.h"

#define SELFTEST_VREF			((float)SELFTEST_VCC_MV / 1000.0f)
#define SELFTEST_SPI_LEN		32
#define SELFTEST_DAC_ADC_LIM	60			// 12-bit LSB: DAC8 3 LSB (table 2.51) and A/D 10.5 LSB (table 2.40)
#define SELFTEST_WIN_TRIG		20			// TML32 intervals of 1 ms per window case

typedef enum {
	SELFTEST_PASS,
	SELFTEST_FAIL,
	SELFTEST_SKIP,
} SELFTEST_RES;

// Compare window case, ADUL 255, for a result of 0 (ANI2 at GND)
typedef struct {
	uint8_t Adll;			// ADLL
	bool bOutside;			// ADRCK
	bool bEvt;				// ADC_ENDI as Window documents it
} SelfTestWin_t;

static void AdcEvtHandler(Device * const pDev, DEV_EVT Evt);
static int SpiEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuffer, int Len);
static void CompEvtHandler(AnalogCompDev_t *pDev, ANALOG_COMP_EVT Evt);

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_UartPins[] = UART_PINS;
static const IOPinCfg_t s_SpiPins[] = SELFTEST_SPI_PINS;

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(512)];
alignas(4) static uint8_t s_UartRxMem[CFIFO_MEMSIZE(16)];

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
	.RxMemSize = sizeof(s_UartRxMem),
	.pRxMem = s_UartRxMem,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

// Machine timer, the LOCO
static const TimerCfg_t s_MtimerCfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtHandler = NULL,
	.bTickInt = false,
};

static const R9a02Tml32Cfg_t s_Tml32Base = {
	.ClkSrc = R9A02_TML32_CLK_LF,
	.Mode = R9A02_TML32_MODE_INT,
	.nsPeriod = 1000000ULL,
	.Handler = NULL,
	.pCtx = NULL,
	.IntPrio = IRQ_PRIO_HIGH,
	.bWakeup = false,
};

static const SPICfg_t s_SpiCfg = {
	.DevNo = SELFTEST_SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.DummyByte = 0xFF,
	.EvtCB = SpiEvtHandler,
};

static const AdcRefVolt_t s_AdcRefVolt[] = {
	{ .Type = ADC_REFVOLT_TYPE_SUPPLY, .Voltage = SELFTEST_VREF, .Pin = -1 },
};

static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_AdcRefVolt,
	.NbRefVolt = sizeof(s_AdcRefVolt) / sizeof(AdcRefVolt_t),
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,
	.OvrSample = 0,
	.bInterrupt = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = AdcEvtHandler,
};

static const DAC_CFG s_DacCfg = {
	.Mode = DAC_MODE_SINGLE_ENDED,
};

static const AnalogCompCfg_t s_CompBase = {
	.DevNo = 0,
	.Mode = ANALOG_COMP_MODE_CONTINUOUS,
	.RefSrc = 0,
	.RefVolt = SELFTEST_VCC_MV,
	.CompVolt = 0,
	.AnalogIn = 0,
	.bHystersys = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtHandler = CompEvtHandler,
};

static const SelfTestWin_t s_Win[] = {
	{ 0, false, true },				// ADLL = result, inside
	{ 1, false, false },			// result below ADLL
	{ 0, true, false },				// ADLL = result, not below Low
	{ 1, true, true },				// result below ADLL
};

static const char * const s_ResName[] = { "PASS", "FAIL", "SKIP" };

static UART s_Uart;
static Timer s_Mtimer;
static SPI s_Spi;
static DacR9a02 s_Dac;
static AdcR9a02 s_Adc;
static AnalogComp s_Comp;

static volatile uint32_t s_TmlCnt = 0;
static volatile uint32_t s_TmlFirst = 0;
static volatile uint32_t s_TmlLast = 0;
static volatile bool s_bTmlMtime = false;
static volatile uint64_t s_TmlStamp[2];
static volatile int s_WakeBy = 0;			// Test 12: 1 comparator, 2 TML32 time-out
static volatile bool s_bSpiDone = false;
static volatile int s_SpiCount = 0;
static volatile int s_AdcEvt = 0;
static volatile int s_CompEvt = 0;			// 1 above, -1 below
static volatile uint32_t s_CompCnt = 0;
static bool s_bClicLatch = false;			// Test 1: CLIC pending set with the input off
static bool s_bWireP106 = false;			// Test 9: DAC1 reaches P106
static int s_ResCnt[3] = { 0, };
static uint8_t s_SpiBuf[SELFTEST_SPI_LEN];

static uint64_t Cycles(void)
{
	uint32_t hi;
	uint32_t lo;
	uint32_t hi2;

	do
	{
		__asm volatile ("csrr %0, mcycleh" : "=r"(hi));
		__asm volatile ("csrr %0, mcycle" : "=r"(lo));
		__asm volatile ("csrr %0, mcycleh" : "=r"(hi2));
	} while (hi != hi2);

	return ((uint64_t)hi << 32) | lo;
}

static uint64_t Mtime(void)
{
	return TimerGetTickCount(s_Mtimer);
}

// Machine timer ticks in Ms, rounded up
static uint64_t MsTicks(uint32_t Ms)
{
	return ((uint64_t)Ms * R9A02_LOCO_FREQ + 999U) / 1000U;
}

static int ToRaw(float Volt)
{
	return (int)(Volt * 4096.0f / SELFTEST_VREF + 0.5f);
}

static int Diff(int A, int B)
{
	return A > B ? A - B : B - A;
}

static void AdcEvtHandler(Device * const pDev, DEV_EVT Evt)
{
	if (Evt == DEV_EVT_DATA_RDY)
	{
		s_AdcEvt = s_AdcEvt + 1;
	}
}

static int SpiEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuffer, int Len)
{
	if (Evt == DEVINTRF_EVT_COMPLETED)
	{
		s_SpiCount = Len;
		s_bSpiDone = true;
	}

	return 0;
}

static void CompEvtHandler(AnalogCompDev_t *pDev, ANALOG_COMP_EVT Evt)
{
	s_CompEvt = Evt == ANALOG_COMP_EVT_HIGHER ? 1 : -1;
	s_CompCnt = s_CompCnt + 1;
	if (s_WakeBy == 0)
	{
		s_WakeBy = 1;
	}
}

// Time stamps of intervals s_TmlFirst and s_TmlLast
static void Tml32CountHandler(void *pCtx)
{
	uint32_t n = s_TmlCnt + 1;

	s_TmlCnt = n;

	if (n == s_TmlFirst || n == s_TmlLast)
	{
		s_TmlStamp[n == s_TmlLast ? 1 : 0] = s_bTmlMtime ? Mtime() : Cycles();
	}
}

static void Tml32TimeoutHandler(void *pCtx)
{
	if (s_WakeBy == 0)
	{
		s_WakeBy = 2;
	}
}

static void Begin(int No, const char *pName)
{
	printf("\r\n%d %s\r\n", No, pName);
}

static void End(SELFTEST_RES Res)
{
	s_ResCnt[Res]++;
	printf("  %s\r\n", s_ResName[Res]);
}

// IELSR of an event route, -1 when none
static int FindSlot(uint8_t EvtId)
{
	for (int i = 0; i < (int)R9A02_ICU_IELSR_CNT; i++)
	{
		if (R9a02IntEvent(i) == EvtId)
		{
			return i;
		}
	}

	return -1;
}

static uint8_t ClicPending(int IntNo)
{
	return (uint8_t)(CLIC_INT_IP_REG(R9A02_CLIC_CTRL_BASE, R9A02_CLIC_ID_IELSR0 + IntNo) & 1U);
}

static AdcChanCfg_t AdcChan(int Chan, int Ani)
{
	AdcChanCfg_t c = {
		.Chan = Chan, .RefVoltIdx = 0, .Type = ADC_CHAN_TYPE_SINGLE_ENDED, .Gain = 1,
		.AcqTime = 2, .BurstMode = false, .PinP = { Ani, ADC_PIN_CONN_NONE },
		.PinN = { 0, ADC_PIN_CONN_NONE }, .FifoMemSize = 0, .pFifoMem = NULL,
	};

	return c;
}

// One round of the open channels, software trigger. Results of the
// channels in pRaw, -1 for a channel without result.
static int AdcConvert(int *pRaw, int Cnt)
{
	AdcData_t d[R9A02_ADC_MAX_CHAN];

	for (int i = 0; i < Cnt; i++)
	{
		pRaw[i] = -1;
	}

	s_AdcEvt = 0;
	if (s_Adc.StartConversion() == false)
	{
		return 0;
	}

	uint64_t end = Mtime() + MsTicks(20);

	while (s_AdcEvt == 0 && Mtime() < end)
	{
	}

	// No end: the round is stopped, the channels can be closed
	if (s_AdcEvt == 0)
	{
		s_Adc.StopConversion();
	}

	int n = s_Adc.Read(d, R9A02_ADC_MAX_CHAN);

	for (int i = 0; i < n; i++)
	{
		if (d[i].Chan >= 0 && d[i].Chan < Cnt)
		{
			pRaw[d[i].Chan] = ToRaw(d[i].Data);
		}
	}

	return n;
}

static void AdcCloseAll(void)
{
	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		s_Adc.CloseChannel(i);
	}
}

// CloseChannel releases the pin, DACOUT0 and DACOUT1 are its ANI16 and
// ANI17: both back to analog for the DAC8 outputs
static void DacPins(void)
{
	IOPinConfig(0, 0, IOPINOP_FUNC31, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 1, IOPINOP_FUNC31, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
}

// The UART stops in Software Standby, its output ends first
static void UartWaitIdle(void)
{
	UARTDev_t *dev = s_Uart;
	uint64_t end = Mtime() + MsTicks(200);

	while (dev->bTxReady == false && Mtime() < end)
	{
	}
	usDelay(200);
}

// A key from the console, -1 after Ms
static int KeyGet(uint32_t Ms)
{
	uint64_t end = Mtime() + MsTicks(Ms);
	uint8_t c;

	while (Mtime() < end)
	{
		if (s_Uart.Rx(&c, 1) > 0)
		{
			return c;
		}
	}

	return -1;
}

static SELFTEST_RES TestIcuPoll(void)
{
	R9a02Tml32Cfg_t cfg = s_Tml32Base;

	cfg.IntPrio = IRQ_PRIO_LOW;
	if (R9a02Tml32Init(&cfg) == false)
	{
		printf("  TML32 init failed\r\n");

		return SELFTEST_FAIL;
	}

	int n = FindSlot(R9A02_EVT_TML32_OUTI);

	if (n < 0)
	{
		R9a02Tml32PowerOff();
		printf("  No route of TML32_OUTI\r\n");

		return SELFTEST_FAIL;
	}

	// The route polled: CLIC input off, nothing pending, flag clear
	R9a02Tml32Stop();
	R9a02IntEnable(n, false);
	R9a02ClearPendingInt(n);
	R9A02_TML32->ITLS0 = 0U;
	R9a02Tml32Start();
	msDelay(3);

	bool ir = R9a02EventFlag(n);
	uint8_t ip = ClicPending(n);
	uint8_t itls = R9A02_TML32->ITLS0;

	R9a02ClearPendingInt(n);
	R9a02Tml32PowerOff();

	s_bClicLatch = ip != 0U;
	printf("  IELSR%d IR %d, CLIC pending %u with the input off, ITLS0 0x%02X\r\n", n, ir ? 1 : 0, ip, itls);

	if ((itls & R9A02_ITLS0_ITF00) == 0U)
	{
		printf("  The TML32 flag did not set\r\n");

		return SELFTEST_FAIL;
	}
	if (ir == false)
	{
		printf("  IR stays 0: the polling drivers (SPI, IICA, ADC, UARTA) must read the CLIC pending bit\r\n");

		return SELFTEST_FAIL;
	}

	return SELFTEST_PASS;
}

// Intervals s_TmlFirst to s_TmlLast timed with the machine timer (LOCO) or
// mcycle (HOCO)
static SELFTEST_RES TestTml32Period(R9A02_TML32_CLK Clk, uint64_t nsPeriod, uint32_t Cnt, bool b32)
{
	R9a02Tml32Cfg_t cfg = s_Tml32Base;
	bool lf = Clk == R9A02_TML32_CLK_LF;

	cfg.ClkSrc = Clk;
	cfg.nsPeriod = nsPeriod;
	cfg.Handler = Tml32CountHandler;

	s_TmlCnt = 0;
	s_TmlFirst = 4;
	s_TmlLast = 4 + Cnt;
	s_bTmlMtime = lf;
	s_TmlStamp[0] = 0;
	s_TmlStamp[1] = 0;

	if (R9a02Tml32Init(&cfg) == false)
	{
		printf("  TML32 init failed\r\n");

		return SELFTEST_FAIL;
	}

	R9A02_TML32_Type *reg = R9A02_TML32;
	uint8_t ctl = reg->ITLCTL0;
	uint8_t div = (uint8_t)((reg->ITLFDIV00 >> R9A02_ITLFDIV_FDIV0_Pos) & 7U);
	uint32_t cmp = reg->ITLCMP00;
	uint64_t ns = R9a02Tml32Period();
	bool m32 = (ctl & (R9A02_ITLCTL0_MD_16BIT | R9A02_ITLCTL0_MD_32BIT)) == R9A02_ITLCTL0_MD_32BIT;

	if (m32)
	{
		cmp |= (uint32_t)reg->ITLCMP01 << 16;
	}

	uint64_t end = Mtime() + MsTicks((uint32_t)(nsPeriod * (Cnt + 8U) / 1000000ULL) + 20U);

	while (s_TmlCnt < s_TmlLast && Mtime() < end)
	{
	}

	R9a02Tml32PowerOff();

	printf("  %s mode, ITLCMP 0x%08lX, divider 2^%u, period %lu ns\r\n", m32 ? "32-bit" : "16-bit",
		   (unsigned long)cmp, div, (unsigned long)ns);

	if (s_TmlCnt < s_TmlLast)
	{
		printf("  %lu intervals only\r\n", (unsigned long)s_TmlCnt);

		return SELFTEST_FAIL;
	}
	if (m32 != b32)
	{
		printf("  %s mode expected\r\n", b32 ? "32-bit" : "16-bit");

		return SELFTEST_FAIL;
	}

	uint64_t meas = s_TmlStamp[1] - s_TmlStamp[0];

	if (lf)
	{
		// Same oscillator: exact count of LOCO cycles per interval
		uint64_t unit = 1ULL << div;
		uint64_t exp = ((uint64_t)cmp + 1U) * unit * Cnt;
		uint64_t less = (uint64_t)cmp * unit * Cnt;
		uint64_t more = ((uint64_t)cmp + 2U) * unit * Cnt;

		printf("  %lu intervals: %lu machine timer ticks, (ITLCMP + 1) x 2^div gives %lu\r\n",
			   (unsigned long)Cnt, (unsigned long)meas, (unsigned long)exp);

		if (Diff((int)meas, (int)exp) <= 3)
		{
			return SELFTEST_PASS;
		}
		if (Diff((int)meas, (int)less) <= 3)
		{
			printf("  The interval is ITLCMP x 2^div: R9a02Tml32Init and R9a02Tml32Period are one count off\r\n");
		}
		else if (Diff((int)meas, (int)more) <= 3)
		{
			printf("  The interval is (ITLCMP + 2) x 2^div\r\n");
		}

		return SELFTEST_FAIL;
	}

	// HOCO against ICLK: the period the driver computed, within 0.1 %
	uint64_t exp = (ns * SystemCoreClock * Cnt + 500000000ULL) / 1000000000ULL;
	int err = (int)(((int64_t)meas - (int64_t)exp) * 1000000LL / (int64_t)exp);

	printf("  %lu intervals: %lu cycles, %lu expected, %d ppm\r\n", (unsigned long)Cnt, (unsigned long)meas,
		   (unsigned long)exp, err);

	return err >= -1000 && err <= 1000 ? SELFTEST_PASS : SELFTEST_FAIL;
}

static SELFTEST_RES TestDtcRepeat(void)
{
	R9a02Tml32Cfg_t cfg = s_Tml32Base;

	cfg.Mode = R9A02_TML32_MODE_DTC;
	if (R9a02Tml32Init(&cfg) == false)
	{
		printf("  TML32 init failed\r\n");

		return SELFTEST_FAIL;
	}

	int n = FindSlot(R9A02_EVT_TML32_OUTI);

	if (n < 0)
	{
		R9a02Tml32PowerOff();
		printf("  No route of TML32_OUTI\r\n");

		return SELFTEST_FAIL;
	}

	msDelay(3);
	CLIC_INT_IP_REG(R9A02_CLIC_CTRL_BASE, R9A02_CLIC_ID_IELSR0 + n) = 0U;

	uint32_t set = 0;
	uint32_t total = 0;
	uint64_t end = Mtime() + MsTicks(20);

	while (Mtime() < end)
	{
		if (R9A02_TML32->ITLS0 & R9A02_ITLS0_ITF00)
		{
			set++;
		}
		total++;
	}

	bool dtce = R9a02DtcEnabled(n);
	bool ir = R9a02EventFlag(n);
	uint8_t ip = ClicPending(n);

	// The slot back to the CPU, nothing clears the flag now
	R9a02DtcDisable(n);
	msDelay(3);

	uint8_t itls = R9A02_TML32->ITLS0;

	R9a02Tml32PowerOff();

	printf("  IELSR%d DTCE %d IR %d, CLIC pending %u, DTCVBR 0x%08lX\r\n", n, dtce ? 1 : 0, ir ? 1 : 0, ip,
		   (unsigned long)R9A02_DTC->DTCVBR);
	printf("  20 ms: flag read set %lu of %lu times, ITLS0 0x%02X once the DTC is off\r\n", (unsigned long)set,
		   (unsigned long)total, itls);

	if (s_bClicLatch)
	{
		printf("  %s\r\n", ip ? "The CLIC input took the events while DTCE was 1: the dispatcher check of DTCE is needed" :
			   "No CLIC request while DTCE was 1");
	}

	if ((itls & R9A02_ITLS0_ITF00) == 0U)
	{
		printf("  The TML32 flag did not set\r\n");

		return SELFTEST_FAIL;
	}
	if (dtce == false || set * 50U > total)
	{
		printf("  The DTC did not clear the TML32 flag at each interval\r\n");

		return SELFTEST_FAIL;
	}

	return SELFTEST_PASS;
}

static SELFTEST_RES TestSpiDtc(void)
{
	static const uint32_t rates[] = { 6000000, 1000000, 100000 };
	SELFTEST_RES res = SELFTEST_PASS;
	bool late = false;

	for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++)
	{
		for (int i = 0; i < 2; i++)
		{
			SPICfg_t cfg = s_SpiCfg;

			cfg.Rate = rates[r];
			cfg.bIntEn = i != 0;

			if (s_Spi.Init(cfg) == false)
			{
				printf("  %lu Hz: SPI init failed\r\n", (unsigned long)rates[r]);
				res = SELFTEST_FAIL;
				continue;
			}

			uint32_t rate = s_Spi.Rate();

			memset(s_SpiBuf, 0, sizeof(s_SpiBuf));
			s_bSpiDone = false;
			s_SpiCount = 0;

			int cnt = s_Spi.Rx(0, s_SpiBuf, SELFTEST_SPI_LEN);

			if (cnt < 0)
			{
				uint64_t end = Mtime() + MsTicks(50);

				while (s_bSpiDone == false && Mtime() < end)
				{
				}
				cnt = s_bSpiDone ? s_SpiCount : -1;
			}

			uint16_t ssr = R9A02_SAU1->SSR[0];

			// Three byte times for a late last byte to land in SDR20
			usDelay(24000000U / (rate ? rate : 1U) + 10U);

			uint16_t ssr2 = R9A02_SAU1->SSR[0];
			int ff = 0;

			for (int k = 0; k < SELFTEST_SPI_LEN; k++)
			{
				if (s_SpiBuf[k] == 0xFFU)
				{
					ff++;
				}
			}

			bool left = (ssr2 & (R9A02_SAU_SSR_BFF | R9A02_SAU_SSR_TSF | R9A02_SAU_SSR_OVF)) != 0U;
			bool ok = cnt == SELFTEST_SPI_LEN && ff == SELFTEST_SPI_LEN && !left;

			printf("  %lu Hz, DTC%s: %d bytes, %d at 0xFF, SSR20 0x%04X then 0x%04X%s\r\n", (unsigned long)rate,
				   cfg.bIntEn ? " and interrupt" : ", call waits", cnt, ff, ssr, ssr2, ok ? "" : ", wrong");

			if (cnt < 0)
			{
				printf("  No end of reception: the CPU interrupt after the last DTC transfer did not come\r\n");
				s_Spi.Reset();
			}
			late = late || (left && cnt == SELFTEST_SPI_LEN);
			if (ok == false)
			{
				res = SELFTEST_FAIL;
			}
		}
	}

	s_Spi.PowerOff();

	if (late)
	{
		printf("  Data or a transfer left in SDR20 after the end: the last byte was read before it ended\r\n");
	}

	return res;
}

static SELFTEST_RES TestDacAdc(void)
{
	static const uint8_t codes[] = { 32, 128, 224 };
	AdcChanCfg_t ch[2] = { AdcChan(0, 16), AdcChan(1, 17) };
	SELFTEST_RES res = SELFTEST_PASS;

	if (s_Adc.OpenChannel(ch, 2) == false)
	{
		printf("  A/D channels refused\r\n");

		return SELFTEST_FAIL;
	}

	for (size_t i = 0; i < sizeof(codes); i++)
	{
		uint8_t c0 = codes[i];
		uint8_t c1 = (uint8_t)(255U - c0);
		int raw[2];

		s_Dac.WriteCode(0, c0);
		s_Dac.WriteCode(1, c1);
		usDelay(100);
		AdcConvert(raw, 2);

		bool ok = Diff(raw[0], c0 * 16) <= SELFTEST_DAC_ADC_LIM && Diff(raw[1], c1 * 16) <= SELFTEST_DAC_ADC_LIM;

		printf("  DAC0 %3u: ANI16 %4d (%4d), DAC1 %3u: ANI17 %4d (%4d)%s\r\n", c0, raw[0], c0 * 16, c1, raw[1],
			   c1 * 16, ok ? "" : ", wrong");
		if (ok == false)
		{
			res = SELFTEST_FAIL;
		}
	}

	AdcCloseAll();
	DacPins();

	return res;
}

static SELFTEST_RES TestAdcScan(void)
{
	AdcChanCfg_t sel[2] = { AdcChan(0, 2), AdcChan(1, 3) };
	int v[2];
	int s1[4];
	int s2[4];

	// Select mode first, twice, the second one kept
	if (s_Adc.OpenChannel(sel, 2) == false)
	{
		printf("  A/D channels refused\r\n");

		return SELFTEST_FAIL;
	}
	AdcConvert(v, 2);
	AdcConvert(v, 2);
	AdcCloseAll();

	printf("  Select mode: ANI2 %d, ANI3 %d\r\n", v[0], v[1]);

	if (v[0] < 0 || v[1] < 0)
	{
		printf("  No conversion result\r\n");

		return SELFTEST_FAIL;
	}
	if (v[0] > 120 || v[1] < 4096 - 120)
	{
		printf("  A2 to GND and A3 to 3.3 V not wired\r\n");

		return SELFTEST_SKIP;
	}

	AdcChanCfg_t scan[4] = { AdcChan(0, 0), AdcChan(1, 1), AdcChan(2, 2), AdcChan(3, 3) };

	if (s_Adc.OpenChannel(scan, 4) == false)
	{
		printf("  A/D channels refused\r\n");

		return SELFTEST_FAIL;
	}

	// The first scan after select mode, an ADC_ENDI before the fourth
	// conversion would leave ADCR2 and ADCR3 of select mode
	AdcConvert(s1, 4);
	AdcConvert(s2, 4);

	bool admd = (R9A02_ADC12->ADM0 & R9A02_ADM0_ADMD) != 0U;

	AdcCloseAll();

	printf("  %s mode: ANI0 %d, ANI1 %d, ANI2 %d and %d, ANI3 %d and %d\r\n", admd ? "Scan" : "Select", s2[0],
		   s2[1], s1[2], s2[2], s1[3], s2[3]);

	if (admd == false)
	{
		printf("  The driver did not select scan mode\r\n");

		return SELFTEST_FAIL;
	}

	return Diff(s1[2], v[0]) <= 40 && Diff(s2[2], v[0]) <= 40 && Diff(s1[3], v[1]) <= 40 &&
		   Diff(s2[3], v[1]) <= 40 ? SELFTEST_PASS : SELFTEST_FAIL;
}

// Change in mV of an input, the only channel open, from DAC1 code 64 to
// 192. P105 and P106 are both analog during it.
static int AniStep(int Ani, bool *pOk)
{
	AdcChanCfg_t ch = AdcChan(0, Ani);
	int mv[2] = { 0, 0 };

	*pOk = false;
	if (s_Adc.OpenChannel(&ch, 1) == false)
	{
		return 0;
	}
	IOPinConfig(1, 5, IOPINOP_FUNC31, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(1, 6, IOPINOP_FUNC31, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	*pOk = true;
	for (int i = 0; i < 2; i++)
	{
		int raw = -1;

		s_Dac.WriteCode(1, i ? 192 : 64);
		usDelay(100);
		for (int k = 0; k < 3; k++)
		{
			AdcConvert(&raw, 1);
		}
		if (raw < 0)
		{
			*pOk = false;
		}
		mv[i] = raw * SELFTEST_VCC_MV / 4096;
	}

	AdcCloseAll();
	IOPinDisable(1, 5);
	IOPinDisable(1, 6);

	return mv[1] - mv[0];
}
static SELFTEST_RES TestAni1819(void)
{
	int step = 128 * SELFTEST_VCC_MV / 256;
	bool ok18;
	bool ok19;
	int d18 = AniStep(18, &ok18);
	int d19 = AniStep(19, &ok19);
	bool t18 = Diff(d18, step) <= 150;
	bool t19 = Diff(d19, step) <= 150;

	if (ok18 == false || ok19 == false)
	{
		printf("  No conversion result\r\n");

		return SELFTEST_FAIL;
	}

	printf("  DAC1 code 64 to 192, %d mV step: ANI18 %d mV, ANI19 %d mV\r\n", step, d18, d19);

	s_bWireP106 = t18 || t19;

	if (t19 && !t18)
	{
		printf("  ANI19 is P106, as the pin tables give\r\n");

		return SELFTEST_PASS;
	}
	if (t18 && !t19)
	{
		printf("  ANI18 is P106 as table 29.12 gives: the driver pin table must swap ANI18 and ANI19\r\n");

		return SELFTEST_FAIL;
	}
	if (!t18 && !t19)
	{
		printf("  No input follows DAC1: P001 to P106 not wired\r\n");

		return SELFTEST_SKIP;
	}

	printf("  Both inputs follow DAC1: P105 tied to P106?\r\n");

	return SELFTEST_FAIL;
}

// DAC1 codes at which comparator 0 goes above, then below, its reference
static bool CompSweep(const AnalogCompCfg_t &Cfg, int *pUp, int *pDown)
{
	*pUp = -1;
	*pDown = -1;

	s_Dac.WriteCode(1, 0);
	usDelay(50);

	if (s_Comp.Init(Cfg) == false)
	{
		return false;
	}
	if (s_Comp.Start() == false)
	{
		s_Comp.PowerOff();

		return false;
	}

	// Events of the settling time dropped
	usDelay(100);
	s_CompEvt = 0;

	for (int c = 0; c < 256 && *pUp < 0; c++)
	{
		s_Dac.WriteCode(1, (uint8_t)c);
		usDelay(20);
		if (s_CompEvt > 0)
		{
			*pUp = c;
		}
	}

	s_CompEvt = 0;
	for (int c = *pUp; c >= 0 && *pDown < 0; c--)
	{
		s_Dac.WriteCode(1, (uint8_t)c);
		usDelay(20);
		if (s_CompEvt < 0)
		{
			*pDown = c;
		}
	}

	s_Comp.PowerOff();

	return true;
}

static SELFTEST_RES TestComp(void)
{
	if (s_bWireP106 == false)
	{
		printf("  P001 to P106 not wired (test 9)\r\n");

		return SELFTEST_SKIP;
	}

	SELFTEST_RES res = SELFTEST_PASS;
	AnalogCompCfg_t cfg = s_CompBase;
	int up;
	int down;

	// Internal reference 1.34 to 1.54 V, offset 40 mV (table 2.50), DAC8 3
	// LSB (table 2.51): codes 97 to 126 at 3.3 V
	int lo = (1340 - 40) * 256 / SELFTEST_VCC_MV - 3;
	int hi = ((1540 + 40) * 256 + SELFTEST_VCC_MV - 1) / SELFTEST_VCC_MV + 3;

	if (CompSweep(cfg, &up, &down) == false)
	{
		printf("  Comparator init failed\r\n");

		return SELFTEST_FAIL;
	}
	printf("  Internal reference: above at DAC1 code %d, below at %d (%d to %d)\r\n", up, down, lo, hi);
	if (up < lo || up > hi || down < 0 || up - down > 4)
	{
		res = SELFTEST_FAIL;
	}

	// IVREF0 (P000) driven by DAC0, range 0 to VCC - 1.4 V
	uint8_t ref = 78;

	s_Dac.WriteCode(0, ref);
	DacPins();
	cfg.RefSrc = 1;
	cfg.CompVolt = ref * SELFTEST_VCC_MV / 256;
	if (CompSweep(cfg, &up, &down) == false)
	{
		printf("  Comparator init with IVREF0 failed\r\n");

		return SELFTEST_FAIL;
	}
	DacPins();
	printf("  IVREF0 at DAC0 code %u: above at DAC1 code %d, below at %d\r\n", ref, up, down);
	// Both DAC8 channels 3 LSB, offset 40 mV
	if (up < 0 || Diff(up, ref) > 10 || down < 0 || up - down > 4)
	{
		res = SELFTEST_FAIL;
	}

	return res;
}

// Voltage that Window turns into the 8-bit Code
static float WinVolt(int Code)
{
	return Code >= 255 ? SELFTEST_VREF : ((float)Code + 0.5f) * SELFTEST_VREF / 256.0f;
}

static SELFTEST_RES TestAdcWindow(void)
{
	AdcChanCfg_t ch = AdcChan(0, 2);
	int rmax = -1;

	// ANI2 at GND: the 8-bit result is 0. A DAC8 code k gives 16 k, on the
	// edge of an 8-bit step, and cannot be used.
	if (s_Adc.OpenChannel(&ch, 1) == false)
	{
		printf("  A/D channel refused\r\n");

		return SELFTEST_FAIL;
	}

	for (int k = 0; k < 8; k++)
	{
		int raw;

		AdcConvert(&raw, 1);
		rmax = raw > rmax ? raw : rmax;
		if (raw < 0)
		{
			rmax = 4096;
		}
	}

	printf("  ANI2 results up to %d\r\n", rmax);

	if (rmax >= 4096)
	{
		AdcCloseAll();
		printf("  No conversion result\r\n");

		return SELFTEST_FAIL;
	}
	if (rmax >= 16)
	{
		AdcCloseAll();
		printf("  A2 not at GND\r\n");

		return SELFTEST_SKIP;
	}

	R9a02Tml32Cfg_t cfg = s_Tml32Base;

	cfg.Mode = R9A02_TML32_MODE_DTC;
	if (R9a02Tml32Init(&cfg) == false || s_Adc.Trigger(R9A02_ADC_TRIG_TML32) == false)
	{
		R9a02Tml32PowerOff();
		AdcCloseAll();
		printf("  TML32 trigger failed\r\n");

		return SELFTEST_FAIL;
	}

	SELFTEST_RES res = SELFTEST_PASS;
	bool adll = true;

	for (size_t i = 0; i < sizeof(s_Win) / sizeof(s_Win[0]); i++)
	{
		const SelfTestWin_t *w = &s_Win[i];

		if (s_Adc.Window(WinVolt(w->Adll), WinVolt(255), w->bOutside) == false)
		{
			printf("  Window %u to 255 refused\r\n", w->Adll);
			res = SELFTEST_FAIL;
			continue;
		}

		s_AdcEvt = 0;
		s_Adc.StartConversion();
		msDelay(SELFTEST_WIN_TRIG);
		s_Adc.StopConversion();

		int cnt = s_AdcEvt;
		bool evt = cnt >= SELFTEST_WIN_TRIG / 2;
		bool steady = evt || cnt == 0;

		printf("  ADRCK %d, ADLL %u, ADUL 255: %2d ADC_ENDI, %s expected%s\r\n", w->bOutside ? 1 : 0, w->Adll, cnt,
			   w->bEvt ? "some" : "none", !steady ? ", unsteady" : evt != w->bEvt ? ", wrong" : "");

		if (steady == false)
		{
			if (res == SELFTEST_PASS)
			{
				res = SELFTEST_SKIP;
			}
		}
		else if (evt != w->bEvt)
		{
			res = SELFTEST_FAIL;
			if (w->bOutside && w->Adll == 0U)
			{
				adll = false;
			}
		}
	}

	s_Adc.WindowOff();
	s_Adc.Trigger(R9A02_ADC_TRIG_SOFT);
	R9a02Tml32PowerOff();
	AdcCloseAll();

	if (adll == false)
	{
		printf("  With ADRCK 1 a result equal to ADLL gives ADC_ENDI (29.3.8 text): Window(bOutside) must set ADLL one below Low\r\n");
	}

	return res;
}
static SELFTEST_RES TestCompWake(void)
{
	if (s_bWireP106 == false)
	{
		printf("  P001 to P106 not wired (test 9)\r\n");

		return SELFTEST_SKIP;
	}

	R9a02Tml32Cfg_t cfg = s_Tml32Base;

	// P106 above the reference: DAC1 at VCC through its 9 kOhm, safe to short
	s_Dac.WriteCode(1, 255);
	usDelay(50);

	s_CompCnt = 0;
	s_WakeBy = 0;
	if (s_Comp.Init(s_CompBase) == false || s_Comp.Start() == false)
	{
		s_Comp.PowerOff();
		printf("  Comparator init failed\r\n");

		return SELFTEST_FAIL;
	}
	usDelay(100);
	if ((R9A02_CMP->COMPMDR & R9A02_COMPMDR_CMON(0)) == 0U)
	{
		s_Comp.PowerOff();
		printf("  P106 is below the reference\r\n");

		return SELFTEST_SKIP;
	}

	// Time-out 15 s, the TML32 on the LOCO runs in Software Standby
	cfg.nsPeriod = 15000000000ULL;
	cfg.Handler = Tml32TimeoutHandler;
	cfg.bWakeup = true;
	if (R9a02WakeupSet(R9A02_EVT_COMP_DET0, true) == false || R9a02Tml32Init(&cfg) == false)
	{
		R9a02WakeupSet(R9A02_EVT_COMP_DET0, false);
		R9a02Tml32PowerOff();
		s_Comp.PowerOff();
		printf("  Wake-up set up failed\r\n");

		return SELFTEST_FAIL;
	}

	s_CompCnt = 0;
	s_WakeBy = 0;
	printf("  Software Standby: touch P106 (or P001) to GND within 15 s\r\n");
	UartWaitIdle();

	// A/D converter ready for Software Standby
	s_Adc.Disable();
	bool sby = R9a02LowPowerEnter(true);
	usDelay(100);
	s_Adc.Enable();

	int by = s_WakeBy;

	R9a02WakeupSet(R9A02_EVT_COMP_DET0, false);
	R9a02Tml32PowerOff();
	s_Comp.PowerOff();

	if (by == 1)
	{
		printf("  Woken up by the comparator\r\n");

		return SELFTEST_PASS;
	}

	if (sby == false || by != 2)
	{
		printf("  %s\r\n", sby ? "Woken up without a comparator or time-out event (debugger?)" :
			   "Software Standby not entered");

		return SELFTEST_SKIP;
	}

	printf("  Woken up by the 15 s time-out. Was P106 touched to GND (y/n)?\r\n");

	// Characters typed during Software Standby are dropped
	uint8_t c;

	while (s_Uart.Rx(&c, 1) > 0)
	{
	}

	int k = KeyGet(30000);

	if (k == 'y' || k == 'Y')
	{
		printf("  The comparator does not end Software Standby\r\n");

		return SELFTEST_FAIL;
	}

	return SELFTEST_SKIP;
}

int main()
{
	IOPinCfg(s_Leds, s_NbLeds);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinClear(s_Leds[1].PortNo, s_Leds[1].PinNo);

	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1) __WFE();
	}
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
	setvbuf(stdout, NULL, _IONBF, 0);

	printf("\r\nR9A02G021 self-test\r\n");
	printf("Wiring: P001 to P106, A2 (P400) to GND, A3 (P401) to 3.3 V\r\n");

	if (s_Mtimer.Init(s_MtimerCfg) == false || s_Adc.Init(s_AdcCfg) == false || s_Dac.Init(s_DacCfg) == false ||
		s_Dac.OpenChannel(0, true) == false || s_Dac.OpenChannel(1, true) == false)
	{
		printf("Machine timer, A/D or D/A init failed\r\n");
		while (1) __WFE();
	}

	Begin(1, "ICU flag of a route with its CLIC input off");
	End(TestIcuPoll());

	Begin(2, "TML32 interval, LOCO, 16-bit mode, machine timer count");
	End(TestTml32Period(R9A02_TML32_CLK_LF, 122070ULL, 256, false));

	Begin(3, "TML32 interval, HOCO, 16-bit mode with divider, mcycle count");
	End(TestTml32Period(R9A02_TML32_CLK_HOCO, 2000000ULL, 16, false));

	Begin(4, "TML32 interval, HOCO, 32-bit mode, mcycle count");
	End(TestTml32Period(R9A02_TML32_CLK_HOCO, 250000000ULL, 4, true));

	Begin(5, "DTC repeat transfer on TML32_OUTI");
	End(TestDtcRepeat());

	Begin(6, "SPI20 reception by the DTC, end of the last byte");
	End(TestSpiDtc());

	Begin(7, "DAC8 outputs read on ANI16 and ANI17, 12-bit results (expected)");
	End(TestDacAdc());

	Begin(8, "A/D scan ANI0 to ANI3 against select mode, 12-bit results");
	End(TestAdcScan());

	Begin(9, "ANI18 and ANI19 pins");
	End(TestAni1819());

	Begin(10, "Comparator 0 references");
	End(TestComp());

	Begin(11, "A/D compare window, result equal to ADLL");
	End(TestAdcWindow());

	if (SELFTEST_CMP_WAKE)
	{
		printf("\r\nTests 1 to 11: %d PASS, %d FAIL, %d SKIP\r\n", s_ResCnt[SELFTEST_PASS], s_ResCnt[SELFTEST_FAIL],
			   s_ResCnt[SELFTEST_SKIP]);

		Begin(12, "Comparator 0 wake-up from Software Standby");
		End(TestCompWake());
	}

	printf("\r\nSelf-test done: %d PASS, %d FAIL, %d SKIP\r\n", s_ResCnt[SELFTEST_PASS], s_ResCnt[SELFTEST_FAIL],
		   s_ResCnt[SELFTEST_SKIP]);

	// LED1 on: no failure, LED2 on: a failure
	if (s_ResCnt[SELFTEST_FAIL] == 0)
	{
		IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	}
	else
	{
		IOPinClear(s_Leds[0].PortNo, s_Leds[0].PinNo);
		IOPinSet(s_Leds[1].PortNo, s_Leds[1].PinNo);
	}

	while (1)
	{
		__WFE();
	}

	return 0;
}
