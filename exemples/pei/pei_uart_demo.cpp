/**-------------------------------------------------------------------------
@file	pei_uart_demo.cpp
@brief	nRF52832 UARTE0 TX EasyDMA event/task chain through PPI.

BUT1 starts a driver-staged eight-byte TX in hardware. ENDTX toggles
LED1 and drives STOPTX; TXSTOPPED toggles RGB blue. The peripheral
TX pin emits the bytes without any remote receiver or CPU event
forwarding. The ordinary UARTE RX DMA remains driver-owned.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/uart.h"
#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static const IOPinCfg_t s_UartPins[] = {
	{BLYST_NANO_DK_UART_RX_PORT, BLYST_NANO_DK_UART_RX_PIN,
		BLYST_NANO_DK_UART_RX_PINOP, IOPINDIR_INPUT,
		IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLYST_NANO_DK_UART_TX_PORT, BLYST_NANO_DK_UART_TX_PIN,
		BLYST_NANO_DK_UART_TX_PINOP, IOPINDIR_OUTPUT,
		IOPINRES_NONE, IOPINTYPE_NORMAL}
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = nullptr,
	.bFifoBlocking = false,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
	.bIrDAMode = false,
	.bIrDAInvert = false,
	.bIrDAFixPulse = false,
	.IrDAPulseDiv = 0,
	.Duplex = UART_DUPLEX_FULL,
	.Mode = UART_MODE_UART
};

static const uint8_t s_TestData[] = {
	0xA5, 0x5A, 0x11, 0x22, 0x33, 0x44, 0x55, 0xCC
};

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 4,
	"PeiUartDemo needs discrete LED1 and RGB blue");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 1,
	"PeiUartDemo needs BUT1");

volatile int g_PeiUartStatus;

int main(void)
{
	Pei pei;
	UART uart;
	PeiEvent_t startEvt = 0U;
	PeiEvent_t endEvt = 0U;
	PeiEvent_t stoppedEvt = 0U;
	int i;

	IOPinConfig(s_Leds[0].PortNo, s_Leds[0].PinNo, s_Leds[0].PinOp,
		s_Leds[0].PinDir, s_Leds[0].Res, s_Leds[0].Type);
	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);
	IOPinConfig(s_Leds[3].PortNo, s_Leds[3].PinNo, s_Leds[3].PinOp,
		s_Leds[3].PinDir, s_Leds[3].Res, s_Leds[3].Type);
	IOPinSet(s_Leds[3].PortNo, s_Leds[3].PinNo);
	for (i = 0; i < 2; i++)
	{
		IOPinConfig(s_Buttons[i].PortNo, s_Buttons[i].PinNo,
			s_Buttons[i].PinOp, s_Buttons[i].PinDir,
			s_Buttons[i].Res, s_Buttons[i].Type);
	}

	if (!uart.Init(s_UartCfg))
	{
		g_PeiUartStatus = -1;
		return 1;
	}
	if (!uart.PreparePeiTx(s_TestData, sizeof(s_TestData)))
	{
		g_PeiUartStatus = -2;
		return 1;
	}
	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiUartStatus = -3;
		return 1;
	}
	if (!uart.ConnectTask(pei, startEvt, UART_PEI_TASK_STARTTX))
	{
		g_PeiUartStatus = -4;
		return 1;
	}
	if (!uart.EnablePeiEvent(pei, endEvt, UART_PEI_EVENT_ENDTX))
	{
		g_PeiUartStatus = -5;
		return 1;
	}
	if (!uart.ConnectTask(pei, endEvt, UART_PEI_TASK_STOPTX))
	{
		g_PeiUartStatus = -6;
		return 1;
	}
	if (!IOPinEnableTask(pei, endEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiUartStatus = -7;
		return 1;
	}
	if (!uart.EnablePeiEvent(pei, stoppedEvt, UART_PEI_EVENT_TXSTOPPED))
	{
		g_PeiUartStatus = -8;
		return 1;
	}
	if (!IOPinEnableTask(pei, stoppedEvt, s_Leds[3].PortNo,
		s_Leds[3].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiUartStatus = -9;
		return 1;
	}
	if (!pei.Enable(stoppedEvt))
	{
		g_PeiUartStatus = -10;
		return 1;
	}
	if (!pei.Enable(endEvt))
	{
		g_PeiUartStatus = -11;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiUartStatus = -12;
		return 1;
	}
	g_PeiUartStatus = 1;

	for (;;)
	{
		__WFE();
	}
}
