/**-------------------------------------------------------------------------
@file	pei_spi_demo.cpp
@brief	nRF52832 SPIM2 hardware START -> END -> STOP through PPI.

The SPIM master transmits eight bytes without an external SPI device.
BUT1 hardware GPIOTE starts the preconfigured EasyDMA transfer.
SPIM END triggers STOP and LED1. SPIM STOPPED triggers RGB blue.
All event forwarding is PPI hardware, not a CPU interrupt.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/spi.h"
#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;

static const IOPinCfg_t s_SpiPins[] = {
	{0, 14, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, // SCK
	{0, 16, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},  // MISO, not connected
	{0, 15, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, // MOSI
};

static const SPICfg_t s_SpiCfg = {
	.DevNo = 2,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(s_SpiPins[0]),
	.Rate = 1000000U,
	.DataSize = 8,
	.MaxRetry = 0,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_LOW,
	.ChipSel = SPICSEL_MAN,
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.DummyByte = 0xFF,
	.EvtCB = nullptr
};

static const uint8_t s_TestData[] = {
	0xA5, 0x5A, 0x11, 0x22, 0x33, 0x44, 0x55, 0xCC,
};

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 4,
	"PeiSpiDemo requires LED1 and RGB blue");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 1,
	"PeiSpiDemo requires BUT1");

volatile int g_PeiSpiStatus;

int main(void)
{
	Pei pei;
	SPI spi;
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

	if (!spi.Init(s_SpiCfg))
	{
		g_PeiSpiStatus = -1;
		return 1;
	}
	if (!spi.PreparePeiTransfer(s_TestData, sizeof(s_TestData), nullptr, 0U))
	{
		g_PeiSpiStatus = -2;
		return 1;
	}
	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiSpiStatus = -3;
		return 1;
	}
	if (!spi.ConnectTask(pei, startEvt, SPI_PEI_TASK_START))
	{
		g_PeiSpiStatus = -4;
		return 1;
	}
	if (!spi.EnablePeiEvent(pei, endEvt, SPI_PEI_EVENT_END))
	{
		g_PeiSpiStatus = -5;
		return 1;
	}
	if (!spi.ConnectTask(pei, endEvt, SPI_PEI_TASK_STOP))
	{
		g_PeiSpiStatus = -6;
		return 1;
	}
	if (!IOPinEnableTask(pei, endEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiSpiStatus = -7;
		return 1;
	}
	if (!spi.EnablePeiEvent(pei, stoppedEvt, SPI_PEI_EVENT_STOPPED))
	{
		g_PeiSpiStatus = -8;
		return 1;
	}
	if (!IOPinEnableTask(pei, stoppedEvt, s_Leds[3].PortNo,
		s_Leds[3].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiSpiStatus = -9;
		return 1;
	}
	if (!pei.Enable(stoppedEvt))
	{
		g_PeiSpiStatus = -10;
		return 1;
	}
	if (!pei.Enable(endEvt))
	{
		g_PeiSpiStatus = -11;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiSpiStatus = -12;
		return 1;
	}

	g_PeiSpiStatus = 1;
	for (;;)
	{
		__WFE();
	}
}
