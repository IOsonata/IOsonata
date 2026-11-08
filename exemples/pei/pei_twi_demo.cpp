/**-------------------------------------------------------------------------
@file	pei_twi_demo.cpp
@brief	nRF52832 TWIM1 error/STOP hardware event-to-task demonstration.

BUT1 sends a one-byte TWIM1 write to an unpopulated address over PPI.
The expected address NACK produces TWIM.ERROR, which triggers STOP
and LED1 via PPI. TWIM.STOPPED toggles RGB blue. No TWI IRQ is
involved; all transfer DMA belongs to the I2C driver.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/i2c.h"
#include "coredev/pei.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const IOPinCfg_t s_Buttons[] = BUTTON_PINS_MAP;
static const IOPinCfg_t s_I2cPins[] = {
	{0, 14, IOPINOP_GPIO, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, // SDA
	{0, 15, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, // SCL
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = 1,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(s_I2cPins[0]),
	.Rate = 100000U,
	.MaxRetry = 0,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCB = nullptr
};

static const uint8_t s_TestData[] = { 0xA5 };

static_assert(sizeof(s_Leds) / sizeof(s_Leds[0]) >= 4,
	"PeiTwiDemo requires separate LED1 and RGB blue");
static_assert(sizeof(s_Buttons) / sizeof(s_Buttons[0]) >= 1,
	"PeiTwiDemo requires BUT1");

volatile int g_PeiTwiStatus;

int main(void)
{
	Pei pei;
	I2C i2c;
	PeiEvent_t startEvt = 0U;
	PeiEvent_t errorEvt = 0U;
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

	if (!i2c.Init(s_I2cCfg))
	{
		g_PeiTwiStatus = -1;
		return 1;
	}
	// The selected address is deliberately not populated on this bus.
	if (!i2c.PreparePeiTransfer(PEI_TWI_EMPTY_ADDRESS, s_TestData,
		sizeof(s_TestData), nullptr, 0U))
	{
		g_PeiTwiStatus = -2;
		return 1;
	}
	if (!IOPinEnableEvent(pei, startEvt, s_Buttons[0].PortNo,
		s_Buttons[0].PinNo, IOPINSENSE_LOW_TRANSITION))
	{
		g_PeiTwiStatus = -3;
		return 1;
	}
	if (!i2c.ConnectTask(pei, startEvt, I2C_PEI_TASK_STARTTX))
	{
		g_PeiTwiStatus = -4;
		return 1;
	}
	if (!i2c.EnablePeiEvent(pei, errorEvt, I2C_PEI_EVENT_ERROR))
	{
		g_PeiTwiStatus = -5;
		return 1;
	}
	if (!i2c.ConnectTask(pei, errorEvt, I2C_PEI_TASK_STOP))
	{
		g_PeiTwiStatus = -6;
		return 1;
	}
	if (!IOPinEnableTask(pei, errorEvt, s_Leds[0].PortNo,
		s_Leds[0].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiTwiStatus = -7;
		return 1;
	}
	if (!i2c.EnablePeiEvent(pei, stoppedEvt, I2C_PEI_EVENT_STOPPED))
	{
		g_PeiTwiStatus = -8;
		return 1;
	}
	if (!IOPinEnableTask(pei, stoppedEvt, s_Leds[3].PortNo,
		s_Leds[3].PinNo, IOPIN_TASK_TOGGLE))
	{
		g_PeiTwiStatus = -9;
		return 1;
	}

	if (!pei.Enable(stoppedEvt))
	{
		g_PeiTwiStatus = -10;
		return 1;
	}
	if (!pei.Enable(errorEvt))
	{
		g_PeiTwiStatus = -11;
		return 1;
	}
	if (!pei.Enable(startEvt))
	{
		g_PeiTwiStatus = -12;
		return 1;
	}
	g_PeiTwiStatus = 1;

	for (;;)
	{
		__WFE();
	}
}
