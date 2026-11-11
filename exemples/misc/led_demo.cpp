/**-------------------------------------------------------------------------
@example	led_demo.cpp

@brief	Example code using LED object.


@author	Hoang Nguyen Hoan
@date	Feb. 23, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "coredev/pwm.h"
#include "idelay.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "miscdev/led.h"

#include "board.h"

#ifndef LED_DEMO_SPI_ENABLE
#define LED_DEMO_SPI_ENABLE 0
#endif

#if LED_DEMO_SPI_ENABLE
#include "coredev/spi.h"
#include "miscdev/led_ws2812.h"

#if defined(LED_DEMO_TRACE) && LED_DEMO_TRACE
#include <stdio.h>
#include "coredev/uart.h"
#include "stddev.h"

static const IOPinCfg_t s_UartPins[] = UART_PINS;
static uint8_t s_UartTx[CFIFO_MEMSIZE(256)];
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
	.IntPrio = 6,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTx),
	.pTxMem = s_UartTx,
	.bDMAMode = false
};
static UART s_Uart;
#define LED_TRACE(Message) printf("LedDemo: " Message "\r\n")
#else
#define LED_TRACE(Message) ((void)0)
#endif

static const IOPinCfg_t s_SpiPins[] = LED_SPI_PINS;
static const SPICfg_t s_SpiCfg = {
	.DevNo = LED_SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = LED_SPI_RATE,
	.DataSize = 8,
	.MaxRetry = 0,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_MAN,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 6,
	.DummyByte = 0,
	.EvtCB = NULL
};
static const WS2812_CFG s_LedCfg = {
	.NbLed = 1,
	.Order = WS2812_ORDER_GRB,
	.Brightness = 32,
	.DevCs = 0
};

SPI g_Spi;
LedWs2812 g_Led;

int main()
{
#if defined(LED_DEMO_TRACE) && LED_DEMO_TRACE
	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
	LED_TRACE("start");
	if (!g_Spi.Init(s_SpiCfg))
	{
		LED_TRACE("SPI init failed");
		while (1)
		{
		}
	}
	LED_TRACE("SPI initialized");
	if (!g_Led.Init(s_LedCfg, g_Spi))
	{
		LED_TRACE("LED init failed");
		while (1)
		{
		}
	}
	LED_TRACE("initial off");
	g_Led.Off();
	LED_TRACE("initial off returned");
	while (1)
	{
		LED_TRACE("red");
		g_Led.Level(WS2812_RED);
		LED_TRACE("transfer returned");
		msDelay(500);
		LED_TRACE("green");
		g_Led.Level(WS2812_GREEN);
		LED_TRACE("transfer returned");
		msDelay(500);
		LED_TRACE("blue");
		g_Led.Level(WS2812_BLUE);
		LED_TRACE("transfer returned");
		msDelay(500);
		LED_TRACE("off");
		g_Led.Off();
		LED_TRACE("transfer returned");
		msDelay(500);
	}
}
#else
#ifndef LED_DEMO_PWM_ENABLE
#define LED_DEMO_PWM_ENABLE 1
#endif

#if LED_DEMO_PWM_ENABLE
static const PwmCfg_t s_PwmCfg = {
	.DevNo = 0,
	.Freq = 100,
	.Mode = PWM_MODE_EDGE,
	.bIntEn = false,
	.IntPrio = 6,
	.pEvtHandler = NULL
};

static const PwmChanCfg_t s_PwmChanCfg[] = {
	{
		.Chan = 0,
		.Pol = PWM_POL_HIGH,
		.Port = LED2_PORT,
		.Pin = LED2_PIN,
	},
	{
		.Chan = 1,
		.Pol = PWM_POL_HIGH,
		.Port = LED3_PORT,
		.Pin = LED3_PIN,
	},
#ifdef LED4_PORT
	{
		.Chan = 2,
		.Pol = PWM_POL_HIGH,
		.Port = LED4_PORT,
		.Pin = LED4_PIN,
	},
#endif
};

const int s_NbPwmChan = sizeof(s_PwmChanCfg) / sizeof(PwmChanCfg_t);

Pwm g_Pwm;
LedPwm g_Led2Pwm;
#endif

Led g_Led1;
Led g_Led2;
Led g_Led3;
#ifdef LED4_PORT
Led g_Led4;
#endif

int main()
{
#if LED_DEMO_PWM_ENABLE
	uint32_t x = 0;

	g_Pwm.Init(s_PwmCfg);
#endif

	g_Led1.Init(LED1_PORT, LED1_PIN, LED1_ACTIVE);

	g_Led2.Init(LED2_PORT, LED2_PIN, LED2_ACTIVE);
	g_Led3.Init(LED3_PORT, LED3_PIN, LED3_ACTIVE);
#ifdef LED4_PORT
	g_Led4.Init(LED4_PORT, LED4_PIN, LED4_ACTIVE);
#endif

#if !LED_DEMO_PWM_ENABLE
	while (1)
#endif
	{
		g_Led1.Toggle();
		msDelay(1000);
		g_Led2.On();
		msDelay(1000);
		g_Led2.Off();
		g_Led3.Toggle();
		msDelay(1000);
		g_Led3.Off();
#ifdef LED4_PORT
		g_Led4.Toggle();
		msDelay(1000);
		g_Led4.Off();
#endif
	}

#if LED_DEMO_PWM_ENABLE
	g_Led2Pwm.Init(&g_Pwm, (PwmChanCfg_t*)s_PwmChanCfg, s_NbPwmChan);
	g_Led1.On();
	g_Led2Pwm.Level(0xFF);
	g_Led2Pwm.Level(0xFF00);
	g_Led2Pwm.Level(0xFF0000);
	g_Led2.Toggle();
	g_Led1.Off();

	while (1)
	{
		g_Led1.Toggle();
		g_Led2Pwm.Level(x);

		msDelay(100);

		//g_Led2.Toggle();

//		x += 0xFF800F;

		x >>= 1;

		if (x == 0)
		{
			x = 0xFFFFFF;
		}
		msDelay(10);
	}
#endif
}

#endif // LED_DEMO_SPI_ENABLE
