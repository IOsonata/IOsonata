/**-------------------------------------------------------------------------
@file	blinky.c
@brief	ESP32-C6-DEV-KIT-N8 onboard WS2812B RGB LED demonstration.

RMT channel 0 sends one pixel using the 40 MHz crystal, independently of CPU
speed and compiler optimization. GPIO selection belongs in board.h. This
example owns the RMT peripheral; it does not enable interrupts or DMA.

@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026
@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"
#include "idelay.h"
#include "board.h"

#define PCR_RMT_CONF			(*(volatile uint32_t *)0x6009602CUL)
#define PCR_RMT_SCLK_CONF	(*(volatile uint32_t *)0x60096030UL)
#define RMT_DATA			(*(volatile uint32_t *)0x60006000UL)
#define RMT_CONF			(*(volatile uint32_t *)0x60006010UL)
#define RMT_INT_RAW			(*(volatile uint32_t *)0x60006038UL)
#define RMT_INT_ENA			(*(volatile uint32_t *)0x60006040UL)
#define RMT_INT_CLR			(*(volatile uint32_t *)0x60006044UL)
#define RMT_SYS_CONF		(*(volatile uint32_t *)0x60006068UL)
#define RMT_REF_CNT_RST		(*(volatile uint32_t *)0x60006070UL)

#define RMT_TX_START		(1UL << 0)
#define RMT_MEM_RD_RST		(1UL << 1)
#define RMT_APB_MEM_RST		(1UL << 2)
#define RMT_CONF_UPDATE		(1UL << 24)
#define RMT_TX_END			(1UL << 0)
#define RMT_TX_ERROR		(1UL << 4)

// Channel divider 1, one 48-word memory block, idle output enabled and low.
#define RMT_PIXEL_CONF		((1UL << 8) | (1UL << 16) | (1UL << 6))
// 25 ns ticks: 0 = 400 ns high / 850 ns low; 1 = 800 ns / 450 ns.
#define WS2812_ZERO			(16UL | (1UL << 15) | (34UL << 16))
#define WS2812_ONE			(32UL | (1UL << 15) | (18UL << 16))

static void LedInit(void)
{
	PCR_RMT_CONF = 3U;
	PCR_RMT_CONF = 1U;
	// XTAL clock source, no fractional or integer prescaling.
	PCR_RMT_SCLK_CONF = (3UL << 20) | (1UL << 22);
	// FIFO access, memory clock on, memory powered, register clock on.
	RMT_SYS_CONF = (1UL << 1) | (1UL << 3) | (1UL << 31);
	RMT_INT_ENA = 0U;
	RMT_INT_CLR = 0xFFFFFFFFUL;
	RMT_CONF = RMT_PIXEL_CONF;
	RMT_CONF = RMT_PIXEL_CONF | RMT_CONF_UPDATE;
	RMT_REF_CNT_RST = 1U;
	IOPinConfig(LED1_PORT, LED1_PIN, LED1_PINOP,
				IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	usDelay(300);
}

static bool LedPixel(uint8_t Red, uint8_t Green, uint8_t Blue)
{
	uint32_t pixel = ((uint32_t)Green << 16) | ((uint32_t)Red << 8) | Blue;
	uint32_t mask;
	uint32_t start;
	uint32_t status;

	RMT_CONF = RMT_PIXEL_CONF | RMT_MEM_RD_RST | RMT_APB_MEM_RST;
	RMT_CONF = RMT_PIXEL_CONF;
	for (mask = 1UL << 23; mask != 0U; mask >>= 1)
	{
		RMT_DATA = (pixel & mask) != 0U ? WS2812_ONE : WS2812_ZERO;
	}
	// 300 us low reset/latch, followed by the zero-duration end marker.
	RMT_DATA = 12000U;
	RMT_DATA = 0U;
	RMT_INT_CLR = RMT_TX_END | RMT_TX_ERROR;
	RMT_CONF = RMT_PIXEL_CONF | RMT_CONF_UPDATE;
	RMT_CONF = RMT_PIXEL_CONF | RMT_TX_START;
	start = IOsonataRiscvCycle32();
	do
	{
		status = RMT_INT_RAW;
		if ((uint32_t)(IOsonataRiscvCycle32() - start) >= SystemCoreClock / 1000U)
		{
			return false;
		}
	} while ((status & (RMT_TX_END | RMT_TX_ERROR)) == 0U);

	return (status & RMT_TX_ERROR) == 0U;
}

int main(void)
{
	LedInit();
	while (1)
	{
		if (!LedPixel(32, 0, 0))
		{
			break;
		}
		msDelay(500);
		if (!LedPixel(0, 32, 0))
		{
			break;
		}
		msDelay(500);
		if (!LedPixel(0, 0, 32))
		{
			break;
		}
		msDelay(500);
	}

	// Stop here on a peripheral error so it is visible in the debugger.
	while (1)
	{
	}
}
