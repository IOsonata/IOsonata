/**-------------------------------------------------------------------------
@example	ble_nvm_test.cpp

@brief	Exercise NVM operations while Bluetooth advertising is active.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

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
#include <string.h>

#include "istddef.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"
#include "idelay.h"
#include "board.h"
#include "storage/nvm.h"

#include "storage/nvm_mcu_intrf.h"
#include "bluetooth/bt_app.h"
#include "app_evt_handler.h"
#include "stddev.h"

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif
#ifndef UART_BAUDRATE
#define UART_BAUDRATE 1000000
#endif

static const IOPinCfg_t s_UartPins[] = UART_PINS;

#define UART_TXFIFO_MEMSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartTxFifoMem[UART_TXFIFO_MEMSIZE];

static const UARTCfg_t s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = UART_TXFIFO_MEMSIZE,
	.pTxMem = s_UartTxFifoMem,
	.bDMAMode = true,
};

static UART s_Uart;
static Nvm s_Nvm;
static NvmMcuIntrf s_MemIntrf;
static Timer s_AdvTimer;

// The on-air counter changes once per second, independently of NVM writes.
// Keep the existing four-byte manufacturer field for legacy advertising.
static uint32_t s_AdvCounter;
static uint32_t s_Verified;
static bool s_HeartbeatReady;
alignas(4) static uint8_t s_EvtQueue[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static const BtAppCfg_t s_BtAppCfg = {
	.Role = BTAPP_ROLE_BROADCASTER,
	.pDevName = (char*)"BleNvmStress",
	.VendorId = ISYST_BLUETOOTH_ID,
	.pAdvManData = (uint8_t*)&s_AdvCounter,
	.AdvManDataLen = sizeof(s_AdvCounter),
	.AdvInterval = 50,
	.AdvTimeout = 0,
};

static const TimerCfg_t s_AdvTimerCfg = {
	.DevNo = 2,				// nRF52832 RTC2, free of SoftDevice RTC0/RTC1
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = 6,
};

static void AdvHeartbeatUpdate(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_AdvCounter++;
	if (!BtAppAdvManDataSet((uint8_t*)&s_AdvCounter, sizeof(s_AdvCounter), NULL, 0))
	{
		printf("Advertising heartbeat update failed\r\n");
	}
	else
	{
		printf("Advertising heartbeat %lu, verified %lu\r\n",
			(unsigned long)s_AdvCounter,
			(unsigned long)s_Verified);
	}
}

// The timer callback runs in an IRQ. Reconfigure advertising in the app queue.
static void AdvTimerHandler(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	(void)AppEvtHandlerQue(0, NULL, AdvHeartbeatUpdate);
}

static int MemIntrfEvtCB(DevIntrf_t * const, DEVINTRF_EVT Evt, uint8_t *, int)
{
	s_Nvm.IntrfEvent(Evt);
	return 0;
}

// Keep dispatching BLE events while the controller schedules flash access.
static bool WaitForNvm()
{
	unsigned retries = 10000;

	do
	{
		AppEvtHandlerExec();
		BtAppCheckStatus();
		if (!s_Nvm.IsBusy())
		{
			return s_Nvm.Sync() == 0;
		}
		msDelay(1);
	} while (--retries);

	printf("NVM completion timed out\r\n");
	return false;
}

int main()
{
	NvmCfg_t cfg = {};
	NvmMcuIntrfStat_t before;
	NvmMcuIntrfStat_t after;
	const unsigned records = 64;
	uint32_t data[4];
	uint32_t readback[4];
	uint32_t unit;
	uint32_t offset;
	uint32_t seq;
	unsigned round;
	unsigned slot;
	unsigned word;
	int count;

	AppEvtHandlerInit(s_EvtQueue, sizeof(s_EvtQueue));
	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
	setvbuf(stdout, NULL, _IONBF, 0);
	if (!NvmMcuCfgInit(cfg, 1))
	{
		printf("Missing reserved NVM1 test region\r\n");
		return 1;
	}
	unit = cfg.EraseSize ? cfg.EraseSize : records * sizeof(data);
	if (unit < records * sizeof(data) || cfg.TotalSize < unit)
	{
		printf("NVM1 test region too small\r\n");
		return 1;
	}
	cfg.bIntEn = true;
	if (!s_MemIntrf.Init(MemIntrfEvtCB, true))
	{
		return 1;
	}
	if (!s_Nvm.Init(cfg, &s_MemIntrf, 0, unit))
	{
		return 1;
	}
	if (!BtAppInit(&s_BtAppCfg))
	{
		printf("BLE init failed\r\n");
		return 1;
	}
	// A heartbeat setup failure must not abort the already-running BLE/NVM
	// stress test before AppRun can service Bluetooth events.
	if (!s_AdvTimer.Init(s_AdvTimerCfg))
	{
		printf("WARNING: advertising heartbeat timer init failed\r\n");
	}
	else if (s_AdvTimer.EnableTimerTrigger(0, (uint32_t)1000,
		TIMER_TRIG_TYPE_CONTINUOUS, AdvTimerHandler) == 0)
	{
		printf("WARNING: advertising heartbeat trigger failed\r\n");
	}
	else
	{
		s_HeartbeatReady = true;
	}

	printf("Advertising BleNvmStress, 50 ms interval. Testing NVM1 at 0x%08lX\r\n",
		   (unsigned long)cfg.BaseAddr);
	NvmMcuIntrfGetStat(&before);
	for (round = 0; round < 8; round++)
	{
		if (cfg.EraseSize)
		{
			if (s_Nvm.Erase(0, unit) != 0 || !WaitForNvm())
			{
				printf("Round %u erase failed\r\n", round + 1);
				return 1;
			}
		}
		for (slot = 0; slot < records; slot++)
		{
			offset = slot * sizeof(data);
			seq = round * records + slot;
			for (word = 0; word < 4; word++)
			{
				data[word] = (seq + 1) * (0x9E3779B9U + word);
			}
			count = s_Nvm.Write(offset, data, sizeof(data));
			if (count != sizeof(data) || !WaitForNvm())
			{
				printf("Round %u record %u write failed\r\n", round + 1, slot);
				return 1;
			}
		}
		// Verify the entire batch after the last write to catch overwritten records.
		for (slot = 0; slot < records; slot++)
		{
			offset = slot * sizeof(data);
			seq = round * records + slot;
			count = s_Nvm.Read(offset, readback, sizeof(readback));
			for (word = 0; word < 4; word++)
			{
				if (count != sizeof(readback) || readback[word] != (seq + 1) * (0x9E3779B9U + word))
				{
					printf("Round %u record %u readback failed\r\n", round + 1, slot);
					return 1;
				}
			}
		}
		s_Verified += records;
		printf("Round %u verified, records %lu, heartbeat %lu\r\n",
			   round + 1, (unsigned long)s_Verified,
			   (unsigned long)s_AdvCounter);
	}
	NvmMcuIntrfGetStat(&after);
	printf("Flash operations via SoftDevice %lu, timeslot %lu, direct %lu\r\n",
		   (unsigned long)(after.Sd - before.Sd), (unsigned long)(after.Slot - before.Slot),
		   (unsigned long)(after.Direct - before.Direct));
#if defined(NRF52832_XXAA)
	if (after.Direct != before.Direct || (after.Slot == before.Slot && after.Sd == before.Sd))
	{
		printf("FAIL: radio did not arbitrate all flash access\r\n");
		return 1;
	}
#endif
	printf("PASS: 512 NVM records verified during advertising\r\n");
	if (!s_HeartbeatReady)
	{
		printf("WARNING: on-air advertising heartbeat was not enabled\r\n");
	}
	AppRun();
	return 0;
}
