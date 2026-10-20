/**-------------------------------------------------------------------------
@file	ethernet_fw.cpp

@brief	Production Ethernet driver register-model firmware.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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
#include <stdint.h>
#include <string.h>
#include "stm32h7xx.h"
#include "coredev/ethernet.h"

extern "C" uint32_t SystemD2Clock;
uint32_t SystemD2Clock = 200000000;
extern "C" uint32_t SystemMicroSecLoopCnt;
uint32_t SystemMicroSecLoopCnt = 1;
extern "C" uint32_t SystemnsDelayFactor;
uint32_t SystemnsDelayFactor = 1;

static EthernetDev_t s_Eth;
static EthernetDev_t s_Other;
static uint8_t s_Data[1514];
static uint8_t s_Rx[1514];
static int s_Events;
static const IOPinCfg_t s_Pins[9] = {};

extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}

extern "C" void IOPinDisable(int, int)
{
}

static int Event(DevIntrf_t *, DEVINTRF_EVT, uint8_t *, int)
{
	s_Events++;

	return 0;
}

static const EthernetCfg_t s_Cfg = {0,s_Pins,9,{2,0,0,0,0,1},0,6,true,Event};

extern "C" const uintptr_t g_EthRegs[14];
const uintptr_t g_EthRegs[14] = {
	(uintptr_t)&ETH->DMAMR, (uintptr_t)&ETH->MACMDIOAR, (uintptr_t)&ETH->MACMDIODR,
	(uintptr_t)&ETH->DMACSR, (uintptr_t)&ETH->MTLTQOMR, (uintptr_t)&ETH->DMACTDLAR,
	(uintptr_t)&ETH->DMACRDLAR, (uintptr_t)&ETH->DMACCR, (uintptr_t)&ETH->DMACRCR,
	(uintptr_t)&ETH->MACCR, (uintptr_t)&ETH->DMACTDTPR, (uintptr_t)&ETH->DMACRDTPR,
	(uintptr_t)&SCB->DCCMVAC, (uintptr_t)&SCB->DCIMVAC
};

#define CHECK(X) \
	do \
	{ \
		if (!(X)) \
		{ \
			return __LINE__; \
		} \
	} while (0)
extern "C" void ETH_IRQHandler(void);
extern "C" int RunEthernetTest(int Test);

int RunEthernetTest(int Test)
{
	DevIntrf_t *eth = &s_Eth.DevIntrf;

	switch (Test)
	{
		case 0:
			CHECK(EthernetInit(&s_Eth, &s_Cfg));
			CHECK(DeviceIntrfGetRate(eth) == 0);
			break;
		case 1:
			memset(s_Data, 0xA5, sizeof(s_Data));
			for (int i = 0; i < 4; i++)
			{
				CHECK(DeviceIntrfTx(eth, 0, s_Data, 64) == 64);
			}
			CHECK(DeviceIntrfTx(eth, 0, s_Data, 64) == 0);
			CHECK(DeviceIntrfStartTx(eth, 0));
			DeviceIntrfStopTx(eth);
			break;
		case 2:
			CHECK(DeviceIntrfTx(eth, 0, s_Data, 1514) == 1514);
			CHECK(DeviceIntrfTx(eth, 0, s_Data, 1515) == 0);
			break;
		case 3:
			CHECK(DeviceIntrfRx(eth, 0, s_Rx, sizeof(s_Rx)) == 64);
			for (int i = 0; i < 64; i++)
			{
				CHECK(s_Rx[i] == (uint8_t)(i ^ 0x5A));
			}
			break;
		case 5:
			CHECK(DeviceIntrfRx(eth, 0, s_Rx, 4) == 0);
			CHECK(DeviceIntrfRx(eth, 0, s_Rx, sizeof(s_Rx)) == 0);
			break;
		case 6:
			DeviceIntrfDisable(eth);
			CHECK(DeviceIntrfGetRate(eth) == 0);
			CHECK(DeviceIntrfTx(eth, 0, s_Data, 64) == 0);
			DeviceIntrfEnable(eth);
			CHECK(DeviceIntrfGetRate(eth) == 100000000);
			break;
		case 7:
			EthernetClose(&s_Eth);
			CHECK(EthernetInit(&s_Other, &s_Cfg));
			EthernetClose(&s_Other);
			CHECK(EthernetInit(&s_Eth, &s_Cfg));
			break;
		case 8:
			CHECK(!EthernetInit(&s_Other, &s_Cfg));
			CHECK(eth->GetHandle(eth) == &s_Eth);
			break;
		case 9:
		{
			EthernetCfg_t cfg = s_Cfg;

			cfg.PhyAddr = 32;
			CHECK(!EthernetInit(&s_Eth, &cfg));
			cfg = s_Cfg;
			cfg.MacAddr[0] = 1;
			CHECK(!EthernetInit(&s_Eth, &cfg));
			cfg = s_Cfg;
			cfg.IntPrio = 16;
			CHECK(!EthernetInit(&s_Eth, &cfg));
			break;
		}
		case 10:
			EthernetPollLink(&s_Eth);
			CHECK(DeviceIntrfGetRate(eth) == 100000000);
			break;
		case 11:
		{
			int events = s_Events;

			ETH_IRQHandler();
			CHECK(s_Events == events + 2);
			break;
		}
		case 12:
			EthernetPollLink(&s_Eth);
			CHECK(DeviceIntrfGetRate(eth) == 0);
			break;
		case 13:
			EthernetPollLink(&s_Eth);
			CHECK(DeviceIntrfGetRate(eth) == 10000000);
			break;
		case 14:
			eth->EvtCB = nullptr;
			DeviceIntrfReset(eth);
			CHECK(eth->GetHandle(eth) == &s_Eth);
			CHECK(eth->EvtCB == nullptr);
			break;
		case 100:
			CHECK(!EthernetInit(&s_Eth, &s_Cfg));
			break;
		default:
			return -1;
	}

	return 0;
}
