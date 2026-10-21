/**-------------------------------------------------------------------------
@file	echo_demo_test.cpp

@brief	Host checks for the optional Ethernet echo integrity example.

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
#include <assert.h>
#include <algorithm>
#include <deque>
#include <string>
#include <vector>
#define main EthernetDemoMain
#include "../../exemples/net/ethernet_demo.cpp"
#undef main

static std::deque<uint8_t> s_Reply;
static std::string s_Output;
static int s_SendLimit = 4096;
static int s_ReadLimit = 4097;
static bool s_Corrupt;
static bool s_Drop;
static bool s_Extra;
static bool s_Refuse;
static bool s_ShortUdp;

extern "C"
{
void DeviceIntrfEnable(DevIntrf_t *)
{
}
void DeviceIntrfDisable(DevIntrf_t *)
{
}
int DeviceIntrfRead(DevIntrf_t *, uint32_t, const uint8_t *, int, uint8_t *, int)
{
	return 0;
}
int DeviceIntrfWrite(DevIntrf_t *, uint32_t, const uint8_t *, int, const uint8_t *, int)
{
	return 0;
}
int CFifoAvail(hCFifo_t)
{
	return 0;
}
bool UARTInit(UARTDev_t *, const UARTCfg_t *)
{
	return false;
}
void UARTSetCtrlLineState(UARTDev_t *, uint32_t)
{
}
void EthernetClose(EthernetDev_t *)
{
}
void SockIntrfClose(SockIntrfDev_t *pDev)
{
	pDev->bConnected = false;
}
SysLog_t *SysLogGet(void)
{
	return nullptr;
}
int SysLogPrintf(SysLog_t *, const char *pFormat, ...)
{
	char text[256];
	va_list args;
	va_start(args, pFormat);
	vsnprintf(text, sizeof(text), pFormat, args);
	va_end(args);
	s_Output += text;
	return (int)strlen(text);
}
int DeviceIntrfTx(DevIntrf_t *, uint32_t, const uint8_t *pData, int Len)
{
	if (s_Refuse)
	{
		return 0;
	}
	int count = std::min(Len, s_SendLimit);
	if (s_Drop == false)
	{
		for (int i = 0; i < count; i++)
		{
			s_Reply.push_back(pData[i] ^ (s_Corrupt ? 1 : 0));
		}
		if (s_Extra)
		{
			s_Reply.push_back(0);
		}
		if (s_ShortUdp && s_Reply.size() > 1)
		{
			s_Reply.pop_back();
		}
	}
	return count;
}
int DeviceIntrfRx(DevIntrf_t *, uint32_t, uint8_t *pData, int Len)
{
	int count = std::min({Len, s_ReadLimit, (int)s_Reply.size()});
	for (int i = 0; i < count; i++)
	{
		pData[i] = s_Reply.front();
		s_Reply.pop_front();
	}
	return count;
}
}

static void Reset(void)
{
	s_TestPassed = s_TestBytes = s_TestStart = s_TestSent = 0;
	s_TestLength = s_TestTxOffset = s_TestRxOffset = 0;
	s_TestStarted = s_TestDone = false;
	s_Reply.clear();
	s_Output.clear();
	s_Corrupt = s_Drop = s_Extra = s_Refuse = s_ShortUdp = false;
	s_SendLimit = 4096;
	s_ReadLimit = 4097;
}

int main()
{
	Reset();
	if (s_SockCfg.Proto == SOCKINTRF_PROTO_TCP)
	{
		s_SendLimit = 71;
		s_ReadLimit = 13;
	}
	for (uint32_t now = 0; now < 2000000 && s_TestDone == false; now += 10)
	{
		EchoTest(now);
	}
	assert(s_TestPassed == NET_DEMO_TEST_COUNT);
	assert(s_Output.find("Echo PASS:") != std::string::npos);
	assert(s_Output.find("FAIL") == std::string::npos);
	assert(s_TestBytes > 1472);
	// A stopped test must not restart or send again.
	EchoTest(2000000);
	assert(s_TestPassed == NET_DEMO_TEST_COUNT && s_Reply.empty());

	Reset();
	s_Corrupt = true;
	EchoTest(0);
	assert(s_TestDone && s_Output.find("data mismatch") != std::string::npos);

	Reset();
	s_Extra = true;
	EchoTest(0);
	assert(s_TestDone && s_Output.find("receive length") != std::string::npos);

	Reset();
	s_Drop = true;
	EchoTest(0);
	EchoTest(4999);
	assert(s_TestDone == false);
	EchoTest(5000);
	assert(s_TestDone && s_Output.find("timeout") != std::string::npos);

	Reset();
	s_Refuse = true;
	EchoTest(0xfffffff0U);
	EchoTest(0xfffffff0U + 5000U);
	assert(s_TestDone && s_Output.find("timeout") != std::string::npos);

	Reset();
	s_Refuse = true;
	EchoTest(0);
	s_Refuse = false;
	EchoTest(10);
	assert(s_TestPassed == 1 && s_TestDone == false);

	if (s_SockCfg.Proto == SOCKINTRF_PROTO_UDP)
	{
		Reset();
		s_TestPassed = 1; // Exercise a 15-byte datagram.
		s_ShortUdp = true;
		EchoTest(0);
		assert(s_TestDone && s_Output.find("receive length") != std::string::npos);

		Reset();
		s_TestPassed = 1;
		s_SendLimit = 7;
		EchoTest(0);
		assert(s_TestDone && s_Output.find("send length") != std::string::npos);
	}
	puts("Echo example integrity, partial transfers and fault checks: PASS");
}
