/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * LTE application recovery with the real CoAP client and simulated modem/time.
 * No modem registers, cellular network or radio behavior are exercised here.
 */
#include <assert.h>
#include <stdio.h>
#include <string>
#include <vector>
#define main LteCoapDemoMain
#include "../../exemples/lte/lte_coap.cpp"
#undef main

class DemoRng : public RngEngine
{
public:
	bool Enable() override
	{
		return true;
	}
	void Disable() override
	{
	}
	void Reset() override
	{
	}
	CRYPTO_STATUS Random(uint8_t *pData, size_t Len) override
	{
		memset(pData, 0, Len);
		return CRYPTO_STATUS_OK;
	}
};

static DemoRng s_TestRng;
static uint32_t s_TestNow;
static uint32_t s_TestInitDuration;
static unsigned s_TestLteInits;
static unsigned s_TestSocketInits;
static unsigned s_TestSocketCloses;
static unsigned s_TestQueueCalls;
static bool s_TestRegistered;
static bool s_TestLteWorks;
static bool s_TestQueueWorks;
static AppEvtHandler_t s_TestQueued;
static std::vector<std::vector<uint8_t>> s_TestPackets;
static std::string s_TestLog;

static uint64_t Ticks(TimerDev_t *)
{
	return s_TestNow;
}

static bool TransferStart(DevIntrf_t *pDev, uint32_t)
{
	return ((SockIntrfDev_t *)pDev->pDevData)->bConnected;
}

static void TransferStop(DevIntrf_t *)
{
}

static int Receive(DevIntrf_t *, uint8_t *, int)
{
	return 0;
}

static int Send(DevIntrf_t *, const uint8_t *pData, int Len)
{
	s_TestPackets.emplace_back(pData, pData + Len);
	return Len;
}

CryptoRngNrf *CryptoRngNrfInstance(void)
{
	return nullptr;
}

extern "C"
{
bool LteInit(const LteCfg_t *)
{
	s_TestLteInits++;
	s_TestNow += s_TestInitDuration;
	return s_TestLteWorks;
}

bool LteRegistered(void)
{
	return s_TestRegistered;
}

bool LteGetInfo(LTE_INFO, char *, int)
{
	return false;
}

bool SockIntrfInit(SockIntrfDev_t *pDev, const SockIntrfCfg_t *pCfg)
{
	DevIntrf_t *intrf = &pDev->DevIntrf;

	assert(pCfg->Proto == SOCKINTRF_PROTO_UDP && pCfg->EvtCB == nullptr);
	s_TestSocketInits++;
	intrf->pDevData = pDev;
	intrf->StartRx = intrf->StartTx = TransferStart;
	intrf->StopRx = intrf->StopTx = TransferStop;
	intrf->RxData = Receive;
	intrf->TxData = Send;
	intrf->EnCnt = 1;
	intrf->MaxRetry = 0;
	atomic_flag_clear(&intrf->bBusy);
	pDev->Hdl = 1;
	pDev->bConnected = true;
	return true;
}

void SockIntrfClose(SockIntrfDev_t *pDev)
{
	if (pDev->Hdl >= 0)
	{
		s_TestSocketCloses++;
	}
	pDev->Hdl = -1;
	pDev->bConnected = false;
}

bool AppEvtHandlerInit(uint8_t *, size_t)
{
	return true;
}

bool AppEvtHandlerQue(uint32_t, void *, AppEvtHandler_t Handler)
{
	s_TestQueueCalls++;
	if (s_TestQueueWorks)
	{
		assert(s_TestQueued == nullptr);
		s_TestQueued = Handler;
	}
	return s_TestQueueWorks;
}

void AppRun(void)
{
}

bool TimerInit(TimerDev_t *pDev, const TimerCfg_t *)
{
	pDev->Freq = 1000;
	pDev->GetTickCount = Ticks;
	return true;
}

bool UARTInit(UARTDev_t *, const UARTCfg_t *)
{
	return true;
}

void UARTSetCtrlLineState(UARTDev_t *, uint32_t)
{
}

int CFifoAvail(hCFifo_t)
{
	return 0;
}

SysLog_t *SysLogGet(void)
{
	return nullptr;
}

bool SysLogInit(SysLog_t *, const SysLogCfg_t *, DevIntrf_t *, uint32_t,
	TimerDev_t *, uint32_t)
{
	return true;
}

int SysLogPrintf(SysLog_t *, const char *pFormat, ...)
{
	char line[256];
	va_list args;
	int length;

	va_start(args, pFormat);
	length = vsnprintf(line, sizeof(line), pFormat, args);
	va_end(args);
	assert(length >= 0 && (size_t)length < sizeof(line));
	s_TestLog += line;
	return length;
}
}

static void Reset(uint32_t Now)
{
	s_Coap.Reset();
	s_Socket.Close();
	s_TestNow = Now;
	s_TestInitDuration = 0;
	s_TestLteInits = s_TestSocketInits = s_TestSocketCloses = s_TestQueueCalls = 0;
	s_TestRegistered = s_TestLteWorks = s_TestQueueWorks = true;
	s_TestQueued = nullptr;
	s_TestPackets.clear();
	s_TestLog.clear();
	s_Started = s_Waiting = s_RecoveryWaiting = s_ReportQueued = false;
	s_LteStarted = true;
	s_ProbeDelay = s_ProbeStarted = s_Retry = s_LteRetry = 0;
	s_Checked = s_Failures = s_Cycle = 0;
	s_Step = s_PayloadLen = 0;
	s_LastRequest = Now - COAP_DEMO_INTERVAL;
	s_pRng = &s_TestRng;
	TimerInit(&s_Timer, &s_TimerCfg);
}

static void Pump(uint32_t Now)
{
	s_TestNow = Now;
	Report(0, nullptr);
}

static uint32_t SendUnanswered(uint32_t Now)
{
	Reset(Now);
	// A normal large binary step makes the default one-byte/second wait
	// longer than modem restart and CoAP response deadlines.
	s_Step = 2;
	s_PayloadLen = sizeof(s_Payload);
	memset(s_Payload, 0xa5, s_PayloadLen);
	Pump(Now);
	Pump(Now);
	assert(s_Waiting && s_TestPackets.size() == 1);
	assert(((s_TestPackets[0][0] >> 4) & 3U) == COAP_NON);
	assert(s_Coap.RetryAfter(Now) == s_TestPackets[0].size() * 1000U);
	return s_Coap.RetryAfter(Now);
}

static void RegistrationLoss(uint32_t Now)
{
	uint32_t debt = SendUnanswered(Now);
	LteStatus_t status = {};
	unsigned count;

	s_TestNow = Now + 100U;
	s_TestRegistered = false;
	status.RegStat = LTE_REG_SEARCHING;
	LteEvent(LTE_EVT_UNREGISTERED, &status);
	assert(!s_Started && !s_Socket.Connected() && s_TestSocketCloses == 1);
	assert(s_Coap.RetryAfter(s_TestNow) == 0 && s_ProbeDelay == debt - 100U);
	s_TestNow = Now + 1000U;
	LteEvent(LTE_EVT_MODEM_FAULT, &status);
	assert(s_ProbeDelay == debt - 1000U && !s_LteStarted);
	s_TestNow = Now + 2000U;
	LteEvent(LTE_EVT_MODEM_FAULT, &status);
	assert(s_ProbeDelay == debt - 2000U);
	Pump(Now + 31999U);
	assert(s_TestLteInits == 0);
	Pump(Now + 32000U);
	assert(s_TestLteInits == 1 && s_LteStarted);
	s_TestRegistered = true;
	count = s_TestSocketInits;
	Pump(Now + debt - 1U);
	assert(s_TestSocketInits == count && !s_Started);
	Pump(Now + debt);
	assert(s_TestSocketInits == count + 1U && s_Started);
	assert(s_Waiting && s_Step == 0);
}

static void LiveSocketTimeout(void)
{
	const uint32_t now = 10000U;
	uint32_t debt = SendUnanswered(now);

	Pump(now + 94999U);
	assert(s_Waiting && s_Failures == 0);
	Pump(now + 95000U);
	assert(s_Failures == 1 && s_Started && s_Socket.Connected());
	assert(s_TestSocketInits == 1 && s_TestSocketCloses == 0);
	assert(!s_Waiting && !s_Coap.Busy() && s_RecoveryWaiting);
	assert(s_ProbeDelay == debt - 95000U);
	Pump(now + debt - 1U);
	assert(s_TestPackets.size() == 1 && !s_Waiting);
	Pump(now + debt);
	Pump(now + debt);
	assert(s_Waiting && s_TestPackets.size() == 2);
	assert(s_TestSocketInits == 1 && s_TestSocketCloses == 0);
}

static void ModemRetry(void)
{
	const uint32_t start = 0xfffff000U;
	uint32_t finished;

	Reset(start);
	s_TestRegistered = false;
	s_TestLteWorks = false;
	s_TestInitDuration = 700U;
	StartLte();
	finished = s_TestNow;
	assert(!s_LteStarted && s_TestLteInits == 1);
	Pump(finished + LTE_COAP_RESTART_MS - 1U);
	assert(s_TestLteInits == 1);
	Pump(finished + LTE_COAP_RESTART_MS);
	assert(s_TestLteInits == 2 && !s_LteStarted);
	finished = s_TestNow;
	s_TestLteWorks = true;
	Pump(finished + LTE_COAP_RESTART_MS - 1U);
	assert(s_TestLteInits == 2);
	Pump(finished + LTE_COAP_RESTART_MS);
	assert(s_TestLteInits == 3 && s_LteStarted);
	Pump(s_TestNow + 20U);
	assert(s_TestSocketInits == 0);
}

static void QueueRecovery(void)
{
	AppEvtHandler_t queued;

	Reset(10000U);
	s_TestQueueWorks = false;
	ReportTimer(&s_Timer, LTE_COAP_TRIG_REPORT, nullptr);
	assert(!s_ReportQueued && s_TestQueueCalls == 1 && s_TestSocketInits == 0);
	s_TestQueueWorks = true;
	ReportTimer(&s_Timer, LTE_COAP_TRIG_REPORT, nullptr);
	assert(s_ReportQueued && s_TestQueued && s_TestSocketInits == 0);
	ReportTimer(&s_Timer, LTE_COAP_TRIG_REPORT, nullptr);
	assert(s_TestQueueCalls == 2);
	queued = s_TestQueued;
	s_TestQueued = nullptr;
	queued(0, nullptr);
	assert(!s_ReportQueued && s_TestSocketInits == 1);
}

int main()
{
	RegistrationLoss(10000U);
	RegistrationLoss(0xfffff000U);
	LiveSocketTimeout();
	ModemRetry();
	QueueRecovery();
	if (s_SockCfg.pHost && !s_SockCfg.pHost[0])
	{
		Reset(0);
		assert(LteCoapDemoMain() == 0);
		assert(s_TestLteInits == 0 && s_TestSocketInits == 0);
		assert(s_TestLog.find("Set COAP_DEMO_HOST") != std::string::npos);
	}
	puts("LTE CoAP application: recovery, pacing, clock wrap and deferred pump PASS");
}
