/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Real example setup with simulated network, UART, timer and event queue.
 * Compile for each NET_DEMO_TEST_MEDIUM; no hardware or radio is exercised.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include "../../exemples/net/net_demo.cpp"

class TestRng : public RngEngine
{
public:
	bool Works = false;

	bool Enable() override
	{
		return Works;
	}
	void Disable() override
	{
	}
	void Reset() override
	{
	}
	CRYPTO_STATUS Random(uint8_t *pData, size_t Len) override
	{
		memset(pData, 7, Len);
		return CRYPTO_STATUS_OK;
	}
};

#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
class TestInterface : public DeviceIntrf
{
public:
	DevIntrf_t Data = {};

	operator DevIntrf_t *() override
	{
		return &Data;
	}
	uint32_t Rate(uint32_t Value) override
	{
		return Value;
	}
	uint32_t Rate() override
	{
		return 0;
	}
};

static TestInterface s_TestInterface;
#else
static SockIntrfDev_t *s_TestSocket;
#endif
static TestRng s_TestRng;
static uint32_t s_TestNow = 0xfffff000U;
static uint32_t s_TestInitDuration = 700U;
static uint32_t s_TestOpenDuration = 900U;
static uint32_t s_TestCloseDuration = 500U;
static uint32_t s_TestHandlerTime;
static unsigned s_TestInits;
static unsigned s_TestOpens;
static unsigned s_TestCloses;
static unsigned s_TestQueues;
static unsigned s_TestPumps;
#if !NET_DEMO_MODEM_TLS && !NET_DEMO_LTE_RECOVERY_TEST
static unsigned s_TestProcesses;
#endif
static unsigned s_TestWrites;
static bool s_TestInitWorks;
static bool s_TestOpenWorks;
static bool s_TestReady;
static bool s_TestConnected;
static bool s_TestResource;
static bool s_TestQueueWorks;
static bool s_TestTriggerWorks;
static bool s_TestInterrupt;
static bool s_TestDatagram;
static bool s_TestHandlerReady;
static bool s_TestHandlerConnected;
static AppEvtHandler_t s_TestQueued;
static TimerTrigEvtHandler_t s_TestTick;
static std::string s_TestLog;
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
static LteEvtHandler_t s_TestLteEvent;
#if NET_DEMO_MODEM_TLS || NET_DEMO_LTE_RECOVERY_TEST
static bool s_TestRadio;
static bool s_TestDisconnectWorks;
static bool s_TestConnectWorks;
static unsigned s_TestDisconnects;
static unsigned s_TestConnects;
#endif
#if NET_DEMO_LTE_RECOVERY_TEST
static bool s_TestRecoveryActive;
static uint32_t s_TestDisconnectFinished;
static uint32_t s_TestConnectFinished;
#endif
#if NET_DEMO_MODEM_TLS
static bool s_TestCredentialStored;
static bool s_TestCredentialWriteWorks;
static bool s_TestCredentialCheckWorks;
static bool s_TestCredentialChecked;
static unsigned s_TestCredentialQueries;
static unsigned s_TestCredentialWrites;
#endif
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
static bool s_TestIpWorks;
static unsigned s_TestEthernetCloses;
#endif

RngEngine *NetDemoTestRng()
{
	return &s_TestRng;
}

static uint64_t Ticks(TimerDev_t *)
{
	return s_TestNow;
}

static uint64_t Trigger(TimerDev_t *, int TrigNo, uint64_t Period,
	TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t Handler, void *)
{
	assert(!s_TestInterrupt && TrigNo == 1 && Period == 20000000U);
	assert(Type == TIMER_TRIG_TYPE_CONTINUOUS);
	if (!s_TestTriggerWorks)
	{
		return 0;
	}
	s_TestTick = Handler;
	return Period;
}

static bool TransferStart(DevIntrf_t *, uint32_t)
{
	return s_TestConnected;
}

static void TransferStop(DevIntrf_t *)
{
}

static int Send(DevIntrf_t *, const uint8_t *pData, int Len)
{
	assert(!s_TestInterrupt && s_TestResource && s_TestConnected);
	assert(Len == 4 && memcmp(pData, "test", 4) == 0);
	s_TestWrites++;
	return Len;
}

static void PrepareInterface(DevIntrf_t *pDev)
{
	pDev->StartRx = pDev->StartTx = TransferStart;
	pDev->StopRx = pDev->StopTx = TransferStop;
	pDev->TxData = Send;
	pDev->MaxRetry = 99;
	pDev->EnCnt = 1;
	atomic_flag_clear(&pDev->bBusy);
}

static bool Open(bool Datagram, const char *pHost, uint16_t Port)
{
	assert(!s_TestInterrupt && s_TestReady && !s_TestResource);
	assert(strcmp(pHost, "test.example") == 0 && Port == 1234);
	s_TestDatagram = Datagram;
	s_TestOpens++;
	s_TestNow += s_TestOpenDuration;
	// Failed initialization may still need to release a partially opened handle.
	s_TestResource = true;
	s_TestConnected = s_TestOpenWorks;
	return s_TestOpenWorks;
}

static void Close()
{
	assert(!s_TestInterrupt);
	if (s_TestResource)
	{
		s_TestCloses++;
		s_TestNow += s_TestCloseDuration;
	}
	s_TestResource = s_TestConnected = false;
}

extern "C"
{
bool AppEvtHandlerInit(uint8_t *pMem, size_t Len)
{
	assert(pMem && Len >= APPEVT_HANDLER_QUE_MEMSIZE(16));
	return true;
}

bool AppEvtHandlerQue(uint32_t, void *, AppEvtHandler_t Handler)
{
	s_TestQueues++;
	if (!s_TestQueueWorks)
	{
		return false;
	}
	assert(!s_TestQueued);
	s_TestQueued = Handler;
	return true;
}

bool TimerInit(TimerDev_t *pTimer, const TimerCfg_t *)
{
	pTimer->Freq = 1000;
	pTimer->GetTickCount = Ticks;
	pTimer->EnableTrigger = Trigger;
	return true;
}

bool UARTInit(UARTDev_t *, const UARTCfg_t *pCfg)
{
	assert(pCfg->TxMemSize == CFIFO_MEMSIZE(1024) && pCfg->pTxMem);
	assert(pCfg->DataBits == 8 && pCfg->StopBits == 1 && pCfg->bIntMode);
	return true;
}

void UARTSetCtrlLineState(UARTDev_t *, uint32_t)
{
}

int CFifoAvail(hCFifo_t)
{
	return 0;
}

SysLog_t *SysLogGet()
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

	assert(!s_TestInterrupt);
	va_start(args, pFormat);
	length = vsnprintf(line, sizeof(line), pFormat, args);
	va_end(args);
	assert(length >= 0 && (size_t)length < sizeof(line));
	s_TestLog += line;
	return length;
}

#if NET_DEMO_MEDIUM != NET_DEMO_MEDIUM_CUSTOM
bool SockIntrfInit(SockIntrfDev_t *pDev, const SockIntrfCfg_t *pCfg)
{
	assert(pCfg->LocalPort == 0 && !pCfg->EvtCB);
#if NET_DEMO_MODEM_TLS
	assert(pCfg->Proto == SOCKINTRF_PROTO_TLS);
	assert(pCfg->SecTag == NET_DEMO_TLS_SEC_TAG && pCfg->bPeerVerify);
	assert(s_TestRadio && s_TestCredentialStored && s_TestCredentialChecked);
#else
	assert(pCfg->Proto == SOCKINTRF_PROTO_TCP || pCfg->Proto == SOCKINTRF_PROTO_UDP);
	assert(pCfg->SecTag == -1 && !pCfg->bPeerVerify);
#endif
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
	assert(pCfg->Timeout == 100U);
#else
	assert(pCfg->Timeout == 10000U);
#endif
	s_TestSocket = pDev;
	PrepareInterface(&pDev->DevIntrf);
	pDev->Hdl = 5;
	pDev->bConnected = Open(pCfg->Proto == SOCKINTRF_PROTO_UDP, pCfg->pHost, pCfg->Port);
	return pDev->bConnected;
}

void SockIntrfClose(SockIntrfDev_t *pDev)
{
	Close();
	pDev->Hdl = -1;
	pDev->bConnected = false;
}
#endif

#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
bool LteInit(const LteCfg_t *pCfg)
{
	assert(!s_TestInterrupt && pCfg->pTimer && pCfg->TimerTrigNo == 0);
	assert(!pCfg->bPsm && !pCfg->bRai && !pCfg->EdrxCycle);
	s_TestLteEvent = pCfg->EvtHandler;
	s_TestInits++;
	s_TestNow += s_TestInitDuration;
#if NET_DEMO_MODEM_TLS || NET_DEMO_LTE_RECOVERY_TEST
	s_TestRadio = s_TestInitWorks;
#endif
#if NET_DEMO_MODEM_TLS
	// A registered status can be stale while the radio-off URC is queued.
	// Readiness must remain gated until secure preparation finishes.
	s_TestReady = true;
	s_TestCredentialChecked = false;
#endif
	return s_TestInitWorks;
}

#if NET_DEMO_MODEM_TLS || NET_DEMO_LTE_RECOVERY_TEST
bool LteDisconnect()
{
	assert(!s_TestInterrupt && s_TestRadio);
#if NET_DEMO_LTE_RECOVERY_TEST
	if (s_TestRecoveryActive)
	{
		// Interrupt the active transport; the ordinary pump must observe and
		// close it only after the radio-off command has succeeded.
		assert(s_TestResource && NetDemoReady() && NetDemoConnected());
	}
	else
#endif
	{
		assert(!s_TestResource);
	}
	s_TestDisconnects++;
	s_TestNow += 150U;
#if NET_DEMO_LTE_RECOVERY_TEST
	s_TestDisconnectFinished = s_TestNow;
#endif
	if (!s_TestDisconnectWorks)
	{
		return false;
	}
	s_TestRadio = false;
	return true;
}

bool LteConnect()
{
	assert(!s_TestInterrupt && !s_TestResource && !s_TestRadio && !NetDemoReady());
#if NET_DEMO_MODEM_TLS
	assert(s_TestCredentialStored && s_TestCredentialChecked);
#endif
	s_TestConnects++;
	s_TestNow += 250U;
#if NET_DEMO_LTE_RECOVERY_TEST
	s_TestConnectFinished = s_TestNow;
#endif
	s_TestRadio = s_TestConnectWorks;
	return s_TestConnectWorks;
}
#endif

#if NET_DEMO_MODEM_TLS
bool SockIntrfCredExists(int Tag, SOCKINTRF_CRED Type)
{
	assert(!s_TestInterrupt && !s_TestRadio && !NetDemoReady());
	assert(Tag == NET_DEMO_TLS_SEC_TAG && Type == SOCKINTRF_CRED_CA_CERT);
	s_TestCredentialQueries++;
	s_TestNow += 20U;
	s_TestCredentialChecked = s_TestCredentialStored && s_TestCredentialCheckWorks;
	return s_TestCredentialChecked;
}

bool SockIntrfCredWrite(int Tag, SOCKINTRF_CRED Type, const char *pData)
{
	const char *expected = NET_DEMO_TLS_CA_CERT;

	assert(!s_TestInterrupt && !s_TestRadio && !NetDemoReady());
	assert(Tag == NET_DEMO_TLS_SEC_TAG && Type == SOCKINTRF_CRED_CA_CERT);
	assert(expected && pData && strcmp(pData, expected) == 0);
	s_TestCredentialWrites++;
	s_TestNow += 30U;
	if (s_TestCredentialWriteWorks)
	{
		s_TestCredentialStored = true;
	}
	s_TestCredentialChecked = false;
	return s_TestCredentialWriteWorks;
}

bool SockIntrfCredDelete(int, SOCKINTRF_CRED)
{
	assert(false && "Demo must not delete credentials");
	return false;
}
#endif

bool LteRegistered()
{
	return s_TestReady;
}

bool LteGetInfo(LTE_INFO, char *, int)
{
	return false;
}
#elif NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
bool EthernetInit(EthernetDev_t *, const EthernetCfg_t *)
{
	assert(!s_TestInterrupt);
	s_TestInits++;
	s_TestNow += s_TestInitDuration;
	return s_TestInitWorks;
}

void EthernetClose(EthernetDev_t *)
{
	assert(!s_TestInterrupt);
	s_TestEthernetCloses++;
}

bool IpInit(const IpCfg_t *pCfg, EthernetDev_t *, TimerDev_t *pTimer, int TriggerNo)
{
	assert(!s_TestInterrupt && pCfg->bDhcp && pTimer && TriggerNo == 0);
	return s_TestIpWorks;
}

bool IpReady()
{
	return s_TestReady;
}

bool IpAddress(uint8_t Addr[4])
{
	static const uint8_t addr[] = {192, 0, 2, 20};

	memcpy(Addr, addr, sizeof(addr));
	return s_TestReady;
}

void IpProcess()
{
	assert(!s_TestInterrupt);
	s_TestProcesses++;
}
#endif
}

#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
bool NetDemoCustomInit(TimerDev_t *pTimer)
{
	assert(!s_TestInterrupt && pTimer);
	s_TestInits++;
	s_TestNow += s_TestInitDuration;
	return s_TestInitWorks;
}

void NetDemoCustomProcess(uint32_t Now)
{
	assert(!s_TestInterrupt && Now == s_TestNow);
	s_TestProcesses++;
	// The application's time must also be refreshed after medium service.
	s_TestNow += 17U;
}

bool NetDemoCustomReady()
{
	return s_TestReady;
}

DeviceIntrf *NetDemoCustomOpen(bool Datagram, const char *pHost, uint16_t Port)
{
	PrepareInterface(&s_TestInterface.Data);
	return Open(Datagram, pHost, Port) ? &s_TestInterface : nullptr;
}

bool NetDemoCustomConnected()
{
	return s_TestConnected;
}

void NetDemoCustomClose()
{
	Close();
}
#endif

static void Application(uint32_t Now)
{
	assert(!s_TestInterrupt && Now == s_TestNow);
	s_TestPumps++;
	s_TestHandlerTime = Now;
	s_TestHandlerReady = NetDemoReady();
	s_TestHandlerConnected = NetDemoConnected();
}

static void Tick()
{
	assert(s_TestTick);
	s_TestInterrupt = true;
	s_TestTick(nullptr, 1, nullptr);
	s_TestInterrupt = false;
}

static void Dispatch()
{
	AppEvtHandler_t handler = s_TestQueued;

	assert(handler);
	s_TestQueued = nullptr;
	handler(0, nullptr);
}

static void PumpAt(uint32_t Now)
{
	s_TestNow = Now;
	Tick();
	Dispatch();
}

static DeviceIntrf *OpenEndpoint(bool Datagram)
{
	DeviceIntrf *intrf = NetDemoOpen(Datagram, "test.example", 1234);
	DevIntrf_t *dev;

	assert(intrf && NetDemoConnected() && s_TestDatagram == Datagram);
	dev = *intrf;
	assert(dev->MaxRetry == 0);
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_CUSTOM
	assert(intrf == &s_TestInterface && dev == &s_TestInterface.Data);
#else
	assert(dev == &s_TestSocket->DevIntrf);
#endif
	assert(intrf->Tx(0, (const uint8_t *)"test", 4) == 4);
	return intrf;
}

#if NET_DEMO_LTE_RECOVERY_TEST
static void RecoveryPump(uint32_t Now)
{
	unsigned pumps = s_TestPumps;

	PumpAt(Now);
	assert(s_TestPumps == pumps + 1U && s_TestHandlerTime == NetDemoNow());
}

static void RecoveryBlocked()
{
	unsigned opens = s_TestOpens;

	assert(!NetDemoReady() && !NetDemoConnected());
	assert(!NetDemoOpen(false, "test.example", 1234) && s_TestOpens == opens);
}

static void RecoveryEvent(LTE_EVT Evt)
{
	LteStatus_t status = {};

	status.RegStat = Evt == LTE_EVT_REGISTERED ? LTE_REG_ROAMING : LTE_REG_NONE;
	status.Rat = LTE_RAT_LTEM;
	s_TestLteEvent(Evt, &status);
}

static void RecoveryNoRepeat()
{
	unsigned inits = s_TestInits;
	unsigned disconnects = s_TestDisconnects;
	unsigned connects = s_TestConnects;

	assert(NetDemoReady());
	if (!NetDemoConnected())
	{
		OpenEndpoint(false);
	}
	RecoveryPump(NetDemoNow());
	RecoveryPump(NetDemoNow() + 600000U);
	assert(NetDemoConnected());
	NetDemoClose();
	OpenEndpoint(false);
	RecoveryPump(NetDemoNow());
	RecoveryPump(NetDemoNow() + 600000U);
	assert(NetDemoConnected());
	assert(s_TestInits == inits && s_TestDisconnects == disconnects && s_TestConnects == connects);
	NetDemoClose();
	assert(!s_TestResource);
}

static void RecoveryRestart()
{
	uint32_t finished = NetDemoNow();
	unsigned inits = s_TestInits;
#if NET_DEMO_MODEM_TLS
	unsigned queries = s_TestCredentialQueries;
	unsigned writes = s_TestCredentialWrites;
#endif

	RecoveryBlocked();
	s_TestRecoveryActive = false;
	s_TestDisconnectWorks = s_TestConnectWorks = true;
	RecoveryPump(finished + 29999U);
	assert(s_TestInits == inits);
	RecoveryBlocked();
	RecoveryPump(finished + 30000U);
	assert(s_TestInits == inits + 1U && NetDemoReady() && !NetDemoConnected());
#if NET_DEMO_MODEM_TLS
	assert(s_TestCredentialQueries == queries + 1U && s_TestCredentialWrites == writes);
#endif
	s_TestRecoveryActive = true;
	RecoveryNoRepeat();
}

static int RecoverySetup(const char *pScenario)
{
	bool offWrap = strcmp(pScenario, "off-wrap") == 0;
	bool offFails = strcmp(pScenario, "disconnect-failure") == 0;
	bool onFails = strcmp(pScenario, "connect-failure") == 0;
	bool noOffEvent = strcmp(pScenario, "no-off-event") == 0;
	bool noRegistration = strcmp(pScenario, "no-registration") == 0;
	bool faultOff = strcmp(pScenario, "fault-off") == 0;
	bool faultRetry = strcmp(pScenario, "fault-retry") == 0;
	bool faultRegistration = strcmp(pScenario, "fault-registration") == 0;
	uint32_t firstArmed;
	uint32_t armed;
	uint32_t offFinished;
	uint32_t onFinished;
	unsigned disconnects;
	unsigned connects;
	unsigned closes;
	unsigned pumps;
#if NET_DEMO_MODEM_TLS
	unsigned queries;
	unsigned writes;
#endif

	assert(strcmp(pScenario, "normal") == 0 || offWrap || offFails || onFails ||
		noOffEvent || noRegistration || faultOff || faultRetry || faultRegistration);
	s_TestNow = offWrap ? 0xfff80000U : 0xfffa0000U;
	s_TestRng.Works = s_TestTriggerWorks = s_TestQueueWorks = true;
	s_TestInitWorks = s_TestOpenWorks = s_TestDisconnectWorks = s_TestConnectWorks = true;
#if NET_DEMO_MODEM_TLS
	s_TestCredentialStored = s_TestCredentialWriteWorks = s_TestCredentialCheckWorks = true;
#endif
	assert(NetDemoInit() && NetDemoStart(Application));
	assert(s_TestInits == 1 && s_TestRadio && !NetDemoConnected());
	disconnects = s_TestDisconnects;
	connects = s_TestConnects;
#if NET_DEMO_MODEM_TLS
	queries = s_TestCredentialQueries;
	writes = s_TestCredentialWrites;
	assert(queries == 1 && writes == 0);
#endif
	s_TestRecoveryActive = true;
	// Neither attachment time nor an idle registered modem starts the delay.
	s_TestReady = false;
	RecoveryPump(NetDemoNow() + 120000U);
	s_TestReady = true;
	RecoveryPump(NetDemoNow() + 120000U);
	assert(s_TestDisconnects == disconnects && s_TestConnects == connects);
	OpenEndpoint(false);
	RecoveryPump(NetDemoNow());
	firstArmed = NetDemoNow();
	RecoveryPump(firstArmed + 1000U);
	// Readiness must be continuous. A closed/replaced endpoint starts a new
	// delay even when no timer callback occurred between close and reopen.
	s_TestSocket->bConnected = false;
	RecoveryPump(firstArmed + 1020U);
	s_TestSocket->bConnected = true;
	RecoveryPump(firstArmed + 1040U);
	RecoveryPump(firstArmed + 120000U);
	assert(s_TestDisconnects == disconnects && NetDemoConnected());
	s_TestNow += 1000U;
	NetDemoClose();
	OpenEndpoint(false);
	RecoveryPump(NetDemoNow());
	armed = NetDemoNow();
	assert(s_TestDisconnects == disconnects && NetDemoConnected());
	RecoveryPump(armed + 119999U);
	assert(s_TestDisconnects == disconnects && NetDemoConnected());
	if (!offWrap)
	{
		assert((uint32_t)(armed + 120000U) < armed);
	}
	s_TestDisconnectWorks = !offFails;
	closes = s_TestCloses;
	pumps = s_TestPumps;
	s_TestNow = armed + 120000U;
	Tick();
	assert(s_TestDisconnects == disconnects && s_TestPumps == pumps);
	Dispatch();
	assert(s_TestDisconnects == disconnects + 1U && s_TestPumps == pumps + 1U);
	assert(s_TestHandlerTime == NetDemoNow());
	if (offFails)
	{
		assert(s_TestRadio && NetDemoConnected() && s_TestCloses == closes);
		assert(s_TestHandlerReady && s_TestHandlerConnected && s_TestConnects == connects);
		assert(s_TestLog.find("radio-off failed; test aborted") != std::string::npos);
		RecoveryNoRepeat();
		return 0;
	}
	offFinished = s_TestDisconnectFinished;
	assert(!s_TestRadio && !s_TestResource && s_TestCloses == closes + 1U);
	assert(NetDemoNow() == offFinished + s_TestCloseDuration);
	assert(!s_TestHandlerReady && !s_TestHandlerConnected);
	RecoveryBlocked();
	if (offWrap)
	{
		assert((uint32_t)(offFinished + 60000U) < offFinished);
	}
	if (faultOff)
	{
		RecoveryPump(offFinished + 1000U);
		RecoveryEvent(LTE_EVT_MODEM_FAULT);
		assert(s_TestLog.find("aborted by modem fault") != std::string::npos);
		RecoveryRestart();
		return 0;
	}
	if (!noOffEvent)
	{
		RecoveryEvent(LTE_EVT_UNREGISTERED);
	}
	// Cached registration and even an early registered event cannot undo
	// the deliberate radio-off hold. Every pump still reaches the app.
	assert(s_TestReady);
	RecoveryEvent(LTE_EVT_REGISTERED);
	RecoveryBlocked();
	RecoveryPump(offFinished + 30000U);
	RecoveryBlocked();
	RecoveryPump(offFinished + 59999U);
	assert(s_TestConnects == connects && s_TestInits == 1);
	s_TestConnectWorks = !(onFails || faultRetry);
	RecoveryPump(offFinished + 60000U);
	assert(s_TestConnects == connects + 1U && s_TestInits == 1);
	onFinished = s_TestConnectFinished;
	RecoveryBlocked();
	assert((s_TestLog.find("offline notification missing; loss unverified") != std::string::npos) == noOffEvent);
	if (onFails || faultRetry)
	{
		assert(!s_TestRadio);
		assert(s_TestLog.find("radio-on failed; retry in 30 s") != std::string::npos);
		RecoveryEvent(LTE_EVT_REGISTERED);
		RecoveryBlocked();
		if (faultRetry)
		{
			RecoveryEvent(LTE_EVT_MODEM_FAULT);
			RecoveryRestart();
			return 0;
		}
		RecoveryPump(onFinished + 29999U);
		assert(s_TestConnects == connects + 1U && s_TestInits == 1);
		s_TestConnectWorks = true;
		RecoveryPump(onFinished + 30000U);
		assert(s_TestConnects == connects + 2U && s_TestInits == 1);
		onFinished = s_TestConnectFinished;
	}
	assert(s_TestRadio && !s_TestResource);
	RecoveryPump(onFinished + 1000U);
	RecoveryBlocked();
	if (faultRegistration)
	{
		RecoveryEvent(LTE_EVT_MODEM_FAULT);
		RecoveryRestart();
		return 0;
	}
	if (noRegistration)
	{
		RecoveryPump(onFinished + 179999U);
		RecoveryBlocked();
		assert(s_TestInits == 1);
		RecoveryPump(onFinished + 180000U);
		RecoveryBlocked();
		assert(s_TestInits == 1);
		assert(s_TestLog.find("registration unverified; LTE reinitialization in 30 s") != std::string::npos);
		RecoveryRestart();
		return 0;
	}
	RecoveryEvent(LTE_EVT_REGISTERED);
	assert(NetDemoReady() && !NetDemoConnected());
	RecoveryPump(NetDemoNow());
	assert(s_TestHandlerReady && !s_TestHandlerConnected && s_TestInits == 1);
#if NET_DEMO_MODEM_TLS
	assert(s_TestCredentialQueries == queries && s_TestCredentialWrites == writes);
#endif
	assert(s_TestLog.find("registration restored; waiting for protocol recovery") != std::string::npos);
	RecoveryNoRepeat();
	puts("LTE recovery setup: foreground outage, readiness gating and one-shot recovery PASS");
	return 0;
}
#endif

#if NET_DEMO_MODEM_TLS && !NET_DEMO_LTE_RECOVERY_TEST
static int SecureSetup(const char *pScenario)
{
	const char *ca = NET_DEMO_TLS_CA_CERT;
	bool configured = ca && ca[0];
	bool existing = strcmp(pScenario, "stored") == 0;
	bool fails = strcmp(pScenario, "missing") != 0 && !existing;
	uint32_t finished;
	unsigned writes;
	unsigned queries;
	unsigned connects;
	unsigned opens;

	s_TestRng.Works = s_TestTriggerWorks = s_TestQueueWorks = true;
	s_TestInitWorks = strcmp(pScenario, "init-failure") != 0;
	s_TestDisconnectWorks = strcmp(pScenario, "disconnect-failure") != 0;
	s_TestConnectWorks = strcmp(pScenario, "reconnect-failure") != 0;
	s_TestCredentialStored = existing;
	s_TestCredentialWriteWorks = strcmp(pScenario, "write-failure") != 0;
	s_TestCredentialCheckWorks = strcmp(pScenario, "check-failure") != 0;
	fails = fails || (!configured && (!existing || NET_DEMO_TLS_REPLACE_CA));
	assert(NetDemoInit() && NetDemoStart(Application));
	assert(s_TestInits == 1 && s_TestOpens == 0);
	assert(strcmp(NetDemoSecurityName(), "TLS 1.2, CA/hostname verified") == 0);
	finished = NetDemoNow();
	if (fails)
	{
		assert(!NetDemoReady() && !NetDemoConnected());
		assert(!NetDemoOpen(false, "test.example", 1234) && s_TestOpens == 0);
		if (!s_TestInitWorks)
		{
			assert(s_TestDisconnects == 0 && s_TestCredentialQueries == 0);
		}
		if (!s_TestDisconnectWorks)
		{
			assert(s_TestDisconnects == 1 && s_TestCredentialQueries == 0);
		}
		if (!s_TestInitWorks || !s_TestDisconnectWorks || !s_TestCredentialWriteWorks ||
			!s_TestCredentialCheckWorks || !configured)
		{
			assert(s_TestConnects == 0);
		}
		// Correct the failed prerequisite; retry pacing starts after the
		// blocking attempt and crosses the wrapping millisecond clock.
		s_TestInitWorks = s_TestDisconnectWorks = s_TestConnectWorks = true;
		s_TestCredentialWriteWorks = s_TestCredentialCheckWorks = true;
		if (!configured)
		{
			// Existing-only mode can recover after external CA provisioning.
			s_TestCredentialStored = true;
		}
		PumpAt(finished + 29999U);
		assert(s_TestInits == 1 && !NetDemoReady());
		PumpAt(finished + 30000U);
		assert(s_TestInits == 2);
	}
	assert(NetDemoReady() && !NetDemoConnected());
	assert(s_TestConnects >= 1 && s_TestRadio && s_TestCredentialChecked);
	if (existing && !NET_DEMO_TLS_REPLACE_CA)
	{
		assert(s_TestCredentialWrites == 0);
	}
	else if (configured)
	{
		assert(s_TestCredentialWrites >= 1);
	}
	assert(!NetDemoOpen(true, "test.example", 1234) && s_TestOpens == 0);
	writes = s_TestCredentialWrites;
	queries = s_TestCredentialQueries;
	connects = s_TestConnects;
	finished = NetDemoNow();
	assert(!NetDemoOpen(false, "test.example", 1234));
	assert(!s_TestResource && !NetDemoConnected() && s_TestCloses == 1);
	assert(NetDemoNow() == finished + s_TestOpenDuration + s_TestCloseDuration);
	assert(s_TestCredentialWrites == writes && s_TestCredentialQueries == queries);
	assert(s_TestConnects == connects);

	s_TestOpenWorks = true;
	OpenEndpoint(false);
	opens = s_TestOpens;
	assert(!NetDemoOpen(false, "test.example", 1234) && s_TestOpens == opens);
	// A lost secure endpoint opens a fresh TLS socket without reprovisioning.
	s_TestSocket->bConnected = false;
	assert(!NetDemoConnected());
	NetDemoClose();
	OpenEndpoint(false);
	assert(s_TestOpens == opens + 1U && s_TestWrites == 2);
	assert(s_TestCredentialWrites == writes && s_TestCredentialQueries == queries);
	assert(s_TestConnects == connects);

	LteStatus_t status = {};
	unsigned inits = s_TestInits;

	status.RegStat = LTE_REG_ROAMING;
	status.Rat = LTE_RAT_LTEM;
	s_TestLteEvent(LTE_EVT_UNREGISTERED, &status);
	assert(!NetDemoConnected() && !s_TestResource);
	s_TestLteEvent(LTE_EVT_REGISTERED, &status);
	OpenEndpoint(false);
	assert(s_TestCredentialWrites == writes && s_TestCredentialQueries == queries);
	s_TestLteEvent(LTE_EVT_MODEM_FAULT, &status);
	finished = NetDemoNow();
	assert(!NetDemoReady() && !NetDemoConnected());
	PumpAt(finished + 29999U);
	assert(s_TestInits == inits);
	PumpAt(finished + 30000U);
	assert(s_TestInits == inits + 1U && NetDemoReady() && !NetDemoConnected());
	assert(s_TestCredentialWrites == writes + (NET_DEMO_TLS_REPLACE_CA ? 1U : 0U));
	OpenEndpoint(false);
	NetDemoClose();
	assert(!s_TestResource);
	assert(s_TestLog.find("LTE recovery test:") == std::string::npos);
	puts("Generic TLS setup: credential gating, no plaintext fallback and secure reopen PASS");
	return 0;
}
#endif

int main(int argc, char **argv)
{
#if NET_DEMO_LTE_RECOVERY_TEST
	assert(argc == 2);
	return RecoverySetup(argv[1]);
#elif NET_DEMO_MODEM_TLS
	assert(argc == 2);
	return SecureSetup(argv[1]);
#else
	uint32_t finished;
	unsigned inits;
	unsigned opens;
	unsigned closes;

	(void)argc;
	(void)argv;
	assert(!NetDemoReady() && !NetDemoConnected());
	assert(!NetDemoStart(Application));
	assert(!NetDemoInit());
	assert(s_TestInits == 0 && s_TestOpens == 0);
	s_TestRng.Works = true;
	assert(NetDemoInit() && NetDemoInit() && NetDemoRng() == &s_TestRng);
	assert(!NetDemoStart(nullptr) && !NetDemoStart(Application));
	assert(s_TestInits == 0);
	s_TestTriggerWorks = true;
	assert(NetDemoStart(Application) && !NetDemoStart(Application));
	finished = NetDemoNow();
	assert(s_TestInits == 1 && s_TestPumps == 0 && !NetDemoReady());
	assert(!NetDemoOpen(false, "test.example", 1234) && s_TestOpens == 0);

	// A full event queue is retried, while repeated timer ticks coalesce.
	Tick();
	assert(s_TestQueues == 1 && !s_TestQueued);
	s_TestQueueWorks = true;
	Tick();
	for (unsigned i = 0; i < 100; i++)
	{
		Tick();
	}
	assert(s_TestQueues == 2 && s_TestPumps == 0 && s_TestInits == 1);
	Dispatch();
	assert(s_TestPumps == 1 && !s_TestHandlerReady && !s_TestHandlerConnected);
	PumpAt(finished + 29999U);
	assert(s_TestInits == 1);
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	// A MAC can start while IP initialization fails; release it before retry.
	s_TestInitWorks = true;
#endif
	PumpAt(finished + 30000U);
	finished = NetDemoNow();
	assert(s_TestInits == 2 && !NetDemoReady());
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	assert(s_TestEthernetCloses == 1);
	s_TestIpWorks = true;
#endif
	s_TestInitWorks = true;
	PumpAt(finished + 29999U);
	assert(s_TestInits == 2);
	PumpAt(finished + 30000U);
	assert(s_TestInits == 3 && !s_TestHandlerReady && s_TestHandlerTime == NetDemoNow());
	s_TestReady = true;
	PumpAt(NetDemoNow());
	assert(NetDemoReady() && s_TestHandlerReady);
#if NET_DEMO_MEDIUM != NET_DEMO_MEDIUM_LTE
	assert(s_TestProcesses == 2);
#else
	assert(s_TestProcesses == 0);
#endif
	assert(!NetDemoOpen(false, nullptr, 1234));
	assert(!NetDemoOpen(false, "", 1234));
	assert(!NetDemoOpen(false, "test.example", 0) && s_TestOpens == 0);

	// Failed and successful opens can block; expose the actual clock afterward.
	finished = NetDemoNow();
	assert(!NetDemoOpen(false, "test.example", 1234));
	assert(!s_TestResource && !NetDemoConnected() && s_TestCloses == 1);
	assert(NetDemoNow() == finished + s_TestOpenDuration + s_TestCloseDuration);
	s_TestOpenWorks = true;
	OpenEndpoint(true);
	opens = s_TestOpens;
	assert(!NetDemoOpen(false, "test.example", 1234) && s_TestOpens == opens);
	finished = NetDemoNow();
	NetDemoClose();
	assert(NetDemoNow() == finished + s_TestCloseDuration && !NetDemoConnected());
	closes = s_TestCloses;
	NetDemoClose();
	assert(s_TestCloses == closes);
	OpenEndpoint(false);
	assert(s_TestWrites == 2);

	// Readiness loss invalidates an endpoint before the protocol callback runs.
	s_TestReady = false;
	assert(!NetDemoConnected());
	PumpAt(NetDemoNow());
	assert(!s_TestResource && !s_TestHandlerConnected && !s_TestHandlerReady);
	s_TestReady = true;
	PumpAt(NetDemoNow());
	assert(s_TestHandlerReady && !s_TestHandlerConnected);
	OpenEndpoint(false);
	inits = s_TestInits;
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_LTE
	LteStatus_t status = {};

	status.RegStat = LTE_REG_ROAMING;
	status.Rat = LTE_RAT_LTEM;
	s_TestLteEvent(LTE_EVT_RRC_IDLE, &status);
	assert(NetDemoConnected());
	// Registration may disappear and return between application timer ticks.
	s_TestReady = false;
	s_TestLteEvent(LTE_EVT_UNREGISTERED, &status);
	s_TestReady = true;
	s_TestLteEvent(LTE_EVT_REGISTERED, &status);
	assert(!NetDemoConnected() && !s_TestResource);
	PumpAt(NetDemoNow());
	assert(s_TestHandlerReady && !s_TestHandlerConnected);
	OpenEndpoint(false);
	s_TestNow = 0xfffff000U;
	s_TestLteEvent(LTE_EVT_MODEM_FAULT, &status);
	finished = NetDemoNow();
	assert(!NetDemoReady() && !NetDemoConnected());
	PumpAt(finished + 29999U);
	assert(s_TestInits == inits);
	PumpAt(finished + 30000U);
	assert(s_TestInits == inits + 1U && s_TestHandlerReady && !s_TestHandlerConnected);
#else
	NetDemoClose();
	assert(s_TestInits == inits);
#endif
	assert(s_TestLog.find("retry in 30 s") != std::string::npos);
	assert(s_TestLog.find("LTE recovery test:") == std::string::npos);
	puts("Generic network setup: deferred pump, retry timing, readiness and direct interface injection PASS");
#endif
}
