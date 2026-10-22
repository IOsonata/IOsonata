/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Shared CoAP client interoperability and recovery over LTE UDP.
 * A public read-only GET smoke test or the Python/coap_server.py validation.
 */
#include <string.h>
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "lte/lte.h"
#include "net/coap.h"
#include "net/sock_intrf.h"
#include "syslog.h"
#include "board.h"
#include COAP_DEMO_RNG_HEADER

#define LTE_COAP_TRIG_MODEM	0
#define LTE_COAP_TRIG_REPORT	1
#define LTE_COAP_RESTART_MS	30000U
#define LTE_COAP_PUBLIC_PAYLOAD_MAX	1024U

static_assert(COAP_DEMO_INTERVAL >= 1000U && COAP_DEMO_INTERVAL <= 2147483647U,
	"COAP_DEMO_INTERVAL must be 1000..INT32_MAX milliseconds");
static_assert(COAP_DEMO_PUBLIC_TEST == 0 || COAP_DEMO_PUBLIC_TEST == 1,
	"COAP_DEMO_PUBLIC_TEST must be 0 or 1");

typedef enum
{
	DEMO_EMPTY,
	DEMO_HELLO,
	DEMO_BINARY,
	DEMO_SLOW,
	DEMO_PUBLIC
} DEMO_RESPONSE;

typedef struct
{
	COAP_TYPE Type;
	uint8_t Method;
	const char *pPath;
	uint8_t Code;
	DEMO_RESPONSE Response;
	bool bPayload;
} CoapDemoStep_t;

static void LteEvent(LTE_EVT Evt, const LteStatus_t * const pStatus);

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(1024)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = true,
};
static const TimerCfg_t s_TimerCfg = {
	.DevNo = LTE_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = IRQ_PRIO_LOW,
};
static TimerDev_t s_Timer;
static const LteCfg_t s_LteCfg = {
	.Rat = LTE_COAP_RAT,
	.RatPref = LTE_COAP_RAT_PREF,
	.bGnss = false,
	.pBand = nullptr,
	.NbBand = 0,
	.pApn = LTE_COAP_APN,
	.PdnType = LTE_PDN_IPV4V6,
	.bPsm = false,
	.PsmTau = 0,
	.PsmActive = 0,
	.EdrxCycle = 0,
	.bRai = false,
	.IntPrio = IRQ_PRIO_LOW,
	.pTimer = &s_Timer,
	.TimerTrigNo = LTE_COAP_TRIG_MODEM,
	.UrcMemSize = 0,
	.EvtHandler = LteEvent,
	.UrcHandler = nullptr,
};
static const SockIntrfCfg_t s_SockCfg = {
	.Proto = SOCKINTRF_PROTO_UDP,
	.pHost = COAP_DEMO_HOST,
	.Port = COAP_DEMO_PORT,
	.LocalPort = 0,
	.SecTag = -1,
	.bPeerVerify = false,
	.Timeout = 100,
	.EvtCB = nullptr,
};
static const CoapDemoStep_t s_Steps[] = {
#if COAP_DEMO_PUBLIC_TEST
	{COAP_CON, COAP_GET, COAP_DEMO_RESOURCE, COAP_CONTENT, DEMO_PUBLIC, false},
#else
	{COAP_CON, COAP_GET, "hello", COAP_CONTENT, DEMO_HELLO, false},
	{COAP_CON, COAP_POST, "echo", COAP_CONTENT, DEMO_BINARY, true},
	{COAP_NON, COAP_POST, "echo", COAP_CONTENT, DEMO_BINARY, true},
	{COAP_CON, COAP_PUT, "value", COAP_CHANGED, DEMO_EMPTY, true},
	{COAP_CON, COAP_GET, "value", COAP_CONTENT, DEMO_BINARY, false},
	{COAP_CON, COAP_DELETE, "value", COAP_DELETED, DEMO_EMPTY, false},
	{COAP_CON, COAP_GET, "value", COAP_NOT_FOUND, DEMO_EMPTY, false},
	{COAP_CON, COAP_GET, "slow", COAP_CONTENT, DEMO_SLOW, false},
#endif
};
static const uint8_t s_Hello[] = "IOsonata CoAP\n";
static const uint8_t s_Slow[] = "separate response\n";
static const uint8_t s_BinaryFormat[] = {42};
static const size_t s_Lengths[] = {1, 16, 257, 900};
static UART s_Uart;
static SockIntrf s_Socket;
static Coap s_Coap;
static RngEngine *s_pRng;
static uint8_t s_Payload[900];
static size_t s_PayloadLen;
static size_t s_Step;
static uint32_t s_Cycle;
static uint32_t s_Checked;
static uint32_t s_Failures;
static uint32_t s_Retry;
static uint32_t s_ProbeStarted;
static uint32_t s_ProbeDelay;
static uint32_t s_LastRequest;
static uint32_t s_LteRetry;
static bool s_Started;
static bool s_Waiting;
static bool s_RecoveryWaiting;
static bool s_LteStarted;
static volatile bool s_ReportQueued;
alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 192)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 192,
	.bBlocking = false,
};

static void PreserveProbe(uint32_t Now)
{
	uint32_t delay = s_Coap.RetryAfter(Now);
	uint32_t elapsed = Now - s_ProbeStarted;

	// Repeated network faults must not erase a wait retained after Reset.
	if (elapsed < s_ProbeDelay && s_ProbeDelay - elapsed > delay)
	{
		delay = s_ProbeDelay - elapsed;
	}
	s_ProbeDelay = delay;
	s_ProbeStarted = Now;
}

static void CoapDown(uint32_t Now, const char *pReason, bool Close)
{
	bool keep = !Close && s_Started && LteRegistered() && s_Socket.Connected() &&
		s_Coap.Error() != COAP_ERROR_TRANSPORT;
	uint32_t wait;

	s_Failures++;
	SysLogPrintf(SysLogGet(), "CoAP FAIL: %s, error=%u checked=%lu failures=%lu\r\n",
		pReason, (unsigned)s_Coap.Error(), (unsigned long)s_Checked,
		(unsigned long)s_Failures);
	PreserveProbe(Now);
	if (keep)
	{
		// Preserve peer IDs, duplicate acknowledgements and rate state.
		s_Coap.Release();
	}
	else
	{
		s_Coap.Reset();
		s_Socket.Close();
		s_Started = false;
	}
	s_Waiting = false;
	s_Step = 0;
	s_Retry = Now;
	s_RecoveryWaiting = true;
	wait = s_ProbeDelay > 3000U ? s_ProbeDelay : 3000U;
	SysLogPrintf(SysLogGet(), "CoAP retry wait %lu ms\r\n", (unsigned long)wait);
}

static bool CheckResponse(const CoapMessage_t *pResponse)
{
	const CoapDemoStep_t *step = &s_Steps[s_Step];
	const uint8_t *expected = nullptr;
	size_t len = 0;

	if (step->Response == DEMO_PUBLIC)
	{
		// Public content can change. This verifies a bounded 2.05 response,
		// without claiming the helper's methods or binary echo coverage.
		return pResponse->Code == COAP_CONTENT &&
			pResponse->PayloadLen <= LTE_COAP_PUBLIC_PAYLOAD_MAX;
	}
	if (step->Response == DEMO_HELLO)
	{
		expected = s_Hello;
		len = sizeof(s_Hello) - 1U;
	}
	else if (step->Response == DEMO_BINARY)
	{
		expected = s_Payload;
		len = s_PayloadLen;
	}
	else if (step->Response == DEMO_SLOW)
	{
		expected = s_Slow;
		len = sizeof(s_Slow) - 1U;
		if (pResponse->Type != COAP_CON)
		{
			return false;
		}
	}
	return pResponse->Code == step->Code && pResponse->PayloadLen == len &&
		(len == 0 || memcmp(pResponse->pPayload, expected, len) == 0);
}

static void StartRequest(uint32_t Now)
{
	const CoapDemoStep_t *step = &s_Steps[s_Step];
	CoapOption_t options[3] = {};
	CoapRequest_t request = {};
	uint32_t pattern;
	size_t i;

	if (s_Step == 1)
	{
		s_PayloadLen = s_Lengths[s_Cycle % (sizeof(s_Lengths) / sizeof(s_Lengths[0]))];
		pattern = s_Cycle + 1U;
		for (i = 0; i < s_PayloadLen; i++)
		{
			pattern = pattern * 1664525U + 1013904223U;
			s_Payload[i] = (uint8_t)(pattern >> 24);
		}
	}
	if (step->Response == DEMO_PUBLIC)
	{
		// Identify the named origin even when it shares an IP address.
		options[0].Number = COAP_OPTION_URI_HOST;
		options[0].pValue = (const uint8_t *)s_SockCfg.pHost;
		options[0].Len = strlen(s_SockCfg.pHost);
		request.OptionCount = 1;
	}
	options[request.OptionCount].Number = COAP_OPTION_URI_PATH;
	options[request.OptionCount].pValue = (const uint8_t *)step->pPath;
	options[request.OptionCount].Len = strlen(step->pPath);
	request.OptionCount++;
	request.Type = step->Type;
	request.Code = step->Method;
	request.pOptions = options;
	if (step->bPayload)
	{
		options[request.OptionCount].Number = COAP_OPTION_CONTENT_FORMAT;
		options[request.OptionCount].pValue = s_BinaryFormat;
		options[request.OptionCount].Len = sizeof(s_BinaryFormat);
		request.OptionCount++;
		request.pPayload = s_Payload;
		request.PayloadLen = s_PayloadLen;
	}
	if (!s_Coap.Request(request, Now))
	{
		if (s_Coap.Error() == COAP_ERROR_RATE)
		{
			PreserveProbe(Now);
			return;
		}
		CoapDown(Now, "request rejected", false);
		return;
	}
	s_Waiting = true;
	s_LastRequest = Now;
}

static void CoapTest(uint32_t Now)
{
	const CoapCfg_t cfg = {s_pRng, 95000, COAP_DEMO_PROBING_RATE};
	const CoapMessage_t *response;
	DevIntrf_t *transport;
	COAP_STATE state;

	if (s_ProbeDelay && (uint32_t)(Now - s_ProbeStarted) >= s_ProbeDelay)
	{
		s_ProbeDelay = 0;
	}
	if (s_RecoveryWaiting && (uint32_t)(Now - s_Retry) >= 3000U)
	{
		s_RecoveryWaiting = false;
	}
	if (!s_Started)
	{
		if (s_RecoveryWaiting || s_ProbeDelay)
		{
			return;
		}
		if (!s_Socket.Init(s_SockCfg))
		{
			CoapDown(TimerGetMilisecond(&s_Timer), "UDP open", true);
			return;
		}
		transport = s_Socket;
		transport->MaxRetry = 0;
		Now = TimerGetMilisecond(&s_Timer);
		if (!s_Coap.Init(cfg, &s_Socket, Now))
		{
			CoapDown(Now, "initialization", true);
			return;
		}
		s_Started = true;
		SysLogPrintf(SysLogGet(), "CoAP UDP peer %s:%u, plaintext, interval %lu ms\r\n",
			s_SockCfg.pHost, (unsigned)s_SockCfg.Port, (unsigned long)COAP_DEMO_INTERVAL);
	}
	state = s_Coap.Process(Now, s_Socket.Connected());
	Now = TimerGetMilisecond(&s_Timer);
	if (state == COAP_FAILED)
	{
		CoapDown(Now, "exchange", false);
		return;
	}
	response = s_Coap.Response();
	if (response)
	{
		if (!s_Waiting || !CheckResponse(response))
		{
			SysLogPrintf(SysLogGet(), "CoAP response rejected: code=%u.%02u bytes=%u\r\n",
				(unsigned)(response->Code >> 5), (unsigned)(response->Code & 31U),
				(unsigned)response->PayloadLen);
			CoapDown(Now, "response code, type or payload mismatch", false);
			return;
		}
		s_Checked++;
		if (s_Cycle == 0 || s_Checked % 10U == 0)
		{
			if (COAP_DEMO_PUBLIC_TEST)
			{
				SysLogPrintf(SysLogGet(), "CoAP public GET checked %lu code=2.05 bytes=%u failures=%lu\r\n",
					(unsigned long)s_Checked, (unsigned)response->PayloadLen,
					(unsigned long)s_Failures);
			}
			else
			{
				SysLogPrintf(SysLogGet(), "CoAP checked %lu step=%u code=%u.%02u bytes=%u failures=%lu\r\n",
					(unsigned long)s_Checked, (unsigned)(s_Step + 1U),
					(unsigned)(response->Code >> 5), (unsigned)(response->Code & 31U),
					(unsigned)response->PayloadLen, (unsigned long)s_Failures);
			}
		}
		s_Coap.Release();
		s_Waiting = false;
		s_Step++;
		if (s_Step == sizeof(s_Steps) / sizeof(s_Steps[0]))
		{
			s_Step = 0;
			s_Cycle++;
			if (s_Cycle == 1)
			{
				SysLogPrintf(SysLogGet(), COAP_DEMO_PUBLIC_TEST ?
					"CoAP public GET smoke test verified\r\n" :
					"CoAP methods, CON/NON and separate response verified\r\n");
			}
		}
	}
	if (!s_Waiting && !s_Coap.Busy() && !s_RecoveryWaiting && !s_ProbeDelay &&
		s_Coap.RetryAfter(Now) == 0 &&
		(uint32_t)(Now - s_LastRequest) >= COAP_DEMO_INTERVAL)
	{
		StartRequest(Now);
	}
}

static void StartLte()
{
	char info[48];

	s_LteStarted = LteInit(&s_LteCfg);
	s_LteRetry = TimerGetMilisecond(&s_Timer);
	if (!s_LteStarted)
	{
		s_Failures++;
		SysLogPrintf(SysLogGet(), "LTE init failed, retry in 30 s, failures=%lu\r\n",
			(unsigned long)s_Failures);
		return;
	}
	if (LteGetInfo(LTE_INFO_FWVER, info, sizeof(info)))
	{
		SysLogPrintf(SysLogGet(), "Modem firmware %s\r\n", info);
	}
	SysLogPrintf(SysLogGet(), "Attaching; PSM, eDRX and RAI disabled for this test\r\n");
}

static void LteEvent(LTE_EVT Evt, const LteStatus_t * const pStatus)
{
	uint32_t now = TimerGetMilisecond(&s_Timer);

	switch (Evt)
	{
		case LTE_EVT_REGISTERED:
			SysLogPrintf(SysLogGet(), "Registered %s, %s\r\n",
				pStatus->RegStat == LTE_REG_ROAMING ? "roaming" : "home",
				pStatus->Rat == LTE_RAT_NBIOT ? "NB-IoT" : "LTE-M");
			break;
		case LTE_EVT_UNREGISTERED:
			SysLogPrintf(SysLogGet(), "Unregistered, state %u\r\n", (unsigned)pStatus->RegStat);
			if (s_Started)
			{
				CoapDown(now, "network down", true);
			}
			break;
		case LTE_EVT_REG_STATE:
			SysLogPrintf(SysLogGet(), "Registration state %u\r\n", (unsigned)pStatus->RegStat);
			break;
		case LTE_EVT_RRC_CONNECTED:
			SysLogPrintf(SysLogGet(), "RRC connected\r\n");
			break;
		case LTE_EVT_RRC_IDLE:
			SysLogPrintf(SysLogGet(), "RRC idle\r\n");
			break;
		case LTE_EVT_MODEM_FAULT:
			CoapDown(now, "modem fault", true);
			s_LteStarted = false;
			s_LteRetry = now;
			SysLogPrintf(SysLogGet(), "Modem restart in 30 s\r\n");
			break;
		default:
			break;
	}
}

static void Report(uint32_t Evt, void *pCtx)
{
	uint32_t now = TimerGetMilisecond(&s_Timer);

	(void)Evt;
	(void)pCtx;
	s_ReportQueued = false;
	if (!s_LteStarted)
	{
		if ((uint32_t)(now - s_LteRetry) >= LTE_COAP_RESTART_MS)
		{
			StartLte();
		}
		return;
	}
	if (!LteRegistered())
	{
		if (s_Started)
		{
			CoapDown(now, "network down", true);
		}
		return;
	}
	CoapTest(now);
}

static void ReportTimer(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	if (!s_ReportQueued)
	{
		// A refused enqueue is retried at the next timer interrupt.
		s_ReportQueued = AppEvtHandlerQue(0, nullptr, Report);
	}
}

static bool PublicResourceValid(const char *pResource)
{
	// One raw Uri-Path segment, not a URI or a slash-separated path.
	return pResource && pResource[0] && strlen(pResource) <= 255U &&
		strpbrk(pResource, "/?#") == nullptr && strcmp(pResource, ".") != 0 &&
		strcmp(pResource, "..") != 0;
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_LogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
	SysLogPrintf(SysLogGet(), "CoAP over LTE UDP\r\n");
	SysLogPrintf(SysLogGet(), COAP_DEMO_PUBLIC_TEST ?
		"Profile: public read-only GET\r\n" : "Profile: eight-step helper validation\r\n");
	s_pRng = COAP_DEMO_RNG();
	if (!s_SockCfg.pHost || !s_SockCfg.pHost[0])
	{
		SysLogPrintf(SysLogGet(), "Set COAP_DEMO_HOST to your reachable CoAP server in board.h\r\n");
	}
	else if (COAP_DEMO_PUBLIC_TEST && !PublicResourceValid(COAP_DEMO_RESOURCE))
	{
		SysLogPrintf(SysLogGet(), "COAP_DEMO_RESOURCE must be one 1..255-byte path segment\r\n");
	}
	else if (!s_pRng || !s_pRng->Enable())
	{
		SysLogPrintf(SysLogGet(), "RNG initialization failed\r\n");
	}
	else if (!TimerInit(&s_Timer, &s_TimerCfg))
	{
		SysLogPrintf(SysLogGet(), "Timer initialization failed\r\n");
	}
	else if (!msTimerEnableTrigger(&s_Timer, LTE_COAP_TRIG_REPORT, 20,
		TIMER_TRIG_TYPE_CONTINUOUS, ReportTimer, nullptr))
	{
		SysLogPrintf(SysLogGet(), "CoAP timer initialization failed\r\n");
	}
	else
	{
		StartLte();
	}
	AppRun();
	return 0;
}
