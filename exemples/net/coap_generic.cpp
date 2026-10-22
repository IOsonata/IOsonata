/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * CoAP application over an injected connected datagram DeviceIntrf.
 * Network setup is supplied by the selected net_demo implementation.
 */
#include <string.h>
#include "app_evt_handler.h"
#include "net/coap.h"
#include "syslog.h"
#include "net_demo.h"
#include "board.h"

#define COAP_DEMO_PUBLIC_PAYLOAD_MAX	1024U
#define COAP_DEMO_RECOVERY_MS			3000U

static_assert(COAP_DEMO_INTERVAL >= 1000U && COAP_DEMO_INTERVAL <= 2147483647U,
	"COAP_DEMO_INTERVAL must be 1000..INT32_MAX milliseconds");
static_assert(COAP_DEMO_PUBLIC_TEST == 0 || COAP_DEMO_PUBLIC_TEST == 1,
	"COAP_DEMO_PUBLIC_TEST must be 0 or 1");
static_assert(COAP_DEMO_PORT >= 1 && COAP_DEMO_PORT <= 65535,
	"COAP_DEMO_PORT must be 1..65535");

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
static const char *s_Host = COAP_DEMO_HOST;
static Coap s_Coap;
static DeviceIntrf *s_pIntrf;
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
static bool s_Started;
static bool s_Waiting;
static bool s_RecoveryWaiting;

static void PreserveProbe(uint32_t Now)
{
	uint32_t delay = s_Coap.RetryAfter(Now);
	uint32_t elapsed = Now - s_ProbeStarted;

	// A repeated loss after Reset must not erase an already retained wait.
	if (elapsed < s_ProbeDelay && s_ProbeDelay - elapsed > delay)
	{
		delay = s_ProbeDelay - elapsed;
	}
	s_ProbeDelay = delay;
	s_ProbeStarted = Now;
}

static void CoapDown(uint32_t Now, const char *pReason, bool Close)
{
	bool keep = !Close && s_Started && NetDemoReady() && NetDemoConnected() &&
		s_Coap.Error() != COAP_ERROR_TRANSPORT;
	uint32_t wait;

	s_Failures++;
	SysLogPrintf(SysLogGet(), "CoAP FAIL: %s, error=%u checked=%lu failures=%lu\r\n",
		pReason, (unsigned)s_Coap.Error(), (unsigned long)s_Checked,
		(unsigned long)s_Failures);
	PreserveProbe(Now);
	if (keep)
	{
		// Keep peer IDs, duplicate acknowledgements and rate state intact.
		s_Coap.Release();
	}
	else
	{
		// The next interface may be another medium or the same object reopened.
		s_Coap.Reset();
		NetDemoClose();
		s_pIntrf = nullptr;
		s_Started = false;
	}
	s_Waiting = false;
	s_Step = 0;
	// A socket close may block. Recovery starts when it finishes, while the
	// retained probing wait keeps aging from its original capture above.
	Now = NetDemoNow();
	s_Retry = Now;
	s_RecoveryWaiting = true;
	wait = Now - s_ProbeStarted;
	wait = wait < s_ProbeDelay ? s_ProbeDelay - wait : 0;
	if (wait < COAP_DEMO_RECOVERY_MS)
	{
		wait = COAP_DEMO_RECOVERY_MS;
	}
	SysLogPrintf(SysLogGet(), "CoAP retry wait %lu ms\r\n", (unsigned long)wait);
}

static bool CheckResponse(const CoapMessage_t *pResponse)
{
	const CoapDemoStep_t *step = &s_Steps[s_Step];
	const uint8_t *expected = nullptr;
	size_t len = 0;

	if (step->Response == DEMO_PUBLIC)
	{
		// Public content may change. This is only a bounded GET smoke test.
		return pResponse->Code == COAP_CONTENT &&
			pResponse->PayloadLen <= COAP_DEMO_PUBLIC_PAYLOAD_MAX;
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
		options[0].Number = COAP_OPTION_URI_HOST;
		options[0].pValue = (const uint8_t *)s_Host;
		options[0].Len = s_Host ? strlen(s_Host) : 0;
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
	const CoapCfg_t cfg = {s_pRng, 95000U, COAP_DEMO_PROBING_RATE};
	const CoapMessage_t *response;
	COAP_STATE state;

	if (s_ProbeDelay && (uint32_t)(Now - s_ProbeStarted) >= s_ProbeDelay)
	{
		s_ProbeDelay = 0;
	}
	if (s_RecoveryWaiting && (uint32_t)(Now - s_Retry) >= COAP_DEMO_RECOVERY_MS)
	{
		s_RecoveryWaiting = false;
	}
	if (!s_Started)
	{
		if (s_RecoveryWaiting || s_ProbeDelay)
		{
			return;
		}
		s_pIntrf = NetDemoOpen(true, s_Host, COAP_DEMO_PORT);
		Now = NetDemoNow();
		if (!s_pIntrf)
		{
			CoapDown(Now, "UDP open", true);
			return;
		}
		// The protocol sees the same DeviceIntrf for every selected medium.
		if (!s_Coap.Init(cfg, s_pIntrf, Now))
		{
			CoapDown(Now, "initialization", true);
			return;
		}
		s_Started = true;
		SysLogPrintf(SysLogGet(), "CoAP UDP peer %s:%u, plaintext, interval %lu ms\r\n",
			s_Host, (unsigned)COAP_DEMO_PORT, (unsigned long)COAP_DEMO_INTERVAL);
	}
	state = s_Coap.Process(Now, NetDemoConnected());
	Now = NetDemoNow();
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
		// A reply can be accepted just before its ACK exposes a transport error.
		// Count that reply once, then replace the failed endpoint.
		if (s_Coap.Error() == COAP_ERROR_TRANSPORT || !NetDemoConnected())
		{
			CoapDown(Now, "transport closed after response", true);
			return;
		}
		s_Coap.Release();
	}
	if (!NetDemoConnected())
	{
		CoapDown(Now, "transport closed", true);
		return;
	}
	if (!s_Waiting && !s_Coap.Busy() && !s_RecoveryWaiting && !s_ProbeDelay &&
		s_Coap.RetryAfter(Now) == 0 &&
		(uint32_t)(Now - s_LastRequest) >= COAP_DEMO_INTERVAL)
	{
		StartRequest(Now);
	}
}

static void Report(uint32_t Now)
{
	if (!NetDemoReady())
	{
		if (s_Started)
		{
			CoapDown(Now, "network down", true);
		}
		return;
	}
	CoapTest(Now);
}

static bool PublicResourceValid(const char *pResource)
{
	return pResource && pResource[0] && strlen(pResource) <= 255U &&
		strpbrk(pResource, "/?#") == nullptr && strcmp(pResource, ".") != 0 &&
		strcmp(pResource, "..") != 0;
}

int main()
{
	if (!NetDemoInit())
	{
		return 1;
	}
	SysLogPrintf(SysLogGet(), "Generic CoAP over %s UDP\r\n", NetDemoMediumName());
	SysLogPrintf(SysLogGet(), COAP_DEMO_PUBLIC_TEST ?
		"Profile: public read-only GET\r\n" : "Profile: eight-step helper validation\r\n");
	s_pRng = NetDemoRng();
	if (!s_Host || !s_Host[0])
	{
		SysLogPrintf(SysLogGet(), "Set COAP_DEMO_HOST to your reachable CoAP server in board.h\r\n");
	}
	else if (COAP_DEMO_PUBLIC_TEST && !PublicResourceValid(COAP_DEMO_RESOURCE))
	{
		SysLogPrintf(SysLogGet(), "COAP_DEMO_RESOURCE must be one 1..255-byte path segment\r\n");
	}
	else if (!s_pRng)
	{
		SysLogPrintf(SysLogGet(), "RNG initialization failed\r\n");
	}
	else if (!NetDemoStart(Report))
	{
		SysLogPrintf(SysLogGet(), "CoAP application pump initialization failed\r\n");
	}
	AppRun();
	return 0;
}
