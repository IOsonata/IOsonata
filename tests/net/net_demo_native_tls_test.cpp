/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Actual native TLS example setup, with scripted TLS and hardware operations.
 * Cryptographic peer verification is tested separately by run_tls_cert.py.
 */
#define NET_DEMO_NATIVE_TLS 1
#define NET_DEMO_TLS_SERVER_NAME "server.example"
#define main GenericSetupMain
#include "net_demo_setup_test.cpp"
#undef main

static const uint8_t s_TestCaDer[] = {0x30, 0x00};
const X509Cert_t g_NetDemoTlsTrustAnchors[] = {{s_TestCaDer, sizeof(s_TestCaDer)}};
#ifdef NET_DEMO_TEST_NO_CA
const uint32_t g_NetDemoTlsTrustAnchorCount = 0;
#else
const uint32_t g_NetDemoTlsTrustAnchorCount = 1;
#endif
static bool s_TestTlsInitWorks = true;
static bool s_TestTlsReady;
static bool s_TestTlsFailure;
static uint64_t s_TestTlsTime;
static unsigned s_TestTlsInits;
static unsigned s_TestTlsProcesses;

extern "C"
{
bool TlsInit(TlsDev_t *pDev, const TlsCfg_t *pCfg, DevIntrf_t *pTransport, uint32_t Now)
{
	assert(!s_TestInterrupt && pDev && pCfg && pTransport && Now == NetDemoNow());
	assert(!pCfg->pPsk && !pCfg->PskLen && !pCfg->pIdentity && !pCfg->IdentityLen);
	assert(pCfg->pHash && pCfg->pMac && pCfg->pAead && pCfg->pRng && pCfg->pKeyAgree);
	assert(pCfg->pCert && pCfg->pCert->pSign && pCfg->pCert->TrustAnchorCount == 1U);
	assert(pCfg->pCert->pTrustAnchors == g_NetDemoTlsTrustAnchors);
	assert(pCfg->pCert->pHandshakeBuffer && pCfg->pCert->HandshakeBufferSize == 8192U);
	assert(pCfg->pCert->pTranscriptBuffer && pCfg->pCert->TranscriptBufferSize == 12288U);
	assert(strcmp(pCfg->pServerName, "server.example") == 0 && pCfg->Timeout == 30000U);
	assert(pTransport->MaxRetry == 0 && s_TestResource && s_TestConnected);
	s_TestTlsInits++;
	s_TestTlsTime = pCfg->pCert->UnixTime;
	pDev->pTransport = pTransport;
	pDev->State = s_TestTlsInitWorks ? TLS_SERVER_HELLO : TLS_FAILED;
	pDev->Error = s_TestTlsInitWorks ? TLS_ERROR_NONE : TLS_ERROR_CONFIG;
	pDev->CertificateError = X509_STATUS_OK;
	return s_TestTlsInitWorks;
}

TLS_STATE TlsProcess(TlsDev_t *pDev, uint32_t Now, bool Open)
{
	assert(!s_TestInterrupt && pDev && Now == NetDemoNow());
	s_TestTlsProcesses++;
	if (!Open || s_TestTlsFailure)
	{
		pDev->State = TLS_FAILED;
		pDev->Error = Open ? TLS_ERROR_CERTIFICATE : TLS_ERROR_TRANSPORT;
		pDev->CertificateError = Open ? X509_STATUS_TIME : X509_STATUS_OK;
	}
	else if (s_TestTlsReady)
	{
		pDev->State = TLS_READY;
	}
	return pDev->State;
}

void TlsReset(TlsDev_t *pDev)
{
	pDev->State = TLS_IDLE;
	pDev->Error = TLS_ERROR_NONE;
	pDev->CertificateError = X509_STATUS_OK;
	pDev->pTransport = nullptr;
}

bool TlsClose(TlsDev_t *)
{
	return false;
}
}

static void ConsoleCommand(const std::string &Line)
{
	size_t before;

	assert(s_TestConsoleInput.empty());
	s_TestConsoleInput = Line;
	do
	{
		before = s_TestConsoleInput.size();
		PumpAt(NetDemoNow());
		assert(before - s_TestConsoleInput.size() <= 32U);
	}
	while (!s_TestConsoleInput.empty());
}

int main()
{
	DeviceIntrf *intrf;
	uint32_t clockSet;
	uint64_t expected;
	unsigned closes;
	unsigned calls;

	s_TestRng.Works = s_TestTriggerWorks = s_TestQueueWorks = true;
	s_TestInitWorks = s_TestOpenWorks = s_TestReady = true;
#if NET_DEMO_MEDIUM == NET_DEMO_MEDIUM_ETHERNET
	s_TestIpWorks = true;
#endif
	assert(NetDemoInit());
#ifdef NET_DEMO_TEST_NO_CA
	assert(!NetDemoStart(Application));
	assert(!s_TestInits && !s_TestOpens && !NetDemoReady());
	assert(!NetDemoOpen(false, "test.example", 1234));
	assert(s_TestLog.find("configuration missing") != std::string::npos);
	puts("Native TLS setup: missing CA blocks medium and connection startup PASS");
	return 0;
#endif
	assert(NetDemoStart(Application) && s_TestInits == 1);
	assert(!NetDemoReady() && !NetDemoOpening() && !NetDemoConnected());
	assert(!NetDemoOpen(false, "test.example", 1234) && !s_TestOpens);
	assert(strcmp(NetDemoSecurityName(), "TLS 1.3, CA/hostname/time verified") == 0);
	for (const char *line : {"time 0\n", "time -1\n", "time 253402300800\n",
		"time 18446744073709551616\n", "time 1700000000x\n", "time \n",
		"clock 1700000000\n", "time 000000000000000000000000000000000000000000000000000000000001\n"})
	{
		ConsoleCommand(line);
		assert(!s_UtcValid && !NetDemoReady() && !s_TestOpens);
	}
	ConsoleCommand("time 1700000000\r\n");
	clockSet = s_UtcTick;
	assert(s_UtcValid && NetDemoReady() && s_UtcSeconds == 1700000000U);
	assert(s_TestLog.find("TLS UTC set from console") != std::string::npos);
	PumpAt(clockSet + 6000U);
	assert(NetDemoNow() < clockSet);
	expected = 1700000000U + (uint32_t)(NetDemoNow() - clockSet) / 1000U;
	assert(s_UtcSeconds == expected);
	assert(s_UtcRemainder == (uint32_t)(NetDemoNow() - clockSet) % 1000U);
	assert(!NetDemoOpen(true, "test.example", 1234) && !s_TestOpens);
	assert(!NetDemoOpen(false, nullptr, 1234));
	assert(!NetDemoOpen(false, "", 1234));
	assert(!NetDemoOpen(false, "test.example", 0));

	// Native TLS is layered over ordinary TCP; it never selects the modem TLS
	// socket or hands the raw transport to MQTT while authentication is pending.
	intrf = NetDemoOpen(false, "test.example", 1234);
	assert(intrf == &s_Tls && s_pEndpoint && intrf != s_pEndpoint);
	assert(NetDemoOpening() && !NetDemoConnected() && s_TestTlsInits == 1U);
	assert(s_TestTlsTime == 1700000000U + (uint32_t)(NetDemoNow() - clockSet) / 1000U);
	assert(!NetDemoOpen(false, "test.example", 1234) && s_TestOpens == 1);
	ConsoleCommand("time 1800000000\n");
	assert(s_TestLog.find("active connection") != std::string::npos && s_UtcSeconds < 1800000000U);
	assert(NetDemoOpening() && !s_TestHandlerConnected);
	calls = s_TestTlsProcesses;
	s_TestTlsReady = true;
	PumpAt(NetDemoNow());
	assert(s_TestTlsProcesses == calls + 1U && !NetDemoOpening());
	assert(NetDemoConnected() && s_TestHandlerConnected);

	// Connection loss reaches the application after the TLS layer is reset.
	s_TestReady = false;
	PumpAt(NetDemoNow());
	assert(!NetDemoConnected() && !NetDemoOpening() && !s_TestResource);
	assert(!s_TestHandlerConnected && s_UtcValid);
	s_TestReady = true;
	s_TestTlsReady = false;
	assert(NetDemoOpen(false, "test.example", 1234) == &s_Tls);
	assert(s_TestTlsInits == 2U && NetDemoOpening());
	s_TestTlsFailure = true;
	PumpAt(NetDemoNow());
	assert(!NetDemoOpening() && !NetDemoConnected() && s_TestResource);
	assert(s_TestLog.find("certificate_error=-5") != std::string::npos);
	NetDemoClose();
	assert(!s_TestResource && !s_pEndpoint);
	closes = s_TestCloses;
	NetDemoClose();
	assert(s_TestCloses == closes);

	s_TestTlsFailure = false;
	s_TestTlsInitWorks = false;
	assert(!NetDemoOpen(false, "test.example", 1234));
	assert(!s_TestResource && s_TestCloses == closes + 1U);
	assert(!NetDemoOpening() && !NetDemoConnected());
	ConsoleCommand("time 1800000000\n");
	assert(s_UtcSeconds == 1800000000U);
	puts("Native TLS setup: UTC gating/wrap, bounded console, handshake gating and reconnect PASS");
	return 0;
}
