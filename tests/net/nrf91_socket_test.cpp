/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Actual nRF91 socket port with deterministic Nordic API substitutes.
 * No modem firmware, radio, DNS server or remote TCP endpoint is exercised.
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include "nrf_socket.h"
#include "nrf_errno.h"
#include "net/sock_intrf.h"
#include "syslog.h"

enum TestFailure
{
	FAIL_NONE,
	FAIL_DNS,
	FAIL_SOCKET,
	FAIL_CONNECT_TIMEOUT,
	FAIL_TAG,
	FAIL_VERIFY,
	FAIL_HOST,
	FAIL_BIND,
	FAIL_CONNECT,
	FAIL_SEND_TIMEOUT,
	FAIL_POLLCB
};

enum TestDns
{
	DNS_IPV4,
	DNS_IPV6,
	DNS_BOTH
};

struct TestSocket
{
	int Family;
	int Type;
	int Protocol;
	bool Open = true;
	bool Connected = false;
	uint32_t SendTimeout = 0;
	uint32_t ConnectTimeout = 0;
	uint16_t LocalPort = 0;
	bool Tag = false;
	bool Verify = false;
	bool Host = false;
	nrf_modem_pollcb_t Callback = nullptr;
	std::vector<uint32_t> Timeouts;
};

static TestFailure s_Failure;
static TestDns s_Dns;
static int s_FailFamily;
static int s_FailErrno;
static int s_DnsResult;
static uint32_t s_ConnectDuration;
static uint32_t s_SendDuration;
static unsigned s_Closes;
static unsigned s_Frees;
static unsigned s_Connects;
static bool s_DnsOwned;
static uint16_t s_RemotePort;
static std::vector<int> s_Queries;
static std::vector<int> s_Families;
static std::vector<TestSocket> s_Sockets;
static std::vector<std::string> s_Trace;
static std::vector<std::string> s_Log;
static nrf_addrinfo s_AddrInfo[2];
static nrf_sockaddr_in s_Addr4;
static nrf_sockaddr_in6 s_Addr6;
static SysLog_t s_TestLog;

static TestSocket &Socket(int Handle)
{
	assert(Handle >= 0 && (size_t)Handle < s_Sockets.size());
	assert(s_Sockets[Handle].Open);
	return s_Sockets[Handle];
}

static bool Fails(TestFailure Stage, int Family)
{
	if (s_Failure == Stage && (s_FailFamily == 0 || s_FailFamily == Family))
	{
		errno = s_FailErrno;
		return true;
	}
	return false;
}

static size_t OpenCount()
{
	size_t count = 0;

	for (const TestSocket &socket : s_Sockets)
	{
		count += socket.Open;
	}
	return count;
}

static void Address(nrf_addrinfo *pInfo, int Family)
{
	*pInfo = {};
	pInfo->ai_family = Family;
	pInfo->ai_socktype = NRF_SOCK_STREAM;
	if (Family == NRF_AF_INET)
	{
		s_Addr4 = {};
		s_Addr4.sin_family = NRF_AF_INET;
		s_Addr4.sin_addr.s_addr = 0x0100007fU;
		pInfo->ai_addr = (nrf_sockaddr *)&s_Addr4;
		pInfo->ai_addrlen = sizeof(s_Addr4);
	}
	else
	{
		s_Addr6 = {};
		s_Addr6.sin6_family = NRF_AF_INET6;
		s_Addr6.sin6_addr.s6_addr[15] = 1;
		pInfo->ai_addr = (nrf_sockaddr *)&s_Addr6;
		pInfo->ai_addrlen = sizeof(s_Addr6);
	}
}

extern "C"
{
int nrf_getaddrinfo(const char *pNode, const char *pService,
	const nrf_addrinfo *pHints, nrf_addrinfo **pResult)
{
	assert(pNode && strcmp(pNode, "broker.example") == 0 && pService == nullptr);
	assert(pHints && pResult && !s_DnsOwned);
	s_Queries.push_back(pHints->ai_family);
	s_Trace.push_back("resolve");
	*pResult = nullptr;
	if (Fails(FAIL_DNS, pHints->ai_family))
	{
		return s_DnsResult;
	}
	if (pHints->ai_family == NRF_AF_UNSPEC)
	{
		Address(s_AddrInfo, s_Dns == DNS_IPV4 ? NRF_AF_INET : NRF_AF_INET6);
		if (s_Dns == DNS_BOTH)
		{
			Address(s_AddrInfo + 1, NRF_AF_INET);
			s_AddrInfo[0].ai_next = s_AddrInfo + 1;
		}
	}
	else
	{
		assert(pHints->ai_family == NRF_AF_INET);
		Address(s_AddrInfo, NRF_AF_INET);
	}
	*pResult = s_AddrInfo;
	s_DnsOwned = true;
	return 0;
}

void nrf_freeaddrinfo(nrf_addrinfo *pInfo)
{
	assert(pInfo == s_AddrInfo && s_DnsOwned);
	s_DnsOwned = false;
	s_Frees++;
	s_Trace.push_back("free");
	errno = 992;
}

int nrf_socket(int Family, int Type, int Protocol)
{
	s_Families.push_back(Family);
	s_Trace.push_back("socket");
	if (Fails(FAIL_SOCKET, Family))
	{
		return -1;
	}
	s_Sockets.push_back({Family, Type, Protocol});
	return (int)s_Sockets.size() - 1;
}

int nrf_setsockopt(int Handle, int Level, int Name, const void *pValue, nrf_socklen_t Len)
{
	TestSocket &socket = Socket(Handle);
	TestFailure stage;
	const char *name;

	if (Level == NRF_SOL_SOCKET && Name == NRF_SO_SNDTIMEO)
	{
		assert(Len == sizeof(nrf_timeval));
		const nrf_timeval *tv = (const nrf_timeval *)pValue;

		stage = socket.Connected ? FAIL_SEND_TIMEOUT : FAIL_CONNECT_TIMEOUT;
		name = socket.Connected ? "send-timeout" : "connect-timeout";
		s_Trace.push_back(name);
		if (Fails(stage, socket.Family))
		{
			return -1;
		}
		assert(tv->tv_usec < 1000000U);
		socket.SendTimeout = tv->tv_sec * 1000U + tv->tv_usec / 1000U;
		socket.Timeouts.push_back(socket.SendTimeout);
		return 0;
	}
	if (Level == NRF_SOL_SOCKET && Name == NRF_SO_POLLCB)
	{
		assert(Len == sizeof(nrf_modem_pollcb) && socket.Connected);
		s_Trace.push_back("pollcb");
		if (Fails(FAIL_POLLCB, socket.Family))
		{
			return -1;
		}
		socket.Callback = ((const nrf_modem_pollcb *)pValue)->callback;
		assert(socket.Callback != nullptr);
		return 0;
	}
	assert(Level == NRF_SOL_SECURE && !socket.Connected);
	if (Name == NRF_SO_SEC_TAG_LIST)
	{
		stage = FAIL_TAG;
		assert(Len == sizeof(nrf_sec_tag_t) && *(const nrf_sec_tag_t *)pValue == 42U);
		socket.Tag = true;
	}
	else if (Name == NRF_SO_SEC_PEER_VERIFY)
	{
		stage = FAIL_VERIFY;
		assert(Len == sizeof(int) && *(const int *)pValue == NRF_SO_SEC_PEER_VERIFY_REQUIRED);
		socket.Verify = true;
	}
	else
	{
		assert(Name == NRF_SO_SEC_HOSTNAME);
		stage = FAIL_HOST;
		assert(Len == strlen("broker.example") && !memcmp(pValue, "broker.example", Len));
		socket.Host = true;
	}
	s_Trace.push_back("secure-option");
	return Fails(stage, socket.Family) ? -1 : 0;
}

int nrf_bind(int Handle, const nrf_sockaddr *pAddress, nrf_socklen_t Len)
{
	TestSocket &socket = Socket(Handle);

	assert(!socket.Connected && pAddress->sa_family == socket.Family);
	assert(Len == (socket.Family == NRF_AF_INET ? sizeof(nrf_sockaddr_in) : sizeof(nrf_sockaddr_in6)));
	s_Trace.push_back("bind");
	if (Fails(FAIL_BIND, socket.Family))
	{
		return -1;
	}
	socket.LocalPort = socket.Family == NRF_AF_INET ?
		((const nrf_sockaddr_in *)pAddress)->sin_port :
		((const nrf_sockaddr_in6 *)pAddress)->sin6_port;
	assert(socket.LocalPort == nrf_htons(4545));
	return 0;
}

int nrf_connect(int Handle, const nrf_sockaddr *pAddress, nrf_socklen_t Len)
{
	TestSocket &socket = Socket(Handle);
	uint16_t port = socket.Family == NRF_AF_INET ?
		((const nrf_sockaddr_in *)pAddress)->sin_port :
		((const nrf_sockaddr_in6 *)pAddress)->sin6_port;

	assert(!socket.Connected && pAddress->sa_family == socket.Family);
	assert(Len == (socket.Family == NRF_AF_INET ? sizeof(nrf_sockaddr_in) : sizeof(nrf_sockaddr_in6)));
	assert(port == nrf_htons(s_RemotePort));
	if (socket.Protocol == NRF_SPROTO_TLS1v2)
	{
		assert(socket.Tag && socket.Verify && socket.Host);
	}
	s_Connects++;
	s_Trace.push_back("connect");
	socket.ConnectTimeout = socket.SendTimeout;
	if (Fails(FAIL_CONNECT, socket.Family))
	{
		return -1;
	}
	// Model a normal cellular TCP setup that takes longer than a short TX wait.
	if (socket.Type == NRF_SOCK_STREAM && socket.SendTimeout != 0 &&
		socket.SendTimeout < s_ConnectDuration)
	{
		errno = NRF_EINPROGRESS;
		return -1;
	}
	socket.Connected = true;
	return 0;
}

int nrf_close(int Handle)
{
	TestSocket &socket = Socket(Handle);

	socket.Open = false;
	socket.Connected = false;
	s_Closes++;
	s_Trace.push_back("close");
	errno = 991;
	return 0;
}

int nrf_poll(nrf_pollfd *pFds, nrf_nfds_t Count, int Timeout)
{
	assert(Count == 1 && Timeout == 0);
	(void)Socket(pFds[0].fd);
	pFds[0].revents = 0;
	return 0;
}

ssize_t nrf_send(int Handle, const void *, size_t Len, int Flags)
{
	TestSocket &socket = Socket(Handle);

	assert(socket.Connected && Flags == 0);
	if (socket.SendTimeout != 0 && socket.SendTimeout < s_SendDuration)
	{
		errno = NRF_EAGAIN;
		return -1;
	}
	return (ssize_t)Len;
}

ssize_t nrf_recv(int Handle, void *, size_t, int Flags)
{
	(void)Socket(Handle);
	assert(Flags == NRF_MSG_DONTWAIT);
	errno = NRF_EAGAIN;
	return -1;
}

int nrf_modem_at_printf(const char *, ...)
{
	return -1;
}

int nrf_modem_at_cmd(char *, size_t, const char *, ...)
{
	return -1;
}

SysLog_t *SysLogGet(void)
{
	return &s_TestLog;
}

int SysLogPrintf(SysLog_t *pLog, const char *pFormat, ...)
{
	char line[512];
	va_list args;
	int count;

	assert(pLog == &s_TestLog);
	va_start(args, pFormat);
	count = vsnprintf(line, sizeof(line), pFormat, args);
	va_end(args);
	assert(count > 0 && (size_t)count < sizeof(line));
	s_Log.emplace_back(line);
	s_Trace.push_back("log");
	errno = 993;
	return count;
}
}

static SockIntrfCfg_t Config(SOCKINTRF_PROTO Proto = SOCKINTRF_PROTO_TCP, uint32_t Timeout = 100)
{
	SockIntrfCfg_t cfg = {};

	cfg.Proto = Proto;
	cfg.pHost = "broker.example";
	cfg.Port = s_RemotePort = 1883;
	cfg.Timeout = Timeout;
	cfg.SecTag = 42;
	cfg.bPeerVerify = true;
	return cfg;
}

static void Reset(TestDns Dns = DNS_IPV4)
{
	assert(OpenCount() == 0 && !s_DnsOwned);
	s_Dns = Dns;
	s_Failure = FAIL_NONE;
	s_FailFamily = 0;
	s_FailErrno = NRF_ECONNREFUSED;
	s_DnsResult = -3;
	s_ConnectDuration = 750;
	s_SendDuration = 0;
	s_Closes = s_Frees = s_Connects = 0;
	s_Queries.clear();
	s_Families.clear();
	s_Sockets.clear();
	s_Trace.clear();
	s_Log.clear();
	errno = 0;
}

static void RequireLog(const char *pStage, int Family, int Result, int Error,
	size_t Index = 0)
{
	char expected[256];

	snprintf(expected, sizeof(expected),
		"nRF91 socket %s failed: family=%d result=%d errno=%d\r\n",
		pStage, Family, Result, Error);
	assert(Index < s_Log.size() && s_Log[Index] == expected);
}

static void RequireClosed(SockIntrfDev_t *pDev)
{
	assert(!SockIntrfConnected(pDev) && pDev->Hdl < 0);
	assert(OpenCount() == 0 && !s_DnsOwned);
	assert(s_Closes == s_Sockets.size());
	unsigned closes = s_Closes;

	SockIntrfClose(pDev);
	assert(s_Closes == closes);
}

static void TestTimeouts()
{
	const uint32_t waits[] = {100, 0, 1234, 65000};

	for (uint32_t wait : waits)
	{
		Reset();
		SockIntrfDev_t dev = {};
		SockIntrfCfg_t cfg = Config(SOCKINTRF_PROTO_TCP, wait);

		// The simulated connection takes 750 ms, independently of TxData wait.
		assert(SockIntrfInit(&dev, &cfg));
		assert(SockIntrfConnected(&dev) && OpenCount() == 1 && !s_DnsOwned);
		TestSocket &socket = Socket(dev.Hdl);

		assert(socket.ConnectTimeout == 30000U);
		assert(socket.SendTimeout == (wait != 0 ? wait : 30000U));
		assert(socket.Type == NRF_SOCK_STREAM && socket.Protocol == NRF_IPPROTO_TCP);
		assert(s_Queries == std::vector<int>{NRF_AF_UNSPEC});
		assert(s_Frees == 1 && s_Log.empty());

		// After connecting, a blocked send observes the caller's shorter wait.
		const uint8_t data[] = {0, 1, 0xff, 2};
		uint8_t rx[4];

		s_SendDuration = 500;
		assert(SockIntrfTx(&dev, data, sizeof(data)) ==
			(wait != 0 && wait < s_SendDuration ? 0 : (int)sizeof(data)));
		assert(SockIntrfConnected(&dev));
		s_SendDuration = 0;
		assert(SockIntrfTx(&dev, data, sizeof(data)) == (int)sizeof(data));
		assert(SockIntrfRx(&dev, rx, sizeof(rx)) == 0);
		assert(SockIntrfConnected(&dev));
		SockIntrfClose(&dev);
		RequireClosed(&dev);
	}

	// The timeout separation also applies when modem TLS is selected, and
	// preserves the existing connected UDP configuration used by CoAP.
	const SOCKINTRF_PROTO protocols[] = {SOCKINTRF_PROTO_TLS, SOCKINTRF_PROTO_UDP};

	for (SOCKINTRF_PROTO proto : protocols)
	{
		Reset();
		SockIntrfDev_t dev = {};
		SockIntrfCfg_t cfg = Config(proto);

		cfg.LocalPort = 4545;
		assert(SockIntrfInit(&dev, &cfg));
		assert(Socket(dev.Hdl).ConnectTimeout == 30000U);
		assert(Socket(dev.Hdl).SendTimeout == 100U);
		assert(Socket(dev.Hdl).LocalPort == nrf_htons(4545));
		assert(s_Log.empty());
		SockIntrfClose(&dev);
		RequireClosed(&dev);
	}
}

static void TestIpv4Fallback()
{
	const TestFailure failures[] = {FAIL_CONNECT, FAIL_SOCKET, FAIL_CONNECT_TIMEOUT};
	const char *stages[] = {"connect", "create", "connect timeout"};
	const int errors[] = {NRF_ENETUNREACH, NRF_EAFNOSUPPORT, NRF_ENOPROTOOPT};

	for (size_t i = 0; i < sizeof(failures) / sizeof(failures[0]); i++)
	{
		Reset(DNS_IPV6);
		SockIntrfDev_t dev = {};
		SockIntrfCfg_t cfg = Config();

		s_Failure = failures[i];
		s_FailFamily = NRF_AF_INET6;
		s_FailErrno = errors[i];
		assert(SockIntrfInit(&dev, &cfg));
		assert((s_Queries == std::vector<int>{NRF_AF_UNSPEC, NRF_AF_INET}));
		assert((s_Families == std::vector<int>{NRF_AF_INET6, NRF_AF_INET}));
		assert(OpenCount() == 1 && Socket(dev.Hdl).Family == NRF_AF_INET);
		assert(Socket(dev.Hdl).ConnectTimeout == 30000U);
		assert(Socket(dev.Hdl).SendTimeout == 100U);
		assert(s_Frees == 2 && s_Log.size() == 1);
		RequireLog(stages[i], NRF_AF_INET6, -1, errors[i]);
		assert(s_Closes == (failures[i] == FAIL_SOCKET ? 0U : 1U));
		SockIntrfClose(&dev);
		RequireClosed(&dev);
	}
}

static void TestLookupLimits()
{
	// An IPv6 connection that works never needs a second lookup.
	Reset(DNS_IPV6);
	SockIntrfDev_t dev = {};
	SockIntrfCfg_t cfg = Config();

	assert(SockIntrfInit(&dev, &cfg));
	assert(s_Queries == std::vector<int>{NRF_AF_UNSPEC});
	assert(s_Families == std::vector<int>{NRF_AF_INET6});
	assert(s_Frees == 1 && s_Log.empty());
	SockIntrfClose(&dev);
	RequireClosed(&dev);

	// If both families were returned, the existing list already contains A.
	Reset(DNS_BOTH);
	s_Failure = FAIL_CONNECT;
	s_FailFamily = NRF_AF_INET6;
	s_FailErrno = NRF_ENETUNREACH;
	assert(SockIntrfInit(&dev, &cfg));
	assert(s_Queries == std::vector<int>{NRF_AF_UNSPEC});
	assert((s_Families == std::vector<int>{NRF_AF_INET6, NRF_AF_INET}));
	assert(Socket(dev.Hdl).Family == NRF_AF_INET && s_Frees == 1);
	assert(s_Closes == 1 && s_Log.size() == 1);
	RequireLog("connect", NRF_AF_INET6, -1, NRF_ENETUNREACH);
	SockIntrfClose(&dev);
	RequireClosed(&dev);

	// A failed A record is not looked up twice, whether returned alone or
	// together with AAAA. Only the IPv6-only result gets one IPv4 retry.
	const TestDns answers[] = {DNS_IPV4, DNS_BOTH, DNS_IPV6};

	for (TestDns answer : answers)
	{
		Reset(answer);
		s_Failure = FAIL_CONNECT;
		s_FailErrno = NRF_ECONNREFUSED;
		assert(!SockIntrfInit(&dev, &cfg));
		assert(s_Queries.size() == (answer == DNS_IPV6 ? 2U : 1U));
		assert(s_Queries[0] == NRF_AF_UNSPEC);
		if (answer == DNS_IPV6)
		{
			assert(s_Queries[1] == NRF_AF_INET);
		}
		assert(s_Frees == s_Queries.size());
		assert(s_Connects == (answer == DNS_IPV4 ? 1U : 2U));
		assert(s_Log.size() == s_Connects);
		for (size_t i = 0; i < s_Families.size(); i++)
		{
			RequireLog("connect", s_Families[i], -1, NRF_ECONNREFUSED, i);
		}
		RequireClosed(&dev);
	}
}

static void TestFailureDiagnostics()
{
	struct FailureCase
	{
		TestFailure Failure;
		const char *pStage;
		int Error;
		SOCKINTRF_PROTO Proto;
	};
	const FailureCase cases[] = {
		{FAIL_DNS, "DNS", NRF_ENETUNREACH, SOCKINTRF_PROTO_TCP},
		{FAIL_SOCKET, "create", NRF_EMFILE, SOCKINTRF_PROTO_TCP},
		{FAIL_CONNECT_TIMEOUT, "connect timeout", NRF_ENOPROTOOPT, SOCKINTRF_PROTO_TCP},
		{FAIL_TAG, "security tag", NRF_EINVAL, SOCKINTRF_PROTO_TLS},
		{FAIL_VERIFY, "peer verify", NRF_EINVAL, SOCKINTRF_PROTO_TLS},
		{FAIL_HOST, "hostname", NRF_EINVAL, SOCKINTRF_PROTO_TLS},
		{FAIL_BIND, "bind", NRF_EADDRINUSE, SOCKINTRF_PROTO_TCP},
		{FAIL_CONNECT, "connect", NRF_EINPROGRESS, SOCKINTRF_PROTO_TCP},
		{FAIL_SEND_TIMEOUT, "TX timeout", NRF_ENOPROTOOPT, SOCKINTRF_PROTO_TCP},
		{FAIL_POLLCB, "poll callback", NRF_ENOPROTOOPT, SOCKINTRF_PROTO_TCP}
	};

	for (const FailureCase &test : cases)
	{
		Reset();
		SockIntrfDev_t dev = {};
		SockIntrfCfg_t cfg = Config(test.Proto);

		cfg.LocalPort = 4545;
		s_Failure = test.Failure;
		s_FailErrno = test.Error;
		assert(!SockIntrfInit(&dev, &cfg));
		assert(s_Queries == std::vector<int>{NRF_AF_UNSPEC});
		assert(s_Log.size() == 1);
		RequireLog(test.pStage, test.Failure == FAIL_DNS ? NRF_AF_UNSPEC : NRF_AF_INET,
			test.Failure == FAIL_DNS ? s_DnsResult : -1, test.Error);
		assert(s_Frees == (test.Failure == FAIL_DNS ? 0U : 1U));
		RequireClosed(&dev);

		// The log captured the failure before close/free and SysLog itself
		// replaced errno. Post-connect option failures must close the socket.
		if (test.Failure != FAIL_DNS)
		{
			size_t event = 0;

			while (event < s_Trace.size() && s_Trace[event] != "log")
			{
				event++;
			}
			assert(event + 1 < s_Trace.size());
			assert(s_Trace[event + 1] == "close" || s_Trace[event + 1] == "free");
		}
		if (test.Failure == FAIL_SEND_TIMEOUT || test.Failure == FAIL_POLLCB)
		{
			assert(s_Connects == 1 && s_Closes == 1);
		}

		// The same C interface can initialize again after every failure.
		s_Failure = FAIL_NONE;
		s_Log.clear();
		assert(SockIntrfInit(&dev, &cfg));
		assert(SockIntrfConnected(&dev) && OpenCount() == 1 && s_Log.empty());
		SockIntrfClose(&dev);
		RequireClosed(&dev);
	}
}

static void TestLocalFailures()
{
	Reset();
	SockIntrfDev_t dev = {};
	SockIntrfCfg_t cfg = Config();

	dev.Hdl = -1;
	cfg.Port = 0;
	assert(!SockIntrfInit(&dev, &cfg));
	assert(s_Queries.empty() && s_Log.size() == 1);
	RequireLog("configuration", NRF_AF_UNSPEC, -1, NRF_EINVAL);
	RequireClosed(&dev);

	Reset();
	SockIntrfDev_t sockets[NRF_MODEM_MAX_SOCKET_COUNT + 1] = {};

	cfg = Config();
	for (int i = 0; i < NRF_MODEM_MAX_SOCKET_COUNT; i++)
	{
		assert(SockIntrfInit(sockets + i, &cfg));
	}
	assert(OpenCount() == NRF_MODEM_MAX_SOCKET_COUNT);
	assert(!SockIntrfInit(sockets + NRF_MODEM_MAX_SOCKET_COUNT, &cfg));
	assert(OpenCount() == NRF_MODEM_MAX_SOCKET_COUNT && s_Closes == 1);
	assert(s_Log.size() == 1);
	RequireLog("socket table", NRF_AF_INET, -1, NRF_ENOMEM);
	assert(!sockets[NRF_MODEM_MAX_SOCKET_COUNT].bConnected);
	assert(sockets[NRF_MODEM_MAX_SOCKET_COUNT].Hdl < 0);
	for (int i = 0; i < NRF_MODEM_MAX_SOCKET_COUNT; i++)
	{
		SockIntrfClose(sockets + i);
	}
	RequireClosed(sockets + NRF_MODEM_MAX_SOCKET_COUNT);
	assert(SockIntrfInit(&dev, &cfg));
	SockIntrfClose(&dev);
	RequireClosed(&dev);
}

int main()
{
	TestTimeouts();
	TestIpv4Fallback();
	TestLookupLimits();
	TestFailureDiagnostics();
	TestLocalFailures();
	puts("nRF91 socket timeout, address fallback, diagnostics and cleanup PASS");
	return 0;
}
