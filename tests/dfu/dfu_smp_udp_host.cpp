// The SMP server on simulated memory behind a UDP socket, SMP over UDP as the
// SMP tools define it (one packet per datagram, port 1337 by default), so a
// real host library can drive it. A reset request runs the stage 0 boot on
// the simulated memory, the way a device reboots into it.
//
// Usage: dfu_smp_udp_host <image dir> [port] [nor|rram]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "dfu/dfu_smp.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];

static bool s_bReset = false;

static void ResetCB(void *)
{
	s_bReset = true;
}

static const char *Boot(const DfuBootCfg_t &Cfg)
{
	if (setjmp(g_HostStartJmp) != 0)
	{
		return "started slot 0";
	}
	static char s[40];
	snprintf(s, sizeof(s), "no application (%d)", DfuBootRun(Cfg));
	return s;
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		fprintf(stderr, "usage: %s <image dir> [port] [nor|rram]\n", argv[0]);
		return 2;
	}
	std::string dir = argv[1];
	int port = argc > 2 ? atoi(argv[2]) : 1337;
	HostMem kind = argc > 3 && strcmp(argv[3], "rram") == 0 ? HOST_MEM_RRAM :
															 HOST_MEM_NOR;

	HostFlashInit(kind);

	std::vector<uint8_t> key = HostReadFile(dir + "/key1.der");
	DfuImgKey_t k = { key.data(), (uint32_t)key.size() };
	DfuBootCfg_t boot = {
		.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem)),
		.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr),
		.pKey = &k,
		.NbKey = 1,
		.bVerifySlot0 = true,
		.bAllowNoRec = false,
	};

	HostNvm slot1(HOST_SLOT1, HOST_SLOT1_SIZE);
	DfuStore_t st;
	DfuSmp mgr;
	DfuStoreNvm(&st, &slot1);
	static DfuMgr s_DfuMgr;
	DfuMgrCfg_t dmcfg = {
		.pStore = &st,
		.bDirect = false,
		.bManifestOnly = false,
		.bAllowDowngrade = true,
		.pHash = boot.pHash,
		.pBoot = nullptr,
	};
	(void)s_DfuMgr.Init(dmcfg);
	DfuSmpCfg_t cfg = {
		.pMgr = &s_DfuMgr,
		.BufSize = 1024,
		.ResetCB = ResetCB,
		.pCtx = nullptr,
	};
	if (mgr.Init(cfg) == false)
	{
		fprintf(stderr, "DfuSmp init failed\n");
		return 1;
	}

	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in a = {};
	a.sin_family = AF_INET;
	a.sin_port = htons((uint16_t)port);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (fd < 0 || bind(fd, (sockaddr *)&a, sizeof(a)) != 0)
	{
		perror("bind");
		return 1;
	}
	printf("listening on 127.0.0.1:%d, %s\n", port,
		   kind == HOST_MEM_NOR ? "NOR" : "RRAM");
	fflush(stdout);

	for (;;)
	{
		uint8_t req[2048], rsp[512];
		sockaddr_in from;
		socklen_t fl = sizeof(from);
		ssize_t n = recvfrom(fd, req, sizeof(req), 0, (sockaddr *)&from, &fl);
		if (n <= 0)
		{
			continue;
		}
		int l = mgr.Process(req, (uint32_t)n, rsp, sizeof(rsp));
		if (l > 0)
		{
			sendto(fd, rsp, (size_t)l, 0, (sockaddr *)&from, fl);
		}
		if (s_bReset)
		{
			s_bReset = false;
			printf("reset: %s\n", Boot(boot));
			printf("nor violations %d\n", g_HostNorViolations);
			fflush(stdout);
			// A new server, as after a real reset.
			mgr = DfuSmp();
			if (mgr.Init(cfg) == false)
			{
				return 1;
			}
		}
	}
}
