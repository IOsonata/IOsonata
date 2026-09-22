// Download an image over a real TCP socket with DfuHttp into simulated slot
// 1, resuming on a new connection whenever the server drops one, then run
// the boot on it.
//
// Usage: dfu_http_host <image dir> <port> <path>
// Prints "installed" when the boot started the downloaded image.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "dfu/dfu_http.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];

// A connected TCP socket as a byte stream DeviceIntrf.
class SockIntrf : public DeviceIntrf {
public:
	int vFd = -1;
	bool vbEof = false;
	DevIntrf_t vDev = {};

	operator DevIntrf_t * () override { return &vDev; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate(void) override { return 0; }

	bool Open(int Port) {
		vFd = socket(AF_INET, SOCK_STREAM, 0);
		sockaddr_in a = {};
		a.sin_family = AF_INET;
		a.sin_port = htons((uint16_t)Port);
		a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		vbEof = false;
		if (connect(vFd, (sockaddr *)&a, sizeof(a)) != 0)
		{
			return false;
		}
		fcntl(vFd, F_SETFL, O_NONBLOCK);
		return true;
	}
	void Close() { close(vFd); vFd = -1; }
	int Rx(uint32_t, uint8_t *p, int Len) override {
		ssize_t n = recv(vFd, p, (size_t)Len, 0);
		if (n == 0) vbEof = true;
		return n > 0 ? (int)n : 0;
	}
	int Tx(uint32_t, const uint8_t *p, int Len) override {
		ssize_t n = send(vFd, p, (size_t)Len, 0);
		return n > 0 ? (int)n : 0;
	}
};

int main(int argc, char **argv)
{
	if (argc < 4)
	{
		fprintf(stderr, "usage: %s <image dir> <port> <path>\n", argv[0]);
		return 2;
	}
	int port = atoi(argv[2]);

	HostFlashInit(HOST_MEM_NOR);
	std::vector<uint8_t> key = HostReadFile(std::string(argv[1]) + "/key1.der");
	DfuImgKey_t k = { key.data(), (uint32_t)key.size() };
	DfuBootCfg_t boot = {
		.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem)),
		.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr),
		.pKey = &k,
		.NbKey = 1,
		.bVerifySlot0 = true,
		.bAllowNoRec = false,
	};

	DfuLayout_t lay = HostLayout();
	static DfuStore_t st;
	static DfuMgr wr;
	static DfuHttp http;
	static SockIntrf s;
	(void)lay;
	bool ok = DfuStoreTgt(&st, HOST_SLOT1, HOST_SLOT1_SIZE);
	DfuMgrCfg_t mc = {
		.pStore = &st, .bDirect = false, .bManifestOnly = false,
		.bAllowDowngrade = true, .pHash = boot.pHash, .pBoot = nullptr,
	};
	if (ok == false || wr.Init(mc) == false || http.Init(&wr) == false)
	{
		return 1;
	}

	int conns = 0;
	for (;;)
	{
		if (s.Open(port) == false)
		{
			perror("connect");
			return 1;
		}
		conns++;
		bool ok = conns == 1 ? http.Get(&s, "127.0.0.1", argv[3]) :
							   http.Resume(&s);
		if (ok == false)
		{
			printf("request failed\n");
			return 1;
		}
		for (;;)
		{
			struct pollfd p = { s.vFd, POLLIN, 0 };
			poll(&p, 1, 2000);
			DFU_HTTP_STATE st2 = http.Poll();
			if (st2 == DFU_HTTP_DONE || st2 == DFU_HTTP_FAILED)
			{
				break;
			}
			if (s.vbEof)
			{
				http.StreamEnded();
				break;
			}
		}
		s.Close();
		printf("connection %d: state %d, %u of %u bytes\n", conns,
			   (int)http.State(), http.Received(), http.Total());
		if (http.State() != DFU_HTTP_BROKEN)
		{
			break;
		}
	}

	if (http.State() != DFU_HTTP_DONE)
	{
		printf("failed %d (%d)\n", http.Error(), http.ImgErr());
		return 1;
	}

	if (setjmp(g_HostStartJmp) != 0)
	{
		printf("installed, %d connections, violations %d\n", conns,
			   g_HostNorViolations);
		return 0;
	}
	printf("boot: %d\n", DfuBootRun(boot));

	return 1;
}
