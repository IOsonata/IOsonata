// Stage 0 recovery on simulated memory behind a pseudo terminal, with the
// wire protocol (DfuWire) over one DfuMgr in direct mode, manifest only: the
// build a target makes with DFU_PROTO_WIRE. The host tool, Python/dfu_wire.py,
// drives it the way it drives a UART or a USB CDC port.
//
// A RESET resets: the boot runs again. When it starts slot 0, the simulated
// application asks for recovery at once (DfuRecoveryRequest), so the tool
// finds the boot again and the test can check what it installed.
//
// Usage: dfu_wire_host <image dir> [nor|rram|ecc16|big256|sect]
// Prints the terminal path, then one line per boot.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "dfu/dfu_wire.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_HASH_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_VERIFY_MEMSIZE];

#define MAXBODY		512

// The pseudo terminal as a byte stream DeviceIntrf, the way a UART is one.
class PtyIntrf : public HostByteIntrf {
public:
	int vFd = -1;

	int RxBytes(uint8_t *pBuff, int BuffLen) override {
		ssize_t n = read(vFd, pBuff, (size_t)BuffLen);
		return n > 0 ? (int)n : 0;
	}
	int TxBytes(const uint8_t *pData, int DataLen) override {
		ssize_t n = write(vFd, pData, (size_t)DataLen);
		return n > 0 ? (int)n : 0;
	}
};

static PtyIntrf s_Pty;
static DfuImgKey_t s_Key;
static std::vector<uint8_t> s_KeyDer;
static DfuBootCfg_t s_Boot;
static DfuStore_t s_Store;
static DfuMgr s_Mgr;
static DfuWire s_Wire;
static uint8_t s_Rx[DFU_WIRE_RXBUF_SIZE(MAXBODY)];
static int s_Boots;

static void Reset(bool bRecovery)
{
	// The response is out: write() to a pty does not queue.
	s_Mgr.Reset(bRecovery);
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		fprintf(stderr, "usage: %s <image dir> [nor|rram|ecc16|big256|sect]\n",
				argv[0]);
		return 2;
	}

	HostMem kind = HOST_MEM_NOR;
	if (argc > 2)
	{
		const HostMem k[] = {
			HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16, HOST_MEM_BIG256,
			HOST_MEM_SECT,
		};
		for (HostMem m : k)
		{
			if (strcasecmp(argv[2], HostMemName(m)) == 0)
			{
				kind = m;
			}
		}
	}
	HostFlashInit(kind);

	s_KeyDer = HostReadFile(std::string(argv[1]) + "/key1.der");
	s_Key = { s_KeyDer.data(), (uint32_t)s_KeyDer.size() };
	s_Boot.pHash = CryptoSoftSha256HashCreate(s_ShaMem, sizeof(s_ShaMem));
	s_Boot.pSign = CryptoUeccVerifyCreate(s_EccMem, sizeof(s_EccMem));
	s_Boot.pKey = &s_Key;
	s_Boot.NbKey = 1;
	s_Boot.bVerifySlot0 = true;

	s_Pty.vFd = posix_openpt(O_RDWR | O_NOCTTY);
	if (s_Pty.vFd < 0 || grantpt(s_Pty.vFd) != 0 || unlockpt(s_Pty.vFd) != 0)
	{
		perror("pty");
		return 1;
	}
	struct termios t;
	tcgetattr(s_Pty.vFd, &t);
	cfmakeraw(&t);
	tcsetattr(s_Pty.vFd, TCSANOW, &t);
	fcntl(s_Pty.vFd, F_SETFL, O_NONBLOCK);

	printf("%s %s\n", ptsname(s_Pty.vFd), HostMemName(kind));
	fflush(stdout);

	if (setjmp(g_HostResetJmp) != 0)
	{
		// Here after every reset, as from the reset vector.
	}

	s_Boots++;
	if (setjmp(g_HostStartJmp) != 0)
	{
		DfuImgInfo_t info;
		DfuLayout_t l = HostLayout();
		int maj = DfuSlot0Parse(l, &info) == DFU_IMG_OK ? info.Hdr.Ver.Major : -1;
		printf("boot %d: started slot 0 v%d, violations %d\n", s_Boots, maj,
			   g_HostNorViolations);
		fflush(stdout);
		// The application asks to go back to the boot.
		DfuRecoveryRequest();
	}

	int res = DfuBootRun(s_Boot);
	printf("boot %d: recovery (%d)\n", s_Boots, res);
	fflush(stdout);

	s_Mgr = DfuMgr();
	// Init sets DfuWire up again from nothing.
	DfuLayout_t lay = HostLayout();
	DfuMgrCfg_t mcfg = {
		.pStore = &s_Store,
		.bDirect = true,
		.bManifestOnly = true,
		.bAllowDowngrade = false,
		.pHash = s_Boot.pHash,
		.pBoot = &s_Boot,
	};
	static const uint8_t devid[8] = { 'I', 'O', 'S', 'O', 'N', 'A', 'T', 'A' };
	DfuWireCfg_t wcfg = {
		.pMgr = &s_Mgr,
		.pIntrf = &s_Pty,
		.pRxBuf = s_Rx,
		.RxBufSize = sizeof(s_Rx),
		.MaxBody = MAXBODY,
		.EraseMaxMs = 100,
		.BootVer = 0x00010000,
		.pDevId = devid,
		.Reset = Reset,
	};
	if (DfuStoreTgt(&s_Store, lay.Slot0, lay.Slot0Size) == false ||
		s_Mgr.Init(mcfg) == false || s_Wire.Init(wcfg) == false)
	{
		fprintf(stderr, "recovery init failed\n");
		return 1;
	}

	for (;;)
	{
		struct pollfd p = { s_Pty.vFd, POLLIN, 0 };
		if (poll(&p, 1, 1000) > 0)
		{
			s_Wire.Poll();
		}
	}
}
