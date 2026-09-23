// Stage 0 recovery on simulated memory behind a pseudo terminal: DfuSmp in
// direct mode, SMP over the serial framing (DfuSerial), so a real host
// library with its serial transport can drive it the way it drives a UART or
// USB CDC port.
//
// A reset request resets: the boot runs again. When it starts slot 0, the
// simulated application asks for recovery at once (DfuRecoveryRequest), so
// the client finds the boot again and can check what it installed.
//
// Usage: dfu_serial_host <image dir> [nor|rram|ecc16|big256]
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
#include "dfu/dfu_smp.h"
#include "dfu/dfu_serial.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];

// The pseudo terminal as a byte stream DeviceIntrf, the way a UART is one.
class PtyIntrf : public DeviceIntrf {
public:
	int vFd = -1;
	DevIntrf_t vDev = {};

	operator DevIntrf_t * () override { return &vDev; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate(void) override { return 0; }

	int Rx(uint32_t, uint8_t *pBuff, int BuffLen) override {
		ssize_t n = read(vFd, pBuff, (size_t)BuffLen);
		return n > 0 ? (int)n : 0;
	}
	int Tx(uint32_t, const uint8_t *pData, int DataLen) override {
		ssize_t n = write(vFd, pData, (size_t)DataLen);
		return n > 0 ? (int)n : 0;
	}
};

static PtyIntrf s_Pty;
static DfuImgKey_t s_Key;
static std::vector<uint8_t> s_KeyDer;
static DfuBootCfg_t s_Boot;
static DfuStore_t s_Store;
static DfuSmp s_Mgr;
static DfuSerial s_Ser;
static uint8_t s_Rx[DFU_SERIAL_RXBUF_SIZE(1024)];
static uint8_t s_Tx[DFU_SERIAL_TXBUF_SIZE];
static int s_Boots;

static void Reset(void)
{
	DfuTgtReset();
}

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		fprintf(stderr, "usage: %s <image dir> [nor|rram|ecc16|big256]\n",
				argv[0]);
		return 2;
	}

	HostMem kind = HOST_MEM_NOR;
	if (argc > 2)
	{
		const HostMem k[] = {
			HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16, HOST_MEM_BIG256,
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
	s_Boot.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem));
	s_Boot.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr);
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
		printf("boot %d: started slot 0, violations %d\n", s_Boots,
			   g_HostNorViolations);
		fflush(stdout);
		// The application asks to go back to the boot.
		DfuRecoveryRequest();
	}

	int res = DfuBootRun(s_Boot);
	printf("boot %d: recovery (%d)\n", s_Boots, res);
	fflush(stdout);

	s_Mgr = DfuSmp();
	s_Ser = DfuSerial();
	DfuLayout_t lay = HostLayout();
	static DfuMgr s_DfuMgr;
	DfuMgrCfg_t dmcfg = {
		.pStore = &s_Store,
		.bDirect = true,
		.bManifestOnly = false,
		.bAllowDowngrade = true,
		.pHash = s_Boot.pHash,
		.pBoot = &s_Boot,
	};
	DfuSmpCfg_t mcfg = {
		.pMgr = &s_DfuMgr,
		.BufSize = 1024,
		.ResetCB = DfuSerial::ResetCB,
		.pCtx = &s_Ser,
	};
	DfuSerialCfg_t scfg = {
		.pMgr = &s_Mgr,
		.pIntrf = &s_Pty,
		.pRxBuf = s_Rx,
		.RxBufSize = sizeof(s_Rx),
		.pTxBuf = s_Tx,
		.TxBufSize = sizeof(s_Tx),
		.Reset = Reset,
		.TxRetry = 0,
	};
	if (DfuStoreTgt(&s_Store, lay.Slot0, lay.Slot0Size) == false ||
		s_DfuMgr.Init(dmcfg) == false || s_Mgr.Init(mcfg) == false || s_Ser.Init(scfg) == false)
	{
		fprintf(stderr, "recovery init failed\n");
		return 1;
	}

	for (;;)
	{
		struct pollfd p = { s_Pty.vFd, POLLIN, 0 };
		if (poll(&p, 1, 1000) > 0)
		{
			s_Ser.Poll();
		}
	}
}
