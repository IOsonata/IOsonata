// SMP over the Bluetooth LE transport, on host: requests split over writes of
// the link MTU, responses split into notifications, the stack refusing some,
// a request arriving while the last response is still going out, and a full
// upload and install through it, with the real DfuSmp behind.
//
// The stack is stubbed at the calls bt_dfu_smp.cpp makes: service add,
// notify, notification enabled, the peer table and the application event
// queue, which the test runs by hand the way BtAppRun does.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <deque>
#include <random>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "app_evt_handler.h"
#include "bluetooth/bt_dfu_smp.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

static int s_Fail = 0;
static int s_Pass = 0;

#define CHECK(c) do { if (c) { s_Pass++; } else { s_Fail++; \
	fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)

// ---------------------------------------------------------------------------
// Stack stubs
// ---------------------------------------------------------------------------

static BtGattSrvc_t *s_pSrvc = nullptr;
static BtDevice_t s_Peer;
static bool s_bNotifyOn = true;
static int s_NotifyRefuse = 0;			// Refuse this many notifications first
static int s_NotifyRefuseEvery = 0;		// Then refuse every n-th, 0 never
static int s_NotifyCnt = 0;
static std::vector<uint8_t> s_Rx;		// What the host received
static int s_MaxNotify = 0;

struct QEvt {
	AppEvtHandler_t Handler;
	uint32_t Evt;
	void *pCtx;
};
static std::deque<QEvt> s_Que;
static size_t s_QueMax = 8;

extern "C" bool AppEvtHandlerQue(uint32_t EvtId, void *pCtx, AppEvtHandler_t Handler)
{
	if (s_Que.size() >= s_QueMax)
	{
		return false;
	}
	s_Que.push_back({ Handler, EvtId, pCtx });
	return true;
}

static void RunQue(void)
{
	while (!s_Que.empty())
	{
		QEvt e = s_Que.front();
		s_Que.pop_front();
		e.Handler(e.Evt, e.pCtx);
	}
}

static AppEvtHandlerIdle_t s_Idle = nullptr;

extern "C" bool AppEvtHandlerIdleRegister(AppEvtHandlerIdle_t Handler)
{
	s_Idle = Handler;
	return true;
}

// The main loop: the queue, then the idle pump, as AppEvtHandlerExec does.
static void Loop(void)
{
	RunQue();
	if (s_Idle != nullptr)
	{
		s_Idle();
	}
	RunQue();
}

extern "C" bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	s_pSrvc = pSrvc;
	return true;
}

extern "C" bool BtGattCharNotifyEnabled(uint16_t ConnHdl, BtGattChar_t *pChar)
{
	return s_bNotifyOn && ConnHdl == s_Peer.Conn.Hdl && pChar != nullptr;
}

extern "C" bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar,
								 void * const pVal, size_t Len)
{
	if (ConnHdl != s_Peer.Conn.Hdl || Len > pChar->MaxDataLen ||
		Len > (size_t)s_Peer.Conn.MaxMtu - 3)
	{
		CHECK(false);
		return false;
	}
	if (s_NotifyRefuse > 0)
	{
		s_NotifyRefuse--;
		return false;
	}
	s_NotifyCnt++;
	if (s_NotifyRefuseEvery != 0 && (s_NotifyCnt % s_NotifyRefuseEvery) == 0)
	{
		return false;
	}
	s_Rx.insert(s_Rx.end(), (uint8_t *)pVal, (uint8_t *)pVal + Len);
	if ((int)Len > s_MaxNotify)
	{
		s_MaxNotify = (int)Len;
	}
	return true;
}

extern "C" uint16_t BtPeerCount(void)
{
	return 1;
}

extern "C" BtDevice_t *BtPeerSlot(uint16_t Idx)
{
	return Idx == 0 ? &s_Peer : nullptr;
}

extern "C" BtDevice_t *BtPeerFindByHdl(uint16_t ConnHdl)
{
	return ConnHdl == s_Peer.Conn.Hdl && ConnHdl != BT_CONN_HDL_INVALID ?
		   &s_Peer : nullptr;
}

// ---------------------------------------------------------------------------

static std::string s_Dir;
alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];
static uint8_t s_SmpRx[1024], s_SmpTx[512];
static int s_Resets = 0;

static void ResetCB(void *)
{
	s_Resets++;
}

static BtGattChar_t *Char(void)
{
	return &s_pSrvc->pCharArray[0];
}

// The host: write a request in MTU sized pieces.
static void Write(const std::vector<uint8_t> &Req, uint32_t Mtu)
{
	for (size_t off = 0; off < Req.size(); off += Mtu - 3)
	{
		size_t n = Req.size() - off < Mtu - 3 ? Req.size() - off : Mtu - 3;
		Char()->WrCB(Char(), (uint8_t *)Req.data() + off, 0, (int)n);
	}
}

// The stack reports completions until the response is out.
static void Drain(void)
{
	for (int i = 0; i < 1000; i++)
	{
		RunQue();
		if (BtDfuSmpTxBusy() == false)
		{
			return;
		}
		Char()->TxCompleteCB(Char(), 0);
	}
	CHECK(false);
}

static uint8_t s_Seq = 0;

static std::vector<uint8_t> Packet(int Op, int Grp, int Id,
								   const std::vector<uint8_t> &Body)
{
	std::vector<uint8_t> p = {
		(uint8_t)((1 << 3) | Op), 0, (uint8_t)(Body.size() >> 8),
		(uint8_t)Body.size(), (uint8_t)(Grp >> 8), (uint8_t)Grp, ++s_Seq,
		(uint8_t)Id,
	};
	p.insert(p.end(), Body.begin(), Body.end());
	return p;
}

// One response off the received bytes, by its header length.
static bool TakeRsp(std::vector<uint8_t> &Rsp)
{
	if (s_Rx.size() < 8)
	{
		return false;
	}
	size_t n = 8 + ((s_Rx[2] << 8) | s_Rx[3]);
	if (s_Rx.size() < n)
	{
		return false;
	}
	Rsp.assign(s_Rx.begin(), s_Rx.begin() + n);
	s_Rx.erase(s_Rx.begin(), s_Rx.begin() + n);
	return true;
}

static std::vector<uint8_t> Echo(const char *pStr)
{
	uint8_t b[300];
	CborWr_t w;
	CborWrInit(&w, b, sizeof(b));
	CborPutMap(&w, 1);
	CborPutStr(&w, "d");
	CborPutStr(&w, pStr);
	return Packet(DFUSMP_OP_WRITE, DFUSMP_GRP_OS, DFUSMP_OS_ECHO,
				  std::vector<uint8_t>(b, b + w.Len));
}

static bool EchoOk(const std::vector<uint8_t> &Rsp, const char *pStr)
{
	CborFld_t f[] = { CBOR_FLD("r", CBOR_FLD_TSTR) };
	return Rsp.size() > 8 && Rsp[7] == DFUSMP_OS_ECHO &&
		   CborMapRead(Rsp.data() + 8, (uint32_t)Rsp.size() - 8, f, 1) &&
		   f[0].S.Len == strlen(pStr) && memcmp(f[0].S.p, pStr, f[0].S.Len) == 0;
}

static void TestTransport(void)
{
	std::string big(200, 'x');
	std::vector<uint8_t> rsp;

	for (uint32_t mtu : { 23u, 65u, 185u, 247u })
	{
		s_Peer.Conn.MaxMtu = (uint16_t)mtu;
		s_MaxNotify = 0;

		// A request over several writes, a response over several notifications.
		Write(Echo(big.c_str()), mtu);
		CHECK(s_Rx.empty());		// nothing until the queue runs
		Drain();
		CHECK(TakeRsp(rsp) && EchoOk(rsp, big.c_str()));
		CHECK(s_MaxNotify <= (int)mtu - 3);
		CHECK(s_Rx.empty());

		// The stack refusing: the rest goes on completions.
		s_NotifyRefuse = 1;
		s_NotifyRefuseEvery = 2;
		Write(Echo(big.c_str()), mtu);
		Drain();
		CHECK(TakeRsp(rsp) && EchoOk(rsp, big.c_str()));
		s_NotifyRefuseEvery = 0;

		// The next request arrives while the response is still going out.
		s_NotifyRefuse = 1000;
		Write(Echo("first"), mtu);
		RunQue();
		CHECK(BtDfuSmpTxBusy());
		Write(Echo("second"), mtu);
		RunQue();
		s_NotifyRefuse = 0;
		Drain();
		CHECK(TakeRsp(rsp) && EchoOk(rsp, "first"));
		CHECK(TakeRsp(rsp) && EchoOk(rsp, "second"));
		CHECK(s_Rx.empty());
	}

	// Oversize request: dropped, the next one works.
	std::vector<uint8_t> huge = Packet(DFUSMP_OP_WRITE, 0, 0,
									   std::vector<uint8_t>(1100, 0xA0));
	Write(huge, 247);
	Drain();
	CHECK(s_Rx.empty());
	Write(Echo("after"), 247);
	Drain();
	CHECK(TakeRsp(rsp) && EchoOk(rsp, "after"));

	// Notifications off: no response, nothing stuck.
	s_bNotifyOn = false;
	Write(Echo("quiet"), 247);
	Drain();
	CHECK(s_Rx.empty() && BtDfuSmpTxBusy() == false);
	s_bNotifyOn = true;

	// Queue full at the moment the request completes: dropped, recovered.
	s_QueMax = 0;
	Write(Echo("lost"), 247);
	s_QueMax = 8;
	Drain();
	CHECK(s_Rx.empty());
	Write(Echo("found"), 247);
	Drain();
	CHECK(TakeRsp(rsp) && EchoOk(rsp, "found"));

	// Refused while another characteristic filled the stack queue: no
	// completion of ours ever comes, the main loop goes on with it.
	s_NotifyRefuse = 3;
	Write(Echo(big.c_str()), 65);
	for (int i = 0; i < 10 && (BtDfuSmpTxBusy() || s_Que.size()); i++)
	{
		Loop();
	}
	CHECK(TakeRsp(rsp) && EchoOk(rsp, big.c_str()));
	CHECK(BtDfuSmpTxBusy() == false);

	// Notifications turned off part way through a response: the rest is
	// dropped, the next request is answered.
	s_Peer.Conn.MaxMtu = 23;
	s_NotifyRefuse = 1000;
	Write(Echo(big.c_str()), 23);
	RunQue();
	CHECK(BtDfuSmpTxBusy());
	s_bNotifyOn = false;
	s_NotifyRefuse = 0;
	Loop();
	CHECK(BtDfuSmpTxBusy() == false);
	s_bNotifyOn = true;
	s_Rx.clear();
	Write(Echo("again"), 23);
	Drain();
	CHECK(TakeRsp(rsp) && EchoOk(rsp, "again"));
	s_Peer.Conn.MaxMtu = 247;

	// A disconnection in the middle of a request.
	std::vector<uint8_t> e = Echo(big.c_str());
	Char()->WrCB(Char(), e.data(), 0, 20);
	BtDfuSmpReset();
	Write(Echo("clean"), 247);
	Drain();
	CHECK(TakeRsp(rsp) && EchoOk(rsp, "clean"));
}

// Upload, test, reset through the transport, then the boot installs it.
static void TestUpload(const DfuBootCfg_t &Boot)
{
	std::vector<uint8_t> img = HostReadFile(s_Dir + "/v2.bin");
	std::vector<uint8_t> rsp;
	uint32_t mtu = 247;
	s_Peer.Conn.MaxMtu = (uint16_t)mtu;
	s_NotifyRefuseEvery = 3;

	// Largest chunk that keeps the request in the 1024 byte buffer.
	uint32_t chunk = 900;
	for (uint32_t off = 0; off < img.size(); )
	{
		uint32_t n = (uint32_t)img.size() - off < chunk ?
					 (uint32_t)img.size() - off : chunk;
		uint8_t b[1024];
		CborWr_t w;
		CborWrInit(&w, b, sizeof(b));
		CborPutMap(&w, off == 0 ? 3 : 2);
		if (off == 0)
		{
			CborPutStr(&w, "len");
			CborPutUint(&w, img.size());
		}
		CborPutStr(&w, "off");
		CborPutUint(&w, off);
		CborPutStr(&w, "data");
		CborPutBstr(&w, img.data() + off, n);
		CHECK(w.bOvf == false);
		Write(Packet(DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_UPLOAD,
					 std::vector<uint8_t>(b, b + w.Len)), mtu);
		Drain();
		CHECK(TakeRsp(rsp));
		CborFld_t f[] = { CBOR_FLD("off", CBOR_FLD_UINT) };
		CHECK(CborMapRead(rsp.data() + 8, (uint32_t)rsp.size() - 8, f, 1));
		CHECK(f[0].U == off + n);
		off = (uint32_t)f[0].U;
	}

	DfuMap_t m = { (uintptr_t)img.data(), (uint32_t)img.size() };
	DfuImgInfo_t info;
	CHECK(DfuImgParse(DfuMapRead, &m, m.Size, &info) == DFU_IMG_OK);
	uint8_t b[64];
	CborWr_t w;
	CborWrInit(&w, b, sizeof(b));
	CborPutMap(&w, 2);
	CborPutStr(&w, "hash");
	CborPutBstr(&w, info.Sha, 32);
	CborPutStr(&w, "confirm");
	CborPutBool(&w, false);
	Write(Packet(DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_STATE,
				 std::vector<uint8_t>(b, b + w.Len)), mtu);
	Drain();
	CHECK(TakeRsp(rsp) && rsp.size() > 100);	// the image list

	Write(Packet(DFUSMP_OP_WRITE, DFUSMP_GRP_OS, DFUSMP_OS_RESET, { 0xA0 }), mtu);
	Drain();
	CHECK(TakeRsp(rsp) && s_Resets == 1);

	bool started = false;
	if (setjmp(g_HostStartJmp) != 0)
	{
		started = true;
	}
	else
	{
		(void)DfuBootRun(Boot);
	}
	CHECK(started && g_HostStartAddr == HOST_SLOT0);
	CHECK(memcmp(HostFlash(HOST_SLOT0), img.data() + info.Hdr.HdrSize,
				 info.Hdr.ImgSize) == 0);
	CHECK(g_HostNorViolations == 0);
	s_NotifyRefuseEvery = 0;

	// An upload cut off by a disconnection does not leave the server busy:
	// erase works after the reconnection.
	{
		uint8_t b2[1024];
		CborWr_t w2;
		CborWrInit(&w2, b2, sizeof(b2));
		CborPutMap(&w2, 3);
		CborPutStr(&w2, "len");
		CborPutUint(&w2, img.size());
		CborPutStr(&w2, "off");
		CborPutUint(&w2, 0);
		CborPutStr(&w2, "data");
		CborPutBstr(&w2, img.data(), 600);
		Write(Packet(DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_UPLOAD,
					 std::vector<uint8_t>(b2, b2 + w2.Len)), mtu);
		Drain();
		CHECK(TakeRsp(rsp));
		BtDfuSmpReset();
		Write(Packet(DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_ERASE, { 0xA0 }),
			  mtu);
		Drain();
		CHECK(TakeRsp(rsp) && rsp.size() == 9 && rsp[8] == 0xA0);
	}
}

int main(int argc, char **argv)
{
	s_Dir = argc > 1 ? argv[1] : "build/img";

	HostFlashInit(HOST_MEM_NOR);

	memset(&s_Peer, 0, sizeof(s_Peer));
	s_Peer.Conn.Hdl = 1;

	static HostNvm slot1(HOST_SLOT1, HOST_SLOT1_SIZE);
	static DfuStore_t st;
	static DfuSmp mgr;
	HashEngine *sha = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem));
	CHECK(DfuStoreNvm(&st, &slot1));
	static DfuMgr s_DfuMgr;
	DfuMgrCfg_t dmcfg = {
		.pStore = &st,
		.bDirect = false,
		.bManifestOnly = false,
		.bAllowDowngrade = true,
		.pHash = sha,
		.pBoot = nullptr,
	};
	(void)s_DfuMgr.Init(dmcfg);
	DfuSmpCfg_t mcfg = {
		.pMgr = &s_DfuMgr,
		.BufSize = sizeof(s_SmpRx),
		.ResetCB = ResetCB,
		.pCtx = nullptr,
	};
	CHECK(mgr.Init(mcfg));

	BtDfuSmpCfg_t cfg = {
		.pMgr = &mgr,
		.pRxBuf = s_SmpRx,
		.RxBufSize = sizeof(s_SmpRx),
		.pTxBuf = s_SmpTx,
		.TxBufSize = sizeof(s_SmpTx),
		.SecType = BT_GAP_SECTYPE_NONE,
	};
	CHECK(BtDfuSmpInit(cfg));
	CHECK(s_pSrvc == BtDfuSmpSrvc() && s_pSrvc->bCustom);

	// The UUIDs as the host tools look for them.
	BtGattChar_t *c = Char();
	CHECK(s_pSrvc->UuidSrvc == 0xDC1D && s_pSrvc->UuidBase[15] == 0x8D &&
		  s_pSrvc->UuidBase[0] == 0x84);
	CHECK(c->Uuid == 0x7828 && c->pUuidBase != nullptr &&
		  c->pUuidBase[15] == 0xDA && c->pUuidBase[0] == 0x48);
	CHECK(c->Property == (BT_GATT_CHAR_PROP_WRITE_WORESP |
						  BT_GATT_CHAR_PROP_NOTIFY));

	TestTransport();

	std::vector<uint8_t> k = HostReadFile(s_Dir + "/key1.der");
	DfuImgKey_t key = { k.data(), (uint32_t)k.size() };
	DfuBootCfg_t boot = {
		.pHash = sha,
		.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr),
		.pKey = &key,
		.NbKey = 1,
		.bVerifySlot0 = true,
		.bAllowNoRec = false,
	};
	TestUpload(boot);

	printf("bt_dfu_smp_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail != 0;
}
