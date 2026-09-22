// DFU wire protocol on simulated memory: every operation and its errors,
// frames damaged, oversized, escaped, cut by garbage or fed a byte at a time,
// a manifest in pieces, resends, and an install that stage 0 then starts.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "crc.h"
#include "dfu/dfu_wire.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

static int s_Fail = 0;
static int s_Pass = 0;

#define CHECK(c) do { if (c) { s_Pass++; } else { s_Fail++; \
	fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)

static std::string s_Dir;

// The verify only engines stage 0 uses.
alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_HASH_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_VERIFY_MEMSIZE];
static HashEngine *s_Sha;
static std::vector<uint8_t> s_Key1;
static DfuImgKey_t s_Key;
static DfuBootCfg_t s_BootCfg;

#define MAXBODY		512

static uint8_t s_Rx[DFU_WIRE_RXBUF_SIZE(MAXBODY)];

class LoopIntrf : public HostByteIntrf {
public:
	std::vector<uint8_t> vOut;
	std::vector<uint8_t> vIn;
	size_t vInPos = 0;
	int vTxMax = 1 << 30;

	int RxBytes(uint8_t *p, int Len) override {
		int n = 0;
		while (n < Len && vInPos < vIn.size())
		{
			p[n++] = vIn[vInPos++];
		}
		return n;
	}
	int TxBytes(const uint8_t *p, int Len) override {
		int n = Len < vTxMax ? Len : vTxMax;
		vOut.insert(vOut.end(), p, p + n);
		return n;
	}
};

// Bytes arriving on the link, then the protocol served.
static bool Push(DfuWire &W, LoopIntrf &L, const uint8_t *p, size_t n)
{
	L.vIn.assign(p, p + n);
	L.vInPos = 0;
	return W.Poll();
}

static std::vector<uint8_t> Img(const char *pName)
{
	return HostReadFile(s_Dir + "/" + pName);
}

static void Put32(std::vector<uint8_t> &v, uint32_t x)
{
	for (int i = 0; i < 4; i++) v.push_back((uint8_t)(x >> (8 * i)));
}

static uint32_t Get32(const uint8_t *p)
{
	return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

// A host frame: SLIP around Op, Seq, Body, CRC.
static std::vector<uint8_t> Frame(uint8_t Op, uint8_t Seq,
								  const std::vector<uint8_t> &Body,
								  bool bBadCrc = false)
{
	std::vector<uint8_t> raw = { Op, Seq };
	raw.insert(raw.end(), Body.begin(), Body.end());
	uint32_t c = crc32_ieee(raw.data(), (int)raw.size());
	Put32(raw, bBadCrc ? c ^ 1 : c);

	std::vector<uint8_t> f = { DFU_WIRE_END };
	for (uint8_t b : raw)
	{
		if (b == DFU_WIRE_END) { f.push_back(DFU_WIRE_ESC); f.push_back(DFU_WIRE_ESC_END); }
		else if (b == DFU_WIRE_ESC) { f.push_back(DFU_WIRE_ESC); f.push_back(DFU_WIRE_ESC_ESC); }
		else f.push_back(b);
	}
	f.push_back(DFU_WIRE_END);

	return f;
}

struct Resp {
	int Op = -1;
	int Seq = -1;
	std::vector<uint8_t> Body;
};

// Responses in what the device sent, checked as a host would.
static std::vector<Resp> Decode(const std::vector<uint8_t> &Out)
{
	std::vector<Resp> r;
	std::vector<uint8_t> cur;
	bool in = false, esc = false;

	for (uint8_t b : Out)
	{
		if (b == DFU_WIRE_END)
		{
			if (in && cur.size() >= 6)
			{
				uint32_t c = crc32_ieee(cur.data(), (int)cur.size() - 4);
				CHECK(c == Get32(&cur[cur.size() - 4]));
				Resp x;
				x.Op = cur[0];
				x.Seq = cur[1];
				x.Body.assign(cur.begin() + 2, cur.end() - 4);
				r.push_back(x);
			}
			cur.clear();
			in = true;
			esc = false;
			continue;
		}
		if (esc)
		{
			cur.push_back(b == DFU_WIRE_ESC_END ? DFU_WIRE_END : DFU_WIRE_ESC);
			esc = false;
		}
		else if (b == DFU_WIRE_ESC)
		{
			esc = true;
		}
		else
		{
			cur.push_back(b);
		}
	}

	return r;
}

static uint8_t s_Seq = 0;

// One request, one response expected.
static Resp Req(DfuWire &W, LoopIntrf &L, uint8_t Op,
				const std::vector<uint8_t> &Body = {})
{
	L.vOut.clear();
	std::vector<uint8_t> f = Frame(Op, ++s_Seq, Body);
	Push(W, L, f.data(), f.size());

	std::vector<Resp> r = Decode(L.vOut);
	CHECK(r.size() == 1);
	if (r.size() != 1)
	{
		return Resp();
	}
	CHECK(r[0].Op == (Op | DFU_WIRE_OP_RSP));
	CHECK(r[0].Seq == s_Seq);
	CHECK(r[0].Body.size() >= 1);

	return r[0];
}

static int St(const Resp &R)
{
	return R.Body.empty() ? -1 : R.Body[0];
}

struct Split {
	std::vector<uint8_t> Man;
	std::vector<uint8_t> Pay;
};

static Split SplitImg(const std::vector<uint8_t> &F)
{
	DfuImgHdr_t h;
	memcpy(&h, F.data(), sizeof(h));
	Split s;
	s.Man.assign(F.begin(), F.begin() + h.HdrSize);
	s.Man.insert(s.Man.end(), F.begin() + h.HdrSize + h.ImgSize, F.end());
	s.Pay.assign(F.begin() + h.HdrSize, F.begin() + h.HdrSize + h.ImgSize);
	return s;
}

static int Begin(DfuWire &W, LoopIntrf &L, const std::vector<uint8_t> &Man,
				 uint32_t Piece)
{
	int st = -1;

	for (uint32_t off = 0; off < Man.size(); off += Piece)
	{
		uint32_t l = Man.size() - off < Piece ? (uint32_t)(Man.size() - off) : Piece;
		std::vector<uint8_t> b = { (uint8_t)Man.size(), (uint8_t)(Man.size() >> 8),
								   (uint8_t)off, (uint8_t)(off >> 8) };
		b.insert(b.end(), Man.begin() + off, Man.begin() + off + l);
		st = St(Req(W, L, DFU_WIRE_OP_BEGIN, b));
		if (st != DFU_WIRE_OK)
		{
			break;
		}
	}

	return st;
}

static Resp Write(DfuWire &W, LoopIntrf &L, uint32_t Off, const uint8_t *p,
				  uint32_t Len)
{
	std::vector<uint8_t> b;
	Put32(b, Off);
	b.insert(b.end(), p, p + Len);
	return Req(W, L, DFU_WIRE_OP_WRITE, b);
}

static uintptr_t Reboot(void)
{
	if (setjmp(g_HostStartJmp) != 0)
	{
		return g_HostStartAddr;
	}
	(void)DfuBootRun(s_BootCfg);
	return 0;
}

static int s_ResetCnt = 0;
static bool s_ResetRec = false;
static void ResetCB(bool bRecovery)
{
	s_ResetCnt++;
	s_ResetRec = bRecovery;
}

static int Upload(DfuWire &W, LoopIntrf &L, const Split &S, std::mt19937 &Rng)
{
	int st = Begin(W, L, S.Man, 64 + Rng() % (MAXBODY - 63));
	if (st != DFU_WIRE_OK)
	{
		return st;
	}

	uint32_t off = 0;
	while (off < S.Pay.size())
	{
		uint32_t l = 1 + Rng() % MAXBODY;
		if (l > S.Pay.size() - off) l = (uint32_t)(S.Pay.size() - off);
		Resp r = Write(W, L, off, S.Pay.data() + off, l);
		if (St(r) != DFU_WIRE_OK)
		{
			return St(r);
		}
		CHECK(r.Body.size() == 9);
		off += l;
		CHECK(Get32(&r.Body[1]) == off);
		CHECK(Get32(&r.Body[5]) == crc32_ieee((uint8_t *)S.Pay.data(), (int)off));
		if (Rng() % 7 == 0)
		{
			// A resend of the last piece, as after a lost response.
			Resp d = Write(W, L, off - l, S.Pay.data() + off - l, l);
			CHECK(St(d) == DFU_WIRE_OK && Get32(&d.Body[1]) == off);
		}
	}

	return St(Req(W, L, DFU_WIRE_OP_FINISH));
}

static void Test(HostMem Kind)
{
	HostFlashInit(Kind);
	printf("wire %s\n", HostMemName(Kind));

	DfuStore_t st;
	CHECK(DfuStoreTgt(&st, HOST_SLOT0, HOST_SLOT0_SIZE));

	DfuMgrCfg_t mc = {
		.pStore = &st, .bDirect = true, .bManifestOnly = true,
		.bAllowDowngrade = false, .pHash = s_Sha, .pBoot = &s_BootCfg,
	};
	DfuMgr mgr;
	CHECK(mgr.Init(mc));

	LoopIntrf loop;
	static const uint8_t devid[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	DfuWireCfg_t wc = {
		.pMgr = &mgr, .pIntrf = &loop, .pRxBuf = s_Rx,
		.RxBufSize = sizeof(s_Rx), .MaxBody = MAXBODY, .EraseMaxMs = 90,
		.BootVer = 0x01020304, .pDevId = devid, .Reset = ResetCB,
	};
	DfuWire w;

	DfuWireCfg_t bad = wc;
	bad.RxBufSize = 100;
	CHECK(w.Init(bad) == false);
	bad = wc;
	bad.MaxBody = 500;
	CHECK(w.Init(bad) == false);
	CHECK(w.Init(wc));

	std::mt19937 rng(300 + (unsigned)Kind);

	// INFO
	Resp r = Req(w, loop, DFU_WIRE_OP_INFO);
	CHECK(St(r) == DFU_WIRE_OK);
	CHECK(r.Body.size() == DFU_WIRE_INFO_LEN);
	if (r.Body.size() == DFU_WIRE_INFO_LEN)
	{
		const uint8_t *b = r.Body.data();
		CHECK(b[1] == DFU_WIRE_PROTO_VER);
		CHECK(b[2] == DFU_WIRE_STATE_IDLE);
		CHECK(b[3] == 0);
		CHECK((b[4] | (b[5] << 8)) == MAXBODY);
		CHECK((b[6] | (b[7] << 8)) == (int)HostUnit());
		CHECK((b[8] | (b[9] << 8)) == 90);
		CHECK((b[10] | (b[11] << 8)) == DFU_MGR_MANIFEST_MAX);
		CHECK(Get32(b + 12) == mgr.MaxLen());
		CHECK(b[20] == 0);
		CHECK(memcmp(b + 28, devid, 8) == 0);
		CHECK(Get32(b + 36) == 0x01020304);
	}
	CHECK(St(Req(w, loop, DFU_WIRE_OP_INFO, { 0 })) == DFU_WIRE_ERR_LEN);

	Split s1 = SplitImg(Img("v1.bin")), s2 = SplitImg(Img("v2.bin")),
		  sb = SplitImg(Img("v1_badsig.bin")), sk = SplitImg(Img("v2_key2.bin"));

	// Bad manifests: status, and no memory touched.
	int ops = g_HostMemOps;
	CHECK(Begin(w, loop, sb.Man, 1000) == DFU_WIRE_ERR_SIG);
	CHECK(Begin(w, loop, sk.Man, 1000) == DFU_WIRE_ERR_KEY);
	{
		std::vector<uint8_t> m = s1.Man;
		m[0] ^= 1;
		CHECK(Begin(w, loop, m, 1000) == DFU_WIRE_ERR_HDR);
	}
	CHECK(g_HostMemOps == ops);

	// BEGIN pieces out of order or inconsistent.
	{
		std::vector<uint8_t> b = { (uint8_t)s1.Man.size(), (uint8_t)(s1.Man.size() >> 8),
								   16, 0, 1, 2, 3 };
		CHECK(St(Req(w, loop, DFU_WIRE_OP_BEGIN, b)) == DFU_WIRE_ERR_LEN);
		CHECK(St(Req(w, loop, DFU_WIRE_OP_BEGIN, { 1, 0 })) == DFU_WIRE_ERR_LEN);
		std::vector<uint8_t> big = { 0xFF, 0xFF, 0, 0, 1 };
		CHECK(St(Req(w, loop, DFU_WIRE_OP_BEGIN, big)) == DFU_WIRE_ERR_LEN);
	}

	// Operations out of place.
	CHECK(St(Write(w, loop, 0, s1.Pay.data(), 16)) == DFU_WIRE_ERR_STATE);
	CHECK(St(Req(w, loop, DFU_WIRE_OP_FINISH)) == DFU_WIRE_ERR_STATE);
	CHECK(St(Req(w, loop, DFU_WIRE_OP_FINISH, { 0 })) == DFU_WIRE_ERR_LEN);
	CHECK(St(Req(w, loop, DFU_WIRE_OP_RESET)) == DFU_WIRE_ERR_LEN);
	CHECK(g_HostMemOps == ops);

	// v1, manifest in small pieces.
	CHECK(Begin(w, loop, s1.Man, 17) == DFU_WIRE_OK);
	r = Req(w, loop, DFU_WIRE_OP_INFO);
	CHECK(r.Body.size() > 2 && r.Body[2] == DFU_WIRE_STATE_RECV);

	// Offset rules.
	r = Write(w, loop, 8, s1.Pay.data() + 8, 8);
	CHECK(St(r) == DFU_WIRE_ERR_OFFSET && Get32(&r.Body[1]) == 0);
	{
		// Longer than the receive buffer: dropped, no response.
		std::vector<uint8_t> b;
		Put32(b, 0);
		b.insert(b.end(), s1.Pay.data(), s1.Pay.data() + MAXBODY + 1);
		std::vector<uint8_t> f = Frame(DFU_WIRE_OP_WRITE, 1, b);
		loop.vOut.clear();
		Push(w, loop, f.data(), f.size());
		CHECK(loop.vOut.empty());
	}
	CHECK(St(Req(w, loop, DFU_WIRE_OP_WRITE, { 0, 0, 0, 0 })) == DFU_WIRE_ERR_LEN);
	CHECK(St(Req(w, loop, DFU_WIRE_OP_ABORT)) == DFU_WIRE_OK);

	CHECK(Upload(w, loop, s1, rng) == DFU_WIRE_OK);
	// A FINISH resent after its response was lost.
	CHECK(St(Req(w, loop, DFU_WIRE_OP_FINISH)) == DFU_WIRE_OK);
	r = Req(w, loop, DFU_WIRE_OP_INFO);
	CHECK(r.Body.size() > 20 && r.Body[2] == DFU_WIRE_STATE_DONE && r.Body[20] == 1);
	CHECK(Reboot() == HOST_SLOT0);

	// RESET: response first, then the callback.
	s_ResetCnt = 0;
	CHECK(St(Req(w, loop, DFU_WIRE_OP_RESET, { 1 })) == DFU_WIRE_OK);
	CHECK(s_ResetCnt == 1 && s_ResetRec == false);
	CHECK(St(Req(w, loop, DFU_WIRE_OP_RESET, { 0 })) == DFU_WIRE_OK);
	CHECK(s_ResetCnt == 2 && s_ResetRec == true);
	CHECK(St(Req(w, loop, DFU_WIRE_OP_ENTER)) == DFU_WIRE_OK);
	CHECK(s_ResetCnt == 3 && s_ResetRec == true);

	// Older image refused.
	Split s0 = SplitImg(Img("v1.bin"));
	CHECK(Upload(w, loop, s2, rng) == DFU_WIRE_OK);
	CHECK(Reboot() == HOST_SLOT0);
	ops = g_HostMemOps;
	CHECK(Begin(w, loop, s0.Man, 1000) == DFU_WIRE_ERR_VERSION);
	CHECK(g_HostMemOps == ops);

	// Tampered payload.
	{
		Split t = SplitImg(Img("v3.bin"));
		t.Pay[100] ^= 0x80;
		CHECK(Upload(w, loop, t, rng) == DFU_WIRE_ERR_HASH);
		CHECK(Reboot() == 0);
	}

	// Damaged and stray bytes: dropped, no response; the next frame served.
	{
		uint32_t d0 = w.Dropped();
		loop.vOut.clear();
		std::vector<uint8_t> f = Frame(DFU_WIRE_OP_INFO, 9, {}, true);
		Push(w, loop, f.data(), f.size());
		f = Frame(0x55, 9, {});
		Push(w, loop, f.data(), f.size());
		const uint8_t junk[] = "boot text\r\n\xDB\x01\xC0\xC0";
		Push(w, loop, junk, sizeof(junk) - 1);
		std::vector<uint8_t> big(sizeof(s_Rx) + 10, 0x11);
		big.insert(big.begin(), DFU_WIRE_END);
		big.push_back(DFU_WIRE_END);
		Push(w, loop, big.data(), big.size());
		CHECK(loop.vOut.empty());
		CHECK(w.Dropped() >= d0 + 4);

		// Fed one byte at a time, with junk before that ends in an escape.
		// As in RFC 1055, the byte after an escape is data even when it is
		// END, so the first frame joins the junk and is dropped; the host
		// sends it again and that one is served.
		loop.vOut.clear();
		f = Frame(DFU_WIRE_OP_INFO, 77, {});
		std::vector<uint8_t> all = { 'x', 'y', DFU_WIRE_ESC };
		all.insert(all.end(), f.begin(), f.end());
		all.insert(all.end(), f.begin(), f.end());
		for (uint8_t b : all)
		{
			Push(w, loop, &b, 1);
		}
		std::vector<Resp> rs = Decode(loop.vOut);
		CHECK(rs.size() == 1 && rs[0].Seq == 77);

		// A frame whose decoded last byte is the escape code, then one more:
		// the byte left in the buffer is not taken for an escape.
		loop.vOut.clear();
		std::vector<uint8_t> esc0 = { DFU_WIRE_ESC, DFU_WIRE_ESC_ESC,
									  DFU_WIRE_END };
		Push(w, loop, esc0.data(), esc0.size());
		f = Frame(DFU_WIRE_OP_INFO, 78, {});
		Push(w, loop, f.data(), f.size());
		rs = Decode(loop.vOut);
		CHECK(rs.size() == 1 && rs[0].Seq == 78);
	}

	// A payload full of the framing bytes, over Poll, a byte per Tx.
	{
		Split t = SplitImg(Img("v3.bin"));
		loop.vTxMax = 1;
		CHECK(Upload(w, loop, t, rng) == DFU_WIRE_OK);
		loop.vTxMax = 1 << 30;
		CHECK(Reboot() == HOST_SLOT0);

		loop.vOut.clear();
		std::vector<uint8_t> f = Frame(DFU_WIRE_OP_INFO, 5, {});
		loop.vIn = f;
		loop.vInPos = 0;
		CHECK(w.Poll());
		CHECK(Decode(loop.vOut).size() == 1);
	}

	// SetIntrf: the upload goes on over another link.
	{
		LoopIntrf l2;
		Split t = SplitImg(Img("v4.bin"));
		CHECK(Begin(w, loop, t.Man, 1000) == DFU_WIRE_OK);
		CHECK(St(Write(w, loop, 0, t.Pay.data(), 256)) == DFU_WIRE_OK);
		w.SetIntrf(&l2);
		Resp x = Req(w, l2, DFU_WIRE_OP_INFO);
		CHECK(x.Body.size() > 20 && x.Body[2] == DFU_WIRE_STATE_RECV &&
			  Get32(&x.Body[16]) == 256);
		for (uint32_t off = 256; off < t.Pay.size(); off += MAXBODY)
		{
			uint32_t l = t.Pay.size() - off < MAXBODY ? (uint32_t)(t.Pay.size() - off)
													  : MAXBODY;
			CHECK(St(Write(w, l2, off, t.Pay.data() + off, l)) == DFU_WIRE_OK);
		}
		CHECK(St(Req(w, l2, DFU_WIRE_OP_FINISH)) == DFU_WIRE_OK);
		CHECK(Reboot() == HOST_SLOT0);
		w.SetIntrf(&loop);
	}
}

// Random frames at the protocol: nothing may crash, and without a manifest
// signed by the boot key not one memory operation may happen.
static void Fuzz(HostMem Kind)
{
	HostFlashInit(Kind);
	printf("fuzz %s\n", HostMemName(Kind));

	DfuStore_t st;
	CHECK(DfuStoreTgt(&st, HOST_SLOT0, HOST_SLOT0_SIZE));
	DfuMgrCfg_t mc = {
		.pStore = &st, .bDirect = true, .bManifestOnly = true,
		.bAllowDowngrade = true, .pHash = s_Sha, .pBoot = &s_BootCfg,
	};
	DfuMgr mgr;
	CHECK(mgr.Init(mc));
	LoopIntrf loop;
	DfuWireCfg_t wc = {
		.pMgr = &mgr, .pIntrf = &loop, .pRxBuf = s_Rx,
		.RxBufSize = sizeof(s_Rx), .MaxBody = MAXBODY, .EraseMaxMs = 90,
		.BootVer = 1, .pDevId = nullptr, .Reset = ResetCB,
	};
	DfuWire w;
	CHECK(w.Init(wc));

	std::mt19937 rng(900 + (unsigned)Kind);
	Split s1 = SplitImg(Img("v1.bin"));
	int ops = g_HostMemOps;

	for (int i = 0; i < 200000; i++)
	{
		std::vector<uint8_t> body(rng() % 64 == 0 ? rng() % 1200 : rng() % 48);
		for (auto &b : body)
		{
			b = (uint8_t)rng();
		}
		uint8_t op = (uint8_t)(1 + rng() % 8);
		if (op == DFU_WIRE_OP_BEGIN && body.size() >= 4 && rng() % 2)
		{
			// Often a manifest shaped one: right lengths, right magic,
			// random signature.
			std::vector<uint8_t> m = s1.Man;
			for (size_t k = 0; k < 1 + rng() % 8; k++)
			{
				m[rng() % m.size()] ^= (uint8_t)(1 + rng() % 255);
			}
			// Keep the signature bytes random, the rest often intact.
			for (size_t k = m.size() - 64; k < m.size(); k++)
			{
				m[k] = (uint8_t)rng();
			}
			body = { (uint8_t)m.size(), (uint8_t)(m.size() >> 8), 0, 0 };
			body.insert(body.end(), m.begin(), m.end());
		}
		std::vector<uint8_t> f = Frame(op == 8 ? (uint8_t)rng() : op,
									   (uint8_t)rng(), body, rng() % 16 == 0);
		if (rng() % 8 == 0)
		{
			// Damage the frame itself.
			f[rng() % f.size()] = (uint8_t)rng();
		}
		loop.vOut.clear();
		Push(w, loop, f.data(), f.size());
	}
	CHECK(g_HostMemOps == ops);
	CHECK(Reboot() == 0);

	// A genuine manifest with its header changed anywhere: the change is
	// found at FINISH at the latest, and nothing ever starts.
	Split s2 = SplitImg(Img("v2.bin"));
	DfuImgHdr_t h;
	memcpy(&h, s2.Man.data(), sizeof(h));
	for (int i = 0; i < 40; i++)
	{
		Split t = s2;
		size_t at = rng() % h.HdrSize;
		t.Man[at] ^= (uint8_t)(1 + rng() % 255);
		int res = Upload(w, loop, t, rng);
		CHECK(res != DFU_WIRE_OK);
		CHECK(Reboot() == 0);
	}
	CHECK(Upload(w, loop, s2, rng) == DFU_WIRE_OK);
	CHECK(Reboot() == HOST_SLOT0);
}

int main(int argc, char **argv)
{
	s_Dir = argc > 1 ? argv[1] : "build/img";

	s_Sha = CryptoSoftSha256HashCreate(s_ShaMem, sizeof(s_ShaMem));
	s_Key1 = Img("key1.der");
	s_Key = { s_Key1.data(), (uint32_t)s_Key1.size() };
	s_BootCfg.pHash = s_Sha;
	s_BootCfg.pSign = CryptoUeccVerifyCreate(s_EccMem, sizeof(s_EccMem));
	s_BootCfg.pKey = &s_Key;
	s_BootCfg.NbKey = 1;
	s_BootCfg.bVerifySlot0 = true;
	s_BootCfg.bAllowNoRec = false;

	const HostMem kinds[] = {
		HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16, HOST_MEM_BIG256,
		HOST_MEM_SECT,
	};
	for (HostMem k : kinds)
	{
		Test(k);
	}
	Fuzz(HOST_MEM_NOR);
	Fuzz(HOST_MEM_BIG256);

	CHECK(g_HostNorViolations == 0);

	printf("dfu_wire_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail ? 1 : 0;
}
