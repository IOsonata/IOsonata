// SMP serial transport: framing both ways, console text on the same line,
// damaged and oversized packets, a packet fed one byte at a time, a reset
// done only after its response is out.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "dfu/dfu_smp.h"
#include "dfu/dfu_serial.h"
#include "crypto/crypto_softsha256.h"

static int s_Fail = 0;
static int s_Pass = 0;

#define CHECK(c) do { if (c) { s_Pass++; } else { s_Fail++; \
	fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];

// What the transport sends, and what it is given to read.
class LoopIntrf : public DeviceIntrf {
public:
	std::vector<uint8_t> vOut;
	std::vector<uint8_t> vIn;
	size_t vInPos = 0;
	int vTxMax = 1 << 30;		// bytes one Tx call takes
	DevIntrf_t vDev = {};

	operator DevIntrf_t * () override { return &vDev; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate(void) override { return 0; }
	int Rx(uint32_t, uint8_t *p, int Len) override {
		int n = 0;
		while (n < Len && vInPos < vIn.size())
		{
			p[n++] = vIn[vInPos++];
		}
		return n;
	}
	int Tx(uint32_t, const uint8_t *p, int Len) override {
		int n = Len < vTxMax ? Len : vTxMax;
		vOut.insert(vOut.end(), p, p + n);
		return n;
	}
};

static const char s_B64[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// The frames a host sends for one SMP packet, LineMax bytes per line at most.
static std::vector<uint8_t> Frame(const std::vector<uint8_t> &Pkt,
								  size_t LineMax = 127, bool bBadCrc = false)
{
	std::vector<uint8_t> b;
	uint16_t crc = DfuSerialCrc16(0, Pkt.data(), (uint32_t)Pkt.size());
	if (bBadCrc)
	{
		crc ^= 1;
	}
	size_t l = Pkt.size() + 2;
	b.push_back((uint8_t)(l >> 8));
	b.push_back((uint8_t)l);
	b.insert(b.end(), Pkt.begin(), Pkt.end());
	b.push_back((uint8_t)(crc >> 8));
	b.push_back((uint8_t)crc);

	std::string s;
	for (size_t i = 0; i < b.size(); i += 3)
	{
		uint32_t v = (uint32_t)b[i] << 16;
		size_t n = b.size() - i < 3 ? b.size() - i : 3;
		if (n > 1) v |= (uint32_t)b[i + 1] << 8;
		if (n > 2) v |= b[i + 2];
		s += s_B64[(v >> 18) & 63];
		s += s_B64[(v >> 12) & 63];
		s += n > 1 ? s_B64[(v >> 6) & 63] : '=';
		s += n > 2 ? s_B64[v & 63] : '=';
	}

	std::vector<uint8_t> out;
	size_t per = (LineMax - 3) / 4 * 4;
	for (size_t i = 0; i < s.size(); i += per)
	{
		out.push_back(i == 0 ? 6 : 4);
		out.push_back(i == 0 ? 9 : 20);
		size_t n = s.size() - i < per ? s.size() - i : per;
		out.insert(out.end(), s.begin() + i, s.begin() + i + n);
		out.push_back('\n');
	}
	return out;
}

// Decode what the transport sent back into SMP packets, checking each frame.
static std::vector<std::vector<uint8_t>> Unframe(const std::vector<uint8_t> &In)
{
	std::vector<std::vector<uint8_t>> pkts;
	std::vector<uint8_t> cur;
	std::string b64;
	size_t i = 0;

	while (i < In.size())
	{
		size_t e = i;
		while (e < In.size() && In[e] != '\n')
		{
			e++;
		}
		CHECK(e < In.size());
		CHECK(e - i + 1 <= 127);
		bool first = In[i] == 6 && In[i + 1] == 9;
		bool cont = In[i] == 4 && In[i + 1] == 20;
		CHECK(first || cont);
		if (first)
		{
			b64.clear();
		}
		b64.append((const char *)&In[i + 2], e - i - 2);
		CHECK(b64.size() % 4 == 0);

		std::vector<uint8_t> d;
		for (size_t k = 0; k + 3 < b64.size(); k += 4)
		{
			uint32_t v = 0;
			int pad = 0;
			for (int j = 0; j < 4; j++)
			{
				const char *p = strchr(s_B64, b64[k + j]);
				if (b64[k + j] == '=')
				{
					pad++;
					v <<= 6;
				}
				else
				{
					v = (v << 6) | (uint32_t)(p - s_B64);
				}
			}
			d.push_back((uint8_t)(v >> 16));
			if (pad < 2) d.push_back((uint8_t)(v >> 8));
			if (pad < 1) d.push_back((uint8_t)v);
		}
		if (d.size() >= 2 && d.size() >= (size_t)((d[0] << 8) | d[1]) + 2)
		{
			size_t l = (d[0] << 8) | d[1];
			std::vector<uint8_t> pkt(d.begin() + 2, d.begin() + l);
			uint16_t crc = (uint16_t)((d[l] << 8) | d[l + 1]);
			CHECK(crc == DfuSerialCrc16(0, pkt.data(), (uint32_t)pkt.size()));
			CHECK(d.size() == l + 2);
			pkts.push_back(pkt);
			b64.clear();
		}
		i = e + 1;
	}
	return pkts;
}

static std::vector<uint8_t> Echo(const char *s, uint8_t Seq)
{
	uint8_t body[1024];
	CborWr_t w;
	CborWrInit(&w, body, sizeof(body));
	CborPutMap(&w, 1);
	CborPutStr(&w, "d");
	CborPutStr(&w, s);
	std::vector<uint8_t> p = {
		DFUSMP_OP_WRITE | (1 << 3), 0, (uint8_t)(w.Len >> 8), (uint8_t)w.Len,
		0, DFUSMP_GRP_OS, Seq, DFUSMP_OS_ECHO,
	};
	p.insert(p.end(), body, body + w.Len);
	return p;
}

static std::string EchoOf(const std::vector<uint8_t> &Rsp)
{
	if (Rsp.size() < 8)
	{
		return "";
	}
	CborFld_t f[] = { CBOR_FLD("r", CBOR_FLD_TSTR) };
	if (CborMapRead(Rsp.data() + 8, (uint32_t)(Rsp.size() - 8), f, 1) == false ||
		f[0].bFound == false)
	{
		return "";
	}
	return std::string((const char *)f[0].S.p, f[0].S.Len);
}

static int s_Resets;
static bool s_bOutAtReset;
static LoopIntrf *s_pLoop;

static void Reset(void)
{
	s_Resets++;
	// The response is out before this.
	s_bOutAtReset = s_pLoop->vOut.size() > 0;
}

int main(int argc, char **argv)
{
	(void)argc;
	(void)argv;

	// CRC-16/XMODEM check value.
	CHECK(DfuSerialCrc16(0, (const uint8_t *)"123456789", 9) == 0x31C3);

	HostFlashInit(HOST_MEM_NOR);
	DfuLayout_t lay = HostLayout();
	static DfuStore_t st;
	static HostNvm slot1(HOST_SLOT1, HOST_SLOT1_SIZE);
	CHECK(DfuStoreNvm(&st, &slot1));

	static LoopIntrf loop;
	static DfuSmp mgr;
	static DfuSerial ser;
	static uint8_t rx[DFU_SERIAL_RXBUF_SIZE(512)], tx[DFU_SERIAL_TXBUF_SIZE];
	s_pLoop = &loop;

	static DfuMgr s_DfuMgr;
	DfuMgrCfg_t dmcfg = {
		.pStore = &st,
		.bDirect = false,
		.bManifestOnly = false,
		.bAllowDowngrade = true,
		.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem)),
		.pBoot = nullptr,
	};
	(void)s_DfuMgr.Init(dmcfg);
	DfuSmpCfg_t mcfg = {
		.pMgr = &s_DfuMgr,
		.BufSize = 512,
		.ResetCB = DfuSerial::ResetCB,
		.pCtx = &ser,
	};
	DfuSerialCfg_t scfg = {
		.pMgr = &mgr,
		.pIntrf = &loop,
		.pRxBuf = rx,
		.RxBufSize = sizeof(rx),
		.pTxBuf = tx,
		.TxBufSize = sizeof(tx),
		.Reset = Reset,
		.TxRetry = 0,
	};
	CHECK(mgr.Init(mcfg));
	CHECK(ser.Init(scfg));
	(void)lay;

	// One packet in one frame.
	std::vector<uint8_t> f = Frame(Echo("hello", 1));
	CHECK(ser.Feed(f.data(), (uint32_t)f.size()));
	auto r = Unframe(loop.vOut);
	CHECK(r.size() == 1 && EchoOf(r[0]) == "hello" && r[0][6] == 1);
	loop.vOut.clear();

	// A long one, many frames each way, with console text between packets
	// and lines of other traffic.
	std::string big(250, 'x');
	for (size_t i = 0; i < big.size(); i++)
	{
		big[i] = (char)('a' + i % 26);
	}
	std::vector<uint8_t> in;
	const char *con = "boot: console line\r\n\x06junk\n\x04\x14not in a packet\n";
	in.insert(in.end(), con, con + strlen(con));
	std::vector<uint8_t> f2 = Frame(Echo(big.c_str(), 2), 40);
	in.insert(in.end(), f2.begin(), f2.end());
	in.insert(in.end(), con, con + strlen(con));
	std::vector<uint8_t> f3 = Frame(Echo("third", 3));
	in.insert(in.end(), f3.begin(), f3.end());
	loop.vIn = in;
	loop.vInPos = 0;
	CHECK(ser.Poll());
	r = Unframe(loop.vOut);
	CHECK(r.size() == 2 && EchoOf(r[0]) == big && EchoOf(r[1]) == "third");
	CHECK(loop.vOut.size() > 2 * 127);
	loop.vOut.clear();

	// One byte at a time, and a transmit FIFO taking 7 bytes a call.
	loop.vTxMax = 7;
	std::vector<uint8_t> f4 = Frame(Echo(big.c_str(), 4), 127);
	bool served = false;
	for (uint8_t b : f4)
	{
		served |= ser.Feed(&b, 1);
	}
	CHECK(served);
	r = Unframe(loop.vOut);
	CHECK(r.size() == 1 && EchoOf(r[0]) == big);
	loop.vOut.clear();
	loop.vTxMax = 1 << 30;

	// Bad CRC: dropped, no answer. A good packet after it is served.
	uint32_t drop = ser.Dropped();
	std::vector<uint8_t> f5 = Frame(Echo("bad", 5), 127, true);
	CHECK(ser.Feed(f5.data(), (uint32_t)f5.size()) == false);
	CHECK(loop.vOut.empty() && ser.Dropped() == drop + 1);
	std::vector<uint8_t> f6 = Frame(Echo("good", 6));
	CHECK(ser.Feed(f6.data(), (uint32_t)f6.size()));
	r = Unframe(loop.vOut);
	CHECK(r.size() == 1 && EchoOf(r[0]) == "good");
	loop.vOut.clear();

	// A frame cut short by a new start frame: the first is dropped.
	std::vector<uint8_t> f7 = Frame(Echo(big.c_str(), 7), 40);
	std::vector<uint8_t> cut(f7.begin(), f7.begin() + 50);
	cut.push_back('\n');
	std::vector<uint8_t> f8 = Frame(Echo("after", 8));
	cut.insert(cut.end(), f8.begin(), f8.end());
	CHECK(ser.Feed(cut.data(), (uint32_t)cut.size()));
	r = Unframe(loop.vOut);
	CHECK(r.size() == 1 && EchoOf(r[0]) == "after");
	loop.vOut.clear();

	// Larger than the receive buffer: dropped whole, then served again.
	std::string huge(700, 'y');
	drop = ser.Dropped();
	std::vector<uint8_t> f9 = Frame(Echo(huge.c_str(), 9));
	CHECK(ser.Feed(f9.data(), (uint32_t)f9.size()) == false);
	CHECK(loop.vOut.empty() && ser.Dropped() > drop);
	CHECK(ser.Feed(f6.data(), (uint32_t)f6.size()));
	loop.vOut.clear();

	// Characters outside base64 inside a packet: dropped.
	std::vector<uint8_t> fa = Frame(Echo("abc", 10));
	fa[5] = '*';
	CHECK(ser.Feed(fa.data(), (uint32_t)fa.size()) == false);
	CHECK(loop.vOut.empty());

	// Reset: after the response.
	uint8_t hdr[8] = { DFUSMP_OP_WRITE | (1 << 3), 0, 0, 1, 0, DFUSMP_GRP_OS, 11,
					   DFUSMP_OS_RESET };
	std::vector<uint8_t> rp(hdr, hdr + 8);
	rp.push_back(0xA0);
	std::vector<uint8_t> fr = Frame(rp);
	CHECK(ser.Feed(fr.data(), (uint32_t)fr.size()));
	CHECK(s_Resets == 1 && s_bOutAtReset);
	r = Unframe(loop.vOut);
	CHECK(r.size() == 1 && r[0][7] == DFUSMP_OS_RESET);

	printf("dfu_serial_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail != 0;
}
