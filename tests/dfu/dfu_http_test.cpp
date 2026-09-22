// Image download over HTTP into slot 1: a whole image, a stream that breaks
// and resumes with a Range request, a server that ignores the range, the
// answers that are refused, a response head fed a byte at a time; then the
// boot installs what was downloaded.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "dfu/dfu_http.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

static int s_Fail = 0;
static int s_Pass = 0;

#define CHECK(c) do { if (c) { s_Pass++; } else { s_Fail++; \
	fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];
static std::string s_Dir;
static DfuBootCfg_t s_Boot;
static std::vector<uint8_t> s_KeyDer;
static DfuImgKey_t s_Key;

// A server behind a stream: answers the request it is sent, from a file.
class FakeServer : public DeviceIntrf {
public:
	std::vector<uint8_t> vFile;
	std::string vReq;
	std::vector<uint8_t> vOut;
	size_t vPos = 0;
	size_t vDropAt = (size_t)-1;	// stream ends after this many bytes
	size_t vMaxRx = 300;			// bytes per Rx call
	int vMode = 0;					// 0 normal, 1 ignore Range, 2 404,
									// 3 chunked, 4 bad range, 5 no length
	DevIntrf_t vDev = {};

	operator DevIntrf_t * () override { return &vDev; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate(void) override { return 0; }

	int Tx(uint32_t, const uint8_t *p, int Len) override {
		vReq.append((const char *)p, Len);
		if (vReq.find("\r\n\r\n") != std::string::npos && vOut.empty())
		{
			Answer();
		}
		return Len;
	}
	int Rx(uint32_t, uint8_t *p, int Len) override {
		size_t n = (size_t)Len < vMaxRx ? (size_t)Len : vMaxRx;
		if (vPos + n > vOut.size()) n = vOut.size() - vPos;
		if (vPos + n > vDropAt) n = vDropAt > vPos ? vDropAt - vPos : 0;
		memcpy(p, vOut.data() + vPos, n);
		vPos += n;
		return (int)n;
	}
	bool Ended() const { return vPos >= vOut.size() || vPos >= vDropAt; }

	void Answer() {
		size_t from = 0;
		size_t r = vReq.find("Range: bytes=");
		if (r != std::string::npos)
		{
			from = strtoul(vReq.c_str() + r + 13, nullptr, 10);
		}
		std::string h;
		std::vector<uint8_t> body;
		if (vMode == 2)
		{
			h = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
		}
		else if (vMode == 3)
		{
			h = "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n";
		}
		else if (from != 0 && vMode != 1)
		{
			size_t s = vMode == 4 ? from + 4 : from;
			body.assign(vFile.begin() + s, vFile.end());
			h = "HTTP/1.1 206 Partial Content\r\nContent-Range: bytes " +
				std::to_string(s) + "-" + std::to_string(vFile.size() - 1) +
				"/" + std::to_string(vFile.size()) + "\r\nContent-Length: " +
				std::to_string(body.size()) + "\r\n\r\n";
		}
		else
		{
			body = vFile;
			h = "HTTP/1.1 200 OK\r\nServer: test\r\nX-Long: " +
				std::string(300, 'z') + "\r\n" +
				(vMode == 5 ? "" : "content-LENGTH:   " +
				 std::to_string(body.size()) + "\r\n") + "\r\n";
		}
		vOut.assign(h.begin(), h.end());
		vOut.insert(vOut.end(), body.begin(), body.end());
	}
};

static std::vector<uint8_t> Img(const char *pName)
{
	return HostReadFile(s_Dir + "/" + pName);
}

// Poll until the stream has nothing more, then tell the client it ended.
static DFU_HTTP_STATE Run(DfuHttp &Http, FakeServer &Srv)
{
	for (int i = 0; i < 100000; i++)
	{
		DFU_HTTP_STATE s = Http.Poll();
		if (s != DFU_HTTP_HEADER && s != DFU_HTTP_BODY)
		{
			return s;
		}
		if (Srv.Ended())
		{
			Http.StreamEnded();
			return Http.State();
		}
	}
	return Http.State();
}

static bool Reboot(void)
{
	if (setjmp(g_HostStartJmp) != 0)
	{
		return g_HostStartAddr == HOST_SLOT0;
	}
	(void)DfuBootRun(s_Boot);
	return false;
}

static bool Slot0Is(const std::vector<uint8_t> &File)
{
	const DfuImgHdr_t *h = (const DfuImgHdr_t *)File.data();
	return memcmp(HostFlash(HOST_SLOT0), File.data() + h->HdrSize,
				  h->ImgSize) == 0;
}

int main(int argc, char **argv)
{
	s_Dir = argc > 1 ? argv[1] : "build/img";
	s_KeyDer = Img("key1.der");
	s_Key = { s_KeyDer.data(), (uint32_t)s_KeyDer.size() };
	s_Boot.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem));
	s_Boot.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr);
	s_Boot.pKey = &s_Key;
	s_Boot.NbKey = 1;
	s_Boot.bVerifySlot0 = true;

	HostFlashInit(HOST_MEM_NOR);
	DfuLayout_t lay = HostLayout();
	static DfuStore_t st;
	static DfuMgr wr;
	static DfuHttp http;
	CHECK(DfuStoreTgt(&st, HOST_SLOT1, HOST_SLOT1_SIZE));
	DfuMgrCfg_t mc = {
		.pStore = &st, .bDirect = false, .bManifestOnly = false,
		.bAllowDowngrade = true, .pHash = s_Boot.pHash, .pBoot = nullptr,
	};
	CHECK(wr.Init(mc));
	CHECK(http.Init(&wr));
	(void)lay;

	std::vector<uint8_t> v1 = Img("v1.bin"), v2 = Img("v2.bin"),
						 v3 = Img("v3.bin");

	// Whole image in one response; the request is what it should be.
	{
		FakeServer s;
		s.vFile = v1;
		CHECK(http.Get(&s, "fw.example.com", "/img/v1.bin"));
		CHECK(s.vReq == "GET /img/v1.bin HTTP/1.1\r\nHost: fw.example.com\r\n"
						"Connection: close\r\n\r\n");
		CHECK(Run(http, s) == DFU_HTTP_DONE);
		CHECK(HostPending() == DFU_TRAILER_PENDING);
		CHECK(Reboot() && Slot0Is(v1));
	}

	// Broken twice, resumed with Range each time, the response head fed a
	// byte at a time on the last stream.
	{
		FakeServer a, b, c;
		a.vFile = b.vFile = c.vFile = v3;
		a.vDropAt = 5000;
		b.vDropAt = 30000;
		c.vMaxRx = 1;
		CHECK(http.Get(&a, "h", "/v3"));
		CHECK(Run(http, a) == DFU_HTTP_BROKEN);
		uint32_t got = http.Received();
		CHECK(got > 0 && got < v3.size() && http.Total() == v3.size());
		CHECK(http.Resume(&b));
		CHECK(b.vReq.find("Range: bytes=" + std::to_string(got) + "-\r\n") !=
			  std::string::npos);
		CHECK(Run(http, b) == DFU_HTTP_BROKEN);
		CHECK(http.Received() > got);
		c.vMaxRx = 1;
		CHECK(http.Resume(&c));
		// Past the head, bigger reads again.
		for (int i = 0; i < 400 && http.State() == DFU_HTTP_HEADER; i++)
		{
			http.Poll();
		}
		c.vMaxRx = 700;
		CHECK(Run(http, c) == DFU_HTTP_DONE);
		CHECK(Reboot() && Slot0Is(v3));
	}

	// Broken before the body: the resume starts the image over.
	{
		FakeServer a, b;
		a.vFile = b.vFile = v2;
		a.vDropAt = 20;
		CHECK(http.Get(&a, "h", "/v2"));
		CHECK(Run(http, a) == DFU_HTTP_BROKEN);
		CHECK(http.Resume(&b));
		CHECK(b.vReq.find("Range") == std::string::npos);
		CHECK(Run(http, b) == DFU_HTTP_DONE);
		CHECK(Reboot() && Slot0Is(v2));
	}

	// A server that answers a Range request with the whole image.
	{
		FakeServer a, b;
		a.vFile = b.vFile = v1;
		a.vDropAt = 3000;
		b.vMode = 1;
		CHECK(http.Get(&a, "h", "/v1"));
		CHECK(Run(http, a) == DFU_HTTP_BROKEN);
		CHECK(http.Resume(&b));
		CHECK(Run(http, b) == DFU_HTTP_DONE);
		CHECK(Reboot() && Slot0Is(v1));
	}

	// Refused answers; slot 1 is not pending after any of them.
	struct { int Mode; const char *pFile; int Err; } bad[] = {
		{ 2, "v2.bin", DFU_HTTP_ERR_STATUS },
		{ 3, "v2.bin", DFU_HTTP_ERR_HEADER },
		{ 5, "v2.bin", DFU_HTTP_ERR_HEADER },
		{ 0, "v1_tamper.bin", DFU_HTTP_ERR_IMAGE },
		{ 0, "vec.bin", DFU_HTTP_ERR_IMAGE },
	};
	for (auto &b : bad)
	{
		FakeServer s;
		s.vFile = Img(b.pFile);
		s.vMode = b.Mode;
		CHECK(http.Get(&s, "h", "/x"));
		CHECK(Run(http, s) == DFU_HTTP_FAILED && http.Error() == b.Err);
		CHECK(HostPending() != DFU_TRAILER_PENDING || HostDone() == DFU_TRAILER_DONE);
	}

	// A 206 that does not start where asked.
	{
		FakeServer a, b;
		a.vFile = b.vFile = v2;
		a.vDropAt = 4000;
		b.vMode = 4;
		CHECK(http.Get(&a, "h", "/v2"));
		CHECK(Run(http, a) == DFU_HTTP_BROKEN);
		CHECK(http.Resume(&b));
		CHECK(Run(http, b) == DFU_HTTP_FAILED &&
			  http.Error() == DFU_HTTP_ERR_RANGE);
	}

	// Larger than slot 1.
	{
		FakeServer s;
		s.vFile.assign(HOST_SLOT1_SIZE, 0);
		memcpy(s.vFile.data(), v1.data(), 64);
		CHECK(http.Get(&s, "h", "/big"));
		CHECK(Run(http, s) == DFU_HTTP_FAILED &&
			  http.Error() == DFU_HTTP_ERR_SIZE);
	}

	// Bad arguments.
	FakeServer s;
	CHECK(http.Get(&s, "h", "no-slash") == false);
	CHECK(http.Resume(&s) == false);

	// The image that runs is the last one installed.
	CHECK(Reboot() && Slot0Is(v1));
	CHECK(g_HostNorViolations == 0);

	printf("dfu_http_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail != 0;
}
