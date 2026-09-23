/**-------------------------------------------------------------------------
@example	lte_ota.cpp

@brief	Firmware update over LTE on nRF91: download a signed image over
		HTTP or HTTPS into slot 1, reset into the stage 0 boot.

The image is the one imgtool signs for the nRF91 layout, served as a plain
file by any web server. DfuHttp writes it into slot 1 through the same
writer the SMP server uses and marks it pending once its hash and entry
check; the boot checks the signature and installs it, see
docs/architecture/dfu.md. A download cut by the network resumes where it
stopped, with a Range request on a new connection.

	imgtool sign -k dfu_dev_key.pem --header-size 0x20 --pad-header --align 4 \
		-v 1.0.1 -S <slot 1 size - 32> app.bin app_signed.bin

What the modem needs is the application's: the Modem library, its OS glue
(nrf_modem_os) and the shared memory set aside for it, the same as for any
nRF91 socket application. IOsonata does not provide these; this example
starts once nrf_modem_init has run. With TLS the server certificate goes in
the modem under LTE_OTA_SEC_TAG beforehand (AT%CMNG).

Configure the server here or from board.h:

	LTE_OTA_HOST		server name
	LTE_OTA_PORT		80, or 443 with LTE_OTA_TLS
	LTE_OTA_PATH		path of the signed image
	LTE_OTA_TLS			1 for HTTPS
	LTE_OTA_SEC_TAG		modem security tag holding the CA certificate

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "nrf.h"
#include "nrf_modem.h"
#include "nrf_modem_at.h"
#include "nrf_socket.h"
#include "nrf_errno.h"

#include "idelay.h"
#include "device_intrf.h"
#include "dfu/dfu_http.h"
#include "crypto/crypto_softsha256.h"

#ifndef LTE_OTA_HOST
#define LTE_OTA_HOST		"fw.example.com"
#endif
#ifndef LTE_OTA_TLS
#define LTE_OTA_TLS			0
#endif
#ifndef LTE_OTA_PORT
#define LTE_OTA_PORT		(LTE_OTA_TLS ? 443 : 80)
#endif
#ifndef LTE_OTA_PATH
#define LTE_OTA_PATH		"/app_signed.bin"
#endif
#ifndef LTE_OTA_SEC_TAG
#define LTE_OTA_SEC_TAG		42
#endif

// Tries at a connection for one download.
#define LTE_OTA_CONN_TRIES	5

/// A connected modem socket as a byte stream DeviceIntrf, the way a UART is
/// one, so DfuHttp reads and writes it like any other stream.
class NrfSockIntrf : public DeviceIntrf {
public:
	bool Open(const char *pHost, uint16_t Port, bool bTls, int SecTag);
	void Close(void);
	bool Ended(void) const { return vbEnded; }

	operator DevIntrf_t * () override { return &vDev; }
	uint32_t Rate(uint32_t) override { return 0; }
	uint32_t Rate(void) override { return 0; }

	int Rx(uint32_t, uint8_t *pBuff, int BuffLen) override {
		ssize_t n = nrf_recv(vFd, pBuff, (size_t)BuffLen, NRF_MSG_DONTWAIT);

		if (n == 0 || (n < 0 && errno != NRF_EAGAIN))
		{
			vbEnded = true;
		}
		return n > 0 ? (int)n : 0;
	}
	int Tx(uint32_t, const uint8_t *pData, int DataLen) override {
		ssize_t n = nrf_send(vFd, pData, (size_t)DataLen, 0);

		if (n < 0)
		{
			vbEnded = true;
		}
		return n > 0 ? (int)n : 0;
	}

private:
	int vFd = -1;
	bool vbEnded = true;
	DevIntrf_t vDev = {};
};

bool NrfSockIntrf::Open(const char *pHost, uint16_t Port, bool bTls, int SecTag)
{
	struct nrf_addrinfo hints = {};
	struct nrf_addrinfo *res = nullptr;

	hints.ai_family = NRF_AF_INET;
	hints.ai_socktype = NRF_SOCK_STREAM;

	if (nrf_getaddrinfo(pHost, nullptr, &hints, &res) != 0 || res == nullptr)
	{
		return false;
	}

	((struct nrf_sockaddr_in *)res->ai_addr)->sin_port = nrf_htons(Port);

	vFd = nrf_socket(NRF_AF_INET, NRF_SOCK_STREAM,
					 bTls ? NRF_SPROTO_TLS1v2 : NRF_IPPROTO_TCP);
	if (vFd < 0)
	{
		nrf_freeaddrinfo(res);
		return false;
	}

	if (bTls)
	{
		nrf_sec_tag_t tag = (nrf_sec_tag_t)SecTag;
		int verify = NRF_SO_SEC_PEER_VERIFY_REQUIRED;

		if (nrf_setsockopt(vFd, NRF_SOL_SECURE, NRF_SO_SEC_TAG_LIST, &tag,
						   sizeof(tag)) != 0 ||
			nrf_setsockopt(vFd, NRF_SOL_SECURE, NRF_SO_SEC_PEER_VERIFY,
						   &verify, sizeof(verify)) != 0 ||
			nrf_setsockopt(vFd, NRF_SOL_SECURE, NRF_SO_SEC_HOSTNAME, pHost,
						   strlen(pHost)) != 0)
		{
			nrf_freeaddrinfo(res);
			Close();
			return false;
		}
	}

	int r = nrf_connect(vFd, res->ai_addr, res->ai_addrlen);
	nrf_freeaddrinfo(res);
	if (r != 0)
	{
		Close();
		return false;
	}

	vbEnded = false;

	return true;
}

void NrfSockIntrf::Close(void)
{
	if (vFd >= 0)
	{
		nrf_close(vFd);
	}
	vFd = -1;
	vbEnded = true;
}

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];

static DfuStore_t s_Slot1;
static DfuMgr s_Mgr;
static DfuHttp s_Http;
static NrfSockIntrf s_Sock;

// Attached once CEREG reports home (1) or roaming (5).
static bool LteAttach(void)
{
	char rsp[64];

	if (nrf_modem_at_printf("AT+CEREG=0") != 0 ||
		nrf_modem_at_printf("AT+CFUN=1") != 0)
	{
		return false;
	}

	for (int i = 0; i < 600; i++)
	{
		if (nrf_modem_at_cmd(rsp, sizeof(rsp), "AT+CEREG?") == 0)
		{
			const char *p = strchr(rsp, ',');
			if (p != nullptr && (p[1] == '1' || p[1] == '5'))
			{
				return true;
			}
		}
		msDelay(500);
	}

	return false;
}

// One download, resumed over new connections while the network allows.
static bool LteOtaDownload(void)
{
	DFU_HTTP_STATE st = DFU_HTTP_IDLE;

	for (int conn = 0; conn < LTE_OTA_CONN_TRIES; conn++)
	{
		if (s_Sock.Open(LTE_OTA_HOST, LTE_OTA_PORT, LTE_OTA_TLS,
						LTE_OTA_SEC_TAG) == false)
		{
			msDelay(2000);
			continue;
		}

		bool sent = st == DFU_HTTP_BROKEN ? s_Http.Resume(&s_Sock) :
						s_Http.Get(&s_Sock, LTE_OTA_HOST, LTE_OTA_PATH);
		if (sent == false)
		{
			s_Sock.Close();
			return false;
		}

		for (;;)
		{
			st = s_Http.Poll();
			if (st == DFU_HTTP_DONE || st == DFU_HTTP_FAILED)
			{
				break;
			}
			if (s_Sock.Ended())
			{
				s_Http.StreamEnded();
				st = s_Http.State();
				break;
			}
			msDelay(10);
		}
		s_Sock.Close();

		if (st != DFU_HTTP_BROKEN)
		{
			break;
		}
	}

	return st == DFU_HTTP_DONE;
}

// Slot 1 through the target layer, images in order as HTTP brings them,
// nothing older than what runs.
static DfuMgrCfg_t s_MgrCfg(HashEngine *pSha)
{
	DfuMgrCfg_t c = {
		.pStore = &s_Slot1,
		.bDirect = false,
		.bManifestOnly = false,
		.bAllowDowngrade = false,
		.pHash = pSha,
		.pBoot = nullptr,
	};

	return c;
}

int main()
{
	DfuLayout_t lay;
	HashEngine *sha = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem));

	// No radio stack shares the flash controller with the application on
	// nRF91: slot 1 is written through the target layer.
	if (DfuLayoutGet(&lay) == false ||
		DfuStoreTgt(&s_Slot1, lay.Slot1, lay.Slot1Size) == false ||
		s_Mgr.Init(s_MgrCfg(sha)) == false ||
		s_Http.Init(&s_Mgr) == false)
	{
		for (;;)
		{
			__WFE();
		}
	}

	// The Modem library is up here: nrf_modem_init has run from the
	// application's modem integration.
	if (LteAttach() && LteOtaDownload())
	{
		// Pending in slot 1: the boot installs it at this reset.
		DfuTgtReset();
	}

	for (;;)
	{
		__WFE();
	}
}
