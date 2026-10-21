/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * RFC 7252 unicast CoAP client over a connected synchronous UDP interface.
 */
#ifndef __NET_COAP_H__
#define __NET_COAP_H__

#include "crypto/icrypto.h"

#define COAP_MESSAGE_SIZE		1152U
#define COAP_MAX_OPTIONS			16U
#define COAP_TOKEN_SIZE			8U
#define COAP_ACK_TIMEOUT			2000U
#define COAP_MAX_RETRANSMIT		4U
#define COAP_MAX_TRANSMIT_SPAN	45000U
#define COAP_EXCHANGE_LIFETIME	247000U
#define COAP_DUPLICATE_COUNT		4U

typedef enum
{
	COAP_CON,
	COAP_NON,
	COAP_ACK,
	COAP_RST
} COAP_TYPE;

typedef enum
{
	COAP_GET = 1,
	COAP_POST = 2,
	COAP_PUT = 3,
	COAP_DELETE = 4,
	COAP_CREATED = 65,
	COAP_DELETED = 66,
	COAP_VALID = 67,
	COAP_CHANGED = 68,
	COAP_CONTENT = 69,
	COAP_BAD_REQUEST = 128,
	COAP_NOT_FOUND = 132,
	COAP_METHOD_NOT_ALLOWED = 133
} COAP_CODE;

typedef enum
{
	COAP_OPTION_IF_MATCH = 1,
	COAP_OPTION_URI_HOST = 3,
	COAP_OPTION_ETAG = 4,
	COAP_OPTION_IF_NONE_MATCH = 5,
	COAP_OPTION_URI_PORT = 7,
	COAP_OPTION_LOCATION_PATH = 8,
	COAP_OPTION_URI_PATH = 11,
	COAP_OPTION_CONTENT_FORMAT = 12,
	COAP_OPTION_MAX_AGE = 14,
	COAP_OPTION_URI_QUERY = 15,
	COAP_OPTION_ACCEPT = 17,
	COAP_OPTION_LOCATION_QUERY = 20,
	COAP_OPTION_SIZE1 = 60
} COAP_OPTION;

typedef enum
{
	COAP_IDLE,
	COAP_READY,
	COAP_SENDING,
	COAP_WAIT_ACK,
	COAP_WAIT_RESPONSE,
	COAP_RESPONSE,
	COAP_FAILED
} COAP_STATE;

typedef enum
{
	COAP_ERROR_NONE,
	COAP_ERROR_CONFIG,
	COAP_ERROR_TRANSPORT,
	COAP_ERROR_PROTOCOL,
	COAP_ERROR_LIMIT,
	COAP_ERROR_TIMEOUT,
	COAP_ERROR_RESET,
	COAP_ERROR_OPTION,
	COAP_ERROR_RANDOM,
	COAP_ERROR_CANCELLED,
	COAP_ERROR_RATE
} COAP_ERROR;

typedef struct
{
	uint16_t Number;
	const uint8_t *pValue;
	size_t Len;
} CoapOption_t;

/* Views into the decoded datagram. Option values and token are not strings.
 * Unknown elective options remain visible. The codec performs wire validation;
 * the client additionally checks supported options and response matching.
 */
typedef struct
{
	COAP_TYPE Type;
	uint8_t Code;
	uint16_t MessageId;
	const uint8_t *pToken;
	size_t TokenLen;
	CoapOption_t Options[COAP_MAX_OPTIONS];
	size_t OptionCount;
	const uint8_t *pPayload;
	size_t PayloadLen;
} CoapMessage_t;

typedef struct
{
	COAP_TYPE Type;		/* CON or NON. */
	uint8_t Code;		/* GET, POST, PUT or DELETE. */
	const CoapOption_t *pOptions;
	size_t OptionCount;
	const uint8_t *pPayload;
	size_t PayloadLen;
} CoapRequest_t;

typedef struct
{
	RngEngine *pRng;		/* Already enabled synchronous RNG; no ownership transfer. */
	uint32_t Timeout;	/* Entire request deadline, ms, 1..INT32_MAX; normally 93000. */
	uint32_t ProbingRate;	/* Unanswered traffic, bytes/second. Zero selects RFC default 1. */
} CoapCfg_t;

typedef struct
{
	uint32_t Received;
	uint16_t MessageId;
	bool bValid;
} CoapDuplicate_t;

/* Zero-initialize before first use. One foreground owner, no ISR/callback use.
 * No allocation. The connected datagram transport filters the peer IP/port.
 * One request at a time. Data stores the request, then the accepted response.
 * Response views remain valid until Release, Cancel, Reset or reinitialization.
 * Keep Process running while holding a response so duplicate CONs are ACKed.
 * A later transport error sets Error without discarding an accepted response.
 */
typedef struct
{
	DevIntrf_t *pTransport;
	RngEngine *pRng;
	COAP_STATE State;
	COAP_ERROR Error;
	CoapMessage_t Message;
	CoapDuplicate_t Duplicates[COAP_DUPLICATE_COUNT];
	uint32_t Timeout;
	uint32_t Started;
	uint32_t FirstSent;
	uint32_t LastSent;
	uint32_t RetryDelay;
	uint32_t LastId;
	uint32_t IdCount;
	uint32_t ControlStarted;
	uint32_t ProbingRate;
	uint32_t ProbeStarted;
	uint32_t ProbeDelay;
	uint16_t NextId;
	uint16_t RequestId;
	uint8_t Retransmit;
	uint8_t DuplicateNext;
	uint8_t Token[COAP_TOKEN_SIZE];
	uint8_t Control[4];
	bool bControl;
	bool bSent;
	bool bConfirmable;
	size_t DataLen;
	uint8_t Data[COAP_MESSAGE_SIZE];
	uint8_t Rx[COAP_MESSAGE_SIZE + 1U];
} CoapDev_t;

#ifdef __cplusplus
extern "C"
{
#endif

/* Codec buffers must not overlap their input views. Options are ordered by
 * number, with repeated options adjacent. URI components are raw decoded byte
 * spans: use one Uri-Path per segment and one Uri-Query per query component.
 * An empty final Uri-Path represents a trailing slash. Request path segments
 * must not be "." or "..". No URI parser is hidden here. Integer request option
 * values use shortest big-endian form (zero is empty).
 * String option spans are caller-supplied Net-Unicode UTF-8. The codec neither
 * normalizes nor validates string encoding; received values remain raw bytes.
 */
COAP_ERROR CoapEncode(const CoapMessage_t *pMessage, uint8_t *pData,
					 size_t Size, size_t *pLen);
COAP_ERROR CoapDecode(CoapMessage_t *pMessage, const uint8_t *pData, size_t Len);

/* Transport is already connected UDP, with MaxRetry=0. Each RxData consumes
 * one datagram (including any truncated tail); each TxData accepts the whole
 * datagram or returns zero. Negative async results and short TX are unsupported.
 * The extra receive byte detects packets above COAP_MESSAGE_SIZE.
 * A fresh Init is for a fresh UDP endpoint; do not reset IDs on a live endpoint.
 */
bool CoapInit(CoapDev_t *pDev, const CoapCfg_t *pCfg, DevIntrf_t *pTransport,
			  uint32_t Now);
/* Copies request options/payload before return; caller storage may then expire.
 * Rejects an invalid request without starting an exchange. Busy rejection leaves
 * the active exchange unchanged. 4.xx/5.xx replies are ordinary responses.
 * An early new request to an unresponsive peer returns false with ERROR_RATE;
 * RetryAfter reports its remaining wait. Release/Cancel retain this rate state.
 */
bool CoapRequest(CoapDev_t *pDev, const CoapRequest_t *pRequest, uint32_t Now);
/* Bounded pump: at most one whole-datagram read and one write per call.
 * Pass the socket Connected() state. Deadlines use a wrapping millisecond clock.
 */
COAP_STATE CoapProcess(CoapDev_t *pDev, uint32_t Now, bool bTransportOpen);
/* True for occupied protocol state, including a held response,
 * a failed exchange awaiting Release, and an unsent ACK/Reset control packet.
 * A READY context can also be rate limited; check RetryAfter with the same clock.
 */
bool CoapBusy(const CoapDev_t *pDev);
uint32_t CoapRetryAfter(const CoapDev_t *pDev, uint32_t Now);
/* Release a response or failed exchange. Retains peer duplicate and ID state. */
void CoapRelease(CoapDev_t *pDev);
/* Abandon a request locally. Does not undo an operation already sent. */
void CoapCancel(CoapDev_t *pDev);
/* Clear all state without closing the caller-owned transport. */
void CoapReset(CoapDev_t *pDev);

#ifdef __cplusplus
}

class Coap : public Device
{
public:
	Coap() = default;
	~Coap()
	{
		CoapReset(&vDev);
	}
	Coap(const Coap &) = delete;
	Coap &operator=(const Coap &) = delete;
	bool Init(const CoapCfg_t &Cfg, DeviceIntrf *pIntrf, uint32_t Now)
	{
		Interface(pIntrf);
		vbEnabled = CoapInit(&vDev, &Cfg, pIntrf ? (DevIntrf_t *)*pIntrf : nullptr, Now);
		Valid(vbEnabled);
		return vbEnabled;
	}
	bool Enable() override
	{
		vbEnabled = vDev.pTransport != nullptr;
		return vbEnabled;
	}
	// Pause the pump; request and duplicate deadlines continue to elapse.
	void Disable() override
	{
		vbEnabled = false;
	}
	void Reset() override
	{
		CoapReset(&vDev);
		vbEnabled = false;
		Valid(false);
	}
	void PowerOff() override
	{
		Reset();
	}
	bool Request(const CoapRequest_t &Request, uint32_t Now)
	{
		return vbEnabled && CoapRequest(&vDev, &Request, Now);
	}
	COAP_STATE Process(uint32_t Now, bool Open)
	{
		if (vbEnabled)
		{
			CoapProcess(&vDev, Now, Open);
		}
		Valid(vDev.State != COAP_IDLE && vDev.State != COAP_FAILED);
		return vDev.State;
	}
	bool Busy() const
	{
		return CoapBusy(&vDev);
	}
	uint32_t RetryAfter(uint32_t Now) const
	{
		return CoapRetryAfter(&vDev, Now);
	}
	COAP_ERROR Error() const
	{
		return vDev.Error;
	}
	const CoapMessage_t *Response() const
	{
		return vDev.State == COAP_RESPONSE ? &vDev.Message : nullptr;
	}
	void Release()
	{
		CoapRelease(&vDev);
	}
	void Cancel()
	{
		CoapCancel(&vDev);
	}

private:
	CoapDev_t vDev = {};
	bool vbEnabled = false;
};
#endif
#endif
