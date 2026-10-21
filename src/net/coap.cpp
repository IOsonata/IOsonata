/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * RFC 7252 unicast CoAP. One exchange, fixed storage, synchronous datagrams.
 */
#include <string.h>
#include "net/coap.h"

namespace
{
uint16_t Get16(const uint8_t *pData)
{
	return (uint16_t)((uint16_t)pData[0] << 8 | pData[1]);
}

void Put16(uint8_t *pData, uint16_t Value)
{
	pData[0] = (uint8_t)(Value >> 8);
	pData[1] = (uint8_t)Value;
}

bool Header(COAP_TYPE Type, uint8_t Code, size_t TokenLen,
			size_t OptionCount, size_t PayloadLen)
{
	uint8_t cls = Code >> 5;

	if (Type < COAP_CON || Type > COAP_RST || TokenLen > COAP_TOKEN_SIZE)
	{
		return false;
	}
	if (Code == 0)
	{
		return Type != COAP_NON && TokenLen == 0 && OptionCount == 0 && PayloadLen == 0;
	}
	if (Type == COAP_RST || (cls != 0 && cls != 2 && cls != 4 && cls != 5))
	{
		return false;
	}
	return Type != COAP_ACK || cls != 0;
}

size_t Extra(size_t Value)
{
	if (Value < 13)
	{
		return 0;
	}
	return Value < 269 ? 1 : 2;
}

uint8_t Nibble(size_t Value)
{
	if (Value < 13)
	{
		return (uint8_t)Value;
	}
	return Value < 269 ? 13 : 14;
}

size_t PutExtended(uint8_t *pData, size_t Value)
{
	if (Value < 13)
	{
		return 0;
	}
	if (Value < 269)
	{
		pData[0] = (uint8_t)(Value - 13);
		return 1;
	}
	Put16(pData, (uint16_t)(Value - 269));
	return 2;
}

bool Extended(const uint8_t *pData, size_t Len, size_t *pOffset,
			  uint8_t Value, size_t *pValue)
{
	size_t extra;

	if (Value == 15)
	{
		return false;
	}
	extra = Value < 13 ? 0 : (size_t)(Value - 12);
	if (extra > Len - *pOffset)
	{
		return false;
	}
	*pValue = Value;
	if (extra == 1)
	{
		*pValue = 13U + pData[*pOffset];
	}
	else if (extra == 2)
	{
		*pValue = 269U + Get16(pData + *pOffset);
	}
	*pOffset += extra;
	return true;
}

bool Active(const CoapDev_t *pDev)
{
	return pDev->State == COAP_SENDING || pDev->State == COAP_WAIT_ACK ||
		   pDev->State == COAP_WAIT_RESPONSE;
}

COAP_STATE Fail(CoapDev_t *pDev, COAP_ERROR Error)
{
	pDev->Error = Error;
	if (pDev->State == COAP_RESPONSE)
	{
		// An accepted response remains readable even if a later duplicate read
		// or acknowledgement send discovers a transport failure.
		return pDev->State;
	}
	pDev->State = COAP_FAILED;
	pDev->DataLen = 0;
	pDev->bSent = false;
	pDev->Message = {};
	return pDev->State;
}

/* A single, paired interface transfer. Do not use stream accumulation or the
 * generic retry loop. The injected interface must complete synchronously.
 */
int Send(CoapDev_t *pDev, const uint8_t *pData, size_t Len)
{
	int count;
	DevIntrf_t *transport = pDev->pTransport;

	if (!DeviceIntrfStartTx(transport, 0))
	{
		return 0;
	}
	transport->bNoStop = false;
	count = transport->TxData(transport, pData, (int)Len);
	DeviceIntrfStopTx(transport);
	return count;
}

int Receive(CoapDev_t *pDev)
{
	int count;
	DevIntrf_t *transport = pDev->pTransport;

	if (!DeviceIntrfStartRx(transport, 0))
	{
		return 0;
	}
	transport->bNoStop = false;
	count = transport->RxData(transport, pDev->Rx, (int)sizeof(pDev->Rx));
	DeviceIntrfStopRx(transport);
	return count;
}

/* Request options are the supported RFC 7252 options only. For responses,
 * unsupported critical options reject the response. Invalid or misplaced
 * elective options and extra nonrepeatable elective occurrences are ignored
 * per 5.4.3/5.4.5; unknown elective options remain available as raw views.
 */
COAP_ERROR Options(CoapMessage_t *pMessage, bool Response)
{
	size_t i;
	size_t out = 0;
	size_t minimum;
	size_t maximum;
	uint16_t previous = 0;
	bool havePrevious = false;
	bool repeat;
	bool integer;
	bool allowed;
	bool known;
	bool location = false;
	bool reservedLocation = false;
	CoapOption_t option;

	for (i = 0; i < pMessage->OptionCount; i++)
	{
		option = pMessage->Options[i];
		if (option.Number == COAP_OPTION_LOCATION_PATH ||
			option.Number == COAP_OPTION_LOCATION_QUERY)
		{
			location = true;
		}
		if (option.Number == 128 || option.Number == 132 ||
			option.Number == 136 || option.Number == 140)
		{
			reservedLocation = true;
		}
		minimum = 0;
		maximum = 0;
		repeat = false;
		integer = false;
		allowed = true;
		known = true;
		switch (option.Number)
		{
			case COAP_OPTION_IF_MATCH:
				maximum = 8;
				repeat = true;
				allowed = !Response;
				break;
			case COAP_OPTION_URI_HOST:
				minimum = 1;
				maximum = 255;
				allowed = !Response;
				break;
			case COAP_OPTION_ETAG:
				minimum = 1;
				maximum = 8;
				repeat = !Response;
				break;
			case COAP_OPTION_IF_NONE_MATCH:
				allowed = !Response;
				break;
			case COAP_OPTION_URI_PORT:
			case COAP_OPTION_ACCEPT:
				maximum = 2;
				integer = true;
				allowed = !Response;
				break;
			case COAP_OPTION_LOCATION_PATH:
			case COAP_OPTION_LOCATION_QUERY:
				maximum = 255;
				repeat = true;
				allowed = Response;
				break;
			case COAP_OPTION_URI_PATH:
			case COAP_OPTION_URI_QUERY:
				maximum = 255;
				repeat = true;
				allowed = !Response;
				break;
			case COAP_OPTION_CONTENT_FORMAT:
				maximum = 2;
				integer = true;
				break;
			case COAP_OPTION_MAX_AGE:
				maximum = 4;
				integer = true;
				allowed = Response;
				break;
			case COAP_OPTION_SIZE1:
				maximum = 4;
				integer = true;
				break;
			default:
				known = false;
				allowed = Response && (option.Number & 1U) == 0;
				break;
		}
		if (known && (option.Len < minimum || option.Len > maximum ||
			(havePrevious && option.Number == previous && !repeat)))
		{
			allowed = false;
		}
		if (!Response && integer && option.Len && option.pValue[0] == 0)
		{
			allowed = false;
		}
		if (((!Response && option.Number == COAP_OPTION_URI_PATH) ||
			 (Response && option.Number == COAP_OPTION_LOCATION_PATH)) &&
			((option.Len == 1 && option.pValue[0] == '.') ||
			 (option.Len == 2 && option.pValue[0] == '.' && option.pValue[1] == '.')))
		{
			allowed = false;
		}
		if (!allowed)
		{
			if (!Response || (option.Number & 1U))
			{
				return COAP_ERROR_OPTION;
			}
			continue;
		}
		pMessage->Options[out++] = option;
		previous = option.Number;
		havePrevious = true;
	}
	// RFC 7252 section 5.10.7, verified erratum 4949: these future Location
	// components make a partial location unusable, despite their even numbers.
	if (Response && location && reservedLocation)
	{
		return COAP_ERROR_OPTION;
	}
	pMessage->OptionCount = out;
	return COAP_ERROR_NONE;
}

void Control(CoapDev_t *pDev, COAP_TYPE Type, uint16_t MessageId, uint32_t Now)
{
	pDev->Control[0] = (uint8_t)(0x40U | ((unsigned)Type << 4));
	pDev->Control[1] = 0;
	Put16(pDev->Control + 2, MessageId);
	pDev->ControlStarted = Now;
	pDev->bControl = true;
}

bool Duplicate(CoapDev_t *pDev, uint16_t MessageId, uint32_t Now)
{
	size_t i;
	CoapDuplicate_t *entry;

	for (i = 0; i < COAP_DUPLICATE_COUNT; i++)
	{
		entry = pDev->Duplicates + i;
		if (entry->bValid && (uint32_t)(Now - entry->Received) >= COAP_EXCHANGE_LIFETIME)
		{
			entry->bValid = false;
		}
		if (entry->bValid && entry->MessageId == MessageId)
		{
			return true;
		}
	}
	return false;
}

void Remember(CoapDev_t *pDev, uint16_t MessageId, uint32_t Now)
{
	CoapDuplicate_t *entry = pDev->Duplicates + pDev->DuplicateNext;

	entry->MessageId = MessageId;
	entry->Received = Now;
	entry->bValid = true;
	pDev->DuplicateNext = (uint8_t)((pDev->DuplicateNext + 1U) % COAP_DUPLICATE_COUNT);
}

bool Matches(const CoapDev_t *pDev, const CoapMessage_t *pMessage)
{
	if (!Active(pDev) || !pDev->bSent || pMessage->TokenLen != COAP_TOKEN_SIZE ||
		memcmp(pMessage->pToken, pDev->Token, COAP_TOKEN_SIZE) != 0)
	{
		return false;
	}
	return pMessage->Type != COAP_ACK ||
		   (pDev->bConfirmable && pMessage->MessageId == pDev->RequestId);
}

void Packet(CoapDev_t *pDev, size_t Len, uint32_t Now)
{
	CoapMessage_t message = {};
	COAP_ERROR error;
	COAP_TYPE type;
	uint16_t id;
	bool matching;

	if (Len < 4 || (pDev->Rx[0] >> 6) != 1)
	{
		return;
	}
	type = (COAP_TYPE)((pDev->Rx[0] >> 4) & 3U);
	id = Get16(pDev->Rx + 2);
	error = CoapDecode(&message, pDev->Rx, Len);
	if (error != COAP_ERROR_NONE)
	{
		if (type == COAP_CON)
		{
			Control(pDev, COAP_RST, id, Now);
		}
		// A bounded decoder cannot retain an oversized matching response.
		// Malformed/unrelated packets never terminate the pending exchange.
		matching = type != COAP_RST && Len >= 4U + COAP_TOKEN_SIZE &&
			(pDev->Rx[0] & 15U) == COAP_TOKEN_SIZE &&
			((pDev->Rx[1] >> 5) == 2 || (pDev->Rx[1] >> 5) == 4 ||
			 (pDev->Rx[1] >> 5) == 5) && Active(pDev) && pDev->bSent &&
			memcmp(pDev->Rx + 4, pDev->Token, COAP_TOKEN_SIZE) == 0 &&
			(type != COAP_ACK || (pDev->bConfirmable && id == pDev->RequestId));
		if (error == COAP_ERROR_LIMIT && matching)
		{
			pDev->ProbeDelay = 0;
			Fail(pDev, error);
		}
		return;
	}
	if (message.Type == COAP_RST)
	{
		if (Active(pDev) && pDev->bSent && id == pDev->RequestId)
		{
			pDev->ProbeDelay = 0;
			Fail(pDev, COAP_ERROR_RESET);
		}
		return;
	}
	if (message.Code == 0)
	{
		if (message.Type == COAP_CON)
		{
			Control(pDev, COAP_RST, id, Now);
		}
		else if (message.Type == COAP_ACK && pDev->State == COAP_WAIT_ACK &&
				 id == pDev->RequestId)
		{
			pDev->State = COAP_WAIT_RESPONSE;
		}
		return;
	}
	if (message.Type == COAP_CON && Duplicate(pDev, id, Now))
	{
		Control(pDev, COAP_ACK, id, Now);
		return;
	}
	if ((message.Code >> 5) == 0 || !Matches(pDev, &message))
	{
		if (message.Type == COAP_CON)
		{
			Control(pDev, COAP_RST, id, Now);
		}
		return;
	}
	// A matching response demonstrates that this peer answers requests even
	// when its options exceed this client's supported profile.
	pDev->ProbeDelay = 0;
	error = Options(&message, true);
	if (error != COAP_ERROR_NONE)
	{
		if (message.Type == COAP_CON)
		{
			Control(pDev, COAP_RST, id, Now);
		}
		Fail(pDev, error);
		return;
	}
	if (message.Type == COAP_CON)
	{
		Remember(pDev, id, Now);
		Control(pDev, COAP_ACK, id, Now);
	}
	memcpy(pDev->Data, pDev->Rx, Len);
	CoapDecode(&pDev->Message, pDev->Data, Len);
	Options(&pDev->Message, true);
	pDev->DataLen = Len;
	pDev->State = COAP_RESPONSE;
	pDev->Error = COAP_ERROR_NONE;
	pDev->bSent = false;
}
} // namespace

COAP_ERROR CoapEncode(const CoapMessage_t *pMessage, uint8_t *pData,
					 size_t Size, size_t *pLen)
{
	size_t i;
	size_t total;
	size_t delta;
	size_t count;
	size_t offset;
	uint16_t previous = 0;
	const CoapOption_t *option;

	if (pLen)
	{
		*pLen = 0;
	}
	if (!pMessage || !pData || !pLen ||
		(pMessage->TokenLen && !pMessage->pToken) ||
		(pMessage->PayloadLen && !pMessage->pPayload))
	{
		return COAP_ERROR_CONFIG;
	}
	if (!Header(pMessage->Type, pMessage->Code, pMessage->TokenLen,
				pMessage->OptionCount, pMessage->PayloadLen))
	{
		return COAP_ERROR_PROTOCOL;
	}
	if (pMessage->OptionCount > COAP_MAX_OPTIONS)
	{
		return COAP_ERROR_LIMIT;
	}
	if (Size > COAP_MESSAGE_SIZE)
	{
		Size = COAP_MESSAGE_SIZE;
	}
	total = 4U + pMessage->TokenLen;
	if (total > Size)
	{
		return COAP_ERROR_LIMIT;
	}
	for (i = 0; i < pMessage->OptionCount; i++)
	{
		option = pMessage->Options + i;
		if (option->Number < previous || (option->Len && !option->pValue))
		{
			return COAP_ERROR_CONFIG;
		}
		if (option->Len > COAP_MESSAGE_SIZE)
		{
			return COAP_ERROR_LIMIT;
		}
		delta = option->Number - previous;
		count = 1U + Extra(delta) + Extra(option->Len) + option->Len;
		if (count > Size - total)
		{
			return COAP_ERROR_LIMIT;
		}
		total += count;
		previous = option->Number;
	}
	if (pMessage->PayloadLen)
	{
		if (total == Size || pMessage->PayloadLen > Size - total - 1U)
		{
			return COAP_ERROR_LIMIT;
		}
		total += 1U + pMessage->PayloadLen;
	}
	pData[0] = (uint8_t)(0x40U | ((unsigned)pMessage->Type << 4) | pMessage->TokenLen);
	pData[1] = pMessage->Code;
	Put16(pData + 2, pMessage->MessageId);
	offset = 4;
	if (pMessage->TokenLen)
	{
		memcpy(pData + offset, pMessage->pToken, pMessage->TokenLen);
		offset += pMessage->TokenLen;
	}
	previous = 0;
	for (i = 0; i < pMessage->OptionCount; i++)
	{
		option = pMessage->Options + i;
		delta = option->Number - previous;
		pData[offset++] = (uint8_t)((Nibble(delta) << 4) | Nibble(option->Len));
		offset += PutExtended(pData + offset, delta);
		offset += PutExtended(pData + offset, option->Len);
		if (option->Len)
		{
			memcpy(pData + offset, option->pValue, option->Len);
			offset += option->Len;
		}
		previous = option->Number;
	}
	if (pMessage->PayloadLen)
	{
		pData[offset++] = 0xff;
		memcpy(pData + offset, pMessage->pPayload, pMessage->PayloadLen);
	}
	*pLen = total;
	return COAP_ERROR_NONE;
}

COAP_ERROR CoapDecode(CoapMessage_t *pMessage, const uint8_t *pData, size_t Len)
{
	CoapMessage_t message = {};
	size_t offset;
	size_t delta;
	size_t length;
	size_t number = 0;
	uint8_t byte;

	if (!pMessage)
	{
		return COAP_ERROR_CONFIG;
	}
	*pMessage = {};
	if (!pData || Len < 4)
	{
		return COAP_ERROR_PROTOCOL;
	}
	if (Len > COAP_MESSAGE_SIZE)
	{
		return COAP_ERROR_LIMIT;
	}
	if ((pData[0] >> 6) != 1 || (pData[0] & 15U) > COAP_TOKEN_SIZE)
	{
		return COAP_ERROR_PROTOCOL;
	}
	message.Type = (COAP_TYPE)((pData[0] >> 4) & 3U);
	message.Code = pData[1];
	message.MessageId = Get16(pData + 2);
	message.TokenLen = pData[0] & 15U;
	offset = 4U + message.TokenLen;
	if (offset > Len)
	{
		return COAP_ERROR_PROTOCOL;
	}
	message.pToken = pData + 4;
	while (offset < Len)
	{
		byte = pData[offset++];
		if (byte == 0xff)
		{
			if (offset == Len)
			{
				return COAP_ERROR_PROTOCOL;
			}
			message.pPayload = pData + offset;
			message.PayloadLen = Len - offset;
			break;
		}
		if (!Extended(pData, Len, &offset, byte >> 4, &delta) ||
			!Extended(pData, Len, &offset, byte & 15U, &length) ||
			length > Len - offset)
		{
			return COAP_ERROR_PROTOCOL;
		}
		if (delta > 65535U - number || message.OptionCount == COAP_MAX_OPTIONS)
		{
			return COAP_ERROR_LIMIT;
		}
		number += delta;
		message.Options[message.OptionCount++] =
			{(uint16_t)number, pData + offset, length};
		offset += length;
	}
	if (!Header(message.Type, message.Code, message.TokenLen,
				message.OptionCount, message.PayloadLen))
	{
		return COAP_ERROR_PROTOCOL;
	}
	*pMessage = message;
	return COAP_ERROR_NONE;
}

void CoapReset(CoapDev_t *pDev)
{
	if (pDev)
	{
		memset(pDev, 0, sizeof(*pDev));
	}
}

bool CoapInit(CoapDev_t *pDev, const CoapCfg_t *pCfg, DevIntrf_t *pTransport,
			  uint32_t Now)
{
	uint8_t seed[2];

	if (!pDev)
	{
		return false;
	}
	CoapReset(pDev);
	if (!pCfg || !pTransport || !pCfg->pRng || pCfg->pRng->IsAsync() ||
		!pCfg->Timeout || pCfg->Timeout > 0x7fffffffU || pTransport->MaxRetry != 0 ||
		!pTransport->StartTx || !pTransport->StopTx || !pTransport->TxData ||
		!pTransport->StartRx || !pTransport->StopRx || !pTransport->RxData)
	{
		Fail(pDev, COAP_ERROR_CONFIG);
		return false;
	}
	if (pCfg->pRng->Random(seed, sizeof(seed)) != CRYPTO_STATUS_OK)
	{
		Fail(pDev, COAP_ERROR_RANDOM);
		return false;
	}
	pDev->pTransport = pTransport;
	pDev->pRng = pCfg->pRng;
	pDev->Timeout = pCfg->Timeout;
	pDev->ProbingRate = pCfg->ProbingRate ? pCfg->ProbingRate : 1U;
	pDev->NextId = Get16(seed);
	pDev->LastId = Now;
	pDev->State = COAP_READY;
	return true;
}

bool CoapBusy(const CoapDev_t *pDev)
{
	return !pDev || pDev->State != COAP_READY || pDev->bControl;
}

uint32_t CoapRetryAfter(const CoapDev_t *pDev, uint32_t Now)
{
	uint32_t elapsed;

	if (!pDev || !pDev->ProbeDelay)
	{
		return 0;
	}
	elapsed = (uint32_t)(Now - pDev->ProbeStarted);
	return elapsed < pDev->ProbeDelay ? pDev->ProbeDelay - elapsed : 0;
}

bool CoapRequest(CoapDev_t *pDev, const CoapRequest_t *pRequest, uint32_t Now)
{
	CoapMessage_t message = {};
	COAP_ERROR error;
	size_t i;
	size_t length;
	uint8_t random[COAP_TOKEN_SIZE + 2U];

	if (CoapBusy(pDev))
	{
		return false;
	}
	if (CoapRetryAfter(pDev, Now))
	{
		pDev->Error = COAP_ERROR_RATE;
		return false;
	}
	if (!pRequest || (pRequest->Type != COAP_CON && pRequest->Type != COAP_NON) ||
		pRequest->Code < COAP_GET || pRequest->Code > COAP_DELETE ||
		(pRequest->OptionCount && !pRequest->pOptions))
	{
		pDev->Error = COAP_ERROR_CONFIG;
		return false;
	}
	if (pRequest->OptionCount > COAP_MAX_OPTIONS || pDev->IdCount == 65536U)
	{
		// Conservatively wait a full quiet lifetime before cycling all 65536 IDs.
		if (pRequest->OptionCount > COAP_MAX_OPTIONS ||
			(uint32_t)(Now - pDev->LastId) < COAP_EXCHANGE_LIFETIME)
		{
			pDev->Error = COAP_ERROR_LIMIT;
			return false;
		}
		pDev->IdCount = 0;
	}
	message.Type = pRequest->Type;
	message.Code = pRequest->Code;
	message.MessageId = pDev->NextId;
	message.pToken = pDev->Token;
	message.TokenLen = COAP_TOKEN_SIZE;
	message.OptionCount = pRequest->OptionCount;
	message.pPayload = pRequest->pPayload;
	message.PayloadLen = pRequest->PayloadLen;
	for (i = 0; i < message.OptionCount; i++)
	{
		message.Options[i] = pRequest->pOptions[i];
	}
	error = CoapEncode(&message, pDev->Data, sizeof(pDev->Data), &length);
	if (error == COAP_ERROR_NONE)
	{
		error = Options(&message, false);
	}
	if (error != COAP_ERROR_NONE)
	{
		pDev->Error = error;
		return false;
	}
	if (pDev->pRng->Random(random, sizeof(random)) != CRYPTO_STATUS_OK)
	{
		pDev->Error = COAP_ERROR_RANDOM;
		return false;
	}
	memcpy(pDev->Token, random, COAP_TOKEN_SIZE);
	memcpy(pDev->Data + 4, random, COAP_TOKEN_SIZE);
	pDev->RequestId = pDev->NextId++;
	pDev->IdCount++;
	pDev->LastId = Now;
	pDev->RetryDelay = COAP_ACK_TIMEOUT + ((uint32_t)Get16(random + COAP_TOKEN_SIZE) * 1001U >> 16);
	pDev->Retransmit = 0;
	pDev->ProbeDelay = 0;
	pDev->bConfirmable = pRequest->Type == COAP_CON;
	pDev->bSent = false;
	pDev->DataLen = length;
	pDev->Message = {};
	pDev->Started = Now;
	pDev->State = COAP_SENDING;
	pDev->Error = COAP_ERROR_NONE;
	return true;
}

void CoapRelease(CoapDev_t *pDev)
{
	if (!pDev || (pDev->State != COAP_RESPONSE && pDev->State != COAP_FAILED))
	{
		return;
	}
	pDev->DataLen = 0;
	pDev->bSent = false;
	pDev->Message = {};
	pDev->Error = COAP_ERROR_NONE;
	pDev->State = pDev->pTransport ? COAP_READY : COAP_IDLE;
}

void CoapCancel(CoapDev_t *pDev)
{
	if (!pDev || !pDev->pTransport)
	{
		return;
	}
	pDev->DataLen = 0;
	pDev->bSent = false;
	pDev->Message = {};
	pDev->Error = COAP_ERROR_CANCELLED;
	pDev->State = COAP_READY;
}

COAP_STATE CoapProcess(CoapDev_t *pDev, uint32_t Now, bool bTransportOpen)
{
	int count;
	uint32_t age;
	uint32_t initial;
	bool transmit = false;

	if (!pDev)
	{
		return COAP_FAILED;
	}
	if (!pDev->pTransport || pDev->State == COAP_IDLE)
	{
		return pDev->State;
	}
	if (!bTransportOpen)
	{
		return Fail(pDev, COAP_ERROR_TRANSPORT);
	}
	if (Active(pDev) && (uint32_t)(Now - pDev->Started) >= pDev->Timeout)
	{
		Fail(pDev, COAP_ERROR_TIMEOUT);
	}
	if (pDev->bControl && (uint32_t)(Now - pDev->ControlStarted) >= pDev->Timeout)
	{
		pDev->bControl = false;
	}
	// Hold one unsent control packet under backpressure. Do not overwrite it by
	// consuming another incoming datagram before it has been sent or expired.
	if (!pDev->bControl && pDev->State != COAP_FAILED)
	{
		count = Receive(pDev);
		if (count < 0 || (size_t)count > sizeof(pDev->Rx))
		{
			return Fail(pDev, COAP_ERROR_TRANSPORT);
		}
		if (count)
		{
			Packet(pDev, (size_t)count, Now);
		}
	}
	if (pDev->bControl)
	{
		count = Send(pDev, pDev->Control, sizeof(pDev->Control));
		if (count < 0 || (count && count != (int)sizeof(pDev->Control)))
		{
			return Fail(pDev, COAP_ERROR_TRANSPORT);
		}
		if (count)
		{
			pDev->bControl = false;
		}
		return pDev->State;
	}
	if (pDev->State == COAP_SENDING)
	{
		transmit = true;
	}
	else if (pDev->State == COAP_WAIT_ACK)
	{
		age = (uint32_t)(Now - pDev->FirstSent);
		initial = pDev->RetryDelay >> pDev->Retransmit;
		if (age >= initial * 31U)
		{
			return Fail(pDev, COAP_ERROR_TIMEOUT);
		}
		transmit = pDev->Retransmit < COAP_MAX_RETRANSMIT &&
			age <= COAP_MAX_TRANSMIT_SPAN &&
			(uint32_t)(Now - pDev->LastSent) >= pDev->RetryDelay;
	}
	if (transmit)
	{
		count = Send(pDev, pDev->Data, pDev->DataLen);
		if (count < 0 || (count && (size_t)count != pDev->DataLen))
		{
			return Fail(pDev, COAP_ERROR_TRANSPORT);
		}
		if (count)
		{
			if (!pDev->ProbeDelay)
			{
				pDev->ProbeStarted = Now;
			}
			// At most five sends of at most 1152 bytes per exchange. Even the
			// default one-byte/second rate fits a wrapping uint32_t ms deadline.
			pDev->ProbeDelay += ((uint32_t)pDev->DataLen * 1000U - 1U) /
				pDev->ProbingRate + 1U;
			if (pDev->bSent)
			{
				pDev->Retransmit++;
				pDev->RetryDelay *= 2U;
			}
			else
			{
				pDev->FirstSent = Now;
				pDev->bSent = true;
			}
			pDev->LastSent = Now;
			pDev->LastId = Now;
			pDev->State = pDev->bConfirmable ? COAP_WAIT_ACK : COAP_WAIT_RESPONSE;
		}
	}
	return pDev->State;
}
