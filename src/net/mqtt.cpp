/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * MQTT 3.1.1, OASIS standard plus Errata 01. QoS 0/1, clean sessions.
 */
#include <string.h>
#include "net/mqtt.h"

namespace
{
void Wipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len--)
	{
		*p++ = 0;
	}
}
MQTT_STATE Fail(MqttDev_t *d, MQTT_ERROR Error)
{
	Wipe(d->Tx, sizeof(d->Tx));
	Wipe(d->Rx, sizeof(d->Rx));
	d->Message = {};
	d->bMessage = false;
	d->State = MQTT_FAILED;
	d->Error = Error;
	return d->State;
}
size_t Length(const char *s)
{
	size_t n = 0;
	if (!s)
	{
		return 0;
	}
	while (n <= MQTT_PACKET_SIZE && s[n])
	{
		n++;
	}
	return n;
}
bool Utf8(const uint8_t *s, size_t n)
{
	uint32_t c;
	uint32_t minimum;
	unsigned extra;
	size_t i;
	for (i = 0; i < n;)
	{
		c = s[i++];
		extra = 0;
		minimum = 0;
		if (c >= 0xc2 && c <= 0xdf)
		{
			c &= 31;
			extra = 1;
			minimum = 0x80;
		}
		else if (c >= 0xe0 && c <= 0xef)
		{
			c &= 15;
			extra = 2;
			minimum = 0x800;
		}
		else if (c >= 0xf0 && c <= 0xf4)
		{
			c &= 7;
			extra = 3;
			minimum = 0x10000;
		}
		else if (c >= 0x80)
		{
			return false;
		}
		if (extra > n - i)
		{
			return false;
		}
		while (extra--)
		{
			if ((s[i] & 0xc0) != 0x80)
			{
				return false;
			}
			c = (c << 6) | (s[i++] & 63);
		}
		if (c < minimum || c == 0 || c > 0x10ffff || (c >= 0xd800 && c <= 0xdfff))
		{
			return false;
		}
	}
	return true;
}
bool Topic(const uint8_t *p, size_t n, bool Filter)
{
	size_t i;

	if (!p || n == 0 || n > MQTT_PACKET_SIZE || !Utf8(p, n))
	{
		return false;
	}
	for (i = 0; i < n; i++)
	{
		if (p[i] == '#' && (!Filter || i + 1 != n || (i && p[i - 1] != '/')))
		{
			return false;
		}
		if (p[i] == '+' && (!Filter || (i && p[i - 1] != '/') || (i + 1 < n && p[i + 1] != '/')))
		{
			return false;
		}
	}
	return true;
}
void Put16(uint8_t *p, size_t n)
{
	p[0] = (uint8_t)(n >> 8);
	p[1] = (uint8_t)n;
}
uint16_t Get16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}
void String(uint8_t *p, const void *s, size_t n)
{
	Put16(p, n);
	if (n)
	{
		memcpy(p + 2, s, n);
	}
}
void Queue(MqttDev_t *d, uint8_t Type, size_t n, uint32_t Now)
{
	uint8_t header[5];
	size_t h = 1;
	size_t len = n;
	header[0] = Type;
	do
	{
		header[h] = (uint8_t)(len % 128);
		len /= 128;
		if (len)
		{
			header[h] |= 128;
		}
		h++;
	} while (len);
	memmove(d->Tx + h, d->Tx + 5, n);
	memcpy(d->Tx, header, h);
	d->TxLen = h + n;
	d->TxOffset = 0;
	d->TxType = Type;
	d->TxStarted = Now;
}
void Pending(MqttDev_t *d, uint8_t Type, uint32_t Now)
{
	d->PendingId = ++d->NextId;
	if (!d->PendingId)
	{
		d->PendingId = ++d->NextId;
	}
	d->PendingType = Type;
	d->bPendingSent = false;
	d->Started = Now;
	d->Result = 0;
}
bool Packet(MqttDev_t *d)
{
	const uint8_t *p = d->Rx + d->HeaderLen;
	size_t n = d->RxNeed - d->HeaderLen;
	size_t topiclen;
	size_t offset;
	uint16_t id;
	uint8_t type = d->Rx[0];
	uint8_t qos = (type >> 1) & 3;
	if (d->State == MQTT_CONNECTING)
	{
		if (type != 0x20 || n != 2 || p[0] != 0 || p[1] > 5 || d->TxLen)
		{
			Fail(d, MQTT_ERROR_PROTOCOL);
			return false;
		}
		d->Result = p[1];
		if (p[1])
		{
			Fail(d, MQTT_ERROR_REFUSED);
			return false;
		}
		d->PendingType = 0;
		d->State = MQTT_READY;
		return true;
	}
	if ((type >> 4) == 3)
	{
		if (qos > 1 || (qos == 0 && (type & 8)) || n < 2)
		{
			Fail(d, MQTT_ERROR_PROTOCOL);
			return false;
		}
		topiclen = Get16(p);
		offset = 2 + topiclen;
		if (offset > n || !Topic(p + 2, topiclen, false) || (qos && n - offset < 2))
		{
			Fail(d, MQTT_ERROR_PROTOCOL);
			return false;
		}
		if (qos)
		{
			id = Get16(p + offset);
			if (!id)
			{
				Fail(d, MQTT_ERROR_PROTOCOL);
				return false;
			}
			d->AckId = id;
			offset += 2;
		}
		d->Message = {p + 2, topiclen, p + offset, n - offset, qos, (bool)(type & 1), (bool)(type & 8)};
		d->bMessage = true;
		return true;
	}
	if (type == 0xd0 && n == 0 && d->bPing)
	{
		d->bPing = false;
		return true;
	}
	if ((type == 0x40 || type == 0x90 || type == 0xb0) && n == (type == 0x90 ? 3U : 2U) &&
		d->PendingType == (type >> 4) && Get16(p) == d->PendingId && d->bPendingSent)
	{
		if (type == 0x90 && p[2] != 0x80 && p[2] > d->RequestedQos)
		{
			Fail(d, MQTT_ERROR_PROTOCOL);
			return false;
		}
		d->Result = type == 0x90 ? p[2] : 0;
		d->PendingType = 0;
		return true;
	}
	Fail(d, MQTT_ERROR_PROTOCOL);
	return false;
}
} // namespace

void MqttReset(MqttDev_t *d)
{
	if (d)
	{
		Wipe(d, sizeof(*d));
	}
}
bool MqttBusy(const MqttDev_t *d)
{
	return !d || d->State != MQTT_READY || d->TxLen || d->PendingType || d->AckId || d->bPing;
}
bool MqttInit(MqttDev_t *d, const MqttCfg_t *cfg, DevIntrf_t *transport, uint32_t now)
{
	size_t client;
	size_t user;
	size_t n;
	uint8_t *p;
	if (!d)
	{
		return false;
	}
	MqttReset(d);
	if (!cfg || !transport || !transport->StartTx || !transport->StopTx || !transport->TxData ||
		!transport->StartRx || !transport->StopRx || !transport->RxData || !cfg->Timeout ||
		cfg->Timeout > 0x7fffffffU)
	{
		Fail(d, MQTT_ERROR_CONFIG);
		return false;
	}
	client = Length(cfg->pClientId);
	user = Length(cfg->pUsername);
	if (!client || client > MQTT_PACKET_SIZE || user > MQTT_PACKET_SIZE ||
		cfg->PasswordLen > MQTT_PACKET_SIZE || (cfg->PasswordLen && !cfg->pPassword) ||
		(cfg->pPassword && !cfg->pUsername) || !Utf8((const uint8_t *)cfg->pClientId, client) ||
		!Utf8((const uint8_t *)cfg->pUsername, user))
	{
		Fail(d, MQTT_ERROR_CONFIG);
		return false;
	}
	n = 12 + client + (cfg->pUsername ? 2 + user : 0) + (cfg->pPassword ? 2 + cfg->PasswordLen : 0);
	if (n > MQTT_PACKET_SIZE)
	{
		Fail(d, MQTT_ERROR_LIMIT);
		return false;
	}
	d->pTransport = transport;
	d->Timeout = cfg->Timeout;
	d->KeepAlive = cfg->KeepAlive;
	d->Now = d->Started = d->LastTx = now;
	d->RxNeed = 2;
	d->State = MQTT_CONNECTING;
	d->PendingType = 2;
	p = d->Tx + 5;
	memcpy(p, "\0\4MQTT\4", 7);
	p[7] = 2 | (cfg->pUsername ? 128 : 0) | (cfg->pPassword ? 64 : 0);
	Put16(p + 8, cfg->KeepAlive);
	String(p + 10, cfg->pClientId, client);
	p += 12 + client;
	if (cfg->pUsername)
	{
		String(p, cfg->pUsername, user);
		p += 2 + user;
	}
	if (cfg->pPassword)
	{
		String(p, cfg->pPassword, cfg->PasswordLen);
	}
	Queue(d, 0x10, n, now);
	return true;
}
bool MqttPublish(MqttDev_t *d, const char *topic, const uint8_t *data, size_t len, uint8_t qos, bool retain,
				 uint32_t now)
{
	size_t n = Length(topic);
	size_t offset = 2 + n + (qos ? 2 : 0);
	if (MqttBusy(d) || qos > 1 || !Topic((const uint8_t *)topic, n, false) || offset > MQTT_PACKET_SIZE ||
		len > MQTT_PACKET_SIZE - offset || (len && !data))
	{
		return false;
	}
	String(d->Tx + 5, topic, n);
	if (qos)
	{
		Pending(d, 4, now);
		Put16(d->Tx + 7 + n, d->PendingId);
	}
	if (len)
	{
		memcpy(d->Tx + 5 + offset, data, len);
	}
	Queue(d, (uint8_t)(0x30 | (qos << 1) | retain), offset + len, now);
	return true;
}
bool MqttSubscribe(MqttDev_t *d, const char *filter, uint8_t qos, uint32_t now)
{
	size_t n = Length(filter);
	if (MqttBusy(d) || qos > 1 || n > MQTT_PACKET_SIZE - 5 || !Topic((const uint8_t *)filter, n, true))
	{
		return false;
	}
	Pending(d, 9, now);
	d->RequestedQos = qos;
	Put16(d->Tx + 5, d->PendingId);
	String(d->Tx + 7, filter, n);
	d->Tx[9 + n] = qos;
	Queue(d, 0x82, n + 5, now);
	return true;
}
bool MqttUnsubscribe(MqttDev_t *d, const char *filter, uint32_t now)
{
	size_t n = Length(filter);
	if (MqttBusy(d) || n > MQTT_PACKET_SIZE - 4 || !Topic((const uint8_t *)filter, n, true))
	{
		return false;
	}
	Pending(d, 11, now);
	Put16(d->Tx + 5, d->PendingId);
	String(d->Tx + 7, filter, n);
	Queue(d, 0xa2, n + 4, now);
	return true;
}
void MqttRelease(MqttDev_t *d)
{
	if (!d || !d->bMessage)
	{
		return;
	}
	Wipe(d->Rx, sizeof(d->Rx));
	d->Message = {};
	d->bMessage = false;
	d->RxCount = 0;
	d->RxNeed = 2;
	d->HeaderLen = 0;
}
bool MqttDisconnect(MqttDev_t *d, uint32_t now)
{
	if (MqttBusy(d) || d->bMessage)
	{
		return false;
	}
	Queue(d, 0xe0, 0, now);
	d->State = MQTT_CLOSING;
	return true;
}
MQTT_STATE MqttProcess(MqttDev_t *d, uint32_t now, bool open)
{
	int count;
	uint32_t remaining = 0;
	uint32_t factor = 1;
	uint8_t digit;
	size_t i;
	if (!d)
	{
		return MQTT_FAILED;
	}
	if (d->State == MQTT_IDLE || d->State == MQTT_CLOSED || d->State == MQTT_FAILED)
	{
		return d->State;
	}
	d->Now = now;
	if ((d->PendingType && (uint32_t)(now - d->Started) >= d->Timeout) ||
		(d->TxLen && (uint32_t)(now - d->TxStarted) >= d->Timeout) ||
		(d->bPing && (uint32_t)(now - d->PingStarted) >= d->Timeout) ||
		(!d->bMessage && d->RxCount && (uint32_t)(now - d->RxStarted) >= d->Timeout))
	{
		return Fail(d, MQTT_ERROR_TIMEOUT);
	}
	if (d->TxLen == 0 && d->State == MQTT_READY)
	{
		if (d->AckId && !d->bMessage)
		{
			Put16(d->Tx + 5, d->AckId);
			Queue(d, 0x40, 2, now);
			d->AckId = 0;
		}
		else if (d->KeepAlive && !d->bPing && (uint32_t)(now - d->LastTx) >= d->KeepAlive * 1000U)
		{
			Queue(d, 0xc0, 0, now);
			d->bPing = true;
			d->PingStarted = now;
		}
	}
	if (d->TxLen)
	{
		if (!open)
		{
			return Fail(d, MQTT_ERROR_TRANSPORT);
		}
		count = DeviceIntrfTx(d->pTransport, 0, d->Tx + d->TxOffset, (int)(d->TxLen - d->TxOffset));
		if (count < 0 || (size_t)count > d->TxLen - d->TxOffset)
		{
			return Fail(d, MQTT_ERROR_TRANSPORT);
		}
		d->TxOffset += (size_t)count;
		if (d->TxOffset == d->TxLen)
		{
			Wipe(d->Tx, sizeof(d->Tx));
			d->TxLen = d->TxOffset = 0;
			d->LastTx = now;
			if (d->TxType == 0x82 || d->TxType == 0xa2 || (d->TxType & 0xf6) == 0x32)
			{
				d->bPendingSent = true;
			}
			if (d->State == MQTT_CLOSING)
			{
				d->State = MQTT_CLOSED;
				return d->State;
			}
		}
	}
	// Do not overwrite a deferred PUBACK while another packet occupies TX.
	if (d->bMessage || d->AckId)
	{
		return open ? d->State : Fail(d, MQTT_ERROR_TRANSPORT);
	}
	count = DeviceIntrfRx(d->pTransport, 0, d->Rx + d->RxCount, (int)(d->RxNeed - d->RxCount));
	if (count < 0 || (size_t)count > d->RxNeed - d->RxCount || (!count && !open))
	{
		return Fail(d, MQTT_ERROR_TRANSPORT);
	}
	if (count && !d->RxCount)
	{
		d->RxStarted = now;
	}
	d->RxCount += (size_t)count;
	if (d->RxCount != d->RxNeed)
	{
		return d->State;
	}
	if (!d->HeaderLen)
	{
		for (i = 1; i < d->RxCount; i++)
		{
			digit = d->Rx[i];
			remaining += (digit & 127U) * factor;
			factor *= 128;
		}
		if (remaining > MQTT_PACKET_SIZE)
		{
			return Fail(d, MQTT_ERROR_LIMIT);
		}
		if (d->Rx[d->RxCount - 1] & 128)
		{
			if (d->RxCount == 5)
			{
				return Fail(d, MQTT_ERROR_PROTOCOL);
			}
			d->RxNeed++;
			return d->State;
		}
		if (d->RxCount > 2 && d->Rx[d->RxCount - 1] == 0)
		{
			return Fail(d, MQTT_ERROR_PROTOCOL);
		}
		d->HeaderLen = d->RxCount;
		d->RxNeed += remaining;
	}
	if (d->RxCount == d->RxNeed && Packet(d) && !d->bMessage)
	{
		Wipe(d->Rx, sizeof(d->Rx));
		d->RxCount = 0;
		d->RxNeed = 2;
		d->HeaderLen = 0;
	}
	return d->State;
}
