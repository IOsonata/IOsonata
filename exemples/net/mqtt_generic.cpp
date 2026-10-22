/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * MQTT QoS 1 echo through an injected stream DeviceIntrf.
 * The selected medium is initialized by the example's net_demo.cpp.
 */
#include <string.h>
#include "app_evt_handler.h"
#include "net/mqtt.h"
#include "syslog.h"
#include "net_demo.h"
#include "board.h"

#define MQTT_DEMO_RESTART_MS	30000U
#define MQTT_DEMO_TIMEOUT_MS	30000U

static_assert(MQTT_DEMO_INTERVAL >= 1000U && MQTT_DEMO_INTERVAL <= 2147483647U,
	"MQTT_DEMO_INTERVAL must be 1000..INT32_MAX milliseconds");
static_assert(MQTT_DEMO_PORT > 0 && MQTT_DEMO_PORT <= 65535U,
	"MQTT_DEMO_PORT must be 1..65535");

static const char s_Hex[] = "0123456789abcdef";
static Mqtt s_Mqtt;
static DeviceIntrf *s_pIntrf;
static char s_ClientId[23];
static char s_Topic[36];
static uint8_t s_Payload[] = "IOsonata MQTT 00000000";
static uint32_t s_Sequence;
static uint32_t s_Checked;
static uint32_t s_Failures;
static uint32_t s_ExtraQos0;
static uint32_t s_SentAt;
static uint32_t s_Retry;
static bool s_Started;
static bool s_SubscribeSent;
static bool s_Subscribed;
static bool s_Waiting;
static bool s_EchoReceived;
static bool s_SessionChecked;
static bool s_RecoveryWaiting;

static bool CreateIdentity()
{
	RngEngine *rng = NetDemoRng();
	uint8_t random[8];
	size_t i;

	if (!rng || rng->Random(random, sizeof(random)) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	// Keep this boot's identity when reopening or changing the interface.
	memcpy(s_ClientId, "iosnet", 6);
	for (i = 0; i < sizeof(random); i++)
	{
		s_ClientId[6 + i * 2] = s_Hex[random[i] >> 4];
		s_ClientId[7 + i * 2] = s_Hex[random[i] & 15U];
	}
	s_ClientId[22] = 0;
	memcpy(s_Topic, "iosonata/demo/", 14);
	memcpy(s_Topic + 14, s_ClientId + 6, 16);
	memcpy(s_Topic + 30, "/echo", 6);
	return true;
}

static void MqttDown(const char *pReason)
{
	s_Failures++;
	SysLogPrintf(SysLogGet(), "MQTT FAIL: %s, mqtt_error=%u mqtt_result=%u checked=%lu failures=%lu\r\n",
		pReason, (unsigned)s_Mqtt.Error(), (unsigned)s_Mqtt.Result(),
		(unsigned long)s_Checked, (unsigned long)s_Failures);
	s_Mqtt.Reset();
	NetDemoClose();
	s_pIntrf = nullptr;
	s_Started = false;
	s_SubscribeSent = false;
	s_Subscribed = false;
	s_Waiting = false;
	s_EchoReceived = false;
	s_SessionChecked = false;
	// Closing the previous endpoint can block. Start backoff after it returns.
	s_Retry = NetDemoNow();
	s_RecoveryWaiting = true;
	SysLogPrintf(SysLogGet(), "MQTT reconnect in 30 s when the selected medium is ready\r\n");
}

static bool MatchesPayload(const MqttMessage_t *pMessage)
{
	return pMessage->PayloadLen == sizeof(s_Payload) - 1U &&
		memcmp(pMessage->pPayload, s_Payload, sizeof(s_Payload) - 1U) == 0;
}

static bool RejectMessage(const MqttMessage_t *pMessage, const char *pReason)
{
	char sample[65];
	size_t count = pMessage->PayloadLen;
	size_t i;

	if (count > (sizeof(sample) - 1U) / 2U)
	{
		count = (sizeof(sample) - 1U) / 2U;
	}
	for (i = 0; i < count; i++)
	{
		sample[i * 2] = s_Hex[pMessage->pPayload[i] >> 4];
		sample[i * 2 + 1] = s_Hex[pMessage->pPayload[i] & 15U];
	}
	sample[count * 2] = 0;
	// The borrowed message is wiped by Release(). Capture its bounded byte
	// sample now; hexadecimal also keeps binary data out of terminal control.
	SysLogPrintf(SysLogGet(), "MQTT echo match=%u done=%u\r\n",
		(unsigned)MatchesPayload(pMessage), (unsigned)(!s_Waiting && s_EchoReceived));
	SysLogPrintf(SysLogGet(), "MQTT echo rejected: %s qos=%u dup=%u len=%lu expected_seq=%08lx\r\n",
		pReason, (unsigned)pMessage->Qos, (unsigned)pMessage->bDuplicate,
		(unsigned long)pMessage->PayloadLen, (unsigned long)s_Sequence);
	SysLogPrintf(SysLogGet(), "MQTT echo payload hex (%u/%lu bytes): %s\r\n",
		(unsigned)count, (unsigned long)pMessage->PayloadLen, sample);
	return false;
}

static bool CheckMessage(const MqttMessage_t *pMessage)
{
	const size_t prefix = sizeof(s_Payload) - 1U - 8U;
	uint32_t sequence = 0;
	uint8_t value;
	size_t i;

	// Retained data and messages on other topics do not verify a live echo.
	if (pMessage->bRetain || pMessage->TopicLen != sizeof(s_Topic) - 1U ||
		memcmp(pMessage->pTopic, s_Topic, sizeof(s_Topic) - 1U) != 0)
	{
		return true;
	}
	// An exact extra copy cannot undo an already verified QoS 1 exchange.
	// Keep it visible separately; it never satisfies an outstanding echo.
	if (pMessage->Qos == 0 && !s_Waiting && s_EchoReceived && MatchesPayload(pMessage))
	{
		s_ExtraQos0++;
		if (s_ExtraQos0 == 1U || s_ExtraQos0 % 10U == 0)
		{
			SysLogPrintf(SysLogGet(), "MQTT extra QoS 0 copy ignored %lu, checked=%lu\r\n",
				(unsigned long)s_ExtraQos0, (unsigned long)s_Checked);
		}
		return true;
	}
	if (pMessage->Qos != 1)
	{
		return RejectMessage(pMessage, "QoS");
	}
	if (pMessage->PayloadLen != sizeof(s_Payload) - 1U)
	{
		return RejectMessage(pMessage, "length");
	}
	if (memcmp(pMessage->pPayload, s_Payload, prefix) != 0)
	{
		return RejectMessage(pMessage, "prefix");
	}
	for (i = prefix; i < sizeof(s_Payload) - 1U; i++)
	{
		value = pMessage->pPayload[i];
		if (value >= '0' && value <= '9')
		{
			value -= '0';
		}
		else if (value >= 'a' && value <= 'f')
		{
			value = (uint8_t)(value - 'a' + 10U);
		}
		else
		{
			return RejectMessage(pMessage, "sequence encoding");
		}
		sequence = (sequence << 4) | value;
	}
	if (sequence == s_Sequence)
	{
		if (s_Waiting)
		{
			s_EchoReceived = true;
		}
		return true;
	}
	// QoS 1 permits duplicate delivery. Acknowledge old echoes without counting.
	if ((uint32_t)(s_Sequence - sequence) < 0x80000000U)
	{
		return true;
	}
	SysLogPrintf(SysLogGet(), "MQTT echo sequence received=%08lx\r\n", (unsigned long)sequence);
	return RejectMessage(pMessage, "sequence ahead");
}

static void Publish(uint32_t Now)
{
	uint32_t sequence = s_Sequence + 1U;
	uint32_t value = sequence;
	size_t i;

	for (i = 0; i < 8; i++)
	{
		s_Payload[sizeof(s_Payload) - 2U - i] = (uint8_t)s_Hex[value & 15U];
		value >>= 4;
	}
	if (!s_Mqtt.Publish(s_Topic, s_Payload, sizeof(s_Payload) - 1U, 1, false, Now))
	{
		MqttDown("publish rejected");
		return;
	}
	s_Sequence = sequence;
	s_SentAt = Now;
	s_Waiting = true;
	s_EchoReceived = false;
}

static void MqttTest(uint32_t Now)
{
	const MqttCfg_t cfg = {s_ClientId, nullptr, nullptr, 0, 60, MQTT_DEMO_TIMEOUT_MS};
	const MqttMessage_t *message;
	MQTT_STATE state;
	bool valid;

	if (s_RecoveryWaiting && (uint32_t)(Now - s_Retry) >= MQTT_DEMO_RESTART_MS)
	{
		s_RecoveryWaiting = false;
	}
	if (!s_Started)
	{
		if (s_RecoveryWaiting)
		{
			return;
		}
		SysLogPrintf(SysLogGet(), "MQTT connecting: %s:%u\r\n",
			MQTT_DEMO_HOST, (unsigned)MQTT_DEMO_PORT);
		// Just like a sensor's injected SPI/I2C interface, the protocol receives
		// the already opened interface without knowing its concrete medium.
		s_pIntrf = NetDemoOpen(false, MQTT_DEMO_HOST, MQTT_DEMO_PORT);
		Now = NetDemoNow();
		if (!s_pIntrf)
		{
			MqttDown("transport initialization");
			return;
		}
		if (!s_Mqtt.Init(cfg, s_pIntrf, Now))
		{
			MqttDown("initialization");
			return;
		}
		s_Started = true;
		s_SentAt = Now - MQTT_DEMO_INTERVAL;
		SysLogPrintf(SysLogGet(), "MQTT peer %s:%u, %s, client %s\r\n",
			MQTT_DEMO_HOST, (unsigned)MQTT_DEMO_PORT, NetDemoSecurityName(), s_ClientId);
	}
	state = s_Mqtt.Process(Now, NetDemoConnected());
	Now = NetDemoNow();
	if (state == MQTT_FAILED || state == MQTT_CLOSED)
	{
		MqttDown("session ended");
		return;
	}
	if (!s_Mqtt.Connected())
	{
		return;
	}
	message = s_Mqtt.Message();
	if (message)
	{
		valid = CheckMessage(message);
		s_Mqtt.Release();
		if (!valid)
		{
			MqttDown("echo payload or QoS mismatch");
			return;
		}
	}
	if (!s_SubscribeSent && !s_Mqtt.Busy())
	{
		if (!s_Mqtt.Subscribe(s_Topic, 1, Now))
		{
			MqttDown("subscribe rejected");
			return;
		}
		s_SubscribeSent = true;
	}
	else if (!s_Subscribed && !s_Mqtt.Busy())
	{
		if (s_Mqtt.Result() != 1)
		{
			MqttDown("broker did not grant QoS 1 subscription");
			return;
		}
		s_Subscribed = true;
		SysLogPrintf(SysLogGet(), "MQTT subscribed: %s QoS=1\r\n", s_Topic);
	}
	// Verify the broker's PUBACK and self-echo in either arrival order.
	if (s_Waiting && s_EchoReceived && !s_Mqtt.Busy())
	{
		s_Waiting = false;
		s_Checked++;
		if (!s_SessionChecked || s_Checked % 10U == 0)
		{
			SysLogPrintf(SysLogGet(), "MQTT QoS 1 echo checked %lu, failures=%lu\r\n",
				(unsigned long)s_Checked, (unsigned long)s_Failures);
		}
		s_SessionChecked = true;
	}
	if (s_Waiting && (uint32_t)(Now - s_SentAt) >= MQTT_DEMO_TIMEOUT_MS)
	{
		MqttDown("30 second PUBACK or echo timeout");
		return;
	}
	if (s_Subscribed && !s_Waiting && !s_Mqtt.Busy() &&
		(uint32_t)(Now - s_SentAt) >= MQTT_DEMO_INTERVAL)
	{
		Publish(Now);
	}
}

static void Report(uint32_t Now)
{
	if (!NetDemoReady())
	{
		if (s_Started)
		{
			MqttDown("network down");
		}
		return;
	}
	MqttTest(Now);
}

int main()
{
	const char *host = MQTT_DEMO_HOST;

	if (!NetDemoInit())
	{
		return 1;
	}
	SysLogPrintf(SysLogGet(), "Generic MQTT QoS 1 echo through %s\r\n", NetDemoMediumName());
	if (!host || !host[0])
	{
		SysLogPrintf(SysLogGet(), "Set MQTT_DEMO_HOST in board.h\r\n");
	}
	else if (!CreateIdentity())
	{
		SysLogPrintf(SysLogGet(), "MQTT random identity initialization failed\r\n");
	}
	else if (!NetDemoStart(Report))
	{
		SysLogPrintf(SysLogGet(), "MQTT example startup failed\r\n");
	}
	AppRun();
	return 0;
}
