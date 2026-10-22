/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * MQTT 3.1.1 clean-session client over a connected synchronous stream.
 */
#ifndef __NET_MQTT_H__
#define __NET_MQTT_H__
#include <stddef.h>
#include "device_intrf.h"

#define MQTT_PACKET_SIZE 1024U

typedef enum
{
	MQTT_IDLE,
	MQTT_CONNECTING,
	MQTT_READY,
	MQTT_CLOSING,
	MQTT_CLOSED,
	MQTT_FAILED
} MQTT_STATE;
typedef enum
{
	MQTT_ERROR_NONE,
	MQTT_ERROR_CONFIG,
	MQTT_ERROR_TRANSPORT,
	MQTT_ERROR_PROTOCOL,
	MQTT_ERROR_LIMIT,
	MQTT_ERROR_TIMEOUT,
	MQTT_ERROR_REFUSED
} MQTT_ERROR;
typedef struct
{
	const char *pClientId; /* Nonempty UTF-8, unique at the broker. */
	const char *pUsername; /* Optional; password requires a username. */
	const uint8_t *pPassword;
	size_t PasswordLen;
	uint16_t KeepAlive; /* Seconds; zero disables keepalive. */
	uint32_t Timeout;	/* Connect, output, receive and acknowledgement deadline, ms. */
} MqttCfg_t;
typedef struct
{
	const uint8_t *pTopic; /* Not NUL terminated. */
	size_t TopicLen;
	const uint8_t *pPayload;
	size_t PayloadLen;
	uint8_t Qos;
	bool bRetain;
	bool bDuplicate;
} MqttMessage_t;
/* Zero-initialize. One foreground owner, no concurrent calls or callbacks.
 * No allocation. Receive view is valid until MqttRelease/Reset/failure.
 * One outgoing acknowledged operation and one received message at a time.
 */
typedef struct
{
	DevIntrf_t *pTransport;
	MQTT_STATE State;
	MQTT_ERROR Error;
	MqttMessage_t Message;
	uint32_t Now, Timeout, Started, TxStarted, RxStarted, LastTx, PingStarted;
	uint16_t KeepAlive, NextId, PendingId, AckId;
	uint8_t PendingType, RequestedQos, Result, TxType;
	bool bMessage, bPing, bPendingSent;
	size_t TxLen, TxOffset, RxCount, RxNeed, HeaderLen;
	uint8_t Tx[MQTT_PACKET_SIZE + 5U];
	uint8_t Rx[MQTT_PACKET_SIZE + 5U];
} MqttDev_t;
#ifdef __cplusplus
extern "C"
{
#endif
/* Init copies credentials into CONNECT. Transport is already open; TLS, if used,
 * must already be authenticated. Set transport MaxRetry=0 for bounded polling.
 */
bool MqttInit(MqttDev_t *pDev, const MqttCfg_t *pCfg, DevIntrf_t *pTransport, uint32_t Now);
MQTT_STATE MqttProcess(MqttDev_t *pDev, uint32_t Now, bool bTransportOpen);
bool MqttPublish(MqttDev_t *pDev, const char *pTopic, const uint8_t *pData, size_t Len, uint8_t Qos,
				 bool bRetain, uint32_t Now);
bool MqttSubscribe(MqttDev_t *pDev, const char *pFilter, uint8_t Qos, uint32_t Now);
bool MqttUnsubscribe(MqttDev_t *pDev, const char *pFilter, uint32_t Now);
bool MqttDisconnect(MqttDev_t *pDev, uint32_t Now);
void MqttReset(MqttDev_t *pDev);
void MqttRelease(MqttDev_t *pDev);
bool MqttBusy(const MqttDev_t *pDev);
#ifdef __cplusplus
}
#include "device.h"

class Mqtt : public Device
{
public:
	Mqtt() = default;
	~Mqtt()
	{
		MqttReset(&vDev);
	}
	Mqtt(const Mqtt &) = delete;
	Mqtt &operator=(const Mqtt &) = delete;
	bool Init(const MqttCfg_t &Cfg, DeviceIntrf *pIntrf, uint32_t Now)
	{
		Interface(pIntrf);
		vbEnabled = MqttInit(&vDev, &Cfg, pIntrf ? (DevIntrf_t *)*pIntrf : nullptr, Now);
		Valid(false);
		return vbEnabled;
	}
	bool Enable() override
	{
		vbEnabled = vDev.pTransport != nullptr;
		return vbEnabled;
	}
	// Pause the foreground pump; deadlines continue to elapse while paused.
	void Disable() override
	{
		vbEnabled = false;
	}
	void Reset() override
	{
		MqttReset(&vDev);
		vbEnabled = false;
		Valid(false);
	}
	void PowerOff() override
	{
		Reset();
	}
	MQTT_STATE Process(uint32_t Now, bool Open)
	{
		if (vbEnabled)
		{
			MqttProcess(&vDev, Now, Open);
		}
		Valid(vDev.State == MQTT_READY);
		return vDev.State;
	}
	bool Connected() const
	{
		return vDev.State == MQTT_READY;
	}
	bool Busy() const
	{
		return MqttBusy(&vDev);
	}
	MQTT_ERROR Error() const
	{
		return vDev.Error;
	}
	// Last CONNACK/SUBACK result. SUBACK 0x80 means denied, not disconnected.
	uint8_t Result() const
	{
		return vDev.Result;
	}
	bool Publish(const char *Topic, const uint8_t *Data, size_t Len, uint8_t Qos, bool Retain, uint32_t Now)
	{
		return MqttPublish(&vDev, Topic, Data, Len, Qos, Retain, Now);
	}
	bool Subscribe(const char *Filter, uint8_t Qos, uint32_t Now)
	{
		return MqttSubscribe(&vDev, Filter, Qos, Now);
	}
	bool Unsubscribe(const char *Filter, uint32_t Now)
	{
		return MqttUnsubscribe(&vDev, Filter, Now);
	}
	const MqttMessage_t *Message() const
	{
		return vDev.bMessage ? &vDev.Message : nullptr;
	}
	void Release()
	{
		MqttRelease(&vDev);
	}
	bool Disconnect(uint32_t Now)
	{
		return MqttDisconnect(&vDev, Now);
	}

private:
	MqttDev_t vDev = {};
	bool vbEnabled = false;
};
#endif
#endif
