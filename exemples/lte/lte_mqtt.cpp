/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Shared MQTT client over LTE TCP, using a unique public test topic.
 */
#include <string.h>
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "lte/lte.h"
#include "net/mqtt.h"
#include "net/sock_intrf.h"
#include "syslog.h"
#include "board.h"
#include MQTT_DEMO_RNG_HEADER

#define LTE_MQTT_TRIG_MODEM	0
#define LTE_MQTT_TRIG_REPORT	1
#define LTE_MQTT_RESTART_MS	30000U
#define LTE_MQTT_TIMEOUT_MS	30000U

static_assert(MQTT_DEMO_INTERVAL >= 1000U && MQTT_DEMO_INTERVAL <= 2147483647U,
	"MQTT_DEMO_INTERVAL must be 1000..INT32_MAX milliseconds");

static void LteEvent(LTE_EVT Evt, const LteStatus_t * const pStatus);

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(1024)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = true,
};
static const TimerCfg_t s_TimerCfg = {
	.DevNo = LTE_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = IRQ_PRIO_LOW,
};
static TimerDev_t s_Timer;
static const LteCfg_t s_LteCfg = {
	.Rat = LTE_MQTT_RAT,
	.RatPref = LTE_MQTT_RAT_PREF,
	.bGnss = false,
	.pBand = nullptr,
	.NbBand = 0,
	.pApn = LTE_MQTT_APN,
	.PdnType = LTE_PDN_IPV4V6,
	.bPsm = false,
	.PsmTau = 0,
	.PsmActive = 0,
	.EdrxCycle = 0,
	.bRai = false,
	.IntPrio = IRQ_PRIO_LOW,
	.pTimer = &s_Timer,
	.TimerTrigNo = LTE_MQTT_TRIG_MODEM,
	.UrcMemSize = 0,
	.EvtHandler = LteEvent,
	.UrcHandler = nullptr,
};
static const SockIntrfCfg_t s_SockCfg = {
	.Proto = SOCKINTRF_PROTO_TCP,
	.pHost = MQTT_DEMO_HOST,
	.Port = MQTT_DEMO_PORT,
	.LocalPort = 0,
	.SecTag = -1,
	.bPeerVerify = false,
	.Timeout = 100,
	.EvtCB = nullptr,
};
static const char s_Hex[] = "0123456789abcdef";
static UART s_Uart;
static SockIntrf s_Socket;
static Mqtt s_Mqtt;
static RngEngine *s_pRng;
static char s_ClientId[23];
static char s_Topic[35];
static uint8_t s_Payload[] = "IOsonata LTE MQTT 00000000";
static uint32_t s_Sequence;
static uint32_t s_Checked;
static uint32_t s_Failures;
static uint32_t s_SentAt;
static uint32_t s_Retry;
static uint32_t s_LteRetry;
static bool s_Started;
static bool s_SubscribeSent;
static bool s_Subscribed;
static bool s_Waiting;
static bool s_EchoReceived;
static bool s_SessionChecked;
static bool s_RecoveryWaiting;
static bool s_LteStarted;
static volatile bool s_ReportQueued;
alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 192)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 192,
	.bBlocking = false,
};

static bool CreateIdentity()
{
	uint8_t random[8];
	size_t i;

	if (!s_pRng || s_pRng->Random(random, sizeof(random)) != CRYPTO_STATUS_OK)
	{
		return false;
	}
	// Keep the random identity for this boot so a PC can follow reconnects.
	memcpy(s_ClientId, "ioslte", 6);
	for (i = 0; i < sizeof(random); i++)
	{
		s_ClientId[6 + i * 2] = s_Hex[random[i] >> 4];
		s_ClientId[7 + i * 2] = s_Hex[random[i] & 15U];
	}
	s_ClientId[22] = 0;
	memcpy(s_Topic, "iosonata/lte/", 13);
	memcpy(s_Topic + 13, s_ClientId + 6, 16);
	memcpy(s_Topic + 29, "/echo", 6);
	return true;
}

static void MqttDown(uint32_t Now, const char *pReason)
{
	s_Failures++;
	SysLogPrintf(SysLogGet(), "MQTT FAIL: %s, mqtt_error=%u mqtt_result=%u checked=%lu failures=%lu\r\n",
		pReason, (unsigned)s_Mqtt.Error(), (unsigned)s_Mqtt.Result(),
		(unsigned long)s_Checked, (unsigned long)s_Failures);
	s_Mqtt.Reset();
	s_Socket.Close();
	s_Started = false;
	s_SubscribeSent = false;
	s_Subscribed = false;
	s_Waiting = false;
	s_EchoReceived = false;
	s_SessionChecked = false;
	Now = TimerGetMilisecond(&s_Timer);
	s_Retry = Now;
	s_RecoveryWaiting = true;
	SysLogPrintf(SysLogGet(), "MQTT reconnect in 30 s after network registration\r\n");
}

static bool CheckMessage(const MqttMessage_t *pMessage)
{
	const size_t prefix = sizeof(s_Payload) - 1U - 8U;
	uint32_t sequence = 0;
	uint8_t value;
	size_t i;

	// Public brokers can contain retained messages. Only our live echo counts.
	if (pMessage->bRetain || pMessage->TopicLen != sizeof(s_Topic) - 1U ||
		memcmp(pMessage->pTopic, s_Topic, sizeof(s_Topic) - 1U) != 0)
	{
		return true;
	}
	if (pMessage->Qos != 1 || pMessage->PayloadLen != sizeof(s_Payload) - 1U ||
		memcmp(pMessage->pPayload, s_Payload, prefix) != 0)
	{
		return false;
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
			return false;
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
	return (uint32_t)(s_Sequence - sequence) < 0x80000000U;
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
		MqttDown(Now, "publish rejected");
		return;
	}
	s_Sequence = sequence;
	s_SentAt = Now;
	s_Waiting = true;
	s_EchoReceived = false;
}

static void MqttTest(uint32_t Now)
{
	const MqttCfg_t cfg = {s_ClientId, nullptr, nullptr, 0, 60, LTE_MQTT_TIMEOUT_MS};
	const MqttMessage_t *message;
	DevIntrf_t *transport;
	MQTT_STATE state;
	bool valid;

	if (s_RecoveryWaiting && (uint32_t)(Now - s_Retry) >= LTE_MQTT_RESTART_MS)
	{
		s_RecoveryWaiting = false;
	}
	if (!s_Started)
	{
		if (s_RecoveryWaiting)
		{
			return;
		}
		SysLogPrintf(SysLogGet(), "MQTT TCP connecting: %s:%u\r\n",
			s_SockCfg.pHost, (unsigned)s_SockCfg.Port);
		if (!s_Socket.Init(s_SockCfg))
		{
			MqttDown(TimerGetMilisecond(&s_Timer), "socket initialization");
			return;
		}
		transport = s_Socket;
		transport->MaxRetry = 0;
		Now = TimerGetMilisecond(&s_Timer);
		if (!s_Mqtt.Init(cfg, &s_Socket, Now))
		{
			MqttDown(Now, "initialization");
			return;
		}
		s_Started = true;
		s_SentAt = Now - MQTT_DEMO_INTERVAL;
		SysLogPrintf(SysLogGet(), "MQTT TCP peer %s:%u, plaintext, client %s\r\n",
			s_SockCfg.pHost, (unsigned)s_SockCfg.Port, s_ClientId);
	}
	state = s_Mqtt.Process(Now, s_Socket.Connected());
	Now = TimerGetMilisecond(&s_Timer);
	if (state == MQTT_FAILED || state == MQTT_CLOSED)
	{
		MqttDown(Now, "session ended");
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
			MqttDown(Now, "echo payload or QoS mismatch");
			return;
		}
	}
	if (!s_SubscribeSent && !s_Mqtt.Busy())
	{
		if (!s_Mqtt.Subscribe(s_Topic, 1, Now))
		{
			MqttDown(Now, "subscribe rejected");
			return;
		}
		s_SubscribeSent = true;
	}
	else if (!s_Subscribed && !s_Mqtt.Busy())
	{
		if (s_Mqtt.Result() != 1)
		{
			MqttDown(Now, "broker did not grant QoS 1 subscription");
			return;
		}
		s_Subscribed = true;
		SysLogPrintf(SysLogGet(), "MQTT subscribed: %s QoS=1\r\n", s_Topic);
	}
	// Verify the broker's PUBACK and the self-echo, in either arrival order.
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
	if (s_Waiting && (uint32_t)(Now - s_SentAt) >= LTE_MQTT_TIMEOUT_MS)
	{
		MqttDown(Now, "30 second PUBACK or echo timeout");
		return;
	}
	if (s_Subscribed && !s_Waiting && !s_Mqtt.Busy() &&
		(uint32_t)(Now - s_SentAt) >= MQTT_DEMO_INTERVAL)
	{
		Publish(Now);
	}
}

static void StartLte()
{
	char info[48];

	s_LteStarted = LteInit(&s_LteCfg);
	s_LteRetry = TimerGetMilisecond(&s_Timer);
	if (!s_LteStarted)
	{
		s_Failures++;
		SysLogPrintf(SysLogGet(), "LTE init failed, retry in 30 s, failures=%lu\r\n",
			(unsigned long)s_Failures);
		return;
	}
	if (LteGetInfo(LTE_INFO_FWVER, info, sizeof(info)))
	{
		SysLogPrintf(SysLogGet(), "Modem firmware %s\r\n", info);
	}
	SysLogPrintf(SysLogGet(), "Attaching; PSM, eDRX and RAI disabled for this test\r\n");
}

static void LteEvent(LTE_EVT Evt, const LteStatus_t * const pStatus)
{
	uint32_t now = TimerGetMilisecond(&s_Timer);

	switch (Evt)
	{
		case LTE_EVT_REGISTERED:
			SysLogPrintf(SysLogGet(), "Registered %s, %s\r\n",
				pStatus->RegStat == LTE_REG_ROAMING ? "roaming" : "home",
				pStatus->Rat == LTE_RAT_NBIOT ? "NB-IoT" : "LTE-M");
			break;
		case LTE_EVT_UNREGISTERED:
			SysLogPrintf(SysLogGet(), "Unregistered, state %u\r\n", (unsigned)pStatus->RegStat);
			if (s_Started)
			{
				MqttDown(now, "network down");
			}
			break;
		case LTE_EVT_REG_STATE:
			SysLogPrintf(SysLogGet(), "Registration state %u\r\n", (unsigned)pStatus->RegStat);
			break;
		case LTE_EVT_RRC_CONNECTED:
			SysLogPrintf(SysLogGet(), "RRC connected\r\n");
			break;
		case LTE_EVT_RRC_IDLE:
			SysLogPrintf(SysLogGet(), "RRC idle\r\n");
			break;
		case LTE_EVT_MODEM_FAULT:
			MqttDown(now, "modem fault");
			s_LteStarted = false;
			s_LteRetry = TimerGetMilisecond(&s_Timer);
			SysLogPrintf(SysLogGet(), "Modem restart in 30 s\r\n");
			break;
		default:
			break;
	}
}

static void Report(uint32_t Evt, void *pCtx)
{
	uint32_t now = TimerGetMilisecond(&s_Timer);

	(void)Evt;
	(void)pCtx;
	s_ReportQueued = false;
	if (!s_LteStarted)
	{
		if ((uint32_t)(now - s_LteRetry) >= LTE_MQTT_RESTART_MS)
		{
			StartLte();
		}
		return;
	}
	if (!LteRegistered())
	{
		if (s_Started)
		{
			MqttDown(now, "network down");
		}
		return;
	}
	MqttTest(now);
}

static void ReportTimer(TimerDev_t *pTimer, int TrigNo, void *pCtx)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pCtx;
	if (!s_ReportQueued)
	{
		// A refused enqueue is retried at the next timer interrupt.
		s_ReportQueued = AppEvtHandlerQue(0, nullptr, Report);
	}
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_LogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
	SysLogPrintf(SysLogGet(), "MQTT over LTE TCP\r\n");
	s_pRng = MQTT_DEMO_RNG();
	if (!s_SockCfg.pHost || !s_SockCfg.pHost[0])
	{
		SysLogPrintf(SysLogGet(), "Set MQTT_DEMO_HOST in board.h\r\n");
	}
	else if (!s_pRng || !s_pRng->Enable() || !CreateIdentity())
	{
		SysLogPrintf(SysLogGet(), "MQTT random identity initialization failed\r\n");
	}
	else if (!TimerInit(&s_Timer, &s_TimerCfg))
	{
		SysLogPrintf(SysLogGet(), "Timer initialization failed\r\n");
	}
	else if (!msTimerEnableTrigger(&s_Timer, LTE_MQTT_TRIG_REPORT, 20,
		TIMER_TRIG_TYPE_CONTINUOUS, ReportTimer, nullptr))
	{
		SysLogPrintf(SysLogGet(), "MQTT timer initialization failed\r\n");
	}
	else
	{
		StartLte();
	}
	AppRun();
	return 0;
}
