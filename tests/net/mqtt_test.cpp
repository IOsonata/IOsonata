/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 I-SYST inc.
 * Independent wire fixtures exercise the production MQTT client.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <deque>
#include <vector>
#include <algorithm>
#include "net/mqtt.h"

struct Peer
{
	DevIntrf_t Dev = {};
	std::deque<uint8_t> In;
	std::vector<uint8_t> Out;
	int Chunk = 1024;
	bool Block = false;
	bool BadCount = false;
};
static bool Start(DevIntrf_t *, uint32_t)
{
	return true;
}
static void Stop(DevIntrf_t *)
{
}
static int Read(DevIntrf_t *p, uint8_t *data, int n)
{
	Peer *t = (Peer *)p->pDevData;
	n = std::min(n, std::min(t->Chunk, (int)t->In.size()));
	for (int i = 0; i < n; i++)
	{
		data[i] = t->In.front();
		t->In.pop_front();
	}
	return n;
}
static int Write(DevIntrf_t *p, const uint8_t *data, int n)
{
	Peer *t = (Peer *)p->pDevData;
	if (t->BadCount)
	{
		return n + 1;
	}
	if (t->Block)
	{
		return 0;
	}
	n = std::min(n, t->Chunk);
	t->Out.insert(t->Out.end(), data, data + n);
	return n;
}
static void Setup(Peer &p)
{
	p.Dev.pDevData = &p;
	p.Dev.StartRx = p.Dev.StartTx = Start;
	p.Dev.StopRx = p.Dev.StopTx = Stop;
	p.Dev.RxData = Read;
	p.Dev.TxData = Write;
	p.Dev.EnCnt = 1;
	atomic_flag_clear(&p.Dev.bBusy);
}
static void Feed(Peer &p, std::initializer_list<uint8_t> bytes)
{
	p.In.insert(p.In.end(), bytes);
}
static void Pump(MqttDev_t &d, unsigned now = 1, unsigned times = 1100)
{
	while (times-- && d.State != MQTT_FAILED)
	{
		MqttProcess(&d, now, true);
	}
}
static void Connected(MqttDev_t &d, Peer &p, uint32_t now = 0)
{
	MqttCfg_t cfg = {"board", nullptr, nullptr, 0, 2, 1000};
	assert(MqttInit(&d, &cfg, &p.Dev, now));
	Pump(d, now);
	assert(p.Out == std::vector<uint8_t>(
						{0x10, 17, 0, 4, 'M', 'Q', 'T', 'T', 4, 2, 0, 2, 0, 5, 'b', 'o', 'a', 'r', 'd'}));
	p.Out.clear();
	Feed(p, {0x20, 2, 0, 0});
	Pump(d, now);
	assert(d.State == MQTT_READY);
}
static void Checks(int chunk)
{
	Peer p;
	MqttDev_t d = {};
	Setup(p);
	p.Chunk = chunk;
	Connected(d, p);
	assert(MqttSubscribe(&d, "a/+", 1, 1));
	Pump(d);
	assert(p.Out == std::vector<uint8_t>({0x82, 8, 0, 1, 0, 3, 'a', '/', '+', 1}));
	assert(MqttBusy(&d));
	p.Out.clear();
	Feed(p, {0x90, 3, 0, 1, 1});
	Pump(d);
	assert(!MqttBusy(&d) && d.Result == 1);
	assert(MqttPublish(&d, "a/b", (const uint8_t *)"x\0y", 3, 1, true, 2));
	Pump(d, 2);
	assert(p.Out == std::vector<uint8_t>({0x33, 10, 0, 3, 'a', '/', 'b', 0, 2, 'x', 0, 'y'}));
	p.Out.clear();
	// Incoming QoS 1 can use the same id as our independent outgoing operation.
	Feed(p, {0x3b, 10, 0, 3, 'a', '/', 'b', 0, 2, 'x', 0, 'y', 0x40, 2, 0, 2});
	Pump(d, 2);
	assert(d.bMessage && d.Message.Qos == 1 && d.Message.bDuplicate && d.Message.bRetain);
	assert(d.Message.PayloadLen == 3 && memcmp(d.Message.pPayload, "x\0y", 3) == 0);
	assert(p.Out.empty()); // Ownership is accepted only on Release.
	MqttRelease(&d);
	Pump(d, 2);
	assert(p.Out == std::vector<uint8_t>({0x40, 2, 0, 2}));
	assert(!MqttBusy(&d));
	p.Out.clear();
	assert(MqttUnsubscribe(&d, "a/+", 3));
	Pump(d, 3);
	assert(p.Out == std::vector<uint8_t>({0xa2, 7, 0, 3, 0, 3, 'a', '/', '+'}));
	Feed(p, {0xb0, 2, 0, 3});
	Pump(d, 3);
	assert(!MqttBusy(&d));
	p.Out.clear();
	Pump(d, 2003);
	assert(p.Out == std::vector<uint8_t>({0xc0, 0}));
	Feed(p, {0xd0, 0});
	Pump(d, 2004);
	assert(!d.bPing);
	p.Out.clear();
	assert(MqttDisconnect(&d, 2005));
	Pump(d, 2005);
	assert(d.State == MQTT_CLOSED && p.Out == std::vector<uint8_t>({0xe0, 0}));
	MqttReset(&d);
	assert(d.State == MQTT_IDLE);
	for (auto c : d.Tx)
	{
		assert(c == 0);
	}
	for (auto c : d.Rx)
	{
		assert(c == 0);
	}
}
int main()
{
	for (int chunk : {1, 7, 1024})
	{
		Checks(chunk);
	}
	for (auto bytes : {std::vector<uint8_t>{0x20, 2, 1, 0}, {0x20, 2, 0, 6}, {0x21, 2, 0, 0}})
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		MqttCfg_t cfg = {"b", nullptr, nullptr, 0, 0, 1000};
		assert(MqttInit(&d, &cfg, &p.Dev, 0));
		p.In.insert(p.In.end(), bytes.begin(), bytes.end());
		Pump(d);
		assert(d.Error == MQTT_ERROR_PROTOCOL);
	}
	for (auto bytes : {std::vector<uint8_t>{0x30, 0},
					   {0x36, 2, 0, 0},
					   {0x38, 3, 0, 1, 'a'},
					   {0x30, 3, 0, 1, '#'},
					   {0x32, 5, 0, 1, 'a', 0, 0},
					   {0x30, 0xff, 0x7f},
					   {0xd0, 0},
					   {0x40, 2, 0, 1},
					   {0x30, 0x80, 0},
					   {0x30, 4, 0, 2, 0xc0, 0x80}})
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		p.In.insert(p.In.end(), bytes.begin(), bytes.end());
		Pump(d);
		assert(d.State == MQTT_FAILED);
	}
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p, 0xfffffff0U);
		assert(MqttSubscribe(&d, "a", 0, 0xfffffff0U));
		Pump(d, 0x3d8U);
		assert(d.Error == MQTT_ERROR_TIMEOUT);
	}
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		assert(!MqttPublish(&d, "a/#", nullptr, 0, 0, false, 0));
		assert(!MqttSubscribe(&d, "a+b", 0, 0));
		assert(!MqttSubscribe(&d, "a/#/b", 0, 0));
		assert(!MqttSubscribe(&d, "a", 2, 0));
		assert(MqttSubscribe(&d, "a", 0, 0));
		Feed(p, {0x90, 3, 0, 1, 0x80});
		Pump(d);
		assert(d.State == MQTT_READY && d.Result == 0x80 && !MqttBusy(&d));
		p.Block = true;
		assert(MqttPublish(&d, "a", nullptr, 0, 0, false, 1));
		Pump(d, 1001);
		assert(d.Error == MQTT_ERROR_TIMEOUT);
	}
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		Feed(p, {0x30, 4, 0});
		Pump(d, 1);
		Pump(d, 1001);
		assert(d.Error == MQTT_ERROR_TIMEOUT);
	}
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		Pump(d, 2000);
		Pump(d, 3000);
		assert(d.Error == MQTT_ERROR_TIMEOUT);
	}
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		assert(MqttProcess(&d, 1, false) == MQTT_FAILED && d.Error == MQTT_ERROR_TRANSPORT);
	}
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		p.BadCount = true;
		assert(MqttPublish(&d, "a", nullptr, 0, 0, false, 1));
		Pump(d);
		assert(d.Error == MQTT_ERROR_TRANSPORT);
	}
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		MqttCfg_t cfg = {"b", "u", (const uint8_t *)"p", 1, 0, 1000};
		assert(MqttInit(&d, &cfg, &p.Dev, 0));
		Pump(d);
		assert(p.Out[9] == 0xc2);
		for (auto c : d.Tx)
		{
			assert(c == 0);
		}
		Feed(p, {0x20, 2, 0, 5});
		Pump(d);
		assert(d.Error == MQTT_ERROR_REFUSED && d.Result == 5);
	}
	// Large binary payload and the two-byte Remaining Length encoding.
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		uint8_t data[1021];
		for (size_t i = 0; i < sizeof(data); i++)
		{
			data[i] = (uint8_t)i;
		}
		assert(MqttPublish(&d, "a", data, sizeof(data), 0, false, 1));
		Pump(d);
		assert(p.Out.size() == 1027 && p.Out[1] == 0x80 && p.Out[2] == 8);
		p.In.insert(p.In.end(), p.Out.begin(), p.Out.end());
		Pump(d);
		assert(d.bMessage && d.Message.PayloadLen == sizeof(data));
		assert(memcmp(d.Message.pPayload, data, sizeof(data)) == 0);
		MqttRelease(&d);
		assert(!MqttPublish(&d, "a", data, sizeof(data) + 1, 0, false, 1));
	}
	// Backpressure must not replace the first deferred PUBACK with a later id.
	{
		Peer p;
		MqttDev_t d = {};
		Setup(p);
		Connected(d, p);
		p.Block = true;
		assert(MqttPublish(&d, "a", nullptr, 0, 0, false, 1));
		Feed(p, {0x32, 5, 0, 1, 'a', 0, 7, 0x32, 5, 0, 1, 'b', 0, 8});
		Pump(d);
		assert(d.bMessage && d.AckId == 7);
		MqttRelease(&d);
		Pump(d);
		assert(!d.bMessage && d.AckId == 7 && p.In.size() == 7);
		p.Block = false;
		Pump(d);
		assert(d.bMessage && d.AckId == 8);
		MqttRelease(&d);
		Pump(d);
		assert(p.Out == std::vector<uint8_t>({0x30, 3, 0, 1, 'a', 0x40, 2, 0, 7, 0x40, 2, 0, 8}));
	}
	printf("MQTT wire fixtures, partial I/O, QoS 0/1, keepalive, malformed input, wrap and wiping: PASS\n");
}
