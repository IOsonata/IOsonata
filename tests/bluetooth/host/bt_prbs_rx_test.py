#!/usr/bin/env python3
"""Check the firmware PRBS receiver using simulated time and BLE notifications."""
import os
from pathlib import Path
import subprocess
import tempfile
from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / 'exemples/bluetooth/ble_prbs_rx.cpp').read_text()
start = source.index('typedef struct {')
end = source.index('static void PrbsReset(void)', start)
code = r'''
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "bluetooth/bt_app.h"
#include "app_evt_handler.h"
#define PRBS_RX_NOTIF_COUNT 12
#define PRBS_RX_DURATION_MS 60000U
#define PRBS_RX_REPORT_MS 1000U
#define BLE_MTU_SIZE 247
static uint32_t nowMs;
static unsigned scans;
static unsigned subscribes;
static unsigned disconnects;
static unsigned reportCount;
static uint8_t actualPhy = 1;
static unsigned phyRefused = 1;
static BtDevice_t peer;
static char output[65536];
static unsigned outputSize;
struct
{
	void printf(const char *format, ...)
	{
		char buffer[80];
		va_list args;
		int length;

		va_start(args, format);
		length = vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);
		assert(length >= 0 && length < int(sizeof(buffer)));
		assert(outputSize + length < sizeof(output));
		memcpy(output + outputSize, buffer, length + 1);
		outputSize += length;
		if (strstr(buffer, "Bytes/sec="))
		{
			reportCount++;
		}
	}
} g_Uart;
uint32_t BtGattMsTick()
{
	return nowMs;
}
void BtAppCheckStatus()
{
}
void BtAppScan()
{
	scans++;
}
BtDevice_t *BtPeerFindByHdl(uint16_t)
{
	return &peer;
}
bool BtGapReadPhy(uint16_t, uint8_t *tx, uint8_t *rx)
{
	*tx = actualPhy;
	*rx = actualPhy;
	return true;
}
bool BtGapSetPhy(uint16_t, uint8_t tx, uint8_t rx, uint16_t)
{
	assert(tx == rx && (tx == 1 || tx == 2));
	if (phyRefused > 0)
	{
		phyRefused--;
		return false;
	}
	return true;
}
bool BtGapSetDataLength(uint16_t, uint16_t octets, uint16_t time)
{
	assert(octets == 251 && time == 2120);
	return true;
}
bool BtAppEnableNotify(uint16_t, uint16_t)
{
	subscribes++;
	return true;
}
void BtAppDisconnectConn(uint16_t handle)
{
	disconnects++;
	BtAppEvtDisconnected(handle);
}
static void StartDiscovery(uint16_t)
{
}
'''
code += source[start:end]
for signature in [r'^static void PrbsReset\(', r'^static void PrbsCheck\(', r'^static void PrbsReceive\(',
                  r'^void BtGattClientNotified\(', r'^void BtAppEvtConnected\(',
                  r'^void BtAppEvtDisconnected\(', r'^void BtAppEvtConnectFailed\(',
                  r'^static void PrbsReport\(', r'^bool AppCheckStatus\(']:
    code += extract_function(source, signature) + '\n'
code += r'''
static void RunTrial(unsigned packets)
{
	uint8_t data[244];
	uint8_t previous = 0xff;
	unsigned elapsed;
	unsigned packet;
	unsigned idx;
	unsigned start = nowMs;
	unsigned before = reportCount;

	for (elapsed = 0; elapsed < PRBS_RX_DURATION_MS; elapsed++)
	{
		nowMs = start + elapsed;
		if (elapsed % 15 == 0)
		{
			for (packet = 0; packet < packets; packet++)
			{
				for (idx = 0; idx < sizeof(data); idx++)
				{
					previous = ((previous << 1) | (((previous >> 6) ^ (previous >> 5)) & 1)) & 0x7f;
					data[idx] = previous;
				}
				BtGattClientNotified(s_ConnHdl, s_ValueHdl, data, sizeof(data));
			}
		}
		while (AppEvtHandlerExec())
		{
		}
		AppCheckStatus();
		assert(reportCount - before == elapsed / 1000);
	}
	nowMs = start + PRBS_RX_DURATION_MS;
	AppCheckStatus();
	assert(reportCount - before == 60);
	assert(s_FinalPrinted && s_Finished && s_Disconnected);
	assert(s_Rx.Bytes == packets * 4000 * 244);
	assert(s_Rx.Notifications == packets * 4000);
	assert(s_Rx.Breaks == 0 && s_Rx.OutOfFrame == 0);
	assert(s_Rx.MinLen == 244 && s_Rx.MaxLen == 244);
	BtGattClientNotified(s_ConnHdl, s_ValueHdl, data, sizeof(data));
	assert(s_Rx.Notifications == packets * 4000);
}
int main()
{
	uint8_t invalid[] = {1, 0x80, 3};
	uint8_t zeros[16] = {};
	unsigned idx;

	assert(AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)));
	s_NotifQue = CFifoInit(s_NotifMem, sizeof(s_NotifMem), sizeof(PrbsNotification_t), true);
	assert(s_NotifQue != nullptr && CFifoAvail(s_NotifQue) == 12);
	peer.Conn.MaxMtu = 247;
	BtAppEvtConnected(0);
	assert(s_PhyPending && !s_Failed);
	s_ValueHdl = 0x12;
	s_CccdHdl = 0x13;
	nowMs = 1000;
	AppCheckStatus();
	assert(subscribes == 1 && !s_PhyPending);
	BtGattClientNotified(1, 0x12, invalid, sizeof(invalid));
	BtGattClientNotified(0, 0x14, invalid, sizeof(invalid));
	assert(s_Rx.Bytes == 0);
	RunTrial(4);
	assert(disconnects == 1);
	AppCheckStatus();
	assert(s_TestPhy == 2 && scans == 1 && !s_Rx.Started);
	BtAppEvtConnected(1);
	s_ValueHdl = 0x12;
	s_CccdHdl = 0x13;
	nowMs += 1000;
	AppCheckStatus();
	assert(subscribes == 1); // Wrong actual PHY must not start the 2M stream.
	actualPhy = 2;
	nowMs += 1000;
	AppCheckStatus();
	assert(subscribes == 2);
	RunTrial(8);
	AppCheckStatus();
	assert(scans == 1 && disconnects == 2);
	assert(strstr(output, "Final PHY=1") != nullptr);
	assert(strstr(output, "Final PHY=2") != nullptr);
	assert(strstr(output, "PRBS integrity: FAIL") == nullptr);
	assert(strstr(output, "PHY=1 RESULT: PASS") != nullptr);
	assert(strstr(output, "PHY=2 RESULT: PASS") != nullptr);
	// Aborted 1M startup must not silently advance to the 2M trial.
	s_TestPhy = 1;
	actualPhy = 1;
	BtAppEvtConnected(0);
	BtAppEvtDisconnected(0);
	AppCheckStatus();
	AppCheckStatus();
	assert(s_FinalPrinted && !s_TrialComplete && s_TestPhy == 1 && scans == 1);
	assert(strstr(output, "PHY=1 trial aborted; benchmark stopped.") != nullptr);
	// A startup timeout has the same stop behavior.
	BtAppEvtConnected(0);
	nowMs += 10000;
	AppCheckStatus();
	AppCheckStatus();
	assert(s_FinalPrinted && !s_TrialComplete && s_TestPhy == 1 && scans == 1);
	// Disconnect after some data also stops instead of skipping the trial.
	BtAppEvtConnected(0);
	PrbsCheck(zeros, sizeof(zeros), nowMs);
	nowMs += 1000;
	BtAppEvtDisconnected(0);
	AppCheckStatus();
	AppCheckStatus();
	assert(s_FinalPrinted && !s_TrialComplete && s_TestPhy == 1 && scans == 1);
	// Damage is counted locally, with no UART payload API in this harness.
	PrbsReset();
	s_Finished = false;
	PrbsCheck(invalid, sizeof(invalid), UINT32_MAX - 100);
	assert(s_Rx.Breaks == 2 && s_Rx.OutOfFrame == 1);
	PrbsCheck(invalid, sizeof(invalid), 59900);
	assert(s_Finished && s_Rx.Bytes == 3); // Duration check survives clock wrap.
	PrbsReset();
	s_Finished = false;
	PrbsCheck(zeros, sizeof(zeros), 0);
	assert(!s_Rx.Synced); // Zero has a zero successor but is not valid PRBS.
	// Payload ownership: callback must not check data or retain its pointer.
	PrbsReset();
	s_Finished = false;
	s_Failed = false;
	s_ConnHdl = 0;
	s_ValueHdl = 0x12;
	nowMs = 100;
	BtGattClientNotified(0, 0x12, invalid, sizeof(invalid));
	assert(s_Rx.Bytes == 0 && CFifoUsed(s_NotifQue) == 1);
	memset(invalid, 0, sizeof(invalid));
	nowMs = 200;
	AppEvtHandlerExec();
	assert(s_Rx.Bytes == 3 && s_Rx.OutOfFrame == 1 && s_Rx.FirstMs == 100);
	// Fill all 12 event slots; a refused receive event must be retried.
	PrbsReset();
	for (idx = 0; idx < 12; idx++)
	{
		assert(AppEvtHandlerQue(0, nullptr, [](uint32_t, void *) {}));
	}
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	assert(s_Rx.Bytes == 0 && CFifoUsed(s_NotifQue) == 1);
	while (AppEvtHandlerExec())
	{
	}
	assert(s_Rx.Bytes == sizeof(zeros) && CFifoUsed(s_NotifQue) == 0);
	// Delayed processing still uses arrival time at the 60-second boundary.
	PrbsReset();
	s_Finished = false;
	nowMs = 100;
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	AppEvtHandlerExec();
	nowMs = 60099;
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	nowMs = 60100;
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	nowMs = 60110;
	AppEvtHandlerExec();
	assert(s_Rx.Notifications == 2 && s_Finished);
	s_Finished = false;
	// Full notification FIFO refuses new data without overwriting old data.
	PrbsReset();
	for (idx = 0; idx < 13; idx++)
	{
		BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	}
	assert(CFifoUsed(s_NotifQue) == 12 && s_NotifDropped == 1 && s_Failed);
	while (AppEvtHandlerExec())
	{
	}
	assert(s_Rx.Notifications == 12);

}
'''
with tempfile.TemporaryDirectory(prefix='ble-prbs-rx-') as temp:
    cpp = Path(temp) / 'test.cpp'
    binary = Path(temp) / 'test'
    for port in ['NRFXLIB_SDC', 'IOSONATA_STM32_WBA']:
        cpp.write_text('#define ' + port + ' 1\n' + code)
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O2',
                        '-Wall', '-Wextra', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-I' + str(ROOT / 'include'),
                        str(cpp), str(ROOT / 'src/cfifo.c'),
                        str(ROOT / 'src/app_evt_handler.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS firmware PRBS: 60s 1M + 60s 2M, 1s reports, PHY check, corruption, clock wrap')
