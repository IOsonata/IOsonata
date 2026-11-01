#!/usr/bin/env python3
"""Check the firmware PRBS receiver using simulated time and BLE notifications."""
import os
from pathlib import Path
import subprocess
import tempfile
from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / 'exemples/bluetooth/ble_prbs_rx.cpp').read_text()
start = source.index('alignas(4) uint8_t g_AppEvtHandlerQueMem')
end = source.index('static void PrbsReceive(uint32_t Evt, void *pCtx);', start)
code = r'''
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include "bluetooth/bt_app.h"
#include "app_evt_handler.h"
#include "syslog.h"
#include "ble_prbs_benchmark.h"
#define PRBS_RX_SETUP_TIMEOUT_MS 10000U
#define PRBS_RX_DURATION_MS 60000U
#define PRBS_RX_REPORT_MS 1000U
#define BLE_MTU_SIZE 247
static uint32_t nowMs;
static unsigned scans;
static unsigned subscribes;
static unsigned disconnects;
static unsigned reportCount;
static uint8_t actualPhy = 1;
static unsigned phyRefused;
static unsigned phyReads;
static unsigned phyRequests;
static unsigned dleRefused;
static BtDevice_t peer;
static char output[65536];
static unsigned outputSize;
struct
{
	void printf(const char *format, ...)
	{
		char buffer[160];
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
	int Tx(uint8_t *data, uint32_t len)
	{
		assert(outputSize + len < sizeof(output));
		memcpy(output + outputSize, data, len);
		output[outputSize + len] = 0;
		if (strstr(output + outputSize, "Bytes/sec="))
		{
			reportCount++;
		}
		outputSize += len;
		return len;
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
	phyReads++;
	*tx = actualPhy;
	*rx = actualPhy;
	return true;
}
bool BtGapSetPhy(uint16_t, uint8_t tx, uint8_t rx, uint16_t)
{
	phyRequests++;
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
	if (dleRefused > 0)
	{
		dleRefused--;
		return false;
	}
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
bool BtAppDiscoverDevice(BtDevice_t *)
{
	return true;
}
'''
code += source[start:end]
code += 'static void PrbsReceive(uint32_t, void *);\n'
for signature in [r'^static bool PrbsSchedule\(', r'^static bool PrbsWrite\(', r'^static void PrbsDone\(']:
    code += extract_function(source, signature) + '\n'
a = source.index('static const PrbsBenchAdapters_t s_BenchAdapters')
code += source[a:source.index('static void PrbsReset(void)', a)]
for signature in [r'^static void PrbsReset\(', r'^static void PrbsReceive\([^;]*?\)\s*(?=\{)',
                  r'^void BtGattClientNotified\(', r'^void BtAppEvtConnected\(',
                  r'^void BtAppEvtDisconnected\(', r'^void BtAppEvtConnectFailed\(',
                  r'^bool AppCheckStatus\(']:
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
	unsigned readsBefore = phyReads;
	unsigned requestsBefore = phyRequests;

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
	assert(phyReads == readsBefore && phyRequests == requestsBefore);
	assert(s_Benchmark.FinalPrinted && s_Benchmark.Finished && s_Disconnected);
	assert(s_Benchmark.Stats.Bytes == packets * 4000 * 244);
	assert(s_Benchmark.Stats.Notifications == packets * 4000);
	assert(s_Benchmark.Stats.Breaks == 0 && s_Benchmark.Stats.OutOfFrame == 0);
	assert(s_Benchmark.Stats.MinLen == 244 && s_Benchmark.Stats.MaxLen == 244);
	BtGattClientNotified(s_ConnHdl, s_ValueHdl, data, sizeof(data));
	assert(s_Benchmark.Stats.Notifications == packets * 4000);
}
int main()
{
	uint8_t invalid[] = {1, 0x80, 3};
	uint8_t zeros[16] = {};
	unsigned idx;
	PrbsRxStats_t reportStats = {};
	PrbsReportValues_t report;

	assert(AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)));
	assert(PrbsBenchInit(&s_Benchmark, &s_BenchAdapters, s_TestPhy));
	assert(s_Benchmark.Queue != nullptr && CFifoAvail(s_Benchmark.Queue) == 12);
	peer.Conn.MaxMtu = 247;
	BtAppEvtConnected(0);
	assert(!s_Benchmark.Failed);
	s_ValueHdl = 0x12;
	s_CccdHdl = 0x13;
	nowMs = 1000;
	AppCheckStatus();
	assert(subscribes == 1 && !s_Benchmark.Failed);
	BtGattClientNotified(1, 0x12, invalid, sizeof(invalid));
	BtGattClientNotified(0, 0x14, invalid, sizeof(invalid));
	assert(s_Benchmark.Stats.Bytes == 0);
	RunTrial(4);
	assert(disconnects == 1);
	AppCheckStatus();
	assert(s_TestPhy == 2 && scans == 1 && !s_Benchmark.Stats.Started);
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
	// Explicit 1M preference must reach the stack even on a matching link.
	s_TestPhy = 1;
	actualPhy = 1;
	phyRequests = 0;
	BtAppEvtConnected(0);
	assert(phyRequests == 1 && !s_Benchmark.Failed);
	phyRefused = 1;
	BtAppEvtConnected(0);
	assert(s_Benchmark.Failed);
	AppCheckStatus();
	assert(s_Benchmark.FinalPrinted && !s_Benchmark.Complete);
	dleRefused = 1;
	BtAppEvtConnected(0);
	assert(s_Benchmark.Failed);
	AppCheckStatus();
	assert(s_Benchmark.FinalPrinted && !s_Benchmark.Complete);
	// Aborted 1M startup must not silently advance to the 2M trial.
	s_TestPhy = 1;
	actualPhy = 1;
	BtAppEvtConnected(0);
	BtAppEvtDisconnected(0);
	AppCheckStatus();
	AppCheckStatus();
	assert(s_Benchmark.FinalPrinted && !s_Benchmark.Complete && s_TestPhy == 1 && scans == 1);
	assert(strstr(output, "PHY=1 trial aborted; benchmark stopped.") != nullptr);
	// A startup timeout has the same stop behavior.
	BtAppEvtConnected(0);
	nowMs += 10000;
	AppCheckStatus();
	AppCheckStatus();
	assert(s_Benchmark.FinalPrinted && !s_Benchmark.Complete && s_TestPhy == 1 && scans == 1);
	// Disconnect after some data also stops instead of skipping the trial.
	BtAppEvtConnected(0);
	PrbsBenchCheck(&s_Benchmark, zeros, sizeof(zeros), nowMs);
	nowMs += 1000;
	BtAppEvtDisconnected(0);
	AppCheckStatus();
	AppCheckStatus();
	assert(s_Benchmark.FinalPrinted && !s_Benchmark.Complete && s_TestPhy == 1 && scans == 1);
	// Damage is counted locally, with no UART payload API in this harness.
	PrbsReset();
	s_Benchmark.Finished = false;
	PrbsBenchCheck(&s_Benchmark, invalid, sizeof(invalid), UINT32_MAX - 100);
	assert(s_Benchmark.Stats.Breaks == 2 && s_Benchmark.Stats.OutOfFrame == 1);
	PrbsBenchCheck(&s_Benchmark, invalid, sizeof(invalid), 59900);
	assert(s_Benchmark.Finished && s_Benchmark.Stats.Bytes == 3); // Duration check survives clock wrap.
	PrbsReset();
	s_Benchmark.Finished = false;
	PrbsBenchCheck(&s_Benchmark, zeros, sizeof(zeros), 0);
	assert(!s_Benchmark.Stats.Synced); // Zero has a zero successor but is not valid PRBS.
	// Payload ownership: callback must not check data or retain its pointer.
	PrbsReset();
	s_Benchmark.Finished = false;
	s_Benchmark.Failed = false;
	s_ConnHdl = 0;
	s_ValueHdl = 0x12;
	s_ConnectMs = 100;
	s_LastPollMs = 100;
	nowMs = 100;
	BtGattClientNotified(0, 0x12, invalid, sizeof(invalid));
	assert(s_Benchmark.Stats.Bytes == 0 && CFifoUsed(s_Benchmark.Queue) == 1);
	memset(invalid, 0, sizeof(invalid));
	nowMs = 200;
	AppEvtHandlerExec();
	assert(s_Benchmark.Stats.Bytes == 3 && s_Benchmark.Stats.OutOfFrame == 1 && s_Benchmark.Stats.FirstMs == 100);
	// Fill all 12 event slots; a refused receive event must be retried.
	PrbsReset();
	for (idx = 0; idx < 12; idx++)
	{
		assert(AppEvtHandlerQue(0, nullptr, [](uint32_t, void *) {}));
	}
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	assert(s_Benchmark.Stats.Bytes == 0 && CFifoUsed(s_Benchmark.Queue) == 1);
	while (AppEvtHandlerExec())
	{
	}
	assert(s_Benchmark.Stats.Bytes == sizeof(zeros) && CFifoUsed(s_Benchmark.Queue) == 0);
	// Delayed processing still uses arrival time at the 60-second boundary.
	s_TestPhy = 2;
	PrbsReset();
	s_Benchmark.Finished = false;
	nowMs = 100;
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	AppEvtHandlerExec();
	nowMs = 60099;
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	nowMs = 60100;
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	nowMs = 60110;
	AppEvtHandlerExec();
	assert(s_Benchmark.Stats.Notifications == 2 && s_Benchmark.Finished);
	s_Benchmark.Finished = false;
	// Full notification FIFO refuses new data without overwriting old data.
	PrbsReset();
	s_ConnHdl = 0;
	s_ValueHdl = 0x12;
	for (idx = 0; idx < 13; idx++)
	{
		BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	}
	assert(CFifoUsed(s_Benchmark.Queue) == 12 && s_Benchmark.Dropped == 1 && s_Benchmark.Failed);
	while (AppEvtHandlerExec())
	{
	}
	assert(s_Benchmark.Stats.Notifications == 12);

	// A connection callback only requests reset; it cannot mutate the
	// main-loop statistics. A notification queued before status polling
	// must survive the measurement reset and be checked exactly once.
	BtAppEvtConnected(0);
	assert(s_Benchmark.ResetPending && s_Benchmark.Stats.Notifications == 12);
	s_ValueHdl = 0x12;
	BtGattClientNotified(0, 0x12, zeros, sizeof(zeros));
	assert(CFifoUsed(s_Benchmark.Queue) == 1);
	AppEvtHandlerExec();
	assert(!s_Benchmark.ResetPending && s_Benchmark.Stats.Notifications == 1);
	assert(s_Benchmark.Stats.Bytes == sizeof(zeros) && CFifoUsed(s_Benchmark.Queue) == 0);

	// Reports use the same exact 60-second window even if the task runs late.
	reportStats.Started = true;
	reportStats.Synced = true;
	reportStats.FirstMs = 0xfffffff0u;
	reportStats.Bytes = 7808000;
	reportStats.Notifications = 32000;
	PrbsReportValues(&reportStats, reportStats.FirstMs + 60009u,
		reportStats.FirstMs + 59000u, 7677948, 60000, 2, false, 0, &report);
	assert(report.Elapsed == 60000 && report.Time == reportStats.FirstMs + 60000u);
	assert(report.Rate == 130052 && report.Average == 130133);
	assert(report.PerInterval == 8000 && report.Complete && report.Valid && report.Passed);
	PrbsReportValues(&reportStats, reportStats.FirstMs + 60000u,
		reportStats.FirstMs, 0, 60000, 2, false, 1, &report);
	assert(!report.Valid && !report.Passed);

}
'''
with tempfile.TemporaryDirectory(prefix='ble-prbs-rx-') as temp:
    cpp = Path(temp) / 'test.cpp'
    binary = Path(temp) / 'test'
    board = Path(temp) / 'board.h'
    board.write_text('\n'.join([
        '#define CONNECT_LED_PORT 0', '#define CONNECT_LED_PIN 0',
        '#define CONNECT_LED_LOGIC 1',
    ] + [f'#define UART_{pin}_{suffix} {value}'
         for pin in ('RX', 'TX', 'CTS', 'RTS')
         for suffix, value in [('PORT', '0'), ('PIN', '0'), ('PINOP', 'IOPINOP_GPIO')]]))
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-fsyntax-only',
                    '-I' + str(ROOT / 'include'), '-I' + temp,
                    str(ROOT / 'exemples/bluetooth/ble_prbs_rx.cpp')], check=True)
    for port in ['NRFXLIB_SDC', 'IOSONATA_STM32_WBA', 'S132', 'S140']:
        cpp.write_text('#define ' + port + ' 1\n' + code)
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O2',
                        '-Wall', '-Wextra', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-I' + str(ROOT / 'include'),
                        '-I' + str(ROOT / 'exemples/bluetooth'),
                        str(cpp), str(ROOT / 'src/cfifo.c'),
                        str(ROOT / 'src/app_evt_handler.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
print('PASS firmware PRBS: 60s 1M + 60s 2M, 1s reports, PHY check, corruption, clock wrap')
