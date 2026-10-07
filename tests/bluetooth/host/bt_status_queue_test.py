#!/usr/bin/env python3
"""Exercise production status/queue paths with AppRun and simulated devices."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]


def function(source, name):
	return extract_function(source,
		r"^(?:static )?(?:void|bool) " + name + r"\([^;{}]*\)\s*(?=\{)")


COMMON = r'''
#include <cassert>
#include <csetjmp>
#include <cstdint>
#include <cstdio>
#include <atomic>
#include "app_evt_handler.h"
static jmp_buf idle;
static unsigned normal;
extern "C" void AppWait(void) { longjmp(idle, 1); }
static void noop(uint32_t, void *) { ++normal; }
static void fill() { while (AppEvtHandlerQue(0, nullptr, noop)) {} }
static void run() { if (setjmp(idle) == 0) AppRun(); }
extern "C" bool BtEvtQue(uint32_t e, void *p, AppEvtHandler_t h)
{ return AppEvtHandlerQue(e, p, h); }
'''

sdc_source = (ROOT / 'ARM/Nordic/src/bt_app_sdc.cpp').read_text()
sdc = COMMON + r'''
struct TimerDev_t {};
#define TIMER_EVT_TRIGGER(n) (1U << (n))
struct Conn { void (*Tick)(); };
static unsigned ticks, polls;
static void tick() { ++ticks; }
static void poll() { ++polls; }
static Conn conn = { tick };
static const Conn *s_pBtAppSdcConn = &conn;
static void (*s_pBtAppSdcSecPoll)() = poll;
extern "C" void BtAppCheckStatus(void);
'''
sdc += sdc_source[sdc_source.index('enum {\n\tBT_APP_SDC_TICK_IDLE'):
				  sdc_source.index('void BtAppSetDevName')]
sdc += r'''
int main() {
	assert(AppEvtHandlerInit(nullptr, 0));
	fill();
	BtAppSdcTimerHandler(nullptr, TIMER_EVT_TRIGGER(0));
	assert(ticks == 0 && polls == 0);
	run();
	assert(ticks == 1 && polls == 1 && normal > 0);
	for (int i = 0; i < 100; ++i) BtAppCheckStatus();
	assert(!AppEvtHandlerPending() && ticks == 1);
	for (int i = 0; i < 10; ++i)
		BtAppSdcTimerHandler(nullptr, TIMER_EVT_TRIGGER(0));
	run();
	assert(ticks == 2 && polls == 2);
	BtAppSdcTimerHandler(nullptr, TIMER_EVT_TRIGGER(1));
	assert(!AppEvtHandlerPending());
#ifdef WITH_SECURITY
	extern unsigned cryptoChecks, bondChecks, dfuChecks;
	assert(cryptoChecks > 0 && bondChecks == cryptoChecks && dfuChecks == cryptoChecks);
#else
	assert(BtSmpCheckStatus == nullptr && BtSmpBondNvmCheckStatus == nullptr);
	assert(BtDfuSmpCheckStatus == nullptr);
#endif
	puts("PASS: SDC recovers a refused tick, coalesces events and keeps security optional");
}
'''

smp_source = (ROOT / 'src/bluetooth/bt_smp.cpp').read_text()
smp = COMMON + r'''
#define BT_SMP_MAX_LINK 2
#define BT_CONN_HDL_INVALID 65535
#define BT_SMP_CRYPTO_OP_NONE 0
struct Link { uint16_t ConnHdl; int CryptoOp; bool bRetryBusy; };
static Link s_SmpLink[2] = {{7, 1, true}, {BT_CONN_HDL_INVALID, 0, false}};
static bool s_bSmpCryptoRetryQueued;
static unsigned retries;
static void SmpCryptoRetryQue(void);
void BtSmpCheckStatus(void);
static void SmpCryptoRetryPending() { ++retries; s_SmpLink[0].bRetryBusy = false; }
'''
for name in ['SmpCryptoRetryEvt', 'BtSmpCheckStatus', 'SmpCryptoRetryQue']:
	smp += function(smp_source, name) + '\n'
smp += r'''
extern "C" void BtAppCheckStatus(void) { BtSmpCheckStatus(); }
int main() {
	assert(AppEvtHandlerInit(nullptr, 0));
	fill(); SmpCryptoRetryQue();
	assert(!s_bSmpCryptoRetryQueued && retries == 0);
	run();
	assert(retries == 1 && !AppEvtHandlerPending());
	BtSmpCheckStatus(); assert(!AppEvtHandlerPending());
	s_SmpLink[0].bRetryBusy = true;
	for (int i = 0; i < 10; ++i) BtSmpCheckStatus();
	run(); assert(retries == 2);
	s_SmpLink[0].ConnHdl = BT_CONN_HDL_INVALID;
	s_SmpLink[0].bRetryBusy = true;
	BtSmpCheckStatus(); assert(!AppEvtHandlerPending());
	puts("PASS: crypto retry survives a full queue and is queued once");
}
'''

bond_source = (ROOT / 'src/bluetooth/bt_smp_bond_nvm.cpp').read_text()
bond = COMMON
bond += bond_source[bond_source.index('#define BT_SMP_BOND_PEND_MAX'):
					bond_source.index('static void BondSaveHandler(uint32_t Evt, void *pCtx)\n{')]
bond += r'''
static unsigned saves;
static bool fail;
static void BondSaveHandler(uint32_t, void *) {
	s_SaveHandlerQueued.store(false);
	++saves;
	if (fail) BondSaveDeferRetry();
	else { s_PendMask.store(0); s_LocalIdPend.store(false); }
}
extern "C" void BtAppCheckStatus(void) { BtSmpBondNvmCheckStatus(); }
int main() {
	assert(AppEvtHandlerInit(nullptr, 0));
	fill(); s_PendMask.store(1); BondSaveSchedule();
	assert(!s_SaveHandlerQueued.load() && saves == 0);
	run(); assert(saves == 1 && !AppEvtHandlerPending());
	s_LocalIdPend.store(true);
	for (int i = 0; i < 10; ++i) BtSmpBondNvmCheckStatus();
	run(); assert(saves == 2);
	fail = true; s_PendMask.store(1); BondSaveSchedule();
	run(); assert(saves == 3);
	for (int delay = BT_SMP_BOND_RETRY_IDLE_CYCLES; delay > 0; --delay) {
		for (int i = 0; i < 100; ++i) BtSmpBondNvmCheckStatus();
		assert(!AppEvtHandlerPending() && saves == 3);
		assert(s_RetryIdleCycles.load() == delay);
		BtSmpBondNvmPoll();
	}
	fail = false; run();
	assert(saves == 4 && s_PendMask.load() == 0);
	puts("PASS: bond/local-ID saves recover; idle checks preserve timer backoff");
}
'''

sensor_cases = []
for name, path, timer, extra in [
		('thingy_sensor', 'ARM/Nordic/exemples/BlueIOThingy/BlueIOThingy.cpp',
		 'AppTimerHandler', ['-DUSE_TIMER_UPDATE']),
		('tph_sensor', 'ARM/Nordic/exemples/TPHSensorTag.cpp',
		 'TimerHandler', ['-DUSE_TIMER_UPDATE']),
		('tph_adv_timeout', 'ARM/Nordic/exemples/TPHSensorTag.cpp',
		 'TimerHandler', [])]:
	source = (ROOT / path).read_text()
	code = COMMON + r'''
struct TimerDev_t {};
#define TIMER_EVT_TRIGGER(n) (1U << (n))
static unsigned updates, starts, checks;
static bool interruptDuringRead, fillOnCheck, triggerOnCheck;
static void trigger();
static void ReadPTHData() {
	++updates;
	if (interruptDuringRead) {
		interruptDuringRead = false;
		trigger();
	}
}
static void BtAdvStart() { ++starts; }
extern "C" void BtAppCheckStatus() {
	++checks;
	if (triggerOnCheck) {
		triggerOnCheck = false;
		trigger();
	}
	if (fillOnCheck) {
		fillOnCheck = false;
		fill();
	}
}
'''
	start = source.index('static volatile bool s_bAdvDataPending')
	code += source[start:source.index(';', start) + 1] + '\n'
	for function_name in ['SchedAdvData', 'AdvDataQue', timer, 'AppCheckStatus']:
		code += function(source, function_name) + '\n'
	if timer == 'AppTimerHandler':
		code += 'static void trigger() { AppTimerHandler(nullptr, 0, nullptr); }\n'
		code += 'static void unrelated() { AppTimerHandler(nullptr, 1, nullptr); }\n'
	else:
		code += function(source, 'BtAppAdvTimeoutHandler') + '\n'
		code += 'static void trigger() { TimerHandler(nullptr, TIMER_EVT_TRIGGER(0)); }\n'
		code += 'static void unrelated() { TimerHandler(nullptr, TIMER_EVT_TRIGGER(1)); }\n'
	code += r'''
int main() {
	assert(AppEvtHandlerInit(nullptr, 0));
	assert(AppCheckStatus() && checks == 1);
	unrelated();
	assert(!s_bAdvDataPending && !AppEvtHandlerPending());
	fill();
	for (int i = 0; i < 10; ++i) trigger();
	assert(updates == 0); // No sensor or BLE work in the timer interrupt.
	// Bluetooth recovery takes the queue space before the sensor can retry.
	fillOnCheck = true;
	run();
	assert(updates == 1 && normal > 0 && !s_bAdvDataPending);
	assert(AppCheckStatus() && !AppEvtHandlerPending());
	for (int i = 0; i < 100; ++i) assert(AppCheckStatus());
	assert(updates == 1); // Status checks must not invent timer ticks.
	interruptDuringRead = true;
	trigger();
	run();
	assert(updates == 3 && !s_bAdvDataPending); // Request during read survives.
	triggerOnCheck = true; // Timer interrupt after AppRun's empty check.
	run();
	assert(updates == 4); // The status check must not queue it a second time.
	fillOnCheck = true;
	assert(!AppCheckStatus()); // Bluetooth work also prevents sleep.
	run();
#ifndef USE_TIMER_UPDATE
	BtAppAdvTimeoutHandler();
	assert(updates == 4);
	run();
	assert(updates == 5 && starts == updates);
#else
	assert(starts == 0);
#endif
	assert(AppCheckStatus());
	puts("PASS: sensor work is deferred, coalesced and recovered before sleep");
}
'''
	sensor_cases.append((name, code, extra))

# Compile each other production status override with and without the optional
# DFU module. This catches missing hooks and C/C++ linkage mismatches without
# needing a vendor SDK for the entire port.
dfu_cases = []
for name, path, stubs in [
		('generic', 'src/bluetooth/bt_app.cpp', ''),
		('nrf52', 'ARM/Nordic/nRF52/src/bt_app_nrf52.cpp', ''),
		('bm', 'ARM/Nordic/nRF54/src/bt_app_bm.cpp', r'''
#define BTAPP_STATE_UNKNOWN 0
#define CONFIG_NRF_SDH_BLE_TOTAL_LINK_COUNT 1
#define BLE_CONN_HANDLE_INVALID 65535
static struct { int State; } g_BtAppData = {0};
static uint16_t s_SecurePendingHdl[1] = {BLE_CONN_HANDLE_INVALID};
static void BtLescCheckStatus() {}
static void SecurePendingQue() {}
'''),
		('stm32wba', 'ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp', r'''
static bool s_bHciUserEvtOwed, s_bBleHostOwed;
static void hci_notify_asynch_evt(void *) {}
static void BLE_RESUME_FLOW_PROCESS_Callback() {}
''')]:
	source = (ROOT / path).read_text()
	code = COMMON + stubs + '\nextern "C" void BtAppCheckStatus(void);\n'
	code += '\n'.join(re.findall(
		r'^extern[^\n]*void Bt(?:DfuSmp|Lesc)CheckStatus[^\n]*weak[^\n]*;',
		source, re.M)) + '\n'
	code += function(source.replace('__attribute__((weak)) ', ''), 'BtAppCheckStatus')
	code += r'''
int main() {
	assert(AppEvtHandlerInit(nullptr, 0));
	run();
	assert(!AppEvtHandlerPending());
#ifdef WITH_SECURITY
	extern unsigned dfuChecks;
	assert(dfuChecks == 1);
#else
	assert(BtDfuSmpCheckStatus == nullptr);
#endif
	puts("PASS: production status check keeps DFU optional and calls it when linked");
}
'''
	dfu_cases.append((name, code))

with tempfile.TemporaryDirectory(prefix='iosonata-status-') as directory:
	directory = Path(directory)
	security = directory / 'security.cpp'
	security.write_text('unsigned cryptoChecks, bondChecks, dfuChecks;\n'
		'extern "C" void BtSmpCheckStatus() { ++cryptoChecks; }\n'
		'extern "C" void BtSmpBondNvmCheckStatus() { ++bondChecks; }\n'
		'void BtDfuSmpCheckStatus() { ++dfuChecks; }\n')
	port_cases = [(name + suffix, code, extra)
		for name, code in dfu_cases
		for suffix, extra in [('_no_dfu', []),
			('_dfu', ['-DWITH_SECURITY', str(security)])]]
	for name, code, extra in [('sdc', sdc, []),
			('sdc_security', sdc, ['-DWITH_SECURITY', str(security)]),
			('crypto', smp, []), ('bond', bond, []), *sensor_cases, *port_cases]:
		path = directory / (name + '.cpp')
		binary = directory / name
		path.write_text(code)
		subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O1',
			'-fsanitize=undefined', '-fno-sanitize-recover=all',
			'-I' + str(ROOT / 'include'), '-x', 'c++', str(path),
			str(ROOT / 'src/app_run.cpp'), str(ROOT / 'src/app_evt_handler.cpp'),
			str(ROOT / 'src/cfifo.c'), *extra, '-o', str(binary)], check=True)
		subprocess.run([str(binary)], check=True)
