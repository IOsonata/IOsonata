#!/usr/bin/env python3
"""Run example work paths with production AppEvt/CFifo and simulated I/O.

Queue refusal and FIFO preemption are deterministic injections. These checks
do not replace MCU builds or hardware traffic tests.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
SOURCE_ROOT = Path(os.environ.get('EXAMPLE_SOURCE_ROOT', ROOT))
CXX = shlex.split(os.environ.get('CXX', 'g++'))


def source(name):
	return (SOURCE_ROOT / name).read_text()


def function(text, name):
	match = re.search(r'(?:static\s+)?(?:void|bool|int)\s+' + name +
					  r'\([^;]*?\)\s*\{', text)
	assert match, name
	end, depth = match.end(), 1
	while depth:
		depth += (text[end] == '{') - (text[end] == '}')
		end += 1
	return text[match.start():end] + '\n'


def flag(text, name):
	match = re.search(r'static volatile bool ' + name + r'\s*=\s*false;', text)
	assert match, name
	return match.group() + '\n'


COMMON = r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
extern "C" void AppWait(void) {}
'''
cases = []

# Compile the actual admission/status functions. Peripheral callbacks are
# delivery probes, retaining their production signature and pending reset.
for name, items in [
	('usb_cdc_ble_central', [('UsbToBle', 'UsbToBleEvt'), ('BleToUsb', 'BleToUsbEvt')]),
	('uart_ble', [('UartRx', 'UartRxChedHandler'), ('SysLogFlush', 'SysLogFlushEvt')]),
	('uart_ble_bridge', [('UartRx', 'UartRxChedHandler')]),
	('uart_ble_central', [('UartRx', 'UartRxSchedHandler'), ('BleTx', 'BleTxSchedHandler')]),
	('ble_test_dut', [('DutUart', 'DutUartHandler'), ('DutBle', 'DutBleHandler')]),
	('bleintrf_prbs_tx', [('Prbs', 'PrbsChedHandler')])]:
	text = source('exemples/bluetooth/' + name + '.cpp')
	code = r'''
static unsigned calls[2], checks;
static bool refill;
static void Noop(uint32_t, void *) {}
static void Fill() { while (AppEvtHandlerQue(0, nullptr, Noop)) {} }
extern "C" void BtAppCheckStatus() {
	++checks;
	if (refill) { refill = false; Fill(); }
}
extern "C" void UsbCheckStatus() { ++checks; }
static void Drain() {
	for (int i = 0; i < 100; ++i) {
		if (!AppEvtHandlerExec() && AppCheckStatus()) return;
	}
	assert(false && "pending application work never drained");
}
'''
	for index, (prefix, handler) in enumerate(items):
		pending = 's_b' + prefix + 'Pending'
		code += flag(text, pending)
		callback = function(text, handler)
		reset = re.search(pending + r'\s*=\s*false;', callback)
		assert reset, handler
		code += callback[:callback.index('{') + 1] + reset.group()
		code += f' ++calls[{index}]; }}\n'
		code += function(text, prefix + 'Que')
	code += function(text, 'AppCheckStatus')
	code += 'int main() { assert(AppEvtHandlerInit(nullptr, 0));\n'
	code += 'for (unsigned cycle = 1; cycle <= 3; ++cycle) { Fill();\n'
	for prefix, handler in items:
		code += f'{prefix}Que(); {prefix}Que(); assert(s_b{prefix}Pending);\n'
	code += 'refill = true; Drain(); assert(checks && !AppEvtHandlerPending());\n'
	for index, (prefix, handler) in enumerate(items):
		code += f'assert(calls[{index}] == cycle && !s_b{prefix}Pending);\n'
	code += '} assert(AppCheckStatus()); }\n'
	cases.append((name + '_retry', code, []))

usb = source('exemples/bluetooth/usb_cdc_ble_central.cpp')
taktos = source('exemples/bluetooth/usb_cdc_ble_central_taktos.cpp')
central = source('exemples/bluetooth/uart_ble_central.cpp')

# A refused worker request must survive draining that worker's full queue.
code = r'''
#define TAKTOS_OK 0
struct Work_t { uint32_t EvtId; void *pCtx; void (*Handler)(uint32_t, void *); };
struct TaktOSQueue_t { Work_t items[4]; unsigned count; };
static TaktOSQueue_t s_UsbWorkQue, s_BleWorkQue;
static unsigned calls[3];
static int TaktOSQueueSend(TaktOSQueue_t *q, const Work_t *w, bool, int) {
	if (q->count == 4) return -1;
	q->items[q->count++] = *w; return TAKTOS_OK;
}
static void Noop(uint32_t, void *) {}
static void Drain(TaktOSQueue_t *q) {
	while (q->count) {
		Work_t w = q->items[0];
		memmove(q->items, q->items + 1, --q->count * sizeof(w));
		w.Handler(w.EvtId, w.pCtx);
	}
}
'''
for index, prefix in enumerate(['UsbRead', 'BleWrite', 'HostTx']):
	code += flag(taktos, 's_b' + prefix + 'Queued')
	code += flag(taktos, 's_b' + prefix + 'Owed')
	code += f'static void {prefix}Evt(uint32_t, void *) {{'
	code += f's_b{prefix}Queued = false; ++calls[{index}]; }}\n'
for name in ['WorkSend', 'Que', 'UsbReadQue', 'BleWriteQue', 'HostTxQue', 'BridgeCheckStatus']:
	code += function(taktos, name)
worker = function(taktos, 'WorkThread')
assert worker.index('BridgeCheckStatus(pQue);') < worker.index('&work, true, TAKTOS_WAIT_FOREVER')
code += r'''
int main() {
	for (int i = 0; i < 4; ++i) {
		assert(WorkSend(&s_UsbWorkQue, 0, nullptr, Noop));
		assert(WorkSend(&s_BleWorkQue, 0, nullptr, Noop));
	}
	UsbReadQue(); BleWriteQue(); HostTxQue();
	assert(s_bUsbReadOwed && s_bBleWriteOwed && s_bHostTxOwed);
	Drain(&s_UsbWorkQue);
	BridgeCheckStatus(&s_UsbWorkQue); BridgeCheckStatus(&s_UsbWorkQue);
	assert(s_UsbWorkQue.count == 2 && s_bBleWriteOwed);
	Drain(&s_UsbWorkQue);
	assert(calls[0] == 1 && calls[1] == 0 && calls[2] == 1);
	Drain(&s_BleWorkQue); BridgeCheckStatus(&s_BleWorkQue);
	assert(s_BleWorkQue.count == 1 && !s_bBleWriteOwed);
	Drain(&s_BleWorkQue); assert(calls[1] == 1);
	assert(!s_bUsbReadOwed && !s_bHostTxOwed);
}
'''
cases.append(('taktos_worker_retry', code, []))

# Inject an ISR producer immediately after release of a full FIFO's head.
code = r'''
alignas(8) static uint8_t mem[CFIFO_MEMSIZE(1024)];
static hCFifo_t s_hBleRxFifo;
static uint32_t s_BleRxDropCnt;
static volatile bool s_bBleToUsbPending;
static uint8_t s_UsbTxBuf[64], captured[2048];
static int s_UsbTxLen, s_UsbTxOff, capturedLen;
static bool inject = true;
static void BleToUsbQue(void) {}
struct Cdc {
	bool IsPortOpen() { return true; }
	int Tx(int, const uint8_t *p, int n) {
		memcpy(captured + capturedLen, p, n); capturedLen += n; return n;
	}
} g_Cdc;
'''
code += function(usb, 'ToHostPut') + r'''
static uint8_t *GetWithInterrupt(hCFifo_t fifo, int *len) {
	uint8_t *p = CFifoGetMultiple(fifo, len);
	if (p && inject) {
		inject = false;
		uint8_t next[64]; memset(next, 0x22, sizeof(next));
		ToHostPut(next, sizeof(next));
	}
	return p;
}
'''
code += function(usb, 'BleToUsbEvt').replace('CFifoGetMultiple(', 'GetWithInterrupt(')
code += r'''
int main() {
	s_hBleRxFifo = CFifoInit(mem, sizeof(mem), 1, true);
	uint8_t original[1024]; memset(original, 0x11, sizeof(original));
	ToHostPut(original, sizeof(original)); BleToUsbEvt(0, nullptr);
	assert(capturedLen == 1088 && !s_BleRxDropCnt);
	for (int i = 0; i < 1088; ++i) assert(captured[i] == (i < 1024 ? 0x11 : 0x22));
}
'''
cases.append(('usb_fifo_irq_ownership', code, []))

# A runnable consumer can switch in at publication, including at FIFO wrap.
code = r'''
alignas(8) static uint8_t mem[CFIFO_MEMSIZE(512)];
static hCFifo_t s_hToBleFifo;
static volatile bool s_bUsbReadQueued;
static uint8_t received[USB_PKT_SIZE];
static int receivedLen;
static bool once = true;
static void BleWriteQue(void) {}
struct Cdc { int Rx(int, uint8_t *p, int n) {
	if (!once) return 0;
	once = false;
	for (int i = 0; i < n; ++i) p[i] = uint8_t(i + 1);
	return n;
} } g_Cdc;
static uint8_t *PutWithSwitch(hCFifo_t fifo, int *len) {
	uint8_t *p = CFifoPutMultiple(fifo, len);
	if (p) {
		int n = *len;
		uint8_t *q = CFifoGetMultiple(fifo, &n);
		assert(q && n == *len);
		memcpy(received + receivedLen, q, n); receivedLen += n;
	}
	return p;
}
'''
code += function(taktos, 'UsbReadEvt').replace('CFifoPutMultiple(', 'PutWithSwitch(')
code += r'''
int main() {
	s_hToBleFifo = CFifoInit(mem, sizeof(mem), 1, true);
	int n = 480; assert(CFifoPutMultiple(s_hToBleFifo, &n));
	assert(CFifoGetMultiple(s_hToBleFifo, &n));
	UsbReadEvt(0, nullptr);
	assert(receivedLen == USB_PKT_SIZE && CFifoUsed(s_hToBleFifo) == 0);
	for (int i = 0; i < receivedLen; ++i) assert(received[i] == uint8_t(i + 1));
}
'''
for size in [64, 512]:
	cases.append(('taktos_fifo_publish_' + str(size), code, ['-DUSB_PKT_SIZE=' + str(size)]))

# Refused BLE writes keep exactly the same bytes until acceptance.
code = r'''
#define BLE_WRITE_MAX 20
#define BT_CONN_HDL_INVALID 0xffff
#define BT_ATT_HANDLE_INVALID 0xffff
struct Led { int PortNo, PinNo; };
static const Led s_Leds[] = {{0, 0}};
static void IOPinToggle(int, int) {}
struct Conn { uint16_t Hdl; };
struct Dev { struct Conn Conn; } g_ConnectedDev = {{1}};
static uint16_t g_BleTxCharHdl = 2;
static hCFifo_t g_UartRx2BleFifo;
static bool accept;
static int sent;
static uint8_t received[60];
static bool BtAppWrite(uint16_t, uint16_t, const uint8_t *p, int n) {
	if (!accept) return false;
	memcpy(received + sent, p, n); sent += n; return true;
}
void BleTxSchedHandler(uint32_t, void *);
'''
code += flag(central, 's_bBleTxPending')
code += function(central, 'BleTxQue') + function(central, 'BleTxSchedHandler')
code += r'''
int main() {
	alignas(8) uint8_t mem[CFIFO_MEMSIZE(128)];
	g_UartRx2BleFifo = CFifoInit(mem, sizeof(mem), 1, true);
	int len = 60;
	uint8_t *p = CFifoPutMultiple(g_UartRx2BleFifo, &len);
	for (int i = 0; i < len; ++i) p[i] = i;
	assert(AppEvtHandlerInit(nullptr, 0)); BleTxQue();
	for (int i = 0; i < 3; ++i) AppEvtHandlerDispatch();
	assert(!sent && CFifoUsed(g_UartRx2BleFifo) == 60);
	accept = true;
	for (int i = 0; i < 10 && AppEvtHandlerPending(); ++i) AppEvtHandlerExec();
	assert(sent == 60 && CFifoUsed(g_UartRx2BleFifo) == 0 && !AppEvtHandlerPending());
	for (int i = 0; i < sent; ++i) assert(received[i] == i);
}
'''
cases.append(('central_write_refusal', code, []))

# A producer can reuse a full FIFO's released head before the BLE task resumes.
code = r'''
#define BLE_WRITE_MAX 20
alignas(8) static uint8_t mem[CFIFO_MEMSIZE(512)];
static hCFifo_t s_hToBleFifo;
static volatile bool s_bBleWriteQueued;
static uint8_t s_BleTxBuf[BLE_WRITE_MAX], received[532];
static int s_BleTxLen, receivedLen;
static uint16_t s_ConnHdl, s_BleTxCharHdl;
static bool inject = true;
static bool BridgeReady() { return true; }
static void UsbReadQue() {}
static void BleWriteQue() {}
static int TaktOSCurrentThread() { return 0; }
static void TaktOSThreadSleepTicks(int, int) {}
static bool BtAppWrite(uint16_t, uint16_t, const uint8_t *p, int n) {
	memcpy(received + receivedLen, p, n); receivedLen += n; return true;
}
static uint8_t *GetWithSwitch(hCFifo_t fifo, int *len) {
	uint8_t *p = CFifoGetMultiple(fifo, len);
	if (p && inject) {
		inject = false;
		int n = BLE_WRITE_MAX;
		uint8_t *q = CFifoResvMultiple(fifo, &n);
		assert(q && n == BLE_WRITE_MAX);
		memset(q, 0x22, n); (void)CFifoPutMultiple(fifo, &n);
	}
	return p;
}
'''
code += function(taktos, 'BleWriteEvt').replace('CFifoGetMultiple(', 'GetWithSwitch(')
code += r'''
int main() {
	s_hToBleFifo = CFifoInit(mem, sizeof(mem), 1, true);
	int n = 512;
	uint8_t *p = CFifoPutMultiple(s_hToBleFifo, &n); memset(p, 0x11, n);
	BleWriteEvt(0, nullptr);
	assert(receivedLen == 532 && CFifoUsed(s_hToBleFifo) == 0);
	for (int i = 0; i < receivedLen; ++i) assert(received[i] == (i < 512 ? 0x11 : 0x22));
}
'''
cases.append(('taktos_fifo_release', code, []))

# A partial FIFO put must retain only the uncopied UART suffix.
code = r'''
#define PACKET_SIZE 32
#define BLE_SC_METHOD 0
#define BLE_SC_NONE 0
static bool s_bUartRxPending;
static uint8_t g_UartRxExtBuff[PACKET_SIZE];
static int g_UartRxExtBuffLen, g_DropCnt;
static hCFifo_t g_UartRx2BleFifo;
static void BleTxQue() {}
static void UartRxQue() {}
static bool UartBleSecTryCommand(const uint8_t *, int) { return false; }
static bool UartBleOobTryCommand(const uint8_t *, int) { return false; }
struct Uart { int Rx(uint8_t *, int) { return 0; } } g_Uart;
'''
code += function(central, 'UartRxSchedHandler')
code += r'''
int main() {
	alignas(8) uint8_t mem[CFIFO_MEMSIZE(16)];
	g_UartRx2BleFifo = CFifoInit(mem, sizeof(mem), 1, true);
	int n = 12; assert(CFifoPutMultiple(g_UartRx2BleFifo, &n));
	assert(CFifoGetMultiple(g_UartRx2BleFifo, &n));
	g_UartRxExtBuffLen = 12;
	for (int i = 0; i < 12; ++i) g_UartRxExtBuff[i] = i + 1;
	UartRxSchedHandler(0, nullptr);
	assert(g_UartRxExtBuffLen == 8 && CFifoUsed(g_UartRx2BleFifo) == 4);
	for (int i = 0; i < 8; ++i) assert(g_UartRxExtBuff[i] == i + 5);
	UartRxSchedHandler(0, nullptr);
	assert(g_UartRxExtBuffLen == 0 && CFifoUsed(g_UartRx2BleFifo) == 12);
	for (int i = 0; i < 12; ++i) assert(*CFifoGet(g_UartRx2BleFifo) == i + 1);
}
'''
cases.append(('central_partial_fifo_put', code, []))

for name, path in [('taktos', 'exemples/bluetooth/uart_ble_taktos.cpp'),
				   ('freertos', 'ARM/Nordic/exemples/UartBleFreeRTOS.cpp')]:
	text = source(path)
	code = r'''
static unsigned services, dispatches;
static int g_UartBleSrvc;
static bool BtGattSrvcAdd(int *) { ++services; return true; }
static void BtGattEvtHandler(uint32_t, void *) { ++dispatches; }
'''
	code += function(text, 'BtAppInitUserServices')
	code += function(text, 'BtAppPeriphEvtHandler')
	code += r'''
int main() {
	BtAppInitUserServices(); assert(services == 1);
	BtGattEvtHandler(1, nullptr); // Port dispatches before the application callback.
	BtAppPeriphEvtHandler(1, nullptr); assert(dispatches == 1);
}
'''
	cases.append((name + '_service_hooks', code, []))

with tempfile.TemporaryDirectory(prefix='bt-example-work-') as directory:
	for name, code, extra in cases:
		cpp = Path(directory) / (name + '.cpp')
		binary = cpp.with_suffix('')
		cpp.write_text(COMMON + code)
		subprocess.run(CXX + ['-std=gnu++17', '-O1', '-g', '-fsanitize=undefined',
			'-fno-sanitize-recover=all', '-I' + str(ROOT / 'include'), str(cpp),
			str(ROOT / 'src/app_evt_handler.cpp'), str(ROOT / 'src/app_run.cpp'),
			str(ROOT / 'src/cfifo.c'), '-o', str(binary)] + extra, check=True)
		subprocess.run([str(binary)], check=True, timeout=10)
		print('PASS:', name, flush=True)
