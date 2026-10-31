#!/usr/bin/env python3
"""Check central link setup and notification accounting before forwarding."""
import os
from pathlib import Path
import subprocess
import tempfile
from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / 'exemples/bluetooth/uart_ble_central.cpp').read_text()
code = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include "cfifo.h"
#define CENTRAL_PHY 2
#define BLE_SC_METHOD 0
#define BLE_SC_NONE 0
struct { struct { uint16_t Hdl; } Conn; } g_ConnectedDev;
struct { void printf(const char *, ...) {} } g_Uart;
static bool phyAccepted = true;
static unsigned phyCalls, discoveryCalls;
static uint16_t discoveryHandle;
static bool BtGapSetPhy(uint16_t h, uint8_t tx, uint8_t rx, uint16_t opt) {
 assert(h == g_ConnectedDev.Conn.Hdl && tx == 2 && rx == 2 && opt == 0);
 ++phyCalls; return phyAccepted;
}
static void StartPeerDiscovery(uint16_t h) { ++discoveryCalls; discoveryHandle = h; }
static uint16_t g_BleRxCharHdl = 0x12;
static hCFifo_t s_hBleToUartFifo;
static volatile uint32_t s_BleToUartReceived, s_BleToUartDropped;
static volatile uint32_t s_BleNotifications, s_BleFirstRxMs;
static volatile uint16_t s_BleNotifyMinLen = UINT16_MAX, s_BleNotifyMaxLen;
static unsigned queued;
static uint32_t nowMs;
static uint32_t BtGattMsTick() { return nowMs; }
static void BleToUartQue() { ++queued; }
'''
for signature in [r'^void BtAppEvtConnected\(', r'^void BtGattClientNotified\(']:
 code += extract_function(source, signature) + '\n'
code += r'''
int main() {
 BtAppEvtConnected(7);
 assert(phyCalls == 1 && discoveryCalls == 1 && discoveryHandle == 7);
 phyAccepted = false;
 BtAppEvtConnected(9);
 assert(phyCalls == 2 && discoveryCalls == 2 && discoveryHandle == 9);
 alignas(8) uint8_t mem[CFIFO_MEMSIZE(4096)];
 uint8_t data[244];
 for (unsigned i = 0; i < sizeof(data); ++i) data[i] = uint8_t(i);
 s_hBleToUartFifo = CFifoInit(mem, sizeof(mem), 1, true);
 nowMs = 100;
 BtGattClientNotified(8, 0x12, data, sizeof(data));
 BtGattClientNotified(9, 0x13, data, sizeof(data));
 BtGattClientNotified(9, 0x12, nullptr, sizeof(data));
 BtGattClientNotified(9, 0x12, data, 0);
 assert(s_BleNotifications == 0 && queued == 0);
 for (unsigned burst = 0; burst < 3; ++burst) {
  for (unsigned i = 0; i < 8; ++i) BtGattClientNotified(9, 0x12, data, sizeof(data));
  assert(CFifoUsed(s_hBleToUartFifo) == 1952);
  for (unsigned i = 0; i < 1952; ++i) {
   auto p = CFifoGet(s_hBleToUartFifo);
   assert(p && *p == data[i % 244]);
  }
  nowMs += 15;
 }
 assert(s_BleNotifications == 24 && s_BleToUartReceived == 5856);
 assert(s_BleNotifyMinLen == 244 && s_BleNotifyMaxLen == 244);
 assert(s_BleFirstRxMs == 100 && s_BleToUartDropped == 0);
 // A full forwarding FIFO must not hide notifications received over BLE.
 int n = 4096;
 while (CFifoPutMultiple(s_hBleToUartFifo, &n)) n = 4096;
 BtGattClientNotified(9, 0x12, data, 20);
 assert(s_BleNotifications == 25 && s_BleToUartReceived == 5876);
 assert(s_BleToUartDropped == 20 && s_BleNotifyMinLen == 20);
}
'''
with tempfile.TemporaryDirectory(prefix='central-receive-') as temp:
 cpp = Path(temp) / 'test.cpp'
 cpp.write_text(code)
 binary = Path(temp) / 'test'
 subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O2',
  '-Wall', '-Wextra', '-fsanitize=undefined', '-fno-sanitize-recover=all',
  '-I' + str(ROOT / 'include'), str(cpp), str(ROOT / 'src/cfifo.c'),
  '-o', str(binary)], check=True)
 subprocess.run([str(binary)], check=True)
 print('PASS central PHY request, notification bursts, FIFO wrap and pre-forwarding counts')
