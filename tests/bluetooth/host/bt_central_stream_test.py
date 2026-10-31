#!/usr/bin/env python3
"""Exercise production central buffer setup and UART draining with fake I/O."""
import os
from pathlib import Path
import re
import subprocess
import tempfile

from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
SOURCE_ROOT = Path(os.environ.get('CENTRAL_SOURCE_ROOT', ROOT))
CXX = os.environ.get('CXX', 'g++')
app = (SOURCE_ROOT / 'src/bluetooth/bt_app_hci.cpp').read_text()
central = (SOURCE_ROOT / 'exemples/bluetooth/uart_ble_central.cpp').read_text()
defines = '\n'.join(re.findall(r'^#define BT_APP_HCI_\w*PACKET_COUNT\s+\d+', app, re.M))
setup = r"""
#include <cassert>
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci_ctlr.h"
static BtHciDevice_t s_BtHciDev;
static BtHciCtlrDev_t s_BtHciCtlr;
static BtHciCtlrCfg_t captured;
static bool enabled = true;
static void BtAppCtlrRx(BtHciCtlrDev_t *, bool, uint8_t *) {}
extern "C" bool BtHciCtlrEnable(BtHciCtlrDev_t *, const BtHciCtlrCfg_t *cfg) {
	captured = *cfg; return enabled;
}
extern "C" void BtCryptoCtlrReset() {}
static bool BtAppAclBufRead(uint16_t *len, uint8_t *count) {
	*len = 251; *count = 10; return true;
}
void BtHciSetLeAclBuffer(BtHciDevice_t *, uint16_t, uint8_t) {}
#define DEBUG_PRINTF(...)
""" + defines + '\n' + extract_function(app, r'^bool BtAppStackInit\(') + r"""
int main() {
	BtAppCfg_t cfg = {};
	const BTAPP_ROLE roles[] = {BTAPP_ROLE_CENTRAL, BTAPP_ROLE_MIXED,
		BTAPP_ROLE_PERIPHERAL, BTAPP_ROLE_OBSERVER};
	for (auto role : roles) {
		cfg.Role = role;
		assert(BtAppStackInit(&cfg));
		assert(captured.RxPktCount == ((role & BTAPP_ROLE_CENTRAL) ? 10 : 2));
		assert(captured.TxPktCount == 10);
		assert(captured.MaxDataLen == 251);
	}
	enabled = false;
	assert(!BtAppStackInit(&cfg));
}
"""
drain = r"""
#include <cassert>
#include <cstring>
#include "cfifo.h"
#define UART_MAX_DATA_LEN 128
static hCFifo_t s_hBleToUartFifo;
static unsigned s_BleToUartAccepted;
struct FakeUart {
	int room = 0;
	int limit = 1000;
	int used = 0;
	unsigned calls = 0;
	uint8_t data[1024];
	bool RequestToSend(int n) { return n <= room; }
	int Tx(uint8_t *p, int n) {
		++calls;
		assert(n <= room); // A blocking UART would wait if this fails.
		if (n > limit) n = limit;
		memcpy(data + used, p, n); used += n; room -= n;
		return n;
	}
} g_Uart;
""" + extract_function(central, r'^static void BleToUartDrain\(') + r"""
int main() {
	alignas(8) uint8_t mem[CFIFO_MEMSIZE(512)];
	s_hBleToUartFifo = CFifoInit(mem, sizeof(mem), 1, true);
	int n = 400;
	assert(CFifoPutMultiple(s_hBleToUartFifo, &n));
	assert(CFifoGetMultiple(s_hBleToUartFifo, &n));
	for (unsigned i = 0; i < 400; ++i) *CFifoPut(s_hBleToUartFifo) = uint8_t(i);
	BleToUartDrain();
	assert(g_Uart.calls == 0 && CFifoUsed(s_hBleToUartFifo) == 400);
	g_Uart.room = 37; g_Uart.limit = 7; // partial writes and FIFO wrap
	BleToUartDrain();
	assert(g_Uart.used == 37 && CFifoUsed(s_hBleToUartFifo) == 363);
	g_Uart.room = 1000; g_Uart.limit = 1000;
	BleToUartDrain();
	assert(g_Uart.used == 293); // bounded work: 256 bytes this pass
	BleToUartDrain();
	assert(g_Uart.used == 400 && s_BleToUartAccepted == 400);
	assert(CFifoUsed(s_hBleToUartFifo) == 0);
	for (unsigned i = 0; i < 400; ++i) assert(g_Uart.data[i] == uint8_t(i));
}
"""
discovery = r"""
#include <cassert>
#include <cstring>
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_att.h"
static BtDevice_t peers[2];
static BtGattDBSrvc_t cache[2][BT_DEV_SERVICE_MAXCNT];
static BtHciDevice_t hci;
static unsigned mtuCalls, serviceCalls;
static bool mtuAccept = true, serviceAccept = true;
static uint16_t localMtu = 247;
extern "C" uint16_t BtAttGetMtu() { return localMtu; }
extern "C" BtDevice_t *BtPeerFindByHdl(uint16_t h) { return h < 2 ? &peers[h] : nullptr; }
extern "C" uint16_t BtPeerCount() { return 2; }
extern "C" BtDevice_t *BtPeerSlot(uint16_t i) { return i < 2 ? &peers[i] : nullptr; }
extern "C" bool BtDeviceSrvcCacheAttach(BtDevice_t *p) {
 p->pServices = cache[p->Conn.Hdl]; return true;
}
extern "C" bool BtAttExchangeMtuRequest(BtHciDevice_t *, uint16_t h, uint16_t mtu) {
 ++mtuCalls; assert(mtu == 247);
 peers[h].bAttReqPending = mtuAccept; return mtuAccept;
}
extern "C" bool BtAttStartReadByGroupTypeRequest(BtHciDevice_t *, uint16_t h,
 uint16_t first, uint16_t last, BtUuid_t *uuid) {
 assert(first == 1 && last == 0xffff);
 assert(uuid->Uuid16 == BT_UUID_DECLARATIONS_PRIMARY_SERVICE);
 assert(!peers[h].bAttReqPending);
 ++serviceCalls; peers[h].bAttReqPending = serviceAccept; return serviceAccept;
}
""" + '\n'.join(re.findall(r'^#define BT_APP_HCI_DISC_\w+\s+\d+', app, re.M)) + '\n'
for name in ['BtAppDiscoverDevice', 'BtAppHciMtuExchanged', 'BtAppHciDiscoveryRetry']:
 discovery += extract_function(app, r'^(?:static )?(?:bool|void) ' + name + r'\(') + '\n'
discovery += r"""
int main() {
 for (unsigned i = 0; i < 2; ++i) {
  peers[i].Conn.Hdl = i; peers[i].Conn.MaxMtu = 23; peers[i].pHciDev = &hci;
 }
 assert(!BtAppDiscoverDevice(nullptr));
 mtuAccept = false;
 assert(!BtAppDiscoverDevice(&peers[0]));
 assert(peers[0].Discovery.Phase == 0);
 mtuAccept = true;
 assert(BtAppDiscoverDevice(&peers[0]));
 assert(BtAppDiscoverDevice(&peers[1]));
 assert(!BtAppDiscoverDevice(&peers[0]));
 assert(serviceCalls == 0 && mtuCalls == 3);
 // Response on link 1 must not advance link 0.
 peers[1].bAttReqPending = false;
 peers[1].Conn.MaxMtu = 247;
 BtAppHciMtuExchanged(1);
 assert(serviceCalls == 1 && peers[0].Discovery.Phase == 1);
 // Reject the next service request to simulate exhausted controller credits.
 peers[0].bAttReqPending = false;
 serviceAccept = false;
 BtAppHciMtuExchanged(0);
 assert(peers[0].Discovery.Phase == 2);
 serviceAccept = true;
 BtAppHciDiscoveryRetry();
 assert(serviceCalls == 3 && mtuCalls == 3);
 assert(peers[0].Discovery.Phase == 3);
 peers[0].bAttReqPending = false;
 assert(BtAppDiscoverDevice(&peers[0]));
 assert(mtuCalls == 3); // Rediscovery does not repeat the MTU exchange.
 peers[0].bAttReqPending = false;
 peers[0].Discovery.Phase = 0;
 localMtu = 23;
 assert(BtAppDiscoverDevice(&peers[0]));
 assert(mtuCalls == 3);
}
"""
with tempfile.TemporaryDirectory(prefix='central-stream-') as temp:
	for name, code, extra in [
		('buffers', setup, []),
		('discovery', discovery, []),
		('uart', drain, [str(ROOT / 'src/cfifo.c')]),
	]:
		cpp = Path(temp) / (name + '.cpp')
		cpp.write_text(code)
		binary = cpp.with_suffix('')
		subprocess.run([CXX, '-std=gnu++20', '-O2', '-Wall', '-Wextra',
			'-fsanitize=undefined', '-fno-sanitize-recover=all',
			'-I' + str(ROOT / 'include'), str(cpp), *extra, '-o', str(binary)], check=True)
		subprocess.run([str(binary)], check=True)
		print('PASS central streaming:', name)
