#!/usr/bin/env python3
"""Verify port-owned discovery admission while SoftDevice GATT is busy."""
from pathlib import Path
import re
import subprocess
import tempfile
from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
source = (ROOT / 'ARM/Nordic/nRF52/src/bt_app_nrf52.cpp').read_text()
code = r'''
#include <cassert>
#include <cstring>
#include "bluetooth/bt_app.h"
#define NRF_SUCCESS 0
#define NRF_ERROR_BUSY 17
static BtDevice_t peers[2];
static BtGattDBSrvc_t cache[2][BT_DEV_SERVICE_MAXCNT];
static unsigned result[2];
static unsigned calls[2];
static unsigned disconnected;
static bool cacheAvailable = true;
extern "C" uint16_t BtPeerCount(void) { return 2; }
extern "C" BtDevice_t *BtPeerSlot(uint16_t i) { return &peers[i]; }
extern "C" bool BtDeviceSrvcCacheAttach(BtDevice_t *p)
{
	if (!cacheAvailable) return false;
	p->pServices = cache[p->Conn.Hdl];
	return true;
}
void BtAppDisconnectConn(uint16_t h)
{
	assert(h == 1);
	disconnected++;
}
static unsigned sd_ble_gattc_primary_services_discover(uint16_t h,
	uint16_t start, const void *uuid)
{
	assert(h < 2 && start == 1 && uuid == nullptr);
	calls[h]++;
	return result[h];
}
'''
code += re.search(r'enum \{ BTAPP_DISC_IDLE.*?;', source).group() + '\n'
for name in ('BtAppDiscoverDevice', 'BtAppDiscoveryRetry'):
    code += extract_function(source, r'^(?:static )?(?:bool|void) ' + name + r'\(') + '\n'
code += r'''
int main()
{
	peers[0].Conn.Hdl = 0;
	peers[1].Conn.Hdl = 1;
	assert(!BtAppDiscoverDevice(nullptr));
	cacheAvailable = false;
	assert(!BtAppDiscoverDevice(&peers[0]));
	assert(calls[0] == 0);
	cacheAvailable = true;
	result[0] = NRF_ERROR_BUSY;
	result[1] = NRF_ERROR_BUSY;
	assert(BtAppDiscoverDevice(&peers[0]));
	assert(BtAppDiscoverDevice(&peers[1]));
	assert(peers[0].Discovery.Phase == BTAPP_DISC_PENDING);
	// MTU completion frees one bearer; the other remains pending.
	result[0] = NRF_SUCCESS;
	BtAppDiscoveryRetry();
	assert(peers[0].Discovery.Phase == BTAPP_DISC_ACTIVE);
	assert(peers[1].Discovery.Phase == BTAPP_DISC_PENDING);
	assert(calls[0] == 2 && calls[1] == 2);
	// A duplicate request cannot clear an in-progress discovery database.
	peers[0].NbSrvc = 1;
	assert(BtAppDiscoverDevice(&peers[0]));
	assert(calls[0] == 2 && peers[0].NbSrvc == 1);
	// A permanent retry failure stops retrying and closes that link.
	result[1] = 99;
	BtAppDiscoveryRetry();
	assert(disconnected == 1 && peers[1].Discovery.Phase == BTAPP_DISC_IDLE);
	BtAppDiscoveryRetry();
	assert(calls[1] == 3 && disconnected == 1);
	// Free/reuse clears pending state, so the old request cannot restart.
	memset(&peers[1], 0, sizeof(peers[1]));
	peers[1].Conn.Hdl = BT_CONN_HDL_INVALID;
	BtAppDiscoveryRetry();
	assert(calls[1] == 3);
	assert(!BtAppDiscoverDevice(&peers[1]));
	peers[1].Conn.Hdl = 1;
	assert(!BtAppDiscoverDevice(&peers[1]));
	assert(disconnected == 1);
}
'''
with tempfile.TemporaryDirectory(prefix='sd-discovery-') as tmp:
    cpp = Path(tmp) / 'test.cpp'
    exe = Path(tmp) / 'test'
    cpp.write_text(code)
    for opt in ('-O0', '-O2'):
        subprocess.run(['g++', '-std=gnu++17', opt, '-Wall', '-Wextra',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(ROOT / 'include'), str(cpp), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
print('PASS SoftDevice discovery busy retry, independent links, errors and handle reuse')
