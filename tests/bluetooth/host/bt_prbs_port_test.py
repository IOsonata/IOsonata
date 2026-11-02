#!/usr/bin/env python3
"""Exercise SoftDevice implementations required by the generic PRBS receiver."""
from pathlib import Path
import subprocess
import tempfile
from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
# nRF52 and nRF54 SoftDevice ports
PORTS = (
    ('nRF52', 'ARM/Nordic/nRF52/src/bt_app_nrf52.cpp', 'ARM/Nordic/nRF52/src/bt_gap_nrf52.cpp'),
    ('nRF54', 'ARM/Nordic/nRF54/src/bt_app_bm.cpp', 'ARM/Nordic/nRF54/src/bt_gap_bm.cpp'),
)
head = r'''
#include <cassert>
#include <cstring>
#include "bluetooth/bt_app.h"
#define NRF_SUCCESS 0
struct ble_gap_data_length_params_t
{
	uint16_t max_tx_octets;
	uint16_t max_rx_octets;
	uint16_t max_tx_time_us;
	uint16_t max_rx_time_us;
};
static ble_gap_data_length_params_t requested;
static unsigned requests;
static unsigned result;
static uint32_t s_BtAppPhy[BT_DEV_CONN_MAX];
static unsigned sd_ble_gap_data_length_update(uint16_t ConnHdl,
	const ble_gap_data_length_params_t *pParams, void *)
{
	assert(ConnHdl == 7);
	requested = *pParams;
	requests++;
	return result;
}
'''
test = r'''
int main(void)
{
	uint8_t tx = 9;
	uint8_t rx = 9;
	int i;

	assert(!BtGapReadPhy(0, &tx, &rx));
	assert(tx == 9 && rx == 9);
	BtAppPhySet(0, 1, 1);
	BtAppPhySet(7, 1, 1);
	assert(BtGapReadPhy(0, &tx, &rx) && tx == 1 && rx == 1);
	BtAppPhySet(7, 2, 1);
	assert(BtGapReadPhy(7, &tx, &rx) && tx == 2 && rx == 1);
	assert(BtGapReadPhy(0, &tx, &rx) && tx == 1 && rx == 1);
	assert(!BtGapReadPhy(7, nullptr, &rx));
	assert(!BtGapReadPhy(7, &tx, nullptr));
	assert(!BtGapReadPhy(BT_CONN_HDL_INVALID, &tx, &rx));
	BtAppPhySet(7, 0, 0);
	assert(!BtGapReadPhy(7, &tx, &rx));
	BtAppPhySet(0, 0, 0);
	for (i = 0; i < BT_DEV_CONN_MAX; i++)
	{
		BtAppPhySet(i + 100, 1, 1);
	}
	for (i = 0; i < BT_DEV_CONN_MAX; i++)
	{
		assert(BtGapReadPhy(i + 100, &tx, &rx) && tx == 1 && rx == 1);
		BtAppPhySet(i + 100, 0, 0);
	}
	BtAppPhySet(7, 1, 1);
	assert(BtGapReadPhy(7, &tx, &rx) && tx == 1 && rx == 1);
	assert(BtGapSetDataLength(7, 251, 2120));
	assert(requested.max_tx_octets == 251 && requested.max_tx_time_us == 2120);
	assert(requested.max_rx_octets == 0 && requested.max_rx_time_us == 0);
	assert(!BtGapSetDataLength(7, 26, 2120));
	assert(!BtGapSetDataLength(7, 252, 2120));
	assert(!BtGapSetDataLength(7, 251, 327));
	assert(requests == 1);
	result = 1;
	assert(!BtGapSetDataLength(7, 251, 2120));
	assert(requests == 2);
}
'''
for name, app_path, gap_path in PORTS:
    app = (ROOT / app_path).read_text()
    gap = (ROOT / gap_path).read_text()
    code = head
    code += extract_function(app, r'^static void BtAppPhySet\(')
    code += extract_function(app, r'^bool BtGapReadPhy\(')
    code += extract_function(gap, r'^bool BtGapSetDataLength\(')
    code += test
    with tempfile.TemporaryDirectory(prefix='prbs-port-') as tmp:
        cpp = Path(tmp) / 'test.cpp'
        exe = Path(tmp) / 'test'
        cpp.write_text(code)
        for opt in ('-O0', '-O2'):
            subprocess.run(['g++', '-std=gnu++17', opt, '-Wall', '-Wextra',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            '-I' + str(ROOT / 'include'), str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
    print('PASS %s SoftDevice PHY records, handle reuse, DLE arguments and errors (O0/O2 UBSan)' % name)
