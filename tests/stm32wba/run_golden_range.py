#!/usr/bin/env python3
"""Check the RSSI golden range the WBA port gives the link layer.

The ST host resets the LE power control golden range to -77 to -68 dBm in
LECTRL_Reset (HCI reset inside BleStack_Init). The Basic Plus link layer then
asks a peer that supports LE power control to lower its TX power until the
received level is in that range. The port writes the app_conf.h range with
ACI_HAL_WRITE_CONFIG_DATA after BleStack_Init. This test runs the port
function against a mock of that command and checks the bytes, without
hardware.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def extract_function(text, name):
    match = re.search(r'^static void ' + name + r'\(void\)\s*\{', text, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end]


source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp').read_text()
appconf = (ROOT / 'ARM/ST/STM32WBAxx/STM32WBA65/include/app_conf.h').read_text()
defines = ''.join(re.findall(r'#ifndef CFG_BLE_RSSI_GOLDEN_RANGE_\w+\n#define .*\n#endif\n', appconf))
assert defines.count('#define') == 2

code = r'''
#include <cassert>
#include <cstdint>
#define SysLogPrintf(...) ((void)0)
#define SysLogGet() nullptr
typedef uint8_t tBleStatus;
#define BLE_STATUS_SUCCESS 0
// ble_defs.h values
#define CONFIG_DATA_LL_RSSI_GOLDEN_RANGE_OFFSET 0xC2U
#define CONFIG_DATA_LL_RSSI_GOLDEN_RANGE_LEN 2
static int calls;
static int8_t lower, upper;
static tBleStatus aci_hal_write_config_data(uint8_t Offset, uint8_t Length, const uint8_t *pValue)
{
	assert(Offset == 0xC2 && Length == 2);
	calls++;
	lower = (int8_t)pValue[0];
	upper = (int8_t)pValue[1];
	// ll_intf_set_cstm_rssi_golden_range refuses lower > upper (0x12)
	return lower <= upper ? BLE_STATUS_SUCCESS : 0x12;
}
''' + defines + extract_function(source, 'BtAppWbaGoldenRangeSet') + r'''
int main()
{
	BtAppWbaGoldenRangeSet();
	assert(calls == 1 && lower <= upper);
	// Host default range set by LECTRL_Reset, which the link layer aims at
	const int8_t stLower = -77, stUpper = -68;
	assert(lower <= stLower && upper >= stUpper);
	// Every level the receiver reports is inside: no autonomous request
	assert(lower <= -100 && upper >= 0);
	return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='wba-golden-range-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    binary = Path(temp) / 'test'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O2', '-Wall',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
    print('PASS WBA RSSI golden range: one write after init, covers every received level')
