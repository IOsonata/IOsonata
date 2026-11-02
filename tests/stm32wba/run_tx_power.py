#!/usr/bin/env python3
"""Check the WBA mapping of BtAppCfg_t::TxPower to the ST PA level.

TxPower is in dBm on every port; 0 is 0 dBm. The WBA port always calls
aci_hal_set_tx_power_level with the closest PA level, so 0 selects 0x19
(-0.3 dBm) and not the stack default, the maximum level. This test runs the
port mapping and checks the call in BtAppStackInit, without hardware.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp').read_text()

# The TX power call in the stack init is unconditional
init = source[source.index('bool BtAppStackInit('):]
init = init[:init.index('\n}\n')]
call = re.search(r'\n([^\n]*)aci_hal_set_tx_power_level\(0, BtAppWbaPaLevel\(pCfg->TxPower\)\);', init)
assert call, 'aci_hal_set_tx_power_level call not found in BtAppStackInit'
# One tab: at function level, not inside a condition
assert call.group(1) == '\t', 'TX power call must not depend on a condition'

table = re.search(r'static const int16_t s_PaLevelTenthDbm\[\] = \{.*?\};', source, re.S).group(0)
start = source.index('static uint8_t BtAppWbaPaLevel(int8_t Dbm)')
end, depth = source.index('{', start) + 1, 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
code = '#include <cassert>\n#include <cstdint>\n' + table + '\n' + source[start:end] + r'''
int main()
{
	assert(BtAppWbaPaLevel(0) == 0x19);		// -0.3 dBm
	assert(BtAppWbaPaLevel(10) == 0x23);	// +10 dBm, maximum
	assert(BtAppWbaPaLevel(20) == 0x23);	// above the range: maximum
	assert(BtAppWbaPaLevel(-20) == 0x00);	// -20.4 dBm, first of the minimum levels
	assert(BtAppWbaPaLevel(-40) == 0x00);	// below the range: minimum
	assert(BtAppWbaPaLevel(4) == 0x1D);		// +3.9 dBm
	assert(BtAppWbaPaLevel(-1) == 0x18);	// -1.5 dBm
	return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='wba-tx-power-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    binary = Path(temp) / 'test'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O2', '-Wall',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
    print('PASS WBA TX power: always set, 0 dBm maps to PA level 0x19')
