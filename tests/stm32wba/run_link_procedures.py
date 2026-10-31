#!/usr/bin/env python3
"""Check WBA PHY/DLE sequencing with ST's shared pending-event ownership."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_gap_stm32wba.cpp').read_text()

def function(name):
    match = re.search(r'^bool ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'

code = r'''
#include <cassert>
#include <cstring>
#include "bluetooth/bt_app.h"
#define CFG_BLE_NUM_LINK 2
#define BLE_STATUS_SUCCESS 0
#define HCI_ALL_PHYS_TX_NO_PREF 1
#define HCI_ALL_PHYS_RX_NO_PREF 2
#define SysLogPrintf(...) ((void)0)
using tBleStatus = uint8_t;
static uint8_t phyStatus, dleStatus;
static unsigned phyCalls, dleCalls;
// ST v2.10.0: no-change PHY queues a pointer to event type 9. DLE's
// LL timeout initialization reuses the pointed-to record with type 1.
static uint8_t eventType[2];
static uint8_t *pendingEvent[2];
static uint8_t hci_le_set_phy(uint16_t hdl, uint8_t, uint8_t, uint8_t, uint16_t)
{
	phyCalls++;
	if (!phyStatus)
	{
		eventType[hdl] = 9;
		pendingEvent[hdl] = &eventType[hdl];
	}
	return phyStatus;
}
static uint8_t hci_le_set_data_length(uint16_t hdl, uint16_t octets, uint16_t time)
{
	assert(octets == 251 && time == 2120);
	dleCalls++;
	if (!dleStatus)
	{
		eventType[hdl] = 1;
	}
	return dleStatus;
}
'''
a = source.index('// ST v2.10.0 queues')
code += source[a:source.index('// aci_gap_init returns the service', a)]
code += function('BtGapSetPhy') + function('BtGapSetDataLength')
code += r'''
static void Complete(uint16_t hdl)
{
	assert(pendingEvent[hdl] && *pendingEvent[hdl] == 9);
	pendingEvent[hdl] = nullptr;
	BtGapWbaPhyUpdateComplete(hdl);
}
int main()
{
	// Reproduce the vendor-record overwrite without the port serialization.
	assert(hci_le_set_phy(0, 0, 1, 1, 0) == 0);
	assert(hci_le_set_data_length(0, 251, 2120) == 0);
	assert(*pendingEvent[0] == 1);
	pendingEvent[0] = nullptr;
	phyCalls = dleCalls = 0;

	assert(BtGapSetPhy(0, 1, 1, 0));
	assert(BtGapSetDataLength(0, 251, 2120));
	assert(dleCalls == 0 && *pendingEvent[0] == 9);
	assert(!BtGapSetPhy(0, 2, 2, 0));
	assert(phyCalls == 1);
	assert(BtGapSetPhy(1, 1, 1, 0));
	assert(BtGapSetDataLength(1, 251, 2120));
	Complete(0);
	assert(dleCalls == 1 && *pendingEvent[1] == 9);
	assert(!BtGapWbaLinkProcBusy(0) && BtGapWbaLinkProcBusy(1));
	Complete(1);
	assert(dleCalls == 2 && !BtGapWbaLinkProcBusy(1));

	// Refused PHY does not retain a reservation.
	phyStatus = 0x0c;
	assert(!BtGapSetPhy(0, 1, 1, 0));
	assert(!BtGapWbaLinkProcBusy(0));
	phyStatus = 0;
	assert(BtGapSetPhy(0, 1, 1, 0));
	assert(BtGapSetDataLength(0, 251, 2120));
	dleStatus = 0x0c;
	Complete(0);
	assert(BtGapWbaLinkProcBusy(0));
	dleStatus = 0;
	BtGapWbaLinkProcProcess();
	assert(!BtGapWbaLinkProcBusy(0));

	// Disconnect cancels deferred work; a stale completion cannot send DLE.
	assert(BtGapSetPhy(0, 1, 1, 0));
	assert(BtGapSetDataLength(0, 251, 2120));
	BtGapWbaLinkProcClear(0);
	dleCalls = 0;
	BtGapWbaPhyUpdateComplete(0);
	assert(dleCalls == 0);
	assert(BtGapSetPhy(0, 1, 1, 0));
	Complete(0);
	assert(!BtGapWbaLinkProcBusy(0));
}
'''
with tempfile.TemporaryDirectory(prefix='wba-link-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    for opt in ['-O0', '-O2']:
        binary = Path(temp) / 'test'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(ROOT / 'include'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        print('PASS WBA link:', opt, 'shared event ownership, per-link deferral, refusal, disconnect')
