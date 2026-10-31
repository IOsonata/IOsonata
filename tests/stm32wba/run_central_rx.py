#!/usr/bin/env python3
"""Exercise WBA discovery admission and owned event payloads without radio hardware."""
from pathlib import Path
import os
import subprocess
import re
import tempfile

ROOT = Path(__file__).resolve().parents[2]
def extract_function(text, name):
    match = re.search(r'^(?:extern "C" )?(?:static )?(?:void|bool|uint8_t) ' +
                      name + r'\([^;]*?\)\s*\{', text, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end]


source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp').read_text()
gap = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_gap_stm32wba.cpp').read_text()
code = r'''
#include <cassert>
#include <cstring>
#include "bluetooth/bt_app.h"
#define BLE_STATUS_SUCCESS 0
#define BLE_STATUS_FAILED 1
#define GATT_MTU_SIZE_DEFAULT 23
#define BT_APP_WBA_EVT_SLOTS 8
#define BT_APP_WBA_EVT_MAXLEN 258
#define HCI_EVENT_PKT_TYPE 4
static struct { uint16_t MaxMtu; bool bSecure; } s_BtAppData = {247, false};
static BtDevice_t peers[2];
static BtGattDBSrvc_t services[2][BT_DEV_SERVICE_MAXCNT];
static unsigned mtuCalls, serviceCalls, charCalls, descCalls, discovered;
static unsigned refusals;
static uint8_t commandStatus;
static uint8_t Admit()
{
	if (refusals)
	{
		refusals--;
		return 1;
	}
	return 0;
}
BtDevice_t *BtPeerFindByHdl(uint16_t hdl)
{
	return hdl < 2 ? &peers[hdl] : nullptr;
}
uint16_t BtPeerCount() { return 2; }
BtDevice_t *BtPeerSlot(uint16_t idx) { return &peers[idx]; }
bool BtDeviceSrvcCacheAttach(BtDevice_t *peer)
{
	peer->pServices = services[peer->Conn.Hdl];
	return true;
}
void BtDeviceDiscovered(BtDevice_t *) { discovered++; }
static uint8_t aci_gatt_exchange_config(uint16_t)
{
	mtuCalls++;
	return Admit();
}
static uint8_t aci_gatt_disc_all_primary_services(uint16_t)
{
	serviceCalls++;
	return Admit();
}
static uint8_t aci_gatt_disc_all_char_of_service(uint16_t, uint16_t, uint16_t)
{
	charCalls++;
	return Admit();
}
static uint8_t aci_gatt_disc_all_char_desc(uint16_t, uint16_t, uint16_t)
{
	descCalls++;
	return Admit();
}
static uint8_t hci_le_set_data_length(uint16_t hdl, uint16_t octets, uint16_t time)
{
	assert(hdl == 1 && octets == 251 && time == 2120);
	return commandStatus;
}
struct aci_gatt_proc_complete_event_rp0 { uint16_t Connection_Handle; uint8_t Error_Code; };
static void BtAppDiscStartDesc(BtDevice_t *);
'''
a = source.index('enum {\n\tDISC_IDLE')
code += source[a:source.index('};', a) + 3]
for name in ['BtAppDiscStartChar', 'BtAppDiscStartDesc', 'WbaDiscProcComplete',
             'BtAppDiscoverDevice', 'BtAppDiscoveryRetry']:
    code += extract_function(source, name) + '\n'
code += extract_function(gap, 'BtGapSetDataLength') + '\n'
code += r"""
#define NDEBUG
#define STATUS_PRINTF(...) ((void)0)
static struct { bool bOobEnabled; } s_WbaData;
BtDevice_t *BtPeerConnected(uint16_t hdl, uint8_t role, uint8_t,
	const uint8_t *, uint8_t, const uint8_t *)
{
	memset(&peers[hdl], 0, sizeof(peers[hdl]));
	peers[hdl].Conn.Hdl = hdl;
	peers[hdl].Conn.Role = role;
	return &peers[hdl];
}
static void BtAdvWbaConnected() {}
static void BtAppConnLedOn() {}
static void BtAppOobPeerDataPush(uint16_t, uint8_t, uint8_t *) {}
static uint8_t aci_gap_send_pairing_req(uint16_t, uint8_t) { return 0; }
static uint8_t aci_gap_peripheral_security_req(uint16_t) { return 0; }
#define SysLogPrintf(...) ((void)0)
static bool connectDiscover;
void BtAppEvtConnected(uint16_t hdl)
{
	assert(peers[hdl].Conn.MaxMtu == GATT_MTU_SIZE_DEFAULT);
	if (connectDiscover)
	{
		assert(BtAppDiscoverDevice(&peers[hdl]));
	}
}
"""
code += extract_function(source, 'BtAppWbaConnected') + '\n'
code += 'void BtAppEvtConnectFailed() {}\nvoid BtAdvStart() {}\n'

a = source.index('typedef struct __Bt_App_Wba_Evt')
code += source[a:source.index('static BtWbaEvtFlow_t BtAppHciEvtHandler', a)]
code += r'''
static unsigned irqState, routed, queued, hostWake;
static bool s_bEvtDrainQueued;
static uint32_t __get_PRIMASK() { return irqState; }
static void __disable_irq() { irqState = 1; }
static void __set_PRIMASK(uint32_t state) { irqState = state; }
static void BtAppEvtDrainQue() { queued++; }
static void BleStackCB_Process() { hostWake++; }
static void BtWbaEventRouterRun(void *data)
{
	uint8_t *p = (uint8_t *)data;
	unsigned idx;

	assert(p[0] == 4 && p[1] == 255 && p[2] == 251);
	for (idx = 3; idx < 254; idx++)
	{
		assert(p[idx] == uint8_t(idx));
	}
	routed++;
}
'''
code += extract_function(source, 'BLECB_Indication') + '\n'
code += extract_function(source, 'BtAppEvtDrain') + '\n'
code += r'''
int main()
{
	aci_gatt_proc_complete_event_rp0 done = {0, 0};
	uint8_t packet[254];
	unsigned idx;

	// Exercise the real connection handler, including discovery from its callback.
	connectDiscover = true;
	BtAppWbaConnected(0, 0, BT_CONN_ROLE_CENTRAL, 0, nullptr);
	assert(mtuCalls == 1 && serviceCalls == 0);
	assert(peers[0].Discovery.Phase == DISC_MTU);
	connectDiscover = false;
	BtAppWbaConnected(0, 0, BT_CONN_ROLE_CENTRAL, 0, nullptr);
	BtAppWbaConnected(0, 1, BT_CONN_ROLE_CENTRAL, 0, nullptr);
	mtuCalls = 0;
	refusals = 1;
	assert(!BtAppDiscoverDevice(&peers[0]));
	assert(peers[0].Discovery.Phase == DISC_IDLE);
	assert(BtAppDiscoverDevice(&peers[0]));
	assert(BtAppDiscoverDevice(&peers[0]));
	assert(mtuCalls == 2 && serviceCalls == 0);
	assert(BtAppDiscoverDevice(&peers[1]));
	peers[0].Conn.MaxMtu = 247;
	refusals = 1;
	WbaDiscProcComplete(&done);
	assert(peers[0].Discovery.Phase == DISC_SERVICES_PENDING);
	assert(peers[1].Discovery.Phase == DISC_MTU);
	BtAppDiscoveryRetry();
	assert(serviceCalls == 2 && peers[0].Discovery.Phase == DISC_SERVICES);
	peers[0].NbSrvc = 1;
	services[0][0].handle_range = {1, 10};
	refusals = 1;
	WbaDiscProcComplete(&done);
	assert(peers[0].Discovery.Phase == DISC_CHARS_PENDING);
	assert(peers[0].Discovery.SrvIdx == 0 && !discovered);
	BtAppDiscoveryRetry();
	assert(charCalls == 2 && peers[0].Discovery.Phase == DISC_CHARS);
	services[0][0].char_count = 1;
	services[0][0].characteristics[0].characteristic.handle_value = 3;
	refusals = 1;
	WbaDiscProcComplete(&done);
	assert(peers[0].Discovery.Phase == DISC_DESCS_PENDING);
	assert(peers[0].Discovery.CharIdx == 0 && !discovered);
	BtAppDiscoveryRetry();
	assert(descCalls == 2 && peers[0].Discovery.Phase == DISC_DESCS);
	WbaDiscProcComplete(&done);
	assert(discovered == 1);
	assert(BtAppDiscoverDevice(&peers[0]));
	assert(mtuCalls == 3); // rediscovery does not exchange again
	assert(BtGapSetDataLength(1, 251, 2120));
	commandStatus = 1;
	assert(!BtGapSetDataLength(1, 251, 2120));
	assert(!BtGapSetDataLength(1, 252, 2120));
	assert(!BtGapSetDataLength(1, 251, 0));

	for (idx = 0; idx < sizeof(packet); idx++)
	{
		packet[idx] = uint8_t(idx);
	}
	packet[0] = 4;
	packet[1] = 255;
	packet[2] = 251;
	assert(BLECB_Indication(nullptr, 0, nullptr, 0) == BLE_STATUS_FAILED);
	assert(BLECB_Indication(packet, 2, nullptr, 0) == BLE_STATUS_FAILED);
	assert(BLECB_Indication(packet, 10, nullptr, 244) == BLE_STATUS_FAILED);
	assert(BLECB_Indication(packet, 10, nullptr, 0) == BLE_STATUS_FAILED);
	irqState = 1;
	for (idx = 0; idx < 8; idx++)
	{
		assert(BLECB_Indication(packet, 10, packet + 10, 244) == 0);
	}
	assert(irqState == 1 && queued == 8);
	assert(BLECB_Indication(packet, sizeof(packet), nullptr, 0) == BLE_STATUS_FAILED);
	memset(packet + 3, 0, sizeof(packet) - 3); // queued payload has its own storage
	BtAppEvtDrain(0, nullptr);
	assert(routed == 8 && hostWake == 1 && irqState == 1 && s_EvtCnt == 0);
}
'''
with tempfile.TemporaryDirectory(prefix='wba-central-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    for opt in ['-O0', '-O2']:
        binary = Path(temp) / 'test'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(ROOT / 'include'), str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        print('PASS WBA central:', opt, 'MTU sequencing, per-peer retries, DLE, event ownership/backpressure')
