#!/usr/bin/env python3
"""Validate WBA event routing against the actual ST middleware wire structures.

Usage: python3 tests/stm32wba/run_event_router.py /path/to/stm32-mw-wpan
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
WPAN = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / 'external/stm32-mw-wpan'
assert (WPAN / 'ble/stack/include/ble_core.h').is_file(), 'Supply the STM32 WPAN middleware directory'
code = r'''
#include <cassert>
#include <cstring>
#include <cstddef>
#define __STATIC_INLINE static inline
#include "bluetooth/bt_app.h"
#define __PACKED_STRUCT struct __attribute__((packed))
#define __PACKED_UNION union __attribute__((packed))
#include "ble_types.h"
#include "ble_core.h"
#include "bt_wba_event_router.h"
static unsigned delivered;
void BtGapWbaConnParamsDisconnected(uint16_t) {}
static unsigned phyCompleted;
void BtGapWbaPhyUpdateComplete(uint16_t hdl) { assert(hdl == 0x123); phyCompleted++; }
static unsigned connected;
void BtGapWbaConnParamsConnected(uint16_t handle, uint8_t role,
		uint16_t interval, uint16_t latency, uint16_t timeout)
{
	assert(handle == 0x123);
	assert(role == 0);
	assert(interval == 12);
	assert(latency == 0);
	assert(timeout == 500);
	connected++;
}
void BtGapWbaConnParamsUpdateComplete(uint16_t, uint8_t, uint16_t, uint16_t, uint16_t) {}
void BtGapWbaConnParamsL2capResponse(uint16_t, uint16_t) {}
void BtGattWbaNotificationComplete(uint16_t, uint16_t) {}
void BtGattHandleValueConfirm(uint16_t) {}
static BtWbaEvtFlow_t Receive(void *)
{
	delivered++;
	return BTWBA_EVT_FLOW_ENABLE;
}
static void ValueEvent(uint16_t code)
{
	uint8_t packet[258] = {};
	unsigned before = delivered;
	unsigned len;

	packet[0] = 4;
	packet[1] = HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE;
	packet[3] = (uint8_t)code;
	packet[4] = (uint8_t)(code >> 8);
	// Both vendor value events: handle(2), attribute(2), length(1), value.
	packet[9] = 244;
	packet[2] = 251;
	BtWbaEventRouterRun(packet);
	assert(delivered == before + 1);
	before = delivered;
	for (len = 0; len < 251; len++)
	{
		packet[2] = (uint8_t)len;
		BtWbaEventRouterRun(packet);
		assert(delivered == before);
	}
	packet[9] = 0;
	packet[2] = 7;
	BtWbaEventRouterRun(packet);
	assert(delivered == before + 1);
}
static void EnhancedConnectionEvent()
{
	uint8_t packet[4 + sizeof(hci_le_enhanced_connection_complete_event_rp0)] = {};
	hci_le_enhanced_connection_complete_event_rp0 event = {};
	unsigned before;
	unsigned len;

	packet[0] = 4;
	packet[1] = HCI_LE_META_EVT_CODE;
	packet[2] = sizeof(event) + 1;
	packet[3] = HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE;
	event.Connection_Handle = 0x123;
	event.Conn_Interval = 12;
	event.Supervision_Timeout = 500;
	memcpy(packet + 4, &event, sizeof(event));
	before = delivered;
	BtWbaEventRouterRun(packet);
	assert(connected == 1);
	assert(delivered == before + 1);
	// Failed connection attempts still reach the application, not GAP state.
	packet[4] = 0x3e;
	BtWbaEventRouterRun(packet);
	assert(connected == 1);
	assert(delivered == before + 2);
	packet[4] = 0;
	before = delivered;
	for (len = 0; len < sizeof(event) + 1; len++)
	{
		packet[2] = (uint8_t)len;
		BtWbaEventRouterRun(packet);
		assert(connected == 1);
		assert(delivered == before);
	}
}
static void PhyCompleteEvent()
{
	uint8_t packet[4 + sizeof(hci_le_phy_update_complete_event_rp0)] = {};
	hci_le_phy_update_complete_event_rp0 event = {};
	unsigned len;

	packet[0] = 4;
	packet[1] = HCI_LE_META_EVT_CODE;
	packet[3] = HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE;
	event.Connection_Handle = 0x123;
	memcpy(packet + 4, &event, sizeof(event));
	for (len = 0; len < sizeof(event) + 1; len++)
	{
		packet[2] = (uint8_t)len;
		BtWbaEventRouterRun(packet);
		assert(phyCompleted == 0);
	}
	packet[2] = sizeof(event) + 1;
	BtWbaEventRouterRun(packet);
	assert(phyCompleted == 1);
	packet[4] = 0x0c;
	BtWbaEventRouterRun(packet);
	assert(phyCompleted == 2);
}
static void ModifiedEvent()
{
	uint8_t packet[258] = {};
	aci_gatt_attribute_modified_event_rp0 event = {};
	unsigned before = delivered;
	unsigned fixed = offsetof(aci_gatt_attribute_modified_event_rp0, Attr_Data);
	unsigned len;

	packet[0] = 4;
	packet[1] = HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE;
	packet[3] = (uint8_t)ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE;
	packet[4] = (uint8_t)(ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE >> 8);
	event.Attr_Data_Length = 2;
	memcpy(packet + 5, &event, fixed);
	for (len = 0; len < 2 + fixed + 2; len++)
	{
		packet[2] = len;
		BtWbaEventRouterRun(packet);
		assert(delivered == before);
	}
	packet[2] = 2 + fixed + 2;
	BtWbaEventRouterRun(packet);
	assert(delivered == before + 1);
}
static void FixedEvent(uint8_t event, uint16_t vendor, unsigned size)
{
	uint8_t packet[258] = {};
	unsigned before = delivered;
	unsigned prefix = event == HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE ? 2 : 0;
	packet[0] = 4;
	packet[1] = event;
	packet[3] = (uint8_t)vendor;
	packet[4] = (uint8_t)(vendor >> 8);
	for (unsigned len = 0; len < prefix + size; len++)
	{
		packet[2] = len;
		BtWbaEventRouterRun(packet);
		assert(delivered == before);
	}
	packet[2] = prefix + size;
	BtWbaEventRouterRun(packet);
	assert(delivered == before + 1);
}
int main()
{
	// Fixed prefixes must be checked before reading their declared lengths.
	uint8_t tiny[1] = {};
	assert(!BtWbaDiscServiceEventValid(
		(aci_att_read_by_group_type_resp_event_rp0 *)tiny, sizeof(tiny)));
	assert(!BtWbaDiscCharEventValid(
		(aci_att_read_by_type_resp_event_rp0 *)tiny, sizeof(tiny)));
	assert(!BtWbaDiscInfoEventValid(
		(aci_att_find_info_resp_event_rp0 *)tiny, sizeof(tiny)));
	BtWbaEventRouterRegister(Receive);
	FixedEvent(HCI_DISCONNECTION_COMPLETE_EVT_CODE, 0, sizeof(hci_disconnection_complete_event_rp0));
	FixedEvent(HCI_ENCRYPTION_CHANGE_EVT_CODE, 0, sizeof(hci_encryption_change_event_rp0));
	FixedEvent(HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE, ACI_GATT_PROC_COMPLETE_VSEVT_CODE, sizeof(aci_gatt_proc_complete_event_rp0));
	FixedEvent(HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE, ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE, sizeof(aci_att_exchange_mtu_resp_event_rp0));
	FixedEvent(HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE, ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE, sizeof(aci_gap_pairing_complete_event_rp0));
	FixedEvent(HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE, ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE, sizeof(aci_gap_numeric_comparison_value_event_rp0));
	FixedEvent(HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE, ACI_GAP_PASS_KEY_REQ_VSEVT_CODE, sizeof(aci_gap_pass_key_req_event_rp0));
	ModifiedEvent();
	EnhancedConnectionEvent();
	PhyCompleteEvent();
	ValueEvent(ACI_GATT_NOTIFICATION_VSEVT_CODE);
	ValueEvent(ACI_GATT_INDICATION_VSEVT_CODE);
}
'''
with tempfile.TemporaryDirectory(prefix='wba-router-') as temp:
    cpp = Path(temp) / 'test.cpp'
    cpp.write_text(code)
    for opt in ['-O0', '-O2']:
        binary = Path(temp) / 'test'
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-ffunction-sections', '-Wl,--gc-sections',
                        '-I' + str(ROOT / 'include'),
                        '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/include'),
                        '-I' + str(WPAN / 'ble/stack/include'),
                        '-I' + str(WPAN / 'ble/stack/include/auto'),
                        str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        print('PASS WBA router:', opt, 'ST wire layouts, full and truncated value events')
