#!/usr/bin/env python3
"""Check private port state and scan transitions without a vendor SDK.

Compile production state/operation definitions from all four app ports against
common bt_app.cpp. This checks the changed boundary, not a complete target build.
"""
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from extract_bt_app_init import extract_function

ROOT = Path(__file__).resolve().parents[3]
CXX = os.environ.get('CXX', 'g++')
LINK_GC = '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections'
PORTS = [
	'ARM/Nordic/nRF52/src/bt_app_nrf52.cpp',
	'ARM/Nordic/nRF54/src/bt_app_bm.cpp',
	'src/bluetooth/bt_app_hci.cpp',
	'ARM/ST/STM32WBAxx/src/bt_app_stm32wba.cpp',
]
SCANS = [
	'ARM/Nordic/nRF52/src/bt_scan_nrf52.cpp',
	'ARM/Nordic/nRF54/src/bt_scan_bm.cpp',
	'ARM/ST/STM32WBAxx/src/bt_scan_stm32wba.cpp',
	'src/bluetooth/bt_scan_hci.cpp',
]

def run(args):
	subprocess.run(args, check=True)


def build(directory, name, source, flags, extra=()):
	cpp = directory / (name + '.cpp')
	binary = directory / name
	cpp.write_text(source)
	run([CXX, '-std=gnu++20', '-Wall', '-Wextra', '-Werror',
		'-ffunction-sections', '-fdata-sections', '-I' + str(ROOT / 'include'),
		*flags, str(cpp), *map(str, extra), LINK_GC, '-o', str(binary)])
	run([str(binary)])
	nm_flags = ['-gU'] if sys.platform == 'darwin' else ['-g', '--defined-only']
	symbols = subprocess.check_output(['nm', *nm_flags, str(binary)], text=True)
	assert 's_BtAppData' not in symbols
	assert 'g_BtAppData' not in symbols


def check_ports(directory, flags):
	for path in PORTS:
		source = (ROOT / path).read_text()
		state = re.search(r'static struct \{[^}]+\} s_BtAppData = \{[^}]+\};', source)
		assert state is not None, path
		code = '''#include <cstdlib>
#include "bluetooth/bt_app.h"
#define CHECK(x) do { if (!(x)) std::abort(); } while (0)
static int s_Level = -1;
static int s_Disconnected;
static void IOPinSet(int, int) { s_Level = 1; }
static void IOPinClear(int, int) { s_Level = 0; }
'''
		code += state.group(0) + '\n'
		for name in ['BtInitialized', 'BtAppConnLedOff', 'BtAppConnLedOn']:
			code += extract_function(source, r'^(?:bool|void) ' + name + r'\(') + '\n'
		for name in ['BtAppSecConfigGet', 'BtAppSecIsStarted', 'BtAppSecStarted', 'BtAppSecTypeGet']:
			if re.search(r'^(?:bool|void|BTGAP_SECTYPE) ' + name + r'\(', source, re.M):
				code += extract_function(source, r'^(?:bool|void|BTGAP_SECTYPE) ' + name + r'\(') + '\n'
		code += '''
void BtPeerConnectionStateChanged(bool);
void BtPeerConnectionRejected(uint16_t);
extern "C" BtDevice_t *BtPeerGetActive(void) { return nullptr; }
extern "C" bool BtPeerIsConnected(void) { return false; }
extern "C" void BtAppDisconnectConn(uint16_t) { ++s_Disconnected; }
int main()
{
	CHECK(!BtInitialized());
	BtPeerConnectionStateChanged(true);
	CHECK(!BtInitialized());
	CHECK(s_Level == -1);
	s_BtAppData.ConnLedPort = 0;
	s_BtAppData.ConnLedPin = 1;
	for (int active = 0; active <= 1; ++active)
	{
		s_BtAppData.ConnLedActLevel = active;
		BtPeerConnectionStateChanged(true);
		CHECK(s_Level == active);
		BtPeerConnectionStateChanged(false);
		CHECK(s_Level == !active);
	}
	s_BtAppData.bInitialized = true;
	BtPeerConnectionStateChanged(false);
	CHECK(BtInitialized());
	BtPeerConnectionRejected(1);
	CHECK(s_Disconnected == 1);
	BtPeerConnectionRejected(BT_CONN_HDL_INVALID);
	CHECK(s_Disconnected == 1);
'''
		if 'BtAppSecStarted(void)' in source:
			code += '''
	CHECK(!BtAppSecIsStarted());
	s_BtAppData.SecType = BTGAP_SECTYPE_LESC_MITM;
	s_BtAppData.SecExchg = BTAPP_SECEXCHG_DISPLAY;
	uint8_t exchange = 0;
	CHECK(BtAppSecConfigGet(&exchange) == BTGAP_SECTYPE_LESC_MITM);
	CHECK(exchange == BTAPP_SECEXCHG_DISPLAY);
	BtAppSecStarted();
	CHECK(BtAppSecIsStarted());
'''
		code += '\n}\n'
		build(directory, Path(path).stem, code, flags, [ROOT / 'src/bluetooth/bt_app.cpp'])
		print('PASS private state/common peer operations:', path, flush=True)


def check_scans(directory, flags):
	for path in SCANS:
		source = (ROOT / path).read_text()
		code = '''#include <cstdlib>
#include "bluetooth/bt_app.h"
#define CHECK(x) do { if (!(x)) std::abort(); } while (0)
static bool s_bScanning;
static bool s_StartResult;
static int s_Starts;
static int s_Next;
static uint8_t s_BleScanBuff[255];
static struct { uint8_t *p_data; unsigned len; } g_BleScanReportData = {s_BleScanBuff, 255};
extern "C" bool BtGapScanInit(BtGapScanCfg_t *) { return true; }
extern "C" bool BtGapScanStart(uint8_t *, uint16_t) { ++s_Starts; return s_StartResult; }
extern "C" bool BtGapScanNext(uint8_t *, uint16_t) { ++s_Next; return true; }
extern "C" void BtGapScanStop(void) {}
extern "C" void BtHciScanReportEnable(void) {}
'''
		for name in ['BtScanIsActive', 'BtScanEnded', 'BtAppScanInit', 'BtAppScan', 'BtAppScanStop']:
			code += extract_function(source, r'^(?:bool|void) ' + name + r'\(') + '\n'
		code += '''
int main()
{
	(void)g_BleScanReportData;
	BtGapScanCfg_t cfg = {};
	CHECK(!BtScanIsActive());
	BtAppScanInit(&cfg);
	CHECK(!BtScanIsActive());
	int before = s_Starts;
	BtAppScan();
	CHECK(s_Starts == before + 1);
	CHECK(!BtScanIsActive());
	s_StartResult = true;
	BtAppScan();
	CHECK(BtScanIsActive());
	BtScanEnded();
	CHECK(!BtScanIsActive());
	before = s_Starts;
	BtAppScan();
	CHECK(s_Starts == before + 1);
	CHECK(BtScanIsActive());
	BtAppScanStop();
	CHECK(!BtScanIsActive());
}
'''
		build(directory, Path(path).stem, code, flags)
		print('PASS failed scan/retry/controller stop:', path, flush=True)


def check_data_length(directory, flags):
	hci = (ROOT / 'src/bluetooth/bt_app_hci.cpp').read_text()
	nrf = (ROOT / 'ARM/Nordic/nRF52/src/bt_app_nrf52.cpp').read_text()
	hci_calc = hci[hci.index('uint32_t mtu = max('):hci.index('uint8_t datalen[4];')]
	nrf_calc = re.search(r'uint8_t dlen = [^;]+;', nrf).group(0)
	code = r"""
#include <cstdint>
#include <cstdlib>
#include "istddef.h"
struct Config { uint16_t MaxMtu; };
static uint16_t hciLength(uint16_t requested, uint16_t maxTxOctets) {
	Config cfg = {requested};
	Config *pCfg = &cfg;
""" + hci_calc + r"""
	return txOctets;
}
static uint8_t sdLength(uint16_t requested) {
	Config s_BtAppData = {requested};
""" + nrf_calc + r"""
	return dlen;
}
int main() {
	const uint16_t mtus[] = {23, 27, 247, 248, 512, 65535};
	const uint16_t octets[] = {27, 31, 251, 251, 251, 251};
	for (unsigned i = 0; i < sizeof(mtus) / sizeof(mtus[0]); i++) {
		if (hciLength(mtus[i], 251) != octets[i]) std::abort();
		if (sdLength(mtus[i]) != octets[i]) std::abort();
		if (hciLength(mtus[i], 27) != 27) std::abort();
	}
	if (hciLength(0, 251) != 27) std::abort();
}
"""
	build(directory, 'data_length', code, flags)
	# Arm GCC defines uint32_t as unsigned long. Linux uses unsigned int and
	# excludes the target min/max overloads, hiding mixed-operand ambiguity.
	overloads = r"""
static unsigned min(unsigned a, unsigned b) { return a < b ? a : b; }
static unsigned long min(unsigned long a, unsigned long b) { return a < b ? a : b; }
static unsigned max(unsigned a, unsigned b) { return a > b ? a : b; }
static unsigned long max(unsigned long a, unsigned long b) { return a > b ? a : b; }
"""
	arm_types = code.replace('uint32_t', 'unsigned long')
	arm_types = arm_types.replace('struct Config', overloads + '\nstruct Config', 1)
	build(directory, 'data_length_arm_types', arm_types, flags + ['-Wno-unused-function'])
	print('PASS HCI and S132 data length bounds', flush=True)


def check_softdevice_streaming(directory, flags):
	source = (ROOT / 'ARM/Nordic/nRF52/src/bt_app_nrf52.cpp').read_text()
	code = r"""
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "istddef.h"
#include "bluetooth/bt_gap.h"
#define CHECK(x) do { if (!(x)) std::abort(); } while (0)
#define APP_ERROR_CHECK(x) CHECK((x) == 0)
#define BLE_CONN_HANDLE_INVALID 0xffff
#define BLE_COMMON_OPT_CONN_EVT_EXT 1
#define BLE_GATTC_OPT_UUID_DISC 2
using ret_code_t = int;
struct Options {
 struct { struct { unsigned enable; } conn_evt_ext; } common_opt;
 struct { struct { unsigned auto_add_vs_enable; } uuid_disc; } gattc_opt;
};
using ble_opt_t = Options;
static int s_Gatt;
static struct { unsigned Role; uint16_t MaxMtu; } s_BtAppData;
static unsigned central, peripheral, dataCalls, extendCalls, uuidCalls;
static uint8_t dataLength;
static void BtGattEvtHandler() {}
static int nrf_ble_gatt_init(int *, void (*)()) { return 0; }
static int nrf_ble_gatt_att_mtu_periph_set(int *, uint16_t mtu) {
 CHECK(mtu == s_BtAppData.MaxMtu); ++peripheral; return 0;
}
static int nrf_ble_gatt_att_mtu_central_set(int *, uint16_t mtu) {
 CHECK(mtu == s_BtAppData.MaxMtu); ++central; return 0;
}
static int nrf_ble_gatt_data_length_set(int *, uint16_t hdl, uint8_t len) {
 CHECK(hdl == BLE_CONN_HANDLE_INVALID); ++dataCalls; dataLength = len; return 0;
}
static int sd_ble_opt_set(unsigned id, const ble_opt_t *opt) {
 if (id == BLE_COMMON_OPT_CONN_EVT_EXT) {
  CHECK(opt->common_opt.conn_evt_ext.enable == 1); ++extendCalls;
 } else {
  CHECK(id == BLE_GATTC_OPT_UUID_DISC);
  CHECK(opt->gattc_opt.uuid_disc.auto_add_vs_enable == 1); ++uuidCalls;
 }
 return 0;
}
""" + extract_function(source, r'^void BtGattInit\(') + r"""
int main() {
 const unsigned roles[] = {BT_GAP_ROLE_CENTRAL, BT_GAP_ROLE_PERIPHERAL,
  BT_GAP_ROLE_CENTRAL | BT_GAP_ROLE_PERIPHERAL, BT_GAP_ROLE_OBSERVER};
 const uint16_t mtus[] = {23, 247, 512};
 for (auto role : roles) {
  for (auto mtu : mtus) {
   central = peripheral = dataCalls = extendCalls = uuidCalls = 0;
   s_BtAppData.Role = role; s_BtAppData.MaxMtu = mtu;
   BtGattInit();
   CHECK(central == !!(role & BT_GAP_ROLE_CENTRAL));
   CHECK(uuidCalls == central);
   CHECK(peripheral == !!(role & BT_GAP_ROLE_PERIPHERAL));
   CHECK(dataCalls == !!(central || peripheral));
   CHECK(extendCalls == dataCalls);
   if (dataCalls) CHECK(dataLength == (mtu == 23 ? 27 : 251));
  }
 }
}
"""
	build(directory, 'softdevice_streaming', code, flags)
	print('PASS SoftDevice central/peripheral/mixed streaming settings', flush=True)


# Check complete source files too: extracted functions do not cover every init
# and event path. Value-returning accessors cannot replace state assignments.
VALUE_ACCESSORS = (
	'BtScanIsActive', 'BtInitialized', 'BtAdvHandle', 'BtAppHciDevice',
	'BtAppSecConfigGet', 'BtAppSecIsStarted', 'BtAppSecTypeGet',
)
accessor_write = re.compile(
	r'\b(?:' + '|'.join(VALUE_ACCESSORS) + r')\s*\([^;{}]*?\)\s*'
	r'(?:=(?!=)|[+*/%&|^-]=|<<=|>>=|\+\+|--)')
for directory in ('ARM', 'src'):
	for path in (ROOT / directory).rglob('*.cpp'):
		assert not accessor_write.search(path.read_text()), path

# Application headers must not expose either the old storage or its replacement.
for header in (ROOT / 'include').rglob('*.h'):
	assert not re.search(r'\b(?:BtAppData_t|g_BtAppData|s_BtAppData)\b', header.read_text()), header

with tempfile.TemporaryDirectory(prefix='bt-private-') as temporary:
	directory = Path(temporary)
	for mode, flags in [('Debug', ['-O0', '-g', '-DDEBUG']), ('Release', ['-O2', '-DNDEBUG'])]:
		print(mode, flush=True)
		check_ports(directory, flags)
		check_scans(directory, flags)
		check_data_length(directory, flags)
		check_softdevice_streaming(directory, flags)
