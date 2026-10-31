#!/usr/bin/env python3
"""Exercise WBA advertising reconfiguration and scan address selection."""
from pathlib import Path
import re
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
adv = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_adv_stm32wba.cpp').read_text()
gap = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_gap_stm32wba.cpp').read_text()
scan = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_scan_stm32wba.cpp').read_text()
def extract(source, name):
    match = re.search(r'^(?:__attribute__\(\(weak\)\) )?(?:static )?(?:bool|void|uint16_t) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]
code = r'''
#include <cassert>
#include <cstring>
#include "bluetooth/bt_app.h"
#define DEBUG_PRINTF(...) ((void)0)
#define BLE_STATUS_SUCCESS 0
#define BT_WBA_MSEC_TO_0_625(ms) (((uint64_t)(ms) * 8U) / 5U)
#define BT_WBA_MSEC_TO_10MS(ms) ((ms) / 10U + ((ms) % 10U != 0U))
#define BT_WBA_ADV_HANDLE 0
#define BT_WBA_ADV_CHAN_MAP_ALL 7
#define BT_WBA_PHY_2M 2
#define BT_WBA_ADV_DATA_OP_COMPLETE 3
#define BT_WBA_ADV_DATA_FRAG_PREF_NONE 1
static bool s_bAdvertising, s_bExtAdv;
static uint8_t s_AdvRole;
static uint16_t s_AdvVendorId, s_AdvDuration, s_PeripheralLinkCount;
static BtAdvPacket_t s_BtAppExtAdvPkt = {}, s_BtAppExtSrPkt = {}, s_BtAppAdvPkt = {}, s_BtAppSrPkt = {};
struct Adv_Set_t { uint8_t Advertising_Handle; uint16_t Duration; uint8_t Max_Extended_Advertising_Events; };
static bool refuseStop;
static unsigned stops, configs;
static uint32_t interval;
static uint16_t MapAdvEvtProps(uint8_t) { return 0; }
static uint8_t aci_gap_adv_set_enable(uint8_t enable, uint8_t, Adv_Set_t *) { assert(!enable); stops++; return refuseStop; }
static uint8_t aci_gap_adv_set_configuration(uint8_t, uint8_t, uint16_t, uint32_t lo, uint32_t hi, uint8_t, uint8_t, uint8_t, uint8_t *, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t) { assert(lo == hi); interval = lo; configs++; return 0; }
static uint8_t aci_gap_adv_set_adv_data(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t *) { return 0; }
static uint8_t aci_gap_adv_set_scan_resp_data(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t *) { return 0; }
bool BtAdvEncode(const BtAppCfg_t *, BtAdvPacket_t *, BtAdvPacket_t *, bool *ext, bool *scannable) { *ext = false; *scannable = true; return true; }
using tBleStatus = uint8_t;
#define BT_GAP_HCI_SCAN_INTERVAL_MIN 4
#define BT_GAP_HCI_SCAN_INTERVAL_MAX 0x4000
#define BT_GAP_HCI_SCAN_ACTIVE 1
#define BT_GAP_HCI_SCAN_PASSIVE 0
#define GAP_OBSERVATION_PROC 0x80
#define HCI_SCANNING_PHYS_LE_1M 1
#define CONFIG_DATA_RANDOM_ADDRESS_OFFSET 0x2e
#define CONFIG_DATA_RANDOM_ADDRESS_LEN 6
static struct { uint16_t ScanInterval, ScanWindow; uint8_t ScanType, ScanFilterDup; } s_GapWba;
static uint8_t s_OwnAddrType, addressType;
static bool s_bRandomAddrSet;
static uint8_t randomAddr[6];
static bool s_bScanning;
static unsigned scanStops;
static uint8_t scanStopStatus;
static uint8_t configStatus;
static uint8_t hci_read_bd_addr(uint8_t *p) { memset(p, 0x12, 6); return 0; }
static uint8_t aci_hal_write_config_data(uint8_t off, uint8_t len, const uint8_t *p) { assert(off == 0x2e && len == 6); memcpy(randomAddr, p, 6); return configStatus; }
struct Scan_Param_Phy_t { uint8_t Scan_Type; uint16_t Scan_Interval, Scan_Window; };
static uint8_t aci_gap_start_scan(uint8_t, uint8_t, uint8_t own, uint8_t, uint16_t, uint16_t, uint8_t, uint8_t, Scan_Param_Phy_t *) { addressType = own; return 0; }
static uint8_t aci_gap_terminate_gap_proc(uint8_t proc) { assert(proc == GAP_OBSERVATION_PROC); scanStops++; return scanStopStatus; }
void BtScanEnded() { s_bScanning = false; }
'''
for name in ['BtAdvStop', 'PushAdvDataToStack', 'BtAppAdvInit']:
    code += extract(adv, name) + '\n'
for name in ['BtGapWbaScanParamsConvert', 'BtGapScanInit', 'BtGapScanStart', 'BtGapScanStop']:
    code += extract(gap, name) + '\n'
code += extract(scan, 'BtAppScanStop')
code += r'''
int main() {
 BtAppCfg_t cfg = {};
 cfg.AdvInterval = 1;
 assert(BtAppAdvInit(&cfg) && interval == 0x20);
 cfg.AdvInterval = 100;
 s_bAdvertising = true;
 assert(BtAppAdvInit(&cfg) && interval == 160 && stops == 1);
 s_bAdvertising = true;
 refuseStop = true;
 unsigned old = configs;
 assert(!BtAppAdvInit(&cfg) && s_bAdvertising && configs == old);
 refuseStop = false;
 cfg.AdvInterval = UINT32_MAX;
 assert(BtAppAdvInit(&cfg) && interval == 0xffffff);
 BtGapScanCfg_t sc = {};
 sc.Param.Interval = 100;
 sc.Param.Duration = 50;
 sc.Param.OwnAddrType = BTADDR_TYPE_RAND;
 configStatus = 1;
 assert(!BtGapScanInit(&sc) && !s_bRandomAddrSet);
 configStatus = 0;
 assert(BtGapScanInit(&sc) && (randomAddr[5] & 0xc0) == 0xc0);
 assert(BtGapScanStart(nullptr, 0) && addressType == 1);
 sc.Param.OwnAddrType = BTADDR_TYPE_PUBLIC;
 assert(BtGapScanInit(&sc));
 assert(BtGapScanStart(nullptr, 0) && addressType == 0);
 sc.Param.OwnAddrType = BTADDR_TYPE_RAND;
 configStatus = 1;
 assert(BtGapScanInit(&sc) && s_OwnAddrType == 1); // already configured: no rewrite
 sc.Param.OwnAddrType = BTADDR_TYPE_RESOLV;
 assert(!BtGapScanInit(&sc));
 s_bScanning = true;
 scanStopStatus = 1;
 BtAppScanStop();
 assert(s_bScanning);
 scanStopStatus = 0;
 BtGapScanStop();
 BtAppScanStop();
 assert(scanStops == 2 && !s_bScanning);
}
'''
with tempfile.TemporaryDirectory() as temp:
    p = Path(temp)
    (p / 'test.cpp').write_text(code)
    for opt in ['-O0', '-O2']:
        subprocess.run(['g++', '-std=c++17', opt, '-fsanitize=undefined', '-fno-sanitize-recover=all', '-I' + str(ROOT / 'include'), str(p / 'test.cpp'), '-o', str(p / 'test')], check=True)
        subprocess.run([str(p / 'test')], check=True)
        print('PASS WBA GAP:', opt, 'interval bounds, active reconfiguration, address selection, scan stop')
