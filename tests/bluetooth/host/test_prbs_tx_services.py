#!/usr/bin/env python3
"""Register the BleIntrfPrbsTx example's real services in its configured ATT pool."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


class PrbsBleServices(unittest.TestCase):
    def test_services_fit_application_pool(self):
        example = (ROOT / "exemples/bluetooth/bleintrf_prbs_tx.cpp").read_text()
        # Use the example's declarations so changes to payload sizes, services,
        # or the application pool are tested without a second configuration.
        declarations = example.split("#define PACKET_SIZE", 1)[1]
        declarations = '#define DEVICE_NAME "BlePrbs_SDC"\n#define PACKET_SIZE' + declarations.split(
            "#define BLEINTRF_FIFOSIZE", 1)[0]
        source = r'''#include <cstdio>
#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/blueio_blesrvc.h"
#include "bluetooth/services/bt_dis.h"
#define BLUEIO_CONNECT_LED_PORT 0
#define BLUEIO_CONNECT_LED_PIN 17
''' + declarations + r'''
void ReadCharSetNotif(BtGattChar_t *, bool, uint16_t) {}
extern "C" bool BtAppConnInit(void) { return true; }
bool BtPeerIsConnected(void) { return false; }
uint16_t BtPeerCount(void) { return 0; }
BtDevice_t *BtPeerSlot(uint16_t) { return nullptr; }
BtDevice_t *BtPeerFindByHdl(uint16_t) { return nullptr; }
int main()
{
	BtGapCfg_t gap = {};
	BtAttDBMark_t mark;

	gap.Role = s_BleAppCfg.Role;
	gap.SecType = s_BleAppCfg.SecType;
	if (!BtGapInit(&gap) || !BtGattSrvcAdd(&g_UartBleSrvc) ||
		!BtDisInit(&s_BleAppCfg))
	{
		std::puts("BleIntrfPrbsTx service registration failed");
		return 1;
	}
	BtAttDBMark(&mark);
	std::printf("BleIntrfPrbsTx: %u attributes, %zu bytes plus %zu-byte tail, pool %zu\n",
		mark.LastHdl, mark.MemUsed, sizeof(BtAttDBEntry_t), g_BtAttDBMemCfg.Size);
	if (mark.LastHdl != 38 ||
		mark.MemUsed + sizeof(BtAttDBEntry_t) > g_BtAttDBMemCfg.Size)
	{
		return 1;
	}
	return 0;
}
'''
        sources = ["bt_gap.cpp", "bt_gatt.cpp", "bt_att.cpp", "bt_uuid.cpp",
                   "services/bt_dis.cpp"]
        with tempfile.TemporaryDirectory() as tmp:
            cpp = Path(tmp) / "prbs_tx_services.cpp"
            exe = Path(tmp) / "prbs_tx_services"
            cpp.write_text(source)
            subprocess.run([
                os.environ.get("CXX", "g++"), "-std=gnu++20", "-O1", "-g",
                "-ffunction-sections", "-fdata-sections",
                "-fsanitize=undefined", "-fno-sanitize-recover=all",
                "-I" + str(ROOT / "include"), str(cpp), '-DNRFXLIB_SDC',
                str(ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/BleIntrfPrbsTx/src/ble_prbs_target.cpp"),
                *[str(ROOT / "src/bluetooth" / name) for name in sources],
                "-Wl,--gc-sections", "-o", str(exe)
            ], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
