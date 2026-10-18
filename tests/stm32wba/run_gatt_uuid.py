#!/usr/bin/env python3
"""Host-check WBA service UUID registration and legacy advertising encoding.

Compiles the production registration functions with mocked ST GATT calls.
This does not replace a CubeWBA build or radio test.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/ST/STM32WBAxx/src/bt_gatt_stm32wba.cpp').read_text()
registration = source[source.index('static uint8_t WbaSecPerm'):source.index('bool BtGattCharSetValue')]
code = r'''
#include <cstdlib>
#define CHECK(x) do { if (!(x)) std::abort(); } while (0)
#include <cstring>
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/blueio_blesrvc.h"
#define WBA_CHAR_VALUE_OFFSET 1
#define WBA_CHAR_CCCD_OFFSET 2
#define ATTR_PERMISSION_AUTHEN_READ 1
#define ATTR_PERMISSION_AUTHEN_WRITE 2
#define ATTR_PERMISSION_ENCRY_READ 4
#define ATTR_PERMISSION_ENCRY_WRITE 8
#define ATTR_PERMISSION_NONE 0
#define UUID_TYPE_128 2
#define UUID_TYPE_16 1
#define PRIMARY_SERVICE 1
#define GATT_NOTIFY_ATTRIBUTE_WRITE 1
#define GATT_NOTIFY_NOTIFICATION_COMPLETION 2
#define MIN_ENCRY_KEY_SIZE 7
#define CHAR_VALUE_LEN_VARIABLE 1
#define BLE_STATUS_SUCCESS 0
using tBleStatus = unsigned char;
union Char_UUID_t { uint8_t Char_UUID_128[16]; uint16_t Char_UUID_16; };
union Service_UUID_t { uint8_t Service_UUID_128[16]; uint16_t Service_UUID_16; };
BTGAP_SECTYPE BtAppSecTypeGet() { return BTGAP_SECTYPE_NONE; }
static tBleStatus aci_gatt_add_service(uint8_t, Service_UUID_t *, uint8_t, uint8_t, uint16_t *h)
{
 *h = 10;
 return 0;
}
static tBleStatus aci_gatt_add_char(uint16_t, uint8_t, Char_UUID_t *, uint16_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint16_t *h)
{
 *h = 11;
 return 0;
}
void BtGattInsertSrvcList(BtGattSrvc_t *) {}
'''
code += registration
code += r'''
int main()
{
 BtGattChar_t chars[] = {BT_CHAR(BLUEIO_UUID_UART_RX_CHAR, 20, BT_GATT_CHAR_PROP_READ, "RX")};
 BtGattSrvc_t service = BT_SRVC_CUSTOM(BLUEIO_UUID_BASE, BLUEIO_UUID_UART_SERVICE, chars);
 CHECK(BtGattSrvcAdd(&service));
 uint8_t base[16];
 CHECK(BtUuidGetBase(1, base));
 CHECK(std::memcmp(base, service.UuidBase, 16) == 0);
 CHECK(service.Uuid.BaseIdx == 1);
 CHECK(chars[0].BaseUuidIdx == 1);
 BtUuidArr_t uuids = {.BaseIdx = 1, .Type = BT_UUID_TYPE_16, .Count = 1, .Uuid16 = {BLUEIO_UUID_UART_SERVICE}};
 uint8_t manufacturer[8] = {};
 BtAppCfg_t cfg = {};
 cfg.Role = BTAPP_ROLE_PERIPHERAL;
 cfg.pDevName = "BLESec";
 cfg.pAdvManData = manufacturer;
 cfg.AdvManDataLen = sizeof(manufacturer);
 cfg.pAdvUuid = &uuids;
 uint8_t adv[251], scan[251];
 BtAdvPacket_t a = {251, 0, adv}, b = {251, 0, scan};
 bool extended = true, scannable = false;
 CHECK(BtAdvEncode(&cfg, &a, &b, &extended, &scannable));
 CHECK(!extended && scannable);
 CHECK(a.Len == 23 && b.Len == 18);
 CHECK(scan[1] == BT_GAP_DATA_TYPE_INCOMPLETE_SRVC_UUID128);
}
'''
with tempfile.TemporaryDirectory(prefix='wba-gatt-uuid-') as temporary:
    path = Path(temporary)
    (path / 'test.cpp').write_text(code)
    for mode, flags in [('Debug', ['-O0', '-DDEBUG']), ('Release', ['-Os', '-DNDEBUG'])]:
        subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++2b',
            '-ffunction-sections', '-fdata-sections', *flags,
            '-I' + str(ROOT / 'include'), str(path / 'test.cpp'),
            str(ROOT / 'src/bluetooth/bt_adv.cpp'),
            str(ROOT / 'src/bluetooth/bt_uuid.cpp'),
            '-Wl,--gc-sections', '-o', str(path / 'test')], check=True)
        subprocess.run([str(path / 'test')], check=True)
        print(mode + ': WBA UUID registration and BLESec legacy encoding PASS')
