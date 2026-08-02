import base64
import zlib
from pathlib import Path

root = Path(__file__).resolve().parent
payload = "".join(
    (root / f"bt_fix_part_{index:02}.txt").read_text().strip()
    for index in range(13)
)
source = zlib.decompress(base64.b64decode(payload)).decode()
workflow_old = 'replace_once(\n    ".github/workflows/bluetooth-host-tests.yml",\n    \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\',\n    \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\',\n)\nreplace_once(\n    ".github/workflows/bluetooth-host-tests.yml",\n    \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\',\n    \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\',\n)'
workflow_new = 'path = ".github/workflows/bluetooth-host-tests.yml"\nold = \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\'\nnew = \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\'\ntext = read(path)\nif text.count(old) != 2:\n    raise RuntimeError(f"{path}: expected two target path blocks, found {text.count(old)}")\nwrite(path, text.replace(old, new))'
if source.count(workflow_old) != 1:
    raise RuntimeError(f"patch loader: expected workflow replacement block once, found {source.count(workflow_old)}")
source = source.replace(workflow_old, workflow_new)
marker = 'print("Bluetooth failure-path fixes applied")'
replacement = '\nreplace_once(\n    "src/bluetooth/bt_att.cpp",\n    "__attribute__((weak)) bool BtGattCccdCanStore",\n    """__attribute__((weak)) void BtGattCccdChanged(uint16_t ConnHdl,\n                                             uint16_t CccdHdl,\n                                             uint16_t OldValue)\n{\n    (void)ConnHdl;\n    (void)CccdHdl;\n    (void)OldValue;\n}\n\n__attribute__((weak)) bool BtGattCccdCanStore""",\n)\n\nreplace_once(\n    "src/bluetooth/bt_app.cpp",\n    "\\n\\t\\t.TxUntracked = 0,",\n    "",\n)\n\nreplace_once(\n    "tests/bluetooth/host/bt_att_req_test.cpp",\n    """bool BtGattCccdSet(uint16_t, uint16_t, uint16_t Value)\n{\n\\t++g_CccdSetCount;\n\\tg_CccdSetVal = Value;\n\\treturn true;\n}\nvoid BtGattClientNotified""",\n    """bool BtGattCccdSet(uint16_t, uint16_t, uint16_t Value)\n{\n\\t++g_CccdSetCount;\n\\tg_CccdSetVal = Value;\n\\treturn true;\n}\nvoid BtGattCccdChanged(uint16_t, uint16_t CccdHdl, uint16_t)\n{\n\\t++g_CccdSetCount;\n\\tfor (uint8_t i = 0; i < s_StubPeer.Conn.NbCccd; i++)\n\\t{\n\\t\\tif (s_StubPeer.Conn.Cccd[i].Hdl == CccdHdl)\n\\t\\t{\n\\t\\t\\tg_CccdSetVal = s_StubPeer.Conn.Cccd[i].Value;\n\\t\\t\\tbreak;\n\\t\\t}\n\\t}\n}\nvoid BtGattClientNotified""",\n)\n\nprint("Bluetooth failure-path fixes applied")'
if source.count(marker) != 1:
    raise RuntimeError(f"patch loader: expected final marker once, found {source.count(marker)}")
source = source.replace(marker, replacement)
exec(compile(source, "<bt-fixes>", "exec"))
