import base64
import zlib
from pathlib import Path

root = Path(__file__).resolve().parent
payload = "".join(
    (root / f"bt_fix_part_{index:02}.txt").read_text().strip()
    for index in range(13)
)
source = zlib.decompress(base64.b64decode(payload)).decode()
old = 'replace_once(\n    ".github/workflows/bluetooth-host-tests.yml",\n    \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\',\n    \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\',\n)\nreplace_once(\n    ".github/workflows/bluetooth-host-tests.yml",\n    \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\',\n    \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\',\n)'
new = 'path = ".github/workflows/bluetooth-host-tests.yml"\nold = \'\'\'      - "ARM/Nordic/nRF52/src/bt_gatt_nrf52.cpp"\n      - "ARM/Nordic/nRF54/src/bt_gatt_bm.cpp"\'\'\'\nnew = \'\'\'      - "ARM/**/src/bt_*.cpp"\'\'\'\ntext = read(path)\nif text.count(old) != 2:\n    raise RuntimeError(f"{path}: expected two target path blocks, found {text.count(old)}")\nwrite(path, text.replace(old, new))'
if source.count(old) != 1:
    raise RuntimeError(f"patch loader: expected workflow replacement block once, found {source.count(old)}")
source = source.replace(old, new)
exec(compile(source, "<bt-fixes>", "exec"))
